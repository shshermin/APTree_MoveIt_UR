#!/usr/bin/env python3
"""
Simple moveTo task: Move robot end-effector to a specified Cartesian pose.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    PlanningScene,
    AllowedCollisionMatrix,
    AllowedCollisionEntry
)
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import GetPlanningScene
import time
import argparse
import math


# Map end-effector types to their tip link names
EE_LINK_MAP = {
    'none': 'tool0',
    'gripper': 'gripper_tip',
    'nailgun': 'nailgun_tip',
}

# Links belonging to each end-effector
EE_LINKS = {
    'gripper': ['gripper_base', 'gripper_left_finger', 'gripper_right_finger', 'gripper_tip'],
    'nailgun': ['nailgun_base', 'nailgun_tip'],
}


class MoveToTask(Node):
    """Simple task to move robot to a Cartesian pose."""
    
    def __init__(self, end_effector_type='none'):
        super().__init__('move_to_task')
        
        # Action client for MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Configuration
        self.planning_group = 'ur_manipulator'
        self.end_effector_link = EE_LINK_MAP.get(end_effector_type, 'tool0')
        self.reference_frame = 'base_link'
        self.get_logger().info(f'End-effector link: {self.end_effector_link}')
        
        # Wait for action server
        self.get_logger().info('Waiting for move_group action server...')
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action server not available!')
            raise RuntimeError('MoveGroup action server timeout')
        
        self.get_logger().info('Connected to move_group')
    
    def disable_inactive_ee(self, active_ee):
        """
        When launched with 'both' end-effectors, disable ALL collisions for the
        inactive one by publishing an ACM update via /planning_scene.
        This includes collisions with robot links AND scene objects (e.g. table).
        """
        inactive = 'nailgun' if active_ee == 'gripper' else 'gripper'
        inactive_links = EE_LINKS.get(inactive, [])
        if not inactive_links:
            return
        
        # Get current planning scene (ACM + world objects)
        client = self.create_client(GetPlanningScene, '/get_planning_scene')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('GetPlanningScene service not available, skipping ACM update')
            return
        
        req = GetPlanningScene.Request()
        req.components.components = (
            req.components.ALLOWED_COLLISION_MATRIX | req.components.WORLD_OBJECT_NAMES
        )
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result().scene
        current_acm = result.allowed_collision_matrix
        
        # Gather all scene object names (table, etc.)
        scene_object_names = [obj.id for obj in result.world.collision_objects]
        
        # Build updated ACM: allow collisions between inactive links and everything
        entry_names = list(current_acm.entry_names)
        entry_values = [list(e.enabled) for e in current_acm.entry_values]
        
        # Also ensure scene objects are in the ACM
        for obj_name in scene_object_names:
            if obj_name not in entry_names:
                idx = len(entry_names)
                entry_names.append(obj_name)
                for row in entry_values:
                    row.append(False)
                entry_values.append([False] * len(entry_names))
        
        # Now set all entries for inactive links to True (allow all collisions)
        for link in inactive_links:
            if link not in entry_names:
                idx = len(entry_names)
                entry_names.append(link)
                for row in entry_values:
                    row.append(True)
                entry_values.append([True] * len(entry_names))
            else:
                idx = entry_names.index(link)
                for i in range(len(entry_names)):
                    entry_values[idx][i] = True
                    entry_values[i][idx] = True
        
        # Publish updated ACM
        new_acm = AllowedCollisionMatrix()
        new_acm.entry_names = entry_names
        for row in entry_values:
            entry = AllowedCollisionEntry()
            entry.enabled = row
            new_acm.entry_values.append(entry)
        
        scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.allowed_collision_matrix = new_acm
        
        # Publish a few times to make sure it's received
        for _ in range(5):
            scene_pub.publish(scene_msg)
            time.sleep(0.1)
        
        self.get_logger().info(f'Disabled collisions for inactive end-effector: {inactive}')
    
    def move_to(self, target_pose, velocity_scaling=0.1, acceleration_scaling=0.1, 
                planning_time=5.0, tolerance_position=0.001, tolerance_orientation=0.01,
                path_constraints=None):
        """
        Move end-effector to target Cartesian pose.
        
        Args:
            target_pose: geometry_msgs/Pose - target position and orientation
            velocity_scaling: Max velocity as fraction of maximum (0.0-1.0)
            acceleration_scaling: Max acceleration as fraction of maximum (0.0-1.0)
            planning_time: Maximum time for planning in seconds
            tolerance_position: Position tolerance in meters
            tolerance_orientation: Orientation tolerance in radians
            
        Returns:
            bool: True if motion succeeded, False otherwise
        """
        self.get_logger().info(f'Planning motion to pose: '
                              f'pos=({target_pose.position.x:.3f}, '
                              f'{target_pose.position.y:.3f}, '
                              f'{target_pose.position.z:.3f})')
        
        # Create goal
        goal = MoveGroup.Goal()
        
        # Set planning parameters
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = planning_time
        goal.request.max_velocity_scaling_factor = velocity_scaling
        goal.request.max_acceleration_scaling_factor = acceleration_scaling
        goal.request.workspace_parameters.header.frame_id = self.reference_frame
        goal.request.pipeline_id = 'ompl'
        goal.request.planner_id = 'RRTstar'
        
        # Set target pose as goal constraint
        constraints = Constraints()
        constraints.name = 'target_pose'
        
        # Position constraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.reference_frame
        position_constraint.link_name = self.end_effector_link
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        
        # Define constraint region as small box around target
        constraint_region = BoundingVolume()
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [tolerance_position * 2, tolerance_position * 2, tolerance_position * 2]
        constraint_region.primitives.append(box)
        
        # Box pose at target position
        box_pose = Pose()
        box_pose.position = target_pose.position
        box_pose.orientation.w = 1.0
        constraint_region.primitive_poses.append(box_pose)
        
        position_constraint.constraint_region = constraint_region
        position_constraint.weight = 1.0
        
        # Orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.reference_frame
        orientation_constraint.link_name = self.end_effector_link
        orientation_constraint.orientation = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = tolerance_orientation
        orientation_constraint.absolute_y_axis_tolerance = tolerance_orientation
        orientation_constraint.absolute_z_axis_tolerance = tolerance_orientation
        orientation_constraint.weight = 1.0
        
        # Add constraints to goal
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        goal.request.goal_constraints.append(constraints)
        
        # Path constraints (applied throughout the entire trajectory)
        if path_constraints is not None:
            goal.request.path_constraints = path_constraints
            self.get_logger().info('Path constraints applied to trajectory')
        
        # Planning options
        goal.planning_options.plan_only = False  # Plan AND execute on real robot
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5
        goal.planning_options.replan_delay = 0.5
        
        # Send goal
        self.get_logger().info('Sending goal to move_group...')
        send_goal_future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by move_group')
            return False
        
        self.get_logger().info('Goal accepted, executing motion...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        
        # Check result
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('Motion completed successfully!')
            return True
        else:
            error_codes = {
                -1: 'FAILURE',
                -2: 'PLANNING_FAILED',
                -3: 'INVALID_MOTION_PLAN',
                -4: 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
                -5: 'CONTROL_FAILED',
                -6: 'UNABLE_TO_AQUIRE_SENSOR_DATA',
                -7: 'TIMED_OUT',
                -10: 'PREEMPTED',
                -11: 'START_STATE_IN_COLLISION',
                -12: 'START_STATE_VIOLATES_PATH_CONSTRAINTS',
                -13: 'GOAL_IN_COLLISION',
                -14: 'GOAL_VIOLATES_PATH_CONSTRAINTS',
                -15: 'GOAL_CONSTRAINTS_VIOLATED',
                -21: 'INVALID_GROUP_NAME',
                -23: 'INVALID_GOAL_CONSTRAINTS',
                -31: 'NO_IK_SOLUTION',
            }
            error_name = error_codes.get(result.error_code.val, 'UNKNOWN_ERROR')
            self.get_logger().error(f'Motion failed: {error_name} (code: {result.error_code.val})')
            return False


def main():
    """Example: Add object to scene and move to grasp position above it."""
    parser = argparse.ArgumentParser(description='Move robot to a target pose')
    parser.add_argument('--x', type=float, default=-0.5, help='Target X coordinate (default: -0.5)')
    parser.add_argument('--y', type=float, default=-0.5, help='Target Y coordinate (default: -0.5)')
    parser.add_argument('--z', type=float, default=0.001, help='Target Z coordinate (default: 0.001)')
    parser.add_argument('--yaw', type=float, default=180.0, help='Tool0 yaw in degrees (rotation around Z while facing down, default: 180)')
    parser.add_argument('--end_effector_type', type=str, default='none',
                        choices=['none', 'gripper', 'nailgun'],
                        help='End-effector type to plan with (default: none = flange)')
    parser.add_argument('--no_object', action='store_true',
                        help='Skip attaching object to gripper')
    parser.add_argument('--both_loaded', action='store_true',
                        help='Set when launched with end_effector_type:=both to disable inactive EE collisions')
    args = parser.parse_args()

    # Nailgun always uses fixed orientation (facing down, yaw=90 to align with X)
    if args.end_effector_type == 'nailgun':
        args.yaw = 90.0
        args.no_object = True

    rclpy.init()
    
    # Import scene manager
    from dynamic_scene_example import DynamicSceneManager
    
    # Create scene manager and motion planner
    scene = DynamicSceneManager()
    move_to_task = MoveToTask(end_effector_type=args.end_effector_type)
    
    # When both EEs are loaded, disable collisions for the inactive one
    if args.both_loaded and args.end_effector_type in ('gripper', 'nailgun'):
        move_to_task.disable_inactive_ee(args.end_effector_type)
    
    if not args.no_object:
        # Step 1: Attach object to gripper so it moves with the robot
        # Object mesh: X(-0.01 to 0.01), Y(-0.1874 to 0.1876), Z(-0.02 to 0.0)
        # gripper_base axes now match tool0: Z+ outward, X+ left, Y+ up
        # gripper_tip is at (0.01, 0.0, 0.148) in gripper_base frame
        # Place object at fingertips along Z (outward), centered on X/Y
        move_to_task.get_logger().info('Step 1: Attaching object to gripper')

        touch_links = ['gripper_base', 'gripper_left_finger', 'gripper_right_finger', 'tool0']
        if args.both_loaded:
            touch_links.extend(['nailgun_base', 'nailgun_tip'])

        scene.attach_mesh_object(
            object_id='target_object',
            mesh_path='object',
            link_name='gripper_base',
            pos=(0.0, 0.0, 0.1225),
            scale=(1.0, 1.0, 1.0),
            touch_links=touch_links
        )
        
        # Wait for scene to fully update before planning
        time.sleep(5.0)
    
    # Step 2: Define target pose
    move_to_task.get_logger().info('Step 2: Moving to target position')
    
    target_pose = Pose()
    
    # Position: 15cm above the object
    target_pose.position.x = args.x
    target_pose.position.y = args.y
    target_pose.position.z = args.z 

    # Orientation: tool0 facing straight down + yaw around Z
    # RPY = (pi, 0, yaw) → quaternion = (cos(yaw/2), sin(yaw/2), 0, 0)
    yaw_rad = math.radians(args.yaw)
    cy = math.cos(yaw_rad / 2)
    sy = math.sin(yaw_rad / 2)
    target_pose.orientation.x = cy
    target_pose.orientation.y = sy
    target_pose.orientation.z = 0.0
    target_pose.orientation.w = 0.0
    move_to_task.get_logger().info(f'Orientation: facing down, tool0 yaw={args.yaw} deg')
    
    # Build path constraints for nailgun: keep facing down throughout trajectory
    path_constraints = None
    if args.end_effector_type == 'nailgun':
        path_constraints = Constraints()
        path_constraints.name = 'nailgun_facing_down'
        
        path_oc = OrientationConstraint()
        path_oc.header.frame_id = 'base_link'
        path_oc.link_name = 'tool0'  # must be in ur_manipulator group
        # Same orientation as goal: facing down with given yaw
        path_oc.orientation = target_pose.orientation
        # Tight on roll/pitch (stay facing down), loose on yaw (allow some rotation)
        path_oc.absolute_x_axis_tolerance = 0.1   # ~6 deg tolerance on roll
        path_oc.absolute_y_axis_tolerance = 0.1   # ~6 deg tolerance on pitch
        path_oc.absolute_z_axis_tolerance = 3.14   # allow yaw freedom
        path_oc.weight = 1.0
        path_constraints.orientation_constraints.append(path_oc)
        move_to_task.get_logger().info('Nailgun mode: path constrained to always face down')
    
    # Execute motion
    success = move_to_task.move_to(
        target_pose,
        velocity_scaling=0.05,  # Very slow - 5% of max speed
        acceleration_scaling=0.05,  # Very slow - 5% of max acceleration
        planning_time=10.0,
        path_constraints=path_constraints
    )
    
    if success:
        move_to_task.get_logger().info('Successfully reached grasp position!')
        # Detach object from gripper, leaving it in the scene at its current pose
        if not args.no_object:
            scene.detach_object('target_object', link_name='gripper_base')
            move_to_task.get_logger().info('Object detached and left in scene')
    else:
        move_to_task.get_logger().error('Failed to reach grasp position')
    
    # Cleanup
    scene.destroy_node()
    move_to_task.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
