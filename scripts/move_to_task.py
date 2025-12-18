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
    BoundingVolume
)
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
import time


class MoveToTask(Node):
    """Simple task to move robot to a Cartesian pose."""
    
    def __init__(self):
        super().__init__('move_to_task')
        
        # Action client for MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Configuration
        self.planning_group = 'ur_manipulator'
        self.end_effector_link = 'tool0'
        self.reference_frame = 'base_link'
        
        # Wait for action server
        self.get_logger().info('Waiting for move_group action server...')
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action server not available!')
            raise RuntimeError('MoveGroup action server timeout')
        
        self.get_logger().info('Connected to move_group')
    
    def move_to(self, target_pose, velocity_scaling=0.1, acceleration_scaling=0.1, 
                planning_time=5.0, tolerance_position=0.001, tolerance_orientation=0.01):
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
        
        # Planning options
        goal.planning_options.plan_only = True  # Plan AND execute on real robot
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        
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
    rclpy.init()
    
    # Import scene manager
    from dynamic_scene_example import DynamicSceneManager
    
    # Create scene manager and motion planner
    scene = DynamicSceneManager()
    move_to_task = MoveToTask()
    
    # Step 1: Add object to scene
    move_to_task.get_logger().info('Step 1: Adding object to scene')

    object_position = (-0.6, 0.199, 0.1)  # Position on table (5cm above base)

    scene.add_mesh_object(
        object_id='target_object',
        mesh_path='/home/shermin/ws_moveit/src/hello_moveit/meshes/collision/object.stl',
        pos=object_position,  # Use same position
        scale=(1.0, 1.0, 1.0)
    )
    
    # Wait for object to be added
    time.sleep(1.0)
    
    # Step 2: Define grasp pose (above the object)
    move_to_task.get_logger().info('Step 2: Moving to grasp position above object')
    
    target_pose = Pose()
    
    # Position: 15cm above the object
    target_pose.position.x = object_position[0]
    target_pose.position.y = object_position[1]
    target_pose.position.z = object_position[2] 

    # Orientation: tool0 Z-axis pointing down, with more rotational freedom
    # 180-degree rotation around Y-axis (allows wrist to rotate freely)
    target_pose.orientation.x = 0.0
    target_pose.orientation.y = 1.0
    target_pose.orientation.z = 0.0
    target_pose.orientation.w = 0.0
    
    # Execute motion
    success = move_to_task.move_to(
        target_pose,
        velocity_scaling=0.05,  # Very slow - 5% of max speed
        acceleration_scaling=0.05,  # Very slow - 5% of max acceleration
        planning_time=10.0
    )
    
    if success:
        move_to_task.get_logger().info('Successfully reached grasp position!')
    else:
        move_to_task.get_logger().error('Failed to reach grasp position')
    
    # Cleanup
    scene.destroy_node()
    move_to_task.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
