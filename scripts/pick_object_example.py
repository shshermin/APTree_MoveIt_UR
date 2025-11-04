#!/usr/bin/env python3
"""
Example demonstrating a complete pick task using MoveIt 2.
This script shows how to:
1. Set up the planning scene with objects
2. Plan a motion to pre-grasp pose
3. Move to grasp pose
4. Close gripper
5. Attach object to gripper
6. Move to retreat pose
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, 
    Constraints, 
    JointConstraint,
    PositionIKRequest,
    RobotState,
    CollisionObject,
    AttachedCollisionObject
)
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time


class PickTaskExample(Node):
    """Complete pick task example using MoveIt 2."""
    
    def __init__(self):
        super().__init__('pick_task_example')
        
        # Action client for MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Publishers
        self.collision_pub = self.create_publisher(
            CollisionObject, '/collision_object', 10
        )
        self.attached_pub = self.create_publisher(
            AttachedCollisionObject, '/attached_collision_object', 10
        )
        
        # Wait for action server
        self.get_logger().info('Waiting for move_group action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('Connected to move_group')
        
        # Configuration
        self.planning_group = 'ur_manipulator'
        self.end_effector_link = 'tool0'
        
        time.sleep(1.0)
    
    def add_box_object(self, object_id, pos, size=(0.05, 0.05, 0.05)):
        """Add a box collision object to the scene."""
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = 'base_link'
        collision_obj.header.stamp = self.get_clock().now().to_msg()
        collision_obj.id = object_id
        collision_obj.operation = CollisionObject.ADD
        
        # Create box primitive
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size)  # [x, y, z] dimensions
        
        # Set pose
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.w = 1.0
        
        collision_obj.primitives.append(box)
        collision_obj.primitive_poses.append(pose)
        
        # Publish
        for _ in range(3):  # Publish multiple times to ensure receipt
            self.collision_pub.publish(collision_obj)
            time.sleep(0.1)
        
        self.get_logger().info(f'Added box object: {object_id}')
        time.sleep(0.5)
    
    def remove_object(self, object_id):
        """Remove object from scene."""
        collision_obj = CollisionObject()
        collision_obj.id = object_id
        collision_obj.operation = CollisionObject.REMOVE
        self.collision_pub.publish(collision_obj)
        time.sleep(0.3)
    
    def attach_object(self, object_id, link_name='tool0'):
        """Attach object to the robot (simulates grasping)."""
        attached_obj = AttachedCollisionObject()
        attached_obj.object.id = object_id
        attached_obj.link_name = link_name
        attached_obj.object.operation = CollisionObject.ADD
        
        # Touch links - links allowed to touch the object
        attached_obj.touch_links = [link_name, 'wrist_3_link']
        
        self.attached_pub.publish(attached_obj)
        self.get_logger().info(f'Attached object {object_id} to {link_name}')
        time.sleep(0.5)
    
    def detach_object(self, object_id):
        """Detach object from robot."""
        attached_obj = AttachedCollisionObject()
        attached_obj.object.id = object_id
        attached_obj.object.operation = CollisionObject.REMOVE
        
        self.attached_pub.publish(attached_obj)
        self.get_logger().info(f'Detached object {object_id}')
        time.sleep(0.5)
    
    def plan_to_pose(self, target_pose):
        """
        Plan and execute motion to target pose.
        
        Args:
            target_pose: geometry_msgs/Pose
        
        Returns:
            bool: True if successful
        """
        # Create goal message
        goal = MoveGroup.Goal()
        
        # Set planning group
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        
        # Set target pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'base_link'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = target_pose
        
        goal.request.goal_constraints.append(Constraints())
        goal.request.goal_constraints[0].name = 'target_pose'
        # Note: Full pose constraint setup requires position and orientation constraints
        # For simplicity, this example shows the structure
        
        # Alternative: Set joint space goal (more reliable for testing)
        # We'll use a simplified approach here
        
        goal.planning_options.plan_only = False  # Plan and execute
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        
        self.get_logger().info('Sending goal to move_group...')
        
        # Send goal
        send_goal_future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False
        
        self.get_logger().info('Goal accepted, waiting for result...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('Motion executed successfully')
            return True
        else:
            self.get_logger().error(f'Motion failed with error code: {result.error_code.val}')
            return False
    
    def execute_pick_sequence(self):
        """Execute complete pick sequence."""
        
        # 1. Add table to scene
        self.get_logger().info('Step 1: Adding table to scene')
        self.add_box_object('table', pos=(0.5, 0.0, -0.05), size=(0.6, 0.6, 0.1))
        
        # 2. Add object to pick
        self.get_logger().info('Step 2: Adding object to scene')
        object_pos = (0.5, 0.0, 0.05)  # On top of table
        self.add_box_object('target_object', pos=object_pos, size=(0.05, 0.05, 0.05))
        
        # 3. Define pick poses
        # Pre-grasp: above object
        pre_grasp_pose = Pose()
        pre_grasp_pose.position.x = object_pos[0]
        pre_grasp_pose.position.y = object_pos[1]
        pre_grasp_pose.position.z = object_pos[2] + 0.15  # 15cm above
        pre_grasp_pose.orientation.x = -0.707  # Point down
        pre_grasp_pose.orientation.y = 0.0
        pre_grasp_pose.orientation.z = 0.0
        pre_grasp_pose.orientation.w = 0.707
        
        # Grasp: at object
        grasp_pose = Pose()
        grasp_pose.position.x = object_pos[0]
        grasp_pose.position.y = object_pos[1]
        grasp_pose.position.z = object_pos[2] + 0.02  # Just above object
        grasp_pose.orientation = pre_grasp_pose.orientation
        
        # Retreat: lift object
        retreat_pose = Pose()
        retreat_pose.position.x = object_pos[0]
        retreat_pose.position.y = object_pos[1]
        retreat_pose.position.z = object_pos[2] + 0.20
        retreat_pose.orientation = pre_grasp_pose.orientation
        
        # 4. Move to pre-grasp
        self.get_logger().info('Step 3: Moving to pre-grasp pose')
        if not self.plan_to_pose(pre_grasp_pose):
            self.get_logger().error('Failed to reach pre-grasp pose')
            return False
        
        # 5. Move to grasp
        self.get_logger().info('Step 4: Moving to grasp pose')
        if not self.plan_to_pose(grasp_pose):
            self.get_logger().error('Failed to reach grasp pose')
            return False
        
        # 6. Close gripper (simulated - you'd send command to real gripper here)
        self.get_logger().info('Step 5: Closing gripper')
        time.sleep(1.0)  # Simulate gripper closing
        
        # 7. Attach object to gripper
        self.get_logger().info('Step 6: Attaching object to gripper')
        self.remove_object('target_object')  # Remove from scene
        self.attach_object('target_object', link_name=self.end_effector_link)
        
        # 8. Retreat
        self.get_logger().info('Step 7: Retreating with object')
        if not self.plan_to_pose(retreat_pose):
            self.get_logger().error('Failed to retreat')
            return False
        
        self.get_logger().info('Pick sequence completed successfully!')
        return True


def main():
    """Main function demonstrating pick task."""
    rclpy.init()
    
    node = PickTaskExample()
    
    try:
        # Execute pick sequence
        node.get_logger().info('Starting pick task demonstration...')
        success = node.execute_pick_sequence()
        
        if success:
            node.get_logger().info('Pick task completed! Press Ctrl+C to exit.')
            # Keep node alive to visualize result
            rclpy.spin(node)
        else:
            node.get_logger().error('Pick task failed.')
    
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
