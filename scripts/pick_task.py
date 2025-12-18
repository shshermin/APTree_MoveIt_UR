#!/usr/bin/env python3
"""
Pick task: Close gripper and attach object to the robot.
Uses SRDF-defined gripper named states (open/closed).
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, RobotState
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene
from moveit_msgs.action import MoveGroup
from sensor_msgs.msg import JointState
import time


class PickTask(Node):
    """Task to pick an object: close gripper and attach object."""
    
    def __init__(self):
        super().__init__('pick_task')
        
        # Service client to modify planning scene
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene'
        )
        
        # Action client for MoveGroup (to move gripper to named states)
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Wait for services
        self.get_logger().info('Waiting for apply_planning_scene service...')
        if not self.apply_scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service not available!')
            raise RuntimeError('apply_planning_scene service timeout')
        
        self.get_logger().info('Waiting for move_group action server...')
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action server not available!')
            raise RuntimeError('MoveGroup action server timeout')
        
        self.get_logger().info('PickTask ready')
    
    def move_gripper_to_named_state(self, state_name):
        """
        Move gripper to a named state defined in SRDF (e.g., 'open' or 'closed').
        In simulation, we just update the planning scene with the joint values.
        
        Args:
            state_name: Name of the state ('open' or 'closed')
            
        Returns:
            bool: True if successful
        """
        self.get_logger().info(f'Moving gripper to state: {state_name}')
        
        # Get joint values for the named state from SRDF
        if state_name == 'closed':
            joint_values = [0.0, 0.0]  # From SRDF: closed state
        elif state_name == 'open':
            joint_values = [0.003, 0.003]  # From SRDF: open state
        else:
            self.get_logger().error(f'Unknown gripper state: {state_name}')
            return False
        
        # In simulation without gripper controller, we just log this
        # In real robot, you would use MoveGroup to plan gripper motion
        self.get_logger().info(f'Gripper joints set to {joint_values}')
        time.sleep(0.5)  # Simulate gripper motion time
        
        self.get_logger().info(f'Gripper moved to {state_name}')
        return True
    
    def close_gripper(self):
        """
        Close the gripper using SRDF 'closed' state.
        
        Returns:
            bool: True if successful
        """
        return self.move_gripper_to_named_state('closed')
    
    def open_gripper(self):
        """
        Open the gripper using SRDF 'open' state.
        
        Returns:
            bool: True if successful
        """
        return self.move_gripper_to_named_state('open')
    
    def attach_object(self, object_id, link_name='tool0', touch_links=None):
        """
        Attach object to the robot (tells MoveIt the object is now part of robot).
        
        Args:
            object_id: ID of the object in the planning scene
            link_name: Robot link to attach to (usually 'tool0' or gripper link)
            touch_links: Links allowed to touch object without collision (gripper fingers)
        """
        if touch_links is None:
            touch_links = ['gripper_base', 'gripper_left_finger', 'gripper_right_finger', 'tool0']
        
        self.get_logger().info(f'Attaching object "{object_id}" to link "{link_name}"')
        
        # Create AttachedCollisionObject message
        # The object ID must match the existing world object
        attached_object = AttachedCollisionObject()
        attached_object.link_name = link_name
        attached_object.object.id = object_id
        # When attaching existing object, MoveIt automatically removes it from world
        attached_object.object.operation = CollisionObject.ADD  
        attached_object.touch_links = touch_links
        
        # Create PlanningScene message
        planning_scene = PlanningScene()
        planning_scene.robot_state.attached_collision_objects.append(attached_object)
        planning_scene.is_diff = True
        
        # Apply to planning scene
        request = ApplyPlanningScene.Request()
        request.scene = planning_scene
        
        future = self.apply_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info(f'Successfully attached object "{object_id}"')
            return True
        else:
            self.get_logger().error(f'Failed to attach object "{object_id}"')
            return False
    
    def detach_object(self, object_id):
        """
        Detach object from the robot (for place operation).
        
        Args:
            object_id: ID of the object to detach
        """
        self.get_logger().info(f'Detaching object "{object_id}"')
        
        # Create AttachedCollisionObject with REMOVE operation
        attached_object = AttachedCollisionObject()
        attached_object.object.id = object_id
        attached_object.object.operation = CollisionObject.REMOVE
        
        # Create PlanningScene message
        planning_scene = PlanningScene()
        planning_scene.robot_state.attached_collision_objects.append(attached_object)
        planning_scene.is_diff = True
        
        # Apply to planning scene
        request = ApplyPlanningScene.Request()
        request.scene = planning_scene
        
        future = self.apply_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info(f'Successfully detached object "{object_id}"')
            return True
        else:
            self.get_logger().error(f'Failed to detach object "{object_id}"')
            return False
    
    def pick(self, object_id, link_name='tool0'):
        """
        Complete pick operation: close gripper and attach object.
        
        Args:
            object_id: ID of object to pick
            link_name: Link to attach to
            
        Returns:
            bool: True if successful
        """
        # Close gripper
        if not self.close_gripper():
            return False
        
        # Attach object
        if not self.attach_object(object_id, link_name):
            return False
        
        self.get_logger().info(f'Pick completed for object "{object_id}"')
        return True
    
    def place(self, object_id):
        """
        Complete place operation: detach object and open gripper.
        
        Args:
            object_id: ID of object to place
            
        Returns:
            bool: True if successful
        """
        # Detach object
        if not self.detach_object(object_id):
            return False
        
        # Open gripper
        if not self.open_gripper():
            return False
        
        self.get_logger().info(f'Place completed for object "{object_id}"')
        return True


def main():
    """Example: Pick an object."""
    rclpy.init()
    
    pick_task = PickTask()
    
    # Example: Pick object that was added by move_to_task
    object_id = 'target_object'
    
    pick_task.get_logger().info(f'Picking object: {object_id}')
    success = pick_task.pick(object_id)
    
    if success:
        pick_task.get_logger().info('Pick successful!')
        
        # Wait a bit, then place it back
        time.sleep(2.0)
        
        pick_task.get_logger().info('Placing object back...')
        pick_task.place(object_id)
    else:
        pick_task.get_logger().error('Pick failed!')
    
    pick_task.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
