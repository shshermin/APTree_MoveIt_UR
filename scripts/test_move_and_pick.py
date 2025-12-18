#!/usr/bin/env python3
"""
Test script: Move to object and pick it.
"""
import rclpy
from geometry_msgs.msg import Pose
import time

from move_to_task import MoveToTask
from pick_task import PickTask
from dynamic_scene_example import DynamicSceneManager


def main():
    rclpy.init()
    
    # Initialize
    scene = DynamicSceneManager()
    move_task = MoveToTask()
    pick_task = PickTask()
    
    object_id = 'target_object'
    object_position = (0.199, 0.597, 0.02)  # 2cm above table to avoid collision
    
    # Step 1: Add object
    move_task.get_logger().info('Step 1: Adding object to scene')
    scene.add_mesh_object(
        object_id=object_id,
        mesh_path='/home/shermin/ws_moveit/src/hello_moveit/meshes/collision/object.stl',
        pos=object_position,
        scale=(1.0, 1.0, 1.0)
    )
    time.sleep(1.0)
    
    # Step 2: Move to grasp position
    move_task.get_logger().info('Step 2: Moving to grasp position')
    
    grasp_pose = Pose()
    grasp_pose.position.x = object_position[0]
    grasp_pose.position.y = object_position[1]
    grasp_pose.position.z = object_position[2] + 0.15  # 15cm above object (adjust for gripper length)
    
    # Orientation: pointing down
    grasp_pose.orientation.x = 0.0
    grasp_pose.orientation.y = 1.0
    grasp_pose.orientation.z = 0.0
    grasp_pose.orientation.w = 0.0
    
    if not move_task.move_to(grasp_pose, velocity_scaling=0.2, acceleration_scaling=0.2):
        move_task.get_logger().error('Failed to move to grasp position!')
        return
    
    time.sleep(1.0)
    
    # Step 3: Pick object
    move_task.get_logger().info('Step 3: Picking object')
    
    if not pick_task.pick(object_id):
        move_task.get_logger().error('Failed to pick object!')
        return
    
    move_task.get_logger().info('SUCCESS! Object picked.')
    
    # Wait longer for planning scene to update with attached object
    move_task.get_logger().info('Waiting for planning scene to update...')
    time.sleep(2.0)
    
    # Step 4: Lift object upwards
    move_task.get_logger().info('Step 4: Lifting object')
    
    lift_pose = Pose()
    lift_pose.position.x = object_position[0]
    lift_pose.position.y = object_position[1]
    lift_pose.position.z = object_position[2] + 0.50  # Lift to 50cm above table (more significant motion)
    
    # Keep same orientation (pointing down)
    lift_pose.orientation.x = 0.0
    lift_pose.orientation.y = 1.0
    lift_pose.orientation.z = 0.0
    lift_pose.orientation.w = 0.0
    
    if not move_task.move_to(lift_pose, velocity_scaling=0.1, acceleration_scaling=0.1, planning_time=10.0):
        move_task.get_logger().error('Failed to lift object!')
        return
        return
    
    move_task.get_logger().info('SUCCESS! Object lifted with gripper.')
    
    # Cleanup
    scene.destroy_node()
    move_task.destroy_node()
    pick_task.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
