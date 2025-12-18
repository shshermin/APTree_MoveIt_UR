#!/usr/bin/env python3
"""
Test just the lift motion (run after test_move_and_pick.py grasp step).
"""
import rclpy
from geometry_msgs.msg import Pose
import time

from move_to_task import MoveToTask


def main():
    rclpy.init()
    
    move_task = MoveToTask()
    
    object_position = (0.199, 0.597, 0.0)
    
    # Lift motion
    move_task.get_logger().info('Lifting object')
    
    lift_pose = Pose()
    lift_pose.position.x = object_position[0]
    lift_pose.position.y = object_position[1]
    lift_pose.position.z = object_position[2] + 0.50
    
    lift_pose.orientation.x = 0.0
    lift_pose.orientation.y = 1.0
    lift_pose.orientation.z = 0.0
    lift_pose.orientation.w = 0.0
    
    if move_task.move_to(lift_pose, velocity_scaling=0.1, acceleration_scaling=0.1, planning_time=10.0):
        move_task.get_logger().info('Lift planned successfully!')
    else:
        move_task.get_logger().error('Lift planning failed!')
    
    move_task.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
