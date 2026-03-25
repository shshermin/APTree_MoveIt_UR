#!/usr/bin/env python3
"""Publish default joint states for gripper joints on /joint_states."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class GripperJointStatePublisher(Node):
    def __init__(self):
        super().__init__('gripper_joint_state_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.02, self.publish)  # 50 Hz
        self.get_logger().info('Publishing gripper joint states (closed position)')

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        msg.position = [0.0, 0.0]  # closed position
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = GripperJointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
