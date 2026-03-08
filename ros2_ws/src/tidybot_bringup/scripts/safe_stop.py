#!/usr/bin/env python3
"""
Safe Stop — Emergency stop and return-to-home for TidyBot2.

Run from a separate terminal to immediately:
  1. Stop the mobile base
  2. Return the arm to home position
  3. Open the gripper
  4. Reset the camera to neutral tilt
  5. Signal the explore_and_find_object node to shut down

Usage:
    ros2 run tidybot_bringup safe_stop.py
"""

import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String
from tidybot_msgs.msg import ArmCommand


class SafeStop(Node):

    def __init__(self):
        super().__init__('safe_stop')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(ArmCommand, '/right_arm/cmd', 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/right_gripper/cmd', 10)
        self.pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10)
        self.safe_stop_pub = self.create_publisher(String, '/safe_stop', 10)

    def execute(self):
        # Brief pause so publishers are discovered by subscribers
        time.sleep(0.5)

        self.get_logger().info('=== SAFE STOP ===')

        # 1. Stop the base
        self.get_logger().info('Stopping base...')
        for _ in range(10):
            self.cmd_vel_pub.publish(Twist())
            time.sleep(0.05)

        # 2. Signal the explore node to shut down
        self.get_logger().info('Sending safe_stop signal...')
        msg = String()
        msg.data = 'stop'
        for _ in range(10):
            self.safe_stop_pub.publish(msg)
            time.sleep(0.05)

        # 3. Open gripper
        self.get_logger().info('Opening gripper...')
        grip_msg = Float64MultiArray()
        grip_msg.data = [0.0]
        for _ in range(10):
            self.gripper_pub.publish(grip_msg)
            time.sleep(0.05)

        # 4. Return arm to home
        self.get_logger().info('Returning arm to home position...')
        arm_msg = ArmCommand()
        arm_msg.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        arm_msg.duration = 3.0
        self.arm_pub.publish(arm_msg)

        # 5. Reset camera tilt to neutral
        self.get_logger().info('Resetting camera tilt...')
        pt_msg = Float64MultiArray()
        pt_msg.data = [0.0, 0.3]
        self.pan_tilt_pub.publish(pt_msg)

        # Wait for arm to finish moving
        self.get_logger().info('Waiting for arm to reach home (3s)...')
        time.sleep(3.5)

        self.get_logger().info('=== SAFE STOP COMPLETE ===')


def main(args=None):
    rclpy.init(args=args)
    node = SafeStop()
    try:
        node.execute()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
