#!/usr/bin/env python3
"""
Simplest possible arm movement test - direct joint commands (no IK/planning).

Usage:
    ros2 run tidybot_bringup test_simple_arm.py
"""

import rclpy
from rclpy.node import Node
from tidybot_msgs.msg import ArmCommand
import time


class SimpleArmTest(Node):
    def __init__(self):
        super().__init__('simple_arm_test')

        # Publisher for right arm commands
        self.pub = self.create_publisher(ArmCommand, '/right_arm/cmd', 10)

        self.get_logger().info('Simple Arm Test - bypassing motion planner')

        # Wait a moment for publisher to connect
        time.sleep(0.5)

        # Send a simple joint command
        cmd = ArmCommand()
        # Move to a simple configuration: all joints to small angles
        cmd.joint_positions = [0.0, 0.5, 0.5, 0.0, 0.5, 0.0]  # radians
        cmd.duration = 3.0  # 3 seconds

        self.get_logger().info(f'Sending joint command: {cmd.joint_positions}')
        self.pub.publish(cmd)

        self.get_logger().info('Command sent! Arm should move over 3 seconds.')
        self.get_logger().info('Watch the sim to see if it moves.')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleArmTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
