#!/usr/bin/env python3
"""
Simple arm movement test with predefined joint positions (no IK).

Builds on test_simple_arm.py - sends a sequence of joint commands to move the arm.

Usage:
    ros2 run tidybot_bringup test_motion_planner.py
"""

import rclpy
from rclpy.node import Node
from tidybot_msgs.msg import ArmCommand
import time


class ArmSequenceTest(Node):
    """Test arm with a sequence of predefined joint positions."""

    def __init__(self):
        super().__init__('arm_sequence_test')

        # Publisher for right arm commands
        self.pub = self.create_publisher(ArmCommand, '/right_arm/cmd', 10)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Arm Sequence Test - Direct Joint Commands')
        self.get_logger().info('No IK, just predefined joint positions')
        self.get_logger().info('=' * 60)

        # Wait for publisher to connect
        time.sleep(0.5)

        # Run the sequence
        self.run_sequence()

    def send_joint_command(self, name: str, joint_positions: list, duration: float = 3.0):
        """Send a joint command to the arm."""
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'Movement: {name}')
        self.get_logger().info(f'Joint positions: {joint_positions}')
        self.get_logger().info(f'Duration: {duration}s')

        cmd = ArmCommand()
        cmd.joint_positions = joint_positions
        cmd.duration = duration

        self.pub.publish(cmd)
        self.get_logger().info('✅ Command sent! Arm is moving...')

        # Wait for motion to complete
        time.sleep(duration + 0.5)

    def run_sequence(self):
        """Run a sequence of arm movements."""
        # Joint order: [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]

        self.get_logger().info('')
        self.get_logger().info('Starting movement sequence...')
        self.get_logger().info('')

        # Move 1: Return to home position
        self.send_joint_command(
            'Move 1: Home position',
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            duration=2.0
        )

        # Move 2: Reach forward (shoulder + elbow)
        self.send_joint_command(
            'Move 2: Reach forward',
            [0.0, 0.5, 0.5, 0.0, 0.5, 0.0],
            duration=3.0
        )

        # Move 3: Reach to the side (waist rotation)
        self.send_joint_command(
            'Move 3: Reach to right side',
            [0.5, 0.5, 0.5, 0.0, 0.5, 0.0],
            duration=3.0
        )

        # Move 4: Reach to left side
        self.send_joint_command(
            'Move 4: Reach to left side',
            [-0.5, 0.5, 0.5, 0.0, 0.5, 0.0],
            duration=3.0
        )

        # Move 5: Higher reach (more shoulder movement)
        self.send_joint_command(
            'Move 5: Reach higher',
            [0.0, 0.8, 0.3, 0.0, 0.8, 0.0],
            duration=3.0
        )

        # Move 6: Lower reach (more elbow bend)
        self.send_joint_command(
            'Move 6: Reach lower',
            [0.0, 0.3, 1.0, 0.0, 0.5, 0.0],
            duration=3.0
        )

        # Move 7: Return home
        self.send_joint_command(
            'Move 7: Return home',
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            duration=3.0
        )

        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Sequence complete!')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = ArmSequenceTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
