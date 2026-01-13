#!/usr/bin/env python3
"""
TidyBot2 Base Position Control Demo

Sends position targets to the base - the robot drives there automatically.
Matches the real TidyBot2's set_target_position() interface.

Topics used:
- /base/target_pose (Pose2D) - target position [x, y, theta]
- /base/goal_reached (Bool) - notification when goal is reached

Usage:
    # Terminal 1: Start simulation
    ros2 launch tidybot_bringup sim.launch.py

    # Terminal 2: Run this demo
    ros2 run tidybot_bringup test_base.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import time


class TestBase(Node):

    def __init__(self):
        super().__init__('test_base')

        # Publisher for target pose
        self.target_pub = self.create_publisher(Pose2D, '/base/target_pose', 10)

        # Subscriber for goal reached
        self.goal_reached = False
        self.goal_sub = self.create_subscription(
            Bool, '/base/goal_reached', self.goal_callback, 10
        )

        # State machine
        self.state = 'MOVE_1'
        self.state_start_time = None

        # Control loop at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('=' * 50)
        self.get_logger().info('TidyBot2 Position Control Demo')
        self.get_logger().info('=' * 50)

    def goal_callback(self, msg: Bool):
        if msg.data:
            self.goal_reached = True

    def send_target(self, x, y, theta=0.0):
        msg = Pose2D()
        msg.x = x
        msg.y = y
        msg.theta = theta
        self.target_pub.publish(msg)

    def control_loop(self):
        now = time.time()

        if self.state_start_time is None:
            self.state_start_time = now

        elapsed = now - self.state_start_time

        # State: Move forward 0.5m
        if self.state == 'MOVE_1':
            if elapsed < 0.2:
                self.send_target(0.5, 0.0)
                self.get_logger().info('Moving to x=0.5m...')
            elif self.goal_reached:
                self.get_logger().info('Reached first target. Waiting 5 seconds...')
                self.state = 'WAIT'
                self.state_start_time = now
                self.goal_reached = False

        # State: Wait 5 seconds
        elif self.state == 'WAIT':
            if elapsed >= 5.0:
                self.get_logger().info('Moving to x=1.0m...')
                self.state = 'MOVE_2'
                self.state_start_time = now

        # State: Move another 0.5m forward (total 1.0m)
        elif self.state == 'MOVE_2':
            if elapsed < 0.2:
                self.send_target(1.0, 0.0)
            elif self.goal_reached:
                self.get_logger().info('')
                self.get_logger().info('=' * 50)
                self.get_logger().info('Done!')
                self.get_logger().info('=' * 50)
                self.state = 'DONE'

        elif self.state == 'DONE':
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TestBase()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
