#!/usr/bin/env python3
"""
Test TidyBot2 Grippers in Simulation

This script tests the grippers by publishing to /right_gripper/cmd and
/left_gripper/cmd topics in the MuJoCo simulation.

Usage:
    # First, launch the simulation:
    ros2 launch tidybot_bringup sim.launch.py

    # Then run this test:
    ros2 run tidybot_bringup test_grippers_sim.py

Topics used:
    /right_gripper/cmd (Float64MultiArray) - 0.0 = open, 1.0 = closed
    /left_gripper/cmd (Float64MultiArray) - 0.0 = open, 1.0 = closed
    /joint_states (JointState) - Monitor gripper positions
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class TestGrippersSim(Node):
    """Test grippers in simulation."""

    def __init__(self):
        super().__init__('test_grippers_sim')

        # Gripper publishers
        self.right_gripper_pub = self.create_publisher(
            Float64MultiArray, '/right_gripper/cmd', 10
        )
        self.left_gripper_pub = self.create_publisher(
            Float64MultiArray, '/left_gripper/cmd', 10
        )

        # Gripper state tracking
        self.joint_states_received = False
        self.right_gripper_pos = None
        self.left_gripper_pos = None

        # Subscriber for joint states (simulation publishes on /joint_states)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # State machine
        self.state = 'INIT'
        self.state_start_time = None
        self.logged_state = None

        self.get_logger().info('Waiting for joint states...')

    def joint_state_callback(self, msg):
        if not self.joint_states_received:
            self.joint_states_received = True
            self.get_logger().info('Connected to simulation!')

        # Extract gripper positions
        if 'right_gripper' in msg.name:
            idx = msg.name.index('right_gripper')
            if idx < len(msg.position):
                self.right_gripper_pos = msg.position[idx]

        if 'left_gripper' in msg.name:
            idx = msg.name.index('left_gripper')
            if idx < len(msg.position):
                self.left_gripper_pos = msg.position[idx]

    def send_gripper_command(self, pub, position):
        """Send gripper command with normalized position (0.0 = open, 1.0 = closed)."""
        msg = Float64MultiArray()
        msg.data = [float(position)]
        pub.publish(msg)

    def control_loop(self):
        """Main control loop - continuously sends gripper commands."""
        now = time.time()

        # Wait for connection first
        if self.state == 'INIT':
            if not self.joint_states_received:
                return  # Keep waiting

            # Connected! Print header
            self.get_logger().info('')
            self.get_logger().info('=' * 60)
            self.get_logger().info('GRIPPER TEST (Simulation)')
            self.get_logger().info('=' * 60)
            self.get_logger().info('Using /right_gripper/cmd and /left_gripper/cmd topics')
            self.get_logger().info('Command: 0.0 = open, 1.0 = closed')
            self.get_logger().info('')
            self.get_logger().info('[Step 1/3] Opening both grippers...')

            self.state = 'OPEN_GRIPPERS'
            self.state_start_time = now
            return

        # Initialize state timing
        if self.state_start_time is None:
            self.state_start_time = now

        elapsed = now - self.state_start_time

        # Log state change once
        if self.state != self.logged_state:
            if self.state == 'WAIT_FOR_INPUT_1':
                if self.right_gripper_pos is not None:
                    self.get_logger().info(f'  Right gripper: {self.right_gripper_pos:.4f} rad')
                if self.left_gripper_pos is not None:
                    self.get_logger().info(f'  Left gripper: {self.left_gripper_pos:.4f} rad')
                self.get_logger().info('')
                self.get_logger().info('>>> Grippers should be OPEN. Press Enter to CLOSE...')
                input()  # Wait for user
                self.get_logger().info('[Step 2/3] Closing both grippers...')
                self.state = 'CLOSE_GRIPPERS'
                self.state_start_time = now
            elif self.state == 'WAIT_FOR_INPUT_2':
                if self.right_gripper_pos is not None:
                    self.get_logger().info(f'  Right gripper: {self.right_gripper_pos:.4f} rad')
                if self.left_gripper_pos is not None:
                    self.get_logger().info(f'  Left gripper: {self.left_gripper_pos:.4f} rad')
                self.get_logger().info('')
                self.get_logger().info('>>> Grippers should be CLOSED. Press Enter to OPEN again...')
                input()  # Wait for user
                self.get_logger().info('[Step 3/3] Opening both grippers...')
                self.state = 'REOPEN_GRIPPERS'
                self.state_start_time = now
            elif self.state == 'DONE':
                if self.right_gripper_pos is not None:
                    self.get_logger().info(f'  Right gripper: {self.right_gripper_pos:.4f} rad')
                if self.left_gripper_pos is not None:
                    self.get_logger().info(f'  Left gripper: {self.left_gripper_pos:.4f} rad')
                self.get_logger().info('')
                self.get_logger().info('=' * 60)
                self.get_logger().info('Gripper test complete!')
                self.get_logger().info('Grippers should now be OPEN.')
                self.get_logger().info('=' * 60)
            self.logged_state = self.state

        # State: Open grippers (initially)
        if self.state == 'OPEN_GRIPPERS':
            self.send_gripper_command(self.right_gripper_pub, 0.0)
            self.send_gripper_command(self.left_gripper_pub, 0.0)
            if elapsed > 2.0:  # Give it 2 seconds to open
                self.state = 'WAIT_FOR_INPUT_1'
                self.state_start_time = now

        # State: Close grippers
        elif self.state == 'CLOSE_GRIPPERS':
            self.send_gripper_command(self.right_gripper_pub, 1.0)
            self.send_gripper_command(self.left_gripper_pub, 1.0)
            if elapsed > 2.0:  # Give it 2 seconds to close
                self.state = 'WAIT_FOR_INPUT_2'
                self.state_start_time = now

        # State: Reopen grippers
        elif self.state == 'REOPEN_GRIPPERS':
            self.send_gripper_command(self.right_gripper_pub, 0.0)
            self.send_gripper_command(self.left_gripper_pub, 0.0)
            if elapsed > 2.0:  # Give it 2 seconds to open
                self.state = 'DONE'
                self.state_start_time = now

        # State: Done (keep grippers open)
        elif self.state == 'DONE':
            self.send_gripper_command(self.right_gripper_pub, 0.0)
            self.send_gripper_command(self.left_gripper_pub, 0.0)


def main():
    rclpy.init()
    node = TestGrippersSim()

    # Create timer for control loop (50Hz like test_arms_sim.py)
    timer = node.create_timer(0.02, node.control_loop)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
