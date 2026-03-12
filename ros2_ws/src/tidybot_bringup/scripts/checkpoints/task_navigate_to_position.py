#!/usr/bin/env python3
"""
Interactive nav using goal pose in RViz

Usage:
    ros2 run tidybot_bringup task_navigate_to_position.py

Press 'G' or find the 2D Goal Pose button to set goal pose
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import Odometry
from enum import Enum
import numpy as np
from scipy.spatial.transform import Rotation


class State(Enum):
    INIT = 0
    NAVIGATE_TO_TARGET = 1
    ARRIVED = 2
    DONE = 3


class NavigationTaskNode(Node):
    """Click goals in RViz to navigate TidyBot around."""

    # Arrival tolerances
    POSITION_TOLERANCE = 0.1  #meters
    ANGLE_TOLERANCE = 0.15  #rads

    # Hold time at goal before accepting new goal pose
    ARRIVAL_HOLD_TIME = 1.0

    def __init__(self):
        super().__init__('navigation_task_node')

        # Publishing movement info
        self.base_pose_pub = self.create_publisher(Pose2D, '/base/target_pose', 10)

        # Subscriber
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # State tracking
        self.state = State.INIT
        self.state_start_time = self.get_clock().now()

        # Current pose (from odometry)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_received = False

        # Target pose from RViz
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0
        self.has_goal = False

        # Main control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Ready! Click "2D Nav Goal" in RViz (or press G) to navigate.')

    def odom_callback(self, msg: Odometry):
        """Track where the robot currently is."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract heading from quaternion
        quat = msg.pose.pose.orientation
        rotation = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        euler = rotation.as_euler('xyz', degrees=False)
        mujoco_theta = euler[2]

        # Convert from MuJoCo frame (theta=π/2 is +X) to user frame (theta=0 is +X)
        self.current_theta = mujoco_theta - np.pi / 2

        self.odom_received = True

    def goal_callback(self, msg: PoseStamped):
        """Handle RViz clicks - extract target position and heading."""
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y

        # Extract heading from the arrow direction
        quat = msg.pose.orientation
        rotation = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        euler = rotation.as_euler('xyz', degrees=False)
        self.target_theta = euler[2]

        self.has_goal = True

        # If we're sitting idle, start navigating immediately
        if self.state in [State.ARRIVED, State.DONE]:
            self.state = State.NAVIGATE_TO_TARGET
            self.state_start_time = self.get_clock().now()

        self.get_logger().info(
            f'New goal: ({self.target_x:.2f}, {self.target_y:.2f}) '
            f'facing {np.rad2deg(self.target_theta):.0f}°'
        )

    def get_distance_to_target(self) -> float:
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        return np.sqrt(dx**2 + dy**2)

    def get_angle_error(self) -> float:
        error = self.target_theta - self.current_theta
        # Wrap to [-π, π]
        while error > np.pi:
            error -= 2 * np.pi
        while error < -np.pi:
            error += 2 * np.pi
        return error

    def is_at_target(self) -> bool:
        distance = self.get_distance_to_target()
        angle_error = abs(self.get_angle_error())
        return distance < self.POSITION_TOLERANCE and angle_error < self.ANGLE_TOLERANCE


    def control_loop(self):
        current_time = self.get_clock().now()

        if self.state == State.INIT:
            self.handle_init(current_time)
        elif self.state == State.NAVIGATE_TO_TARGET:
            self.handle_navigate(current_time)
        elif self.state == State.ARRIVED:
            self.handle_arrived(current_time)
        elif self.state == State.DONE:
            self.handle_done()

    def handle_init(self, current_time):
        """Wait for odometry and first goal click."""
        if not self.odom_received or not self.has_goal:
            return

        self.get_logger().info(f'Starting from ({self.current_x:.2f}, {self.current_y:.2f})')
        self.transition_to_state(State.NAVIGATE_TO_TARGET, current_time)

    def handle_navigate(self, current_time):
        """Publish target and check if we've arrived."""
        # Send target to base controller
        target = Pose2D()
        target.x = self.target_x
        target.y = self.target_y
        target.theta = self.target_theta
        self.base_pose_pub.publish(target)

        # Log progress every 2 seconds
        time_in_state = (current_time - self.state_start_time).nanoseconds / 1e9
        if int(time_in_state) % 2 == 0 and int(time_in_state * 10) % 20 == 0:
            distance = self.get_distance_to_target()
            self.get_logger().info(f'{distance:.2f}m away...')

        # Are we there yet?
        if self.is_at_target():
            self.get_logger().info('Arrived!')
            self.transition_to_state(State.ARRIVED, current_time)

    def handle_arrived(self, current_time):
        """Hold position briefly, then mark ready for next goal."""
        # Keep holding the target position
        target = Pose2D()
        target.x = self.target_x
        target.y = self.target_y
        target.theta = self.target_theta
        self.base_pose_pub.publish(target)

        # After holding for a bit, mark as done
        time_in_state = (current_time - self.state_start_time).nanoseconds / 1e9
        if time_in_state >= self.ARRIVAL_HOLD_TIME:
            self.get_logger().info('Ready for next goal!')
            self.transition_to_state(State.DONE, current_time)

    def handle_done(self):
        """Idle state - waiting for next RViz click."""
        # Hold current position
        target = Pose2D()
        target.x = self.current_x
        target.y = self.current_y
        target.theta = self.current_theta
        self.base_pose_pub.publish(target)

    def transition_to_state(self, new_state: State, current_time):
        self.state = new_state
        self.state_start_time = current_time


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = NavigationTaskNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
