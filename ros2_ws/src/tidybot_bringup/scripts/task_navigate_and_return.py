#!/usr/bin/env python3
"""
Navigate to YOLO target, then return to initial odometry pose via waypoints.

On the way to the object, waypoints are recorded periodically. On the way back,
the robot traces through those waypoints in reverse order to reach home.
This avoids relying on YOLO or direct go-to-goal for the return path.

Usage:
    ros2 run tidybot_bringup task_navigate_and_return.py
    ros2 run tidybot_bringup task_navigate_and_return.py --ros-args -p target_class_id:=46
"""

import numpy as np
import rclpy

from task_navigate_with_avoidance import NavigationTaskNode


class NavigateAndReturnNode(NavigationTaskNode):
    """Extends seek-target behavior by returning to start via recorded waypoints."""

    RETURN_ENABLED = True
    MIN_WAYPOINT_DISTANCE_M = 0.35
    RETURN_POSITION_TOLERANCE = 0.12
    RETURN_ANGLE_TOLERANCE = 0.20
    RETURN_TURN_STEP_RAD = 0.10

    def __init__(self):
        super().__init__()
        self.return_enabled = bool(
            self.declare_parameter('return_enabled', self.RETURN_ENABLED).value
        )
        self.min_waypoint_distance_m = float(
            self.declare_parameter(
                'min_waypoint_distance_m',
                self.MIN_WAYPOINT_DISTANCE_M,
            ).value
        )
        self.return_position_tolerance = float(
            self.declare_parameter(
                'return_position_tolerance',
                self.RETURN_POSITION_TOLERANCE,
            ).value
        )
        self.return_angle_tolerance = float(
            self.declare_parameter(
                'return_angle_tolerance',
                self.RETURN_ANGLE_TOLERANCE,
            ).value
        )
        self.return_turn_step_rad = float(
            self.declare_parameter('return_turn_step_rad', self.RETURN_TURN_STEP_RAD).value
        )

        self.home_pose_set = False
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_theta = 0.0
        self.return_home_active = False
        self.return_home_complete = False

        # Waypoint recording (outbound)
        self.waypoints = []
        self.last_waypoint_x = 0.0
        self.last_waypoint_y = 0.0

        # Waypoint-based return
        self.return_waypoint_index = -1

        self.get_logger().info(
            f'Return-home behavior: {"enabled" if self.return_enabled else "disabled"} '
            f'(waypoint strategy, min_dist={self.min_waypoint_distance_m:.2f}m)'
        )

    def odom_callback(self, msg):
        super().odom_callback(msg)
        if not self.home_pose_set and self.odom_received:
            self.home_x = self.current_x
            self.home_y = self.current_y
            self.home_theta = self.current_theta
            self.last_waypoint_x = self.home_x
            self.last_waypoint_y = self.home_y
            self.home_pose_set = True
            self.get_logger().info(
                f'Home pose recorded: ({self.home_x:.2f}, {self.home_y:.2f}), '
                f'heading {np.rad2deg(self.home_theta):.0f} deg'
            )

        # Record waypoints while seeking target (outbound only)
        if (
            self.return_enabled
            and self.home_pose_set
            and not self.return_home_active
            and not self.red_target_reached_logged
            and self.navigation_mode == 'seek_target'
        ):
            dist = float(
                np.hypot(
                    self.current_x - self.last_waypoint_x,
                    self.current_y - self.last_waypoint_y,
                )
            )
            if dist >= self.min_waypoint_distance_m:
                self.waypoints.append((self.current_x, self.current_y))
                self.last_waypoint_x = self.current_x
                self.last_waypoint_y = self.current_y

    def handle_init(self, current_time):
        if self.odom_received and not self.home_pose_set:
            self.home_x = self.current_x
            self.home_y = self.current_y
            self.home_theta = self.current_theta
            self.last_waypoint_x = self.home_x
            self.last_waypoint_y = self.home_y
            self.home_pose_set = True
            self.get_logger().info(
                f'Home pose recorded: ({self.home_x:.2f}, {self.home_y:.2f}), '
                f'heading {np.rad2deg(self.home_theta):.0f} deg'
            )
        super().handle_init(current_time)

    def handle_seek_target(self, current_time):
        if self.return_home_active:
            self._handle_return_home()
            return

        super().handle_seek_target(current_time)

        if (
            self.return_enabled
            and self.red_target_reached_logged
            and not self.return_home_active
            and not self.return_home_complete
            and self.home_pose_set
        ):
            self.return_home_active = True
            self.seek_mode = 'search_scan'
            self.return_waypoint_index = len(self.waypoints) - 1 if self.waypoints else -1
            self.get_logger().info(
                f'Target reached. Returning to home via {len(self.waypoints)} waypoints. '
                f'Home: ({self.home_x:.2f}, {self.home_y:.2f})'
            )

    def _handle_return_home(self):
        if not self.home_pose_set:
            self.publish_hold_position()
            return

        # Navigate to waypoints in reverse order, then home
        if self.return_waypoint_index >= 0:
            # Going to next waypoint
            wx, wy = self.waypoints[self.return_waypoint_index]
            dx = wx - self.current_x
            dy = wy - self.current_y
            distance = float(np.hypot(dx, dy))

            if distance <= self.return_position_tolerance:
                self.return_waypoint_index -= 1
                remaining = self.return_waypoint_index + 1
                self.get_logger().info(
                    f'Reached waypoint. {remaining} waypoint(s) remaining to home.'
                )
                return

            # Obstacle: rotate to clear
            if self.is_obstacle_blocking():
                angle_to_waypoint = np.arctan2(dy, dx)
                angle_error = self._wrapped_angle_diff(angle_to_waypoint, self.current_theta)
                rotate_theta = (
                    self.current_theta
                    + np.sign(angle_error) * self.return_turn_step_rad
                )
                self.publish_pose_target(self.current_x, self.current_y, rotate_theta)
                return

            # Face toward waypoint so robot drives forward
            target_theta = np.arctan2(dy, dx)
            self.publish_pose_target(wx, wy, target_theta)
            return

        # Going to home (final goal)
        dx = self.home_x - self.current_x
        dy = self.home_y - self.current_y
        distance = float(np.hypot(dx, dy))
        angle_error = self._wrapped_angle_diff(self.home_theta, self.current_theta)

        if (
            distance <= self.return_position_tolerance
            and abs(angle_error) <= self.return_angle_tolerance
        ):
            if not self.return_home_complete:
                self.get_logger().info('Returned to home pose. Holding position.')
            self.return_home_complete = True
            self.return_home_active = False
            self.publish_hold_position()
            return

        if self.is_obstacle_blocking():
            rotate_theta = (
                self.current_theta
                + np.sign(angle_error) * self.return_turn_step_rad
            )
            self.publish_pose_target(self.current_x, self.current_y, rotate_theta)
            return

        self.publish_pose_target(self.home_x, self.home_y, self.home_theta)


def main(args=None):
    rclpy.init(args=args)
    node = NavigateAndReturnNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

