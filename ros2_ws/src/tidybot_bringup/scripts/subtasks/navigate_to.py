"""
NavigateTo subtask — Drive to a target pose with reactive obstacle avoidance.

Uses /cmd_vel exclusively with a proportional heading controller and
depth-based obstacle detection in three sectors (left/center/right).
"""

import time
from enum import Enum

import numpy as np
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

import tf2_ros

from . import SubtaskStatus


class _NavState(Enum):
    IDLE = 0
    ROTATING_TO_GOAL = 1
    NAVIGATING = 2
    FINAL_ROTATE = 3
    ARRIVED = 4
    FAILED = 5


class NavigateTo:
    """Navigate to a target pose in odom frame with reactive depth obstacle avoidance."""

    def __init__(self, node):
        self.node = node
        self.started = False
        self._nav_state = _NavState.IDLE

        # Target (set in start())
        self._target_x = 0.0
        self._target_y = 0.0
        self._target_theta_mujoco = 0.0  # in MuJoCo convention (pi/2 = facing +X)

        # Parameters
        self._position_tolerance = 0.15
        self._angle_tolerance = 0.2
        self._obstacle_threshold = 0.5
        self._linear_speed = 0.25
        self._timeout = 30.0

        # Internal
        self._start_time = 0.0
        self._final_pose = None

    def start(self, target_x, target_y, target_theta,
              position_tolerance=0.15, angle_tolerance=0.2,
              obstacle_threshold=0.5, linear_speed=0.25, timeout=30.0):
        """Start navigation to target pose.

        Args:
            target_x, target_y: Target position in odom frame (meters).
            target_theta: Target heading as real-world angle (0 = facing +X).
                          Internally converted to MuJoCo convention.
            position_tolerance: Distance to accept arrival (meters).
            angle_tolerance: Angle to accept arrival (radians).
            obstacle_threshold: Depth distance to trigger avoidance (meters).
            linear_speed: Forward speed (m/s).
            timeout: Maximum navigation time (seconds).
        """
        self.started = True
        self._target_x = target_x
        self._target_y = target_y
        # Convert real-world heading to MuJoCo convention: mujoco_th = heading + pi/2
        self._target_theta_mujoco = target_theta + np.pi / 2

        self._position_tolerance = position_tolerance
        self._angle_tolerance = angle_tolerance
        self._obstacle_threshold = obstacle_threshold
        self._linear_speed = linear_speed
        self._timeout = timeout

        self._start_time = time.time()
        self._nav_state = _NavState.NAVIGATING
        self._final_pose = None

        # Reset camera tilt to near-level for depth obstacle sensing
        msg = Float64MultiArray()
        msg.data = [0.0, 0.1]
        self.node.pan_tilt_pub.publish(msg)

        self.node.get_logger().info(
            f'[NAV] Navigating to ({target_x:.2f}, {target_y:.2f}, '
            f'{np.degrees(target_theta):.0f}°)')

    def update(self) -> SubtaskStatus:
        """Called at 20Hz. Returns current status."""
        if not self.started:
            return SubtaskStatus.FAILED

        if self._nav_state == _NavState.NAVIGATING:
            return self._handle_navigating()
        elif self._nav_state == _NavState.FINAL_ROTATE:
            return self._handle_final_rotate()
        elif self._nav_state == _NavState.ARRIVED:
            return SubtaskStatus.SUCCEEDED
        elif self._nav_state == _NavState.FAILED:
            return SubtaskStatus.FAILED

        return SubtaskStatus.RUNNING

    def get_result(self):
        """Return final pose (x, y, theta) in odom frame."""
        return self._final_pose

    def stop(self):
        """Emergency stop — zero velocities."""
        self._publish_twist(0.0, 0.0)
        self.started = False

    # ── State handlers ────────────────────────────────────────────────

    def _handle_navigating(self) -> SubtaskStatus:
        """Main navigation loop with obstacle avoidance."""
        # Check timeout
        if time.time() - self._start_time > self._timeout:
            self._publish_twist(0.0, 0.0)
            self.node.get_logger().warn('[NAV] Navigation timed out')
            self._nav_state = _NavState.FAILED
            return SubtaskStatus.FAILED

        # Get current pose
        pose = self._get_base_pose()
        if pose is None:
            return SubtaskStatus.RUNNING

        cx, cy, ctheta = pose
        dx = self._target_x - cx
        dy = self._target_y - cy
        distance = np.sqrt(dx**2 + dy**2)

        # Check position arrival
        if distance < self._position_tolerance:
            self._publish_twist(0.0, 0.0)
            self._nav_state = _NavState.FINAL_ROTATE
            self.node.get_logger().info('[NAV] At position, rotating to final heading...')
            return SubtaskStatus.RUNNING

        # Desired world heading toward target
        desired_heading = np.arctan2(dy, dx)
        # Robot's actual heading: MuJoCo theta - pi/2
        actual_heading = ctheta - np.pi / 2
        heading_error = self._normalize_angle(desired_heading - actual_heading)

        # Analyze obstacles
        clearance, distances = self._analyze_depth_obstacles()
        left_clear, center_clear, right_clear = clearance
        left_dist, center_dist, right_dist = distances

        twist = Twist()

        if center_clear:
            # No obstacle ahead — drive toward target
            if abs(heading_error) > 0.4:
                # Large heading error: rotate first
                twist.angular.z = np.clip(2.0 * heading_error, -1.0, 1.0)
            else:
                # Drive forward with heading correction
                twist.linear.x = min(self._linear_speed, distance * 0.5)
                twist.angular.z = np.clip(1.5 * heading_error, -0.8, 0.8)
        else:
            # Center blocked — steer around
            self.node.get_logger().info(
                f'[NAV] Obstacle! L={left_dist:.2f} C={center_dist:.2f} R={right_dist:.2f}')
            if left_dist > right_dist:
                twist.angular.z = 0.5
                twist.linear.x = 0.05
            elif right_dist > left_dist:
                twist.angular.z = -0.5
                twist.linear.x = 0.05
            else:
                # All blocked — rotate in place
                twist.angular.z = 0.6

        # Safety: stop forward if very close to anything
        if min(left_dist, center_dist, right_dist) < 0.25:
            twist.linear.x = 0.0

        self.node.cmd_vel_pub.publish(twist)
        return SubtaskStatus.RUNNING

    def _handle_final_rotate(self) -> SubtaskStatus:
        """Rotate to final heading after reaching position."""
        pose = self._get_base_pose()
        if pose is None:
            return SubtaskStatus.RUNNING

        cx, cy, ctheta = pose
        angle_error = self._normalize_angle(self._target_theta_mujoco - ctheta)

        if abs(angle_error) < self._angle_tolerance:
            self._publish_twist(0.0, 0.0)
            self._final_pose = (cx, cy, ctheta)
            self._nav_state = _NavState.ARRIVED
            self.node.get_logger().info('[NAV] Arrived at target!')
            return SubtaskStatus.SUCCEEDED

        # Check timeout
        if time.time() - self._start_time > self._timeout:
            self._publish_twist(0.0, 0.0)
            self._final_pose = (cx, cy, ctheta)
            self._nav_state = _NavState.ARRIVED
            self.node.get_logger().warn('[NAV] Final rotate timed out, accepting current heading')
            return SubtaskStatus.SUCCEEDED

        twist = Twist()
        twist.angular.z = np.clip(2.0 * angle_error, -1.0, 1.0)
        self.node.cmd_vel_pub.publish(twist)
        return SubtaskStatus.RUNNING

    # ── Obstacle detection ────────────────────────────────────────────

    def _analyze_depth_obstacles(self):
        """Analyze depth image for obstacles in left/center/right sectors.

        Returns:
            (clearance_tuple, distance_tuple) where each is (left, center, right).
        """
        depth = self.node.latest_depth
        if depth is None:
            return (True, True, True), (999.0, 999.0, 999.0)

        h, w = depth.shape

        # Vertical band: ignore top 30% (ceiling/sky) and bottom 20% (ground)
        v_start = int(h * 0.3)
        v_end = int(h * 0.8)

        left_region = depth[v_start:v_end, 0:w // 3]
        center_region = depth[v_start:v_end, w // 3:2 * w // 3]
        right_region = depth[v_start:v_end, 2 * w // 3:w]

        def sector_min_dist(region):
            valid = region[region > 0].astype(np.float64)
            if len(valid) == 0:
                return 999.0
            return np.percentile(valid, 10) / 1000.0  # mm → meters

        left_dist = sector_min_dist(left_region)
        center_dist = sector_min_dist(center_region)
        right_dist = sector_min_dist(right_region)

        threshold = self._obstacle_threshold
        clearance = (left_dist > threshold, center_dist > threshold, right_dist > threshold)
        return clearance, (left_dist, center_dist, right_dist)

    # ── Helpers ───────────────────────────────────────────────────────

    def _get_base_pose(self):
        """Get (x, y, theta) from odom→base_link TF. Theta in MuJoCo convention."""
        try:
            transform = self.node.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = np.arctan2(siny, cosy)
        return (t.x, t.y, theta)

    @staticmethod
    def _normalize_angle(angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def _publish_twist(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.node.cmd_vel_pub.publish(twist)
