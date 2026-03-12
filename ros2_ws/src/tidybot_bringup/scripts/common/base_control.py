"""Base control mixin: base movement, pose estimation, waypoint recording."""

import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import Twist

from common.constants import normalize_angle


class BaseControlMixin:
    """Mixin providing base control, pose estimation, and waypoint recording.

    Expects the host class to have:
        self.cmd_vel_pub, self.tf_buffer, self.waypoints,
        self.WAYPOINT_INTERVAL
    """

    def _stop_base(self):
        self.cmd_vel_pub.publish(Twist())

    def _spin_for(self, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def get_base_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(f'TF odom->base_link: {e}')
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return t.x, t.y, np.arctan2(siny, cosy)

    def get_heading(self):
        p = self.get_base_pose()
        return p[2] if p is not None else None

    @staticmethod
    def _normalize_angle(angle):
        return normalize_angle(angle)

    def _maybe_record_waypoint(self):
        pose = self.get_base_pose()
        if pose is None:
            return
        x, y, _ = pose
        if self.waypoints:
            last_x, last_y = self.waypoints[-1]
            dist = np.hypot(x - last_x, y - last_y)
            if dist < self.WAYPOINT_INTERVAL:
                return
        self.waypoints.append((x, y))
