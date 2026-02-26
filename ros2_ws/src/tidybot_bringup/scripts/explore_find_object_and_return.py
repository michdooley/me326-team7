#!/usr/bin/env python3
"""
TidyBot2 Explore, Find Object, and Return

Wraps the existing Explore & Find behavior, then navigates back to the
robot's starting XY position after the target object is reached.

Usage:
    ros2 run tidybot_bringup explore_find_object_and_return.py --ros-args -p target_color:=red
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration

from explore_and_find_object import ExploreAndFind, ExploreState


class ExploreFindObjectAndReturn(ExploreAndFind):
    RETURNING_HOME = 'returning_home'

    def __init__(self):
        super().__init__()
        self.start_world_pos = None
        self.returning_home = False
        self.return_complete = False

    def _capture_start_position(self):
        if self.start_world_pos is not None:
            return True

        pose = self.get_base_pose()
        if pose is None:
            return False

        self.start_world_pos = (pose[0], pose[1])
        self.get_logger().info(
            f'[RETURN] Start position saved at '
            f'({pose[0]:.2f}, {pose[1]:.2f})')
        return True

    def _plan_return_home(self):
        if self.start_world_pos is None:
            self.get_logger().warn('[RETURN] Start position was never captured.')
            return False

        pose = self.get_base_pose()
        if pose is None:
            return False

        bx, by, _ = pose
        sx, sy = self.start_world_pos

        if math.hypot(sx - bx, sy - by) < self.WAYPOINT_TOLERANCE:
            self._stop_base()
            self.returning_home = False
            self.return_complete = True
            self.state = ExploreState.COMPLETE
            self.get_logger().info('[RETURN] Already back at the start position.')
            return True

        robot_gx, robot_gy = self.world_to_grid(bx, by)
        goal_gx, goal_gy = self.world_to_grid(sx, sy)

        clearances = [None, self.ROBOT_CLEARANCE * 0.5]
        labels = ['full', 'half']
        for clearance, label in zip(clearances, labels):
            path = self._plan_path(
                robot_gx, robot_gy, goal_gx, goal_gy, clearance_m=clearance)
            if path is None:
                continue

            self._stop_base()
            self.nav_waypoints = path
            self.nav_waypoint_idx = 0
            self.nav_start_time = time.time()
            self.nav_clearance = clearance
            self.returning_home = True
            self.state = self.RETURNING_HOME
            self.get_logger().info(
                f'[RETURN] Planned {len(path)} waypoints back to '
                f'({sx:.2f}, {sy:.2f}) ({label} inflation)')
            return True

        self.get_logger().warn(
            f'[RETURN] Could not plan a path back to ({sx:.2f}, {sy:.2f}).')
        return False

    def _state_returning_home(self):
        now = time.time()

        if now - self.nav_start_time > self.NAV_TIMEOUT:
            self._stop_base()
            self.returning_home = False
            self.return_complete = True
            self.get_logger().warn('[RETURN] Timeout while returning home.')
            self.state = ExploreState.COMPLETE
            return

        pose = self.get_base_pose()
        if pose is None:
            return
        bx, by, btheta = pose
        actual_heading = btheta - math.pi / 2

        if self._is_path_blocked():
            self.get_logger().info('[RETURN] Path blocked on map — re-planning...')
            if self._replan_from_here():
                return
            self._stop_base()
            self.returning_home = False
            self.return_complete = True
            self.get_logger().warn('[RETURN] Re-plan failed while returning home.')
            self.state = ExploreState.COMPLETE
            return

        if self.nav_waypoint_idx >= len(self.nav_waypoints):
            self._stop_base()
            self.returning_home = False
            self.return_complete = True
            self.get_logger().info('[RETURN] Start position reached.')
            self.state = ExploreState.COMPLETE
            return

        wp_gx, wp_gy = self.nav_waypoints[self.nav_waypoint_idx]
        wp_wx, wp_wy = self.grid_to_world(wp_gx, wp_gy)
        dx, dy = wp_wx - bx, wp_wy - by
        dist = math.hypot(dx, dy)

        if dist < self.WAYPOINT_TOLERANCE:
            self.nav_waypoint_idx += 1
            self.nav_start_time = time.time()
            if self.nav_waypoint_idx >= len(self.nav_waypoints):
                self._stop_base()
                self.returning_home = False
                self.return_complete = True
                self.get_logger().info('[RETURN] Start position reached.')
                self.state = ExploreState.COMPLETE
                return
            wp_gx, wp_gy = self.nav_waypoints[self.nav_waypoint_idx]
            wp_wx, wp_wy = self.grid_to_world(wp_gx, wp_gy)
            dx, dy = wp_wx - bx, wp_wy - by
            dist = math.hypot(dx, dy)

        desired_heading = math.atan2(dy, dx)
        heading_error = self._normalize_angle(desired_heading - actual_heading)

        if (not hasattr(self, '_return_last_log') or
                now - self._return_last_log > 2.0):
            self._return_last_log = now
            self.get_logger().info(
                f'[RETURN] wp {self.nav_waypoint_idx}/{len(self.nav_waypoints)} '
                f'dist={dist:.2f} pos=({bx:.2f},{by:.2f})')

        cmd = Twist()

        clearance, distances = self._analyze_depth_obstacles()
        left_clear, center_clear, right_clear = clearance
        left_dist, center_dist, right_dist = distances
        min_dist = min(left_dist, center_dist, right_dist)

        if not center_clear:
            if self.depth_steer_start is None:
                self.depth_steer_start = now
            elif now - self.depth_steer_start > self.DEPTH_STEER_TIMEOUT:
                self.get_logger().info(
                    '[RETURN] Stuck on depth obstacle — forcing re-plan.')
                self.depth_steer_start = None
                self._stop_base()
                if self._replan_from_here():
                    return
                self.returning_home = False
                self.return_complete = True
                self.get_logger().warn(
                    '[RETURN] Re-plan failed while returning home.')
                self.state = ExploreState.COMPLETE
                return
            if left_dist > right_dist:
                cmd.angular.z = 1.0
            elif right_dist > left_dist:
                cmd.angular.z = -1.0
            else:
                cmd.angular.z = 1.0
            cmd.linear.x = 0.0
        else:
            self.depth_steer_start = None
            alignment = max(0.0, math.cos(heading_error))
            speed = self.NAV_LINEAR_SPEED * alignment
            if min_dist < 1.5:
                speed *= min(min_dist / 1.5, 1.0)
            cmd.linear.x = float(min(speed, dist * 0.5))
            cmd.angular.z = float(max(min(2.0 * heading_error, 1.2), -1.2))

        if min_dist < 0.35:
            cmd.linear.x = 0.0

        self.last_cmd_angular = cmd.angular.z
        self.cmd_vel_pub.publish(cmd)

    def _state_complete(self):
        if (self.object_world_pos is not None and
                not self.returning_home and
                not self.return_complete):
            if self._plan_return_home():
                return
            self.return_complete = True

        super()._state_complete()

    def run(self):
        self.get_logger().info('Waiting for depth data and TF...')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_depth is not None and self.camera_K is not None:
                try:
                    self.tf_buffer.lookup_transform(
                        'odom', 'camera_depth_optical_frame',
                        rclpy.time.Time(), timeout=Duration(seconds=0.1))
                    if self._capture_start_position():
                        break
                except Exception:
                    pass

        self.get_logger().info(
            f'Sensors ready — exploring for {self.target_color} cube!')
        self._start_scan()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            now = time.time()
            if now - self.last_map_publish >= 1.0 / self.MAP_PUBLISH_RATE:
                self.publish_map()
                if self.object_world_pos is not None:
                    self.publish_object_marker()
                self.last_map_publish = now

            if (self.state == ExploreState.SCANNING and
                    self.object_world_pos is None):
                self._check_for_object()

            if self.state == ExploreState.SCANNING:
                self._state_scanning()
            elif self.state == ExploreState.SELECTING:
                self._state_selecting()
            elif self.state == ExploreState.NAVIGATING:
                self._state_navigating()
            elif self.state == ExploreState.APPROACHING:
                self._state_approaching()
            elif self.state == self.RETURNING_HOME:
                self._state_returning_home()
            elif self.state == ExploreState.COMPLETE:
                self._state_complete()
                if self.return_complete or self.object_world_pos is None:
                    break


def main(args=None):
    rclpy.init(args=args)
    node = ExploreFindObjectAndReturn()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
