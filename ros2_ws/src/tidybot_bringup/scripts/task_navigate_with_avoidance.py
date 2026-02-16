#!/usr/bin/env python3
"""
Interactive nav using goal pose in RViz

Usage:
    ros2 run tidybot_bringup task_navigate_to_position.py

Press 'G' or find the 2D Goal Pose button to set goal pose
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from enum import Enum
import numpy as np
from scipy.spatial.transform import Rotation

try:
    from cv_bridge import CvBridge
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


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
    DEPTH_MIN_MM = 150
    DEPTH_MAX_MM = 1800
    MIN_COMPONENT_PIXELS = 400
    DETECTION_LOG_INTERVAL = 1.0
    STOP_DISTANCE_M = 0.55
    CRITICAL_STOP_DISTANCE_M = 0.35
    FORWARD_REGION_X_MIN_FRAC = 0.25
    FORWARD_REGION_X_MAX_FRAC = 0.75
    FORWARD_REGION_Y_MIN_FRAC = 0.20
    MIN_BLOCKING_AREA_FRAC = 0.01
    STOP_LOG_INTERVAL = 1.0

    def __init__(self):
        super().__init__('navigation_task_node')

        # Publishing movement info
        self.base_pose_pub = self.create_publisher(Pose2D, '/base/target_pose', 10)

        # Subscriber
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Camera-based obstacle detection state
        self.obstacle_bbox_depth = None  # (x, y, w, h) in depth image coordinates
        self.obstacle_distance_m = None
        self.latest_depth_shape = None
        self.last_detection_log_time = self.get_clock().now()
        self.last_stop_log_time = self.get_clock().now()
        self.was_blocked = False

        if CV_AVAILABLE:
            camera_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.cv_bridge = CvBridge()
            self.color_sub = self.create_subscription(
                Image, '/camera/color/image_raw', self.color_callback, camera_qos
            )
            self.depth_sub = self.create_subscription(
                Image, '/camera/depth/image_raw', self.depth_callback, camera_qos
            )
            self.overlay_pub = self.create_publisher(Image, '/camera/obstacle_overlay', 10)
        else:
            self.get_logger().warn(
                'cv_bridge/opencv not available, camera obstacle overlay is disabled.'
            )

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
        if CV_AVAILABLE:
            self.get_logger().info(
                'Obstacle overlay publisher active on /camera/obstacle_overlay'
            )

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

    def depth_callback(self, msg: Image):
        """Detect the nearest obstacle region from depth image."""
        try:
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')
            return

        if depth_image is None:
            return

        if depth_image.ndim > 2:
            depth_image = depth_image[:, :, 0]

        self.latest_depth_shape = depth_image.shape[:2]
        depth_mm = self._to_mm_depth(depth_image)
        self.obstacle_bbox_depth, self.obstacle_distance_m = self._detect_obstacle(depth_mm)

        if self.obstacle_bbox_depth is not None and self.obstacle_distance_m is not None:
            now = self.get_clock().now()
            dt = (now - self.last_detection_log_time).nanoseconds / 1e9
            if dt >= self.DETECTION_LOG_INTERVAL:
                x, y, w, h = self.obstacle_bbox_depth
                self.get_logger().info(
                    f'Obstacle at {self.obstacle_distance_m:.2f}m '
                    f'(depth bbox x={x}, y={y}, w={w}, h={h})'
                )
                self.last_detection_log_time = now

    def color_callback(self, msg: Image):
        """Draw obstacle bounding box onto color frame and publish overlay."""
        if self.latest_depth_shape is None:
            return

        try:
            color_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Color conversion failed: {e}')
            return

        if color_image is None:
            return

        overlay = color_image.copy()

        if self.obstacle_bbox_depth is not None and self.obstacle_distance_m is not None:
            bbox_color = self._map_bbox_to_color(
                self.obstacle_bbox_depth,
                self.latest_depth_shape,
                overlay.shape[:2],
            )
            x, y, w, h = bbox_color
            is_blocking = self.is_obstacle_blocking()
            box_color = (0, 0, 255) if is_blocking else (0, 255, 255)
            cv2.rectangle(overlay, (x, y), (x + w, y + h), box_color, 2)
            status = 'STOP' if is_blocking else 'Obstacle'
            label = f'{status}: {self.obstacle_distance_m:.2f} m'
            cv2.putText(
                overlay,
                label,
                (x, max(20, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                box_color,
                2,
            )

        overlay_msg = self.cv_bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        overlay_msg.header = msg.header
        self.overlay_pub.publish(overlay_msg)

    def _to_mm_depth(self, depth_image: np.ndarray) -> np.ndarray:
        """Normalize depth image to millimeters."""
        if np.issubdtype(depth_image.dtype, np.floating):
            depth_mm = depth_image.astype(np.float32)
            if np.nanmax(depth_mm) < 20.0:
                depth_mm *= 1000.0
        else:
            depth_mm = depth_image.astype(np.float32)
        return depth_mm

    def _detect_obstacle(self, depth_mm: np.ndarray):
        """Return (bbox, distance_m) for nearest valid depth cluster."""
        valid = np.isfinite(depth_mm) & (depth_mm > self.DEPTH_MIN_MM) & (depth_mm < self.DEPTH_MAX_MM)
        if not np.any(valid):
            return None, None

        mask = (valid.astype(np.uint8) * 255)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None

        best_bbox = None
        best_depth_mm = None

        for contour in contours:
            if cv2.contourArea(contour) < self.MIN_COMPONENT_PIXELS:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            roi_depth = depth_mm[y:y + h, x:x + w]
            roi_valid = valid[y:y + h, x:x + w]
            if not np.any(roi_valid):
                continue

            median_depth = float(np.median(roi_depth[roi_valid]))
            if best_depth_mm is None or median_depth < best_depth_mm:
                best_depth_mm = median_depth
                best_bbox = (x, y, w, h)

        if best_bbox is None:
            return None, None

        return best_bbox, best_depth_mm / 1000.0

    def _map_bbox_to_color(self, bbox_depth, depth_shape, color_shape):
        """Scale depth-image bbox to color-image coordinates."""
        dx, dy, dw, dh = bbox_depth
        depth_h, depth_w = depth_shape
        color_h, color_w = color_shape

        scale_x = color_w / float(depth_w)
        scale_y = color_h / float(depth_h)

        x = int(dx * scale_x)
        y = int(dy * scale_y)
        w = int(dw * scale_x)
        h = int(dh * scale_y)

        x = max(0, min(x, color_w - 1))
        y = max(0, min(y, color_h - 1))
        w = max(1, min(w, color_w - x))
        h = max(1, min(h, color_h - y))
        return x, y, w, h

    def is_obstacle_blocking(self) -> bool:
        """True when a near obstacle overlaps the robot's forward camera region."""
        if self.obstacle_bbox_depth is None or self.obstacle_distance_m is None:
            return False
        if self.latest_depth_shape is None:
            return False
        if self.obstacle_distance_m > self.STOP_DISTANCE_M:
            return False

        depth_h, depth_w = self.latest_depth_shape
        x, y, w, h = self.obstacle_bbox_depth
        x2 = x + w
        y2 = y + h

        lane_x1 = int(depth_w * self.FORWARD_REGION_X_MIN_FRAC)
        lane_x2 = int(depth_w * self.FORWARD_REGION_X_MAX_FRAC)
        lane_y1 = int(depth_h * self.FORWARD_REGION_Y_MIN_FRAC)

        overlaps_forward_region = (x2 > lane_x1) and (x < lane_x2) and (y2 > lane_y1)
        area_frac = (w * h) / float(depth_w * depth_h)
        is_large_enough = area_frac >= self.MIN_BLOCKING_AREA_FRAC
        is_critical = self.obstacle_distance_m <= self.CRITICAL_STOP_DISTANCE_M

        return overlaps_forward_region and (is_large_enough or is_critical)

    def publish_hold_position(self):
        """Command base to hold current robot pose."""
        target = Pose2D()
        target.x = self.current_x
        target.y = self.current_y
        target.theta = self.current_theta
        self.base_pose_pub.publish(target)

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
        if self.is_at_target():
            self.get_logger().info('Arrived!')
            self.transition_to_state(State.ARRIVED, current_time)
            return

        if self.is_obstacle_blocking():
            self.publish_hold_position()
            now = self.get_clock().now()
            dt = (now - self.last_stop_log_time).nanoseconds / 1e9
            if dt >= self.STOP_LOG_INTERVAL:
                self.get_logger().warn(
                    f'Stopping for obstacle at {self.obstacle_distance_m:.2f}m (path blocked)'
                )
                self.last_stop_log_time = now
            self.was_blocked = True
            return

        if self.was_blocked:
            self.get_logger().info('Path cleared. Resuming navigation to goal.')
            self.was_blocked = False

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
        self.publish_hold_position()

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
