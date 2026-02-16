"""
ScanForObject subtask — Rotate 360° scanning for a colored cube.

Rotates the robot body while continuously checking the camera for the target
object using HSV color detection. If not found after a full rotation, drives
forward and scans again.
"""

import time
from enum import Enum

import numpy as np
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

import tf2_ros

try:
    import cv2
except ImportError:
    raise ImportError("opencv-python is required: pip install opencv-python")

from . import SubtaskStatus


class _ScanState(Enum):
    TILTING_CAMERA = 0
    ROTATING = 1
    DRIVING_FORWARD = 2
    DONE = 3
    FAILED = 4


class ScanForObject:
    """Rotate 360° scanning for a target-colored object via HSV detection."""

    # Default HSV ranges for red (widened for MuJoCo lighting)
    DEFAULT_HSV_RANGES = [
        (np.array([0, 40, 30]), np.array([20, 255, 255])),
        (np.array([160, 40, 30]), np.array([180, 255, 255])),
    ]

    def __init__(self, node):
        self.node = node
        self.started = False
        self._scan_state = _ScanState.TILTING_CAMERA
        self._detected_position = None

        # Parameters (set in start())
        self._rotation_speed = 0.4
        self._max_scan_attempts = 3
        self._drive_distance = 0.5
        self._camera_tilt = 0.5
        self._hsv_ranges = self.DEFAULT_HSV_RANGES
        self._min_area = 50

        # Internal tracking
        self._scan_attempt = 0
        self._state_start_time = 0.0
        self._start_yaw = None
        self._prev_yaw = None
        self._total_rotated = 0.0

    def start(self, rotation_speed=0.4, max_scan_attempts=3,
              drive_distance=0.5, camera_tilt=0.5,
              hsv_ranges=None, min_area=50):
        """Start the scan subtask.

        Args:
            rotation_speed: Angular velocity for rotation (rad/s).
            max_scan_attempts: Max scan+drive cycles before failing.
            drive_distance: Distance to drive forward between scans (m).
            camera_tilt: Camera tilt angle for scanning (rad).
            hsv_ranges: List of (lower, upper) HSV arrays. Defaults to red.
            min_area: Minimum contour area in pixels to accept a detection.
        """
        self.started = True
        self._rotation_speed = rotation_speed
        self._max_scan_attempts = max_scan_attempts
        self._drive_distance = drive_distance
        self._camera_tilt = camera_tilt
        self._min_area = min_area
        if hsv_ranges is not None:
            self._hsv_ranges = hsv_ranges

        self._scan_attempt = 0
        self._scan_state = _ScanState.TILTING_CAMERA
        self._state_start_time = time.time()
        self._detected_position = None
        self._total_rotated = 0.0
        self._start_yaw = None
        self._prev_yaw = None

        # Tilt camera
        self._send_pan_tilt(0.0, self._camera_tilt)
        self.node.get_logger().info('[SCAN] Starting scan subtask')

    def update(self) -> SubtaskStatus:
        """Called at 20Hz. Returns current status."""
        if not self.started:
            return SubtaskStatus.FAILED

        if self._scan_state == _ScanState.TILTING_CAMERA:
            return self._handle_tilting()
        elif self._scan_state == _ScanState.ROTATING:
            return self._handle_rotating()
        elif self._scan_state == _ScanState.DRIVING_FORWARD:
            return self._handle_driving()
        elif self._scan_state == _ScanState.DONE:
            return SubtaskStatus.SUCCEEDED
        elif self._scan_state == _ScanState.FAILED:
            return SubtaskStatus.FAILED

        return SubtaskStatus.RUNNING

    def get_result(self):
        """Return detected object position as np.array([x, y, z]) in odom."""
        return self._detected_position

    def stop(self):
        """Emergency stop."""
        self._publish_twist(0.0, 0.0)
        self.started = False

    # ── State handlers ────────────────────────────────────────────────

    def _handle_tilting(self) -> SubtaskStatus:
        """Wait for camera to settle after tilting."""
        if time.time() - self._state_start_time > 1.5:
            self._scan_state = _ScanState.ROTATING
            self._state_start_time = time.time()
            self._start_yaw = self._get_current_yaw()
            self._prev_yaw = self._start_yaw
            self._total_rotated = 0.0
            self.node.get_logger().info(
                f'[SCAN] Scan attempt {self._scan_attempt + 1}/{self._max_scan_attempts} — rotating...')
        return SubtaskStatus.RUNNING

    def _handle_rotating(self) -> SubtaskStatus:
        """Rotate while checking for the object."""
        # Check for object
        detection = self._detect_object()
        if detection is not None:
            self._publish_twist(0.0, 0.0)
            self._detected_position = detection
            self._scan_state = _ScanState.DONE
            self.node.get_logger().info(
                f'[SCAN] Object detected at ({detection[0]:.2f}, '
                f'{detection[1]:.2f}, {detection[2]:.2f})')
            return SubtaskStatus.SUCCEEDED

        # Continue rotating
        self._publish_twist(0.0, self._rotation_speed)

        # Track rotation
        current_yaw = self._get_current_yaw()
        if current_yaw is not None and self._prev_yaw is not None:
            delta = self._normalize_angle(current_yaw - self._prev_yaw)
            self._total_rotated += abs(delta)
            self._prev_yaw = current_yaw

        # Check if we've completed 360°
        if self._total_rotated >= 2.0 * np.pi:
            self._publish_twist(0.0, 0.0)
            self._scan_attempt += 1
            self.node.get_logger().info(
                f'[SCAN] Full rotation complete, no object found '
                f'(attempt {self._scan_attempt}/{self._max_scan_attempts})')

            if self._scan_attempt >= self._max_scan_attempts:
                self._scan_state = _ScanState.FAILED
                return SubtaskStatus.FAILED

            # Drive forward and try again
            self._scan_state = _ScanState.DRIVING_FORWARD
            self._state_start_time = time.time()

        return SubtaskStatus.RUNNING

    def _handle_driving(self) -> SubtaskStatus:
        """Drive forward between scan attempts."""
        drive_speed = 0.2
        drive_time = self._drive_distance / drive_speed

        if time.time() - self._state_start_time < drive_time:
            self._publish_twist(drive_speed, 0.0)
        else:
            self._publish_twist(0.0, 0.0)
            self._scan_state = _ScanState.TILTING_CAMERA
            self._state_start_time = time.time()
            self._total_rotated = 0.0
            self._send_pan_tilt(0.0, self._camera_tilt)
            self.node.get_logger().info('[SCAN] Drove forward, starting new scan...')

        return SubtaskStatus.RUNNING

    # ── Detection ─────────────────────────────────────────────────────

    def _detect_object(self):
        """Check current camera frame for target object.

        Returns np.array([x, y, z]) in odom frame, or None.
        """
        rgb = self.node.latest_rgb
        depth = self.node.latest_depth
        if rgb is None or depth is None or self.node.camera_K is None:
            return None

        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # Build mask from all HSV ranges
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in self._hsv_ranges:
            mask |= cv2.inRange(hsv, lower, upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < self._min_area:
            return None

        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        return self._pixel_to_odom(cx, cy, largest)

    def _pixel_to_odom(self, cx, cy, contour=None):
        """Project pixel (cx, cy) + depth to 3D point in odom frame."""
        depth = self.node.latest_depth
        camera_K = self.node.camera_K
        if depth is None or camera_K is None:
            return None

        # Use median depth within contour for robustness
        if contour is not None:
            contour_mask = np.zeros(depth.shape[:2], dtype=np.uint8)
            cv2.drawContours(contour_mask, [contour], -1, 255, -1)
            roi_depths = depth[contour_mask > 0].astype(np.float64)
            valid_depths = roi_depths[(roi_depths > 0) & (roi_depths < 5000)]
            if len(valid_depths) > 0:
                depth_mm = float(np.median(valid_depths))
            else:
                depth_mm = float(depth[cy, cx])
        else:
            depth_mm = float(depth[cy, cx])

        if depth_mm == 0 or depth_mm > 5000:
            return None
        depth_m = depth_mm / 1000.0

        fx = camera_K[0, 0]
        fy = camera_K[1, 1]
        px = camera_K[0, 2]
        py = camera_K[1, 2]

        x_cam = (cx - px) * depth_m / fx
        y_cam = (cy - py) * depth_m / fy
        z_cam = depth_m

        # Transform to odom frame
        try:
            transform = self.node.tf_buffer.lookup_transform(
                'odom', 'camera_color_optical_frame',
                rclpy.time.Time(), timeout=Duration(seconds=0.5))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

        t = transform.transform.translation
        q = transform.transform.rotation
        cam_origin = np.array([t.x, t.y, t.z])

        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
        ])

        point_cam = np.array([x_cam, y_cam, z_cam])
        point_odom = R @ point_cam + cam_origin

        # Sanity check: object should be near ground level in odom (z ~ 0 to 0.5)
        if point_odom[2] < -0.3 or point_odom[2] > 1.0:
            return None

        return point_odom

    # ── Helpers ───────────────────────────────────────────────────────

    def _get_current_yaw(self):
        """Get current yaw from odom->base_link TF."""
        try:
            transform = self.node.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        q = transform.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny, cosy)

    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def _publish_twist(self, linear_x, angular_z):
        """Publish a Twist message to /cmd_vel."""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.node.cmd_vel_pub.publish(twist)

    def _send_pan_tilt(self, pan, tilt):
        """Send camera pan/tilt command."""
        msg = Float64MultiArray()
        msg.data = [float(pan), float(tilt)]
        self.node.pan_tilt_pub.publish(msg)
