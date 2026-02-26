#!/usr/bin/env python3
"""
Navigate to Object Module

Searches for a target object (e.g., 'banana', 'apple') by rotating 360 degrees
in discrete 30-degree steps, checking YOLO detections at each stop. Once the
object is found, centers it in the camera view, then approaches until within
grasp distance.

Uses YOLO detections from the external classifier node (object_classification/classifier.py)
via the /objbbox topic.

Internal sub-states: INIT → SCAN_ROTATE → SCAN_CHECK → CENTER → APPROACH → POSITIONED

Subscribes:
    /objbbox (vision_msgs/Detection2DArray) - YOLO detections from classifier node
    /camera/depth/image_raw (Image) - depth for 3D localization
    /camera/color/camera_info (CameraInfo) - intrinsics

Publishes:
    /cmd_vel (Twist) - base velocity
    /camera/pan_tilt_cmd (Float64MultiArray) - camera pointing

Requires:
    The classifier node must be running: ros2 run object_classification classifier

Usage:
    # As a standalone test:
    ros2 run tidybot_bringup navigate_to_object.py --ros-args -p target_object:=banana

    # Or import in the state machine:
    from navigate_to_object import NavigateToObject
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import numpy as np
import time
from enum import Enum, auto

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

import tf2_ros

from sensor_msgs.msg import Image, CameraInfo, JointState
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy

from tidybot_perception.coord_converter import CoordConverter


# ═══════════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════════

# Camera
CAMERA_TILT_NAV = 0.3          # slight downward tilt for navigation
CAMERA_POSITION_TOL = 0.05     # radians — camera "at target" threshold
CAMERA_SETTLE_TIME = 0.5       # seconds to wait after camera reaches target

# Scanning (discrete 30° steps)
SCAN_STEP_DEG = 30             # degrees per scan stop
SCAN_STEPS = 12                # 360 / 30
SCAN_ROTATE_SPEED = 0.5        # rad/s — moderate rotation speed
SCAN_HEADING_TOL = np.radians(3)   # ~3° — close enough to target heading
SCAN_SETTLE_TIME = 0.5         # seconds to wait at each stop before checking
SCAN_CHECK_DURATION = 1.0      # seconds to check for detections at each stop

# Centering
CENTER_PIXEL_TOL = 20          # pixels from image center — "centered"
CENTER_MAX_ANGULAR = 0.15      # rad/s — slow turning to avoid overshoot
CENTER_GAIN = 0.002            # angular.z = -gain * pixel_error

# Approach
APPROACH_SPEED = 0.15          # m/s — slow forward
APPROACH_HEADING_GAIN = 0.002  # gentle heading correction from pixel error
GRASP_DISTANCE = 0.30          # meters — stop when object this close
OBJECT_LOST_TIMEOUT = 3.0      # seconds before giving up on lost object
DEPTH_EMERGENCY_STOP = 0.25    # meters — hard stop if depth center this close
DEPTH_SLOW_ZONE = 0.50         # meters — slow down when depth center this close

# Dynamic camera tilt during approach
APPROACH_TILT_GAIN = 0.003     # tilt adjustment per pixel of vertical error
APPROACH_TILT_MIN = 0.2        # minimum tilt during approach (slight down)
APPROACH_TILT_MAX = 1.0        # maximum tilt (steep down for close objects)
IMAGE_CENTER_V = 240           # vertical center of 480px image

# YOLO COCO class name → string ID (must match classifier.py output)
YOLO_CLASS_IDS = {'person': '0', 'banana': '46', 'apple': '47', 'orange': '49'}

# Depth back-projection
DEPTH_PATCH_RADIUS = 5         # half-size of patch for robust depth estimation (11x11)
FLOOR_Z_MIN = 0.005            # metres — clamp z above floor


class NavState(Enum):
    """Internal navigation sub-states."""
    INIT         = auto()
    SCAN_ROTATE  = auto()
    SCAN_CHECK   = auto()
    CENTER       = auto()
    APPROACH     = auto()
    POSITIONED   = auto()


# ═══════════════════════════════════════════════════════════════════════
# Helper functions
# ═══════════════════════════════════════════════════════════════════════

def _normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


# ═══════════════════════════════════════════════════════════════════════
# NavigateToObject Node
# ═══════════════════════════════════════════════════════════════════════

class NavigateToObject(Node):
    """
    Navigate to a target object using external YOLO classifier detections.
    Scans in 30-degree increments, centers the object, then approaches.

    Requires the classifier node to be running for /objbbox detections.
    Can be run standalone or used by the state machine.
    """

    def __init__(self, target_object: str = 'banana'):
        super().__init__('navigate_to_object')

        self.target_object = target_object

        # ── TF2 (for odom→base_link lookups) ──
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.coord_conv = CoordConverter(self.tf_buffer)

        # ── Perception: subscribe to external classifier + depth ──
        self._bridge = CvBridge()
        self.latest_depth = None
        self.camera_info = None
        self.latest_detections = None

        qos_be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Detection2DArray, '/objbbox', self._detections_cb, 10)
        self.create_subscription(
            Image, '/camera/depth/image_raw', self._depth_cb, qos_be)
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._info_cb, 10)
        self.joint_states = {}
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)

        # ── Publishers ──
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10)

        # ── State machine ──
        self.nav_state = NavState.INIT
        self.state_start_time = time.time()

        # Scan tracking
        self.scan_start_yaw = None       # yaw when scan began
        self.scan_step_index = 0         # 0..11 (which 30° step)
        self.scan_target_yaw = None      # target heading for current step
        self.scan_check_start = None     # time when SCAN_CHECK began

        # Camera pan/tilt tracking
        self.camera_target_pan = None
        self.camera_target_tilt = None
        self.camera_reached_time = None  # time when camera first reached target

        # Approach/center tracking
        self.target_base_pt = None       # PointStamped in base_link
        self.last_detection_time = 0.0
        self.approach_tilt = CAMERA_TILT_NAV  # current tilt during approach

        # ── Control loop at 10Hz ──
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f'NavigateToObject: searching for "{target_object}"')

    # ═══════════════════════════════════════════════════════════════════
    # Public interface
    # ═══════════════════════════════════════════════════════════════════

    def is_positioned(self) -> bool:
        """Returns True when the robot is positioned within grasping range."""
        return self.nav_state == NavState.POSITIONED

    def get_object_position(self):
        """Returns the latest PointStamped of the target in base_link, or None."""
        return self.target_base_pt

    def set_target(self, target_class: str):
        """Set a new target object class and reset to INIT state."""
        self.target_object = target_class
        self.reset()
        self.get_logger().info(f'New target object: "{target_class}"')

    def reset(self):
        """Reset the full state machine."""
        self.nav_state = NavState.INIT
        self.state_start_time = time.time()
        self.scan_start_yaw = None
        self.scan_step_index = 0
        self.scan_target_yaw = None
        self.scan_check_start = None
        self.target_base_pt = None
        self.last_detection_time = 0.0
        self.camera_target_pan = None
        self.camera_target_tilt = None
        self.camera_reached_time = None
        self.approach_tilt = CAMERA_TILT_NAV
        self._stop_base()

    # ═══════════════════════════════════════════════════════════════════
    # Utility methods
    # ═══════════════════════════════════════════════════════════════════

    def _get_base_pose_in_odom(self):
        """Get (x, y, theta) from odom→base_link TF.

        theta is in MuJoCo convention. Actual heading = theta - pi/2.
        Returns None if TF lookup fails.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
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

    def _get_current_yaw(self):
        """Get current yaw (MuJoCo theta) from TF."""
        pose = self._get_base_pose_in_odom()
        if pose is None:
            return None
        return pose[2]

    def _stop_base(self):
        """Publish zero velocity."""
        self.cmd_vel_pub.publish(Twist())

    def _joint_state_cb(self, msg):
        for i, name in enumerate(msg.name):
            self.joint_states[name] = msg.position[i] if i < len(msg.position) else 0.0

    def _send_pan_tilt(self, pan, tilt):
        """Send camera pan/tilt command and start tracking target."""
        self.camera_target_pan = float(pan)
        self.camera_target_tilt = float(tilt)
        self.camera_reached_time = None
        msg = Float64MultiArray()
        msg.data = [self.camera_target_pan, self.camera_target_tilt]
        self.pan_tilt_pub.publish(msg)

    def _camera_settled(self):
        """Check if camera has reached target and settled.

        Keeps re-publishing the pan/tilt command until the joint states
        confirm arrival, then waits CAMERA_SETTLE_TIME.

        Returns True when the camera is at target AND has settled.
        """
        if self.camera_target_pan is None:
            return True  # no command pending

        # Keep publishing until confirmed
        msg = Float64MultiArray()
        msg.data = [self.camera_target_pan, self.camera_target_tilt]
        self.pan_tilt_pub.publish(msg)

        current_pan = self.joint_states.get('camera_pan', None)
        current_tilt = self.joint_states.get('camera_tilt', None)
        if current_pan is None or current_tilt is None:
            return False

        at_target = (abs(current_pan - self.camera_target_pan) < CAMERA_POSITION_TOL
                     and abs(current_tilt - self.camera_target_tilt) < CAMERA_POSITION_TOL)

        if at_target:
            if self.camera_reached_time is None:
                self.camera_reached_time = time.time()
            if time.time() - self.camera_reached_time >= CAMERA_SETTLE_TIME:
                return True
        else:
            self.camera_reached_time = None

        return False

    # ── Classifier subscription callbacks ──

    def _detections_cb(self, msg):
        self.latest_detections = msg

    def _depth_cb(self, msg):
        self.latest_depth = self._bridge.imgmsg_to_cv2(msg, '16UC1')

    def _info_cb(self, msg):
        self.camera_info = msg

    # ── Detection helpers (consume external classifier) ──

    def _detect(self, target_class):
        """Find the best detection matching target_class from /objbbox.

        Args:
            target_class: COCO class name (e.g. 'banana'). Case-insensitive.

        Returns:
            (u, v) pixel centroid of highest-confidence match, or None.
        """
        if self.latest_detections is None:
            return None
        target_id = YOLO_CLASS_IDS.get(target_class.lower())
        if target_id is None:
            return None

        best, best_conf = None, 0.0
        for det in self.latest_detections.detections:
            for hyp in det.results:
                if hyp.hypothesis.class_id == target_id and hyp.hypothesis.score > best_conf:
                    best = (int(det.bbox.center.position.x),
                            int(det.bbox.center.position.y))
                    best_conf = hyp.hypothesis.score
        return best

    def _detect_and_localize(self, target_class):
        """Detect an object and return its 3D position in base_link.

        Args:
            target_class: COCO class name (e.g. 'banana').

        Returns:
            ((u, v), PointStamped_in_base_link), or None.
        """
        centroid = self._detect(target_class)
        if centroid is None:
            return None

        base_pt = self.coord_conv.pixel_to_base_link(
            centroid[0], centroid[1],
            self.latest_depth, self.camera_info, self.get_clock(),
            patch_radius=DEPTH_PATCH_RADIUS, floor_z_min=FLOOR_Z_MIN)
        if base_pt is None:
            return None

        return (centroid, base_pt)

    def _transition_to(self, new_state):
        """Transition to a new state with logging."""
        old = self.nav_state.name
        self.nav_state = new_state
        self.state_start_time = time.time()
        self.get_logger().info(f'[NAV] {old} → {new_state.name}')

    # ═══════════════════════════════════════════════════════════════════
    # State handlers
    # ═══════════════════════════════════════════════════════════════════

    def _handle_init(self):
        """Wait for sensor data, TF, and camera tilt to be ready."""
        if (self.latest_detections is None
                or self.latest_depth is None
                or self.camera_info is None):
            return

        try:
            self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return

        # Set camera to forward with slight downward tilt (closed-loop)
        if self.camera_target_pan is None:
            self._send_pan_tilt(0.0, CAMERA_TILT_NAV)
            return

        # Wait for camera to reach target and settle
        if not self._camera_settled():
            return

        # Record starting yaw for computing 30° increments
        yaw = self._get_current_yaw()
        if yaw is None:
            return
        self.scan_start_yaw = yaw
        self.scan_step_index = 0
        # First target = current heading (check current heading first)
        self.scan_target_yaw = yaw
        self.scan_check_start = None

        # Check current heading first before rotating
        self._transition_to(NavState.SCAN_CHECK)

    def _handle_scan_rotate(self):
        """Rotate toward the next 30° heading mark."""
        if self.scan_target_yaw is None:
            return

        yaw = self._get_current_yaw()
        if yaw is None:
            return

        heading_error = _normalize_angle(self.scan_target_yaw - yaw)

        if abs(heading_error) < SCAN_HEADING_TOL:
            # Reached target heading — stop and check
            self._stop_base()
            self.scan_check_start = None
            self._transition_to(NavState.SCAN_CHECK)
            return

        # Rotate toward target
        twist = Twist()
        twist.angular.z = np.clip(
            SCAN_ROTATE_SPEED * np.sign(heading_error),
            -SCAN_ROTATE_SPEED, SCAN_ROTATE_SPEED)
        self.cmd_vel_pub.publish(twist)

    def _handle_scan_check(self):
        """Stopped at a scan position. Wait for settle, then check YOLO."""
        self._stop_base()

        now = time.time()
        if self.scan_check_start is None:
            self.scan_check_start = now

        elapsed = now - self.scan_check_start

        # Wait for camera to settle
        if elapsed < SCAN_SETTLE_TIME:
            return

        # Check for target detection
        detection = self._detect(self.target_object)
        if detection is not None:
            u, v = detection
            self.get_logger().info(
                f'[SCAN] Detected {self.target_object} at pixel ({u}, {v}) '
                f'on step {self.scan_step_index}/{SCAN_STEPS}')
            self._transition_to(NavState.CENTER)
            return

        # Still checking — wait for check duration
        if elapsed < SCAN_SETTLE_TIME + SCAN_CHECK_DURATION:
            return

        # No detection at this stop — advance to next step
        self.scan_step_index += 1

        if self.scan_step_index >= SCAN_STEPS:
            self.get_logger().warn(
                f'[SCAN] Full 360° scan complete — {self.target_object} not found')
            # Stay in SCAN_CHECK so task1_retrieve can handle failure
            return

        # Compute next target heading
        step_rad = np.radians(SCAN_STEP_DEG)
        self.scan_target_yaw = _normalize_angle(
            self.scan_start_yaw + self.scan_step_index * step_rad)
        self.scan_check_start = None

        self.get_logger().info(
            f'[SCAN] Step {self.scan_step_index}/{SCAN_STEPS} — '
            f'rotating to {np.degrees(_normalize_angle(self.scan_target_yaw)):.0f}°')
        self._transition_to(NavState.SCAN_ROTATE)

    def _handle_center(self):
        """Slowly rotate to center the detected object in the camera view."""
        pixel = self._detect(self.target_object)
        if pixel is None:
            # Lost object while centering — go back to scanning
            if time.time() - self.state_start_time > OBJECT_LOST_TIMEOUT:
                self.get_logger().warn('[CENTER] Lost object, returning to scan')
                self._stop_base()
                self.scan_check_start = None
                self._transition_to(NavState.SCAN_CHECK)
            return

        u, v = pixel
        image_center_u = 320  # 640 / 2
        pixel_error = u - image_center_u

        if abs(pixel_error) < CENTER_PIXEL_TOL:
            # Centered — transition to approach
            self._stop_base()
            self.last_detection_time = time.time()
            self.get_logger().info(
                f'[CENTER] Object centered (pixel_err={pixel_error})')
            self._transition_to(NavState.APPROACH)
            return

        # Slow angular correction
        angular_z = -CENTER_GAIN * pixel_error
        angular_z = np.clip(angular_z, -CENTER_MAX_ANGULAR, CENTER_MAX_ANGULAR)

        twist = Twist()
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

    def _get_center_depth_m(self):
        """Get the median depth (metres) in a small center patch of the depth image.

        Returns float distance in metres, or None if unavailable.
        """
        if self.latest_depth is None:
            return None
        h, w = self.latest_depth.shape
        cy, cx = h // 2, w // 2
        r = 15  # 31x31 patch
        patch = self.latest_depth[max(0, cy - r):cy + r + 1,
                                  max(0, cx - r):cx + r + 1]
        valid = patch[(patch > 100) & (patch < 10000)]  # 0.1m – 10m
        if len(valid) == 0:
            return None
        return float(np.median(valid)) / 1000.0

    def _handle_approach(self):
        """Drive forward slowly, re-detecting each tick for distance tracking.

        Dynamically tilts the camera down to keep the object in view as the
        robot gets closer. Stops completely when within GRASP_DISTANCE.
        """
        # ── Depth safety check (independent of YOLO) ──
        center_depth = self._get_center_depth_m()
        if center_depth is not None and center_depth < DEPTH_EMERGENCY_STOP:
            self._stop_base()
            self.get_logger().info(
                f'[APPROACH] Depth emergency stop! center_depth={center_depth:.2f}m')
            if self.target_base_pt is not None:
                self._transition_to(NavState.POSITIONED)
            return

        # ── Re-detect for live tracking ──
        detection = self._detect_and_localize(self.target_object)
        if detection is not None:
            pixel, base_pt = detection
            self.target_base_pt = base_pt
            self.last_detection_time = time.time()

            u, v = pixel

            # Object position in base_link: +X=left, -Y=forward
            distance = np.sqrt(base_pt.point.x**2 + base_pt.point.y**2)

            self.get_logger().info(
                f'[APPROACH] dist={distance:.2f}m  pixel=({u},{v})  '
                f'tilt={self.approach_tilt:.2f}rad  depth={center_depth}',
                throttle_duration_sec=1.0)

            # Close enough — STOP completely (don't adjust camera)
            if distance < GRASP_DISTANCE:
                self._stop_base()
                self.get_logger().info(
                    f'[APPROACH] Positioned! dist={distance:.3f}m — stopping.')
                self._transition_to(NavState.POSITIONED)
                return

            # ── Dynamic camera tilt: keep object centered vertically ──
            # Only adjust while still driving (not when close enough to stop)
            v_error = v - IMAGE_CENTER_V  # positive = object below center
            tilt_adjust = APPROACH_TILT_GAIN * v_error
            self.approach_tilt = np.clip(
                self.approach_tilt + tilt_adjust,
                APPROACH_TILT_MIN, APPROACH_TILT_MAX)
            self._send_pan_tilt(0.0, self.approach_tilt)

            # Drive forward with gentle heading correction
            image_center_u = 320
            pixel_error = u - image_center_u
            angular_z = -APPROACH_HEADING_GAIN * pixel_error

            # Scale speed based on distance — slower as we get closer
            speed = min(APPROACH_SPEED, (distance - GRASP_DISTANCE) * 0.5)

            # Also scale down if raw depth shows something close
            if center_depth is not None and center_depth < DEPTH_SLOW_ZONE:
                depth_speed = (center_depth - DEPTH_EMERGENCY_STOP) * 0.5
                speed = min(speed, max(0.02, depth_speed))

            speed = max(0.02, speed)  # minimal creep

            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = np.clip(angular_z, -CENTER_MAX_ANGULAR, CENTER_MAX_ANGULAR)
            self.cmd_vel_pub.publish(twist)
        else:
            # No detection — stop immediately, don't coast blind
            self._stop_base()

            # Tilt camera down more to try to re-acquire the object
            if self.approach_tilt < APPROACH_TILT_MAX:
                self.approach_tilt = min(
                    self.approach_tilt + 0.05, APPROACH_TILT_MAX)
                self._send_pan_tilt(0.0, self.approach_tilt)

            if time.time() - self.last_detection_time > OBJECT_LOST_TIMEOUT:
                self.get_logger().warn('[APPROACH] Lost object, returning to scan')
                self.approach_tilt = CAMERA_TILT_NAV  # reset tilt
                yaw = self._get_current_yaw()
                if yaw is not None:
                    self.scan_start_yaw = yaw
                self.scan_step_index = 0
                self.scan_target_yaw = self.scan_start_yaw
                self.scan_check_start = None
                self._transition_to(NavState.SCAN_CHECK)

    def _handle_positioned(self):
        """Stay stopped — we're in position."""
        self.cmd_vel_pub.publish(Twist())

    # ═══════════════════════════════════════════════════════════════════
    # Control loop
    # ═══════════════════════════════════════════════════════════════════

    def control_loop(self):
        """Navigation state machine at 10Hz."""
        if not self.target_object:
            return

        if self.nav_state == NavState.INIT:
            self._handle_init()
        elif self.nav_state == NavState.SCAN_ROTATE:
            self._handle_scan_rotate()
        elif self.nav_state == NavState.SCAN_CHECK:
            self._handle_scan_check()
        elif self.nav_state == NavState.CENTER:
            self._handle_center()
        elif self.nav_state == NavState.APPROACH:
            self._handle_approach()
        elif self.nav_state == NavState.POSITIONED:
            self._handle_positioned()


# ═══════════════════════════════════════════════════════════════════════
# Standalone entry point
# ═══════════════════════════════════════════════════════════════════════

def main(args=None):
    """Standalone test: search for an object by class name."""
    rclpy.init(args=args)

    node = NavigateToObject(target_object='banana')
    node.declare_parameter('target_object', 'banana')
    target = node.get_parameter('target_object').value
    if target:
        node.set_target(target)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
