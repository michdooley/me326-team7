#!/usr/bin/env python3
"""
TidyBot2 Find Bin & Drop Object

Assumes the robot is already holding an object in the gripper.
Scans for the correct AprilTag-labeled bin, approaches it,
positions the gripper above the opening, and releases the object.

States:
  1. SCANNING     — 360 rotation scan looking for correct AprilTag
  2. CENTERING    — rotate base + tilt camera to center tag in frame
  3. APPROACHING  — drive toward bin with visual servoing on tag
  4. POSITIONING  — use depth to estimate bin pos, move arm above opening
  5. DROPPING     — release gripper, return arm home
  6. COMPLETE     — done

Build:
  cd ros2_ws && colcon build --packages-select tidybot_perception tidybot_bringup && source install/setup.bash

Terminal 1: robot/rviz bringup
  ros2 launch tidybot_bringup sim.launch.py scene:=scene_bins.xml

Terminal 2: apriltag detector
  ros2 run tidybot_perception apriltag_detector_node

Terminal 3: find bin and drop
  ros2 run tidybot_bringup find_bin_and_drop.py --ros-args -p target_fruit:=banana

Or use the launch file:
  ros2 launch tidybot_bringup drop.launch.py target_fruit:=banana
"""

import time
from enum import Enum

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose, Quaternion, Twist
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray, String
from vision_msgs.msg import Detection2DArray
from tidybot_msgs.msg import ArmCommand
from tidybot_msgs.srv import PlanToTarget
from scipy.spatial.transform import Rotation

import tf2_ros

from tidybot_perception.grasp_geometry import yaw_to_grasp_quaternion

try:
    from cv_bridge import CvBridge
except ImportError:
    print("ERROR: cv_bridge is required.")
    raise


class DropState(Enum):
    SCANNING    = 0
    CENTERING   = 1
    APPROACHING = 2
    POSITIONING = 3
    DROPPING    = 4
    COMPLETE    = 5


class FindBinAndDrop(Node):
    """Find the correct AprilTag bin and drop the held object into it."""

    # Tag ID -> fruit name mapping
    TAG_MAP = {"banana": "0", "apple": "1", "orange": "2"}

    # Bin dimensions
    BIN_HEIGHT = 0.3048  # 12 inches in meters
    BIN_HALF_HEIGHT = 0.1524

    # ── Depth filtering ───────────────────────────────────────────────────
    MIN_DEPTH_M = 0.20
    MAX_DEPTH_M = 5.00

    # ── 360 scan ──────────────────────────────────────────────────────────
    SCAN_ANGULAR_VEL  = -0.3   # rad/s (counterclockwise)
    SCAN_SETTLE_TIME  = 0.5    # s
    MAX_SCAN_ATTEMPTS = 3

    # ── Centering ─────────────────────────────────────────────────────────
    CENTER_PIXEL_THRESH   = 40   # px — horizontal
    CENTER_PIXEL_THRESH_V = 40   # px — vertical
    CENTER_ANGULAR_VEL    = 0.25 # rad/s max
    CENTER_TILT_STEP      = 0.02 # rad per tick
    CENTER_TIMEOUT        = 15.0 # s

    # ── Approach ──────────────────────────────────────────────────────────
    APPROACH_LINEAR_SPEED = 0.10  # m/s
    APPROACH_ANGULAR_GAIN = 0.003 # rad/s per pixel offset
    APPROACH_MAX_ANGULAR  = 0.3   # rad/s
    APPROACH_STOP_DIST    = 0.35  # m — stop distance from bin front
    APPROACH_TIMEOUT      = 60.0  # s
    APPROACH_TILT_STEP    = 0.03  # rad

    # ── Positioning ───────────────────────────────────────────────────────
    DROP_HEIGHT_ABOVE_BIN = 0.10  # m above the bin top edge
    ARM_NAME              = 'right'
    MOVE_DURATION         = 2.5   # s for arm motions
    MAX_CONDITION_NUMBER  = 500.0

    # ── Sweet spot for arm reach ──────────────────────────────────────────
    SWEET_SPOT_X      = 0.04   # m — ideal bin X in base_link
    SWEET_SPOT_Y      = -0.26  # m — ideal bin Y in base_link
    SWEET_SPOT_RADIUS = 0.15   # m — tolerance
    NUDGE_SPEED       = 0.04   # m/s

    # ─────────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('find_bin_and_drop')

        # Parameters
        self.declare_parameter('target_fruit', 'banana')
        self.declare_parameter('arm_name', 'right')
        self.target_fruit = (
            self.get_parameter('target_fruit')
            .get_parameter_value().string_value.lower())
        self.ARM_NAME = (
            self.get_parameter('arm_name')
            .get_parameter_value().string_value.lower())

        # Resolve target tag ID
        self.target_tag_id = self.TAG_MAP.get(self.target_fruit)
        if self.target_tag_id is None:
            self.get_logger().error(
                f'Unknown fruit "{self.target_fruit}". '
                f'Known: {list(self.TAG_MAP.keys())}')
            raise ValueError(f'Unknown fruit: {self.target_fruit}')

        # Camera state
        self.latest_depth       = None
        self.latest_depth_stamp = None
        self.latest_rgb         = None
        self.camera_K           = None
        self.cv_bridge          = CvBridge()

        # State machine
        self.state              = DropState.SCANNING
        self.scan_accumulated   = 0.0
        self.scan_last_heading  = None
        self.scan_settle_start  = None
        self.scan_attempt_count = 0
        self.current_tilt       = 0.3

        # Detection state
        self.latest_tag_detection = None  # Detection2D for target tag
        self.bin_world_pos        = None  # (x, y) in odom frame
        self.last_detection_pos   = None

        # Approach state
        self.approach_start_time = None

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Image, '/camera/depth/image_raw',
            self._depth_cb, sensor_qos)
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self._camera_info_cb, sensor_qos)
        self.create_subscription(
            Image, '/camera/rgb/image_raw',
            self._rgb_cb, sensor_qos)

        # AprilTag detections from detector node
        tag_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Detection2DArray, '/apriltag_detections',
            self._apriltag_cb, tag_qos)

        # Safe-stop listener
        self._safe_stop_requested = False
        self.create_subscription(
            String, '/safe_stop', self._safe_stop_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

        # Publishers
        self.cmd_vel_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10)
        self.gripper_pub  = self.create_publisher(
            Float64MultiArray, f'/{self.ARM_NAME}_gripper/cmd', 10)
        self.arm_pub      = self.create_publisher(
            ArmCommand, f'/{self.ARM_NAME}_arm/cmd', 10)

        # Motion planner service
        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')

        self.get_logger().info('=' * 55)
        self.get_logger().info(
            f'Find Bin & Drop — target: {self.target_fruit} '
            f'(tag {self.target_tag_id}), arm: {self.ARM_NAME}')
        self.get_logger().info('=' * 55)

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _depth_cb(self, msg: Image):
        try:
            self.latest_depth = self.cv_bridge.imgmsg_to_cv2(msg, '16UC1')
            self.latest_depth_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')

    def _rgb_cb(self, msg: Image):
        try:
            self.latest_rgb = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def _camera_info_cb(self, msg: CameraInfo):
        self.camera_K = np.array(msg.k).reshape(3, 3)

    def _apriltag_cb(self, msg: Detection2DArray):
        """Filter detections for the target tag ID."""
        for detection in msg.detections:
            if not detection.results:
                continue
            hyp = detection.results[0].hypothesis
            if hyp.class_id == self.target_tag_id:
                self.latest_tag_detection = detection
                return

    def _safe_stop_cb(self, msg):
        self.get_logger().warn('[SAFE STOP] Received safe_stop signal!')
        self._safe_stop_requested = True

    # ── Robot pose helpers ────────────────────────────────────────────────

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
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    # ── Camera tilt helper ────────────────────────────────────────────────

    def _set_camera_tilt(self, tilt):
        self.current_tilt = float(np.clip(tilt, 0.0, 0.9))
        pt_msg = Float64MultiArray()
        pt_msg.data = [0.0, self.current_tilt]
        for _ in range(5):
            self.pan_tilt_pub.publish(pt_msg)
            rclpy.spin_once(self, timeout_sec=0.02)

    # ── Base control ──────────────────────────────────────────────────────

    def _stop_base(self):
        self.cmd_vel_pub.publish(Twist())

    # ── Depth-based position estimation ───────────────────────────────────

    def _estimate_bin_position(self, u, v):
        """Estimate bin world position from tag center pixel + depth."""
        if self.latest_depth is None or self.camera_K is None:
            return None

        depth = self.latest_depth
        h, w = depth.shape

        half_win = 4
        v_lo = max(0, v - half_win)
        v_hi = min(h, v + half_win + 1)
        u_lo = max(0, u - half_win)
        u_hi = min(w, u + half_win + 1)

        patch = depth[v_lo:v_hi, u_lo:u_hi].astype(np.float64)
        valid = patch[patch > 0]
        if len(valid) == 0:
            return None
        depth_m = float(np.median(valid)) / 1000.0

        if depth_m < self.MIN_DEPTH_M or depth_m > self.MAX_DEPTH_M:
            return None

        fx = self.camera_K[0, 0]
        fy = self.camera_K[1, 1]
        cx_k = self.camera_K[0, 2]
        cy_k = self.camera_K[1, 2]

        x_cam = (u - cx_k) * depth_m / fx
        y_cam = (v - cy_k) * depth_m / fy
        z_cam = depth_m
        pt_cam = np.array([x_cam, y_cam, z_cam])

        try:
            tf_ob = self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return None

        try:
            tf_bc = self.tf_buffer.lookup_transform(
                'base_link', 'camera_color_optical_frame',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return None

        def _q2m(q):
            qw, qx, qy, qz = q.w, q.x, q.y, q.z
            return np.array([
                [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)],
            ])

        R_ob = _q2m(tf_ob.transform.rotation)
        R_bc = _q2m(tf_bc.transform.rotation)
        to = tf_ob.transform.translation
        tc = tf_bc.transform.translation

        cam_origin = R_ob @ np.array([tc.x, tc.y, tc.z]) + \
                     np.array([to.x, to.y, to.z])
        R = R_ob @ R_bc

        pt_world = R @ pt_cam + cam_origin
        return (float(pt_world[0]), float(pt_world[1]))

    def _get_bin_position_in_base_link(self, u, v):
        """Get the bin tag center in base_link frame for arm positioning."""
        if self.latest_depth is None or self.camera_K is None:
            return None

        depth = self.latest_depth
        h, w = depth.shape

        half_win = 4
        v_lo = max(0, v - half_win)
        v_hi = min(h, v + half_win + 1)
        u_lo = max(0, u - half_win)
        u_hi = min(w, u + half_win + 1)

        patch = depth[v_lo:v_hi, u_lo:u_hi].astype(np.float64)
        valid = patch[patch > 0]
        if len(valid) == 0:
            return None
        depth_m = float(np.median(valid)) / 1000.0

        if depth_m < self.MIN_DEPTH_M or depth_m > self.MAX_DEPTH_M:
            return None

        fx = self.camera_K[0, 0]
        fy = self.camera_K[1, 1]
        cx_k = self.camera_K[0, 2]
        cy_k = self.camera_K[1, 2]

        x_cam = (u - cx_k) * depth_m / fx
        y_cam = (v - cy_k) * depth_m / fy
        z_cam = depth_m
        pt_cam = np.array([x_cam, y_cam, z_cam])

        try:
            tf_bc = self.tf_buffer.lookup_transform(
                'base_link', 'camera_color_optical_frame',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return None

        def _q2m(q):
            qw, qx, qy, qz = q.w, q.x, q.y, q.z
            return np.array([
                [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)],
            ])

        R_bc = _q2m(tf_bc.transform.rotation)
        tc = tf_bc.transform.translation
        t_bc = np.array([tc.x, tc.y, tc.z])

        pt_base = R_bc @ pt_cam + t_bc
        return pt_base  # numpy array [x, y, z] in base_link

    # ── Arm control helpers ───────────────────────────────────────────────

    def _spin_for(self, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _set_gripper(self, position):
        msg = Float64MultiArray()
        msg.data = [position]
        for _ in range(10):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(0.5)

    def _go_home(self, duration=3.0):
        self.get_logger().info(
            f'[ARM] Returning arm home over {duration}s')
        msg = ArmCommand()
        msg.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.duration = duration
        self.arm_pub.publish(msg)
        time.sleep(duration + 0.5)

    def _plan_and_execute(self, pose, label, use_orientation=True):
        """Call motion planner to move arm to a target pose."""
        req = PlanToTarget.Request()
        req.arm_name = self.ARM_NAME
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = True
        req.duration = self.MOVE_DURATION
        req.max_condition_number = self.MAX_CONDITION_NUMBER

        self.get_logger().info(f'  [{label}] Planning + executing...')

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        if not future.done() or future.exception():
            self.get_logger().error(f'  [{label}] Service call timed out')
            return False

        result = future.result()
        if result.success:
            if result.executed:
                # Keep gripper closed while arm moves
                grip_msg = Float64MultiArray()
                grip_msg.data = [1.0]
                deadline = time.time() + self.MOVE_DURATION + 0.5
                while time.time() < deadline:
                    self.gripper_pub.publish(grip_msg)
                    rclpy.spin_once(self, timeout_sec=0.1)
            return True

        self.get_logger().warn(f'  [{label}] Failed: {result.message}')
        return False

    def _try_ik(self, pose, use_orientation=True):
        """Test IK feasibility without executing."""
        req = PlanToTarget.Request()
        req.arm_name = self.ARM_NAME
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = False
        req.duration = self.MOVE_DURATION
        req.max_condition_number = self.MAX_CONDITION_NUMBER

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done() or future.exception():
            return False, 'service call timed out'

        result = future.result()
        return result.success, result.message

    # ── Safe shutdown ─────────────────────────────────────────────────────

    def _execute_safe_shutdown(self):
        self.get_logger().warn('[SAFE STOP] Executing safe shutdown...')

        # Stop the base
        for _ in range(10):
            self.cmd_vel_pub.publish(Twist())
            time.sleep(0.05)

        # Open gripper
        self._set_gripper(0.0)

        # Return arm to home
        self._go_home(duration=3.0)

        # Reset camera tilt
        self._set_camera_tilt(0.3)

        self.get_logger().warn('[SAFE STOP] Shutdown complete.')

    # ── State: SCANNING ───────────────────────────────────────────────────

    def _start_scan(self):
        self.scan_accumulated  = 0.0
        self.scan_last_heading = None
        self.scan_settle_start = None
        self.scan_attempt_count += 1
        self._set_camera_tilt(0.3)
        self.state = DropState.SCANNING
        self.get_logger().info(
            f'[SCAN] Starting 360 rotation scan '
            f'(attempt {self.scan_attempt_count}/{self.MAX_SCAN_ATTEMPTS})...')

    def _state_scanning(self):
        # Check if tag was detected mid-scan
        det = self.latest_tag_detection
        if det is not None:
            self.latest_tag_detection = None
            u = int(det.bbox.center.position.x)
            v = int(det.bbox.center.position.y)
            self.get_logger().info(
                f'[SCAN] Tag {self.target_tag_id} detected at pixel ({u}, {v})')

            # Try to get world position
            pos = self._estimate_bin_position(u, v)
            if pos is not None:
                self.bin_world_pos = pos
                self.last_detection_pos = pos

            self._stop_base()
            self._start_centering()
            return

        # Settling after full rotation
        if self.scan_settle_start is not None:
            if time.time() - self.scan_settle_start >= self.SCAN_SETTLE_TIME:
                if self.scan_attempt_count >= self.MAX_SCAN_ATTEMPTS:
                    self.get_logger().warn(
                        f'[SCAN] No tag found after '
                        f'{self.MAX_SCAN_ATTEMPTS} scans — giving up.')
                    self.state = DropState.COMPLETE
                else:
                    self.get_logger().info(
                        '[SCAN] No tag found — re-scanning...')
                    self._start_scan()
            return

        # Accumulate rotation
        heading = self.get_heading()
        if heading is None:
            return

        if self.scan_last_heading is not None:
            delta = (heading - self.scan_last_heading + np.pi) % (2*np.pi) - np.pi
            self.scan_accumulated += abs(delta)

        self.scan_last_heading = heading

        if self.scan_accumulated >= 2 * np.pi:
            self._stop_base()
            self.scan_settle_start = time.time()
            self.get_logger().info(
                f'[SCAN] 360 done '
                f'({np.degrees(self.scan_accumulated):.0f} accumulated). '
                f'Settling {self.SCAN_SETTLE_TIME}s...')
        else:
            cmd = Twist()
            cmd.angular.z = self.SCAN_ANGULAR_VEL
            self.cmd_vel_pub.publish(cmd)

    # ── State: CENTERING ──────────────────────────────────────────────────

    def _start_centering(self):
        self.center_start_time = time.time()
        self.state = DropState.CENTERING
        self.get_logger().info('[CENTER] Centering tag in camera frame...')

    def _state_centering(self):
        now = time.time()

        if now - self.center_start_time > self.CENTER_TIMEOUT:
            self._stop_base()
            if self.bin_world_pos is not None:
                self.get_logger().warn(
                    '[CENTER] Timeout — bin known, approaching anyway.')
                self.approach_start_time = time.time()
                self.state = DropState.APPROACHING
            else:
                self.get_logger().warn('[CENTER] Timeout — re-scanning.')
                self._start_scan()
            return

        pose = self.get_base_pose()
        if pose is None:
            return
        bx, by, btheta = pose

        det = self.latest_tag_detection
        if det is not None:
            # Tag visible: fine-center using pixel offset
            self.latest_tag_detection = None
            u = int(det.bbox.center.position.x)
            v = int(det.bbox.center.position.y)

            if self.latest_depth is None:
                return
            img_h, img_w = self.latest_depth.shape
            img_center_x = img_w / 2.0
            img_center_y = img_h / 2.0
            offset_x = u - img_center_x
            offset_y = v - img_center_y

            # Update bin position from fresh detection
            new_pos = self._estimate_bin_position(u, v)
            if new_pos is not None:
                if self.bin_world_pos is not None:
                    shift = np.hypot(
                        new_pos[0] - self.bin_world_pos[0],
                        new_pos[1] - self.bin_world_pos[1])
                    if shift < 1.5:
                        self.bin_world_pos = new_pos
                        self.last_detection_pos = new_pos
                else:
                    self.bin_world_pos = new_pos
                    self.last_detection_pos = new_pos

            # Adjust camera tilt vertically
            if abs(offset_y) > self.CENTER_PIXEL_THRESH_V:
                tilt_adj = offset_y / (img_h / 2.0) * self.CENTER_TILT_STEP
                self._set_camera_tilt(self.current_tilt + tilt_adj)

            h_centered = abs(offset_x) < self.CENTER_PIXEL_THRESH
            v_centered = abs(offset_y) < self.CENTER_PIXEL_THRESH_V

            if h_centered and v_centered:
                self._stop_base()
                self.get_logger().info(
                    f'[CENTER] Tag centered (h={offset_x:.0f}px, '
                    f'v={offset_y:.0f}px) — approaching.')
                self.approach_start_time = time.time()
                self.state = DropState.APPROACHING
                return

            # Rotate to reduce horizontal offset
            if not h_centered:
                cmd = Twist()
                frac = min(abs(offset_x) / (img_w / 2.0), 1.0)
                speed = self.CENTER_ANGULAR_VEL * (0.2 + 0.8 * frac * frac)
                cmd.angular.z = -speed if offset_x > 0 else speed
                self.cmd_vel_pub.publish(cmd)

            if not hasattr(self, '_center_last_log') or now - self._center_last_log > 1.0:
                self._center_last_log = now
                self.get_logger().info(
                    f'[CENTER] h_offset={offset_x:.0f}px, '
                    f'v_offset={offset_y:.0f}px, tilt={self.current_tilt:.2f}')
        else:
            # No tag detection: rotate toward known world position
            if self.bin_world_pos is None:
                elapsed = now - self.center_start_time
                if elapsed < 3.0:
                    return
                self._stop_base()
                self.get_logger().warn('[CENTER] Lost tag — re-scanning.')
                self._start_scan()
                return

            ox, oy = self.bin_world_pos
            bearing = np.arctan2(oy - by, ox - bx)
            actual_heading = btheta - np.pi / 2
            heading_error = self._normalize_angle(bearing - actual_heading)

            if abs(heading_error) < np.radians(5):
                self._stop_base()
                return

            cmd = Twist()
            cmd.angular.z = float(np.clip(
                1.0 * heading_error,
                -self.CENTER_ANGULAR_VEL, self.CENTER_ANGULAR_VEL))
            self.cmd_vel_pub.publish(cmd)

    # ── State: APPROACHING ────────────────────────────────────────────────

    def _state_approaching(self):
        now = time.time()

        if now - self.approach_start_time > self.APPROACH_TIMEOUT:
            self._stop_base()
            self.get_logger().warn('[APPROACH] Timeout — moving to positioning.')
            self.state = DropState.POSITIONING
            return

        pose = self.get_base_pose()
        if pose is None:
            return
        bx, by, btheta = pose
        actual_heading = btheta - np.pi / 2

        # Check distance to bin via world position
        if self.bin_world_pos is not None:
            ox, oy = self.bin_world_pos
            dist_to_bin = np.hypot(ox - bx, oy - by)
            if dist_to_bin < self.APPROACH_STOP_DIST:
                self._stop_base()
                self.get_logger().info(
                    f'[APPROACH] Reached bin! dist={dist_to_bin:.2f}m')
                self.state = DropState.POSITIONING
                return

        det = self.latest_tag_detection
        cmd = Twist()

        if det is not None:
            self.latest_tag_detection = None
            u = int(det.bbox.center.position.x)
            v = int(det.bbox.center.position.y)

            if self.latest_depth is not None:
                img_h, img_w = self.latest_depth.shape
                img_center_x = img_w / 2.0
                img_center_y = img_h / 2.0
                offset_x = u - img_center_x

                # Steer proportionally
                angular = -self.APPROACH_ANGULAR_GAIN * offset_x
                cmd.angular.z = float(np.clip(
                    angular,
                    -self.APPROACH_MAX_ANGULAR, self.APPROACH_MAX_ANGULAR))

                # Adjust tilt
                offset_y = v - img_center_y
                if abs(offset_y) > self.CENTER_PIXEL_THRESH_V:
                    tilt_adj = offset_y / (img_h / 2.0) * self.APPROACH_TILT_STEP
                    self._set_camera_tilt(self.current_tilt + tilt_adj)

            # Refine bin position
            new_pos = self._estimate_bin_position(u, v)
            if new_pos is not None and self.bin_world_pos is not None:
                shift = np.hypot(
                    new_pos[0] - self.bin_world_pos[0],
                    new_pos[1] - self.bin_world_pos[1])
                if shift < 1.5:
                    self.bin_world_pos = new_pos
                    self.last_detection_pos = new_pos

            # Check depth directly
            if self.latest_depth is not None:
                half_win = 4
                v_lo = max(0, v - half_win)
                v_hi = min(self.latest_depth.shape[0], v + half_win + 1)
                u_lo = max(0, u - half_win)
                u_hi = min(self.latest_depth.shape[1], u + half_win + 1)
                patch = self.latest_depth[v_lo:v_hi, u_lo:u_hi].astype(np.float64)
                valid_depths = patch[patch > 0]
                if len(valid_depths) > 0:
                    depth_m = float(np.median(valid_depths)) / 1000.0
                    if depth_m < self.APPROACH_STOP_DIST:
                        self._stop_base()
                        self.get_logger().info(
                            f'[APPROACH] Bin depth={depth_m:.2f}m < '
                            f'{self.APPROACH_STOP_DIST}m — reached!')
                        self.state = DropState.POSITIONING
                        return
        else:
            # No tag: fall back to world heading
            if self.bin_world_pos is not None:
                ox, oy = self.bin_world_pos
                bearing = np.arctan2(oy - by, ox - bx)
                heading_error = self._normalize_angle(bearing - actual_heading)
                cmd.angular.z = float(np.clip(
                    1.0 * heading_error,
                    -self.APPROACH_MAX_ANGULAR, self.APPROACH_MAX_ANGULAR))

        cmd.linear.x = self.APPROACH_LINEAR_SPEED
        self.cmd_vel_pub.publish(cmd)

        if not hasattr(self, '_approach_last_log') or now - self._approach_last_log > 2.0:
            self._approach_last_log = now
            dist_str = ''
            if self.bin_world_pos is not None:
                ox, oy = self.bin_world_pos
                dist_str = f' dist={np.hypot(ox-bx, oy-by):.2f}m'
            tag_str = 'TAG' if det is not None else 'world-heading'
            self.get_logger().info(
                f'[APPROACH] {tag_str}{dist_str} '
                f'pos=({bx:.2f},{by:.2f}) tilt={self.current_tilt:.2f}')

    # ── State: POSITIONING ────────────────────────────────────────────────

    def _state_positioning(self):
        self._stop_base()
        self.get_logger().info('=' * 55)
        self.get_logger().info(
            f'[POSITION] Positioning arm above {self.target_fruit} bin...')

        if not self.plan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                '[POSITION] /plan_to_target service not available!')
            self.state = DropState.COMPLETE
            return

        # Re-detect tag for fresh position
        self.get_logger().info('[POSITION] Looking for tag to get fresh position...')
        self._set_camera_tilt(0.5)
        self._spin_for(1.0)

        # Poll for tag detection
        bin_base = None
        for poll in range(20):  # 4 seconds of polling
            self._spin_for(0.2)
            det = self.latest_tag_detection
            if det is not None:
                self.latest_tag_detection = None
                u = int(det.bbox.center.position.x)
                v = int(det.bbox.center.position.y)
                bin_base = self._get_bin_position_in_base_link(u, v)
                if bin_base is not None:
                    self.get_logger().info(
                        f'[POSITION] Tag center in base_link: '
                        f'({bin_base[0]:.3f}, {bin_base[1]:.3f}, {bin_base[2]:.3f})')
                    break

        if bin_base is None:
            self.get_logger().warn(
                '[POSITION] Could not localize bin — trying with last known position')
            # Fall back to a blind drop attempt above the last known direction
            if self.bin_world_pos is not None:
                # Use a default position in front of the robot
                bin_base = np.array([0.0, -0.30, 0.15])
            else:
                self.get_logger().error('[POSITION] No bin position available!')
                self.state = DropState.COMPLETE
                return

        # Compute drop target: above the bin opening
        # The tag is on a vertical wall, so the tag center z gives us roughly
        # the middle height of the bin wall. The bin opening is at the top:
        # z_opening = bin_base[2] + BIN_HALF_HEIGHT (from tag center to top)
        # Then we go DROP_HEIGHT_ABOVE_BIN above that.
        drop_z = bin_base[2] + self.BIN_HALF_HEIGHT + self.DROP_HEIGHT_ABOVE_BIN
        drop_x = bin_base[0]
        drop_y = bin_base[1]

        self.get_logger().info(
            f'[POSITION] Drop target in base_link: '
            f'({drop_x:.3f}, {drop_y:.3f}, {drop_z:.3f})')

        # Check if bin is within arm sweet spot
        dist_to_sweet = np.hypot(
            drop_x - self.SWEET_SPOT_X,
            drop_y - self.SWEET_SPOT_Y)

        if dist_to_sweet > self.SWEET_SPOT_RADIUS:
            self.get_logger().info(
                f'[POSITION] Bin outside sweet spot '
                f'(dist={dist_to_sweet:.3f}m > {self.SWEET_SPOT_RADIUS}m). '
                f'Nudging base...')
            self._nudge_toward_sweet_spot(drop_x, drop_y)
            # Re-detect after nudge
            self._spin_for(1.0)
            det = self.latest_tag_detection
            if det is not None:
                self.latest_tag_detection = None
                u = int(det.bbox.center.position.x)
                v = int(det.bbox.center.position.y)
                new_base = self._get_bin_position_in_base_link(u, v)
                if new_base is not None:
                    bin_base = new_base
                    drop_z = bin_base[2] + self.BIN_HALF_HEIGHT + self.DROP_HEIGHT_ABOVE_BIN
                    drop_x = bin_base[0]
                    drop_y = bin_base[1]
                    self.get_logger().info(
                        f'[POSITION] Updated drop target: '
                        f'({drop_x:.3f}, {drop_y:.3f}, {drop_z:.3f})')

        # Build a downward-facing gripper pose
        # yaw=0 gives default downward orientation
        qw, qx, qy, qz = yaw_to_grasp_quaternion(0.0)

        drop_pose = Pose()
        drop_pose.position.x = drop_x
        drop_pose.position.y = drop_y
        drop_pose.position.z = float(drop_z)
        drop_pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

        # Try IK first
        success, msg = self._try_ik(drop_pose)
        if not success:
            self.get_logger().warn(
                f'[POSITION] IK failed for drop pose: {msg}')
            # Try a higher position
            drop_pose.position.z = float(drop_z + 0.05)
            success, msg = self._try_ik(drop_pose)
            if not success:
                self.get_logger().warn(
                    f'[POSITION] IK still failed at higher z: {msg}')
                # Try without orientation constraint
                success, msg = self._try_ik(drop_pose, use_orientation=False)
                if not success:
                    self.get_logger().error(
                        '[POSITION] Cannot reach drop position — dropping where arm is')
                    self.state = DropState.DROPPING
                    return

        # First move to a safe pre-drop position (higher)
        pre_drop_pose = Pose()
        pre_drop_pose.position.x = drop_x
        pre_drop_pose.position.y = drop_y
        pre_drop_pose.position.z = float(drop_z + 0.10)
        pre_drop_pose.orientation = drop_pose.orientation

        self.get_logger().info('[POSITION] Moving to pre-drop position...')
        if not self._plan_and_execute(pre_drop_pose, 'PRE-DROP'):
            self.get_logger().warn(
                '[POSITION] Pre-drop move failed, trying drop directly')

        # Move to drop position
        self.get_logger().info('[POSITION] Moving to drop position...')
        if self._plan_and_execute(drop_pose, 'DROP-POS'):
            self.get_logger().info('[POSITION] Arm in position above bin!')
        else:
            self.get_logger().warn(
                '[POSITION] Drop position move failed — dropping where arm is')

        self._spin_for(1.0)
        self.state = DropState.DROPPING

    def _nudge_toward_sweet_spot(self, obj_x, obj_y):
        """Nudge the base so the bin is closer to the arm sweet spot."""
        error_x = obj_x - self.SWEET_SPOT_X
        error_y = obj_y - self.SWEET_SPOT_Y

        duration = min(abs(error_x) / self.NUDGE_SPEED + 0.5, 3.0)

        cmd = Twist()
        # Forward/backward to fix X
        if abs(error_x) > 0.03:
            cmd.linear.x = self.NUDGE_SPEED if error_x > 0 else -self.NUDGE_SPEED
        # Lateral (rotate) to fix Y
        if abs(error_y) > 0.03:
            cmd.angular.z = -0.15 if error_y < 0 else 0.15

        self.get_logger().info(
            f'[NUDGE] error=({error_x:.3f}, {error_y:.3f}), '
            f'cmd=({cmd.linear.x:.2f}, {cmd.angular.z:.2f}), '
            f'dur={duration:.1f}s')

        deadline = time.time() + duration
        while time.time() < deadline and rclpy.ok():
            self.cmd_vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
        self._stop_base()
        self._spin_for(0.5)

    # ── State: DROPPING ───────────────────────────────────────────────────

    def _state_dropping(self):
        self.get_logger().info('=' * 55)
        self.get_logger().info('[DROP] Releasing object into bin...')

        # Open gripper
        msg = Float64MultiArray()
        msg.data = [0.0]
        for _ in range(20):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(1.5)

        self.get_logger().info('[DROP] Object released!')

        # Return arm to home
        self.get_logger().info('[DROP] Returning arm to home position...')
        self._go_home(duration=3.0)

        # Reset camera
        self._set_camera_tilt(0.3)

        self.get_logger().info('=' * 55)
        self.get_logger().info(
            f'[DROP] {self.target_fruit.upper()} dropped in bin! Task complete.')
        self.get_logger().info('=' * 55)

        self.state = DropState.COMPLETE

    # ── Main loop ─────────────────────────────────────────────────────────

    def run(self):
        self.get_logger().info('Waiting for depth data and TF...')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_depth is not None and self.camera_K is not None:
                try:
                    self.tf_buffer.lookup_transform(
                        'odom', 'camera_color_optical_frame',
                        rclpy.time.Time(), timeout=Duration(seconds=0.1))
                    break
                except Exception:
                    pass

        self.get_logger().info('Sensors ready!')

        # Check if tag is already visible before scanning
        self.get_logger().info(
            '[INIT] Tilting camera and checking for tag...')
        self._set_camera_tilt(0.5)
        self._spin_for(1.0)

        # Poll for tag for 3 seconds
        self.latest_tag_detection = None
        found = False
        deadline = time.time() + 3.0
        while time.time() < deadline:
            self._spin_for(0.2)
            det = self.latest_tag_detection
            if det is not None:
                self.latest_tag_detection = None
                u = int(det.bbox.center.position.x)
                v = int(det.bbox.center.position.y)
                self.get_logger().info(
                    f'[INIT] Tag {self.target_tag_id} detected at pixel ({u}, {v})')
                pos = self._estimate_bin_position(u, v)
                if pos is not None:
                    self.bin_world_pos = pos
                    self.last_detection_pos = pos
                found = True
                break

        if found:
            self.get_logger().info('[INIT] Tag already visible — centering!')
            self._start_centering()
        else:
            self.get_logger().info('[INIT] Tag not visible — starting scan')
            self._set_camera_tilt(0.3)
            self.scan_attempt_count = 0
            self._start_scan()

        # Main state machine loop at 20 Hz
        while rclpy.ok():
            deadline = time.time() + 0.05
            while time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0)
                time.sleep(0.002)

            # Check for safe-stop
            if self._safe_stop_requested:
                self._execute_safe_shutdown()
                return

            # Dispatch to state handler
            if self.state == DropState.SCANNING:
                self._state_scanning()
            elif self.state == DropState.CENTERING:
                self._state_centering()
            elif self.state == DropState.APPROACHING:
                self._state_approaching()
            elif self.state == DropState.POSITIONING:
                self._state_positioning()
            elif self.state == DropState.DROPPING:
                self._state_dropping()
            elif self.state == DropState.COMPLETE:
                self.get_logger().info('Task complete. Exiting.')
                return


def main(args=None):
    rclpy.init(args=args)
    node = FindBinAndDrop()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn('KeyboardInterrupt — executing safe shutdown...')
        try:
            node._execute_safe_shutdown()
        except Exception as e:
            node.get_logger().error(f'Error during safe shutdown: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
