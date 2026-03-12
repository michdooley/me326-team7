#!/usr/bin/env python3
"""
Cube Calibration Test — precision pre-grasp alignment using a 1-inch red cube.

Fine-tuning tool for depth→world coordinate transforms. Uses HSV red detection
(no YOLO), skips RANSAC (cube is too small), closes the gripper at pre-grasp
(seam = precise alignment reference), and positions just 2cm above the cube.

Usage:
  Terminal 1: ros2 launch tidybot_bringup real.launch.py
  Terminal 2: ros2 run tidybot_bringup cube_calibration_test.py

Interactive commands (type at the prompt):
  Enter  — retract arm, re-detect, close gripper, move to pre-grasp
  1      — cycle backprojection mode (depth_native / color_aligned / color_raw)
  2      — toggle pixel mapping (rgb_to_depth / direct)
  3+/3-  — adjust X offset ±5mm
  4+/4-  — adjust Y offset ±5mm
  5+/5-  — adjust Z offset ±5mm
  6+/6-  — adjust pre-grasp height ±5mm
  r      — reset all offsets to defaults
  p      — print full diagnostics (intrinsics, TF, etc.)
  g      — execute full grasp (descend + close + lift)
  q      — return arm to sleep and quit
"""

import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray
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

# Try to import depth alignment utility
try:
    sys.path.insert(0, str(Path(__file__).resolve().parent / 'utilities'))
    from align_depth_to_rgb import align_depth
    ALIGN_AVAILABLE = True
except ImportError:
    ALIGN_AVAILABLE = False
    print("WARNING: align_depth_to_rgb not available — color_aligned mode disabled")


# ── HSV red detection ranges (from RGBDObjectDetector) ───────────────────────

RED_HSV_RANGES = [
    ((0, 100, 80), (10, 255, 255)),
    ((165, 100, 80), (180, 255, 255)),
]

MIN_CONTOUR_AREA = 200  # px² — minimum blob to count as cube


# ── Backprojection modes ─────────────────────────────────────────────────────

MODES = ['depth_native', 'color_aligned', 'color_raw']
PIXEL_MAPS = ['rgb_to_depth', 'direct']


class CubeCalibrationNode(Node):

    # Defaults — start neutral
    DEFAULT_X_OFFSET = 0.0       # m
    DEFAULT_Y_OFFSET = 0.0       # m
    DEFAULT_Z_OFFSET = 0.0       # m
    DEFAULT_PRE_HEIGHT = 0.02    # m (2cm — very close for precision)

    GRASP_ARM_NAME = 'right'
    MOVE_DURATION = 2.5
    BBOX_PAD = 1.5               # pad HSV bbox generously
    MIN_DEPTH_M = 0.1
    MAX_DEPTH_M = 2.0
    MIN_POINTS = 10              # fewer points expected from small cube
    TABLE_Z_MIN = 0.005
    OUTLIER_Z_THRESH = 0.05

    # Sleep pose — arm tucked out of camera view
    SLEEP_JOINTS = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]

    def __init__(self):
        super().__init__('cube_calibration_test')

        # ── Tunable parameters ────────────────────────────────────────────
        self.mode_idx = 0            # index into MODES
        self.pixel_map_idx = 0       # index into PIXEL_MAPS
        self.x_offset = self.DEFAULT_X_OFFSET
        self.y_offset = self.DEFAULT_Y_OFFSET
        self.z_offset = self.DEFAULT_Z_OFFSET
        self.pre_height = self.DEFAULT_PRE_HEIGHT

        # ── State ─────────────────────────────────────────────────────────
        self.latest_depth = None
        self.latest_rgb = None
        self.color_K = None
        self.depth_K = None
        self.cv_bridge = CvBridge()
        self.iteration = 0

        # Camera tilt
        self.current_tilt = 0.3

        # ── TF ────────────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscriptions ─────────────────────────────────────────────────
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(
            Image, '/camera/depth/image_raw', self._depth_cb, sensor_qos)
        self.create_subscription(
            Image, '/camera/rgb/image_raw', self._rgb_cb, sensor_qos)
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self._color_info_cb, sensor_qos)
        self.create_subscription(
            CameraInfo, '/camera/depth/camera_info',
            self._depth_info_cb, sensor_qos)

        # ── Publishers ────────────────────────────────────────────────────
        self.arm_pub = self.create_publisher(
            ArmCommand, f'/{self.GRASP_ARM_NAME}_arm/cmd', 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, f'/{self.GRASP_ARM_NAME}_gripper/cmd', 10)
        self.pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10)

        # ── Service client ────────────────────────────────────────────────
        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')

        self.get_logger().info('=' * 60)
        self.get_logger().info('Cube Calibration Test — precision alignment with red cube')
        self.get_logger().info('  Detection: HSV red | RANSAC: disabled | Gripper: closed')
        self.get_logger().info('=' * 60)

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _depth_cb(self, msg):
        try:
            self.latest_depth = self.cv_bridge.imgmsg_to_cv2(msg, '16UC1')
        except Exception:
            pass

    def _rgb_cb(self, msg):
        try:
            self.latest_rgb = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def _color_info_cb(self, msg):
        self.color_K = np.array(msg.k).reshape(3, 3)

    def _depth_info_cb(self, msg):
        self.depth_K = np.array(msg.k).reshape(3, 3)

    # ── Helpers ───────────────────────────────────────────────────────────

    def _spin_for(self, duration):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _get_tf_matrix(self, target, source):
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time(),
                timeout=Duration(seconds=2.0))
            t = tf.transform.translation
            q = tf.transform.rotation
            rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
            mat = np.eye(4, dtype=np.float32)
            mat[:3, :3] = rot.as_matrix().astype(np.float32)
            mat[:3, 3] = [t.x, t.y, t.z]
            return mat
        except Exception as e:
            self.get_logger().error(f'TF lookup failed ({source} → {target}): {e}')
            return None

    def _set_camera_tilt(self, tilt):
        self.current_tilt = tilt
        msg = Float64MultiArray()
        msg.data = [0.0, tilt]
        self.pan_tilt_pub.publish(msg)

    def _set_gripper(self, value):
        msg = Float64MultiArray()
        msg.data = [value]
        for _ in range(5):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def _close_gripper_firm(self):
        msg = Float64MultiArray()
        msg.data = [1.0]
        for _ in range(20):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(2.0)
        for _ in range(10):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    # ── Arm control ───────────────────────────────────────────────────────

    def _move_arm_joints(self, joints, duration=3.0):
        msg = ArmCommand()
        msg.joint_positions = joints
        msg.duration = duration
        self.arm_pub.publish(msg)
        self._spin_for(duration + 0.5)

    def _retract_arm(self):
        """Move arm to sleep pose (out of camera view)."""
        self.get_logger().info('Retracting arm to sleep pose...')
        self._move_arm_joints(self.SLEEP_JOINTS, duration=3.0)

    def _plan_and_execute(self, pose, use_orientation=True):
        req = PlanToTarget.Request()
        req.arm_name = self.GRASP_ARM_NAME
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = True
        req.duration = self.MOVE_DURATION
        req.max_condition_number = 500.0

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done() or future.exception():
            return False, 'service call timed out'

        result = future.result()
        if result.success:
            self._spin_for(self.MOVE_DURATION + 0.5)
        return result.success, result.message

    def _try_ik(self, pose, use_orientation=True):
        req = PlanToTarget.Request()
        req.arm_name = self.GRASP_ARM_NAME
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = False
        req.duration = self.MOVE_DURATION
        req.max_condition_number = 500.0

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done() or future.exception():
            return False, 'service call timed out'

        result = future.result()
        return result.success, result.message

    # ── Pixel remapping ───────────────────────────────────────────────────

    def _rgb_pixel_to_depth_pixel(self, u_rgb, v_rgb):
        if self.color_K is None or self.depth_K is None:
            return u_rgb, v_rgb

        pt = np.array([u_rgb, v_rgb, 1.0])
        pt_norm = np.linalg.inv(self.color_K) @ pt
        pt_depth = self.depth_K @ pt_norm
        return float(pt_depth[0]), float(pt_depth[1])

    def _remap_bbox(self, cx, cy, bbox_w, bbox_h):
        cx_d, cy_d = self._rgb_pixel_to_depth_pixel(cx, cy)
        x0, y0 = self._rgb_pixel_to_depth_pixel(cx - bbox_w / 2, cy - bbox_h / 2)
        x1, y1 = self._rgb_pixel_to_depth_pixel(cx + bbox_w / 2, cy + bbox_h / 2)
        w_d = abs(x1 - x0)
        h_d = abs(y1 - y0)
        return int(cx_d), int(cy_d), w_d, h_d

    # ── HSV red detection ─────────────────────────────────────────────────

    def _detect_red_cube(self):
        """Detect red object via HSV. Returns (cx, cy, bbox_w, bbox_h) or None."""
        if self.latest_rgb is None:
            return None

        hsv = cv2.cvtColor(self.latest_rgb, cv2.COLOR_BGR2HSV)

        # Combine masks for both red hue ranges
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for (lo, hi) in RED_HSV_RANGES:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))

        # Morphological opening to reduce noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        # Take the largest contour above minimum area
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < MIN_CONTOUR_AREA:
            return None

        # Centroid from moments
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        # Bounding box
        x, y, w, h = cv2.boundingRect(largest)

        return cx, cy, float(w), float(h)

    # ── Depth → point cloud (NO RANSAC) ──────────────────────────────────

    def _depth_to_points(self, depth_img, cx, cy, bbox_w, bbox_h):
        """Depth backprojection pipeline — no RANSAC for small cube."""
        diag = {}
        mode = MODES[self.mode_idx]
        pixel_map = PIXEL_MAPS[self.pixel_map_idx]

        if mode == 'depth_native':
            if self.depth_K is None:
                self.get_logger().error('No depth camera_info — cannot use depth_native mode')
                return None, diag
            K = self.depth_K
            tf_frame = 'camera_depth_optical_frame'
            depth_to_use = depth_img

            if pixel_map == 'rgb_to_depth':
                cx, cy, bbox_w, bbox_h = self._remap_bbox(cx, cy, bbox_w, bbox_h)
                diag['pixel_remap'] = f'RGB→Depth: ({cx}, {cy})'
            else:
                diag['pixel_remap'] = 'direct (no remap)'

        elif mode == 'color_aligned':
            if not ALIGN_AVAILABLE:
                self.get_logger().error('align_depth_to_rgb not available')
                return None, diag
            if self.color_K is None or self.latest_rgb is None:
                self.get_logger().error('No color data for alignment')
                return None, diag
            K = self.color_K
            tf_frame = 'camera_color_optical_frame'
            depth_to_use = align_depth(depth_img, self.latest_rgb)
            diag['pixel_remap'] = 'N/A (depth aligned to color)'

        elif mode == 'color_raw':
            if self.color_K is None:
                self.get_logger().error('No color camera_info')
                return None, diag
            K = self.color_K
            tf_frame = 'camera_color_optical_frame'
            depth_to_use = depth_img
            diag['pixel_remap'] = 'direct (buggy baseline)'

        else:
            return None, diag

        fx, fy = K[0, 0], K[1, 1]
        cx_k, cy_k = K[0, 2], K[1, 2]

        diag['intrinsics'] = f'fx={fx:.1f} fy={fy:.1f} cx={cx_k:.1f} cy={cy_k:.1f}'
        diag['tf_frame'] = tf_frame

        # Crop depth ROI
        h, w = depth_to_use.shape
        half_w = int(bbox_w * self.BBOX_PAD / 2)
        half_h = int(bbox_h * self.BBOX_PAD / 2)
        u0 = max(0, cx - half_w)
        v0 = max(0, cy - half_h)
        u1 = min(w, cx + half_w)
        v1 = min(h, cy + half_h)
        depth_roi = depth_to_use[v0:v1, u0:u1]

        # Backproject
        depth_m = depth_roi.astype(np.float32) / 1000.0
        roi_h, roi_w = depth_m.shape

        u_grid, v_grid = np.meshgrid(
            np.arange(roi_w) + u0,
            np.arange(roi_h) + v0)

        mask = (depth_m > self.MIN_DEPTH_M) & (depth_m < self.MAX_DEPTH_M)
        z = depth_m[mask]
        u = u_grid[mask].astype(np.float32)
        v = v_grid[mask].astype(np.float32)

        if len(z) < self.MIN_POINTS:
            self.get_logger().warn(f'Too few depth points: {len(z)}')
            return None, diag

        x_cam = (u - cx_k) * z / fx
        y_cam = (v - cy_k) * z / fy
        points_cam = np.stack([x_cam, y_cam, z], axis=1)

        # Transform to base_link
        tf_mat = self._get_tf_matrix('base_link', tf_frame)
        if tf_mat is None:
            return None, diag

        ones = np.ones((len(points_cam), 1), dtype=np.float32)
        pts_h = np.hstack([points_cam, ones])
        pts_base = (tf_mat @ pts_h.T).T[:, :3]

        diag['raw_points'] = len(pts_base)

        # Filter above table
        pts_base = pts_base[pts_base[:, 2] > self.TABLE_Z_MIN]

        # Outlier removal by Z (keep, but NO RANSAC)
        if len(pts_base) > self.MIN_POINTS:
            median_z = np.median(pts_base[:, 2])
            z_mask = np.abs(pts_base[:, 2] - median_z) < self.OUTLIER_Z_THRESH
            pts_base = pts_base[z_mask]

        diag['final_points'] = len(pts_base)

        if len(pts_base) < self.MIN_POINTS:
            return None, diag

        return pts_base, diag

    # ── Compute poses (fixed yaw=0) ──────────────────────────────────────

    def _compute_poses(self, pts_base):
        """Compute grasp and pre-grasp poses. Fixed yaw=0 for symmetric cube."""
        centroid = np.mean(pts_base, axis=0)
        z_top = float(np.max(pts_base[:, 2]))
        z_bottom = float(np.min(pts_base[:, 2]))

        grasp_yaw = 0.0
        qw, qx, qy, qz = yaw_to_grasp_quaternion(grasp_yaw)

        grasp_x = float(centroid[0]) + self.x_offset
        grasp_y = float(centroid[1]) + self.y_offset
        grasp_z = z_top + self.z_offset

        grasp_pose = Pose()
        grasp_pose.position.x = grasp_x
        grasp_pose.position.y = grasp_y
        grasp_pose.position.z = float(grasp_z)
        grasp_pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

        pre_grasp_pose = Pose()
        pre_grasp_pose.position.x = grasp_x
        pre_grasp_pose.position.y = grasp_y
        pre_grasp_pose.position.z = float(grasp_z + self.pre_height)
        pre_grasp_pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

        info = {
            'centroid': centroid,
            'z_top': z_top,
            'z_bottom': z_bottom,
            'cube_height': z_top - z_bottom,
        }
        return grasp_pose, pre_grasp_pose, info

    # ── Detection ─────────────────────────────────────────────────────────

    def _detect_and_center(self):
        """Detect red cube via HSV and center it in the camera frame."""
        self.get_logger().info('Looking for red cube (HSV)...')
        self._spin_for(0.5)

        result = None
        for _ in range(30):
            self._spin_for(0.2)
            result = self._detect_red_cube()
            if result is not None:
                break

        if result is None:
            self.get_logger().error('No red object detected')
            return None

        # Center object vertically by adjusting camera tilt
        deadline = time.time() + 5.0
        while time.time() < deadline:
            self._spin_for(0.2)
            result = self._detect_red_cube()
            if result is None:
                continue

            cx, cy, bw, bh = result
            if self.latest_depth is None:
                continue
            img_h = self.latest_depth.shape[0]
            offset_y = cy - img_h / 2.0

            if abs(offset_y) < 60:
                break

            tilt_adj = offset_y / (img_h / 2.0) * 0.02
            self._set_camera_tilt(self.current_tilt + tilt_adj)

        # Get fresh detection
        self._spin_for(0.5)
        result = self._detect_red_cube()
        if result is None:
            self.get_logger().error('Lost red cube after centering')
            return None

        return result

    # ── Main iteration ────────────────────────────────────────────────────

    def _run_iteration(self, full_grasp=False):
        self.iteration += 1
        mode = MODES[self.mode_idx]
        pixel_map = PIXEL_MAPS[self.pixel_map_idx]

        print()
        print(f'{"=" * 60}')
        print(f'=== Iteration {self.iteration} | Mode: {mode} | Pixels: {pixel_map} ===')
        print(f'Offsets: X={self.x_offset*1000:.0f}mm  '
              f'Y={self.y_offset*1000:.0f}mm  '
              f'Z={self.z_offset*1000:.0f}mm  '
              f'Pre-grasp height={self.pre_height*1000:.0f}mm')
        print(f'{"=" * 60}')

        # Step 1: Retract arm (clear camera view)
        self._retract_arm()
        self._spin_for(1.0)

        # Step 2: Detect red cube
        result = self._detect_and_center()
        if result is None:
            print('>>> FAILED: No red cube detected')
            return

        cx, cy, bbox_w, bbox_h = result
        print(f'HSV detection: pixel ({cx}, {cy}), bbox {bbox_w:.0f}x{bbox_h:.0f}, '
              f'area ~{bbox_w*bbox_h:.0f}px²')

        # Step 3: Depth → point cloud (no RANSAC)
        if self.latest_depth is None:
            print('>>> FAILED: No depth image')
            return

        pts_base, diag = self._depth_to_points(
            self.latest_depth, cx, cy, bbox_w, bbox_h)

        # Print diagnostics
        if 'pixel_remap' in diag:
            print(f'Pixel mapping: {diag["pixel_remap"]}')
        if 'intrinsics' in diag:
            print(f'Intrinsics: {diag["intrinsics"]}')
        if 'tf_frame' in diag:
            print(f'TF frame: {diag["tf_frame"]}')
        if 'raw_points' in diag:
            print(f'Points: {diag["raw_points"]} raw → {diag.get("final_points", "?")} after filtering')

        if pts_base is None:
            print('>>> FAILED: Insufficient points')
            return

        # Step 4: Compute poses (fixed yaw=0)
        grasp_pose, pre_grasp_pose, info = self._compute_poses(pts_base)
        c = info['centroid']
        print(f'Centroid (base_link): ({c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f})')
        print(f'z_top: {info["z_top"]:.4f}m, z_bottom: {info["z_bottom"]:.4f}m, '
              f'cube height: {info["cube_height"]*1000:.1f}mm '
              f'(expected ~25.4mm)')

        gp = grasp_pose.position
        pp = pre_grasp_pose.position
        print(f'Grasp pose: ({gp.x:.4f}, {gp.y:.4f}, {gp.z:.4f})')
        print(f'Pre-grasp:  ({pp.x:.4f}, {pp.y:.4f}, {pp.z:.4f})')

        # Step 5: Check IK
        ok, msg = self._try_ik(pre_grasp_pose, use_orientation=True)
        if not ok:
            print(f'>>> FAILED: IK failed: {msg}')
            return
        print('IK: SUCCESS')

        # Step 6: Close gripper FIRST (seam = alignment reference)
        print('Closing gripper (alignment seam)...')
        self._close_gripper_firm()

        # Step 7: Move to pre-grasp with gripper closed
        print('Moving to pre-grasp (2cm above cube, gripper closed)...')
        ok, msg = self._plan_and_execute(pre_grasp_pose, use_orientation=True)
        if not ok:
            print(f'>>> FAILED to reach pre-grasp: {msg}')
            return

        self._spin_for(0.5)
        print('>>> At pre-grasp. Inspect gripper seam alignment over cube.')

        if full_grasp:
            self._execute_full_grasp(grasp_pose)

    def _execute_full_grasp(self, grasp_pose):
        """Open gripper, descend, close, lift."""
        print('Opening gripper...')
        self._set_gripper(0.0)
        self._spin_for(0.5)

        print('Descending to grasp...')
        ok, msg = self._plan_and_execute(grasp_pose, use_orientation=True)
        if not ok:
            print(f'>>> FAILED descent: {msg}')
            return

        self._spin_for(0.5)
        print('Closing gripper...')
        self._close_gripper_firm()

        print('Lifting...')
        lift = Pose()
        lift.position.x = grasp_pose.position.x
        lift.position.y = grasp_pose.position.y
        lift.position.z = grasp_pose.position.z + 0.10
        lift.orientation = grasp_pose.orientation
        ok, msg = self._plan_and_execute(lift, use_orientation=True)
        if not ok:
            print(f'Lift failed: {msg}')

        print('>>> Grasp complete. Cube should be lifted.')

    # ── Print diagnostics ─────────────────────────────────────────────────

    def _print_diagnostics(self):
        print()
        print('=' * 60)
        print('DIAGNOSTICS')
        print('=' * 60)

        if self.color_K is not None:
            K = self.color_K
            print(f'Color intrinsics: fx={K[0,0]:.2f} fy={K[1,1]:.2f} '
                  f'cx={K[0,2]:.2f} cy={K[1,2]:.2f}')
        else:
            print('Color intrinsics: NOT RECEIVED')

        if self.depth_K is not None:
            K = self.depth_K
            print(f'Depth intrinsics: fx={K[0,0]:.2f} fy={K[1,1]:.2f} '
                  f'cx={K[0,2]:.2f} cy={K[1,2]:.2f}')
        else:
            print('Depth intrinsics: NOT RECEIVED')

        for src in ['camera_color_optical_frame', 'camera_depth_optical_frame']:
            mat = self._get_tf_matrix('base_link', src)
            if mat is not None:
                t = mat[:3, 3]
                print(f'TF base_link ← {src}:')
                print(f'  translation: ({t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f})')
                R = mat[:3, :3]
                r = Rotation.from_matrix(R)
                euler = r.as_euler('xyz', degrees=True)
                print(f'  rotation (xyz euler): '
                      f'({euler[0]:.1f}, {euler[1]:.1f}, {euler[2]:.1f}) deg')
            else:
                print(f'TF base_link ← {src}: NOT AVAILABLE')

        if self.color_K is not None and self.depth_K is not None:
            test_pts = [(320, 240), (160, 120), (480, 360)]
            print('Pixel remap examples (RGB → Depth):')
            for u, v in test_pts:
                ud, vd = self._rgb_pixel_to_depth_pixel(u, v)
                print(f'  ({u}, {v}) → ({ud:.1f}, {vd:.1f})')

        # Test HSV detection
        result = self._detect_red_cube()
        if result is not None:
            cx, cy, bw, bh = result
            print(f'HSV red detection: pixel ({cx}, {cy}), '
                  f'bbox {bw:.0f}x{bh:.0f}')
        else:
            print('HSV red detection: NO RED OBJECT FOUND')

        print(f'Latest depth: {"available" if self.latest_depth is not None else "NONE"}')
        print(f'Latest RGB: {"available" if self.latest_rgb is not None else "NONE"}')
        print(f'Align available: {ALIGN_AVAILABLE}')
        print(f'Current tilt: {self.current_tilt:.3f} rad')
        print('=' * 60)

    # ── Interactive loop ──────────────────────────────────────────────────

    def run_loop(self):
        print('Waiting for camera data and TF...')
        for _ in range(50):
            self._spin_for(0.2)
            if self.latest_depth is not None and self.latest_rgb is not None:
                break

        if self.latest_depth is None:
            print('ERROR: No depth data received after 10s')
            return
        if self.latest_rgb is None:
            print('ERROR: No RGB data received — HSV detection needs it')
            return
        if self.depth_K is None:
            print('WARNING: No depth camera_info — depth_native mode will not work')

        self._set_camera_tilt(0.3)
        self._spin_for(1.0)

        self._print_status()

        while True:
            try:
                cmd = input('\n>>> Command (Enter=run, 1-6/r/p/g/q): ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if cmd == 'q':
                self._retract_arm()
                print('Exiting.')
                break
            elif cmd == '':
                self._run_iteration(full_grasp=False)
            elif cmd == 'g':
                self._run_iteration(full_grasp=True)
            elif cmd == '1':
                self.mode_idx = (self.mode_idx + 1) % len(MODES)
                print(f'Mode: {MODES[self.mode_idx]}')
            elif cmd == '2':
                self.pixel_map_idx = (self.pixel_map_idx + 1) % len(PIXEL_MAPS)
                print(f'Pixel mapping: {PIXEL_MAPS[self.pixel_map_idx]}')
            elif cmd.startswith('3'):
                delta = 0.005 if '+' in cmd else -0.005
                self.x_offset += delta
                print(f'X offset: {self.x_offset*1000:.0f}mm')
            elif cmd.startswith('4'):
                delta = 0.005 if '+' in cmd else -0.005
                self.y_offset += delta
                print(f'Y offset: {self.y_offset*1000:.0f}mm')
            elif cmd.startswith('5'):
                delta = 0.005 if '+' in cmd else -0.005
                self.z_offset += delta
                print(f'Z offset: {self.z_offset*1000:.0f}mm')
            elif cmd.startswith('6'):
                delta = 0.005 if '+' in cmd else -0.005
                self.pre_height = max(0.01, self.pre_height + delta)
                print(f'Pre-grasp height: {self.pre_height*1000:.0f}mm')
            elif cmd == 'r':
                self.x_offset = self.DEFAULT_X_OFFSET
                self.y_offset = self.DEFAULT_Y_OFFSET
                self.z_offset = self.DEFAULT_Z_OFFSET
                self.pre_height = self.DEFAULT_PRE_HEIGHT
                print('Offsets reset to defaults')
            elif cmd == 'p':
                self._print_diagnostics()
            else:
                print(f'Unknown command: {cmd}')
                self._print_help()

            self._spin_for(0.1)

    def _print_status(self):
        mode = MODES[self.mode_idx]
        pixel_map = PIXEL_MAPS[self.pixel_map_idx]
        print()
        print(f'Current settings:')
        print(f'  Mode: {mode}')
        print(f'  Pixel mapping: {pixel_map}')
        print(f'  X offset: {self.x_offset*1000:.0f}mm')
        print(f'  Y offset: {self.y_offset*1000:.0f}mm')
        print(f'  Z offset: {self.z_offset*1000:.0f}mm')
        print(f'  Pre-grasp height: {self.pre_height*1000:.0f}mm')
        print(f'  Gripper: CLOSED at pre-grasp (seam alignment)')
        print(f'  RANSAC: DISABLED')
        self._print_help()

    def _print_help(self):
        print()
        print('Commands:')
        print('  Enter  — retract arm, detect red cube, close gripper, move to pre-grasp')
        print('  1      — cycle backprojection mode')
        print('  2      — toggle pixel mapping (rgb_to_depth / direct)')
        print('  3+/3-  — X offset ±5mm')
        print('  4+/4-  — Y offset ±5mm')
        print('  5+/5-  — Z offset ±5mm')
        print('  6+/6-  — pre-grasp height ±5mm')
        print('  r      — reset offsets')
        print('  p      — print diagnostics')
        print('  g      — full grasp (open, descend, close, lift)')
        print('  q      — quit')


def main():
    rclpy.init()
    node = CubeCalibrationNode()
    try:
        node.run_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
