#!/usr/bin/env python3
"""
Grasp Calibration Test — interactive pre-grasp pose testing tool.

Rapidly test and troubleshoot depth→world coordinate transforms for grasping.
The robot moves its arm to a pre-grasp pose above a detected object, and you
can visually inspect alignment while toggling different backprojection modes
and offsets.

Usage:
  Terminal 1: ros2 launch tidybot_bringup real.launch.py
  Terminal 2: ros2 run tidybot_bringup grasp_calibration_test.py

Interactive commands (type at the prompt):
  Enter  — retract arm, re-detect, move to pre-grasp
  1      — cycle backprojection mode (depth_native / color_aligned / color_raw)
  2      — toggle pixel mapping (rgb_to_depth / direct)
  3+/3-  — adjust X offset ±5mm
  4+/4-  — adjust Y offset ±5mm
  5+/5-  — adjust Z offset ±5mm
  6+/6-  — adjust pre-grasp height ±10mm
  r      — reset all offsets to defaults
  p      — print full diagnostics (intrinsics, TF, etc.)
  g      — execute full grasp (descend + close + lift)
  q      — return arm to sleep and quit
"""

import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray
from vision_msgs.msg import Detection2DArray
from tidybot_msgs.msg import ArmCommand
from tidybot_msgs.srv import PlanToTarget
from scipy.spatial.transform import Rotation

import tf2_ros

from tidybot_perception.grasp_geometry import (
    yaw_to_grasp_quaternion,
    GRIPPER_MAX_OPENING_M,
)

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


# ── RANSAC floor separation (from explore_and_find_object.py) ────────────────

FLOOR_MARGIN = 0.008

def _ransac_floor_separate(pts, distance_thresh=0.008, max_iterations=200,
                            min_inlier_ratio=0.20, min_pts=20):
    """RANSAC plane fit to find the dominant planar surface (floor/table)."""
    n = len(pts)
    if n < min_pts:
        return float(np.median(pts[:, 2])), pts

    best_inlier_mask = None
    best_count = 0

    rng = np.random.default_rng(42)
    for _ in range(max_iterations):
        idx = rng.choice(n, 3, replace=False)
        p0, p1, p2 = pts[idx]

        v1 = p1 - p0
        v2 = p2 - p0
        normal = np.cross(v1, v2)
        norm_len = np.linalg.norm(normal)
        if norm_len < 1e-10:
            continue
        normal /= norm_len

        dists = np.abs((pts - p0) @ normal)
        inlier_mask = dists < distance_thresh
        count = inlier_mask.sum()

        if count > best_count:
            best_count = count
            best_inlier_mask = inlier_mask

    if best_inlier_mask is None or best_count < min_inlier_ratio * n:
        return float(np.min(pts[:, 2])), pts

    floor_z = float(np.median(pts[best_inlier_mask, 2]))
    outlier_mask = ~best_inlier_mask

    if outlier_mask.sum() < min_pts:
        return floor_z, pts

    return floor_z, pts[outlier_mask]


# ── Backprojection modes ─────────────────────────────────────────────────────

MODES = ['depth_native', 'color_aligned', 'color_raw']
PIXEL_MAPS = ['rgb_to_depth', 'direct']


class GraspCalibrationNode(Node):

    # Defaults (matching explore_and_find_object.py)
    DEFAULT_X_OFFSET = -0.05     # m
    DEFAULT_Y_OFFSET = -0.05     # m
    DEFAULT_Z_OFFSET = 0.00      # m
    DEFAULT_PRE_HEIGHT = 0.05    # m (5cm — close enough to see alignment)

    GRASP_ARM_NAME = 'right'
    MOVE_DURATION = 2.5
    BBOX_PAD = 1.3
    MIN_DEPTH_M = 0.1
    MAX_DEPTH_M = 2.0
    MIN_POINTS = 20
    TABLE_Z_MIN = 0.005
    OUTLIER_Z_THRESH = 0.05
    MIN_CONFIDENCE = 0.25

    # Sleep pose — arm tucked out of camera view
    SLEEP_JOINTS = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]

    # YOLO banana class ID (COCO)
    TARGET_CLASS_ID = 46  # banana

    def __init__(self):
        super().__init__('grasp_calibration_test')

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
        self.color_K = None          # from /camera/color/camera_info
        self.depth_K = None          # from /camera/depth/camera_info
        self.latest_yolo = None
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
        self.create_subscription(
            Detection2DArray, '/objbbox', self._yolo_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

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
        self.get_logger().info('Grasp Calibration Test — interactive pre-grasp tester')
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

    def _yolo_cb(self, msg):
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            if int(hyp.class_id) == self.TARGET_CLASS_ID and hyp.score >= self.MIN_CONFIDENCE:
                self.latest_yolo = det
                return

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
        """Plan IK and execute motion to target pose in base_link."""
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
        """Check IK without executing."""
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
        """Map pixel from RGB image space to depth image space."""
        if self.color_K is None or self.depth_K is None:
            return u_rgb, v_rgb

        pt = np.array([u_rgb, v_rgb, 1.0])
        pt_norm = np.linalg.inv(self.color_K) @ pt
        pt_depth = self.depth_K @ pt_norm
        return float(pt_depth[0]), float(pt_depth[1])

    def _remap_bbox(self, cx, cy, bbox_w, bbox_h):
        """Remap YOLO bbox center and corners from RGB to depth pixel space."""
        cx_d, cy_d = self._rgb_pixel_to_depth_pixel(cx, cy)

        # Remap bbox corners to get approximate size in depth space
        x0, y0 = self._rgb_pixel_to_depth_pixel(cx - bbox_w / 2, cy - bbox_h / 2)
        x1, y1 = self._rgb_pixel_to_depth_pixel(cx + bbox_w / 2, cy + bbox_h / 2)
        w_d = abs(x1 - x0)
        h_d = abs(y1 - y0)

        return int(cx_d), int(cy_d), w_d, h_d

    # ── Depth → point cloud ───────────────────────────────────────────────

    def _depth_to_points(self, depth_img, cx, cy, bbox_w, bbox_h):
        """Full pipeline: crop, backproject, transform to base_link.

        Returns (points_base, diagnostics_dict) or (None, diagnostics_dict).
        """
        diag = {}
        mode = MODES[self.mode_idx]
        pixel_map = PIXEL_MAPS[self.pixel_map_idx]

        # Determine which intrinsics and TF frame to use
        if mode == 'depth_native':
            if self.depth_K is None:
                self.get_logger().error('No depth camera_info — cannot use depth_native mode')
                return None, diag
            K = self.depth_K
            tf_frame = 'camera_depth_optical_frame'
            depth_to_use = depth_img

            # Remap YOLO pixels from RGB to depth space
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
            # Warp depth to color frame
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

        # Outlier removal by Z
        if len(pts_base) > self.MIN_POINTS:
            median_z = np.median(pts_base[:, 2])
            z_mask = np.abs(pts_base[:, 2] - median_z) < self.OUTLIER_Z_THRESH
            pts_base = pts_base[z_mask]

        if len(pts_base) < self.MIN_POINTS:
            return None, diag

        # RANSAC floor separation
        floor_z, obj_pts = _ransac_floor_separate(
            pts_base, distance_thresh=FLOOR_MARGIN,
            min_pts=self.MIN_POINTS)

        if len(obj_pts) < self.MIN_POINTS:
            self.get_logger().warn(
                f'RANSAC removed too many points ({len(pts_base)} → {len(obj_pts)}), using all')
            obj_pts = pts_base

        diag['floor_z'] = floor_z
        diag['points_after_ransac'] = len(obj_pts)

        return obj_pts, diag

    # ── Object analysis (from explore_and_find_object.py) ─────────────────

    def _analyze_object(self, points_base):
        """Find optimal grasp yaw and compute grasp center."""
        centroid = np.mean(points_base, axis=0)
        z_top = float(np.max(points_base[:, 2]))

        pts_xy = points_base[:, :2]
        centered = pts_xy - centroid[:2]

        best_yaw = 0.0
        best_clearance = -1.0
        best_width = float('inf')
        best_center_offset = 0.0

        for angle_deg in range(0, 180, 5):
            angle = np.radians(angle_deg)
            finger_dir = np.array([np.cos(angle), np.sin(angle)])
            projections = centered @ finger_dir
            proj_min = float(projections.min())
            proj_max = float(projections.max())
            width = proj_max - proj_min

            center_offset = (proj_min + proj_max) / 2.0
            clearance = (GRIPPER_MAX_OPENING_M - width) / 2.0

            if clearance > best_clearance:
                best_clearance = clearance
                best_yaw = angle
                best_width = width
                best_center_offset = center_offset

        finger_dir = np.array([np.cos(best_yaw), np.sin(best_yaw)])
        grasp_center = centroid.copy()
        grasp_center[0] += best_center_offset * finger_dir[0]
        grasp_center[1] += best_center_offset * finger_dir[1]

        return {
            'centroid': centroid,
            'grasp_center': grasp_center,
            'grasp_yaw': best_yaw,
            'grip_width': best_width,
            'min_clearance': best_clearance,
            'z_top': z_top,
            'num_points': len(points_base),
        }

    # ── Compute poses ─────────────────────────────────────────────────────

    def _compute_poses(self, analysis):
        """Compute grasp and pre-grasp poses with current offsets."""
        grasp_center = analysis.get('grasp_center', analysis['centroid'])
        grasp_yaw = analysis['grasp_yaw']

        qw, qx, qy, qz = yaw_to_grasp_quaternion(grasp_yaw)

        grasp_x = float(grasp_center[0]) + self.x_offset
        grasp_y = float(grasp_center[1]) + self.y_offset
        grasp_z = analysis['z_top'] + self.z_offset

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

        return grasp_pose, pre_grasp_pose, grasp_yaw

    # ── Detection ─────────────────────────────────────────────────────────

    def _detect_object(self):
        """Wait for a YOLO detection, center it, return detection info."""
        self.get_logger().info('Waiting for YOLO detection...')
        self.latest_yolo = None
        self._spin_for(0.5)

        det = None
        for _ in range(30):
            self._spin_for(0.2)
            if self.latest_yolo is not None:
                det = self.latest_yolo
                break

        if det is None:
            self.get_logger().error('No YOLO detection found')
            return None

        # Center object vertically by adjusting camera tilt
        deadline = time.time() + 5.0
        while time.time() < deadline:
            self._spin_for(0.2)
            det = self.latest_yolo
            if det is None:
                continue
            self.latest_yolo = None

            v = int(det.bbox.center.position.y)
            if self.latest_depth is None:
                continue
            img_h = self.latest_depth.shape[0]
            offset_y = v - img_h / 2.0

            if abs(offset_y) < 60:
                break

            tilt_adj = offset_y / (img_h / 2.0) * 0.02
            self._set_camera_tilt(self.current_tilt + tilt_adj)

        # Get fresh detection
        self.latest_yolo = None
        self._spin_for(0.5)
        for _ in range(10):
            self._spin_for(0.2)
            if self.latest_yolo is not None:
                break

        det = self.latest_yolo
        if det is None:
            self.get_logger().error('Lost object after centering')
            return None

        self.latest_yolo = None
        return det

    # ── Main iteration ────────────────────────────────────────────────────

    def _run_iteration(self, full_grasp=False):
        """One detect-compute-move cycle."""
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

        # Step 1: Retract arm
        self._retract_arm()
        self._spin_for(1.0)

        # Step 2: Detect
        det = self._detect_object()
        if det is None:
            print('>>> FAILED: No detection')
            return

        cx = int(det.bbox.center.position.x)
        cy = int(det.bbox.center.position.y)
        bbox_w = float(det.bbox.size_x)
        bbox_h = float(det.bbox.size_y)
        print(f'YOLO: pixel ({cx}, {cy}), bbox {bbox_w:.0f}x{bbox_h:.0f}')

        # Step 3: Depth → point cloud
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
            print(f'Raw points: {diag["raw_points"]}')
        if 'points_after_ransac' in diag:
            print(f'After RANSAC: {diag["points_after_ransac"]} '
                  f'(floor_z={diag.get("floor_z", 0):.3f}m)')

        if pts_base is None:
            print('>>> FAILED: Insufficient points')
            return

        # Step 4: Analyze + compute poses
        analysis = self._analyze_object(pts_base)
        c = analysis['centroid']
        print(f'Centroid (base_link): ({c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f})')
        print(f'z_top: {analysis["z_top"]:.3f}m, '
              f'grip width: {analysis["grip_width"]*1000:.0f}mm, '
              f'yaw: {np.degrees(analysis["grasp_yaw"]):.0f}deg')

        grasp_pose, pre_grasp_pose, grasp_yaw = self._compute_poses(analysis)
        gp = grasp_pose.position
        pp = pre_grasp_pose.position
        print(f'Grasp pose: ({gp.x:.3f}, {gp.y:.3f}, {gp.z:.3f})')
        print(f'Pre-grasp:  ({pp.x:.3f}, {pp.y:.3f}, {pp.z:.3f})')

        # Step 5: Check IK with orientation, try alternatives if needed
        ok, msg = self._try_ik(pre_grasp_pose, use_orientation=True)
        final_orientation_used = True
        if not ok:
            print(f'IK failed for primary yaw: {msg}')
            found_alt = False
            for alt_offset in [np.pi/2, -np.pi/2, np.pi]:
                alt_yaw = grasp_yaw + alt_offset
                qw, qx, qy, qz = yaw_to_grasp_quaternion(alt_yaw)
                alt_q = Quaternion(w=qw, x=qx, y=qy, z=qz)
                pre_grasp_pose.orientation = alt_q
                grasp_pose.orientation = alt_q
                ok2, _ = self._try_ik(pre_grasp_pose, use_orientation=True)
                if ok2:
                    print(f'Using alt yaw: +{np.degrees(alt_offset):.0f}deg')
                    found_alt = True
                    break
            if not found_alt:
                print('>>> FAILED: All IK orientations failed')
                return
        print('IK: SUCCESS')

        # Step 6: Open gripper and move to pre-grasp
        self._set_gripper(0.0)
        print('Moving to pre-grasp...')
        ok, msg = self._plan_and_execute(pre_grasp_pose, use_orientation=True)
        if not ok:
            print(f'>>> FAILED to reach pre-grasp: {msg}')
            return

        # Re-open gripper (ensure it stayed open during motion)
        self._set_gripper(0.0)
        self._spin_for(0.5)
        print('>>> At pre-grasp. Inspect alignment visually.')

        if full_grasp:
            self._execute_full_grasp(grasp_pose)

    def _execute_full_grasp(self, grasp_pose):
        """Descend to grasp, close gripper, lift."""
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
        lift.position.z = grasp_pose.position.z + 0.15
        lift.orientation = grasp_pose.orientation
        ok, msg = self._plan_and_execute(lift, use_orientation=True)
        if not ok:
            print(f'Lift failed: {msg}')

        print('>>> Grasp complete. Object should be lifted.')

    # ── Print diagnostics ─────────────────────────────────────────────────

    def _print_diagnostics(self):
        """Print all available camera and TF info."""
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

        # TF matrices
        for src in ['camera_color_optical_frame', 'camera_depth_optical_frame']:
            mat = self._get_tf_matrix('base_link', src)
            if mat is not None:
                t = mat[:3, 3]
                print(f'TF base_link ← {src}:')
                print(f'  translation: ({t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f})')
                R = mat[:3, :3]
                r = Rotation.from_matrix(R)
                euler = r.as_euler('xyz', degrees=True)
                print(f'  rotation (xyz euler): ({euler[0]:.1f}, {euler[1]:.1f}, {euler[2]:.1f}) deg')
            else:
                print(f'TF base_link ← {src}: NOT AVAILABLE')

        # Pixel remap example
        if self.color_K is not None and self.depth_K is not None:
            test_pts = [(320, 240), (160, 120), (480, 360)]
            print('Pixel remap examples (RGB → Depth):')
            for u, v in test_pts:
                ud, vd = self._rgb_pixel_to_depth_pixel(u, v)
                print(f'  ({u}, {v}) → ({ud:.1f}, {vd:.1f})')

        print(f'Latest depth: {"available" if self.latest_depth is not None else "NONE"}')
        print(f'Latest RGB: {"available" if self.latest_rgb is not None else "NONE"}')
        print(f'Align available: {ALIGN_AVAILABLE}')
        print(f'Current tilt: {self.current_tilt:.3f} rad')
        print('=' * 60)

    # ── Interactive loop ──────────────────────────────────────────────────

    def run_loop(self):
        """Main interactive loop."""
        # Wait for data
        print('Waiting for camera data and TF...')
        for _ in range(50):
            self._spin_for(0.2)
            if self.latest_depth is not None and self.color_K is not None:
                break

        if self.latest_depth is None:
            print('ERROR: No depth data received after 10s')
            return
        if self.color_K is None:
            print('WARNING: No color camera_info — some modes may not work')
        if self.depth_K is None:
            print('WARNING: No depth camera_info — depth_native mode will not work')

        # Set initial camera tilt for looking at objects
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
                delta = 0.01 if '+' in cmd else -0.01
                self.pre_height = max(0.02, self.pre_height + delta)
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

            # Spin to keep callbacks alive
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
        self._print_help()

    def _print_help(self):
        print()
        print('Commands:')
        print('  Enter  — retract arm, detect, move to pre-grasp')
        print('  1      — cycle backprojection mode')
        print('  2      — toggle pixel mapping (rgb_to_depth / direct)')
        print('  3+/3-  — X offset ±5mm')
        print('  4+/4-  — Y offset ±5mm')
        print('  5+/5-  — Z offset ±5mm')
        print('  6+/6-  — pre-grasp height ±10mm')
        print('  r      — reset offsets')
        print('  p      — print diagnostics')
        print('  g      — full grasp (descend + close + lift)')
        print('  q      — quit')


def main():
    rclpy.init()
    node = GraspCalibrationNode()
    try:
        node.run_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
