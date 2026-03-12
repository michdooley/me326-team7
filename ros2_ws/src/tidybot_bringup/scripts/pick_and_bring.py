#!/usr/bin/env python3
"""
TidyBot2 Pick-and-Bring Pipeline

Integrated pipeline that chains object finding/grasping with person finding/delivery:
  Phase 1 (YOLO): find object -> center -> approach -> grasp
  Phase 2 (YOLO): find person -> center -> approach -> raise arm -> wait for "drop" voice command

Runs alongside:
  - ros2 run object_classification classifier   (YOLO, publishes /objbbox)
  - ros2 run tidybot_bringup voice_command.py   (publishes /target_object, /user_command)

Build:
  cd ros2_ws && colcon build --packages-select tidybot_perception tidybot_bringup && source install/setup.bash

Terminal 1: robot/rviz bringup
  ros2 launch tidybot_bringup sim.launch.py

Terminal 2: YOLO classifier
  ros2 run object_classification classifier

Terminal 3: pick and bring (no voice)
  ros2 run tidybot_bringup pick_and_bring.py --ros-args -p skip_voice:=true -p target_object:=banana
"""

import os
import time
from datetime import datetime
from collections import deque
from enum import Enum

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose, Quaternion, Twist
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA, Float64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
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


class TaskState(Enum):
    WAITING_FOR_COMMAND = 0
    # Phase 1: Find & grasp object (YOLO)
    SCANNING_OBJECT     = 10
    CENTERING_OBJECT    = 11
    APPROACHING_OBJECT  = 12
    OBJECT_REACHED      = 13
    GRASPING            = 14
    # Phase 2: Find person & approach (YOLO)
    SCANNING_PERSON     = 20
    CENTERING_PERSON    = 21
    APPROACHING_PERSON  = 22
    # Phase 3: Deliver
    WAITING_FOR_DROP    = 30
    # Terminal
    TASK_COMPLETE       = 40


# YOLO class name -> ID mapping (COCO dataset)
YOLO_CLASS_MAP = {
    'person': 0, 'bicycle': 1, 'car': 2, 'motorcycle': 3, 'airplane': 4,
    'bus': 5, 'train': 6, 'truck': 7, 'boat': 8, 'traffic light': 9,
    'fire hydrant': 10, 'stop sign': 11, 'parking meter': 12, 'bench': 13,
    'bird': 14, 'cat': 15, 'dog': 16, 'horse': 17, 'sheep': 18, 'cow': 19,
    'elephant': 20, 'bear': 21, 'zebra': 22, 'giraffe': 23,
    'backpack': 24, 'umbrella': 25, 'handbag': 26, 'tie': 27, 'suitcase': 28,
    'frisbee': 29, 'skis': 30, 'snowboard': 31, 'sports ball': 32,
    'kite': 33, 'baseball bat': 34, 'baseball glove': 35, 'skateboard': 36,
    'surfboard': 37, 'tennis racket': 38,
    'bottle': 39, 'wine glass': 40, 'cup': 41, 'fork': 42, 'knife': 43,
    'spoon': 44, 'bowl': 45, 'banana': 46, 'apple': 47, 'sandwich': 48,
    'orange': 49, 'broccoli': 50, 'carrot': 51, 'hot dog': 52, 'pizza': 53,
    'donut': 54, 'cake': 55,
    'chair': 56, 'couch': 57, 'potted plant': 58, 'bed': 59,
    'dining table': 60, 'toilet': 61, 'tv': 62, 'laptop': 63, 'mouse': 64,
    'remote': 65, 'keyboard': 66, 'cell phone': 67, 'microwave': 68,
    'oven': 69, 'toaster': 70, 'sink': 71, 'refrigerator': 72, 'book': 73,
    'clock': 74, 'vase': 75, 'scissors': 76, 'teddy bear': 77,
    'hair drier': 78, 'toothbrush': 79,
}

# Voice command: "get the banana and bring it to me"
VOICE_PHRASE = "get the banana and bring it to me"

# Saved joint poses  [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]
GRASP_VALIDATION_POS = [0.0, -1.0, 0.6, 0.0, 0.4, 0.0]   # good pose for grasp validation (future use)
RETRACT_HOLDING_POS  = [0.0, -1.35, 0.6, 0.0, 0.75, 0.0]  # high overhead hold — clears camera & bin
PRESENT_POSE         = [0.0, -0.5,  0.5,  0.0, -0.5,  0.0]  # raised presentation pose

FLOOR_MARGIN = 0.008  # RANSAC inlier distance threshold for floor plane


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


class PickAndBring(Node):

    # ── Depth filtering ───────────────────────────────────────────────────────
    MIN_DEPTH_M       = 0.15
    MAX_DEPTH_M       = 5.00

    # ── 360 scan ─────────────────────────────────────────────────────────────
    SCAN_ANGULAR_VEL = -0.3   # rad/s
    SCAN_SETTLE_TIME = 0.5    # s
    MAX_SCAN_ATTEMPTS = 3

    # ── Centering (phase-specific overrides below) ────────────────────────────
    CENTER_ANGULAR_VEL  = 0.25  # rad/s
    CENTER_TILT_STEP    = 0.02  # rad
    CENTER_TIMEOUT      = 15.0  # s

    # ── Object detection ──────────────────────────────────────────────────────
    MIN_CONFIDENCE     = 0.25
    MIN_DETECTION_AREA = 50
    DETECT_WINDOW      = 2.0   # s
    DETECT_COUNT_REQ   = 1

    # ── Approach ─────────────────────────────────────────────────────────────
    APPROACH_ANGULAR_GAIN      = 0.003
    APPROACH_MAX_ANGULAR       = 0.3
    APPROACH_TIMEOUT           = 120.0
    APPROACH_TILT_STEP         = 0.03

    # ── Return home ────────────────────────────────────────────────────────
    WAYPOINT_INTERVAL     = 0.20
    RETURN_LINEAR_SPEED   = 0.10
    RETURN_ANGULAR_GAIN   = 2.0
    RETURN_MAX_ANGULAR    = 0.4
    RETURN_WAYPOINT_REACH = 0.10

    # ── Grasp parameters ─────────────────────────────────────────────────────
    GRASP_ARM_NAME               = 'right'
    GRASP_MOVE_DURATION          = 2.0
    GRASP_PRE_HEIGHT             = 0.10
    GRASP_LIFT_HEIGHT            = 0.15
    GRASP_Z_OFFSET               = 0.03
    GRASP_X_OFFSET               = -0.03
    GRASP_Y_OFFSET               = -0.00
    GRASP_BBOX_PAD               = 1.3
    GRASP_SETTLE_TIME            = 1.0
    GRASP_GRIPPER_CLOSE_REPEATS  = 20
    GRASP_GRIPPER_CLOSE_WAIT     = 2.0
    GRASP_MIN_DEPTH_M            = 0.1
    GRASP_MAX_DEPTH_M            = 2.0
    GRASP_MIN_POINTS             = 20
    GRASP_TABLE_Z_MIN            = 0.005
    GRASP_OUTLIER_Z_THRESH       = 0.05
    GRASP_MAX_ATTEMPTS           = 4
    GRASP_MAX_IK_NUDGES          = 6   # nudge-and-retry cycles per attempt

    # ── Sweet spot (grasp positioning) ────────────────────────────────────────
    GRASP_SWEET_SPOT_X      = 0.04
    GRASP_SWEET_SPOT_Y      = -0.26
    GRASP_SWEET_SPOT_RADIUS = 0.12
    GRASP_NUDGE_SPEED       = 0.04

    # ── Phase-specific parameters ─────────────────────────────────────────────
    PHASE_PARAMS = {
        'object': {
            'center_pixel_thresh': 80,
            'center_pixel_thresh_v': 60,
            'approach_stop_dist': 0.35,
            'approach_linear_speed': 0.05,
            'initial_tilt': 0.7854,  # 45 degrees
            'scan_tilt': 0.3,
        },
        'person': {
            'center_pixel_thresh':   80,
            'center_pixel_thresh_v': 60,
            'approach_stop_dist':    0.6,
            'approach_linear_speed': 0.05,
            'initial_tilt':          0.5,
            'scan_tilt':             0.3,
        },
    }

    # ─────────────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('pick_and_bring')

        # Camera state
        self.latest_depth       = None
        self.latest_depth_stamp = None
        self.latest_rgb         = None
        self.camera_K           = None
        self.cv_bridge          = CvBridge()

        # State machine
        self.state             = TaskState.WAITING_FOR_COMMAND
        self._phase            = 'object'  # 'object', 'person'
        self.scan_accumulated  = 0.0
        self.scan_last_heading = None
        self.scan_settle_start = None
        self.scan_attempt_count = 0

        # Camera tilt tracking
        self.current_tilt = 0.3

        # Voice command parameters
        self.declare_parameter('skip_voice', False)
        self.skip_voice = (
            self.get_parameter('skip_voice')
            .get_parameter_value().bool_value)
        self.declare_parameter('target_object', 'banana')
        self.declare_parameter('user_command', 'get')
        self.declare_parameter('arm_name', 'right')

        self.GRASP_ARM_NAME = (
            self.get_parameter('arm_name')
            .get_parameter_value().string_value.lower())

        # Mirror sweet-spot X for arm symmetry: left arm is at +X, right arm at -X
        _arm_x_sign = 1.0 if self.GRASP_ARM_NAME == 'left' else -1.0
        self.GRASP_SWEET_SPOT_X  = self.__class__.GRASP_SWEET_SPOT_X  * _arm_x_sign
        self._ik_nudge_target_x  = 0.12 * _arm_x_sign

        # Command queue for sequential voice targets
        self.command_queue       = deque()
        self.current_object_name = ''
        self.current_action      = ''
        self.target_class_id     = -1
        self._last_voice_action  = 'get'

        # Object detection state (Phase 1 - YOLO)
        self.object_world_pos      = None
        self.detection_times       = []
        self.last_detection_pos    = None
        self.latest_yolo_detection = None

        # Person detection state (Phase 2 - YOLO person)
        self.person_world_pos       = None

        # Drop command flag
        self._drop_pending = False

        # Approach state
        self.approach_start_time = None

        # Centering state
        self.center_start_time = None
        self._center_last_log = 0.0

        # Waypoint recording for return-home
        self.start_pose = None
        self.waypoints = []

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image,      '/camera/depth/image_raw',
                                 self._depth_cb,       sensor_qos)
        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 self._camera_info_cb, sensor_qos)
        self.create_subscription(Image,      '/camera/rgb/image_raw',
                                 self._rgb_cb,         sensor_qos)

        # YOLO detections (Phase 1 & 2)
        yolo_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Detection2DArray, '/objbbox',
            self._yolo_bbox_cb, yolo_qos)

        # Voice commands
        voice_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, '/user_command', self._user_command_cb, voice_qos)
        self.create_subscription(
            String, '/target_object', self._target_object_cb, voice_qos)

        # Safe-stop listener
        self._safe_stop_requested = False
        self.create_subscription(
            String, '/safe_stop', self._safe_stop_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

        # If skip_voice, queue the hardcoded command immediately
        if self.skip_voice:
            obj = (self.get_parameter('target_object')
                   .get_parameter_value().string_value)
            act = (self.get_parameter('user_command')
                   .get_parameter_value().string_value)
            self.command_queue.append((act, obj))

        # Publishers
        self.object_marker_pub = self.create_publisher(
            MarkerArray, '/object_marker', 10)
        self.cmd_vel_pub       = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self.pan_tilt_pub      = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10)

        # Grasp/arm infrastructure
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, f'/{self.GRASP_ARM_NAME}_gripper/cmd', 10)
        self.arm_pub = self.create_publisher(
            ArmCommand, f'/{self.GRASP_ARM_NAME}_arm/cmd', 10)
        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')

        self.get_logger().info('=' * 55)
        self.get_logger().info(
            'Pick-and-Bring Pipeline (YOLO)')
        if self.skip_voice:
            obj = (self.get_parameter('target_object')
                   .get_parameter_value().string_value)
            act = (self.get_parameter('user_command')
                   .get_parameter_value().string_value)
            self.get_logger().info(
                f'  Mode   : skip_voice (hardcoded: {act} {obj})')
        else:
            self.get_logger().info(
                f'  Mode   : voice commands via /target_object')
        self.get_logger().info(f'  Arm    : {self.GRASP_ARM_NAME}')
        self.get_logger().info('=' * 55)

    # ── Sensor callbacks ─────────────────────────────────────────────────────

    def _depth_cb(self, msg: Image):
        try:
            depth = self.cv_bridge.imgmsg_to_cv2(msg, '16UC1')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')
            return
        self.latest_depth       = depth
        self.latest_depth_stamp = msg.header.stamp

    def _rgb_cb(self, msg: Image):
        try:
            self.latest_rgb = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def _camera_info_cb(self, msg: CameraInfo):
        self.camera_K = np.array(msg.k).reshape(3, 3)

    # ── Detection callbacks ──────────────────────────────────────────────────

    def _yolo_bbox_cb(self, msg: Detection2DArray):
        for detection in msg.detections:
            if not detection.results:
                continue
            hyp = detection.results[0].hypothesis
            class_id = int(hyp.class_id)
            if class_id == self.target_class_id and hyp.score >= self.MIN_CONFIDENCE:
                self.latest_yolo_detection = detection
                return

    # ── Phase routing helpers ────────────────────────────────────────────────

    def _get_active_detection(self):
        """Return latest detection for the current phase (without consuming)."""
        return self.latest_yolo_detection

    def _consume_active_detection(self):
        """Return and clear latest detection for the current phase."""
        det = self.latest_yolo_detection
        self.latest_yolo_detection = None
        return det

    @property
    def _active_world_pos(self):
        if self._phase == 'object':
            return self.object_world_pos
        return self.person_world_pos

    @_active_world_pos.setter
    def _active_world_pos(self, val):
        if self._phase == 'object':
            self.object_world_pos = val
        else:
            self.person_world_pos = val

    def _pp(self, key):
        """Get phase-specific parameter."""
        return self.PHASE_PARAMS[self._phase][key]

    # ── Robot pose ────────────────────────────────────────────────────────────

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

    # ── Position estimation ──────────────────────────────────────────────────

    def _estimate_world_position(self, u, v):
        """Estimate world (odom) XY position from pixel (u, v) + depth."""
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
            self.get_logger().warn(
                f'[DEPTH] No valid depth at pixel ({u},{v}), '
                f'patch shape={patch.shape}, depth img shape={depth.shape}')
            return None
        depth_m = float(np.median(valid)) / 1000.0

        min_d = self.MIN_DEPTH_M
        if depth_m < min_d or depth_m > self.MAX_DEPTH_M:
            self.get_logger().warn(
                f'[DEPTH] depth={depth_m:.3f}m out of range '
                f'[{min_d:.2f}, {self.MAX_DEPTH_M:.2f}] at pixel ({u},{v})')
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

    def _get_position_in_base_link(self, u, v):
        """Get pixel (u, v) position in base_link frame (for arm positioning)."""
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

        if depth_m < 0.20 or depth_m > self.MAX_DEPTH_M:
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
        return pt_base

    # ── Publishing ────────────────────────────────────────────────────────────

    def publish_object_marker(self):
        if self.object_world_pos is None:
            return
        m = Marker()
        m.header.stamp       = self.get_clock().now().to_msg()
        m.header.frame_id    = 'odom'
        m.ns                 = 'target_object'
        m.id                 = 0
        m.type               = Marker.SPHERE
        m.action             = Marker.ADD
        m.pose.position.x    = self.object_world_pos[0]
        m.pose.position.y    = self.object_world_pos[1]
        m.pose.position.z    = 0.05
        m.pose.orientation.w = 1.0
        m.scale.x = 0.15;  m.scale.y = 0.15;  m.scale.z = 0.15
        m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
        ma = MarkerArray()
        ma.markers.append(m)
        self.object_marker_pub.publish(ma)

    # ── Camera tilt helper ────────────────────────────────────────────────────

    def _set_camera_tilt(self, tilt):
        self.current_tilt = float(np.clip(tilt, 0.0, 0.9))
        pt_msg = Float64MultiArray()
        pt_msg.data = [0.0, self.current_tilt]
        for _ in range(5):
            self.pan_tilt_pub.publish(pt_msg)
            rclpy.spin_once(self, timeout_sec=0.02)

    # ── Base control ──────────────────────────────────────────────────────────

    def _stop_base(self):
        self.cmd_vel_pub.publish(Twist())

    def _spin_for(self, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    # ── Safe stop ─────────────────────────────────────────────────────────────

    def _safe_stop_cb(self, msg):
        self.get_logger().warn('[SAFE STOP] Received safe_stop signal!')
        self._safe_stop_requested = True

    def _execute_safe_shutdown(self):
        self.get_logger().warn('[SAFE STOP] Executing safe shutdown...')

        for _ in range(10):
            self.cmd_vel_pub.publish(Twist())
            time.sleep(0.05)

        grip_msg = Float64MultiArray()
        grip_msg.data = [0.05]
        for _ in range(10):
            self.gripper_pub.publish(grip_msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        arm_msg = ArmCommand()
        arm_msg.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        arm_msg.duration = 3.0
        self.arm_pub.publish(arm_msg)

        self._set_camera_tilt(0.3)

        t0 = time.time()
        while time.time() - t0 < 3.5:
            self.cmd_vel_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warn('[SAFE STOP] Shutdown complete.')

    # ── Object detection (Phase 1 only) ──────────────────────────────────────

    def _check_for_object(self):
        det = self.latest_yolo_detection
        if det is None or self.camera_K is None:
            return False

        self.latest_yolo_detection = None

        u = int(det.bbox.center.position.x)
        v = int(det.bbox.center.position.y)

        pos = self._estimate_world_position(u, v)
        if pos is None:
            return False

        if self.last_detection_pos is not None:
            d = np.hypot(pos[0] - self.last_detection_pos[0],
                         pos[1] - self.last_detection_pos[1])
            if d > 1.0:
                self.detection_times = []

        now = time.time()
        self.detection_times.append(now)
        self.last_detection_pos = pos

        cutoff = now - self.DETECT_WINDOW
        self.detection_times = [t for t in self.detection_times if t >= cutoff]

        class_id = int(det.results[0].hypothesis.class_id)
        self.get_logger().info(
            f'[DETECT] YOLO class {class_id} candidate at '
            f'({pos[0]:.2f}, {pos[1]:.2f}) '
            f'count={len(self.detection_times)}/{self.DETECT_COUNT_REQ} '
            f'in {self.DETECT_WINDOW}s window')

        if len(self.detection_times) >= self.DETECT_COUNT_REQ:
            self.object_world_pos = pos
            self.get_logger().info(
                f'[DETECT] *** YOLO class {class_id} CONFIRMED at '
                f'({pos[0]:.2f}, {pos[1]:.2f}) ***')
            return True

        return False

    def _check_for_person(self):
        det = self.latest_yolo_detection
        if det is None or self.camera_K is None:
            return False

        self.latest_yolo_detection = None

        u = int(det.bbox.center.position.x)
        v = int(det.bbox.center.position.y)

        pos = self._estimate_world_position(u, v)
        if pos is None:
            return False

        self.person_world_pos = pos
        self.get_logger().info(
            f'[DETECT] YOLO person confirmed at ({pos[0]:.2f}, {pos[1]:.2f})')
        return True

    # ── Voice command callbacks ───────────────────────────────────────────

    DROP_ACTIONS = {'drop', 'release', 'let go', 'put down', 'place'}

    def _accepting_voice(self):
        return (not self.skip_voice or
                self.state in (TaskState.WAITING_FOR_COMMAND,
                               TaskState.WAITING_FOR_DROP))

    def _user_command_cb(self, msg: String):
        action = msg.data.strip().lower()
        if not action:
            return

        if self.state == TaskState.WAITING_FOR_DROP:
            if any(kw in action for kw in self.DROP_ACTIONS):
                self.get_logger().info(f'[VOICE] Drop command received: "{action}"')
                self._drop_pending = True
                return
            self._last_voice_action = action
            self.get_logger().info(f'[VOICE] Action received (while holding): {action}')
            return

        if not self._accepting_voice():
            return
        self._last_voice_action = action
        self.get_logger().info(f'[VOICE] Action received: {action}')

    def _target_object_cb(self, msg: String):
        if not self._accepting_voice():
            return
        obj = msg.data.strip().lower()
        if not obj:
            return

        action = self._last_voice_action
        self.command_queue.append((action, obj))
        self.get_logger().info(f'[VOICE] Queued command: {action} {obj}')

    def _start_next_command(self):
        if not self.command_queue:
            self.state = TaskState.WAITING_FOR_COMMAND
            self.get_logger().info(
                '[CMD] No more commands — waiting for voice input...')
            return

        action, obj_name = self.command_queue.popleft()
        self.current_action = action
        self.current_object_name = obj_name

        class_id = YOLO_CLASS_MAP.get(obj_name.lower())
        if class_id is None:
            self.get_logger().warn(
                f'[CMD] Unknown object "{obj_name}" — not in YOLO class map. '
                f'Skipping.')
            self._start_next_command()
            return

        self.target_class_id = class_id
        self._reset_detection_state()
        self._phase = 'object'

        # Record starting pose for return-home
        self.start_pose = self.get_base_pose()
        self.waypoints = []
        if self.start_pose is not None:
            self.waypoints.append((self.start_pose[0], self.start_pose[1]))
            self.get_logger().info(
                f'[NAV] Start pose recorded: '
                f'({self.start_pose[0]:.2f}, {self.start_pose[1]:.2f})')

        self.get_logger().info('=' * 55)
        self.get_logger().info(
            f'NEW COMMAND: {action} {obj_name} (YOLO class {class_id})')
        self.get_logger().info('=' * 55)

        # Tilt camera and check if object is already visible
        self.get_logger().info(
            '[CMD] Tilting camera to 45deg and checking for object...')
        self._set_camera_tilt(0.7854)
        self._spin_for(1.0)

        self.latest_yolo_detection = None
        found = False
        deadline = time.time() + 3.0
        while time.time() < deadline:
            self._spin_for(0.2)
            det = self.latest_yolo_detection
            if det is not None:
                self.latest_yolo_detection = None
                u = int(det.bbox.center.position.x)
                v = int(det.bbox.center.position.y)
                self.get_logger().info(
                    f'[CMD] YOLO detection at pixel ({u}, {v})')
                pos = self._estimate_world_position(u, v)
                if pos is not None:
                    self.object_world_pos = pos
                    self.last_detection_pos = pos
                    self.get_logger().info(
                        f'[CMD] Object at ({pos[0]:.2f}, {pos[1]:.2f})')
                else:
                    self.get_logger().info(
                        '[CMD] Depth estimate failed but YOLO sees it')
                found = True
                break

        if found:
            self.get_logger().info(
                '[CMD] Object already visible — skipping scan!')
            self._start_centering()
            return

        self.get_logger().info(
            '[CMD] Object not visible — starting scan')
        self._set_camera_tilt(0.3)
        self.scan_attempt_count = 0
        self._start_scan()

    def _reset_detection_state(self):
        self.object_world_pos = None
        self.detection_times = []
        self.last_detection_pos = None
        self.latest_yolo_detection = None
        self.person_world_pos = None

    # ── Unified scan/center/approach states ──────────────────────────────────

    def _start_scan(self):
        self.scan_accumulated  = 0.0
        self.scan_last_heading = None
        self.scan_settle_start = None
        self.scan_attempt_count += 1
        self._set_camera_tilt(self._pp('scan_tilt'))

        if self._phase == 'object':
            self.state = TaskState.SCANNING_OBJECT
        else:
            self.state = TaskState.SCANNING_PERSON

        phase_label = 'OBJECT' if self._phase == 'object' else 'PERSON'
        self.get_logger().info(
            f'[SCAN-{phase_label}] Starting 360 rotation scan '
            f'(attempt {self.scan_attempt_count}/{self.MAX_SCAN_ATTEMPTS})...')

    def _state_scanning(self):
        phase_label = 'OBJECT' if self._phase == 'object' else 'PERSON'

        # Check for detection mid-scan
        if self._phase == 'object':
            if self.object_world_pos is not None:
                self._stop_base()
                self.get_logger().info(
                    f'[SCAN-{phase_label}] Object spotted mid-scan — centering.')
                self._start_centering()
                return
        else:
            # Person: check YOLO for person class
            det = self.latest_yolo_detection
            if det is not None:
                self.latest_yolo_detection = None
                u = int(det.bbox.center.position.x)
                v = int(det.bbox.center.position.y)
                self.get_logger().info(
                    f'[SCAN-PERSON] Person detected at pixel ({u}, {v})')
                pos = self._estimate_world_position(u, v)
                if pos is not None:
                    self.person_world_pos = pos
                self._stop_base()
                self._start_centering()
                return

        if self.scan_settle_start is not None:
            if time.time() - self.scan_settle_start >= self.SCAN_SETTLE_TIME:
                if self.scan_attempt_count >= self.MAX_SCAN_ATTEMPTS:
                    self.get_logger().warn(
                        f'[SCAN-{phase_label}] No detection found after '
                        f'{self.MAX_SCAN_ATTEMPTS} scans — giving up.')
                    if self._phase == 'object':
                        self.state = TaskState.OBJECT_REACHED
                    else:
                        self.state = TaskState.TASK_COMPLETE
                else:
                    self.get_logger().info(
                        f'[SCAN-{phase_label}] No detection found — re-scanning...')
                    self._start_scan()
            return

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
                f'[SCAN-{phase_label}] 360 done '
                f'({np.degrees(self.scan_accumulated):.0f} accumulated). '
                f'Settling {self.SCAN_SETTLE_TIME}s...')
        else:
            cmd = Twist()
            cmd.angular.z = self.SCAN_ANGULAR_VEL
            self.cmd_vel_pub.publish(cmd)

    def _start_centering(self):
        self.center_start_time = time.time()
        if self._phase == 'object':
            self.state = TaskState.CENTERING_OBJECT
        else:
            self.state = TaskState.CENTERING_PERSON
        phase_label = 'OBJECT' if self._phase == 'object' else 'PERSON'
        self.get_logger().info(
            f'[CENTER-{phase_label}] Rotating to center detection in camera...')

    def _state_centering(self):
        now = time.time()
        phase_label = 'OBJECT' if self._phase == 'object' else 'PERSON'
        thresh_h = self._pp('center_pixel_thresh')
        thresh_v = self._pp('center_pixel_thresh_v')

        if now - self.center_start_time > self.CENTER_TIMEOUT:
            self._stop_base()
            if self._active_world_pos is not None:
                self.get_logger().warn(
                    f'[CENTER-{phase_label}] Timeout — known pos, approaching anyway.')
                self.approach_start_time = time.time()
                if self._phase == 'object':
                    self.state = TaskState.APPROACHING_OBJECT
                else:
                    self.state = TaskState.APPROACHING_PERSON
            else:
                self.get_logger().warn(
                    f'[CENTER-{phase_label}] Timeout — re-scanning.')
                self._start_scan()
            return

        pose = self.get_base_pose()
        if pose is None:
            return
        bx, by, btheta = pose

        det = self._get_active_detection()
        if det is not None:
            self.latest_yolo_detection = None

            u = int(det.bbox.center.position.x)
            v = int(det.bbox.center.position.y)

            if self.latest_depth is None:
                return
            img_h, img_w = self.latest_depth.shape
            img_center_x = img_w / 2.0
            img_center_y = img_h / 2.0
            offset_x = u - img_center_x
            offset_y = v - img_center_y

            # Update world position from fresh detection
            new_pos = self._estimate_world_position(u, v)
            if new_pos is not None:
                if self._active_world_pos is not None:
                    shift = np.hypot(
                        new_pos[0] - self._active_world_pos[0],
                        new_pos[1] - self._active_world_pos[1])
                    if shift < 1.5:
                        self._active_world_pos = new_pos
                        self.last_detection_pos = new_pos
                else:
                    self._active_world_pos = new_pos
                    self.last_detection_pos = new_pos

            # Adjust camera tilt vertically
            if abs(offset_y) > thresh_v:
                tilt_adj = offset_y / (img_h / 2.0) * self.CENTER_TILT_STEP
                self._set_camera_tilt(self.current_tilt + tilt_adj)

            h_centered = abs(offset_x) < thresh_h
            v_centered = abs(offset_y) < thresh_v

            if h_centered and v_centered:
                self._stop_base()
                self.get_logger().info(
                    f'[CENTER-{phase_label}] Centered (h={offset_x:.0f}px, '
                    f'v={offset_y:.0f}px) — approaching.')
                self.approach_start_time = time.time()
                if self._phase == 'object':
                    self.state = TaskState.APPROACHING_OBJECT
                else:
                    self.state = TaskState.APPROACHING_PERSON
                return

            # Rotate to reduce horizontal offset
            if not h_centered:
                cmd = Twist()
                frac = min(abs(offset_x) / (img_w / 2.0), 1.0)
                speed = self.CENTER_ANGULAR_VEL * (0.2 + 0.8 * frac * frac)
                cmd.angular.z = -speed if offset_x > 0 else speed
                self.cmd_vel_pub.publish(cmd)

            if now - self._center_last_log > 1.0:
                self._center_last_log = now
                self.get_logger().info(
                    f'[CENTER-{phase_label}] h_offset={offset_x:.0f}px, '
                    f'v_offset={offset_y:.0f}px, tilt={self.current_tilt:.2f}')
        else:
            # No detection: rotate toward known world position
            if self._active_world_pos is None:
                elapsed = now - self.center_start_time
                if elapsed < 3.0:
                    if now - self._center_last_log > 1.0:
                        self._center_last_log = now
                        self.get_logger().info(
                            f'[CENTER-{phase_label}] No world pos yet, waiting...')
                    return
                self._stop_base()
                self.get_logger().warn(
                    f'[CENTER-{phase_label}] Lost detection — re-scanning.')
                self._start_scan()
                return

            ox, oy = self._active_world_pos
            bearing = np.arctan2(oy - by, ox - bx)
            actual_heading = btheta - np.pi / 2
            heading_error = self._normalize_angle(bearing - actual_heading)

            if abs(heading_error) < np.radians(5):
                self._stop_base()
                if now - self._center_last_log > 1.0:
                    self._center_last_log = now
                    self.get_logger().info(
                        f'[CENTER-{phase_label}] Facing target, waiting for detection...')
                return

            cmd = Twist()
            cmd.angular.z = float(np.clip(
                1.0 * heading_error, -self.CENTER_ANGULAR_VEL,
                self.CENTER_ANGULAR_VEL))
            self.cmd_vel_pub.publish(cmd)

            if now - self._center_last_log > 1.0:
                self._center_last_log = now
                self.get_logger().info(
                    f'[CENTER-{phase_label}] bearing err={np.degrees(heading_error):.0f}deg, '
                    f'rotating toward ({ox:.2f},{oy:.2f})')

    def _state_approaching(self):
        now = time.time()
        phase_label = 'OBJECT' if self._phase == 'object' else 'PERSON'
        stop_dist = self._pp('approach_stop_dist')
        linear_speed = self._pp('approach_linear_speed')
        thresh_v = self._pp('center_pixel_thresh_v')

        if now - self.approach_start_time > self.APPROACH_TIMEOUT:
            self._stop_base()
            self.get_logger().warn(f'[APPROACH-{phase_label}] Timeout.')
            if self._phase == 'object':
                self.state = TaskState.OBJECT_REACHED
            else:
                self._transition_to_waiting_for_drop()
            return

        pose = self.get_base_pose()
        if pose is None:
            return
        bx, by, btheta = pose
        actual_heading = btheta - np.pi / 2

        # Check distance to target via world position
        if self._active_world_pos is not None:
            ox, oy = self._active_world_pos
            dist_to_target = np.hypot(ox - bx, oy - by)
            if dist_to_target < stop_dist:
                self._stop_base()
                self.get_logger().info(
                    f'[APPROACH-{phase_label}] Reached target! dist={dist_to_target:.2f}m')
                if self._phase == 'object':
                    self.state = TaskState.OBJECT_REACHED
                else:
                    self._transition_to_waiting_for_drop()
                return

        det = self._get_active_detection()
        cmd = Twist()

        if det is not None:
            self.latest_yolo_detection = None

            u = int(det.bbox.center.position.x)
            v = int(det.bbox.center.position.y)

            if self.latest_depth is not None:
                img_h, img_w = self.latest_depth.shape
                img_center_x = img_w / 2.0
                img_center_y = img_h / 2.0
                offset_x = u - img_center_x

                # Steer proportionally to horizontal pixel offset
                angular = -self.APPROACH_ANGULAR_GAIN * offset_x
                cmd.angular.z = float(np.clip(
                    angular, -self.APPROACH_MAX_ANGULAR, self.APPROACH_MAX_ANGULAR))

                # Adjust tilt to keep target vertically centered
                offset_y = v - img_center_y
                if abs(offset_y) > thresh_v:
                    tilt_adj = offset_y / (img_h / 2.0) * self.APPROACH_TILT_STEP
                    self._set_camera_tilt(self.current_tilt + tilt_adj)

            # Refine world position
            new_pos = self._estimate_world_position(u, v)
            if new_pos is not None and self._active_world_pos is not None:
                shift = np.hypot(
                    new_pos[0] - self._active_world_pos[0],
                    new_pos[1] - self._active_world_pos[1])
                if shift < 1.5:
                    self._active_world_pos = new_pos
                    self.last_detection_pos = new_pos

            # Check depth directly from detection center
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
                    if depth_m < stop_dist:
                        self._stop_base()
                        self.get_logger().info(
                            f'[APPROACH-{phase_label}] Depth={depth_m:.2f}m < '
                            f'{stop_dist}m — reached!')
                        if self._phase == 'object':
                            self.state = TaskState.OBJECT_REACHED
                        else:
                            self._transition_to_waiting_for_drop()
                        return
        else:
            # No detection: fall back to world heading
            if self._active_world_pos is not None:
                ox, oy = self._active_world_pos
                bearing = np.arctan2(oy - by, ox - bx)
                heading_error = self._normalize_angle(bearing - actual_heading)
                cmd.angular.z = float(np.clip(
                    1.0 * heading_error,
                    -self.APPROACH_MAX_ANGULAR,
                    self.APPROACH_MAX_ANGULAR))

        cmd.linear.x = linear_speed
        self.cmd_vel_pub.publish(cmd)

        if not hasattr(self, '_approach_last_log') or now - self._approach_last_log > 2.0:
            self._approach_last_log = now
            dist_str = ''
            if self._active_world_pos is not None:
                ox, oy = self._active_world_pos
                dist_str = f' dist={np.hypot(ox-bx, oy-by):.2f}m'
            det_str = 'VISUAL' if det is not None else 'world-heading'
            self.get_logger().info(
                f'[APPROACH-{phase_label}] {det_str}{dist_str} '
                f'pos=({bx:.2f},{by:.2f}) tilt={self.current_tilt:.2f}')

    # ── State: OBJECT_REACHED ────────────────────────────────────────────────

    def _state_object_reached(self):
        self._stop_base()
        if self.object_world_pos is not None:
            self.publish_object_marker()

        name = self.current_object_name.upper() or 'TARGET'
        self.get_logger().info('=' * 55)
        if self.object_world_pos is not None:
            ox, oy = self.object_world_pos
            self.get_logger().info(
                f'FOUND {name} at ({ox:.2f}, {oy:.2f})')
        else:
            self.get_logger().info(
                f'{name} reached (no world pos, but close enough)')
        self.get_logger().info('=' * 55)

        # This pipeline always does full pick-and-bring (get → grasp → person → deliver).
        # Accept any action (get, place, etc.) since multi-step voice commands
        # may arrive with "place" overwriting "get" due to rapid publishing.
        self.get_logger().info(
            f'[COMPLETE] Action is "{self.current_action}" — transitioning to GRASPING')
        self.state = TaskState.GRASPING

    # ── Grasping (Phase 1) ─────────────────────────────────────────────────

    def _state_grasping(self):
        self.get_logger().info('=' * 55)
        self.get_logger().info('[GRASP] Starting grasp pipeline...')

        if not self.plan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                '[GRASP] /plan_to_target service not available!')
            self._start_next_command()
            return

        for attempt in range(1, self.GRASP_MAX_ATTEMPTS + 1):
            self.get_logger().info(
                f'[GRASP] === Attempt {attempt}/{self.GRASP_MAX_ATTEMPTS} ===')

            if attempt > 1:
                self._go_home()
                # Alternate nudge directions between retries
                nudge_patterns = [
                    (0.04, -0.08, 'forward-right'),
                    (-0.03, 0.08, 'back-left'),
                    (0.05, 0.0, 'forward'),
                ]
                vx, wz, label = nudge_patterns[(attempt - 2) % len(nudge_patterns)]
                self.get_logger().info(
                    f'[GRASP] Retry nudge: {label} (vx={vx}, wz={wz}) for 0.6s')
                cmd = Twist()
                cmd.linear.x = vx
                cmd.angular.z = wz
                end = time.time() + 0.6
                while time.time() < end and rclpy.ok():
                    self.cmd_vel_pub.publish(cmd)
                    rclpy.spin_once(self, timeout_sec=0.05)
                self._stop_base()
                self._spin_for(0.5)

            if self._grasp_attempt():
                return  # success — transitions to person finding

        self.get_logger().error(
            f'[GRASP] All {self.GRASP_MAX_ATTEMPTS} attempts failed')
        self._go_home()
        self._start_next_command()

    def _grasp_detect_and_center(self):
        """Detect object, center it in frame, and return (cx, cy, bbox_w, bbox_h) or None."""
        self.get_logger().info(
            f'[GRASP] Looking for object at current tilt={self.current_tilt:.2f}...')
        det = None
        self.latest_yolo_detection = None
        self._spin_for(0.5)
        for _ in range(15):
            self._spin_for(0.2)
            if self.latest_yolo_detection is not None:
                det = self.latest_yolo_detection
                break

        if det is None:
            self.get_logger().error(
                '[GRASP] No YOLO detection at current camera pose')
            return None

        # Center the object in the camera frame by adjusting tilt
        self.get_logger().info('[GRASP] Centering object in frame...')
        center_deadline = time.time() + 5.0
        while time.time() < center_deadline:
            self._spin_for(0.2)
            det = self.latest_yolo_detection
            if det is None:
                continue
            self.latest_yolo_detection = None

            v = int(det.bbox.center.position.y)
            if self.latest_depth is None:
                continue
            img_h = self.latest_depth.shape[0]
            img_center_y = img_h / 2.0
            offset_y = v - img_center_y

            if abs(offset_y) < self._pp('center_pixel_thresh_v'):
                self.get_logger().info(
                    f'[GRASP] Object centered vertically '
                    f'(offset={offset_y:.0f}px, tilt={self.current_tilt:.2f})')
                break

            tilt_adj = offset_y / (img_h / 2.0) * self.CENTER_TILT_STEP
            self._set_camera_tilt(self.current_tilt + tilt_adj)

        # Get a fresh detection after centering
        self.latest_yolo_detection = None
        self._spin_for(0.5)
        for _ in range(10):
            self._spin_for(0.2)
            if self.latest_yolo_detection is not None:
                break
        det = self.latest_yolo_detection
        if det is None:
            self.get_logger().error('[GRASP] Lost object after centering')
            return None

        self.latest_yolo_detection = None
        return (int(det.bbox.center.position.x),
                int(det.bbox.center.position.y),
                float(det.bbox.size_x),
                float(det.bbox.size_y))

    def _grasp_compute_from_detection(self, cx, cy, bbox_w, bbox_h):
        """From a detection, compute grasp/pre-grasp poses. Returns (grasp_pose, pre_grasp_pose, analysis, info) or None."""
        self.get_logger().info(
            f'[GRASP] YOLO detection at pixel ({cx}, {cy}), '
            f'bbox {bbox_w:.0f}x{bbox_h:.0f}')

        depth_roi, u_off, v_off = self._grasp_crop_depth(
            cx, cy, bbox_w, bbox_h)
        if depth_roi is None:
            self.get_logger().error('[GRASP] Failed to crop depth image')
            return None

        pts_base = self._grasp_depth_to_points(depth_roi, u_off, v_off)
        if pts_base is None:
            self.get_logger().error(
                f'[GRASP] Insufficient points in depth ROI '
                f'(need >= {self.GRASP_MIN_POINTS})')
            return None

        self.get_logger().info(
            f'[GRASP] Point cloud: {len(pts_base)} points')

        analysis = self._grasp_analyze_object(pts_base)
        centroid = analysis['centroid']
        self.get_logger().info(
            f'[GRASP] Centroid: ({centroid[0]:.3f}, {centroid[1]:.3f}, '
            f'{centroid[2]:.3f}), z_top={analysis["z_top"]:.3f}m')

        result = self._grasp_compute_poses(analysis)
        if result is None:
            self.get_logger().error('[GRASP] Pose computation failed')
            return None
        grasp_pose, pre_grasp_pose, info = result
        self.get_logger().info(f'[GRASP] Params: {info}')
        return grasp_pose, pre_grasp_pose, analysis, info

    def _grasp_ik_nudge(self, grasp_x, grasp_y):
        """Compute a nudge direction to move the grasp point into the IK sweet spot.
        Based on IK workspace scan: best zone is x=±0.12, y=-0.20 to -0.30.
        target_x sign is arm-dependent (left=+, right=-)."""
        # Target center of the IK-friendly zone (arm-specific x, set in __init__)
        target_x = self._ik_nudge_target_x
        target_y = -0.24  # center of reliable IK y range

        dx = target_x - grasp_x   # positive = need to move object forward (robot backward)
        dy = target_y - grasp_y   # positive = need to move object right (robot turn right)

        # Robot forward (linear.x>0) → object x decreases in base_link
        # Robot turn left (angular.z>0) → object y decreases (shifts right)
        cmd = Twist()
        cmd.linear.x = float(np.clip(-dx * 0.5, -0.06, 0.06))
        cmd.angular.z = float(np.clip(-dy * 0.8, -0.15, 0.15))

        dist = np.hypot(dx, dy)
        drive_time = float(np.clip(dist / 0.05, 0.3, 1.5))

        self.get_logger().info(
            f'[GRASP] IK nudge: grasp=({grasp_x:.3f}, {grasp_y:.3f}), '
            f'target=({target_x:.3f}, {target_y:.3f}), '
            f'cmd=(vx={cmd.linear.x:.3f}, wz={cmd.angular.z:.3f}), '
            f'drive={drive_time:.1f}s')

        deadline = time.time() + drive_time
        while time.time() < deadline and rclpy.ok():
            self.cmd_vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
        self._stop_base()
        self._spin_for(0.5)

    def _grasp_attempt(self):
        """Single grasp attempt with multiple nudge-and-retry cycles. Returns True on success."""
        self._save_grasp_snapshot()

        detection = self._grasp_detect_and_center()
        if detection is None:
            return False
        cx, cy, bbox_w, bbox_h = detection

        computed = self._grasp_compute_from_detection(cx, cy, bbox_w, bbox_h)
        if computed is None:
            return False
        grasp_pose, pre_grasp_pose, analysis, info = computed

        # Initial sweet-spot nudge
        grasp_x = float(grasp_pose.position.x)
        grasp_y = float(grasp_pose.position.y)
        if self._grasp_reposition_for_sweet_spot(grasp_x, grasp_y):
            self.get_logger().info('[GRASP] Re-detecting after sweet-spot nudge...')
            detection = self._grasp_detect_and_center()
            if detection is None:
                return False
            cx, cy, bbox_w, bbox_h = detection
            computed = self._grasp_compute_from_detection(cx, cy, bbox_w, bbox_h)
            if computed is None:
                return False
            grasp_pose, pre_grasp_pose, analysis, info = computed

        # Try IK — if it fails, nudge and retry up to GRASP_MAX_IK_NUDGES times
        use_orientation = True
        ok, msg = self._try_ik(grasp_pose, use_orientation=True)

        if not ok:
            self.get_logger().warn(f'[GRASP] IK failed: {msg}')
            for nudge_i in range(1, self.GRASP_MAX_IK_NUDGES + 1):
                grasp_x = float(grasp_pose.position.x)
                grasp_y = float(grasp_pose.position.y)
                self.get_logger().info(
                    f'[GRASP] IK nudge {nudge_i}/{self.GRASP_MAX_IK_NUDGES} '
                    f'— repositioning robot...')
                self._grasp_ik_nudge(grasp_x, grasp_y)

                # Re-detect and recompute after nudge
                detection = self._grasp_detect_and_center()
                if detection is None:
                    self.get_logger().warn(
                        f'[GRASP] Lost object after nudge {nudge_i}')
                    continue
                cx, cy, bbox_w, bbox_h = detection
                computed = self._grasp_compute_from_detection(
                    cx, cy, bbox_w, bbox_h)
                if computed is None:
                    continue
                grasp_pose, pre_grasp_pose, analysis, info = computed

                ok, msg = self._try_ik(grasp_pose, use_orientation=True)
                if ok:
                    self.get_logger().info(
                        f'[GRASP] IK succeeded after nudge {nudge_i}!')
                    break
                self.get_logger().warn(
                    f'[GRASP] IK still failed after nudge {nudge_i}: {msg}')

            if not ok:
                self.get_logger().error(
                    '[GRASP] All IK nudge attempts failed — object may be unreachable')
                return False

        self.get_logger().info('[GRASP] Opening gripper')
        self._set_gripper(0.05)

        self.get_logger().info('[GRASP] Moving to pre-grasp')
        if not self._grasp_plan_and_execute(
                pre_grasp_pose, 'pre-grasp', use_orientation,
                keep_gripper_open=True):
            self.get_logger().error('[GRASP] Pre-grasp failed')
            return False

        self.get_logger().info('[GRASP] Re-opening gripper at pre-grasp')
        self._set_gripper(0.05)
        self._spin_for(self.GRASP_SETTLE_TIME)

        self.get_logger().info('[GRASP] Descending to grasp')
        if not self._grasp_plan_and_execute(
                grasp_pose, 'grasp', use_orientation,
                keep_gripper_open=True):
            self.get_logger().error('[GRASP] Descent failed')
            return False

        self._spin_for(self.GRASP_SETTLE_TIME)

        self.get_logger().info('[GRASP] Closing gripper (firm)')
        self._close_gripper_firm()

        self.get_logger().info('[GRASP] Lifting object')
        lift_pose = Pose()
        lift_pose.position.x = grasp_pose.position.x
        lift_pose.position.y = grasp_pose.position.y
        lift_pose.position.z = grasp_pose.position.z + self.GRASP_LIFT_HEIGHT
        lift_pose.orientation = grasp_pose.orientation
        if not self._grasp_plan_and_execute(
                lift_pose, 'lift', use_orientation):
            self.get_logger().warn(
                '[GRASP] Lift failed, continuing anyway')

        # Raise object high (trophy pose)
        self.get_logger().info('[GRASP] Raising object high...')
        trophy_pose = Pose()
        trophy_pose.position.x = float(grasp_pose.position.x)
        trophy_pose.position.y = float(grasp_pose.position.y)
        trophy_pose.position.z = 0.40
        trophy_pose.orientation = grasp_pose.orientation
        if not self._grasp_plan_and_execute(
                trophy_pose, 'trophy', use_orientation):
            trophy_pose.position.z = 0.35
            if not self._grasp_plan_and_execute(
                    trophy_pose, 'trophy-lower', use_orientation):
                self.get_logger().warn(
                    '[GRASP] Trophy pose failed, staying at lift height')

        self.get_logger().info('=' * 55)
        self.get_logger().info('[GRASP] Grasp complete — holding object!')
        self.get_logger().info('=' * 55)

        # Retract arm to overhead holding pose
        self._retract_holding()

        # Transition to Phase 2: find person
        self._transition_to_person_finding()
        return True

    def _transition_to_person_finding(self):
        """After successful grasp, start scanning for a person."""
        self._phase = 'person'
        self.target_class_id = YOLO_CLASS_MAP['person']  # class 0
        self.person_world_pos = None
        self.latest_yolo_detection = None

        self.get_logger().info('=' * 55)
        self.get_logger().info(
            f'[TRANSITION] Phase 2: Finding person to deliver '
            f'{self.current_object_name}')
        self.get_logger().info('=' * 55)

        self.scan_attempt_count = 0
        self._start_scan()

    def _transition_to_waiting_for_drop(self):
        """Enter WAITING_FOR_DROP: raise arm to presentation pose."""
        self.get_logger().info('=' * 55)
        self.get_logger().info('[DELIVER] Person reached — raising arm for delivery')
        self.get_logger().info('[DELIVER] Say "drop" or "release" to release object')
        self.get_logger().info('=' * 55)

        msg = ArmCommand()
        msg.joint_positions = list(PRESENT_POSE)
        msg.duration = 2.0
        self.arm_pub.publish(msg)

        # Keep gripper closed while moving to presentation pose
        grip_msg = Float64MultiArray()
        grip_msg.data = [1.0]
        deadline = time.time() + 2.5
        while time.time() < deadline:
            self.gripper_pub.publish(grip_msg)
            rclpy.spin_once(self, timeout_sec=0.1)

        self._drop_pending = False
        self.state = TaskState.WAITING_FOR_DROP

    # ── State: WAITING_FOR_DROP ──────────────────────────────────────────────

    def _state_waiting_for_drop(self):
        """Wait for a voice command to release the object."""
        if not self._drop_pending:
            return

        self.get_logger().info('[DROP] Drop command received — releasing object')

        # Open gripper
        self._set_gripper(0.0)
        self._spin_for(1.0)

        # Return arm to sleep pose
        self.get_logger().info('[DROP] Returning arm to sleep pose')
        self._go_home(duration=3.0)

        self._set_camera_tilt(0.3)

        self.get_logger().info('=' * 55)
        self.get_logger().info('[DROP] Object released. Task complete!')
        self.get_logger().info('=' * 55)

        self._drop_pending = False
        self.state = TaskState.TASK_COMPLETE

    # ── Grasp helper methods ─────────────────────────────────────────────────

    def _save_grasp_snapshot(self):
        self._spin_for(0.3)
        if self.latest_rgb is None:
            self.get_logger().warn('[GRASP] No RGB image available for snapshot')
            return
        snap_dir = os.path.expanduser('~/grasp_snapshots')
        os.makedirs(snap_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(snap_dir, f'grasp_{stamp}.jpg')
        import cv2
        cv2.imwrite(path, self.latest_rgb)
        self.get_logger().info(f'[GRASP] Snapshot saved: {path}')

    def _grasp_reposition_for_sweet_spot(self, obj_x, obj_y):
        dx = obj_x - self.GRASP_SWEET_SPOT_X
        dy = obj_y - self.GRASP_SWEET_SPOT_Y
        dist = np.hypot(dx, dy)

        if dist <= self.GRASP_SWEET_SPOT_RADIUS:
            self.get_logger().info(
                f'[GRASP] Object at ({obj_x:.3f}, {obj_y:.3f}) '
                f'is within sweet spot (dist={dist:.3f}m)')
            return False

        self.get_logger().info(
            f'[GRASP] Object at ({obj_x:.3f}, {obj_y:.3f}) '
            f'is {dist:.3f}m from sweet spot '
            f'({self.GRASP_SWEET_SPOT_X:.3f}, {self.GRASP_SWEET_SPOT_Y:.3f})')

        cmd = Twist()
        drive_time = 1.0

        if abs(dy) > 0.03:
            cmd.angular.z = float(np.clip(dy * 0.8, -0.15, 0.15))
        if abs(dx) > 0.03:
            cmd.linear.x = float(np.clip(dx * 0.5, -0.05, 0.05))

        self.get_logger().info(
            f'[GRASP] Nudging: vx={cmd.linear.x:.3f}, '
            f'wz={cmd.angular.z:.3f} for {drive_time:.1f}s')

        deadline = time.time() + drive_time
        while time.time() < deadline and rclpy.ok():
            self.cmd_vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
        self._stop_base()
        self._spin_for(0.5)

        return True

    def _grasp_get_tf_matrix(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0))
            t = transform.transform.translation
            q = transform.transform.rotation
            rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
            mat = np.eye(4, dtype=np.float32)
            mat[:3, :3] = rot.as_matrix().astype(np.float32)
            mat[:3, 3] = [t.x, t.y, t.z]
            return mat
        except Exception as e:
            self.get_logger().error(f'[GRASP] TF lookup failed: {e}')
            return None

    def _grasp_crop_depth(self, cx, cy, bbox_w, bbox_h):
        if self.latest_depth is None:
            return None, None, None
        h, w = self.latest_depth.shape
        half_w = int(bbox_w * self.GRASP_BBOX_PAD / 2)
        half_h = int(bbox_h * self.GRASP_BBOX_PAD / 2)
        u0 = max(0, cx - half_w)
        v0 = max(0, cy - half_h)
        u1 = min(w, cx + half_w)
        v1 = min(h, cy + half_h)
        return self.latest_depth[v0:v1, u0:u1], u0, v0

    def _grasp_depth_to_points(self, depth_roi, u_offset, v_offset):
        if self.camera_K is None:
            return None

        fx = self.camera_K[0, 0]
        fy = self.camera_K[1, 1]
        cx = self.camera_K[0, 2]
        cy = self.camera_K[1, 2]

        depth_m = depth_roi.astype(np.float32) / 1000.0
        roi_h, roi_w = depth_m.shape

        u_grid, v_grid = np.meshgrid(
            np.arange(roi_w) + u_offset,
            np.arange(roi_h) + v_offset)

        mask = ((depth_m > self.GRASP_MIN_DEPTH_M) &
                (depth_m < self.GRASP_MAX_DEPTH_M))
        z = depth_m[mask]
        u = u_grid[mask].astype(np.float32)
        v = v_grid[mask].astype(np.float32)

        if len(z) < self.GRASP_MIN_POINTS:
            return None

        x_cam = (u - cx) * z / fx
        y_cam = (v - cy) * z / fy
        points_cam = np.stack([x_cam, y_cam, z], axis=1)

        tf_mat = self._grasp_get_tf_matrix(
            'base_link', 'camera_color_optical_frame')
        if tf_mat is None:
            return None

        ones = np.ones((len(points_cam), 1), dtype=np.float32)
        pts_h = np.hstack([points_cam, ones])
        pts_base = (tf_mat @ pts_h.T).T[:, :3]

        pts_base = pts_base[pts_base[:, 2] > self.GRASP_TABLE_Z_MIN]

        if len(pts_base) > self.GRASP_MIN_POINTS:
            median_z = np.median(pts_base[:, 2])
            z_mask = (np.abs(pts_base[:, 2] - median_z)
                      < self.GRASP_OUTLIER_Z_THRESH)
            pts_base = pts_base[z_mask]

        if len(pts_base) < self.GRASP_MIN_POINTS:
            return None

        floor_z, obj_pts = _ransac_floor_separate(
            pts_base, distance_thresh=FLOOR_MARGIN,
            min_pts=self.GRASP_MIN_POINTS)

        if len(obj_pts) < self.GRASP_MIN_POINTS:
            self.get_logger().warn(
                f'[GRASP] Floor filter removed too many points '
                f'({len(pts_base)} -> {len(obj_pts)}), using all points')
            return pts_base

        self.get_logger().info(
            f'[GRASP] RANSAC floor filter: {len(pts_base)} -> {len(obj_pts)} pts, '
            f'floor_z={floor_z:.3f}m')
        return obj_pts

    def _grasp_analyze_object(self, points_base):
        centroid = np.mean(points_base, axis=0)
        z_top = float(np.max(points_base[:, 2]))

        pts_xy = points_base[:, :2]
        centered = pts_xy - centroid[:2]

        best_yaw = 0.0
        best_clearance = -1.0
        best_width = float('inf')
        best_center_offset = 0.0
        max_width = 0.0

        for angle_deg in range(0, 180, 5):
            angle = np.radians(angle_deg)
            finger_dir = np.array([np.cos(angle), np.sin(angle)])
            projections = centered @ finger_dir
            proj_min = float(projections.min())
            proj_max = float(projections.max())
            width = proj_max - proj_min

            if width > max_width:
                max_width = width

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

        self.get_logger().info(
            f'[GRASP] Max-clearance sweep: '
            f'yaw={np.degrees(best_yaw):.0f}deg, '
            f'grip={best_width*1000:.0f}mm, '
            f'clearance={best_clearance*1000:.1f}mm, '
            f'center_offset={best_center_offset*1000:.1f}mm')

        return {
            'centroid': centroid,
            'grasp_center': grasp_center,
            'grasp_yaw': best_yaw,
            'grip_width': best_width,
            'min_clearance': best_clearance,
            'max_width': max_width,
            'z_top': z_top,
            'num_points': len(points_base),
        }

    def _grasp_compute_poses(self, analysis):
        grasp_center = analysis.get('grasp_center', analysis['centroid'])
        grasp_yaw = analysis['grasp_yaw']
        grip_width = analysis['grip_width']

        # The gripper is 180°-symmetric, so yaw and yaw±π produce identical grasps.
        # _grasp_analyze_object returns yaw in [0, π), but yaw > π/2 means the arm
        # would rotate far from neutral.  Wrap to (-π/2, π/2] so forearm_roll always
        # takes the short path (e.g. yaw=170° → -10° instead of +170°).
        if grasp_yaw > np.pi / 2:
            grasp_yaw -= np.pi

        if grip_width > GRIPPER_MAX_OPENING_M:
            self.get_logger().warn(
                f'[GRASP] Width {grip_width*1000:.0f}mm > '
                f'{GRIPPER_MAX_OPENING_M*1000:.0f}mm max '
                f'(proceeding anyway)')

        qw, qx, qy, qz = yaw_to_grasp_quaternion(grasp_yaw)

        grasp_x = float(grasp_center[0]) + self.GRASP_X_OFFSET
        grasp_y = float(grasp_center[1]) + self.GRASP_Y_OFFSET
        grasp_z = analysis['z_top'] + self.GRASP_Z_OFFSET

        grasp_pose = Pose()
        grasp_pose.position.x = grasp_x
        grasp_pose.position.y = grasp_y
        grasp_pose.position.z = float(grasp_z)
        grasp_pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

        pre_grasp_pose = Pose()
        pre_grasp_pose.position.x = grasp_x
        pre_grasp_pose.position.y = grasp_y
        pre_grasp_pose.position.z = float(grasp_z + self.GRASP_PRE_HEIGHT)
        pre_grasp_pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

        info = (f'yaw={np.degrees(grasp_yaw):.0f}deg, '
                f'xyz=({grasp_x:.3f}, {grasp_y:.3f}, {grasp_z:.3f})m, '
                f'offsets=({self.GRASP_X_OFFSET*1000:.0f}, '
                f'{self.GRASP_Y_OFFSET*1000:.0f}, '
                f'{self.GRASP_Z_OFFSET*1000:.0f})mm, '
                f'width={grip_width*1000:.0f}mm')

        return grasp_pose, pre_grasp_pose, info

    def _try_ik(self, pose, use_orientation=True):
        req = PlanToTarget.Request()
        req.arm_name = self.GRASP_ARM_NAME
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = False
        req.duration = self.GRASP_MOVE_DURATION
        req.max_condition_number = 500.0

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done() or future.exception():
            return False, 'service call timed out'

        result = future.result()
        return result.success, result.message

    def _grasp_plan_and_execute(self, pose, label, use_orientation=True,
                               keep_gripper_open=False):
        req = PlanToTarget.Request()
        req.arm_name = self.GRASP_ARM_NAME
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = True
        req.duration = self.GRASP_MOVE_DURATION
        req.max_condition_number = 500.0

        self.get_logger().info(
            f'  [{label}] Planning + executing...')

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=15.0)
        if not future.done() or future.exception():
            self.get_logger().error(
                f'  [{label}] Service call timed out')
            return False

        result = future.result()
        if result.success:
            if result.executed:
                if keep_gripper_open:
                    grip_msg = Float64MultiArray()
                    grip_msg.data = [0.05]
                    deadline = time.time() + self.GRASP_MOVE_DURATION + 0.5
                    while time.time() < deadline:
                        self.gripper_pub.publish(grip_msg)
                        rclpy.spin_once(self, timeout_sec=0.1)
                else:
                    time.sleep(self.GRASP_MOVE_DURATION + 0.5)
            return True

        self.get_logger().warn(
            f'  [{label}] Failed: {result.message}')
        return False

    # ── Arm/gripper helpers ──────────────────────────────────────────────────

    def _set_gripper(self, position):
        msg = Float64MultiArray()
        msg.data = [position]
        for _ in range(10):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(0.5)

    def _close_gripper_firm(self):
        msg = Float64MultiArray()
        msg.data = [1.0]
        for _ in range(self.GRASP_GRIPPER_CLOSE_REPEATS):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(self.GRASP_GRIPPER_CLOSE_WAIT)
        for _ in range(10):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def _go_home(self, duration=3.0):
        self.get_logger().info(
            f'[ARM] Returning arm home over {duration}s')
        msg = ArmCommand()
        msg.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.duration = duration
        self.arm_pub.publish(msg)
        time.sleep(duration + 0.5)

    def _retract_holding(self, duration=3.0):
        """Retract arm to a raised overhead pose while keeping gripper closed."""
        self.get_logger().info(
            f'[ARM] Retracting arm to overhead pose over {duration}s')
        msg = ArmCommand()
        msg.joint_positions = list(RETRACT_HOLDING_POS)
        msg.duration = duration
        self.arm_pub.publish(msg)
        grip_msg = Float64MultiArray()
        grip_msg.data = [1.0]
        for _ in range(int(duration / 0.1)):
            self.gripper_pub.publish(grip_msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(0.5)

    # ── Waypoint recording ─────────────────────────────────────────────────

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

    # ── State: TASK_COMPLETE ─────────────────────────────────────────────────

    def _state_task_complete(self):
        self.get_logger().info('=' * 55)
        self.get_logger().info('[DONE] Pick-and-bring cycle complete!')
        self.get_logger().info('=' * 55)

        # Reset for next command
        self._phase = 'object'
        self._reset_detection_state()
        self._start_next_command()

    # ── Main loop ─────────────────────────────────────────────────────────────

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

        if self.command_queue:
            self._start_next_command()
        else:
            self.get_logger().info(
                'Waiting for voice command on /target_object ...')

        while rclpy.ok():
            deadline = time.time() + 0.05
            while time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0)
                time.sleep(0.002)

            # Check for safe-stop signal
            if self._safe_stop_requested:
                self._execute_safe_shutdown()
                return

            # Object detection runs during object scanning
            if (self.state == TaskState.SCANNING_OBJECT and
                    self.object_world_pos is None):
                self._check_for_object()

            # Person detection runs during person scanning
            if (self.state == TaskState.SCANNING_PERSON and
                    self.person_world_pos is None):
                self._check_for_person()

            # Record breadcrumb waypoints during navigation
            if self.state in (TaskState.SCANNING_OBJECT,
                              TaskState.CENTERING_OBJECT,
                              TaskState.APPROACHING_OBJECT,
                              TaskState.SCANNING_PERSON,
                              TaskState.CENTERING_PERSON,
                              TaskState.APPROACHING_PERSON):
                self._maybe_record_waypoint()

            # Keep gripper closed during person-finding phase
            if (self._phase == 'person' and
                    self.state not in (TaskState.WAITING_FOR_DROP,
                                       TaskState.TASK_COMPLETE,
                                       TaskState.WAITING_FOR_COMMAND)):
                grip_msg = Float64MultiArray()
                grip_msg.data = [1.0]
                self.gripper_pub.publish(grip_msg)

            # State dispatch
            if self.state == TaskState.WAITING_FOR_COMMAND:
                if self.command_queue:
                    self._start_next_command()
            elif self.state in (TaskState.SCANNING_OBJECT,
                                TaskState.SCANNING_PERSON):
                self._state_scanning()
            elif self.state in (TaskState.CENTERING_OBJECT,
                                TaskState.CENTERING_PERSON):
                self._state_centering()
            elif self.state in (TaskState.APPROACHING_OBJECT,
                                TaskState.APPROACHING_PERSON):
                self._state_approaching()
            elif self.state == TaskState.OBJECT_REACHED:
                self._state_object_reached()
            elif self.state == TaskState.GRASPING:
                self._state_grasping()
            elif self.state == TaskState.WAITING_FOR_DROP:
                self._state_waiting_for_drop()
            elif self.state == TaskState.TASK_COMPLETE:
                self._state_task_complete()


def main(args=None):
    rclpy.init(args=args)
    node = PickAndBring()
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
