#!/usr/bin/env python3
"""
TidyBot2 Find & Grasp Object (Simplified — no mapping/obstacles)

Simplified pipeline for unobstructed environments:
  1. WAITING_FOR_COMMAND — wait for voice or hardcoded command
  2. SCANNING            — 360 rotation scan, check YOLO each frame
  3. CENTERING           — rotate base + tilt camera to center object
  4. APPROACHING         — drive straight toward object with visual servoing
  5. COMPLETE            — object reached
  6. GRASPING            — top-down grasp pipeline

Build
cd ros2_ws && colcon build --packages-select tidybot_perception tidybot_bringup && source install/setup.bash

Terminal 1: robot/rviz bringup
ros2 launch tidybot_bringup sim.launch.py scene:=scene_banana_test.xml show_mujoco_viewer:=false

OR

ros2 launch tidybot_bringup real.launch.py show_mujoco_viewer:=false

Terminal 2: nav + grasp (no voice)
ros2 launch tidybot_bringup nav.launch.py target_object:=banana user_command:=get

With voice:
Terminal 2: nav + grasp (with voice)
ros2 launch tidybot_bringup nav.launch.py skip_voice:=false

Terminal 3 - interactive voice command script that will start nav
    ros2 run tidybot_bringup voice_command.py

To manually publish a voice command instead of using the mic (terminal 3):
    ros2 topic pub --once /user_command std_msgs/msg/String "data: 'get'"
    ros2 topic pub --once /target_object std_msgs/msg/String "data: 'banana'"
"""

import time
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


class ExploreState(Enum):
    WAITING_FOR_COMMAND = -1
    SCANNING    = 0
    CENTERING   = 6
    APPROACHING = 3
    COMPLETE    = 4
    GRASPING    = 5


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


class ExploreAndFind(Node):

    # ── Depth filtering ───────────────────────────────────────────────────────
    MIN_DEPTH_M       = 0.40
    MAX_DEPTH_M       = 5.00

    # ── 360 scan ─────────────────────────────────────────────────────────────
    SCAN_ANGULAR_VEL = 0.3   # rad/s
    SCAN_SETTLE_TIME = 0.5   # s
    MAX_SCAN_ATTEMPTS = 3

    # ── Centering ───────────────────────────────────────────────────────────
    CENTER_PIXEL_THRESH = 40    # px — how close to image center is "centered"
    CENTER_PIXEL_THRESH_V = 60  # px — vertical threshold (more lenient)
    CENTER_ANGULAR_VEL  = 0.25  # rad/s
    CENTER_TILT_STEP    = 0.05  # rad — camera tilt adjustment per tick
    CENTER_TIMEOUT      = 15.0  # s

    # ── Object detection ──────────────────────────────────────────────────────
    MIN_DETECTION_AREA = 50    # px²
    APPROACH_DIST      = 0.35  # m — stop before reaching the object
    DETECT_WINDOW      = 2.0   # s
    DETECT_COUNT_REQ   = 1

    # ── Approach ─────────────────────────────────────────────────────────────
    APPROACH_LINEAR_SPEED = 0.05  # m/s — very slow
    APPROACH_ANGULAR_GAIN = 0.003 # rad/s per pixel offset
    APPROACH_MAX_ANGULAR  = 0.3   # rad/s
    APPROACH_TIMEOUT      = 60.0  # s
    APPROACH_TILT_STEP    = 0.03  # rad — tilt adjustment during approach

    # ── Grasp parameters ─────────────────────────────────────────────────────
    GRASP_ARM_NAME               = 'right'
    GRASP_MOVE_DURATION          = 2.0
    GRASP_PRE_HEIGHT             = 0.10
    GRASP_LIFT_HEIGHT            = 0.15
    GRASP_Z_OFFSET               = -0.02
    GRASP_BBOX_PAD               = 1.3
    GRASP_SETTLE_TIME            = 1.0
    GRASP_GRIPPER_CLOSE_REPEATS  = 20
    GRASP_GRIPPER_CLOSE_WAIT     = 2.0
    GRASP_MIN_DEPTH_M            = 0.1
    GRASP_MAX_DEPTH_M            = 2.0
    GRASP_MIN_POINTS             = 20
    GRASP_TABLE_Z_MIN            = 0.005
    GRASP_OUTLIER_Z_THRESH       = 0.05

    # ─────────────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('explore_and_find')

        # Camera state
        self.latest_depth       = None
        self.latest_depth_stamp = None
        self.latest_rgb         = None
        self.camera_K           = None
        self.cv_bridge          = CvBridge()

        # State machine
        self.state             = ExploreState.WAITING_FOR_COMMAND
        self.scan_accumulated  = 0.0
        self.scan_last_heading = None
        self.scan_settle_start = None
        self.scan_attempt_count = 0

        # Camera tilt tracking
        self.current_tilt = 0.3  # initial tilt (set in _start_next_command)

        # Voice command parameters
        self.declare_parameter('skip_voice', False)
        self.skip_voice = (
            self.get_parameter('skip_voice')
            .get_parameter_value().bool_value)
        self.declare_parameter('target_object', 'banana')
        self.declare_parameter('user_command', 'get')

        # Command queue for sequential voice targets
        self.command_queue       = deque()
        self.current_object_name = ''
        self.current_action      = ''
        self.target_class_id     = -1
        self._last_voice_action  = 'get'

        # Object detection state
        self.object_world_pos      = None
        self.detection_times       = []
        self.last_detection_pos    = None
        self.latest_yolo_detection = None

        # Approach state
        self.approach_start_time = None

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image,      '/camera/depth/image_raw',
                                 self._depth_cb,       sensor_qos)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info',
                                 self._camera_info_cb, sensor_qos)

        yolo_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.bbox_sub = self.create_subscription(
            Detection2DArray,
            '/objbbox',
            self._yolo_bbox_cb,
            yolo_qos)

        voice_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, '/user_command', self._user_command_cb, voice_qos)
        self.create_subscription(
            String, '/target_object', self._target_object_cb, voice_qos)

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

        # Grasp infrastructure
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, f'/{self.GRASP_ARM_NAME}_gripper/cmd', 10)
        self.arm_pub = self.create_publisher(
            ArmCommand, f'/{self.GRASP_ARM_NAME}_arm/cmd', 10)
        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')

        self.get_logger().info('=' * 55)
        self.get_logger().info(
            'Find & Grasp (simplified — no mapping/obstacles)')
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
        self.get_logger().info('=' * 55)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _depth_cb(self, msg: Image):
        try:
            depth = self.cv_bridge.imgmsg_to_cv2(msg, '16UC1')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')
            return
        self.latest_depth       = depth
        self.latest_depth_stamp = msg.header.stamp

    def _camera_info_cb(self, msg: CameraInfo):
        self.camera_K = np.array(msg.k).reshape(3, 3)

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

    # ── Object detection ──────────────────────────────────────────────────────

    def _yolo_bbox_cb(self, msg: Detection2DArray):
        for detection in msg.detections:
            if not detection.results:
                continue
            class_id = int(detection.results[0].hypothesis.class_id)
            if class_id == self.target_class_id:
                self.latest_yolo_detection = detection
                return

    def _check_for_object(self):
        det = self.latest_yolo_detection
        if det is None or self.camera_K is None:
            return False

        self.latest_yolo_detection = None

        u = int(det.bbox.center.position.x)
        v = int(det.bbox.center.position.y)

        pos = self._estimate_object_position(u, v)
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

    def _refine_object_position(self):
        det = self.latest_yolo_detection
        if det is None:
            return
        self.latest_yolo_detection = None

        u = int(det.bbox.center.position.x)
        v = int(det.bbox.center.position.y)
        new_pos = self._estimate_object_position(u, v)
        if new_pos is None:
            return

        old_ox, old_oy = self.object_world_pos
        shift = np.hypot(new_pos[0] - old_ox, new_pos[1] - old_oy)
        if shift < 1.5:
            self.object_world_pos = new_pos
            self.last_detection_pos = new_pos
            if shift > 0.15:
                self.get_logger().info(
                    f'[REFINE] Updated object position: '
                    f'({new_pos[0]:.2f}, {new_pos[1]:.2f}) '
                    f'shift={shift:.2f}m')

    # ── Voice command callbacks ───────────────────────────────────────────

    def _user_command_cb(self, msg: String):
        if self.skip_voice:
            return
        action = msg.data.strip().lower()
        if action:
            self._last_voice_action = action
            self.get_logger().info(f'[VOICE] Action received: {action}')

    def _target_object_cb(self, msg: String):
        if self.skip_voice:
            return
        obj = msg.data.strip().lower()
        if not obj:
            return
        action = self._last_voice_action
        self.command_queue.append((action, obj))
        self.get_logger().info(f'[VOICE] Queued command: {action} {obj}')

    def _start_next_command(self):
        if not self.command_queue:
            self.state = ExploreState.WAITING_FOR_COMMAND
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

        # Tilt camera down so it can see the ground/objects ahead
        self.current_tilt = 0.3
        pt_msg = Float64MultiArray()
        pt_msg.data = [0.0, self.current_tilt]
        self.pan_tilt_pub.publish(pt_msg)

        self.get_logger().info('=' * 55)
        self.get_logger().info(
            f'NEW COMMAND: {action} {obj_name} (YOLO class {class_id})')
        self.get_logger().info('=' * 55)
        self.scan_attempt_count = 0
        self._start_scan()

    def _reset_detection_state(self):
        self.object_world_pos = None
        self.detection_times = []
        self.last_detection_pos = None
        self.latest_yolo_detection = None

    def _estimate_object_position(self, u, v):
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
                'base_link', 'camera_depth_optical_frame',
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
        self.pan_tilt_pub.publish(pt_msg)

    # ── State machine ─────────────────────────────────────────────────────────

    def _start_scan(self):
        self.scan_accumulated  = 0.0
        self.scan_last_heading = None
        self.scan_settle_start = None
        self.scan_attempt_count += 1
        self.state = ExploreState.SCANNING
        self.get_logger().info(
            f'[SCAN] Starting 360 rotation scan '
            f'(attempt {self.scan_attempt_count}/{self.MAX_SCAN_ATTEMPTS})...')

    def _stop_base(self):
        self.cmd_vel_pub.publish(Twist())

    def _state_scanning(self):
        if self.scan_settle_start is not None:
            if time.time() - self.scan_settle_start >= self.SCAN_SETTLE_TIME:
                if self.object_world_pos is not None:
                    self.get_logger().info(
                        '[SCAN] Settled — object detected, centering on it.')
                    self._start_centering()
                elif self.scan_attempt_count >= self.MAX_SCAN_ATTEMPTS:
                    self.get_logger().warn(
                        f'[SCAN] No object found after '
                        f'{self.MAX_SCAN_ATTEMPTS} scans — giving up.')
                    self.state = ExploreState.COMPLETE
                else:
                    self.get_logger().info(
                        '[SCAN] No object found — re-scanning...')
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
                f'[SCAN] 360 done '
                f'({np.degrees(self.scan_accumulated):.0f} accumulated). '
                f'Settling {self.SCAN_SETTLE_TIME} s...')
        else:
            cmd = Twist()
            cmd.angular.z = self.SCAN_ANGULAR_VEL
            self.cmd_vel_pub.publish(cmd)

    def _start_centering(self):
        self.center_start_time = time.time()
        self.state = ExploreState.CENTERING
        self.get_logger().info('[CENTER] Rotating to center object in camera...')

    def _state_centering(self):
        now = time.time()

        if now - self.center_start_time > self.CENTER_TIMEOUT:
            self._stop_base()
            self.get_logger().warn(
                '[CENTER] Timeout — re-scanning.')
            self._start_scan()
            return

        pose = self.get_base_pose()
        if pose is None:
            return
        bx, by, btheta = pose

        det = self.latest_yolo_detection
        if det is not None:
            # ── YOLO visible: fine-center using pixel offset ──────────
            u = int(det.bbox.center.position.x)
            v = int(det.bbox.center.position.y)

            if self.latest_depth is None:
                return
            img_h, img_w = self.latest_depth.shape
            img_center_x = img_w / 2.0
            img_center_y = img_h / 2.0
            offset_x = u - img_center_x  # positive = object right of center
            offset_y = v - img_center_y  # positive = object below center

            # Update object position from fresh detection
            new_pos = self._estimate_object_position(u, v)
            if new_pos is not None:
                if self.object_world_pos is not None:
                    shift = np.hypot(
                        new_pos[0] - self.object_world_pos[0],
                        new_pos[1] - self.object_world_pos[1])
                    if shift < 1.5:
                        self.object_world_pos = new_pos
                        self.last_detection_pos = new_pos
                else:
                    self.object_world_pos = new_pos
                    self.last_detection_pos = new_pos

            # Adjust camera tilt to center vertically
            if abs(offset_y) > self.CENTER_PIXEL_THRESH_V:
                if offset_y > 0:
                    self._set_camera_tilt(self.current_tilt + self.CENTER_TILT_STEP)
                else:
                    self._set_camera_tilt(self.current_tilt - self.CENTER_TILT_STEP)

            h_centered = abs(offset_x) < self.CENTER_PIXEL_THRESH
            v_centered = abs(offset_y) < self.CENTER_PIXEL_THRESH_V

            if h_centered and v_centered:
                self._stop_base()
                self.get_logger().info(
                    f'[CENTER] Object centered (h={offset_x:.0f}px, '
                    f'v={offset_y:.0f}px) — approaching.')
                self.approach_start_time = time.time()
                self.state = ExploreState.APPROACHING
                return

            # Rotate to reduce horizontal offset
            if not h_centered:
                cmd = Twist()
                frac = min(abs(offset_x) / (img_w / 2.0), 1.0)
                speed = self.CENTER_ANGULAR_VEL * (0.4 + 0.6 * frac)
                cmd.angular.z = -speed if offset_x > 0 else speed
                self.cmd_vel_pub.publish(cmd)

            if not hasattr(self, '_center_last_log') or now - self._center_last_log > 1.0:
                self._center_last_log = now
                self.get_logger().info(
                    f'[CENTER] h_offset={offset_x:.0f}px, v_offset={offset_y:.0f}px, '
                    f'tilt={self.current_tilt:.2f}')
        else:
            # ── No YOLO detection: rotate toward known world position ─
            if self.object_world_pos is None:
                self._stop_base()
                self.get_logger().warn('[CENTER] Lost object, re-scanning.')
                self._start_scan()
                return

            ox, oy = self.object_world_pos
            bearing = np.arctan2(oy - by, ox - bx)
            actual_heading = btheta - np.pi / 2
            heading_error = self._normalize_angle(bearing - actual_heading)

            if abs(heading_error) < np.radians(5):
                self._stop_base()
                if not hasattr(self, '_center_last_log') or now - self._center_last_log > 1.0:
                    self._center_last_log = now
                    self.get_logger().info(
                        '[CENTER] Facing object, waiting for YOLO detection...')
                return

            cmd = Twist()
            cmd.angular.z = float(np.clip(
                1.0 * heading_error, -self.CENTER_ANGULAR_VEL,
                self.CENTER_ANGULAR_VEL))
            self.cmd_vel_pub.publish(cmd)

            if not hasattr(self, '_center_last_log') or now - self._center_last_log > 1.0:
                self._center_last_log = now
                self.get_logger().info(
                    f'[CENTER] bearing err={np.degrees(heading_error):.0f}deg, '
                    f'rotating toward ({ox:.2f},{oy:.2f})')

    def _state_approaching(self):
        now = time.time()

        if now - self.approach_start_time > self.APPROACH_TIMEOUT:
            self._stop_base()
            self.get_logger().warn('[APPROACH] Timeout — declaring complete.')
            self.state = ExploreState.COMPLETE
            return

        pose = self.get_base_pose()
        if pose is None:
            return
        bx, by, btheta = pose
        actual_heading = btheta - np.pi / 2

        # Check distance to object via world position
        if self.object_world_pos is not None:
            ox, oy = self.object_world_pos
            dist_to_obj = np.hypot(ox - bx, oy - by)
            if dist_to_obj < self.APPROACH_DIST:
                self._stop_base()
                self.get_logger().info(
                    f'[APPROACH] Reached target! dist={dist_to_obj:.2f}m')
                self.state = ExploreState.COMPLETE
                return

        det = self.latest_yolo_detection
        cmd = Twist()

        if det is not None:
            # ── Visual servoing: steer based on YOLO pixel offset ─────
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

                # Adjust tilt to keep object vertically centered
                offset_y = v - img_center_y
                if abs(offset_y) > self.CENTER_PIXEL_THRESH_V:
                    if offset_y > 0:
                        self._set_camera_tilt(
                            self.current_tilt + self.APPROACH_TILT_STEP)
                    else:
                        self._set_camera_tilt(
                            self.current_tilt - self.APPROACH_TILT_STEP)

            # Refine world position
            new_pos = self._estimate_object_position(u, v)
            if new_pos is not None and self.object_world_pos is not None:
                shift = np.hypot(
                    new_pos[0] - self.object_world_pos[0],
                    new_pos[1] - self.object_world_pos[1])
                if shift < 1.5:
                    self.object_world_pos = new_pos
                    self.last_detection_pos = new_pos

            # Also check depth directly from YOLO bbox center
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
                    if depth_m < self.APPROACH_DIST:
                        self._stop_base()
                        self.get_logger().info(
                            f'[APPROACH] Object depth={depth_m:.2f}m < '
                            f'{self.APPROACH_DIST}m — reached!')
                        self.state = ExploreState.COMPLETE
                        return

            self.latest_yolo_detection = None  # consume

        else:
            # ── No YOLO: fall back to world heading ───────────────────
            if self.object_world_pos is not None:
                ox, oy = self.object_world_pos
                bearing = np.arctan2(oy - by, ox - bx)
                heading_error = self._normalize_angle(bearing - actual_heading)
                cmd.angular.z = float(np.clip(
                    1.0 * heading_error,
                    -self.APPROACH_MAX_ANGULAR,
                    self.APPROACH_MAX_ANGULAR))
            # else: drive straight (angular = 0)

        cmd.linear.x = self.APPROACH_LINEAR_SPEED
        self.cmd_vel_pub.publish(cmd)

        if not hasattr(self, '_approach_last_log') or now - self._approach_last_log > 2.0:
            self._approach_last_log = now
            dist_str = ''
            if self.object_world_pos is not None:
                ox, oy = self.object_world_pos
                dist_str = f' dist={np.hypot(ox-bx, oy-by):.2f}m'
            yolo_str = 'YOLO' if det is not None else 'world-heading'
            self.get_logger().info(
                f'[APPROACH] {yolo_str}{dist_str} '
                f'pos=({bx:.2f},{by:.2f}) tilt={self.current_tilt:.2f}')

    def _state_complete(self):
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
            self.get_logger().info(f'{name} NOT FOUND')
        self.get_logger().info('=' * 55)

        if (self.current_action == 'get' and
                self.object_world_pos is not None):
            self.get_logger().info(
                '[COMPLETE] Action is "get" — transitioning to GRASPING')
            self.state = ExploreState.GRASPING
            return

        self._start_next_command()

    # ── Grasping ─────────────────────────────────────────────────────────────

    def _state_grasping(self):
        self.get_logger().info('=' * 55)
        self.get_logger().info('[GRASP] Starting grasp pipeline...')

        if not self.plan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                '[GRASP] /plan_to_target service not available! '
                'Ensure sim.launch.py has use_motion_planner:=true')
            self._start_next_command()
            return

        det = None
        tilt_angles = [i * 0.1 for i in range(0, 10)]
        for tilt in tilt_angles:
            pt_msg = Float64MultiArray()
            pt_msg.data = [0.0, tilt]
            self.pan_tilt_pub.publish(pt_msg)
            self.get_logger().info(
                f'[GRASP] Camera tilt={tilt:.1f}, waiting for YOLO...')
            self.latest_yolo_detection = None
            self._grasp_spin_for(1.0)
            for _ in range(10):
                self._grasp_spin_for(0.2)
                if self.latest_yolo_detection is not None:
                    det = self.latest_yolo_detection
                    break
            if det is not None:
                self.get_logger().info(
                    f'[GRASP] Found object at tilt={tilt:.1f}')
                break

        if det is None:
            self.get_logger().error(
                '[GRASP] No YOLO detection at any tilt — cannot grasp')
            self._grasp_go_home()
            self._start_next_command()
            return

        self.latest_yolo_detection = None

        cx = int(det.bbox.center.position.x)
        cy = int(det.bbox.center.position.y)
        bbox_w = float(det.bbox.size_x)
        bbox_h = float(det.bbox.size_y)
        self.get_logger().info(
            f'[GRASP] YOLO detection at pixel ({cx}, {cy}), '
            f'bbox {bbox_w:.0f}x{bbox_h:.0f}')

        depth_roi, u_off, v_off = self._grasp_crop_depth(
            cx, cy, bbox_w, bbox_h)
        if depth_roi is None:
            self.get_logger().error('[GRASP] Failed to crop depth image')
            self._grasp_go_home()
            self._start_next_command()
            return

        pts_base = self._grasp_depth_to_points(depth_roi, u_off, v_off)
        if pts_base is None:
            self.get_logger().error(
                f'[GRASP] Insufficient points in depth ROI '
                f'(need >= {self.GRASP_MIN_POINTS})')
            self._grasp_go_home()
            self._start_next_command()
            return

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
            self._grasp_go_home()
            self._start_next_command()
            return
        grasp_pose, pre_grasp_pose, info = result
        self.get_logger().info(f'[GRASP] Params: {info}')

        use_orientation = True
        ok, msg = self._grasp_try_ik(grasp_pose, use_orientation=True)
        if not ok:
            self.get_logger().warn(
                f'[GRASP] IK with orientation failed: {msg}')
            self.get_logger().info(
                '[GRASP] Trying position-only IK...')
            ok, msg = self._grasp_try_ik(
                grasp_pose, use_orientation=False)
            if ok:
                use_orientation = False
            else:
                self.get_logger().error(
                    f'[GRASP] Position-only IK also failed: {msg}')
                self._grasp_go_home()
                self._start_next_command()
                return

        self.get_logger().info('[GRASP] Opening gripper')
        self._grasp_set_gripper(0.0)

        self.get_logger().info('[GRASP] Moving to pre-grasp')
        if not self._grasp_plan_and_execute(
                pre_grasp_pose, 'pre-grasp', use_orientation):
            self.get_logger().error('[GRASP] Pre-grasp failed')
            self._grasp_go_home()
            self._start_next_command()
            return

        self._grasp_spin_for(self.GRASP_SETTLE_TIME)

        self.get_logger().info('[GRASP] Descending to grasp')
        if not self._grasp_plan_and_execute(
                grasp_pose, 'grasp', use_orientation):
            self.get_logger().error('[GRASP] Descent failed')
            self._grasp_go_home()
            self._start_next_command()
            return

        self._grasp_spin_for(self.GRASP_SETTLE_TIME)

        self.get_logger().info('[GRASP] Closing gripper (firm)')
        self._grasp_close_firm()

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

        self.get_logger().info('=' * 55)
        self.get_logger().info('[GRASP] Grasp complete — holding object')
        self.get_logger().info('=' * 55)

        self._grasp_spin_for(3.0)
        self._grasp_set_gripper(0.0)
        self._grasp_spin_for(0.5)
        self._grasp_go_home()

        self._start_next_command()

    # ── Grasp helper methods ─────────────────────────────────────────────────

    def _grasp_spin_for(self, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _grasp_set_gripper(self, position):
        msg = Float64MultiArray()
        msg.data = [position]
        for _ in range(10):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(0.5)

    def _grasp_close_firm(self):
        msg = Float64MultiArray()
        msg.data = [1.0]
        for _ in range(self.GRASP_GRIPPER_CLOSE_REPEATS):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(self.GRASP_GRIPPER_CLOSE_WAIT)
        for _ in range(10):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def _grasp_go_home(self, duration=3.0):
        self.get_logger().info(
            f'[GRASP] Returning arm home over {duration}s')
        msg = ArmCommand()
        msg.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.duration = duration
        self.arm_pub.publish(msg)
        time.sleep(duration + 0.5)

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
            'base_link', 'camera_depth_optical_frame')
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

        return pts_base

    def _grasp_analyze_object(self, points_base):
        centroid = np.mean(points_base, axis=0)
        z_top = float(np.max(points_base[:, 2]))

        pts_xy = points_base[:, :2]
        centered = pts_xy - centroid[:2]

        best_yaw = 0.0
        min_width = float('inf')
        max_width = 0.0
        for angle_deg in range(0, 180, 5):
            angle = np.radians(angle_deg)
            direction = np.array([np.cos(angle), np.sin(angle)])
            projections = centered @ direction
            width = float(np.ptp(projections))
            if width < min_width:
                min_width = width
                best_yaw = angle
            if width > max_width:
                max_width = width

        self.get_logger().info(
            f'[GRASP] Min-width sweep: '
            f'yaw={np.degrees(best_yaw):.0f}deg, '
            f'grip={min_width*1000:.0f}mm, '
            f'max={max_width*1000:.0f}mm')

        return {
            'centroid': centroid,
            'grasp_yaw': best_yaw,
            'grip_width': min_width,
            'max_width': max_width,
            'z_top': z_top,
            'num_points': len(points_base),
        }

    def _grasp_compute_poses(self, analysis):
        centroid = analysis['centroid']
        grasp_yaw = analysis['grasp_yaw']
        grip_width = analysis['grip_width']

        if grip_width > GRIPPER_MAX_OPENING_M:
            self.get_logger().warn(
                f'[GRASP] Width {grip_width*1000:.0f}mm > '
                f'{GRIPPER_MAX_OPENING_M*1000:.0f}mm max '
                f'(proceeding anyway)')

        qw, qx, qy, qz = yaw_to_grasp_quaternion(grasp_yaw)

        grasp_z = analysis['z_top'] + self.GRASP_Z_OFFSET

        grasp_pose = Pose()
        grasp_pose.position.x = float(centroid[0])
        grasp_pose.position.y = float(centroid[1])
        grasp_pose.position.z = float(grasp_z)
        grasp_pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

        pre_grasp_pose = Pose()
        pre_grasp_pose.position.x = float(centroid[0])
        pre_grasp_pose.position.y = float(centroid[1])
        pre_grasp_pose.position.z = float(grasp_z + self.GRASP_PRE_HEIGHT)
        pre_grasp_pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

        info = (f'yaw={np.degrees(grasp_yaw):.0f}deg, '
                f'z={grasp_z:.3f}m, width={grip_width*1000:.0f}mm')

        return grasp_pose, pre_grasp_pose, info

    def _grasp_try_ik(self, pose, use_orientation=True):
        req = PlanToTarget.Request()
        req.arm_name = self.GRASP_ARM_NAME
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = False
        req.duration = self.GRASP_MOVE_DURATION
        req.max_condition_number = 200.0

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done() or future.exception():
            return False, 'service call timed out'

        result = future.result()
        return result.success, result.message

    def _grasp_plan_and_execute(self, pose, label, use_orientation=True):
        req = PlanToTarget.Request()
        req.arm_name = self.GRASP_ARM_NAME
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = True
        req.duration = self.GRASP_MOVE_DURATION
        req.max_condition_number = 100.0

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
                time.sleep(self.GRASP_MOVE_DURATION + 0.5)
            return True

        self.get_logger().warn(
            f'  [{label}] Failed: {result.message}')
        return False

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        self.get_logger().info('Waiting for depth data and TF...')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_depth is not None and self.camera_K is not None:
                try:
                    self.tf_buffer.lookup_transform(
                        'odom', 'camera_depth_optical_frame',
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

            # Object detection runs during scanning
            if (self.state == ExploreState.SCANNING and
                    self.object_world_pos is None):
                self._check_for_object()

            if self.state == ExploreState.WAITING_FOR_COMMAND:
                if self.command_queue:
                    self._start_next_command()
            elif self.state == ExploreState.SCANNING:
                self._state_scanning()
            elif self.state == ExploreState.CENTERING:
                self._state_centering()
            elif self.state == ExploreState.APPROACHING:
                self._state_approaching()
            elif self.state == ExploreState.COMPLETE:
                self._state_complete()
            elif self.state == ExploreState.GRASPING:
                self._state_grasping()


def main(args=None):
    rclpy.init(args=args)
    node = ExploreAndFind()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
