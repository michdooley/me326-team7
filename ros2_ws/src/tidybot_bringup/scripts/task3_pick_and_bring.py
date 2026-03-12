#!/usr/bin/env python3
"""
TidyBot2 Pick-and-Bring Pipeline (Task 3 — Face + Grasp Validation + Bin Placement)

Integrated pipeline:
  Phase 1 (YOLO): find object -> center -> approach -> grasp -> validate
  Phase 2 (Face Recognition): find named person -> center -> approach -> present
  Phase 3 (Voice decision):
    a) "drop the <object>"  -> release in place
    b) "bring it to the <object> bin" -> find bin (AprilTag) -> approach -> position -> drop

Runs alongside:
  - ros2 run object_classification classifier   (YOLO, publishes /objbbox)
  - ros2 run tidybot_perception face_recognition_node --ros-args \
        -p known_faces_dir:=<path_to_face_images> -p person_name:=michael
  - ros2 run tidybot_bringup voice_command.py   (publishes /target_object, /user_command)

Build:
  cd ros2_ws && colcon build --packages-select tidybot_perception tidybot_bringup && source install/setup.bash

Terminal 1: ros2 launch tidybot_bringup sim.launch.py
Terminal 2: ros2 run object_classification classifier
Terminal 3: ros2 run tidybot_perception face_recognition_node --ros-args \
    -p known_faces_dir:=/path/to/face -p person_name:=michael
Terminal 4: ros2 run tidybot_bringup task3_pick_and_bring.py --ros-args \
    -p skip_voice:=true -p target_object:=banana -p target_person:=michael
"""

import sys
import os
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
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import MarkerArray
from vision_msgs.msg import Detection2DArray
from tidybot_msgs.msg import ArmCommand
from tidybot_msgs.srv import PlanToTarget

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

# Add scripts directory to path for common module imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from common.constants import (
    YOLO_CLASS_MAP, TAG_MAP, GRIPPER_OPEN_POS, PRESENT_POSE,
)
from common.perception import PerceptionMixin
from common.grasp_pipeline import GraspMixin
from common.arm_helpers import ArmHelpersMixin
from common.base_control import BaseControlMixin


class TaskState(Enum):
    WAITING_FOR_COMMAND = 0
    # Phase 1: Find & grasp object (YOLO)
    SCANNING_OBJECT     = 10
    CENTERING_OBJECT    = 11
    APPROACHING_OBJECT  = 12
    OBJECT_REACHED      = 13
    GRASPING            = 14
    # Phase 2: Find person & approach (Face Recognition)
    SCANNING_PERSON     = 20
    CENTERING_PERSON    = 21
    APPROACHING_PERSON  = 22
    # Phase 3: Deliver / decide
    WAITING_FOR_DROP    = 30
    # Phase 3b: Find bin & drop (AprilTag)
    SCANNING_BIN        = 31
    CENTERING_BIN       = 32
    APPROACHING_BIN     = 33
    POSITIONING         = 34
    DROPPING            = 35
    # Terminal
    TASK_COMPLETE       = 40


class PickAndBring(PerceptionMixin, GraspMixin, ArmHelpersMixin,
                   BaseControlMixin, Node):

    # ── Depth filtering ───────────────────────────────────────────────────────
    MIN_DEPTH_M       = 0.15
    MAX_DEPTH_M       = 5.00

    # ── 360 scan ─────────────────────────────────────────────────────────────
    SCAN_ANGULAR_VEL = -0.3   # rad/s
    SCAN_SETTLE_TIME = 0.5    # s
    MAX_SCAN_ATTEMPTS = 3

    # ── Centering ────────────────────────────────────────────────────────────
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
    BIN_APPROACH_CENTER_PIXELS = 120

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
    GRASP_Z_OFFSET               = 0.02
    GRASP_X_OFFSET               = -0.05
    GRASP_Y_OFFSET               = -0.02
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
    GRASP_MAX_IK_NUDGES          = 6

    # ── Grasp validation ────────────────────────────────────────────────────
    GRASP_VALIDATION_TILT        = -0.0
    GRASP_VALIDATION_WAIT        = 3.0
    GRASP_VALIDATION_MIN_CONF    = 0.10

    # ── Bin parameters ────────────────────────────────────────────────────────
    BIN_HEIGHT              = 0.3048
    BIN_HALF_HEIGHT         = 0.1524
    DROP_HEIGHT_ABOVE       = 0.10
    BIN_DEPTH_OFFSET        = 0.07
    BIN_MOVE_DURATION       = 2.5
    BIN_MAX_COND_NUM        = 500.0

    # ── Sweet spot ────────────────────────────────────────────────────────────
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
            'initial_tilt': 0.7854,
            'scan_tilt': 0.3,
        },
        'person': {
            'center_pixel_thresh':   80,
            'center_pixel_thresh_v': 60,
            'approach_stop_dist':    0.6,
            'approach_linear_speed': 0.05,
            'initial_tilt':          -0.1,
            'scan_tilt':             -0.1,
        },
        'bin': {
            'center_pixel_thresh':   40,
            'center_pixel_thresh_v': 40,
            'approach_stop_dist':    0.30,
            'approach_linear_speed': 0.10,
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
        self._phase            = 'object'  # 'object', 'person', 'bin'
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
        self.declare_parameter('target_person', 'michael')
        self.declare_parameter('skip_grasp_validation', True)
        _sgv_param = self.get_parameter('skip_grasp_validation').get_parameter_value()
        self.skip_grasp_validation = (
            _sgv_param.bool_value
            or str(_sgv_param.string_value).lower() in ('true', '1', 'yes'))

        self.target_person = (
            self.get_parameter('target_person')
            .get_parameter_value().string_value.lower())

        self.GRASP_ARM_NAME = (
            self.get_parameter('arm_name')
            .get_parameter_value().string_value.lower())

        # Mirror sweet-spot X for arm symmetry
        _arm_x_sign = 1.0 if self.GRASP_ARM_NAME == 'left' else -1.0
        self.GRASP_SWEET_SPOT_X  = self.__class__.GRASP_SWEET_SPOT_X  * _arm_x_sign
        self._ik_nudge_target_x  = 0.12 * _arm_x_sign

        # Command queue
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

        # Person detection state (Phase 2 - Face Recognition)
        self.person_world_pos       = None
        self.latest_face_detection  = None

        # Bin detection state (Phase 3b - AprilTag)
        self.bin_world_pos          = None
        self.latest_tag_detection   = None
        self.target_tag_id          = None

        # Drop / bin command flags
        self._drop_pending = False
        self._bin_pending = False

        # Approach state
        self.approach_start_time = None

        # Centering state
        self.center_start_time = None
        self._center_last_log = 0.0

        # Waypoint recording
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

        # YOLO detections (Phase 1)
        yolo_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Detection2DArray, '/objbbox',
            self._yolo_bbox_cb, yolo_qos)

        # Face detections (Phase 2)
        self.create_subscription(
            Detection2DArray, '/face_detections',
            self._face_detection_cb, yolo_qos)

        # AprilTag detections (Phase 3b)
        tag_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Detection2DArray, '/apriltag_detections',
            self._apriltag_cb, tag_qos)

        # Voice commands
        voice_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, '/user_command', self._user_command_cb, voice_qos)
        self.create_subscription(
            String, '/target_object', self._target_object_cb, voice_qos)
        self.create_subscription(
            String, '/target_location', self._target_location_cb, voice_qos)

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
            'Pick-and-Bring Pipeline (Face Recognition)')
        self.get_logger().info(
            f'  Person : {self.target_person}')
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
        self.get_logger().info(f'  Skip grasp validation: {self.skip_grasp_validation}')
        self.get_logger().info('=' * 55)

    # ── Detection callbacks (task-specific) ───────────────────────────────────

    def _face_detection_cb(self, msg: Detection2DArray):
        for detection in msg.detections:
            if not detection.results:
                continue
            hyp = detection.results[0].hypothesis
            name = hyp.class_id.lower()
            if name == self.target_person and hyp.score >= 0.3:
                self.latest_face_detection = detection
                return

    def _apriltag_cb(self, msg: Detection2DArray):
        for detection in msg.detections:
            if not detection.results:
                continue
            hyp = detection.results[0].hypothesis
            if hyp.class_id == self.target_tag_id:
                self.latest_tag_detection = detection
                return

    # ── Phase routing helpers ────────────────────────────────────────────────

    def _get_active_detection(self):
        if self._phase == 'person':
            return self.latest_face_detection
        if self._phase == 'bin':
            return self.latest_tag_detection
        return self.latest_yolo_detection

    def _consume_active_detection(self):
        if self._phase == 'person':
            det = self.latest_face_detection
            self.latest_face_detection = None
            return det
        if self._phase == 'bin':
            det = self.latest_tag_detection
            self.latest_tag_detection = None
            return det
        det = self.latest_yolo_detection
        self.latest_yolo_detection = None
        return det

    @property
    def _active_world_pos(self):
        if self._phase == 'object':
            return self.object_world_pos
        if self._phase == 'bin':
            return self.bin_world_pos
        return self.person_world_pos

    @_active_world_pos.setter
    def _active_world_pos(self, val):
        if self._phase == 'object':
            self.object_world_pos = val
        elif self._phase == 'bin':
            self.bin_world_pos = val
        else:
            self.person_world_pos = val

    def _pp(self, key):
        return self.PHASE_PARAMS[self._phase][key]

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
        grip_msg.data = [GRIPPER_OPEN_POS]
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
        det = self.latest_face_detection
        if det is None or self.camera_K is None:
            return False

        self.latest_face_detection = None

        u = int(det.bbox.center.position.x)
        v = int(det.bbox.center.position.y)
        name = det.results[0].hypothesis.class_id if det.results else 'face'
        score = det.results[0].hypothesis.score if det.results else 0.0

        pos = self._estimate_world_position(u, v)
        if pos is None:
            return False

        self.person_world_pos = pos
        self.get_logger().info(
            f'[DETECT] Face "{name}" (score={score:.2f}) confirmed at '
            f'({pos[0]:.2f}, {pos[1]:.2f})')
        return True

    # ── Voice command callbacks ───────────────────────────────────────────

    DROP_ACTIONS = {'drop', 'release', 'let go', 'put down'}
    BIN_KEYWORDS = {'bin', 'place', 'bring it to'}

    def _accepting_voice(self):
        return (not self.skip_voice or
                self.state in (TaskState.WAITING_FOR_COMMAND,
                               TaskState.WAITING_FOR_DROP))

    def _parse_bin_target(self, text):
        for obj_name in TAG_MAP:
            if obj_name in text:
                return obj_name
        return None

    def _user_command_cb(self, msg: String):
        action = msg.data.strip().lower()
        if not action:
            return

        if self.state == TaskState.WAITING_FOR_DROP:
            if any(kw in action for kw in self.DROP_ACTIONS):
                self.get_logger().info(f'[VOICE] Drop command received: "{action}"')
                self._drop_pending = True
                return

            if 'bin' in action or 'bring' in action or 'place' in action:
                bin_target = self._parse_bin_target(action)
                if bin_target is not None:
                    self.get_logger().info(
                        f'[VOICE] Bin command received: "{action}" -> {bin_target} bin')
                    self._bin_pending = True
                    self._bin_target_name = bin_target
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

    def _target_location_cb(self, msg: String):
        location = msg.data.strip().lower()
        if not location:
            return
        if self.state != TaskState.WAITING_FOR_DROP:
            return
        bin_target = self._parse_bin_target(location)
        if bin_target is not None:
            self.get_logger().info(
                f'[VOICE] Bin command from location: "{location}" -> {bin_target} bin')
            self._bin_pending = True
            self._bin_target_name = bin_target

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

        # Record starting pose
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
        self.latest_face_detection = None
        self.person_world_pos = None
        self.bin_world_pos = None
        self.latest_tag_detection = None

    # ── Unified scan/center/approach states ──────────────────────────────────

    _PHASE_STATE_MAP_SCAN = {
        'object': TaskState.SCANNING_OBJECT,
        'person': TaskState.SCANNING_PERSON,
        'bin':    TaskState.SCANNING_BIN,
    }

    def _start_scan(self):
        self.scan_accumulated  = 0.0
        self.scan_last_heading = None
        self.scan_settle_start = None
        self.scan_attempt_count += 1
        self._set_camera_tilt(self._pp('scan_tilt'))

        self.state = self._PHASE_STATE_MAP_SCAN[self._phase]

        phase_label = self._phase.upper()
        self.get_logger().info(
            f'[SCAN-{phase_label}] Starting 360 rotation scan '
            f'(attempt {self.scan_attempt_count}/{self.MAX_SCAN_ATTEMPTS})...')

    def _state_scanning(self):
        phase_label = self._phase.upper()

        # Check for detection mid-scan
        if self._phase == 'object':
            if self.object_world_pos is not None:
                self._stop_base()
                self.get_logger().info(
                    f'[SCAN-{phase_label}] Object spotted mid-scan — centering.')
                self._start_centering()
                return
        elif self._phase == 'person':
            det = self.latest_face_detection
            if det is not None:
                self.latest_face_detection = None
                u = int(det.bbox.center.position.x)
                v = int(det.bbox.center.position.y)
                name = det.results[0].hypothesis.class_id if det.results else 'face'
                self.get_logger().info(
                    f'[SCAN-PERSON] Face "{name}" detected at pixel ({u}, {v})')
                pos = self._estimate_world_position(u, v)
                if pos is not None:
                    self.person_world_pos = pos
                self._stop_base()
                self._start_centering()
                return
        elif self._phase == 'bin':
            det = self.latest_tag_detection
            if det is not None:
                self.latest_tag_detection = None
                u = int(det.bbox.center.position.x)
                v = int(det.bbox.center.position.y)
                self.get_logger().info(
                    f'[SCAN-BIN] Tag {self.target_tag_id} detected at pixel ({u}, {v})')
                pos = self._estimate_world_position(u, v)
                if pos is not None:
                    self.bin_world_pos = pos
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

    _PHASE_STATE_MAP_CENTER = {
        'object': TaskState.CENTERING_OBJECT,
        'person': TaskState.CENTERING_PERSON,
        'bin':    TaskState.CENTERING_BIN,
    }

    def _start_centering(self):
        self.center_start_time = time.time()
        self.state = self._PHASE_STATE_MAP_CENTER[self._phase]
        phase_label = self._phase.upper()
        self.get_logger().info(
            f'[CENTER-{phase_label}] Rotating to center detection in camera...')

    _PHASE_STATE_MAP_APPROACH = {
        'object': TaskState.APPROACHING_OBJECT,
        'person': TaskState.APPROACHING_PERSON,
        'bin':    TaskState.APPROACHING_BIN,
    }

    def _state_centering(self):
        now = time.time()
        phase_label = self._phase.upper()
        thresh_h = self._pp('center_pixel_thresh')
        thresh_v = self._pp('center_pixel_thresh_v')

        if now - self.center_start_time > self.CENTER_TIMEOUT:
            self._stop_base()
            if self._active_world_pos is not None:
                self.get_logger().warn(
                    f'[CENTER-{phase_label}] Timeout — known pos, approaching anyway.')
                self.approach_start_time = time.time()
                self.state = self._PHASE_STATE_MAP_APPROACH[self._phase]
            else:
                self.get_logger().warn(
                    f'[CENTER-{phase_label}] Timeout — re-scanning.')
                self._start_scan()
            return

        pose = self.get_base_pose()
        if pose is None:
            return
        bx, by, btheta = pose

        det = self._consume_active_detection()
        if det is not None:
            u = int(det.bbox.center.position.x)
            v = int(det.bbox.center.position.y)

            if self.latest_depth is None:
                return
            img_h, img_w = self.latest_depth.shape
            img_center_x = img_w / 2.0
            img_center_y = img_h / 2.0
            offset_x = u - img_center_x
            offset_y = v - img_center_y

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
                self.state = self._PHASE_STATE_MAP_APPROACH[self._phase]
                return

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

    def _approach_reached(self):
        if self._phase == 'object':
            self.state = TaskState.OBJECT_REACHED
        elif self._phase == 'person':
            self._transition_to_waiting_for_drop()
        elif self._phase == 'bin':
            self.state = TaskState.POSITIONING

    def _state_approaching(self):
        now = time.time()
        phase_label = self._phase.upper()
        stop_dist = self._pp('approach_stop_dist')
        linear_speed = self._pp('approach_linear_speed')
        thresh_v = self._pp('center_pixel_thresh_v')

        if now - self.approach_start_time > self.APPROACH_TIMEOUT:
            self._stop_base()
            self.get_logger().warn(f'[APPROACH-{phase_label}] Timeout.')
            self._approach_reached()
            return

        pose = self.get_base_pose()
        if pose is None:
            return
        bx, by, btheta = pose
        actual_heading = btheta - np.pi / 2

        if self._active_world_pos is not None:
            ox, oy = self._active_world_pos
            dist_to_target = np.hypot(ox - bx, oy - by)
            if dist_to_target < stop_dist:
                self._stop_base()
                self.get_logger().info(
                    f'[APPROACH-{phase_label}] Reached target! dist={dist_to_target:.2f}m')
                self._approach_reached()
                return

        det = self._consume_active_detection()
        cmd = Twist()

        if det is not None:
            u = int(det.bbox.center.position.x)
            v = int(det.bbox.center.position.y)

            if self.latest_depth is not None:
                img_h, img_w = self.latest_depth.shape
                img_center_x = img_w / 2.0
                img_center_y = img_h / 2.0
                offset_x = u - img_center_x

                angular = -self.APPROACH_ANGULAR_GAIN * offset_x
                cmd.angular.z = float(np.clip(
                    angular, -self.APPROACH_MAX_ANGULAR, self.APPROACH_MAX_ANGULAR))

                if self._phase == 'bin':
                    center_fraction = max(
                        0.0, 1.0 - abs(offset_x) / self.BIN_APPROACH_CENTER_PIXELS)
                    linear_speed = self._pp('approach_linear_speed') * center_fraction

                offset_y = v - img_center_y
                if abs(offset_y) > thresh_v:
                    tilt_adj = offset_y / (img_h / 2.0) * self.APPROACH_TILT_STEP
                    self._set_camera_tilt(self.current_tilt + tilt_adj)

            new_pos = self._estimate_world_position(u, v)
            if new_pos is not None and self._active_world_pos is not None:
                shift = np.hypot(
                    new_pos[0] - self._active_world_pos[0],
                    new_pos[1] - self._active_world_pos[1])
                if shift < 1.5:
                    self._active_world_pos = new_pos
                    self.last_detection_pos = new_pos

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
                        self._approach_reached()
                        return
        else:
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

    def _grasp_attempt(self):
        """Single grasp attempt. Returns True on success."""
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

        # Try IK — nudge and retry if needed
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

        # Validate grasp at trophy pose
        if not self._validate_grasp():
            return False

        # Retract arm
        self._retract_holding(trophy_pose=trophy_pose)

        # Transition to Phase 2: find person
        self._transition_to_person_finding()
        return True

    # ── Grasp validation ────────────────────────────────────────────────────

    def _validate_grasp(self):
        if self.skip_grasp_validation:
            self.get_logger().info('[GRASP-VALIDATE] Validation skipped (parameter)')
            return True

        self.get_logger().info('[GRASP-VALIDATE] Validating grasp at trophy pose...')

        self._set_camera_tilt(self.GRASP_VALIDATION_TILT)
        self.get_logger().info(
            f'[GRASP-VALIDATE] Camera tilt -> {self.GRASP_VALIDATION_TILT:.2f}, waiting to settle...')
        self._spin_for(2.0)

        self.latest_yolo_detection = None
        saved_min_conf = self.MIN_CONFIDENCE
        self.MIN_CONFIDENCE = self.GRASP_VALIDATION_MIN_CONF

        detected = False
        deadline = time.time() + self.GRASP_VALIDATION_WAIT
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_yolo_detection is not None:
                det = self.latest_yolo_detection
                score = det.results[0].hypothesis.score
                self.get_logger().info(
                    f'[GRASP-VALIDATE] Object detected (conf={score:.2f}) '
                    f'— grasp CONFIRMED!')
                detected = True
                break

        self.MIN_CONFIDENCE = saved_min_conf

        if detected:
            return True

        self.get_logger().warn(
            '[GRASP-VALIDATE] Object NOT detected — grasp FAILED')

        self._set_camera_tilt(self.PHASE_PARAMS['object']['initial_tilt'])

        self._set_gripper(GRIPPER_OPEN_POS)
        self._spin_for(0.5)

        return False

    # ── Phase transitions ─────────────────────────────────────────────────

    def _transition_to_person_finding(self):
        self._phase = 'person'
        self.person_world_pos = None
        self.latest_face_detection = None

        self.get_logger().info('=' * 55)
        self.get_logger().info(
            f'[TRANSITION] Phase 2: Finding "{self.target_person}" to deliver '
            f'{self.current_object_name}')
        self.get_logger().info('=' * 55)

        self.scan_attempt_count = 0
        self._start_scan()

    def _transition_to_waiting_for_drop(self):
        self.get_logger().info('=' * 55)
        self.get_logger().info('[DELIVER] Person reached — raising arm for delivery')
        self.get_logger().info('[DELIVER] Say "drop the <object>" to release here')
        self.get_logger().info('[DELIVER] Or say "bring it to the <object> bin" to place in bin')
        self.get_logger().info('=' * 55)

        msg = ArmCommand()
        msg.joint_positions = list(PRESENT_POSE)
        msg.duration = 2.0
        self.arm_pub.publish(msg)

        grip_msg = Float64MultiArray()
        grip_msg.data = [1.0]
        deadline = time.time() + 2.5
        while time.time() < deadline:
            self.gripper_pub.publish(grip_msg)
            rclpy.spin_once(self, timeout_sec=0.1)

        self._drop_pending = False
        self._bin_pending = False
        self.state = TaskState.WAITING_FOR_DROP

    # ── State: WAITING_FOR_DROP ──────────────────────────────────────────────

    def _state_waiting_for_drop(self):
        if self._bin_pending:
            self._bin_pending = False
            bin_name = getattr(self, '_bin_target_name', self.current_object_name)
            self.get_logger().info(
                f'[DELIVER] Bin command received — heading to {bin_name} bin')
            self._retract_holding()
            self._transition_to_bin_finding(bin_name)
            return

        if not self._drop_pending:
            return

        self.get_logger().info('[DROP] Drop command received — releasing object')

        self._set_gripper(0.0)
        self._spin_for(1.0)

        self.get_logger().info('[DROP] Returning arm to sleep pose')
        self._go_home(duration=3.0)

        self._set_camera_tilt(0.3)

        self.get_logger().info('=' * 55)
        self.get_logger().info('[DROP] Object released. Task complete!')
        self.get_logger().info('=' * 55)

        self._drop_pending = False
        self.state = TaskState.TASK_COMPLETE

    # ── Bin finding & placement (Phase 3b) ────────────────────────────────

    def _transition_to_bin_finding(self, bin_name=None):
        if bin_name is None:
            bin_name = self.current_object_name
        self._phase = 'bin'
        self.target_tag_id = TAG_MAP.get(bin_name)

        if self.target_tag_id is None:
            self.get_logger().warn(
                f'[TRANSITION] No bin tag mapping for "{bin_name}" '
                f'(known: {list(TAG_MAP.keys())}). Dropping in place instead.')
            self._set_gripper(0.05)
            self._spin_for(1.0)
            self._go_home(duration=3.0)
            self._phase = 'object'
            self.state = TaskState.TASK_COMPLETE
            return

        self.get_logger().info('=' * 55)
        self.get_logger().info(
            f'[TRANSITION] Phase 3b: Finding bin for {bin_name} '
            f'(tag {self.target_tag_id})')
        self.get_logger().info('=' * 55)

        self.bin_world_pos = None
        self.latest_tag_detection = None
        self.last_detection_pos = None
        self.scan_attempt_count = 0

        self.get_logger().info('[TRANSITION] Checking for tag before scanning...')
        self._set_camera_tilt(0.5)
        self._spin_for(1.0)

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
                    f'[TRANSITION] Tag {self.target_tag_id} detected at pixel ({u}, {v})')
                pos = self._estimate_world_position(u, v)
                if pos is not None:
                    self.bin_world_pos = pos
                found = True
                break

        if found:
            self.get_logger().info('[TRANSITION] Tag already visible — centering!')
            self._start_centering()
        else:
            self.get_logger().info('[TRANSITION] Tag not visible — starting scan')
            self._set_camera_tilt(0.3)
            self._start_scan()

    # ── State: POSITIONING (Phase 3b — bin) ────────────────────────────────

    def _state_positioning(self):
        self._stop_base()
        self.get_logger().info('=' * 55)
        self.get_logger().info(
            f'[POSITION] Positioning arm above '
            f'{getattr(self, "_bin_target_name", self.current_object_name)} bin...')

        if not self.plan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                '[POSITION] /plan_to_target service not available!')
            self.state = TaskState.TASK_COMPLETE
            return

        self.get_logger().info('[POSITION] Looking for tag to get fresh position...')
        self._set_camera_tilt(0.5)
        self._spin_for(1.0)

        bin_base = None
        for poll in range(20):
            self._spin_for(0.2)
            det = self.latest_tag_detection
            if det is not None:
                self.latest_tag_detection = None
                u = int(det.bbox.center.position.x)
                v = int(det.bbox.center.position.y)
                bin_base = self._get_position_in_base_link(u, v)
                if bin_base is not None:
                    self.get_logger().info(
                        f'[POSITION] Tag center in base_link: '
                        f'({bin_base[0]:.3f}, {bin_base[1]:.3f}, {bin_base[2]:.3f})')
                    break

        if bin_base is None:
            self.get_logger().warn(
                '[POSITION] Could not localize bin — trying with default position')
            if self.bin_world_pos is not None:
                bin_base = np.array([0.0, -0.30, 0.15])
            else:
                self.get_logger().error('[POSITION] No bin position available!')
                self.state = TaskState.TASK_COMPLETE
                return

        approach_xy = np.array([bin_base[0], bin_base[1]])
        approach_dist = np.linalg.norm(approach_xy)
        if approach_dist > 0.01:
            approach_unit = approach_xy / approach_dist
        else:
            approach_unit = np.array([1.0, 0.0])
        drop_z = bin_base[2] + self.BIN_HALF_HEIGHT + self.DROP_HEIGHT_ABOVE
        drop_x = bin_base[0] + self.BIN_DEPTH_OFFSET * approach_unit[0]
        drop_y = bin_base[1] + self.BIN_DEPTH_OFFSET * approach_unit[1]
        approach_angle = np.degrees(np.arctan2(approach_unit[1], approach_unit[0]))

        self.get_logger().info(
            f'[POSITION] Drop target in base_link: '
            f'({drop_x:.3f}, {drop_y:.3f}, {drop_z:.3f}), '
            f'approach_angle={approach_angle:.0f}deg')

        MAX_DROP_NUDGES = 4
        qw, qx, qy, qz = yaw_to_grasp_quaternion(0.0)

        for nudge_i in range(MAX_DROP_NUDGES + 1):
            drop_pose = Pose()
            drop_pose.position.x = drop_x
            drop_pose.position.y = drop_y
            drop_pose.position.z = float(drop_z)
            drop_pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

            success, ik_msg = self._try_ik(drop_pose)
            if not success:
                drop_pose.position.z = float(drop_z + 0.05)
                success, ik_msg = self._try_ik(drop_pose)
            if not success:
                success, ik_msg = self._try_ik(drop_pose, use_orientation=False)

            if success:
                self.get_logger().info(
                    f'[POSITION] IK OK for drop pose '
                    f'({drop_x:.3f}, {drop_y:.3f}, {float(drop_pose.position.z):.3f})')
                break

            if nudge_i >= MAX_DROP_NUDGES:
                self.get_logger().error(
                    f'[POSITION] IK failed after {MAX_DROP_NUDGES} nudges — '
                    f'dropping where arm is')
                self.state = TaskState.DROPPING
                return

            self.get_logger().info(
                f'[POSITION] IK failed ({ik_msg}), nudging closer '
                f'({nudge_i + 1}/{MAX_DROP_NUDGES})...')
            self._bin_nudge_closer(bin_base[0], bin_base[1])

            self._spin_for(1.0)
            det = self.latest_tag_detection
            if det is not None:
                self.latest_tag_detection = None
                u = int(det.bbox.center.position.x)
                v = int(det.bbox.center.position.y)
                new_base = self._get_position_in_base_link(u, v)
                if new_base is not None:
                    bin_base = new_base
                    approach_xy = np.array([bin_base[0], bin_base[1]])
                    approach_dist = np.linalg.norm(approach_xy)
                    if approach_dist > 0.01:
                        approach_unit = approach_xy / approach_dist
                    else:
                        approach_unit = np.array([1.0, 0.0])
                    drop_z = bin_base[2] + self.BIN_HALF_HEIGHT + self.DROP_HEIGHT_ABOVE
                    drop_x = bin_base[0] + self.BIN_DEPTH_OFFSET * approach_unit[0]
                    drop_y = bin_base[1] + self.BIN_DEPTH_OFFSET * approach_unit[1]
                    self.get_logger().info(
                        f'[POSITION] Updated drop target: '
                        f'({drop_x:.3f}, {drop_y:.3f}, {drop_z:.3f})')

        pre_drop_pose = Pose()
        pre_drop_pose.position.x = drop_x
        pre_drop_pose.position.y = drop_y
        pre_drop_pose.position.z = float(drop_pose.position.z + 0.10)
        pre_drop_pose.orientation = drop_pose.orientation

        self.get_logger().info('[POSITION] Moving to pre-drop position...')
        if not self._bin_plan_and_execute(pre_drop_pose, 'PRE-DROP'):
            self.get_logger().warn(
                '[POSITION] Pre-drop move failed, trying drop directly')

        self.get_logger().info('[POSITION] Moving to drop position...')
        if self._bin_plan_and_execute(drop_pose, 'DROP-POS'):
            self.get_logger().info('[POSITION] Arm in position above bin!')
        else:
            self.get_logger().warn(
                '[POSITION] Drop position move failed — dropping where arm is')

        self._spin_for(1.0)
        self.state = TaskState.DROPPING

    def _bin_nudge_closer(self, tag_x, tag_y):
        cmd = Twist()
        cmd.linear.x = 0.06
        if abs(tag_y) > 0.05:
            cmd.angular.z = float(np.clip(tag_y * 0.5, -0.15, 0.15))

        self.get_logger().info(
            f'[NUDGE-CLOSER] tag_base=({tag_x:.3f}, {tag_y:.3f}), '
            f'cmd=(vx={cmd.linear.x:.2f}, wz={cmd.angular.z:.2f}), dur=0.8s')

        deadline = time.time() + 0.8
        while time.time() < deadline and rclpy.ok():
            self.cmd_vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
        self._stop_base()
        self._spin_for(0.5)

    # ── State: DROPPING (Phase 3b) ────────────────────────────────────────

    def _state_dropping(self):
        bin_name = getattr(self, '_bin_target_name', self.current_object_name)
        self.get_logger().info('=' * 55)
        self.get_logger().info(f'[DROP] Releasing object into {bin_name} bin...')

        msg = Float64MultiArray()
        msg.data = [0.0]
        for _ in range(20):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(1.5)

        self.get_logger().info('[DROP] Object released!')

        self.get_logger().info('[DROP] Returning arm to home position...')
        self._go_home(duration=3.0)

        self._set_camera_tilt(0.3)

        self.get_logger().info('=' * 55)
        self.get_logger().info(
            f'[DROP] {self.current_object_name.upper()} dropped in {bin_name} bin! '
            f'Task complete.')
        self.get_logger().info('=' * 55)

        self.state = TaskState.TASK_COMPLETE

    # ── State: TASK_COMPLETE ─────────────────────────────────────────────────

    def _state_task_complete(self):
        self.get_logger().info('=' * 55)
        self.get_logger().info('[DONE] Pick-and-bring cycle complete!')
        self.get_logger().info('=' * 55)

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

            # Record waypoints during navigation
            if self.state in (TaskState.SCANNING_OBJECT,
                              TaskState.CENTERING_OBJECT,
                              TaskState.APPROACHING_OBJECT,
                              TaskState.SCANNING_PERSON,
                              TaskState.CENTERING_PERSON,
                              TaskState.APPROACHING_PERSON,
                              TaskState.SCANNING_BIN,
                              TaskState.CENTERING_BIN,
                              TaskState.APPROACHING_BIN):
                self._maybe_record_waypoint()

            # Keep gripper closed during person-finding and bin-finding phases
            if (self._phase in ('person', 'bin') and
                    self.state not in (TaskState.WAITING_FOR_DROP,
                                       TaskState.DROPPING,
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
                                TaskState.SCANNING_PERSON,
                                TaskState.SCANNING_BIN):
                self._state_scanning()
            elif self.state in (TaskState.CENTERING_OBJECT,
                                TaskState.CENTERING_PERSON,
                                TaskState.CENTERING_BIN):
                self._state_centering()
            elif self.state in (TaskState.APPROACHING_OBJECT,
                                TaskState.APPROACHING_PERSON,
                                TaskState.APPROACHING_BIN):
                self._state_approaching()
            elif self.state == TaskState.OBJECT_REACHED:
                self._state_object_reached()
            elif self.state == TaskState.GRASPING:
                self._state_grasping()
            elif self.state == TaskState.WAITING_FOR_DROP:
                self._state_waiting_for_drop()
            elif self.state == TaskState.POSITIONING:
                self._state_positioning()
            elif self.state == TaskState.DROPPING:
                self._state_dropping()
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
