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
  7. RETURNING_HOME      — follow recorded waypoints back to start pose
  8. WAITING_FOR_DROP     — wait for voice command to release the object

Build
cd ros2_ws && colcon build --packages-select tidybot_perception tidybot_bringup && source install/setup.bash

Terminal 1: robot/rviz bringup
ros2 launch tidybot_bringup sim.launch.py scene:=scene_banana_test.xml show_mujoco_viewer:=false

Terminal 2: nav + grasp (no voice)
ros2 launch tidybot_bringup nav.launch.py target_object:=banana user_command:=get

With voice:
Terminal 2:
ros2 launch tidybot_bringup nav.launch.py skip_voice:=false

Terminal 3:
    ros2 run tidybot_bringup voice_command.py
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

from common.constants import YOLO_CLASS_MAP, GRIPPER_OPEN_POS
from common.perception import PerceptionMixin
from common.grasp_pipeline import GraspMixin
from common.arm_helpers import ArmHelpersMixin
from common.base_control import BaseControlMixin


class ExploreState(Enum):
    WAITING_FOR_COMMAND = -1
    SCANNING    = 0
    CENTERING   = 6
    APPROACHING = 3
    COMPLETE    = 4
    GRASPING    = 5
    RETURNING_HOME = 7
    WAITING_FOR_DROP = 8


class ExploreAndFind(PerceptionMixin, GraspMixin, ArmHelpersMixin,
                     BaseControlMixin, Node):

    # ── Depth filtering ──────────────────────────────────────────────────────
    MIN_DEPTH_M       = 0.40
    MAX_DEPTH_M       = 5.00

    # ── 360 scan ─────────────────────────────────────────────────────────────
    SCAN_ANGULAR_VEL = -0.3
    SCAN_SETTLE_TIME = 0.5
    MAX_SCAN_ATTEMPTS = 3

    # ── Centering ────────────────────────────────────────────────────────────
    CENTER_PIXEL_THRESH   = 80
    CENTER_PIXEL_THRESH_V = 60
    CENTER_ANGULAR_VEL    = 0.25
    CENTER_TILT_STEP      = 0.02
    CENTER_TIMEOUT        = 15.0

    # ── Object detection ─────────────────────────────────────────────────────
    MIN_CONFIDENCE     = 0.25
    MIN_DETECTION_AREA = 50
    APPROACH_DIST      = 0.35
    DETECT_WINDOW      = 2.0
    DETECT_COUNT_REQ   = 1

    # ── Approach ─────────────────────────────────────────────────────────────
    APPROACH_LINEAR_SPEED = 0.05
    APPROACH_ANGULAR_GAIN = 0.003
    APPROACH_MAX_ANGULAR  = 0.3
    APPROACH_TIMEOUT      = 60.0
    APPROACH_TILT_STEP    = 0.03

    # ── Return home ──────────────────────────────────────────────────────────
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
    GRASP_Z_OFFSET               = 0.00
    GRASP_X_OFFSET               = -0.05
    GRASP_Y_OFFSET               = -0.05
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

    # ── Sweet spot ───────────────────────────────────────────────────────────
    GRASP_SWEET_SPOT_X      = 0.04
    GRASP_SWEET_SPOT_Y      = -0.26
    GRASP_SWEET_SPOT_RADIUS = 0.12
    GRASP_NUDGE_SPEED       = 0.04

    # ── Minimal phase params (needed by _grasp_detect_and_center) ────────────
    PHASE_PARAMS = {
        'object': {
            'center_pixel_thresh': 80,
            'center_pixel_thresh_v': 60,
            'approach_stop_dist': 0.35,
            'approach_linear_speed': 0.05,
            'initial_tilt': 0.7854,
            'scan_tilt': 0.3,
        },
    }

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
        self._phase            = 'object'
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

        # Command queue
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

        # Waypoint recording for return-home
        self.start_pose = None
        self.waypoints = []
        self.return_waypoints = []

        # Drop command flag
        self._drop_pending = False

        # IK nudge target (set here for GraspMixin._grasp_ik_nudge)
        self._ik_nudge_target_x = -0.12  # right arm default

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, '/camera/depth/image_raw',
                                 self._depth_cb, sensor_qos)
        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 self._camera_info_cb, sensor_qos)
        self.create_subscription(Image, '/camera/rgb/image_raw',
                                 self._rgb_cb, sensor_qos)

        yolo_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Detection2DArray, '/objbbox', self._yolo_bbox_cb, yolo_qos)

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

        if self.skip_voice:
            obj = (self.get_parameter('target_object')
                   .get_parameter_value().string_value)
            act = (self.get_parameter('user_command')
                   .get_parameter_value().string_value)
            self.command_queue.append((act, obj))

        # Publishers
        self.object_marker_pub = self.create_publisher(
            MarkerArray, '/object_marker', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pan_tilt_pub = self.create_publisher(
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

    # ── Phase helper (minimal, for GraspMixin compatibility) ─────────────────

    def _pp(self, key):
        return self.PHASE_PARAMS[self._phase][key]

    # ── Object detection ─────────────────────────────────────────────────────

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

    def _refine_object_position(self):
        det = self.latest_yolo_detection
        if det is None:
            return
        self.latest_yolo_detection = None

        u = int(det.bbox.center.position.x)
        v = int(det.bbox.center.position.y)
        new_pos = self._estimate_world_position(u, v)
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

    # ── Voice command callbacks ──────────────────────────────────────────────

    DROP_ACTIONS = {'drop', 'release', 'let go', 'put down', 'place'}

    def _accepting_voice(self):
        return (not self.skip_voice or
                self.state in (ExploreState.WAITING_FOR_COMMAND,
                               ExploreState.WAITING_FOR_DROP))

    def _user_command_cb(self, msg: String):
        action = msg.data.strip().lower()
        if not action:
            return

        if self.state == ExploreState.WAITING_FOR_DROP:
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

        if self.state == ExploreState.WAITING_FOR_DROP:
            self.get_logger().info(
                f'[VOICE] New task while holding — releasing first')
            self._set_gripper(0.0)
            self._spin_for(1.0)
            self._go_home(duration=3.0)
            self.state = ExploreState.WAITING_FOR_COMMAND

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

        # Tilt camera 45 degrees down and check if object is already visible
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

    # ── Safe stop ────────────────────────────────────────────────────────────

    def _safe_stop_cb(self, msg):
        self.get_logger().warn('[SAFE STOP] Received safe_stop signal!')
        self._safe_stop_requested = True

    def _execute_safe_shutdown(self):
        self.get_logger().warn('[SAFE STOP] Executing safe shutdown...')

        for _ in range(10):
            self.cmd_vel_pub.publish(Twist())
            time.sleep(0.05)

        grip_msg = Float64MultiArray()
        grip_msg.data = [0.0]
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

    # ── State: SCANNING ──────────────────────────────────────────────────────

    def _start_scan(self):
        self.scan_accumulated  = 0.0
        self.scan_last_heading = None
        self.scan_settle_start = None
        self.scan_attempt_count += 1
        self._set_camera_tilt(0.3)
        self.state = ExploreState.SCANNING
        self.get_logger().info(
            f'[SCAN] Starting 360 rotation scan '
            f'(attempt {self.scan_attempt_count}/{self.MAX_SCAN_ATTEMPTS})...')

    def _state_scanning(self):
        if self.object_world_pos is not None:
            self._stop_base()
            self.get_logger().info(
                '[SCAN] Object spotted mid-scan — centering on it.')
            self._start_centering()
            return

        if self.scan_settle_start is not None:
            if time.time() - self.scan_settle_start >= self.SCAN_SETTLE_TIME:
                if self.scan_attempt_count >= self.MAX_SCAN_ATTEMPTS:
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

    # ── State: CENTERING ─────────────────────────────────────────────────────

    def _start_centering(self):
        self.center_start_time = time.time()
        self.state = ExploreState.CENTERING
        self.get_logger().info('[CENTER] Rotating to center object in camera...')

    def _state_centering(self):
        now = time.time()

        if now - self.center_start_time > self.CENTER_TIMEOUT:
            self._stop_base()
            if self.object_world_pos is not None:
                self.get_logger().warn(
                    '[CENTER] Timeout — object known, approaching anyway.')
                self.approach_start_time = time.time()
                self.state = ExploreState.APPROACHING
            else:
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

            new_pos = self._estimate_world_position(u, v)
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

            if abs(offset_y) > self.CENTER_PIXEL_THRESH_V:
                tilt_adj = offset_y / (img_h / 2.0) * self.CENTER_TILT_STEP
                self._set_camera_tilt(self.current_tilt + tilt_adj)

            h_centered = abs(offset_x) < self.CENTER_PIXEL_THRESH
            v_centered = abs(offset_y) < self.CENTER_PIXEL_THRESH_V

            if h_centered and v_centered:
                self._stop_base()
                self.get_logger().info(
                    f'[CENTER] Centered (h={offset_x:.0f}px, v={offset_y:.0f}px) '
                    f'— approaching.')
                self.approach_start_time = time.time()
                self.state = ExploreState.APPROACHING
                return

            if not h_centered:
                cmd = Twist()
                frac = min(abs(offset_x) / (img_w / 2.0), 1.0)
                speed = self.CENTER_ANGULAR_VEL * (0.2 + 0.8 * frac * frac)
                cmd.angular.z = -speed if offset_x > 0 else speed
                self.cmd_vel_pub.publish(cmd)
        else:
            if self.object_world_pos is None:
                return

            ox, oy = self.object_world_pos
            bearing = np.arctan2(oy - by, ox - bx)
            actual_heading = btheta - np.pi / 2
            heading_error = self._normalize_angle(bearing - actual_heading)

            if abs(heading_error) < np.radians(5):
                self._stop_base()
                return

            cmd = Twist()
            cmd.angular.z = float(np.clip(
                1.0 * heading_error, -self.CENTER_ANGULAR_VEL,
                self.CENTER_ANGULAR_VEL))
            self.cmd_vel_pub.publish(cmd)

    # ── State: APPROACHING ───────────────────────────────────────────────────

    def _state_approaching(self):
        now = time.time()

        if now - self.approach_start_time > self.APPROACH_TIMEOUT:
            self._stop_base()
            self.get_logger().warn('[APPROACH] Timeout — stopping.')
            self.state = ExploreState.COMPLETE
            return

        pose = self.get_base_pose()
        if pose is None:
            return
        bx, by, btheta = pose
        actual_heading = btheta - np.pi / 2

        if self.object_world_pos is not None:
            ox, oy = self.object_world_pos
            dist = np.hypot(ox - bx, oy - by)
            if dist < self.APPROACH_DIST:
                self._stop_base()
                self.get_logger().info(
                    f'[APPROACH] Reached object at dist={dist:.2f}m')
                self.state = ExploreState.COMPLETE
                return

        det = self.latest_yolo_detection
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

                angular = -self.APPROACH_ANGULAR_GAIN * offset_x
                cmd.angular.z = float(np.clip(
                    angular, -self.APPROACH_MAX_ANGULAR, self.APPROACH_MAX_ANGULAR))

                offset_y = v - img_center_y
                if abs(offset_y) > self.CENTER_PIXEL_THRESH_V:
                    tilt_adj = offset_y / (img_h / 2.0) * self.APPROACH_TILT_STEP
                    self._set_camera_tilt(self.current_tilt + tilt_adj)

            new_pos = self._estimate_world_position(u, v)
            if new_pos is not None and self.object_world_pos is not None:
                shift = np.hypot(
                    new_pos[0] - self.object_world_pos[0],
                    new_pos[1] - self.object_world_pos[1])
                if shift < 1.5:
                    self.object_world_pos = new_pos
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
                    if depth_m < self.APPROACH_DIST:
                        self._stop_base()
                        self.get_logger().info(
                            f'[APPROACH] Depth={depth_m:.2f}m < '
                            f'{self.APPROACH_DIST}m — reached!')
                        self.state = ExploreState.COMPLETE
                        return
        else:
            if self.object_world_pos is not None:
                ox, oy = self.object_world_pos
                bearing = np.arctan2(oy - by, ox - bx)
                heading_error = self._normalize_angle(bearing - actual_heading)
                cmd.angular.z = float(np.clip(
                    1.0 * heading_error,
                    -self.APPROACH_MAX_ANGULAR,
                    self.APPROACH_MAX_ANGULAR))

        cmd.linear.x = self.APPROACH_LINEAR_SPEED
        self.cmd_vel_pub.publish(cmd)

    # ── State: COMPLETE ──────────────────────────────────────────────────────

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
            self.get_logger().info(
                f'{name} reached (no world pos, but close enough)')
        self.get_logger().info('=' * 55)

        if self.current_action in ('get', 'grab', 'pick', 'grasp'):
            self.get_logger().info(
                f'[COMPLETE] Action="{self.current_action}" — transitioning to GRASPING')
            self.state = ExploreState.GRASPING
        else:
            self.get_logger().info(
                f'[COMPLETE] Action="{self.current_action}" — done.')
            self._start_next_command()

    # ── State: GRASPING ──────────────────────────────────────────────────────

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
                return  # success

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

        # Sweet-spot nudge
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

        # IK check with nudge retries
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
                    '[GRASP] All IK nudge attempts failed')
                return False

        # Execute grasp
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

        self.get_logger().info('=' * 55)
        self.get_logger().info('[GRASP] Grasp complete — holding object!')
        self.get_logger().info('=' * 55)

        # Retract arm high
        self._retract_holding()

        # Transition to return-home
        self._start_returning_home()
        return True

    # ── State: RETURNING_HOME ────────────────────────────────────────────────

    def _start_returning_home(self):
        self.return_waypoints = list(reversed(self.waypoints))
        if not self.return_waypoints and self.start_pose is not None:
            self.return_waypoints = [(self.start_pose[0], self.start_pose[1])]

        if not self.return_waypoints:
            self.get_logger().warn('[RETURN] No waypoints recorded — skipping return')
            self.state = ExploreState.WAITING_FOR_DROP
            return

        self.get_logger().info(
            f'[RETURN] Following {len(self.return_waypoints)} waypoints back...')
        self.state = ExploreState.RETURNING_HOME

    def _state_returning_home(self):
        # Keep gripper closed
        grip_msg = Float64MultiArray()
        grip_msg.data = [1.0]
        self.gripper_pub.publish(grip_msg)

        if not self.return_waypoints:
            self._stop_base()
            self.get_logger().info('[RETURN] Reached starting position!')
            self.state = ExploreState.WAITING_FOR_DROP
            self.get_logger().info(
                '[RETURN] Waiting for voice command (drop/release)...')
            return

        target_x, target_y = self.return_waypoints[0]
        pose = self.get_base_pose()
        if pose is None:
            return
        bx, by, btheta = pose
        actual_heading = btheta - np.pi / 2

        dx = target_x - bx
        dy = target_y - by
        dist = np.hypot(dx, dy)

        if dist < self.RETURN_WAYPOINT_REACH:
            self.return_waypoints.pop(0)
            return

        bearing = np.arctan2(dy, dx)
        heading_error = self._normalize_angle(bearing - actual_heading)

        cmd = Twist()
        cmd.angular.z = float(np.clip(
            self.RETURN_ANGULAR_GAIN * heading_error,
            -self.RETURN_MAX_ANGULAR, self.RETURN_MAX_ANGULAR))

        if abs(heading_error) < np.radians(30):
            cmd.linear.x = self.RETURN_LINEAR_SPEED
        else:
            cmd.linear.x = 0.0

        self.cmd_vel_pub.publish(cmd)

    # ── State: WAITING_FOR_DROP ──────────────────────────────────────────────

    def _state_waiting_for_drop(self):
        # Keep gripper closed
        grip_msg = Float64MultiArray()
        grip_msg.data = [1.0]
        self.gripper_pub.publish(grip_msg)

        if not self._drop_pending:
            return

        self.get_logger().info('[DROP] Drop command received — releasing object')

        # Open gripper
        self._set_gripper(0.0)
        self._spin_for(1.0)

        # Return arm to home
        self.get_logger().info('[DROP] Returning arm to home position')
        self._go_home(duration=3.0)

        self._set_camera_tilt(0.3)

        self.get_logger().info('=' * 55)
        self.get_logger().info('[DROP] Object released. Task complete!')
        self.get_logger().info('=' * 55)

        self._drop_pending = False
        self._start_next_command()

    # ── Main loop ────────────────────────────────────────────────────────────

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

            if (self.state == ExploreState.SCANNING and
                    self.object_world_pos is None):
                self._check_for_object()

            if self.state in (ExploreState.SCANNING,
                              ExploreState.CENTERING,
                              ExploreState.APPROACHING):
                self._maybe_record_waypoint()

            # State dispatch
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
            elif self.state == ExploreState.RETURNING_HOME:
                self._state_returning_home()
            elif self.state == ExploreState.WAITING_FOR_DROP:
                self._state_waiting_for_drop()


def main(args=None):
    rclpy.init(args=args)
    node = ExploreAndFind()
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
