#!/usr/bin/env python3
"""
Navigate to YOLO target, grasp it, then return to initial odometry pose.

Extends NavigateAndReturnNode by inserting a top-down grasp sequence
between arriving at the target and returning home.  Optionally takes in
a voice command (via microphone + Gemini) to select the target object.

State machine:
    INIT -> VOICE_COMMAND      (optional: listen, transcribe, set target)
         -> NAVIGATE_TO_TARGET (seek target via YOLO)
         -> GRASPING           (stop base, run grasp pipeline)
         -> RETURNING_HOME     (drive back with object)
         -> DONE               (open gripper, stow arm, hold)

Requires:
    - ros2 launch tidybot_bringup sim.launch.py  (or real.launch.py use_planner:=true)
    - ros2 run object_classification classifier
    - (voice mode) ros2 run tidybot_control microphone_node
    - (voice mode) GEMINI_API_KEY env var or gemini_api_key param

Usage:
    ros2 run tidybot_bringup task_navigate_grasp_return.py
    ros2 run tidybot_bringup task_navigate_grasp_return.py --ros-args -p sim:=false
    ros2 run tidybot_bringup task_navigate_grasp_return.py --ros-args \
        -p target_class_id:=46 -p grasp_arm:=right

    # With voice command:
    ros2 run tidybot_bringup task_navigate_grasp_return.py --ros-args \
        -p use_voice_command:=true -p gemini_api_key:=YOUR_KEY
"""

import os
import tempfile
import time
import wave

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: F401
from scipy.spatial.transform import Rotation

import tf2_ros
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Float64MultiArray
from tidybot_msgs.msg import ArmCommand
from tidybot_msgs.srv import PlanToTarget, AudioRecord

from task_navigate_with_avoidance import NavigationTaskNode


# ---------------------------------------------------------------------------
# Grasp geometry helpers (inlined from tidybot_perception.grasp_geometry)
# ---------------------------------------------------------------------------
GRIPPER_MAX_OPENING_M = 0.074


def yaw_to_grasp_quaternion(yaw_rad):
    """Fingers-down quaternion with yaw around base_link Z."""
    R_down = Rotation.from_quat([0.5, 0.5, -0.5, 0.5])
    R_yaw = Rotation.from_euler('z', yaw_rad)
    R_grasp = R_yaw * R_down
    qx, qy, qz, qw = R_grasp.as_quat()
    return float(qw), float(qx), float(qy), float(qz)


# YOLO COCO class name -> string ID
YOLO_CLASS_IDS = {
    'banana': '46',
    'apple': '47',
    'orange': '49',
}

# Grasp constants
SETTLE_TIME = 1.0
GRIPPER_CLOSE_REPEATS = 20
GRIPPER_CLOSE_WAIT = 2.0
MIN_DEPTH_M = 0.1
MAX_DEPTH_M = 2.0
MIN_POINTS = 20
TABLE_Z_MIN = 0.005
OUTLIER_Z_THRESH = 0.05
SLEEP_POSE = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]


class NavigateGraspReturnNode(NavigationTaskNode):
    """Navigate to target, grasp it via top-down pick, then return home."""

    # Return-home parameters (from NavigateAndReturnNode)
    RETURN_ENABLED = True
    RETURN_POSITION_TOLERANCE = 0.12
    RETURN_ANGLE_TOLERANCE = 0.20
    RETURN_TURN_STEP_RAD = 0.10

    # Override stop distance so banana lands in arm's reachable zone (~0.30-0.40m in base_link Y)
    # Depth camera is offset forward from base_link, so depth distance < base_link Y distance.
    # At 0.39m depth stop, banana was at y=-0.515 (too far). Need to get ~15cm closer.
    SEEK_TARGET_STOP_DISTANCE_M = 0.20
    APPROACH_STOP_DISTANCE_BUFFER_M = 0.05

    # More persistent target tracking — don't drop back to search so easily.
    # Parent defaults: LOST_FRAMES=12(->20 for YOLO), MEMORY=1.5s, MAX_AGE=1.5s
    APPROACH_LOST_FRAMES = 40         # wait 40 frames (~4s) before giving up approach
    APPROACH_TARGET_MEMORY_S = 4.0    # keep approaching last known heading for 4s
    YOLO_DETECTION_MAX_AGE_S = 3.0    # accept detections up to 3s old

    # Wider center tolerance so the robot drives forward more and turns less.
    # Prevents swerving from YOLO bbox jitter on real hardware.
    # APPROACH_CENTER_TOLERANCE = 0.25
    # APPROACH_CENTER_HYSTERESIS = 0.10
    # SEEK_HEADING_GAIN = 0.25          # gentler heading correction
    # SIMPLE_CENTER_TURN_STEP_RAD = 0.05  # smaller turn steps
    # SIMPLE_FORWARD_STEP_M = 0.15     # bigger forward steps when centered

    # Grasp parameters
    GRASP_ARM = 'right'
    GRASP_MOVE_DURATION_SIM = 2.0
    GRASP_MOVE_DURATION_REAL = 3.0
    PRE_GRASP_HEIGHT = 0.10
    LIFT_HEIGHT = 0.15
    Z_OFFSET = -0.02
    Z_BIAS = 0.0
    BBOX_PAD = 1.0

    def __init__(self):
        super().__init__()

        # --- sim/real ---
        self.is_sim = bool(self.declare_parameter('sim', True).value)

        # --- return-home params ---
        self.return_enabled = bool(
            self.declare_parameter('return_enabled', self.RETURN_ENABLED).value)
        self.return_position_tolerance = float(
            self.declare_parameter('return_position_tolerance',
                                   self.RETURN_POSITION_TOLERANCE).value)
        self.return_angle_tolerance = float(
            self.declare_parameter('return_angle_tolerance',
                                   self.RETURN_ANGLE_TOLERANCE).value)
        self.return_turn_step_rad = float(
            self.declare_parameter('return_turn_step_rad',
                                   self.RETURN_TURN_STEP_RAD).value)

        # --- grasp params ---
        self.grasp_arm = str(
            self.declare_parameter('grasp_arm', self.GRASP_ARM).value)
        default_dur = (self.GRASP_MOVE_DURATION_SIM if self.is_sim
                       else self.GRASP_MOVE_DURATION_REAL)
        self.grasp_move_duration = float(
            self.declare_parameter('grasp_duration', default_dur).value)
        self.pre_grasp_height = float(
            self.declare_parameter('pre_grasp_height', self.PRE_GRASP_HEIGHT).value)
        self.lift_height = float(
            self.declare_parameter('lift_height', self.LIFT_HEIGHT).value)
        self.z_offset = float(
            self.declare_parameter('z_offset', self.Z_OFFSET).value)
        self.z_bias = float(
            self.declare_parameter('z_bias', self.Z_BIAS).value)
        self.bbox_pad = float(
            self.declare_parameter('bbox_pad', self.BBOX_PAD).value)

        self.grasp_target_name = 'banana'
        cid = str(self.target_class_id) if hasattr(self, 'target_class_id') else '46'
        for name, sid in YOLO_CLASS_IDS.items():
            if sid == cid:
                self.grasp_target_name = name
                break

        # --- home-pose tracking ---
        self.home_pose_set = False
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_theta = 0.0
        self.return_home_active = False
        self.return_home_complete = False

        # --- grasp state ---
        self.grasp_active = False
        self.grasp_complete = False
        self.grasp_success = False

        # --- TF2 ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- camera info subscriber (needed for depth back-projection) ---
        self.camera_info_data = None
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self._camera_info_cb, 10)

        # --- arm / gripper publishers ---
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, f'/{self.grasp_arm}_gripper/cmd', 10)
        self.arm_pub = self.create_publisher(
            ArmCommand, f'/{self.grasp_arm}_arm/cmd', 10)

        # --- IK planner service (separate callback group so calls work from timer) ---
        self._service_cb_group = MutuallyExclusiveCallbackGroup()
        self.plan_client = self.create_client(
            PlanToTarget, '/plan_to_target',
            callback_group=self._service_cb_group)

        # --- real hardware extras ---
        if not self.is_sim:
            self.depth_camera_info = None
            self.create_subscription(
                CameraInfo, '/camera/depth/camera_info',
                self._depth_info_cb, 10)
            self.joint_states_received = False
            self.current_joint_positions = {}
            self.create_subscription(
                JointState, '/joint_states', self._js_callback, 10)

        # Store raw YOLO detections for the grasp pipeline
        self._raw_yolo_detections = None

        # --- voice command ---
        self.use_voice_command = bool(
            self.declare_parameter('use_voice_command', False).value)
        self.voice_record_duration = float(
            self.declare_parameter('voice_record_duration', 5.0).value)
        self.gemini_api_key = str(
            self.declare_parameter('gemini_api_key', '').value)
        if not self.gemini_api_key:
            self.gemini_api_key = os.environ.get('GEMINI_API_KEY', '')
        self.voice_command_received = not self.use_voice_command

        if self.use_voice_command:
            self.mic_client = self.create_client(
                AudioRecord, '/microphone/record',
                callback_group=self._service_cb_group)

        self.get_logger().info(
            f'NavigateGraspReturn: arm={self.grasp_arm}, '
            f'target={self.grasp_target_name}, sim={self.is_sim}, '
            f'voice_cmd={self.use_voice_command}')

    # ------------------------------------------------------------------
    # Disable depth-based obstacle avoidance — on real hardware the depth
    # image picks up the target, table edges, arms, etc. as "obstacles"
    # causing the robot to freeze or swerve.  YOLO approach already stops
    # at the right distance via depth-to-target.
    # ------------------------------------------------------------------

    def is_obstacle_blocking(self) -> bool:
        return False

    def _handle_simple_yolo_approach(self):
        """Dead-simple approach: see banana → turn toward it → drive straight."""
        now = self.get_clock().now()
        target_visible = self.red_target_visible and self.red_bbox_color is not None

        if not target_visible:
            # Lost sight — hold position briefly, memory will keep us in approach mode
            self.publish_hold_position()
            return

        # Compute horizontal error from bbox center
        rx, _, rw, _ = self.red_bbox_color
        img_w = float(max(self.latest_color_shape[1], 1)) if self.latest_color_shape is not None else 640.0
        x_error = ((rx + 0.5 * rw) / img_w - 0.5) * 2.0  # -1..+1

        distance_m = self.red_target_distance_m
        if distance_m is not None:
            self.last_target_distance_m = distance_m
        else:
            distance_m = self.last_target_distance_m

        # Check if we're close enough to stop
        stop_dist = self.seek_target_stop_distance_m + self.approach_stop_distance_buffer_m
        if distance_m is not None and distance_m <= stop_dist:
            self.publish_hold_position()
            if not self.red_target_reached_logged:
                self.get_logger().info(f'Target reached at {distance_m:.2f}m. Stopping.')
                self.red_target_reached_logged = True
            return
        self.red_target_reached_logged = False

        # Compute heading toward the banana
        bearing = self.approach_turn_sign * 0.3 * x_error
        target_theta = self.current_theta + np.clip(bearing, -0.15, 0.15)

        # Always drive forward while correcting heading (no separate rotate step)
        step = 0.12
        if distance_m is not None:
            step = float(np.clip(distance_m - self.seek_target_stop_distance_m, 0.04, 0.15))
        target_x = self.current_x + step * np.cos(target_theta)
        target_y = self.current_y + step * np.sin(target_theta)
        self.publish_pose_target(target_x, target_y, target_theta)

        # Auto-tilt camera to keep banana in view
        if self.auto_tilt_enable and self.latest_color_shape is not None:
            _, by, _, bh = self.red_bbox_color
            img_h = float(max(self.latest_color_shape[0], 1))
            y_frac = (by + 0.5 * bh) / img_h
            if y_frac > 0.8:
                self._nudge_tilt(down=True, scale=0.5)
            elif y_frac < 0.4:
                self._nudge_tilt(down=False, scale=0.5)

    # ------------------------------------------------------------------
    # Additional callbacks
    # ------------------------------------------------------------------

    def yolo_detections_callback(self, msg):
        """Store raw detections for grasp pipeline, then delegate to parent."""
        self._raw_yolo_detections = msg
        super().yolo_detections_callback(msg)

    def _camera_info_cb(self, msg):
        self.camera_info_data = msg

    def _depth_info_cb(self, msg):
        self.depth_camera_info = msg

    def _js_callback(self, msg):
        self.joint_states_received = True
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    # ------------------------------------------------------------------
    # Home-pose recording (same as NavigateAndReturnNode)
    # ------------------------------------------------------------------

    def odom_callback(self, msg):
        super().odom_callback(msg)
        if not self.home_pose_set and self.odom_received:
            self.home_x = self.current_x
            self.home_y = self.current_y
            self.home_theta = self.current_theta
            self.home_pose_set = True
            self.get_logger().info(
                f'Home pose recorded: ({self.home_x:.2f}, {self.home_y:.2f}), '
                f'heading {np.rad2deg(self.home_theta):.0f} deg')

    def handle_init(self, current_time):
        if self.odom_received and not self.home_pose_set:
            self.home_x = self.current_x
            self.home_y = self.current_y
            self.home_theta = self.current_theta
            self.home_pose_set = True
            self.get_logger().info(
                f'Home pose recorded: ({self.home_x:.2f}, {self.home_y:.2f}), '
                f'heading {np.rad2deg(self.home_theta):.0f} deg')
        super().handle_init(current_time)

    # ------------------------------------------------------------------
    # Voice command pipeline
    # ------------------------------------------------------------------

    def _run_voice_command(self):
        """Record audio via microphone service, then parse with Gemini."""
        self.get_logger().info('=' * 50)
        self.get_logger().info('VOICE COMMAND: Waiting for microphone service...')
        self.get_logger().info('=' * 50)

        if not self.mic_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(
                'Microphone service not available — using default target.')
            self.voice_command_received = True
            return

        # Start recording
        req = AudioRecord.Request()
        req.start = True
        future = self.mic_client.call_async(req)
        if not self._wait_for_future(future, 5.0):
            self.get_logger().error('Failed to start recording.')
            self.voice_command_received = True
            return
        resp = future.result()
        if not resp.success:
            self.get_logger().error(f'Mic start failed: {resp.message}')
            self.voice_command_received = True
            return

        self.get_logger().info(
            f'Recording for {self.voice_record_duration:.0f}s — speak now!')
        self._spin_for(self.voice_record_duration)

        # Stop recording
        req = AudioRecord.Request()
        req.start = False
        future = self.mic_client.call_async(req)
        if not self._wait_for_future(future, 10.0):
            self.get_logger().error('Failed to stop recording.')
            self.voice_command_received = True
            return
        resp = future.result()
        if not resp.success or len(resp.audio_data) == 0:
            self.get_logger().warn('No audio recorded — using default target.')
            self.voice_command_received = True
            return

        self.get_logger().info(
            f'Recorded {resp.duration:.2f}s of audio. Sending to Gemini...')

        target = self._parse_voice_with_gemini(
            resp.audio_data, resp.sample_rate)
        if target and target != 'unknown':
            self._set_target_from_voice(target)
        else:
            self.get_logger().warn(
                f'Could not parse target ("{target}") — keeping default: '
                f'{self.grasp_target_name}')

        self.voice_command_received = True

    def _parse_voice_with_gemini(self, audio_data, sample_rate):
        """Convert audio to WAV, send to Gemini for transcription + parsing."""
        try:
            import google.generativeai as genai

            if not self.gemini_api_key:
                self.get_logger().error('No Gemini API key set.')
                return None

            genai.configure(api_key=self.gemini_api_key)

            # Save float32 PCM -> int16 WAV temp file
            wav_path = os.path.join(tempfile.gettempdir(), 'tidybot_voice.wav')
            audio_int16 = (np.array(audio_data, dtype=np.float32) * 32767
                           ).clip(-32768, 32767).astype(np.int16)
            with wave.open(wav_path, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(int(sample_rate))
                wf.writeframes(audio_int16.tobytes())

            audio_file = genai.upload_file(wav_path)
            model = genai.GenerativeModel('gemini-2.0-flash')

            known_objects = ', '.join(YOLO_CLASS_IDS.keys())
            prompt = (
                'Listen to this audio clip. The user is telling a robot which '
                'object to pick up. The known objects are: '
                f'{known_objects}. '
                'Respond with ONLY the object name in lowercase '
                "(e.g. 'banana'). If you cannot determine the object, "
                "respond with 'unknown'."
            )

            response = model.generate_content([prompt, audio_file])
            target = response.text.strip().lower().strip("'\".")
            self.get_logger().info(f'Gemini response: "{target}"')

            # Clean up
            try:
                os.unlink(wav_path)
            except OSError:
                pass

            return target

        except Exception as e:
            self.get_logger().error(f'Gemini voice parsing failed: {e}')
            return None

    def _set_target_from_voice(self, target_name):
        """Update YOLO target class based on voice command."""
        if target_name in YOLO_CLASS_IDS:
            self.grasp_target_name = target_name
            new_id = int(YOLO_CLASS_IDS[target_name])
            self.target_class_id = new_id
            self.get_logger().info(
                f'Voice command: target set to "{target_name}" '
                f'(class_id={new_id})')
        else:
            self.get_logger().warn(
                f'Unknown object "{target_name}" — keeping default: '
                f'{self.grasp_target_name}')

    # ------------------------------------------------------------------
    # State machine override: intercept target-reached -> grasp -> return
    # ------------------------------------------------------------------

    def handle_seek_target(self, current_time):
        # Phase 0: voice command (blocking — runs once before navigation)
        if not self.voice_command_received:
            self.publish_hold_position()
            self._run_voice_command()
            return

        # Phase 3: returning home after grasp
        if self.return_home_active:
            self._handle_return_home()
            return

        # Phase 2: grasping (blocking — runs full pipeline then resumes)
        if self.grasp_active:
            # Should not be reached since _run_grasp_pipeline is blocking,
            # but guard just in case.
            self.publish_hold_position()
            return

        # Phase 1: normal seek behavior
        super().handle_seek_target(current_time)

        # Check if target was just reached
        if (
            self.red_target_reached_logged
            and not self.grasp_active
            and not self.grasp_complete
        ):
            self.get_logger().info('Target reached — starting grasp sequence.')
            self.grasp_active = True
            self.publish_hold_position()
            self._run_grasp_pipeline()

    # ------------------------------------------------------------------
    # Grasp pipeline (blocking — called from timer callback)
    # ------------------------------------------------------------------

    def _run_grasp_pipeline(self):
        """Execute the full top-down grasp. Blocking."""
        self.get_logger().info('=' * 50)
        self.get_logger().info('GRASP PIPELINE START')
        self.get_logger().info('=' * 50)

        success = False
        try:
            success = self._execute_grasp()
        except Exception as e:
            self.get_logger().error(f'Grasp pipeline exception: {e}')

        self.grasp_active = False
        self.grasp_complete = True
        self.grasp_success = success

        if success:
            self.get_logger().info('Grasp succeeded — initiating return home.')
        else:
            self.get_logger().warn('Grasp failed — returning home empty-handed.')

        # Reset camera to level before driving home
        self._set_camera_tilt(0.0)

        if self.return_enabled and self.home_pose_set:
            self.return_home_active = True
            self.get_logger().info(
                f'Returning to home pose ({self.home_x:.2f}, {self.home_y:.2f}).')

    GRASP_CAMERA_TILT_RAD = 0.65  # ~37 deg down — needs to see banana at close range

    def _set_camera_tilt(self, tilt_rad):
        """Set camera tilt to a specific angle and wait for it to settle."""
        self.camera_tilt_cmd = tilt_rad
        msg = Float64MultiArray()
        msg.data = [self.camera_pan_cmd, self.camera_tilt_cmd]
        self.pan_tilt_pub.publish(msg)
        self.get_logger().info(
            f'  Camera tilt set to {np.rad2deg(tilt_rad):.0f} deg for grasping')
        self._spin_for(1.5)  # wait for camera to settle + new frames

    def _execute_grasp(self):
        """Run detection -> depth -> plan -> pick -> lift. Returns True on success."""

        # Wait for planner service
        self.get_logger().info('Waiting for /plan_to_target service...')
        if not self.plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/plan_to_target service not available!')
            return False

        # Reset camera to a known good tilt angle for grasping.
        # During approach, auto-tilt may have tilted too far down,
        # causing the YOLO bbox to cover too much table/floor.
        self._set_camera_tilt(self.GRASP_CAMERA_TILT_RAD)

        # Spin briefly to get fresh sensor data
        self._spin_for(1.0)

        # --- detect ---
        det = self._detect_target()
        if det is None:
            self.get_logger().error(
                f'Cannot find "{self.grasp_target_name}" in camera view.')
            return False

        u = int(det.bbox.center.position.x)
        v = int(det.bbox.center.position.y)
        bbox_w = float(det.bbox.size_x)
        bbox_h = float(det.bbox.size_y)
        self.get_logger().info(
            f'  YOLO detection at pixel ({u}, {v}), bbox {bbox_w:.0f}x{bbox_h:.0f}')

        # --- depth point cloud ---
        if not self.is_sim and hasattr(self, 'depth_camera_info') and self.depth_camera_info is not None:
            u, v, bbox_w, bbox_h = self._transform_bbox_to_depth(u, v, bbox_w, bbox_h)

        pts_base = self._get_object_points(u, v, bbox_w, bbox_h)
        if pts_base is None:
            self.get_logger().error('Insufficient depth points for grasp.')
            return False
        self.get_logger().info(f'  Point cloud: {len(pts_base)} points')

        # --- analyze ---
        analysis = self._analyze_object(pts_base)
        centroid = analysis['centroid']
        self.get_logger().info(
            f'  Centroid: ({centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f})')

        # --- compute poses ---
        grasp_pose, pre_grasp_pose, info = self._compute_grasp_pose(analysis)
        self.get_logger().info(f'  Grasp params: {info}')

        # --- IK check ---
        self.get_logger().info('Checking IK feasibility...')
        use_orientation = True
        ok, msg = self._try_ik(grasp_pose, use_orientation=True)
        if not ok:
            self.get_logger().warn(f'  IK with orientation failed: {msg}')
            ok, msg = self._try_ik(grasp_pose, use_orientation=False)
            if ok:
                use_orientation = False
            else:
                self.get_logger().error(f'  IK also failed: {msg}')
                return False

        # --- execute pick ---
        self.get_logger().info('Opening gripper...')
        self._set_gripper(0.0)

        self.get_logger().info('Moving to pre-grasp...')
        if not self._plan_and_execute(pre_grasp_pose, 'pre-grasp', use_orientation):
            self.get_logger().error('Pre-grasp failed.')
            self._go_home_arm()
            return False

        time.sleep(SETTLE_TIME)

        self.get_logger().info('Descending to grasp...')
        if not self._plan_and_execute(grasp_pose, 'grasp', use_orientation):
            self.get_logger().error('Grasp descent failed.')
            self._go_home_arm()
            return False

        time.sleep(SETTLE_TIME)

        self.get_logger().info('Closing gripper...')
        self._close_gripper_firm()

        # --- lift ---
        self.get_logger().info('Lifting object...')
        lift_pose = Pose()
        lift_pose.position.x = grasp_pose.position.x
        lift_pose.position.y = grasp_pose.position.y
        lift_pose.position.z = grasp_pose.position.z + self.lift_height
        lift_pose.orientation = grasp_pose.orientation
        if not self._plan_and_execute(lift_pose, 'lift', use_orientation):
            self.get_logger().warn('Lift failed, but continuing...')

        self.get_logger().info('GRASP PIPELINE COMPLETE')
        return True

    # ------------------------------------------------------------------
    # Grasp sub-routines
    # ------------------------------------------------------------------

    def _spin_for(self, seconds):
        """Wait for a duration. With MultiThreadedExecutor, callbacks are
        processed by the executor threads so we just sleep."""
        time.sleep(seconds)

    def _detect_target(self):
        """Find best YOLO detection matching our target class."""
        target_id = YOLO_CLASS_IDS.get(self.grasp_target_name)
        if target_id is None:
            return None
        for _ in range(20):
            self._spin_for(0.2)
            dets = self._raw_yolo_detections
            if dets is None:
                continue
            best, best_conf = None, 0.0
            for det in dets.detections:
                for hyp in det.results:
                    if (hyp.hypothesis.class_id == target_id
                            and hyp.hypothesis.score > best_conf):
                        best = det
                        best_conf = hyp.hypothesis.score
            if best is not None:
                return best
        return None

    def _get_depth_image(self):
        """Get current depth image as float32 array in mm (from parent's depth_callback)."""
        if self.latest_depth_mm is not None:
            return self.latest_depth_mm
        return None

    def _transform_bbox_to_depth(self, u, v, bbox_w, bbox_h):
        """Transform YOLO bbox from RGB pixel space to depth pixel space (real hw)."""
        if self.camera_info_data is None or self.depth_camera_info is None:
            return u, v, bbox_w, bbox_h
        K_rgb = np.array(self.camera_info_data.k).reshape(3, 3)
        K_depth = np.array(self.depth_camera_info.k).reshape(3, 3)
        pt = np.array([u, v, 1.0])
        pt_norm = np.linalg.inv(K_rgb) @ pt
        pt_depth = K_depth @ pt_norm
        u_d, v_d = int(round(pt_depth[0])), int(round(pt_depth[1]))
        scale_x = K_depth[0, 0] / K_rgb[0, 0]
        scale_y = K_depth[1, 1] / K_rgb[1, 1]
        return u_d, v_d, bbox_w * scale_x, bbox_h * scale_y

    def _get_object_points(self, cx, cy, bbox_w, bbox_h):
        """Crop depth to bbox, back-project to base_link point cloud."""
        depth_img = self._get_depth_image()
        if depth_img is None:
            return None

        # Select intrinsics
        if not self.is_sim and hasattr(self, 'depth_camera_info') and self.depth_camera_info is not None:
            K = self.depth_camera_info.k
            tf_frame = 'camera_depth_optical_frame'
        else:
            if self.camera_info_data is None:
                return None
            K = self.camera_info_data.k
            tf_frame = 'camera_color_optical_frame'

        fx, fy, cx_k, cy_k = K[0], K[4], K[2], K[5]

        # Crop to padded bbox
        h, w = depth_img.shape[:2]
        half_w = int(bbox_w * self.bbox_pad / 2)
        half_h = int(bbox_h * self.bbox_pad / 2)
        u0 = max(0, cx - half_w)
        v0 = max(0, cy - half_h)
        u1 = min(w, cx + half_w)
        v1 = min(h, cy + half_h)
        roi = depth_img[v0:v1, u0:u1]

        # latest_depth_mm is float32 in millimeters; convert to meters
        depth_m = roi.astype(np.float32) / 1000.0 if roi.dtype != np.float64 else roi / 1000.0
        roi_h, roi_w = depth_m.shape[:2]

        u_grid, v_grid = np.meshgrid(
            np.arange(roi_w) + u0,
            np.arange(roi_h) + v0)
        mask = (depth_m > MIN_DEPTH_M) & (depth_m < MAX_DEPTH_M)
        z = depth_m[mask]
        u_pts = u_grid[mask].astype(np.float32)
        v_pts = v_grid[mask].astype(np.float32)

        if len(z) < MIN_POINTS:
            return None

        x_cam = (u_pts - cx_k) * z / fx
        y_cam = (v_pts - cy_k) * z / fy
        points_cam = np.stack([x_cam, y_cam, z], axis=1)

        tf_mat = self._get_tf_matrix('base_link', tf_frame)
        if tf_mat is None:
            return None

        ones = np.ones((len(points_cam), 1), dtype=np.float32)
        pts_h = np.hstack([points_cam, ones])
        pts_base = (tf_mat @ pts_h.T).T[:, :3]

        # Filter above table
        pts_base = pts_base[pts_base[:, 2] > TABLE_Z_MIN]

        # Remove Z outliers
        if len(pts_base) > MIN_POINTS:
            median_z = np.median(pts_base[:, 2])
            pts_base = pts_base[np.abs(pts_base[:, 2] - median_z) < OUTLIER_Z_THRESH]

        if len(pts_base) < MIN_POINTS:
            return None
        return pts_base

    def _get_tf_matrix(self, target_frame, source_frame):
        try:
            import rclpy.time
            import rclpy.duration
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
            t = transform.transform.translation
            q = transform.transform.rotation
            rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
            mat = np.eye(4, dtype=np.float32)
            mat[:3, :3] = rot.as_matrix().astype(np.float32)
            mat[:3, 3] = [t.x, t.y, t.z]
            return mat
        except Exception as e:
            self.get_logger().error(f'TF lookup failed: {e}')
            return None

    def _analyze_object(self, points_base):
        """Find centroid, z_top, and optimal grasp yaw via min-width sweep."""
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
            width = float(np.ptp(centered @ direction))
            if width < min_width:
                min_width = width
                best_yaw = angle
            if width > max_width:
                max_width = width

        self.get_logger().info(
            f'  Min-width sweep: yaw={np.degrees(best_yaw):.0f}deg, '
            f'grip_width={min_width*1000:.0f}mm')

        return {
            'centroid': centroid,
            'grasp_yaw': best_yaw,
            'grip_width': min_width,
            'max_width': max_width,
            'z_top': z_top,
            'num_points': len(points_base),
        }

    def _compute_grasp_pose(self, analysis):
        """Compute grasp and pre-grasp poses."""
        centroid = analysis['centroid']
        grasp_yaw = analysis['grasp_yaw']
        grip_width = analysis['grip_width']

        if grip_width > GRIPPER_MAX_OPENING_M:
            self.get_logger().warn(
                f'  Width {grip_width*1000:.0f}mm > gripper max '
                f'{GRIPPER_MAX_OPENING_M*1000:.0f}mm (proceeding anyway)')

        qw, qx, qy, qz = yaw_to_grasp_quaternion(grasp_yaw)
        grasp_z = analysis['z_top'] + self.z_offset + self.z_bias

        grasp_pose = Pose()
        grasp_pose.position.x = float(centroid[0])
        grasp_pose.position.y = float(centroid[1])
        grasp_pose.position.z = float(grasp_z)
        grasp_pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

        pre_grasp_pose = Pose()
        pre_grasp_pose.position.x = float(centroid[0])
        pre_grasp_pose.position.y = float(centroid[1])
        pre_grasp_pose.position.z = float(grasp_z + self.pre_grasp_height)
        pre_grasp_pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

        info = (f'yaw={np.degrees(grasp_yaw):.0f}deg, '
                f'z={grasp_z:.3f}m, width={grip_width*1000:.0f}mm')
        return grasp_pose, pre_grasp_pose, info

    def _wait_for_future(self, future, timeout_sec=10.0):
        """Wait for service future to complete. With MultiThreadedExecutor,
        the service response is processed by the executor on another thread,
        so we just poll the future status."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if future.done():
                return True
            time.sleep(0.05)
        return False

    def _try_ik(self, pose, use_orientation=True):
        """Dry-run IK check. Returns (success, message)."""
        req = PlanToTarget.Request()
        req.arm_name = self.grasp_arm
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = False
        req.duration = self.grasp_move_duration
        req.max_condition_number = 200.0

        future = self.plan_client.call_async(req)
        if not self._wait_for_future(future, timeout_sec=10.0):
            return False, 'service call timed out'
        if future.exception():
            return False, f'service exception: {future.exception()}'
        result = future.result()
        return result.success, result.message

    def _plan_and_execute(self, pose, label, use_orientation=True):
        """Plan and execute arm motion. Returns True on success."""
        req = PlanToTarget.Request()
        req.arm_name = self.grasp_arm
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = True
        req.duration = self.grasp_move_duration
        req.max_condition_number = 100.0

        self.get_logger().info(f'  [{label}] Planning + executing...')
        future = self.plan_client.call_async(req)
        if not self._wait_for_future(future, timeout_sec=15.0):
            self.get_logger().error(f'  [{label}] service timed out')
            return False
        if future.exception():
            self.get_logger().error(f'  [{label}] exception: {future.exception()}')
            return False

        result = future.result()
        if result.success:
            if result.executed:
                time.sleep(self.grasp_move_duration + 0.5)
            return True
        self.get_logger().error(f'  [{label}] failed: {result.message}')
        return False

    def _set_gripper(self, position):
        msg = Float64MultiArray()
        msg.data = [position]
        for _ in range(10):
            self.gripper_pub.publish(msg)
            time.sleep(0.05)
        time.sleep(0.5)

    def _close_gripper_firm(self):
        msg = Float64MultiArray()
        msg.data = [1.0]
        for _ in range(GRIPPER_CLOSE_REPEATS):
            self.gripper_pub.publish(msg)
            time.sleep(0.05)
        time.sleep(GRIPPER_CLOSE_WAIT)
        for _ in range(10):
            self.gripper_pub.publish(msg)
            time.sleep(0.05)

    def _go_home_arm(self, duration=3.0):
        """Return arm to safe position."""
        if self.is_sim:
            self.get_logger().info(f'Returning arm to home over {duration}s...')
            msg = ArmCommand()
            msg.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            msg.duration = duration
            self.arm_pub.publish(msg)
            time.sleep(duration + 0.5)
        else:
            self.get_logger().info('Returning arm to sleep pose...')
            self._go_to_sleep_pose()

    def _go_to_sleep_pose(self, max_joint_speed=0.5):
        """Smooth trajectory to sleep pose (real hardware)."""
        if not hasattr(self, 'current_joint_positions'):
            return
        joint_names = [
            f'{self.grasp_arm}_{j}' for j in
            ['waist', 'shoulder', 'elbow', 'forearm_roll',
             'wrist_angle', 'wrist_rotate']
        ]
        current = np.array([
            self.current_joint_positions.get(n, 0.0) for n in joint_names])
        target = np.array(SLEEP_POSE)
        max_diff = np.max(np.abs(target - current))
        duration = max(max_diff / max_joint_speed, 1.0)

        rate_hz = 50.0
        dt = 1.0 / rate_hz
        num_steps = max(int(duration * rate_hz), 1)

        try:
            from interbotix_xs_msgs.msg import JointGroupCommand
        except ImportError:
            self.get_logger().warn('interbotix_xs_msgs not available for sleep pose')
            return

        pub = self.create_publisher(
            JointGroupCommand,
            f'/{self.grasp_arm}_arm/commands/joint_group', 10)

        for i in range(num_steps + 1):
            t = i / num_steps
            alpha = 0.5 * (1 - np.cos(np.pi * t))
            q = current + alpha * (target - current)
            cmd = JointGroupCommand()
            cmd.name = f'{self.grasp_arm}_arm'
            cmd.cmd = q.tolist()
            pub.publish(cmd)
            if i < num_steps:
                time.sleep(dt)

    # ------------------------------------------------------------------
    # Return-home logic (from NavigateAndReturnNode)
    # ------------------------------------------------------------------

    def _handle_return_home(self):
        if not self.home_pose_set:
            self.publish_hold_position()
            return

        dx = self.home_x - self.current_x
        dy = self.home_y - self.current_y
        distance = float(np.hypot(dx, dy))
        angle_error = self._wrapped_angle_diff(self.home_theta, self.current_theta)

        # Periodic progress log
        now = self.get_clock().now()
        if not hasattr(self, '_last_return_log') or (now - self._last_return_log).nanoseconds / 1e9 > 3.0:
            self._last_return_log = now
            blocked = self.is_obstacle_blocking()
            self.get_logger().info(
                f'  Return home: dist={distance:.2f}m, '
                f'angle_err={np.rad2deg(angle_error):.0f}deg, '
                f'blocked={blocked}')

        if (distance <= self.return_position_tolerance
                and abs(angle_error) <= self.return_angle_tolerance):
            if not self.return_home_complete:
                self.get_logger().info('Returned to home pose.')
                if self.grasp_success:
                    self.get_logger().info('Releasing object and stowing arm...')
                    self._set_gripper(0.0)
                    time.sleep(0.5)
                    self._go_home_arm()
                self.get_logger().info('Task complete. Holding position.')
            self.return_home_complete = True
            self.return_home_active = False
            self.publish_hold_position()
            return

        # Skip obstacle check during return — the arm holding the object
        # appears as an obstacle in the depth image and would block movement.
        self.publish_pose_target(self.home_x, self.home_y, self.home_theta)


def main(args=None):
    rclpy.init(args=args)
    node = NavigateGraspReturnNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
