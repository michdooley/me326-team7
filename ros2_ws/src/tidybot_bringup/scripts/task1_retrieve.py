#!/usr/bin/env python3
"""
Task 1: Object Retrieval State Machine

Orchestrates the full object retrieval task:
    LISTEN → SEARCH → APPROACH → PLAN_GRASP → PRE_GRASP → GRASP →
    VERIFY_GRASP → RETURN → DONE

Uses NavigateToObject for search/approach, YOLOObjectDetector for
class-based object detection, and /plan_to_target for arm motion planning.

VERIFY_GRASP sub-steps:
    1. PRESENT_ARM  — Move grasping arm to a pose visible to the camera
    2. AIM_CAMERA   — Tilt camera to look at the gripper region
    3. DETECT       — YOLO detection for target object class
    4. DECIDE       — If target detected → success → RETURN
                      If not → increment retry counter → back to APPROACH
                      If retries exhausted → ERROR

Usage:
    # Terminal 1: Launch sim
    ros2 launch tidybot_bringup sim.launch.py

    # Terminal 2: Run task
    ros2 run tidybot_bringup task1_retrieve.py
"""

import sys
from pathlib import Path

# Add scripts directory to path (for navigate_to_object import)
_SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SCRIPT_DIR))

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration

import numpy as np
import time
from enum import Enum, auto

import tf2_ros

from geometry_msgs.msg import Pose, Twist, Quaternion, Point
from std_msgs.msg import Float64MultiArray
from tidybot_msgs.msg import ArmCommand
from tidybot_msgs.srv import PlanToTarget

from tidybot_perception.yolo_object_detector import YOLOObjectDetector
from navigate_to_object import NavigateToObject, _normalize_angle


# ═══════════════════════════════════════════════════════════════════════
# Inline helpers (from robot_helpers.py — avoids install-path issues)
# ═══════════════════════════════════════════════════════════════════════

ORIENT_FINGERS_DOWN = (0.5, 0.5, 0.5, -0.5)
ARM_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def create_pose(x, y, z, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
    pose = Pose()
    pose.position = Point(x=float(x), y=float(y), z=float(z))
    pose.orientation = Quaternion(w=float(qw), x=float(qx), y=float(qy), z=float(qz))
    return pose


def open_gripper(publisher):
    msg = Float64MultiArray()
    msg.data = [0.0]
    publisher.publish(msg)


def close_gripper(publisher):
    msg = Float64MultiArray()
    msg.data = [1.0]
    publisher.publish(msg)


# ═══════════════════════════════════════════════════════════════════════
# States
# ═══════════════════════════════════════════════════════════════════════

class Task1State(Enum):
    """States for the object retrieval task."""
    LISTEN       = auto()  # Wait for voice command → get target object
    SEARCH       = auto()  # NavigateToObject: scan + explore + approach
    APPROACH     = auto()  # Retry entry: reset nav_node and re-search
    PLAN_GRASP   = auto()  # Detect object, compute grasp poses
    PRE_GRASP    = auto()  # Move arm to approach pose via /plan_to_target
    GRASP        = auto()  # Execute grasp + close gripper
    VERIFY_GRASP = auto()  # Visual verification of grasp success
    RETURN       = auto()  # Navigate back to start position
    DONE         = auto()
    ERROR        = auto()


class VerifySubState(Enum):
    """Sub-states within VERIFY_GRASP."""
    PRESENT_ARM = auto()   # Move arm to camera-visible pose
    AIM_CAMERA  = auto()   # Point camera at gripper area
    DETECT      = auto()   # YOLO detection for target object
    DECIDE      = auto()   # Evaluate detection results


# ═══════════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════════

# ── Grasp geometry ────────────────────────────────────────────────────
PRE_GRASP_Z_OFFSET = 0.10     # 10 cm above object for approach
GRASP_Z_OFFSET = 0.00         # offset from detected surface top
GRASP_MOVE_DURATION = 2.0     # seconds for arm motions
GRIPPER_SETTLE_TIME = 1.0     # seconds to let gripper close/open
CAMERA_TILT_GRASP = 0.5       # camera tilt to look at table for detection

# ── Verification ──────────────────────────────────────────────────────
MAX_GRASP_RETRIES = 3

VERIFY_ARM_POSE = [0.0, 0.4, -0.3, 0.0, 0.5, 0.0]
VERIFY_ARM_DURATION = 2.5

VERIFY_CAMERA_PAN = 0.0
VERIFY_CAMERA_TILT = 0.3
VERIFY_CAMERA_SETTLE_TIME = 1.5

VERIFY_MIN_BLOB_AREA = 100    # minimum detection area to confirm grasp (unused with YOLO)

# ── Return navigation ────────────────────────────────────────────────
RETURN_SPEED = 0.3             # m/s driving back
RETURN_GOAL_TOLERANCE = 0.5    # meters — close enough to start pose
RETURN_HEADING_THRESHOLD = 0.4 # radians — rotate-in-place if error larger
OBSTACLE_THRESHOLD = 0.5       # meters — trigger avoidance
EMERGENCY_STOP_DISTANCE = 0.25 # meters — hard stop


# ═══════════════════════════════════════════════════════════════════════
# Task1Retrieve Node
# ═══════════════════════════════════════════════════════════════════════

class Task1Retrieve(Node):
    """Object retrieval state machine."""

    def __init__(self, nav_node: NavigateToObject):
        super().__init__('task1_retrieve')

        self.nav_node = nav_node

        # ── Perception (YOLO detection + depth localization) ──
        self.detector = YOLOObjectDetector(self)

        # ── TF2 (for odom→base_link lookups in RETURN) ──
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Publishers ──
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.right_arm_pub = self.create_publisher(ArmCommand, '/right_arm/cmd', 10)
        self.left_arm_pub = self.create_publisher(ArmCommand, '/left_arm/cmd', 10)
        self.right_gripper_pub = self.create_publisher(
            Float64MultiArray, '/right_gripper/cmd', 10)
        self.left_gripper_pub = self.create_publisher(
            Float64MultiArray, '/left_gripper/cmd', 10)
        self.pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10)

        # ── Service client ──
        self.plan_to_target_client = self.create_client(
            PlanToTarget, '/plan_to_target')

        # ── State machine ──
        self.state = Task1State.LISTEN
        self.state_start_time = None

        # ── Task data ──
        self.target_object = 'banana'   # hardcoded for now (skip audio)
        self.active_arm = 'right'
        self.start_pose = None          # (x, y, theta) in odom to return to

        # ── Grasp data ──
        self.pre_grasp_pose = None      # geometry_msgs/Pose
        self.grasp_pose = None          # geometry_msgs/Pose
        self.grasp_object_pos = None    # (ox, oy, oz) in base_link

        # ── Async service call tracking ──
        self.plan_future = None
        self.grasp_phase = 0            # sub-phase within PRE_GRASP/GRASP
        self.phase_start_time = 0.0

        # ── Verify-specific data ──
        self.verify_sub_state = VerifySubState.PRESENT_ARM
        self.grasp_attempt_count = 0
        self.verify_arm_cmd_sent = False
        self.verify_camera_cmd_sent = False
        self.verify_detect_result = None

        # ── Camera tilt sent flag for PLAN_GRASP ──
        self.plan_grasp_camera_sent = False

        # ── Control loop at 10Hz ──
        self.cb_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(
            0.1, self.control_loop, callback_group=self.cb_group)

        self.get_logger().info('=' * 50)
        self.get_logger().info('Task 1: Object Retrieval')
        self.get_logger().info('=' * 50)
        self.get_logger().info(
            'States: LISTEN → SEARCH → PLAN_GRASP → PRE_GRASP → '
            'GRASP → VERIFY → RETURN → DONE'
        )
        self.get_logger().info(f'Target object: {self.target_object}')
        self.get_logger().info(f'Max grasp retries: {MAX_GRASP_RETRIES}')

    # ═══════════════════════════════════════════════════════════════════
    # Helper methods
    # ═══════════════════════════════════════════════════════════════════

    def _get_base_pose_in_odom(self):
        """Get (x, y, theta) from odom→base_link TF.

        theta is MuJoCo convention; actual heading = theta - pi/2.
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

    def _stop_base(self):
        """Publish zero velocity."""
        self.cmd_vel_pub.publish(Twist())

    def _send_arm_cmd(self, joint_positions, duration):
        """Publish an ArmCommand to the active arm."""
        cmd = ArmCommand()
        cmd.joint_positions = list(joint_positions)
        cmd.duration = float(duration)
        if self.active_arm == 'right':
            self.right_arm_pub.publish(cmd)
        else:
            self.left_arm_pub.publish(cmd)

    def _send_pan_tilt(self, pan, tilt):
        """Publish a camera pan/tilt command."""
        msg = Float64MultiArray()
        msg.data = [float(pan), float(tilt)]
        self.pan_tilt_pub.publish(msg)

    def _get_gripper_pub(self):
        """Get the gripper publisher for the active arm."""
        if self.active_arm == 'right':
            return self.right_gripper_pub
        return self.left_gripper_pub

    def _send_plan_to_target(self, target_pose, label):
        """Send async /plan_to_target request. Sets self.plan_future."""
        req = PlanToTarget.Request()
        req.arm_name = self.active_arm
        req.target_pose = target_pose
        req.use_orientation = True
        req.execute = True
        req.duration = GRASP_MOVE_DURATION
        req.max_condition_number = 100.0

        self.get_logger().info(
            f'  [{label}] Calling /plan_to_target '
            f'pos=({target_pose.position.x:.3f}, '
            f'{target_pose.position.y:.3f}, '
            f'{target_pose.position.z:.3f})')
        self.plan_future = self.plan_to_target_client.call_async(req)

    def _analyze_depth_obstacles(self):
        """Analyze depth image for obstacles in left/center/right sectors.

        Returns:
            (clearance, distances) where each is (left, center, right).
            clearance: tuple of bools (True = clear).
            distances: tuple of floats (meters to nearest obstacle).
        """
        depth = self.detector.latest_depth
        if depth is None:
            return (True, True, True), (999.0, 999.0, 999.0)

        h, w = depth.shape
        v_start = int(h * 0.3)
        v_end = int(h * 0.8)

        left_region = depth[v_start:v_end, 0:w // 3]
        center_region = depth[v_start:v_end, w // 3:2 * w // 3]
        right_region = depth[v_start:v_end, 2 * w // 3:w]

        def sector_min_dist(region):
            valid = region[region > 0].astype(np.float64)
            if len(valid) == 0:
                return 999.0
            return np.percentile(valid, 10) / 1000.0  # mm → meters

        left_dist = sector_min_dist(left_region)
        center_dist = sector_min_dist(center_region)
        right_dist = sector_min_dist(right_region)

        clearance = (
            left_dist > OBSTACLE_THRESHOLD,
            center_dist > OBSTACLE_THRESHOLD,
            right_dist > OBSTACLE_THRESHOLD,
        )
        return clearance, (left_dist, center_dist, right_dist)

    def _drive_toward_goal(self, goal_x, goal_y, speed):
        """Drive toward an odom-frame goal with depth-based obstacle avoidance."""
        pose = self._get_base_pose_in_odom()
        if pose is None:
            return

        cx, cy, ctheta = pose
        actual_heading = ctheta - np.pi / 2  # MuJoCo heading convention

        dx = goal_x - cx
        dy = goal_y - cy
        distance = np.sqrt(dx**2 + dy**2)
        desired_heading = np.arctan2(dy, dx)
        heading_error = _normalize_angle(desired_heading - actual_heading)

        clearance, distances = self._analyze_depth_obstacles()
        left_clear, center_clear, right_clear = clearance
        left_dist, center_dist, right_dist = distances

        twist = Twist()

        if center_clear:
            if abs(heading_error) > RETURN_HEADING_THRESHOLD:
                # Large heading error: rotate first
                twist.angular.z = np.clip(2.0 * heading_error, -1.0, 1.0)
            else:
                # Drive forward with heading correction
                twist.linear.x = min(speed, distance * 0.5)
                twist.angular.z = np.clip(1.5 * heading_error, -0.8, 0.8)
        else:
            # Center blocked — steer toward clearer side
            if left_dist > right_dist:
                twist.angular.z = 0.5
                twist.linear.x = 0.05
            elif right_dist > left_dist:
                twist.angular.z = -0.5
                twist.linear.x = 0.05
            else:
                twist.angular.z = 0.6

        # Emergency stop if very close
        if min(left_dist, center_dist, right_dist) < EMERGENCY_STOP_DISTANCE:
            twist.linear.x = 0.0

        self.cmd_vel_pub.publish(twist)

    # ═══════════════════════════════════════════════════════════════════
    # State machine
    # ═══════════════════════════════════════════════════════════════════

    def transition_to(self, new_state: Task1State):
        """Transition to a new state."""
        self.get_logger().info(f'State: {self.state.name} → {new_state.name}')
        self.state = new_state
        self.state_start_time = time.time()

    # ── Verify sub-state machine ──────────────────────────────────────

    def _begin_verify(self):
        """Reset verify sub-state for a fresh verification pass."""
        self.verify_sub_state = VerifySubState.PRESENT_ARM
        self.verify_arm_cmd_sent = False
        self.verify_camera_cmd_sent = False
        self.verify_detect_result = None
        self.grasp_attempt_count += 1
        self.get_logger().info(
            f'Starting grasp verification '
            f'(attempt {self.grasp_attempt_count}/{MAX_GRASP_RETRIES})')

    def _run_verify_grasp(self, elapsed):
        """Execute the VERIFY_GRASP sub-state machine."""

        # ── PRESENT_ARM: move the grasping arm to a camera-visible pose ──
        if self.verify_sub_state == VerifySubState.PRESENT_ARM:
            if not self.verify_arm_cmd_sent:
                self.get_logger().info('  [verify] Moving arm to present pose...')
                self._send_arm_cmd(VERIFY_ARM_POSE, VERIFY_ARM_DURATION)
                self.verify_arm_cmd_sent = True
                self.state_start_time = time.time()
                return

            if elapsed >= VERIFY_ARM_DURATION + 0.5:
                self.verify_sub_state = VerifySubState.AIM_CAMERA
                self.state_start_time = time.time()

        # ── AIM_CAMERA: tilt camera to look at the gripper region ────────
        elif self.verify_sub_state == VerifySubState.AIM_CAMERA:
            if not self.verify_camera_cmd_sent:
                self.get_logger().info('  [verify] Aiming camera at gripper...')
                self._send_pan_tilt(VERIFY_CAMERA_PAN, VERIFY_CAMERA_TILT)
                self.verify_camera_cmd_sent = True
                self.state_start_time = time.time()
                return

            if elapsed >= VERIFY_CAMERA_SETTLE_TIME:
                self.verify_sub_state = VerifySubState.DETECT
                self.state_start_time = time.time()

        # ── DETECT: YOLO detection for target ─────────────────────────────
        elif self.verify_sub_state == VerifySubState.DETECT:
            if elapsed < 0.5:
                return  # brief settle after camera aim

            self.get_logger().info(
                f'  [verify] Checking for "{self.target_object}" via YOLO...')
            centroid = self.detector.detect(
                self.target_object, min_area=VERIFY_MIN_BLOB_AREA)

            self.verify_detect_result = centroid
            self.verify_sub_state = VerifySubState.DECIDE
            self.state_start_time = time.time()

        # ── DECIDE: evaluate detection results ───────────────────────────
        elif self.verify_sub_state == VerifySubState.DECIDE:
            grasp_success = self.verify_detect_result is not None

            if grasp_success:
                u, v = self.verify_detect_result
                self.get_logger().info(
                    f'  [verify] Object detected at pixel ({u}, {v}) '
                    f'— grasp confirmed!')
                # Send arm to home before transitioning
                self._send_arm_cmd(ARM_HOME, 2.0)
                self.transition_to(Task1State.RETURN)
            elif self.grasp_attempt_count >= MAX_GRASP_RETRIES:
                self.get_logger().error(
                    f'  [verify] Grasp failed after {MAX_GRASP_RETRIES} '
                    f'attempts — giving up')
                open_gripper(self._get_gripper_pub())
                self._send_arm_cmd(ARM_HOME, 2.0)
                self.transition_to(Task1State.ERROR)
            else:
                self.get_logger().warn(
                    f'  [verify] Grasp not confirmed — retrying '
                    f'(attempt {self.grasp_attempt_count}/{MAX_GRASP_RETRIES})')
                # Release object, return arm to home, retry
                open_gripper(self._get_gripper_pub())
                self._send_arm_cmd(ARM_HOME, 2.0)
                self.transition_to(Task1State.APPROACH)

    # ═══════════════════════════════════════════════════════════════════
    # Main control loop
    # ═══════════════════════════════════════════════════════════════════

    def control_loop(self):
        """Main control loop — runs the state machine at 10Hz."""
        now = time.time()

        if self.state_start_time is None:
            self.state_start_time = now
            self.get_logger().info(f'Starting state: {self.state.name}')

        elapsed = now - self.state_start_time

        # ==================== LISTEN ====================
        if self.state == Task1State.LISTEN:
            # Hardcode target object (skip audio for now)
            # self.target_object is set in __init__

            # Record start pose from TF
            self.start_pose = self._get_base_pose_in_odom()
            if self.start_pose is None:
                return  # TF not ready, retry next tick

            self.get_logger().info(f'Target: "{self.target_object}"')
            self.get_logger().info(
                f'Start pose: ({self.start_pose[0]:.2f}, '
                f'{self.start_pose[1]:.2f})')

            # Tell NavigateToObject to start searching
            self.nav_node.set_target(self.target_object)
            self.transition_to(Task1State.SEARCH)

        # ==================== SEARCH ====================
        elif self.state == Task1State.SEARCH:
            # NavigateToObject drives autonomously via its own timer.
            # We just poll is_positioned() each tick.
            if self.nav_node.is_positioned():
                self.get_logger().info(
                    'NavigateToObject reports: POSITIONED')
                self._stop_base()
                # Disable nav_node driving so it doesn't conflict
                self.nav_node.nav_state = self.nav_node.nav_state  # stay POSITIONED
                self.transition_to(Task1State.PLAN_GRASP)

        # ==================== APPROACH (retry entry) ====================
        elif self.state == Task1State.APPROACH:
            # Entered on grasp verify failure — reset nav and re-search
            if elapsed < 0.1:
                self.get_logger().info('Grasp retry: resetting navigation...')
                self.nav_node.set_target(self.target_object)
            # Wait for arm to return home (2s from verify DECIDE)
            if elapsed > 2.5:
                self.transition_to(Task1State.SEARCH)

        # ==================== PLAN_GRASP ====================
        elif self.state == Task1State.PLAN_GRASP:
            if not self.plan_grasp_camera_sent:
                # Point camera down at the object for close-range detection
                self._send_pan_tilt(0.0, CAMERA_TILT_GRASP)
                self.plan_grasp_camera_sent = True
                return

            if elapsed < 2.0:
                return  # camera settling + let detector get fresh frames

            # Use YOLOObjectDetector to re-detect object for grasp planning
            result = self.detector.detect_and_localize(self.target_object)
            if result is None:
                if elapsed > 8.0:
                    self.get_logger().error(
                        'Could not re-detect object for grasp planning')
                    self.plan_grasp_camera_sent = False
                    self.transition_to(Task1State.APPROACH)
                return

            (u, v), base_pt = result
            ox = base_pt.point.x
            oy = base_pt.point.y
            oz = base_pt.point.z

            self.grasp_object_pos = (ox, oy, oz)
            self.get_logger().info(
                f'Object at base_link: ({ox:.3f}, {oy:.3f}, {oz:.3f})')

            # Compute grasp and pre-grasp poses (top-down)
            grasp_z = oz + GRASP_Z_OFFSET
            pre_grasp_z = grasp_z + PRE_GRASP_Z_OFFSET
            qw, qx, qy, qz = ORIENT_FINGERS_DOWN

            self.pre_grasp_pose = create_pose(
                ox, oy, pre_grasp_z, qw, qx, qy, qz)
            self.grasp_pose = create_pose(
                ox, oy, grasp_z, qw, qx, qy, qz)

            self.get_logger().info(
                f'Pre-grasp z={pre_grasp_z:.3f}, Grasp z={grasp_z:.3f}')
            self.plan_grasp_camera_sent = False
            self.transition_to(Task1State.PRE_GRASP)

        # ==================== PRE_GRASP ====================
        elif self.state == Task1State.PRE_GRASP:
            if elapsed < 0.1:
                self.grasp_phase = 0
                self.plan_future = None
                open_gripper(self._get_gripper_pub())
                self.get_logger().info('Opening gripper...')
                return

            if self.grasp_phase == 0:
                # Phase 0: Wait for gripper to open, then send plan
                if elapsed > 0.5:
                    self._send_plan_to_target(
                        self.pre_grasp_pose, 'pre-grasp')
                    self.grasp_phase = 1

            elif self.grasp_phase == 1:
                # Phase 1: Wait for /plan_to_target response
                if self.plan_future is not None and self.plan_future.done():
                    try:
                        result = self.plan_future.result()
                    except Exception as e:
                        self.get_logger().error(
                            f'[pre-grasp] Service exception: {e}')
                        self.transition_to(Task1State.ERROR)
                        return

                    if result and result.success:
                        self.get_logger().info(
                            f'  [pre-grasp] OK  '
                            f'pos_err={result.position_error:.4f}m')
                        self.grasp_phase = 2
                        self.phase_start_time = time.time()
                    else:
                        msg = result.message if result else 'No response'
                        self.get_logger().error(
                            f'  [pre-grasp] FAILED: {msg}')
                        self.transition_to(Task1State.ERROR)
                elif elapsed > 20.0:
                    self.get_logger().error('[pre-grasp] Timed out')
                    self.transition_to(Task1State.ERROR)

            elif self.grasp_phase == 2:
                # Phase 2: Wait for arm motion to finish
                if time.time() - self.phase_start_time >= \
                        GRASP_MOVE_DURATION + 0.5:
                    self.transition_to(Task1State.GRASP)

        # ==================== GRASP ====================
        elif self.state == Task1State.GRASP:
            if elapsed < 0.1:
                self.grasp_phase = 0
                self.plan_future = None
                return

            if self.grasp_phase == 0:
                # Phase 0: Send plan_to_target for grasp pose
                self._send_plan_to_target(self.grasp_pose, 'grasp')
                self.grasp_phase = 1

            elif self.grasp_phase == 1:
                # Phase 1: Wait for response
                if self.plan_future is not None and self.plan_future.done():
                    try:
                        result = self.plan_future.result()
                    except Exception as e:
                        self.get_logger().error(
                            f'[grasp] Service exception: {e}')
                        self.transition_to(Task1State.ERROR)
                        return

                    if result and result.success:
                        self.get_logger().info(
                            f'  [grasp] OK  '
                            f'pos_err={result.position_error:.4f}m')
                        self.grasp_phase = 2
                        self.phase_start_time = time.time()
                    else:
                        msg = result.message if result else 'No response'
                        self.get_logger().error(f'  [grasp] FAILED: {msg}')
                        self.transition_to(Task1State.ERROR)
                elif elapsed > 20.0:
                    self.get_logger().error('[grasp] Timed out')
                    self.transition_to(Task1State.ERROR)

            elif self.grasp_phase == 2:
                # Phase 2: Wait for arm motion, then close gripper
                if time.time() - self.phase_start_time >= \
                        GRASP_MOVE_DURATION + 0.5:
                    self.get_logger().info('Closing gripper...')
                    close_gripper(self._get_gripper_pub())
                    self.grasp_phase = 3
                    self.phase_start_time = time.time()

            elif self.grasp_phase == 3:
                # Phase 3: Wait for gripper to close, then verify
                if time.time() - self.phase_start_time >= GRIPPER_SETTLE_TIME:
                    self.transition_to(Task1State.VERIFY_GRASP)

        # ==================== VERIFY_GRASP ====================
        elif self.state == Task1State.VERIFY_GRASP:
            if elapsed < 0.05 \
                    and self.verify_sub_state == VerifySubState.PRESENT_ARM:
                self._begin_verify()
            self._run_verify_grasp(elapsed)

        # ==================== RETURN ====================
        elif self.state == Task1State.RETURN:
            if self.start_pose is None:
                self.get_logger().error('No start_pose recorded!')
                self.transition_to(Task1State.ERROR)
                return

            if elapsed < 0.1:
                # Disable nav_node so it doesn't fight for /cmd_vel
                self.nav_node.target_object = None
                # Aim camera forward for obstacle avoidance
                self._send_pan_tilt(0.0, 0.3)
                return

            # Wait briefly for arm to return home (sent in verify DECIDE)
            if elapsed < 3.0:
                self._stop_base()
                return

            pose = self._get_base_pose_in_odom()
            if pose is None:
                return

            cx, cy, _ = pose
            goal_x, goal_y = self.start_pose[0], self.start_pose[1]
            dx = goal_x - cx
            dy = goal_y - cy
            distance = np.sqrt(dx**2 + dy**2)

            if distance < RETURN_GOAL_TOLERANCE:
                self._stop_base()
                self.get_logger().info(
                    f'Arrived at start pose (dist={distance:.2f}m)')
                self.transition_to(Task1State.DONE)
                return

            # Drive toward start with obstacle avoidance
            self._drive_toward_goal(goal_x, goal_y, RETURN_SPEED)

        # ==================== DONE ====================
        elif self.state == Task1State.DONE:
            if elapsed < 0.1:
                self._stop_base()
                # Open gripper to release object
                open_gripper(self._get_gripper_pub())
                self._send_arm_cmd(ARM_HOME, 2.0)
                self.get_logger().info('')
                self.get_logger().info('=' * 50)
                self.get_logger().info('Task 1 complete!')
                self.get_logger().info('=' * 50)

        # ==================== ERROR ====================
        elif self.state == Task1State.ERROR:
            if elapsed < 0.1:
                self._stop_base()
                open_gripper(self._get_gripper_pub())
                self._send_arm_cmd(ARM_HOME, 2.0)
                self.get_logger().error('Task 1 failed — stopping robot')


# ═══════════════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)

    # Create NavigateToObject (driven by its own timer internally)
    nav_node = NavigateToObject(target_object='banana')

    # Create master state machine, passing nav_node reference
    task_node = Task1Retrieve(nav_node)

    # Run both nodes concurrently
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(nav_node)
    executor.add_node(task_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        task_node.cmd_vel_pub.publish(Twist())
        task_node.destroy_node()
        nav_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
