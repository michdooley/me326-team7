#!/usr/bin/env python3
"""
Task 1: Object Retrieval State Machine

Orchestrates the full object retrieval task:
    LISTEN → SEARCH → APPROACH → PLAN_GRASP → PRE_GRASP → GRASP →
    VERIFY_GRASP → RETURN → DONE

Each state calls a service or publishes a command, then waits for the result.
Heavy logic lives in service nodes (detector, localizer, grasp planner, motion planner).

VERIFY_GRASP sub-steps:
    1. PRESENT_ARM  — Move grasping arm to a pose visible to the camera
    2. AIM_CAMERA   — Tilt camera to look at the gripper region
    3. DETECT       — Call /detect with the target object class
    4. DECIDE       — If target detected near gripper → success → RETURN
                      If not → increment retry counter → back to APPROACH
                      If retries exhausted → ERROR

Topics used:
    /cmd_vel (Twist) - base velocity
    /camera/pan_tilt_cmd (Float64MultiArray) - camera pointing
    /right_arm/cmd, /left_arm/cmd (ArmCommand) - arm positions
    /right_gripper/cmd, /left_gripper/cmd (Float64MultiArray) - gripper control
    /detections (Detection2DArray) - object detections (subscribed)
    /object_poses (ObjectPose) - 3D object positions (subscribed)
    /joint_states (JointState) - robot state (subscribed)
    /odom (Odometry) - base position (subscribed)

Services called:
    /detect (Detect) - on-demand object detection
    /localize_object (LocalizeObject) - 2D→3D conversion
    /plan_grasp (PlanGrasp) - grasp pose planning
    /plan_to_target (PlanToTarget) - arm motion planning + execution

Usage:
    # Terminal 1: Launch sim + perception
    ros2 launch tidybot_bringup task.launch.py

    # Terminal 2: Run task
    ros2 run tidybot_bringup task1_retrieve.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import time
from enum import Enum, auto

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tidybot_msgs.msg import ArmCommand, Detection2DArray, ObjectPose
from tidybot_msgs.srv import Detect, LocalizeObject, PlanGrasp, PlanToTarget


class Task1State(Enum):
    """States for the object retrieval task."""
    LISTEN       = auto()  # Wait for voice command → get target object
    SEARCH       = auto()  # Scan environment for target object
    APPROACH     = auto()  # Navigate to grasping range
    PLAN_GRASP   = auto()  # Call /plan_grasp service
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
    DETECT      = auto()   # Call /detect service for target object
    DECIDE      = auto()   # Evaluate detection results


# ── Verification configuration ──────────────────────────────────────────────

MAX_GRASP_RETRIES = 3

# Arm pose that holds the grasped object in front of the camera.
# [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]
# Shoulder up, elbow bent back, wrist angled so object faces the camera.
VERIFY_ARM_POSE = [0.0, 0.4, -0.3, 0.0, 0.5, 0.0]
VERIFY_ARM_DURATION = 2.5  # seconds to reach the pose

# Camera pan/tilt to look at the gripper region during verification.
VERIFY_CAMERA_PAN = 0.0
VERIFY_CAMERA_TILT = 0.3
VERIFY_CAMERA_SETTLE_TIME = 1.5  # seconds to let the camera settle

# Minimum confidence to accept a detection as the target object in gripper.
VERIFY_DETECTION_CONFIDENCE = 0.4

# Minimum fraction of the image width/height the bounding box must occupy.
# Prevents accepting tiny background detections as "in the gripper."
VERIFY_MIN_BBOX_FRACTION = 0.05


class Task1Retrieve(Node):
    """Object retrieval state machine."""

    def __init__(self):
        super().__init__('task1_retrieve')

        # ---- Publishers ----
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.right_arm_pub = self.create_publisher(ArmCommand, '/right_arm/cmd', 10)
        self.left_arm_pub = self.create_publisher(ArmCommand, '/left_arm/cmd', 10)
        self.right_gripper_pub = self.create_publisher(Float64MultiArray, '/right_gripper/cmd', 10)
        self.left_gripper_pub = self.create_publisher(Float64MultiArray, '/left_gripper/cmd', 10)
        self.pan_tilt_pub = self.create_publisher(Float64MultiArray, '/camera/pan_tilt_cmd', 10)

        # ---- Subscribers ----
        self.detections_sub = self.create_subscription(
            Detection2DArray, '/detections', self.detections_callback, 10
        )
        self.object_poses_sub = self.create_subscription(
            ObjectPose, '/object_poses', self.object_poses_callback, 10
        )
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Camera RGB for resolution tracking (used in verify bounding-box check)
        img_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, img_qos
        )
        self.image_width = 640   # sensible defaults, updated on first frame
        self.image_height = 480

        # ---- Service Clients ----
        self.detect_client = self.create_client(Detect, '/detect')
        self.localize_client = self.create_client(LocalizeObject, '/localize_object')
        self.plan_grasp_client = self.create_client(PlanGrasp, '/plan_grasp')
        self.plan_to_target_client = self.create_client(PlanToTarget, '/plan_to_target')

        # ---- State machine ----
        self.state = Task1State.LISTEN
        self.state_start_time = None

        # ---- Task data ----
        self.target_object = None       # e.g. "apple"
        self.latest_detections = None
        self.latest_object_pose = None
        self.latest_joint_states = None
        self.latest_odom = None
        self.start_pose = None          # where to return to
        self.grasp_plan = None          # result from /plan_grasp
        self.active_arm = 'right'       # which arm to use

        # ---- Verify-specific data ----
        self.verify_sub_state = VerifySubState.PRESENT_ARM
        self.grasp_attempt_count = 0
        self.verify_detect_future = None
        self.verify_arm_cmd_sent = False
        self.verify_camera_cmd_sent = False

        # Control loop at 50Hz
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info('=' * 50)
        self.get_logger().info('Task 1: Object Retrieval')
        self.get_logger().info('=' * 50)
        self.get_logger().info(
            'States: LISTEN → SEARCH → APPROACH → PLAN_GRASP → '
            'PRE_GRASP → GRASP → VERIFY → RETURN → DONE'
        )
        self.get_logger().info(f'Max grasp retries: {MAX_GRASP_RETRIES}')

    # ---- Subscriber callbacks ----

    def detections_callback(self, msg: Detection2DArray):
        self.latest_detections = msg

    def object_poses_callback(self, msg: ObjectPose):
        self.latest_object_pose = msg

    def joint_states_callback(self, msg: JointState):
        self.latest_joint_states = msg

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def rgb_callback(self, msg: Image):
        self.image_width = msg.width
        self.image_height = msg.height

    # ---- State machine ----

    def transition_to(self, new_state: Task1State):
        """Transition to a new state."""
        self.get_logger().info(f'State: {self.state.name} → {new_state.name}')
        self.state = new_state
        self.state_start_time = time.time()

    def _begin_verify(self):
        """Reset verify sub-state for a fresh verification pass."""
        self.verify_sub_state = VerifySubState.PRESENT_ARM
        self.verify_detect_future = None
        self.verify_arm_cmd_sent = False
        self.verify_camera_cmd_sent = False
        self.grasp_attempt_count += 1
        self.get_logger().info(
            f'Starting grasp verification (attempt {self.grasp_attempt_count}/{MAX_GRASP_RETRIES})')

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

    def _detection_matches_target(self, detections) -> bool:
        """
        Return True if any detection in the list is the target object,
        with sufficient confidence and bounding-box size (to confirm it's
        actually in the gripper, not a far-away background instance).
        """
        if not detections:
            return False

        min_w = self.image_width * VERIFY_MIN_BBOX_FRACTION
        min_h = self.image_height * VERIFY_MIN_BBOX_FRACTION

        for det in detections:
            name_match = (det.class_name.lower() == self.target_object.lower())
            conf_ok = det.confidence >= VERIFY_DETECTION_CONFIDENCE
            size_ok = det.width >= min_w and det.height >= min_h
            if name_match and conf_ok and size_ok:
                self.get_logger().info(
                    f'  Verified: "{det.class_name}" conf={det.confidence:.2f} '
                    f'bbox={det.width}x{det.height}')
                return True
        return False

    # ---- Verify sub-state machine (called from control_loop) ----

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

        # ── DETECT: call /detect for the target object class ─────────────
        elif self.verify_sub_state == VerifySubState.DETECT:
            if self.verify_detect_future is None:
                if not self.detect_client.service_is_ready():
                    if elapsed > 5.0:
                        self.get_logger().warn(
                            '  [verify] /detect service not available — skipping visual check')
                        self.verify_sub_state = VerifySubState.DECIDE
                        self.state_start_time = time.time()
                    return

                self.get_logger().info(
                    f'  [verify] Calling /detect for "{self.target_object}"...')
                req = Detect.Request()
                req.target_class = self.target_object
                self.verify_detect_future = self.detect_client.call_async(req)
                return

            if self.verify_detect_future.done():
                self.verify_sub_state = VerifySubState.DECIDE
                self.state_start_time = time.time()
            elif elapsed > 10.0:
                self.get_logger().warn('  [verify] /detect call timed out')
                self.verify_detect_future.cancel()
                self.verify_detect_future = None
                self.verify_sub_state = VerifySubState.DECIDE
                self.state_start_time = time.time()

        # ── DECIDE: evaluate detection results ───────────────────────────
        elif self.verify_sub_state == VerifySubState.DECIDE:
            grasp_success = False

            if self.verify_detect_future is not None and self.verify_detect_future.done():
                try:
                    result = self.verify_detect_future.result()
                    if result.success:
                        grasp_success = self._detection_matches_target(result.detections)
                        if not grasp_success:
                            self.get_logger().info(
                                f'  [verify] Target "{self.target_object}" not found in detections')
                    else:
                        self.get_logger().warn(f'  [verify] Detection failed: {result.message}')
                except Exception as e:
                    self.get_logger().error(f'  [verify] Detection exception: {e}')

            if grasp_success:
                self.get_logger().info(
                    '  [verify] ✓ Grasp confirmed — object in gripper')
                self.transition_to(Task1State.RETURN)
            elif self.grasp_attempt_count >= MAX_GRASP_RETRIES:
                self.get_logger().error(
                    f'  [verify] ✗ Grasp failed after {MAX_GRASP_RETRIES} attempts — giving up')
                self.transition_to(Task1State.ERROR)
            else:
                self.get_logger().warn(
                    f'  [verify] ✗ Grasp not confirmed — retrying '
                    f'(attempt {self.grasp_attempt_count}/{MAX_GRASP_RETRIES})')
                self.transition_to(Task1State.APPROACH)

    # ---- Main control loop ----

    def control_loop(self):
        """Main control loop — runs the state machine at 50Hz."""
        now = time.time()

        if self.state_start_time is None:
            self.state_start_time = now
            self.get_logger().info(f'Starting state: {self.state.name}')

        elapsed = now - self.state_start_time

        # ==================== LISTEN ====================
        if self.state == Task1State.LISTEN:
            # TODO: Call voice_command module to get target object
            # For now, can be hardcoded for testing:
            # self.target_object = "apple"
            # self.start_pose = self.latest_odom
            # self.transition_to(Task1State.SEARCH)
            pass

        # ==================== SEARCH ====================
        elif self.state == Task1State.SEARCH:
            # TODO: Use NavigateToObject class or scan behavior
            # Rotate in place, check /detections for target_object
            # If found, transition to APPROACH
            pass

        # ==================== APPROACH ====================
        elif self.state == Task1State.APPROACH:
            # TODO: Drive toward object using /object_poses
            # Stop when within grasping range
            # Transition to PLAN_GRASP when positioned
            pass

        # ==================== PLAN_GRASP ====================
        elif self.state == Task1State.PLAN_GRASP:
            # TODO: Call /plan_grasp service with object position
            # Store result in self.grasp_plan
            # Transition to PRE_GRASP on success
            pass

        # ==================== PRE_GRASP ====================
        elif self.state == Task1State.PRE_GRASP:
            # TODO: Call /plan_to_target with grasp_plan.pre_grasp_pose
            # Open gripper
            # Transition to GRASP when arm is in position
            pass

        # ==================== GRASP ====================
        elif self.state == Task1State.GRASP:
            # TODO: Call /plan_to_target with grasp_plan.grasp_pose
            # Close gripper
            # Transition to VERIFY_GRASP
            pass

        # ==================== VERIFY_GRASP ====================
        elif self.state == Task1State.VERIFY_GRASP:
            if elapsed < 0.05 and self.verify_sub_state == VerifySubState.PRESENT_ARM:
                self._begin_verify()
            self._run_verify_grasp(elapsed)

        # ==================== RETURN ====================
        elif self.state == Task1State.RETURN:
            # TODO: Navigate back to self.start_pose
            # Transition to DONE when arrived
            pass

        # ==================== DONE ====================
        elif self.state == Task1State.DONE:
            if elapsed < 0.1:
                self.get_logger().info('')
                self.get_logger().info('=' * 50)
                self.get_logger().info('Task 1 complete!')
                self.get_logger().info('=' * 50)

        # ==================== ERROR ====================
        elif self.state == Task1State.ERROR:
            if elapsed < 0.1:
                # Stop everything
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().error('Task 1 failed — stopping robot')


def main(args=None):
    rclpy.init(args=args)
    node = Task1Retrieve()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop base on exit
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
