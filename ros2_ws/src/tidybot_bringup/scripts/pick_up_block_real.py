#!/usr/bin/env python3
"""
TidyBot2 Pick Up Block — Real Hardware

Uses camera vision to detect a red block, estimates its 3D position via depth,
drives the base closer if needed, then plans arm motion to approach, grasp,
and lift it.  Adapted from pick_up_block.py with real-hardware safety features:
  - Slower motion durations
  - Plan-only verification before execution
  - User confirmation prompts at critical steps
  - Smooth return to sleep pose on completion

Prerequisites:
    Launch real hardware with motion planner:
        ros2 launch tidybot_bringup real.launch.py use_planner:=true

Usage:
    ros2 run tidybot_bringup pick_up_block_real.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration

import numpy as np
import time

from geometry_msgs.msg import Pose, Pose2D
from sensor_msgs.msg import Image, CameraInfo, JointState
from std_msgs.msg import Bool, Float64MultiArray
from tidybot_msgs.srv import PlanToTarget
from interbotix_xs_msgs.msg import JointGroupCommand

import tf2_ros

try:
    from cv_bridge import CvBridge
    import cv2
except ImportError:
    print("ERROR: cv_bridge and opencv are required.")
    print("  Install with: pip install opencv-python")
    raise


class PickUpBlockReal(Node):
    """Detects a red block with the camera and picks it up (real hardware)."""

    # Fingers-down grasp orientation (quaternion wxyz in base_link frame)
    ORIENT_FINGERS_DOWN = (0.5, 0.5, 0.5, -0.5)

    # Right arm shoulder position in base_link frame
    RIGHT_SHOULDER = np.array([-0.15, -0.12, 0.45])

    # Maximum comfortable horizontal reach from shoulder (meters)
    MAX_REACH = 0.38

    # Ideal horizontal distance from shoulder to grasp target
    IDEAL_REACH = 0.30

    # Sleep pose for arms (tucked position)
    SLEEP_POSE = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]

    def __init__(self):
        super().__init__('pick_up_block_real')

        # ── Camera subscribers (BEST_EFFORT QoS to match RealSense publisher) ──
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cv_bridge = CvBridge()

        self.latest_rgb = None
        self.latest_depth = None
        self.camera_K = None  # 3x3 intrinsic matrix

        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self._rgb_cb, qos)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self._depth_cb, qos)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._camera_info_cb, qos)

        # ── Camera pan/tilt publisher ──
        self.pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10)

        # ── Gripper publisher ──
        self.right_gripper_pub = self.create_publisher(
            Float64MultiArray, '/right_gripper/cmd', 10)

        # ── Base control ──
        self.base_target_pub = self.create_publisher(
            Pose2D, '/base/target_pose', 10)
        self.base_goal_reached = False
        self.goal_sub = self.create_subscription(
            Bool, '/base/goal_reached', self._goal_reached_cb, 10)

        # ── Motion planning service client ──
        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')

        # ── Direct arm command publishers (for sleep pose return) ──
        self.arm_cmd_pubs = {
            'right': self.create_publisher(
                JointGroupCommand, '/right_arm/commands/joint_group', 10),
            'left': self.create_publisher(
                JointGroupCommand, '/left_arm/commands/joint_group', 10),
        }

        # ── Joint state tracking ──
        self.joint_states_received = False
        self.current_joint_positions = {}
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)

        # ── TF2 for coordinate transforms ──
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('=' * 50)
        self.get_logger().info('TidyBot2 Pick-Up-Block (Real Hardware)')
        self.get_logger().info('=' * 50)

    # ── Subscriber callbacks ─────────────────────────────────────────

    def _rgb_cb(self, msg: Image):
        try:
            self.latest_rgb = self.cv_bridge.imgmsg_to_cv2(msg, 'rgb8')
        except Exception as e:
            self.get_logger().warn(f'RGB conversion failed: {e}')

    def _depth_cb(self, msg: Image):
        try:
            self.latest_depth = self.cv_bridge.imgmsg_to_cv2(msg, '16UC1')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')

    def _camera_info_cb(self, msg: CameraInfo):
        self.camera_K = np.array(msg.k).reshape(3, 3)

    def _goal_reached_cb(self, msg: Bool):
        if msg.data:
            self.base_goal_reached = True

    def _joint_state_cb(self, msg: JointState):
        self.joint_states_received = True
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    # ── Helpers ───────────────────────────────────────────────────────

    def spin_for(self, seconds: float):
        """Spin the node for a given duration to process callbacks."""
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_data(self):
        """Block until camera data, joint states, and TF are available."""
        self.get_logger().info('Waiting for camera feed, depth, intrinsics, joint states, and TF...')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.latest_rgb is not None
                    and self.latest_depth is not None
                    and self.camera_K is not None
                    and self.joint_states_received):
                try:
                    self.tf_buffer.lookup_transform(
                        'base_link', 'camera_color_optical_frame',
                        rclpy.time.Time(), timeout=Duration(seconds=0.1))
                    break
                except (tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException):
                    pass
        self.get_logger().info('All sensor data ready!')

    def wait_for_planner(self):
        """Block until the /plan_to_target service is available."""
        self.get_logger().info('Waiting for /plan_to_target service...')
        if not self.plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                'Service not available! Make sure motion_planner_real_node is running.')
            self.get_logger().error(
                '  Launch with: ros2 launch tidybot_bringup real.launch.py use_planner:=true')
            raise RuntimeError('Planning service not available')
        self.get_logger().info('Planner service connected!')

    def send_pan_tilt(self, pan: float, tilt: float):
        """Send a pan/tilt command."""
        msg = Float64MultiArray()
        msg.data = [pan, tilt]
        self.pan_tilt_pub.publish(msg)

    def send_gripper(self, position: float):
        """Send gripper command (0.0 = open, 1.0 = closed)."""
        msg = Float64MultiArray()
        msg.data = [float(position)]
        self.right_gripper_pub.publish(msg)

    def create_pose(self, x: float, y: float, z: float,
                    qw: float = 1.0, qx: float = 0.0,
                    qy: float = 0.0, qz: float = 0.0) -> Pose:
        """Create a geometry_msgs/Pose."""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = qw
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        return pose

    def call_planner(self, arm_name: str, pose: Pose,
                     use_orientation: bool = True,
                     duration: float = 3.0,
                     plan_only: bool = False) -> bool:
        """Call the /plan_to_target service synchronously.

        Args:
            arm_name: 'right' or 'left'
            pose: Target end-effector pose in base_link
            use_orientation: Whether to track orientation
            duration: Execution duration in seconds
            plan_only: If True, plan without executing (for verification)

        Returns:
            True on success, False on failure.
        """
        request = PlanToTarget.Request()
        request.arm_name = arm_name
        request.target_pose = pose
        request.use_orientation = use_orientation
        request.execute = not plan_only
        request.duration = duration
        request.max_condition_number = 100.0

        pos = pose.position
        mode = "Planning" if plan_only else "Planning+executing"
        self.get_logger().info(
            f'  {mode} {arm_name} arm to: '
            f'({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})')

        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

        if not future.done():
            self.get_logger().error('  Service call timed out!')
            return False

        if future.exception() is not None:
            self.get_logger().error(f'  Service exception: {future.exception()}')
            return False

        result = future.result()
        if result.success:
            self.get_logger().info(f'  SUCCESS: {result.message}')
            if use_orientation:
                self.get_logger().info(
                    f'  Errors: pos={result.position_error:.4f}m, '
                    f'ori={result.orientation_error:.4f}rad '
                    f'({np.degrees(result.orientation_error):.1f}°)')
            if result.executed:
                self.get_logger().info(f'  Executing over {duration}s...')
            return True
        else:
            self.get_logger().warn(f'  FAILED: {result.message}')
            return False

    # ── Arm sleep pose ─────────────────────────────────────────────────

    def get_arm_positions(self, arm_name: str) -> np.ndarray:
        """Get current joint positions for an arm."""
        joint_names = [
            f'{arm_name}_waist', f'{arm_name}_shoulder', f'{arm_name}_elbow',
            f'{arm_name}_forearm_roll', f'{arm_name}_wrist_angle',
            f'{arm_name}_wrist_rotate']
        return np.array([
            self.current_joint_positions.get(name, 0.0) for name in joint_names])

    def go_to_sleep_pose(self, arm_name: str, max_joint_speed: float = 0.5):
        """Send arm to sleep pose using smooth interpolated trajectory.

        Args:
            arm_name: 'right' or 'left'
            max_joint_speed: Maximum joint velocity in rad/s (default 0.5 ~ 30 deg/s)
        """
        rclpy.spin_once(self, timeout_sec=0.1)

        current = self.get_arm_positions(arm_name)
        target = np.array(self.SLEEP_POSE)

        max_diff = np.max(np.abs(target - current))
        duration = max(max_diff / max_joint_speed, 1.0)

        self.get_logger().info(
            f'Moving {arm_name} arm to sleep pose over {duration:.1f}s '
            f'(max joint diff: {max_diff:.2f} rad)')

        rate_hz = 50.0
        dt = 1.0 / rate_hz
        num_steps = max(int(duration * rate_hz), 1)

        for i in range(num_steps + 1):
            t = i / num_steps
            alpha = 0.5 * (1 - np.cos(np.pi * t))

            q = current + alpha * (target - current)

            cmd = JointGroupCommand()
            cmd.name = f'{arm_name}_arm'
            cmd.cmd = q.tolist()
            self.arm_cmd_pubs[arm_name].publish(cmd)

            if i < num_steps:
                time.sleep(dt)

    # ── Vision: detect red block ──────────────────────────────────────

    def detect_red_block(self) -> tuple:
        """Detect the red block in the latest RGB image.

        Returns:
            (cx, cy) pixel coordinates of the block centroid, or None if not found.
        """
        if self.latest_rgb is None:
            return None

        bgr = cv2.cvtColor(self.latest_rgb, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # Red wraps around H=0/180, so use two ranges
        lower_red1 = np.array([0, 100, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < 50:
            return None

        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        self.get_logger().info(f'  Red block detected at pixel ({cx}, {cy}), area={area:.0f}px')
        return (cx, cy)

    # ── Depth → 3D → base_link transform ──────────────────────────────

    def pixel_to_base_link(self, cx: int, cy: int) -> np.ndarray:
        """Convert pixel (cx, cy) + depth to a 3D point in base_link frame.

        Returns:
            np.array([x, y, z]) in base_link, or None on failure.
        """
        if self.latest_depth is None or self.camera_K is None:
            return None

        depth_mm = int(self.latest_depth[cy, cx])
        if depth_mm == 0:
            self.get_logger().warn('  Depth at block center is 0 (invalid)')
            return None
        depth_m = depth_mm / 1000.0

        fx = self.camera_K[0, 0]
        fy = self.camera_K[1, 1]
        px = self.camera_K[0, 2]
        py = self.camera_K[1, 2]

        x_cam = (cx - px) * depth_m / fx
        y_cam = (cy - py) * depth_m / fy
        z_cam = depth_m

        self.get_logger().info(
            f'  Camera frame 3D: ({x_cam:.3f}, {y_cam:.3f}, {z_cam:.3f}), '
            f'depth={depth_m:.3f}m')

        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'camera_color_optical_frame',
                rclpy.time.Time(), timeout=Duration(seconds=1.0))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'  TF lookup failed: {e}')
            return None

        t = transform.transform.translation
        q = transform.transform.rotation
        trans = np.array([t.x, t.y, t.z])

        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
        ])

        point_cam = np.array([x_cam, y_cam, z_cam])
        point_base = R @ point_cam + trans

        self.get_logger().info(
            f'  Base_link 3D: ({point_base[0]:.3f}, {point_base[1]:.3f}, {point_base[2]:.3f})')

        return point_base

    # ── Base movement ─────────────────────────────────────────────────

    def get_base_pose_in_odom(self):
        """Get current base position and heading in the odom frame.

        Returns:
            (x, y, theta) in odom frame, or None on failure.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time(), timeout=Duration(seconds=1.0))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'  TF odom->base_link lookup failed: {e}')
            return None

        t = transform.transform.translation
        q = transform.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = np.arctan2(siny, cosy)
        return (t.x, t.y, theta)

    def compute_reach(self, block_base_link: np.ndarray) -> float:
        """Compute horizontal distance from right arm shoulder to a point in base_link."""
        dx = block_base_link[0] - self.RIGHT_SHOULDER[0]
        dy = block_base_link[1] - self.RIGHT_SHOULDER[1]
        return np.sqrt(dx**2 + dy**2)

    def move_base_closer(self, block_base_link: np.ndarray) -> bool:
        """Drive the base forward so the block is within comfortable arm reach.

        Returns:
            True if the base moved (need to re-detect), False if already close enough.
        """
        reach = self.compute_reach(block_base_link)
        self.get_logger().info(f'  Current reach to block: {reach:.3f}m (max={self.MAX_REACH}m)')

        if reach <= self.MAX_REACH:
            self.get_logger().info('  Block is within arm reach — no base movement needed.')
            return False

        move_forward = reach - self.IDEAL_REACH
        self.get_logger().info(f'  Need to drive {move_forward:.3f}m closer')

        base_pose = self.get_base_pose_in_odom()
        if base_pose is None:
            self.get_logger().error('  Cannot get base pose — skipping base move.')
            return False

        ox, oy, otheta = base_pose
        self.get_logger().info(
            f'  Current base in odom: ({ox:.3f}, {oy:.3f}, '
            f'theta={np.degrees(otheta):.1f}°)')

        user_heading = otheta - np.pi / 2

        target_x = ox + move_forward * np.cos(user_heading)
        target_y = oy + move_forward * np.sin(user_heading)
        target_theta = user_heading

        self.get_logger().info(
            f'  Sending base target: ({target_x:.3f}, {target_y:.3f}, '
            f'theta={np.degrees(target_theta):.1f}°)')

        msg = Pose2D()
        msg.x = target_x
        msg.y = target_y
        msg.theta = target_theta
        self.base_goal_reached = False
        self.base_target_pub.publish(msg)

        # Wait for goal reached (with timeout)
        timeout = time.time() + 20.0  # Longer timeout for real hardware
        while not self.base_goal_reached and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.base_goal_reached:
            self.get_logger().info('  Base reached target position!')
        else:
            self.get_logger().warn('  Base movement timed out — continuing anyway.')

        # Let things settle (extra time for real hardware)
        self.spin_for(2.0)
        return True

    # ── Detect + approach loop ────────────────────────────────────────

    def detect_and_localize(self) -> np.ndarray:
        """Pan camera, detect block, compute 3D position in base_link.

        Returns:
            np.array([x, y, z]) in base_link, or None.
        """
        self.get_logger().info('  Panning camera down...')
        self.send_pan_tilt(0.0, 0.6)
        self.spin_for(2.0)

        pixel = None
        for attempt in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
            pixel = self.detect_red_block()
            if pixel is not None:
                break
            if attempt > 0:
                self.get_logger().info(
                    f'  Attempt {attempt + 1}: no red block found, retrying...')
            self.spin_for(0.5)

        if pixel is None:
            return None

        cx, cy = pixel
        return self.pixel_to_base_link(cx, cy)

    # ── Main pick-up sequence ─────────────────────────────────────────

    def run(self):
        """Execute the full pick-up-block sequence on real hardware."""

        qw, qx, qy, qz = self.ORIENT_FINGERS_DOWN

        # ── Step 0: Wait for everything ──
        self.wait_for_planner()
        self.wait_for_data()
        self.get_logger().info('')

        # ── Step 1: Detect block and drive closer if needed ──
        self.get_logger().info('-' * 40)
        self.get_logger().info('Step 1: Locate block and approach')

        MAX_APPROACH_ATTEMPTS = 5
        block_pos = None

        for attempt in range(MAX_APPROACH_ATTEMPTS):
            self.get_logger().info(
                f'  --- Locate attempt {attempt + 1}/{MAX_APPROACH_ATTEMPTS} ---')

            target = self.detect_and_localize()
            if target is None:
                self.get_logger().error('Could not detect red block!')
                self.get_logger().error('  Make sure a red block is visible to the camera.')
                return

            reach = self.compute_reach(target)
            self.get_logger().info(
                f'  Block at ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}), '
                f'reach={reach:.3f}m')

            if reach <= self.MAX_REACH:
                block_pos = target
                self.get_logger().info('  Block is within reach!')
                break

            moved = self.move_base_closer(target)
            if not moved:
                block_pos = target
                break

        if block_pos is None:
            self.get_logger().error(
                'Could not get close enough to block after max attempts!')
            return

        bx, by, bz = block_pos[0], block_pos[1], block_pos[2]
        self.get_logger().info(f'  Final target: ({bx:.3f}, {by:.3f}, {bz:.3f})')

        # ── Step 2: Verify arm motions with plan-only ──
        self.get_logger().info('-' * 40)
        self.get_logger().info('Step 2: Verifying arm motions (plan only, no execution)')

        approach_pose = self.create_pose(bx, by, bz + 0.10, qw, qx, qy, qz)
        grasp_pose = self.create_pose(bx, by, bz + 0.02, qw, qx, qy, qz)
        lift_pose = self.create_pose(bx, by, bz + 0.15, qw, qx, qy, qz)

        self.get_logger().info('  Verifying approach pose...')
        if not self.call_planner('right', approach_pose, use_orientation=True,
                                 plan_only=True):
            self.get_logger().error('Approach pose is not reachable! Aborting.')
            return

        self.get_logger().info('  Verifying grasp pose...')
        if not self.call_planner('right', grasp_pose, use_orientation=True,
                                 plan_only=True):
            self.get_logger().error('Grasp pose is not reachable! Aborting.')
            return

        self.get_logger().info('  Verifying lift pose...')
        if not self.call_planner('right', lift_pose, use_orientation=True,
                                 plan_only=True):
            self.get_logger().error('Lift pose is not reachable! Aborting.')
            return

        self.get_logger().info('  All poses verified!')

        # ── User confirmation before moving the arm ──
        self.get_logger().info('')
        self.get_logger().info('The robot arm will now move. Make sure the workspace is clear.')
        input('  Press Enter to start arm motions (Ctrl+C to abort)...\n')

        # ── Step 3: Open gripper ──
        self.get_logger().info('-' * 40)
        self.get_logger().info('Step 3: Opening right gripper')
        self.send_gripper(0.0)
        self.spin_for(1.5)
        self.get_logger().info('  Gripper open.')

        # ── Step 4: Move arm above the block (approach) ──
        self.get_logger().info('-' * 40)
        self.get_logger().info('Step 4: Moving arm above block (approach)')
        success = self.call_planner('right', approach_pose,
                                    use_orientation=True, duration=3.0)
        if not success:
            self.get_logger().error('Approach motion failed!')
            return
        self.spin_for(3.5)

        # ── Step 5: Descend to block grasp height ──
        self.get_logger().info('-' * 40)
        self.get_logger().info('Step 5: Descending to grasp height')
        success = self.call_planner('right', grasp_pose,
                                    use_orientation=True, duration=3.0)
        if not success:
            self.get_logger().error('Descend motion failed!')
            return
        self.spin_for(3.5)

        # ── Step 6: Close gripper to grasp ──
        self.get_logger().info('-' * 40)
        self.get_logger().info('Step 6: Closing gripper to grasp block')
        self.send_gripper(1.0)
        self.spin_for(2.0)
        self.get_logger().info('  Gripper closed.')

        # ── Step 7: Lift the block ──
        self.get_logger().info('-' * 40)
        self.get_logger().info('Step 7: Lifting block')
        success = self.call_planner('right', lift_pose,
                                    use_orientation=True, duration=3.0)
        if not success:
            self.get_logger().error('Lift motion failed!')
            return
        self.spin_for(3.5)

        # ── Done ──
        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Block pick-up complete!')
        self.get_logger().info('=' * 50)

        input('\n  Press Enter to return arm to sleep pose...\n')

        # ── Return to sleep pose ──
        self.get_logger().info('Returning right arm to sleep pose...')
        self.send_gripper(0.0)
        self.spin_for(1.0)
        self.go_to_sleep_pose('right')
        self.spin_for(1.0)
        self.get_logger().info('Done!')


def main(args=None):
    rclpy.init(args=args)
    node = PickUpBlockReal()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted — returning arm to sleep pose...')
        try:
            node.send_gripper(0.0)
            node.spin_for(0.5)
            node.go_to_sleep_pose('right')
            node.spin_for(1.0)
        except Exception:
            pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
