#!/usr/bin/env python3
"""
TidyBot2 Pick Up Block Demo

Uses camera vision to detect a red block, estimates its 3D position via depth,
drives the base closer if needed, then plans arm motion to approach, grasp,
and lift it.

Prerequisites:
    Launch simulation with the pickup scene (has a red block):
        ros2 launch tidybot_bringup sim.launch.py scene:=scene_pickup.xml

Usage:
    ros2 run tidybot_bringup pick_up_block.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration

import numpy as np
import time

from geometry_msgs.msg import Pose, Pose2D
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float64MultiArray
from tidybot_msgs.srv import PlanToTarget

import tf2_ros

try:
    from cv_bridge import CvBridge
    import cv2
except ImportError:
    print("ERROR: cv_bridge and opencv are required.")
    print("  Install with: pip install opencv-python")
    raise


class PickUpBlock(Node):
    """Detects a red block with the camera and picks it up."""

    # Fingers-down grasp orientation (quaternion wxyz in base_link frame)
    ORIENT_FINGERS_DOWN = (0.5, 0.5, 0.5, -0.5)

    # Right arm shoulder position in base_link frame
    RIGHT_SHOULDER = np.array([-0.15, -0.12, 0.45])

    # Maximum comfortable horizontal reach from shoulder (meters)
    MAX_REACH = 0.38

    # Ideal horizontal distance from shoulder to grasp target
    IDEAL_REACH = 0.30

    def __init__(self):
        super().__init__('pick_up_block')

        # ── Camera subscribers (BEST_EFFORT QoS to match sim publisher) ──
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

        # ── TF2 for coordinate transforms ──
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('=' * 50)
        self.get_logger().info('TidyBot2 Pick-Up-Block Demo')
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

    # ── Helpers ───────────────────────────────────────────────────────

    def spin_for(self, seconds: float):
        """Spin the node for a given duration to process callbacks."""
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_data(self):
        """Block until camera data and TF are available."""
        self.get_logger().info('Waiting for camera feed, depth, intrinsics, and TF...')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.latest_rgb is not None
                    and self.latest_depth is not None
                    and self.camera_K is not None):
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
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('  Service not available, waiting...')
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
                     duration: float = 2.0) -> bool:
        """Call the /plan_to_target service synchronously and wait for execution."""
        request = PlanToTarget.Request()
        request.arm_name = arm_name
        request.target_pose = pose
        request.use_orientation = use_orientation
        request.execute = True
        request.duration = duration
        request.max_condition_number = 100.0

        pos = pose.position
        self.get_logger().info(
            f'  Planning {arm_name} arm to: '
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
            if result.executed:
                self.get_logger().info(f'  Executing over {duration}s...')
            return True
        else:
            self.get_logger().warn(f'  FAILED: {result.message}')
            return False

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
        # Extract yaw from quaternion
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

        The block position is in base_link frame where -Y is forward.
        We compute how far to drive and send a /base/target_pose command.

        Returns:
            True if the base moved (need to re-detect), False if already close enough.
        """
        reach = self.compute_reach(block_base_link)
        self.get_logger().info(f'  Current reach to block: {reach:.3f}m (max={self.MAX_REACH}m)')

        if reach <= self.MAX_REACH:
            self.get_logger().info('  Block is within arm reach — no base movement needed.')
            return False

        # Compute how far forward to drive.
        # The block is mostly in the -Y direction in base_link (forward).
        # We want to reduce the reach to IDEAL_REACH.
        move_forward = reach - self.IDEAL_REACH
        self.get_logger().info(f'  Need to drive {move_forward:.3f}m closer')

        # Get current base pose in odom frame
        base_pose = self.get_base_pose_in_odom()
        if base_pose is None:
            self.get_logger().error('  Cannot get base pose — skipping base move.')
            return False

        ox, oy, otheta = base_pose
        self.get_logger().info(f'  Current base in odom: ({ox:.3f}, {oy:.3f}, theta={np.degrees(otheta):.1f}°)')

        # The robot's forward direction in odom is determined by its heading.
        # target_pose uses the "user frame" where x/y map directly to odom world coords,
        # and theta=0 means facing +X (forward). The bridge adds +π/2 internally.
        # So the user-facing heading is: otheta - π/2
        user_heading = otheta - np.pi / 2

        # Compute target position in odom/user frame
        target_x = ox + move_forward * np.cos(user_heading)
        target_y = oy + move_forward * np.sin(user_heading)
        # Keep the same user-facing heading (theta=0 relative to initial forward)
        target_theta = user_heading

        self.get_logger().info(
            f'  Sending base target: ({target_x:.3f}, {target_y:.3f}, '
            f'theta={np.degrees(target_theta):.1f}°)')

        # Send target
        msg = Pose2D()
        msg.x = target_x
        msg.y = target_y
        msg.theta = target_theta
        self.base_goal_reached = False
        self.base_target_pub.publish(msg)

        # Wait for goal reached (with timeout)
        timeout = time.time() + 15.0
        while not self.base_goal_reached and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.base_goal_reached:
            self.get_logger().info('  Base reached target position!')
        else:
            self.get_logger().warn('  Base movement timed out — continuing anyway.')

        # Let things settle
        self.spin_for(1.0)
        return True

    # ── Detect + approach loop ────────────────────────────────────────

    def detect_and_localize(self) -> np.ndarray:
        """Pan camera, detect block, compute 3D position in base_link.

        Returns:
            np.array([x, y, z]) in base_link, or None.
        """
        # Pan camera down
        self.get_logger().info('  Panning camera down...')
        self.send_pan_tilt(0.0, 0.6)
        self.spin_for(2.0)

        # Detect red block
        pixel = None
        for attempt in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
            pixel = self.detect_red_block()
            if pixel is not None:
                break
            if attempt > 0:
                self.get_logger().info(f'  Attempt {attempt + 1}: no red block found, retrying...')
            self.spin_for(0.5)

        if pixel is None:
            return None

        cx, cy = pixel

        # Compute 3D in base_link
        return self.pixel_to_base_link(cx, cy)

    # ── Main pick-up sequence ─────────────────────────────────────────

    def run(self):
        """Execute the full pick-up-block sequence."""

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
            self.get_logger().info(f'  --- Locate attempt {attempt + 1}/{MAX_APPROACH_ATTEMPTS} ---')

            target = self.detect_and_localize()
            if target is None:
                self.get_logger().error('Could not detect red block! Is the scene loaded?')
                self.get_logger().error('  Launch with: ros2 launch tidybot_bringup sim.launch.py scene:=scene_pickup.xml')
                return

            reach = self.compute_reach(target)
            self.get_logger().info(f'  Block at ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}), '
                                   f'reach={reach:.3f}m')

            if reach <= self.MAX_REACH:
                block_pos = target
                self.get_logger().info('  Block is within reach!')
                break

            # Drive closer
            moved = self.move_base_closer(target)
            if not moved:
                block_pos = target
                break

        if block_pos is None:
            self.get_logger().error('Could not get close enough to block after max attempts!')
            return

        bx, by, bz = block_pos[0], block_pos[1], block_pos[2]
        self.get_logger().info(f'  Final target: ({bx:.3f}, {by:.3f}, {bz:.3f})')

        # ── Step 2: Open gripper ──
        self.get_logger().info('-' * 40)
        self.get_logger().info('Step 2: Opening right gripper')
        self.send_gripper(0.0)
        self.spin_for(1.0)
        self.get_logger().info('  Gripper open.')

        # ── Step 3: Move arm above the block (approach) ──
        self.get_logger().info('-' * 40)
        self.get_logger().info('Step 3: Moving arm above block (approach)')
        approach_pose = self.create_pose(bx, by, bz + 0.10, qw, qx, qy, qz)
        success = self.call_planner('right', approach_pose, use_orientation=True, duration=2.0)
        if not success:
            self.get_logger().error('Approach motion failed!')
            return
        self.spin_for(2.5)

        # ── Step 4: Descend to block grasp height ──
        self.get_logger().info('-' * 40)
        self.get_logger().info('Step 4: Descending to grasp height')
        grasp_pose = self.create_pose(bx, by, bz + 0.02, qw, qx, qy, qz)
        success = self.call_planner('right', grasp_pose, use_orientation=True, duration=2.0)
        if not success:
            self.get_logger().error('Descend motion failed!')
            return
        self.spin_for(2.5)

        # ── Step 5: Close gripper to grasp ──
        self.get_logger().info('-' * 40)
        self.get_logger().info('Step 5: Closing gripper to grasp block')
        self.send_gripper(1.0)
        self.spin_for(2.0)
        self.get_logger().info('  Gripper closed.')

        # ── Step 6: Lift the block ──
        self.get_logger().info('-' * 40)
        self.get_logger().info('Step 6: Lifting block')
        lift_pose = self.create_pose(bx, by, bz + 0.15, qw, qx, qy, qz)
        success = self.call_planner('right', lift_pose, use_orientation=True, duration=2.0)
        if not success:
            self.get_logger().error('Lift motion failed!')
            return
        self.spin_for(2.5)

        # ── Done ──
        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Block pick-up complete!')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = PickUpBlock()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
