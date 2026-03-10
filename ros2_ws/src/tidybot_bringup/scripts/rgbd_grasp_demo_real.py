#!/usr/bin/env python3
"""
RGB-D Grasp Demo for Real Hardware

Adapted from rgbd_grasp_demo.py for real robot operation. Key differences:
  - Depth-to-RGB alignment using align_depth_to_rgb utility (real D435 lenses
    have different intrinsics and a physical offset between depth/RGB sensors)
  - Sleep pose instead of home (all-zeros); smooth cosine-interpolated return
  - Joint state tracking for safe interpolated motions
  - Safety prompts (input()) before executing motions
  - Conservative durations (3.0s default)

Usage:
    # Terminal 1: Start real hardware with motion planner
    ros2 launch tidybot_bringup real.launch.py use_planner:=true

    # Terminal 2: Run this demo (defaults to red, right arm)
    ros2 run tidybot_bringup rgbd_grasp_demo_real.py

    # Or target a specific color/arm:
    ros2 run tidybot_bringup rgbd_grasp_demo_real.py --ros-args -p color:=green -p arm:=left
"""

import sys
import time
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, ColorRGBA
from visualization_msgs.msg import Marker
from tidybot_msgs.msg import ArmCommand
from tidybot_msgs.srv import PlanToTarget
from interbotix_xs_msgs.msg import JointGroupCommand

from tidybot_perception.rgbd_object_detector import RGBDObjectDetector

# Import depth alignment utility (works from both source and installed locations)
_script_dir = Path(__file__).resolve().parent
for _p in [_script_dir / 'utilities', _script_dir.parent / 'utilities']:
    if _p.is_dir():
        sys.path.insert(0, str(_p))
        break
from align_depth_to_rgb import align_depth  # noqa: E402


# Fingers-down orientation (wxyz) for top-down grasp in base_link
ORIENT_FINGERS_DOWN = (0.5, 0.5, 0.5, -0.5)

# Grasp geometry
PRE_GRASP_Z_OFFSET = 0.10   # 10 cm above grasp for approach
GRASP_Z_OFFSET = 0.00       # offset from detected surface
LIFT_HEIGHT = 0.15           # lift 15 cm after grasping

# ── Verification configuration ──────────────────────────────────────────────
MAX_GRASP_RETRIES = 3

# Arm pose that holds the grasped object in front of the camera.
# [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]
VERIFY_ARM_POSE = [0.0, 0.4, -0.3, 0.0, 0.5, 0.0]
VERIFY_ARM_DURATION = 2.5  # seconds to reach the pose

# Camera pan/tilt to look at the gripper region during verification.
VERIFY_CAMERA_PAN = 0.0
VERIFY_CAMERA_TILT = 0.3
VERIFY_CAMERA_SETTLE_TIME = 1.5

# Minimum blob area (pixels) to accept as "object in gripper."
VERIFY_MIN_BLOB_AREA = 100

# Sleep pose for arms (safe tucked position)
SLEEP_POSE = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]


class RGBDGraspDemoReal(Node):
    """Self-contained node for detect → grasp → verify → place on real hardware."""

    def __init__(self):
        super().__init__('rgbd_grasp_demo_real')

        # Parameters
        self.declare_parameter('color', 'red')
        self.declare_parameter('arm', 'right')
        self.declare_parameter('camera_tilt', 0.5)
        self.declare_parameter('duration', 3.0)  # conservative for real
        self.declare_parameter('max_retries', MAX_GRASP_RETRIES)

        self.target_color = self.get_parameter('color').value
        self.arm_name = self.get_parameter('arm').value
        self.camera_tilt = self.get_parameter('camera_tilt').value
        self.move_duration = self.get_parameter('duration').value
        self.max_retries = self.get_parameter('max_retries').value

        # Perception (subscribes to camera topics + sets up TF2)
        self.detector = RGBDObjectDetector(self)

        # Publishers
        self.pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, f'/{self.arm_name}_gripper/cmd', 10)

        # Arm command publisher (for verify pose via smooth interpolation)
        self.arm_pub = self.create_publisher(
            ArmCommand, f'/{self.arm_name}_arm/cmd', 10)

        # JointGroupCommand publisher (for smooth return to sleep)
        self.arm_joint_group_pub = self.create_publisher(
            JointGroupCommand, f'/{self.arm_name}_arm/commands/joint_group', 10)

        # Debug marker publisher (visible in RViz)
        self.marker_pub = self.create_publisher(Marker, '/grasp_debug_markers', 10)
        self.marker_id = 0

        # Service client
        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')

        # Joint state tracking (needed for smooth interpolation to sleep)
        self.joint_states_received = False
        self.current_joint_positions = {}
        self.create_subscription(JointState, '/joint_states', self._js_callback, 10)

    # ── Joint state tracking ─────────────────────────────────────────

    def _js_callback(self, msg):
        self.joint_states_received = True
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    def get_arm_positions(self):
        """Get current joint positions for the active arm."""
        joint_names = [
            f'{self.arm_name}_waist', f'{self.arm_name}_shoulder',
            f'{self.arm_name}_elbow', f'{self.arm_name}_forearm_roll',
            f'{self.arm_name}_wrist_angle', f'{self.arm_name}_wrist_rotate',
        ]
        return np.array([
            self.current_joint_positions.get(jname, 0.0) for jname in joint_names
        ])

    # ── Utilities ───────────────────────────────────────────────────

    def spin_for(self, seconds):
        """Spin to receive messages for the given duration."""
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def set_camera(self, pan=0.0, tilt=0.65):
        """Publish camera pan/tilt and wait for it to settle."""
        msg = Float64MultiArray()
        msg.data = [pan, tilt]
        for _ in range(5):
            self.pan_tilt_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.spin_for(1.5)

    def set_gripper(self, position):
        """Set gripper (0.0 = open, 1.0 = closed)."""
        msg = Float64MultiArray()
        msg.data = [position]
        for _ in range(10):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(0.5)

    def go_to_sleep_pose(self, max_joint_speed=0.5):
        """Send arm to sleep pose using smooth cosine-interpolated trajectory.

        Args:
            max_joint_speed: Maximum joint velocity in rad/s (default 0.5 rad/s ~ 30 deg/s)
        """
        # Spin to get latest joint states
        rclpy.spin_once(self, timeout_sec=0.1)

        current = self.get_arm_positions()
        target = np.array(SLEEP_POSE)

        # Calculate duration based on max joint difference
        max_diff = np.max(np.abs(target - current))
        duration = max(max_diff / max_joint_speed, 1.0)

        self.get_logger().info(
            f'Moving {self.arm_name} arm to sleep pose over {duration:.1f}s '
            f'(max joint diff: {max_diff:.2f} rad)')

        # Cosine interpolation for smooth acceleration/deceleration
        rate_hz = 50.0
        dt = 1.0 / rate_hz
        num_steps = max(int(duration * rate_hz), 1)

        for i in range(num_steps + 1):
            t = i / num_steps
            alpha = 0.5 * (1 - np.cos(np.pi * t))
            q = current + alpha * (target - current)

            cmd = JointGroupCommand()
            cmd.name = f'{self.arm_name}_arm'
            cmd.cmd = q.tolist()
            self.arm_joint_group_pub.publish(cmd)

            if i < num_steps:
                time.sleep(dt)

    def publish_marker(self, x, y, z, r, g, b, label='', scale=0.03):
        """Publish a sphere marker in base_link for RViz debugging."""
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'grasp_debug'
        m.id = self.marker_id
        self.marker_id += 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.pose.orientation.w = 1.0
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        m.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=0.8)
        m.lifetime.sec = 60
        if label:
            m.text = label
        self.marker_pub.publish(m)
        self.get_logger().info(
            f'  Published marker "{label}" at ({x:.3f}, {y:.3f}, {z:.3f})')

    def plan_and_execute(self, pose, label, use_orientation=True):
        """Call /plan_to_target synchronously. Returns True on success."""
        req = PlanToTarget.Request()
        req.arm_name = self.arm_name
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = True
        req.duration = self.move_duration
        req.max_condition_number = 500.0

        self.get_logger().info(
            f'  [{label}] pos=({pose.position.x:.3f}, {pose.position.y:.3f}, '
            f'{pose.position.z:.3f})')

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

        if not future.done() or future.exception():
            self.get_logger().error(f'  [{label}] service call failed')
            return False

        result = future.result()
        if result.success:
            self.get_logger().info(
                f'  [{label}] OK  pos_err={result.position_error:.4f}m')
            if result.executed:
                time.sleep(self.move_duration + 0.5)
            return True
        else:
            self.get_logger().warn(f'  [{label}] FAILED: {result.message}')
            return False

    # ── Pose helpers ────────────────────────────────────────────────

    @staticmethod
    def make_pose(x, y, z, qw, qx, qy, qz):
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = float(qw)
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        return pose

    def grasp_pose_at(self, x, y, z):
        """Fingers-down pose at the given base_link position."""
        qw, qx, qy, qz = ORIENT_FINGERS_DOWN
        return self.make_pose(x, y, z, qw, qx, qy, qz)

    def set_arm_joints(self, joint_positions, duration):
        """Publish an ArmCommand to the active arm and wait."""
        msg = ArmCommand()
        msg.joint_positions = list(joint_positions)
        msg.duration = float(duration)
        self.arm_pub.publish(msg)
        time.sleep(duration + 0.5)

    # ── Depth alignment ──────────────────────────────────────────────

    def align_depth_image(self):
        """Align the latest depth image to the RGB image in-place.

        On real hardware the D435 depth and RGB sensors have different
        intrinsics and a physical offset. This uses the align_depth
        utility to warp the depth image into the RGB camera frame so
        pixel coordinates match between the two.
        """
        if self.detector.latest_depth is None or self.detector.latest_rgb is None:
            self.get_logger().warn('Cannot align depth — missing RGB or depth image')
            return

        aligned = align_depth(self.detector.latest_depth, self.detector.latest_rgb)
        self.detector.latest_depth = aligned
        self.get_logger().info('  Depth image aligned to RGB frame')

    # ── Post-grasp verification ─────────────────────────────────────

    def verify_grasp(self) -> bool:
        """
        Present the gripper to the camera and check whether the target
        color is visible (i.e. the object is in the gripper).

        Returns True if the object is detected, False otherwise.
        """
        self.get_logger().info('  [verify] Moving arm to present pose...')
        self.set_arm_joints(VERIFY_ARM_POSE, VERIFY_ARM_DURATION)

        self.get_logger().info('  [verify] Aiming camera at gripper...')
        self.set_camera(pan=VERIFY_CAMERA_PAN, tilt=VERIFY_CAMERA_TILT)
        self.spin_for(VERIFY_CAMERA_SETTLE_TIME)

        self.get_logger().info(
            f'  [verify] Checking for "{self.target_color}" in camera view...')
        centroid = self.detector.detect_color(
            self.target_color, min_area=VERIFY_MIN_BLOB_AREA)

        if centroid is not None:
            u, v = centroid
            self.get_logger().info(
                f'  [verify] Object detected at pixel ({u}, {v}) — grasp confirmed')
            return True
        else:
            self.get_logger().warn(
                f'  [verify] Target color "{self.target_color}" not found — grasp failed')
            return False

    # ── Main pipeline ──────────────────────────────────────────────

    def _detect_object(self):
        """Point camera at table, align depth, and localize the target object.
        Returns (ox, oy, oz) in base_link or None."""
        self.get_logger().info('Setting camera tilt')
        self.set_camera(pan=0.0, tilt=self.camera_tilt)
        self.spin_for(2.0)

        # Align depth to RGB before detection (real hardware only)
        self.align_depth_image()

        self.get_logger().info(f'Detecting {self.target_color} object')
        result = self.detector.detect_and_localize(self.target_color)
        if result is None:
            self.get_logger().error(
                f'Could not find {self.target_color} object in camera view!')
            return None
        (u, v), base_pt = result
        ox, oy, oz = base_pt.point.x, base_pt.point.y, base_pt.point.z
        self.get_logger().info(f'  Detected at pixel ({u}, {v})')
        self.get_logger().info(
            f'  Object in base_link: ({ox:.3f}, {oy:.3f}, {oz:.3f})')
        return (ox, oy, oz)

    def _execute_grasp(self, ox, oy, oz):
        """Run the pre-grasp → grasp → close gripper sequence.
        Returns True if all motions succeeded."""
        grasp_z = oz + GRASP_Z_OFFSET
        pre_grasp_z = grasp_z + PRE_GRASP_Z_OFFSET

        self.publish_marker(ox, oy, oz, 0.0, 1.0, 0.0,
                            label='detected_surface', scale=0.02)
        self.publish_marker(ox, oy, grasp_z, 1.0, 0.0, 0.0,
                            label='grasp_target', scale=0.02)
        self.publish_marker(ox, oy, pre_grasp_z, 0.0, 0.0, 1.0,
                            label='pre_grasp', scale=0.02)

        self.get_logger().info('Opening gripper')
        self.set_gripper(0.0)

        self.get_logger().info('Moving to pre-grasp')
        pre_grasp = self.grasp_pose_at(ox, oy, pre_grasp_z)
        if not self.plan_and_execute(pre_grasp, 'pre-grasp'):
            self.get_logger().error('Pre-grasp motion failed')
            return False

        self.get_logger().info('Descending to grasp')
        grasp = self.grasp_pose_at(ox, oy, grasp_z)
        if not self.plan_and_execute(grasp, 'grasp'):
            self.get_logger().error('Grasp motion failed')
            return False

        self.get_logger().info('Closing gripper')
        self.set_gripper(1.0)
        time.sleep(1.0)
        return True

    def run(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info('RGB-D Grasp Demo — REAL HARDWARE')
        self.get_logger().info(f'  Target color : {self.target_color}')
        self.get_logger().info(f'  Arm          : {self.arm_name}')
        self.get_logger().info(f'  Camera tilt  : {self.camera_tilt}')
        self.get_logger().info(f'  Duration     : {self.move_duration}s')
        self.get_logger().info(f'  Max retries  : {self.max_retries}')
        self.get_logger().info('=' * 50)

        # Wait for service with timeout
        self.get_logger().info('Waiting for /plan_to_target service...')
        if not self.plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                'Service not available! Make sure motion_planner_real_node is running.')
            self.get_logger().error(
                'Launch with: ros2 launch tidybot_bringup real.launch.py use_planner:=true')
            return

        self.get_logger().info('Service available.')

        # Wait for joint states
        self.get_logger().info('Waiting for joint states...')
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_states_received:
                break
        if not self.joint_states_received:
            self.get_logger().warn('No joint states received — proceeding anyway')

        # Wait for camera images
        self.get_logger().info('Waiting for camera images...')
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.detector.latest_rgb is not None and self.detector.latest_depth is not None:
                break
        if self.detector.latest_rgb is None or self.detector.latest_depth is None:
            self.get_logger().error('Camera images not received — aborting')
            return

        self.get_logger().info('Camera images received.')
        self.get_logger().info('')

        # Safety prompt
        input('\n  Press Enter to begin grasp demo (Ctrl+C to abort)...\n')

        # ── Grasp + verify loop ─────────────────────────────────────
        grasp_confirmed = False
        for attempt in range(1, self.max_retries + 1):
            self.get_logger().info('')
            self.get_logger().info(f'=== Grasp attempt {attempt}/{self.max_retries} ===')

            # Detect object on the table (with depth alignment)
            detection = self._detect_object()
            if detection is None:
                self.get_logger().error('Object detection failed — aborting')
                return
            ox, oy, oz = detection

            # Safety prompt before execution
            input('\n  Object detected. Press Enter to execute grasp (Ctrl+C to abort)...\n')

            # Execute grasp (pre-grasp → descend → close gripper)
            if not self._execute_grasp(ox, oy, oz):
                self.get_logger().error('Grasp execution failed — aborting')
                return

            # Verify: present to camera and check for target color
            self.get_logger().info('Verifying grasp...')
            if self.verify_grasp():
                grasp_confirmed = True
                break

            # Verification failed — release and retry
            self.get_logger().warn(
                f'Grasp verification failed (attempt {attempt}/{self.max_retries})')
            self.get_logger().info('Opening gripper to release...')
            self.set_gripper(0.0)
            time.sleep(0.5)
            self.get_logger().info('Returning arm to sleep before retry...')
            self.go_to_sleep_pose()

        if not grasp_confirmed:
            self.get_logger().error(
                f'Grasp failed after {self.max_retries} attempts — giving up')
            self.go_to_sleep_pose()
            return

        # ── Grasp verified — continue with post-grasp actions ───────

        self.get_logger().info('')
        self.get_logger().info('Grasp confirmed — proceeding to lift')

        # Lift
        grasp_z = oz + GRASP_Z_OFFSET
        self.get_logger().info('Lifting object')
        lift = self.grasp_pose_at(ox, oy, grasp_z + LIFT_HEIGHT)
        if not self.plan_and_execute(lift, 'lift'):
            self.get_logger().error('Lift motion failed')
            return

        # Place back down
        self.get_logger().info('Placing object back down')
        place = self.grasp_pose_at(ox, oy, grasp_z)
        if not self.plan_and_execute(place, 'place-down'):
            self.get_logger().error('Place motion failed')
            return

        # Release
        self.get_logger().info('Releasing object')
        self.set_gripper(0.0)
        time.sleep(0.5)

        # Retract
        pre_grasp_z = grasp_z + PRE_GRASP_Z_OFFSET
        self.get_logger().info('Retracting arm')
        retract = self.grasp_pose_at(ox, oy, pre_grasp_z)
        self.plan_and_execute(retract, 'retract')

        # Return to sleep
        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Returning arm to sleep pose...')
        self.get_logger().info('=' * 50)
        self.go_to_sleep_pose()

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Grasp demo complete!')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = RGBDGraspDemoReal()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
