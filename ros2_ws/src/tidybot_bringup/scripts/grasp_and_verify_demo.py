#!/usr/bin/env python3
"""
Grasp + Verification Demo

Combines top-down grasping with post-grasp YOLO verification.
Picks up a target object, moves to a verify pose, and uses YOLO detection
to confirm the object is held in the gripper. Retries on failure.

Works on both simulation and real hardware via the `sim` parameter.

Requires:
    Simulation:
        - ros2 launch tidybot_bringup sim.launch.py scene:=scene_fruit_grasp.xml
        - ros2 run object_classification classifier

    Real Hardware:
        - ros2 launch tidybot_bringup real.launch.py use_planner:=true
        - ros2 run object_classification classifier

Usage:
    # Simulation (default):
    ros2 run tidybot_bringup grasp_and_verify_demo.py

    # Real hardware:
    ros2 run tidybot_bringup grasp_and_verify_demo.py --ros-args -p sim:=false

    # Custom parameters:
    ros2 run tidybot_bringup grasp_and_verify_demo.py --ros-args \
        -p target:=apple -p arm:=right -p max_retries:=2 -p sim:=false
"""

import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from std_msgs.msg import Float64MultiArray
from tidybot_msgs.msg import ArmCommand

# Add scripts directory so we can import TopdownGraspDemo
_SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SCRIPT_DIR))

from topdown_grasp_demo import TopdownGraspDemo  # noqa: E402

# ── Verification constants ──────────────────────────────────────────────
# Sim: arm lower + camera looks up (tuned via test_grasp_validation.py)
VERIFY_ARM_POSE_SIM = [0.3, -0.2, 0.7, 0.0, 0.2, 0.0]
VERIFY_CAMERA_PAN_SIM = 0.0
VERIFY_CAMERA_TILT_SIM = -0.6

# Real: arm presents upward + camera looks down at gripper
VERIFY_ARM_POSE_REAL = [0.0, 0.4, -0.3, 0.0, 0.5, 0.0]
VERIFY_CAMERA_PAN_REAL = 0.0
VERIFY_CAMERA_TILT_REAL = 0.3

# Shared
VERIFY_ARM_DURATION = 2.5
VERIFY_SETTLE_TIME = 2.0
VERIFY_DETECT_ATTEMPTS = 10

# Default camera tilt for looking down at the table (reset between retries)
TABLE_CAMERA_TILT = 0.5


class GraspAndVerifyDemo(TopdownGraspDemo):
    """Top-down grasp with post-grasp YOLO verification and retry logic."""

    def __init__(self):
        super().__init__()

        # Additional parameters
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('v_threshold', 320.0)
        self.max_retries = self.get_parameter('max_retries').value
        self.v_threshold = self.get_parameter('v_threshold').value

        # Select verify constants based on sim/real
        if self.is_sim:
            self.verify_arm_pose = VERIFY_ARM_POSE_SIM
            self.verify_camera_pan = VERIFY_CAMERA_PAN_SIM
            self.verify_camera_tilt = VERIFY_CAMERA_TILT_SIM
        else:
            self.verify_arm_pose = VERIFY_ARM_POSE_REAL
            self.verify_camera_pan = VERIFY_CAMERA_PAN_REAL
            self.verify_camera_tilt = VERIFY_CAMERA_TILT_REAL

        # Pan/tilt publisher (TopdownGraspDemo doesn't have one)
        self.pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10)

    # -- Camera control ------------------------------------------------

    def set_camera(self, pan, tilt):
        """Publish camera pan/tilt and wait for it to settle."""
        msg = Float64MultiArray()
        msg.data = [pan, tilt]
        for _ in range(5):
            self.pan_tilt_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.spin_for(1.5)

    # -- Arm joint-space command (sim/real aware) ----------------------

    def set_arm_joints(self, positions, duration):
        """Send arm to specified joint positions. Handles sim vs real."""
        if self.is_sim:
            msg = ArmCommand()
            msg.joint_positions = list(positions)
            msg.duration = float(duration)
            self.arm_pub.publish(msg)
            time.sleep(duration + 0.5)
        else:
            # Real hardware: smooth cosine-interpolated trajectory
            rclpy.spin_once(self, timeout_sec=0.1)
            current = self.get_arm_positions()
            target = np.array(positions)

            rate_hz = 50.0
            dt = 1.0 / rate_hz
            num_steps = max(int(duration * rate_hz), 1)

            for i in range(num_steps + 1):
                t = i / num_steps
                alpha = 0.5 * (1 - np.cos(np.pi * t))
                q = current + alpha * (target - current)

                cmd = self._JointGroupCommand()
                cmd.name = f'{self.arm_name}_arm'
                cmd.cmd = q.tolist()
                self.arm_joint_group_pub.publish(cmd)

                if i < num_steps:
                    time.sleep(dt)

    # -- Grasp verification -------------------------------------------

    def verify_grasp(self):
        """Move to verify pose, aim camera, check YOLO for held object.

        Returns True if object detected in upper portion of frame
        (indicating it's held by the gripper), False otherwise.
        """
        self.get_logger().info('--- Grasp Verification ---')

        self.get_logger().info(
            f'  Moving arm to verify pose {self.verify_arm_pose}...')
        self.set_arm_joints(self.verify_arm_pose, VERIFY_ARM_DURATION)

        self.get_logger().info(
            f'  Pointing camera (pan={self.verify_camera_pan}, '
            f'tilt={self.verify_camera_tilt})...')
        self.set_camera(self.verify_camera_pan, self.verify_camera_tilt)

        self.get_logger().info(
            f'  Waiting {VERIFY_SETTLE_TIME}s for settle...')
        self.spin_for(VERIFY_SETTLE_TIME)

        # Try multiple detection attempts
        det = None
        for attempt in range(VERIFY_DETECT_ATTEMPTS):
            det = self.detect(self.target_object)
            if det is not None:
                break
            self.spin_for(0.3)

        if det is None:
            self.get_logger().warn(
                f'  No "{self.target_object}" detected in camera view')
            self.get_logger().warn('  VERIFICATION: FAIL -- object not visible')
            return False

        u = det.bbox.center.position.x
        v = det.bbox.center.position.y
        w = det.bbox.size_x
        h = det.bbox.size_y
        conf = det.results[0].hypothesis.score if det.results else 0.0

        self.get_logger().info(
            f'  Detection: center=({u:.0f}, {v:.0f}), '
            f'size={w:.0f}x{h:.0f}, conf={conf:.2f}')

        if v < self.v_threshold:
            self.get_logger().info(
                f'  VERIFICATION: PASS -- v={v:.0f} < {self.v_threshold:.0f} '
                f'(object in upper frame = held)')
            return True
        else:
            self.get_logger().warn(
                f'  VERIFICATION: FAIL -- v={v:.0f} >= {self.v_threshold:.0f} '
                f'(object in lower frame = dropped)')
            return False

    # -- Compound pipeline --------------------------------------------

    def run_with_verification(self):
        """Execute grasp + verification loop with retries.

        Returns True if object is verified as held, False after all retries.
        """
        mode = 'SIMULATION' if self.is_sim else 'REAL HARDWARE'
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Grasp + Verify Demo -- {mode}')
        self.get_logger().info(f'  Target     : {self.target_object}')
        self.get_logger().info(f'  Arm        : {self.arm_name}')
        self.get_logger().info(f'  Max retries: {self.max_retries}')
        self.get_logger().info(f'  V threshold: {self.v_threshold}')
        self.get_logger().info('=' * 60)

        for attempt in range(1, self.max_retries + 1):
            self.get_logger().info('')
            self.get_logger().info(
                f'=== Attempt {attempt}/{self.max_retries} ===')

            # Step 1: Execute grasp (inherited from TopdownGraspDemo)
            grasp_ok = self.run()
            if not grasp_ok:
                self.get_logger().error(
                    f'Grasp failed on attempt {attempt}')
                if attempt < self.max_retries:
                    self.get_logger().info('Resetting for retry...')
                    self.go_home()
                    # Point camera back down at table for next detection
                    self.set_camera(0.0, TABLE_CAMERA_TILT)
                    self.spin_for(1.0)
                continue

            # Step 2: Verify grasp
            self.get_logger().info('')
            verified = self.verify_grasp()

            if verified:
                self.get_logger().info('')
                self.get_logger().info('=' * 60)
                self.get_logger().info(
                    f'GRASP VERIFIED on attempt {attempt}!')
                self.get_logger().info('=' * 60)
                return True

            # Verification failed -- release and retry
            self.get_logger().warn(
                f'Verification failed (attempt {attempt}/{self.max_retries})')
            if attempt < self.max_retries:
                self.get_logger().info(
                    'Releasing object and returning home for retry...')
                self.release()
                # Point camera back down at table for next detection
                self.set_camera(0.0, TABLE_CAMERA_TILT)
                self.spin_for(1.0)
            else:
                self.get_logger().info('Releasing object...')
                self.release()

        # All retries exhausted
        self.get_logger().error('')
        self.get_logger().error('=' * 60)
        self.get_logger().error(
            f'FAILED after {self.max_retries} attempts')
        self.get_logger().error('=' * 60)
        return False


def main(args=None):
    rclpy.init(args=args)
    node = GraspAndVerifyDemo()
    try:
        success = node.run_with_verification()
        if success:
            node.get_logger().info(
                'Holding for 3s then releasing (standalone mode)')
            time.sleep(3.0)
            node.release()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
