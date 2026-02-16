#!/usr/bin/env python3
"""
Arm Command Wrapper Node for TidyBot2.

Provides simulation-compatible arm and pan-tilt interface for real hardware
with velocity limiting and joint limit enforcement to protect the robot.

Translates from:
    /right_arm/joint_cmd (Float64MultiArray) - 6 joint positions
    /left_arm/joint_cmd (Float64MultiArray) - 6 joint positions
    /camera/pan_tilt_cmd (Float64MultiArray) - [pan, tilt] in radians
To Interbotix SDK:
    /right_arm/commands/joint_group (JointGroupCommand)
    /left_arm/commands/joint_group (JointGroupCommand)

Safety features:
    - Velocity limiting: commands are rate-limited so the arm cannot move
      faster than max_arm_vel (rad/s). Large jumps are smoothed into gradual
      motion over multiple control cycles.
    - Joint limit clamping: target positions are clamped to safe joint limits
      before being sent to the servos.
    - Control loop: a fixed-rate timer (default 50 Hz) interpolates toward
      targets instead of forwarding commands directly.

Pan-tilt state is published directly by xs_sdk to /camera/pan_tilt_state
(configured in right_arm_pantilt.yaml via group_joint_state_publishers).

Parameters:
    - control_rate (double, default 50.0): Control loop rate in Hz
    - max_arm_vel (double, default 1.5): Max arm joint velocity in rad/s
    - max_pan_tilt_vel (double, default 1.0): Max pan-tilt velocity in rad/s
"""

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from interbotix_xs_msgs.msg import JointGroupCommand


class ArmWrapperNode(Node):
    """Wrapper node with velocity limiting for safe sim-to-real arm control."""

    # Joint limits for WX250s 6-DOF (radians)
    ARM_JOINT_LIMITS = np.array([
        [-3.14159,  3.14159],   # waist
        [-1.88496,  1.98968],   # shoulder
        [-2.14675,  1.60570],   # elbow
        [-3.14159,  3.14159],   # forearm_roll
        [-1.74533,  2.14675],   # wrist_angle
        [-3.14159,  3.14159],   # wrist_rotate
    ])

    # Pan-tilt limits (radians) — full Dynamixel range
    PAN_TILT_LIMITS = np.array([
        [-3.14159,  3.14159],   # pan
        [-1.57080,  1.57080],   # tilt
    ])

    # Joint names for reading /joint_states
    RIGHT_ARM_JOINTS = [
        'right_waist', 'right_shoulder', 'right_elbow',
        'right_forearm_roll', 'right_wrist_angle', 'right_wrist_rotate',
    ]
    LEFT_ARM_JOINTS = [
        'left_waist', 'left_shoulder', 'left_elbow',
        'left_forearm_roll', 'left_wrist_angle', 'left_wrist_rotate',
    ]
    PAN_TILT_JOINTS = ['camera_pan', 'camera_tilt']

    def __init__(self):
        super().__init__('arm_wrapper')

        # Parameters
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('max_arm_vel', 1.0)       # rad/s
        self.declare_parameter('max_pan_tilt_vel', 1.0)   # rad/s

        self.control_rate = self.get_parameter('control_rate').value
        self.max_arm_vel = self.get_parameter('max_arm_vel').value
        self.max_pan_tilt_vel = self.get_parameter('max_pan_tilt_vel').value
        self.dt = 1.0 / self.control_rate

        # Max step per control cycle
        self.arm_max_step = self.max_arm_vel * self.dt
        self.pt_max_step = self.max_pan_tilt_vel * self.dt

        # --- State tracking ---
        # Commanded positions (what we last sent to the servos)
        self.right_cmd = None   # np.ndarray(6) or None (uninitialized)
        self.left_cmd = None
        self.pt_cmd = None      # np.ndarray(2) or None

        # Target positions (what the user wants)
        self.right_target = None
        self.left_target = None
        self.pt_target = None

        # Whether we've synced initial positions from joint states
        self.right_initialized = False
        self.left_initialized = False
        self.pt_initialized = False

        # --- Publishers to Interbotix SDK ---
        self.right_arm_pub = self.create_publisher(
            JointGroupCommand, '/right_arm/commands/joint_group', 10
        )
        self.left_arm_pub = self.create_publisher(
            JointGroupCommand, '/left_arm/commands/joint_group', 10
        )

        # --- Subscribers: sim-compatible command topics ---
        self.right_arm_sub = self.create_subscription(
            Float64MultiArray, '/right_arm/joint_cmd',
            lambda msg: self._arm_target_callback(msg, 'right'), 10
        )
        self.left_arm_sub = self.create_subscription(
            Float64MultiArray, '/left_arm/joint_cmd',
            lambda msg: self._arm_target_callback(msg, 'left'), 10
        )
        self.pan_tilt_cmd_sub = self.create_subscription(
            Float64MultiArray, '/camera/pan_tilt_cmd',
            self._pan_tilt_target_callback, 10
        )

        # --- Subscriber: joint states for initialization ---
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_callback, 10
        )
        self.pt_state_sub = self.create_subscription(
            JointState, '/camera/pan_tilt_state',
            self._pan_tilt_state_callback, 10
        )

        # --- Control timer ---
        self.create_timer(self.dt, self._control_loop)

        self.get_logger().info('Arm wrapper node started (velocity-limited)')
        self.get_logger().info(f'  Control rate: {self.control_rate} Hz')
        self.get_logger().info(f'  Max arm velocity: {self.max_arm_vel} rad/s')
        self.get_logger().info(f'  Max pan-tilt velocity: {self.max_pan_tilt_vel} rad/s')

    # ------------------------------------------------------------------ #
    #  Joint state callbacks — initialize commanded positions from actual
    # ------------------------------------------------------------------ #

    def _joint_state_callback(self, msg: JointState):
        """Initialize commanded positions from actual joint states."""
        if not self.right_initialized:
            pos = self._extract_positions(msg, self.RIGHT_ARM_JOINTS)
            if pos is not None:
                self.right_cmd = pos.copy()
                self.right_initialized = True
                self.get_logger().info('Right arm initialized from joint states')

        if not self.left_initialized:
            pos = self._extract_positions(msg, self.LEFT_ARM_JOINTS)
            if pos is not None:
                self.left_cmd = pos.copy()
                self.left_initialized = True
                self.get_logger().info('Left arm initialized from joint states')

    def _pan_tilt_state_callback(self, msg: JointState):
        """Initialize pan-tilt commanded positions from actual state."""
        if not self.pt_initialized:
            pos = self._extract_positions(msg, self.PAN_TILT_JOINTS)
            if pos is not None:
                self.pt_cmd = pos.copy()
                self.pt_initialized = True
                self.get_logger().info('Pan-tilt initialized from joint states')

    @staticmethod
    def _extract_positions(msg: JointState, joint_names: list) -> np.ndarray | None:
        """Extract ordered positions for given joint names from a JointState msg."""
        positions = []
        for name in joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
            else:
                return None
        return np.array(positions)

    # ------------------------------------------------------------------ #
    #  Command target callbacks — store desired targets (no immediate send)
    # ------------------------------------------------------------------ #

    def _arm_target_callback(self, msg: Float64MultiArray, side: str):
        """Store arm target positions (will be velocity-limited in control loop)."""
        if len(msg.data) < 6:
            self.get_logger().warn(f'{side} arm: expected 6 joints, got {len(msg.data)}')
            return

        target = np.clip(
            np.array(msg.data[:6]),
            self.ARM_JOINT_LIMITS[:, 0],
            self.ARM_JOINT_LIMITS[:, 1],
        )

        if side == 'right':
            self.right_target = target
        else:
            self.left_target = target

    def _pan_tilt_target_callback(self, msg: Float64MultiArray):
        """Store pan-tilt target positions."""
        if len(msg.data) < 2:
            self.get_logger().warn(
                f'pan_tilt: expected 2 values [pan, tilt], got {len(msg.data)}'
            )
            return

        self.pt_target = np.clip(
            np.array(msg.data[:2]),
            self.PAN_TILT_LIMITS[:, 0],
            self.PAN_TILT_LIMITS[:, 1],
        )

    # ------------------------------------------------------------------ #
    #  Control loop — velocity-limited interpolation toward targets
    # ------------------------------------------------------------------ #

    def _control_loop(self):
        """Move commanded positions toward targets at bounded velocity."""
        # Right arm
        if self.right_target is not None and self.right_initialized:
            self.right_cmd = self._step_toward(
                self.right_cmd, self.right_target, self.arm_max_step
            )
            self._publish_arm_cmd(self.right_cmd, 'right')

        # Left arm
        if self.left_target is not None and self.left_initialized:
            self.left_cmd = self._step_toward(
                self.left_cmd, self.left_target, self.arm_max_step
            )
            self._publish_arm_cmd(self.left_cmd, 'left')

        # Pan-tilt
        if self.pt_target is not None and self.pt_initialized:
            self.pt_cmd = self._step_toward(
                self.pt_cmd, self.pt_target, self.pt_max_step
            )
            self._publish_pt_cmd(self.pt_cmd)

    @staticmethod
    def _step_toward(current: np.ndarray, target: np.ndarray, max_step: float) -> np.ndarray:
        """Move current toward target, clamping each joint's step to max_step."""
        diff = target - current
        clamped = np.clip(diff, -max_step, max_step)
        return current + clamped

    # ------------------------------------------------------------------ #
    #  Publishing helpers
    # ------------------------------------------------------------------ #

    def _publish_arm_cmd(self, positions: np.ndarray, side: str):
        """Publish arm joint group command to Interbotix SDK."""
        cmd = JointGroupCommand()
        cmd.name = f'{side}_arm'
        cmd.cmd = positions.tolist()
        if side == 'right':
            self.right_arm_pub.publish(cmd)
        else:
            self.left_arm_pub.publish(cmd)

    def _publish_pt_cmd(self, positions: np.ndarray):
        """Publish pan-tilt joint group command to Interbotix SDK."""
        cmd = JointGroupCommand()
        cmd.name = 'pan_tilt'
        cmd.cmd = positions.tolist()
        self.right_arm_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ArmWrapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
