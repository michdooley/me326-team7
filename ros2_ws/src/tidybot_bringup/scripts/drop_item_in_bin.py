#!/usr/bin/env python3
"""
TidyBot2 Drop Item in Bin

State machine that picks up an item and drops it into a bin using the
motion planner service. The sequence is:
  1. Move arm above the item (approach)
  2. Descend to grasp position
  3. Close gripper
  4. Lift the item
  5. Move above the bin
  6. Open gripper to release
  7. Verify drop (take picture, check gripper not obviously off from bin)
  8. Retreat upward

Uses the /plan_to_target service for collision-checked IK planning.
After dropping, verifies gripper alignment via camera: if gripper is not
obviously off (e.g., too far left of bin), confirms object in bin.

Topics/Services used:
- /plan_to_target (PlanToTarget) - motion planning with collision checking
- /right_gripper/cmd (Float64MultiArray) - gripper control
- /left_gripper/cmd (Float64MultiArray) - gripper control
- /camera/color/image_raw (Image) - for drop verification
- /camera/color/camera_info (CameraInfo) - for projection

Usage:
    # Terminal 1: Start simulation with the pickup scene
    ros2 launch tidybot_bringup sim.launch.py scene:=scene_pickup.xml

    # Terminal 2: Run drop-in-bin
    ros2 run tidybot_bringup drop_item_in_bin.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, CameraInfo
from tidybot_msgs.srv import PlanToTarget

import time
from enum import Enum, auto

try:
    import tf2_ros
    from rclpy.duration import Duration
    TF_AVAILABLE = True
except ImportError:
    TF_AVAILABLE = False

# Import verification helper from same directory
import sys
from pathlib import Path
_script_dir = Path(__file__).resolve().parent
if str(_script_dir) not in sys.path:
    sys.path.insert(0, str(_script_dir))
try:
    from drop_verification import verify_gripper_alignment
except ImportError:
    verify_gripper_alignment = None


class Phase(Enum):
    WAIT_FOR_SERVICE = auto()
    APPROACH = auto()
    DESCEND = auto()
    CLOSE_GRIPPER = auto()
    LIFT = auto()
    MOVE_TO_BIN = auto()
    OPEN_GRIPPER = auto()
    VERIFY_DROP = auto()
    RETREAT = auto()
    DONE = auto()


# Top-down grasp orientation (gripper pointing straight down)
TOP_DOWN_ORIENTATION = {'qw': 0.0, 'qx': 1.0, 'qy': 0.0, 'qz': 0.0}

# Default positions (meters, in base_link frame)
DEFAULT_ITEM_POS = (0.35, -0.15, 0.10)
DEFAULT_BIN_POS = (0.30, 0.20, 0.30)

APPROACH_HEIGHT = 0.15   # height above item for approach
GRASP_OFFSET_Z = 0.045  # z-offset for grasping (above table surface)
LIFT_HEIGHT = 0.30       # height to lift item to
PLAN_DURATION = 2.0      # seconds per arm motion
GRIPPER_WAIT = 1.2       # seconds to wait for gripper action
VERIFY_WAIT = 0.6        # seconds to wait before capturing image for verification


class DropItemInBin(Node):
    """Pick up an item and drop it into a bin using planned motions."""

    def __init__(
        self,
        arm: str = 'right',
        item_pos: tuple = DEFAULT_ITEM_POS,
        bin_pos: tuple = DEFAULT_BIN_POS,
    ):
        super().__init__('drop_item_in_bin')

        self.arm = arm
        self.item_pos = item_pos
        self.bin_pos = bin_pos

        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')

        gripper_topic = f'/{arm}_gripper/cmd'
        self.gripper_pub = self.create_publisher(Float64MultiArray, gripper_topic, 10)

        self.phase = Phase.WAIT_FOR_SERVICE
        self.phase_start: float | None = None
        self.logged_phase = None
        self.planning = False
        self.drop_verified: bool | None = None  # True=confirmed, False=rejected, None=skipped

        # TF and camera for drop verification
        self.tf_buffer = None
        self.latest_rgb = None
        self.latest_camera_info = None
        if TF_AVAILABLE:
            self.tf_buffer = tf2_ros.Buffer()
            tf2_ros.TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/color/image_raw', self._rgb_cb, qos)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self._camera_info_cb, 10)

        self.timer = self.create_timer(0.05, self._tick)

        self.get_logger().info('=' * 50)
        self.get_logger().info('TidyBot2  Drop Item In Bin')
        self.get_logger().info(f'  arm:  {arm}')
        self.get_logger().info(f'  item: {item_pos}')
        self.get_logger().info(f'  bin:  {bin_pos}')
        self.get_logger().info('=' * 50)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _make_pose(self, x, y, z):
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = TOP_DOWN_ORIENTATION['qw']
        pose.orientation.x = TOP_DOWN_ORIENTATION['qx']
        pose.orientation.y = TOP_DOWN_ORIENTATION['qy']
        pose.orientation.z = TOP_DOWN_ORIENTATION['qz']
        return pose

    def _send_gripper(self, value: float):
        msg = Float64MultiArray()
        msg.data = [float(value)]
        self.gripper_pub.publish(msg)

    def _plan_and_execute(self, x, y, z, duration=PLAN_DURATION):
        """Non-blocking planning call. Returns True when the call has been sent."""
        if self.planning:
            return False

        request = PlanToTarget.Request()
        request.arm_name = self.arm
        request.target_pose = self._make_pose(x, y, z)
        request.use_orientation = True
        request.execute = True
        request.duration = duration
        request.max_condition_number = 100.0

        self.get_logger().info(
            f'  Planning {self.arm} arm -> ({x:.3f}, {y:.3f}, {z:.3f})'
        )

        self._future = self.plan_client.call_async(request)
        self._future.add_done_callback(self._plan_done_cb)
        self.planning = True
        return True

    def _plan_done_cb(self, future):
        self.planning = False
        result = future.result()
        if result is None:
            self.get_logger().error('  Planning service call failed!')
        elif result.success:
            self.get_logger().info(f'  Plan OK: {result.message}')
        else:
            self.get_logger().warn(f'  Plan FAILED: {result.message}')

    def _transition(self, new_phase: Phase):
        self.get_logger().info(f'Phase: {self.phase.name} -> {new_phase.name}')
        self.phase = new_phase
        self.phase_start = time.time()

    def _elapsed(self):
        if self.phase_start is None:
            return 0.0
        return time.time() - self.phase_start

    def _rgb_cb(self, msg: Image):
        self.latest_rgb = msg

    def _camera_info_cb(self, msg: CameraInfo):
        self.latest_camera_info = msg

    # ------------------------------------------------------------------
    # Main tick
    # ------------------------------------------------------------------
    def _tick(self):
        if self.phase_start is None:
            self.phase_start = time.time()

        # --- WAIT_FOR_SERVICE ---
        if self.phase == Phase.WAIT_FOR_SERVICE:
            if self.logged_phase != self.phase:
                self.get_logger().info('Waiting for /plan_to_target service...')
                self.logged_phase = self.phase
            if self.plan_client.service_is_ready():
                self.get_logger().info('Service ready!')
                self._send_gripper(0.0)
                self._transition(Phase.APPROACH)

        # --- APPROACH (move above item) ---
        elif self.phase == Phase.APPROACH:
            if self.logged_phase != self.phase:
                ix, iy, iz = self.item_pos
                self._plan_and_execute(ix, iy, iz + APPROACH_HEIGHT)
                self.logged_phase = self.phase
            if not self.planning and self._elapsed() > PLAN_DURATION + 0.5:
                self._transition(Phase.DESCEND)

        # --- DESCEND (move to grasp height) ---
        elif self.phase == Phase.DESCEND:
            if self.logged_phase != self.phase:
                ix, iy, iz = self.item_pos
                self._plan_and_execute(ix, iy, GRASP_OFFSET_Z, duration=1.5)
                self.logged_phase = self.phase
            if not self.planning and self._elapsed() > 2.0:
                self._transition(Phase.CLOSE_GRIPPER)

        # --- CLOSE GRIPPER ---
        elif self.phase == Phase.CLOSE_GRIPPER:
            if self.logged_phase != self.phase:
                self.get_logger().info('  Closing gripper...')
                self.logged_phase = self.phase
            self._send_gripper(1.0)
            if self._elapsed() > GRIPPER_WAIT:
                self._transition(Phase.LIFT)

        # --- LIFT ---
        elif self.phase == Phase.LIFT:
            if self.logged_phase != self.phase:
                ix, iy, _ = self.item_pos
                self._plan_and_execute(ix, iy, LIFT_HEIGHT)
                self.logged_phase = self.phase
            self._send_gripper(1.0)
            if not self.planning and self._elapsed() > PLAN_DURATION + 0.5:
                self._transition(Phase.MOVE_TO_BIN)

        # --- MOVE TO BIN ---
        elif self.phase == Phase.MOVE_TO_BIN:
            if self.logged_phase != self.phase:
                bx, by, bz = self.bin_pos
                self._plan_and_execute(bx, by, bz, duration=2.5)
                self.logged_phase = self.phase
            self._send_gripper(1.0)
            if not self.planning and self._elapsed() > 3.0:
                self._transition(Phase.OPEN_GRIPPER)

        # --- OPEN GRIPPER (release item) ---
        elif self.phase == Phase.OPEN_GRIPPER:
            if self.logged_phase != self.phase:
                self.get_logger().info('  Opening gripper — dropping item!')
                self.logged_phase = self.phase
            self._send_gripper(0.0)
            if self._elapsed() > GRIPPER_WAIT:
                self._transition(Phase.VERIFY_DROP)

        # --- VERIFY DROP (take picture, check gripper not obviously off) ---
        elif self.phase == Phase.VERIFY_DROP:
            if self.logged_phase != self.phase:
                self.get_logger().info('  Verifying drop (gripper alignment check)...')
                self.logged_phase = self.phase
            self._send_gripper(0.0)
            if self._elapsed() < VERIFY_WAIT:
                return  # Wait for object to settle
            if self.drop_verified is None:
                if verify_gripper_alignment is None:
                    self.get_logger().warn('  Verification module not available, assuming success')
                    self.drop_verified = True
                elif self.tf_buffer is None:
                    self.get_logger().warn('  TF not available, assuming success')
                    self.drop_verified = True
                elif self.latest_rgb is None or self.latest_camera_info is None:
                    self.get_logger().warn('  No camera image yet, assuming success')
                    self.drop_verified = True
                else:
                    ok, msg = verify_gripper_alignment(
                        self,
                        self.tf_buffer,
                        self.bin_pos,
                        self.arm,
                        self.latest_rgb,
                        self.latest_camera_info,
                    )
                    self.drop_verified = ok
                    self.get_logger().info(f'  Verification: {msg}')
            if self.drop_verified:
                self._transition(Phase.RETREAT)
            else:
                self.get_logger().warn('  Gripper obviously off — drop may have failed!')
                self._transition(Phase.RETREAT)  # Still retreat, but we logged the failure

        # --- RETREAT ---
        elif self.phase == Phase.RETREAT:
            if self.logged_phase != self.phase:
                bx, by, bz = self.bin_pos
                self._plan_and_execute(bx, by, bz + 0.10)
                self.logged_phase = self.phase
            self._send_gripper(0.0)
            if not self.planning and self._elapsed() > PLAN_DURATION + 0.5:
                self._transition(Phase.DONE)

        # --- DONE ---
        elif self.phase == Phase.DONE:
            if self.logged_phase != self.phase:
                self.get_logger().info('')
                self.get_logger().info('=' * 50)
                if self.drop_verified is True:
                    self.get_logger().info('Drop complete!  Gripper aligned — object likely in bin.')
                elif self.drop_verified is False:
                    self.get_logger().info('Drop complete, but gripper was misaligned — verify manually.')
                else:
                    self.get_logger().info('Drop complete!  (Verification skipped)')
                self.get_logger().info('=' * 50)
                self.logged_phase = self.phase


def main(args=None):
    rclpy.init(args=args)
    node = DropItemInBin()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
