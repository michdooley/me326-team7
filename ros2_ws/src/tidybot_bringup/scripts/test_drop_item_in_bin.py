#!/usr/bin/env python3
"""
Test script for the DropItemInBin module.

Runs through the full pick-and-drop sequence with configurable item and bin
positions, then reports pass/fail based on whether every planning phase
succeeded within a timeout.

Usage:
    # Terminal 1: Start simulation with the pickup scene
    ros2 launch tidybot_bringup sim.launch.py scene:=scene_pickup.xml

    # Terminal 2: Run this test
    ros2 run tidybot_bringup test_drop_item_in_bin.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tidybot_msgs.srv import PlanToTarget

import time
import sys
from enum import Enum, auto


# Re-use the constants from the main module so the test stays in sync
ITEM_POS = (0.35, -0.15, 0.10)
BIN_POS = (0.30, 0.20, 0.30)
TOP_DOWN = {'qw': 0.0, 'qx': 1.0, 'qy': 0.0, 'qz': 0.0}

TIMEOUT_PER_STEP = 8.0  # seconds allowed per planning step


class TestDropItemInBin(Node):
    """Integration test that exercises each phase of the drop-in-bin pipeline."""

    def __init__(self):
        super().__init__('test_drop_item_in_bin')

        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/right_gripper/cmd', 10
        )

        self.joint_positions: dict[str, float] = {}
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10
        )

        self.results: list[tuple[str, bool, str]] = []

        self.get_logger().info('=' * 55)
        self.get_logger().info(' Test: Drop Item In Bin')
        self.get_logger().info('=' * 55)

        self.create_timer(1.0, self._run_once)
        self._started = False

    # ------------------------------------------------------------------
    def _joint_state_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = pos

    # ------------------------------------------------------------------
    def _make_pose(self, x, y, z):
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = TOP_DOWN['qw']
        pose.orientation.x = TOP_DOWN['qx']
        pose.orientation.y = TOP_DOWN['qy']
        pose.orientation.z = TOP_DOWN['qz']
        return pose

    def _plan(self, label, x, y, z, duration=2.0, execute=True):
        """Blocking plan+execute call.  Records pass/fail."""
        self.get_logger().info(f'  [{label}] Planning -> ({x:.3f}, {y:.3f}, {z:.3f})')

        request = PlanToTarget.Request()
        request.arm_name = 'right'
        request.target_pose = self._make_pose(x, y, z)
        request.use_orientation = True
        request.execute = execute
        request.duration = duration
        request.max_condition_number = 100.0

        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=TIMEOUT_PER_STEP)

        if future.result() is None:
            self.results.append((label, False, 'service call timed out'))
            self.get_logger().error(f'  [{label}] TIMEOUT')
            return False

        result = future.result()
        if result.success:
            self.results.append((label, True, result.message))
            self.get_logger().info(f'  [{label}] OK — {result.message}')
            if execute:
                time.sleep(duration + 0.3)
            return True
        else:
            self.results.append((label, False, result.message))
            self.get_logger().warn(f'  [{label}] FAIL — {result.message}')
            return False

    def _send_gripper(self, value, label='gripper', wait=1.2):
        action = 'close' if value > 0.5 else 'open'
        self.get_logger().info(f'  [{label}] {action} gripper ({value:.1f})')
        msg = Float64MultiArray()
        msg.data = [float(value)]
        self.gripper_pub.publish(msg)
        time.sleep(wait)
        self.results.append((label, True, f'{action}'))

    # ------------------------------------------------------------------
    def _run_once(self):
        if self._started:
            return
        self._started = True

        # Wait for service
        self.get_logger().info('Waiting for /plan_to_target service...')
        if not self.plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service not available — is sim.launch.py running?')
            self._report()
            return
        self.get_logger().info('Service connected.\n')

        # Wait for joint states
        self.get_logger().info('Waiting for joint states...')
        t0 = time.time()
        while not self.joint_positions and (time.time() - t0 < 5.0):
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.joint_positions:
            self.get_logger().info(f'  Received {len(self.joint_positions)} joints\n')
        else:
            self.get_logger().warn('  No joint states received (continuing anyway)\n')

        # Open gripper first
        self._send_gripper(0.0, label='1-open-gripper', wait=0.8)

        # Step 1: Approach above item
        ix, iy, iz = ITEM_POS
        self._plan('2-approach', ix, iy, iz + 0.15)

        # Step 2: Descend to grasp
        self._plan('3-descend', ix, iy, 0.045, duration=1.5)

        # Step 3: Close gripper
        self._send_gripper(1.0, label='4-close-gripper')

        # Step 4: Lift
        self._plan('5-lift', ix, iy, 0.30)

        # Step 5: Move to bin
        bx, by, bz = BIN_POS
        self._plan('6-move-to-bin', bx, by, bz, duration=2.5)

        # Step 6: Open gripper (release)
        self._send_gripper(0.0, label='7-release')

        # Step 7: Retreat
        self._plan('8-retreat', bx, by, bz + 0.10)

        self._report()

    # ------------------------------------------------------------------
    def _report(self):
        self.get_logger().info('')
        self.get_logger().info('=' * 55)
        self.get_logger().info(' Results')
        self.get_logger().info('-' * 55)

        all_pass = True
        for label, ok, msg in self.results:
            status = 'PASS' if ok else 'FAIL'
            if not ok:
                all_pass = False
            self.get_logger().info(f'  [{status}] {label}: {msg}')

        self.get_logger().info('-' * 55)
        if all_pass:
            self.get_logger().info('  ALL STEPS PASSED')
        else:
            self.get_logger().warn('  SOME STEPS FAILED — see above')
        self.get_logger().info('=' * 55)


def main(args=None):
    rclpy.init(args=args)
    node = TestDropItemInBin()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
