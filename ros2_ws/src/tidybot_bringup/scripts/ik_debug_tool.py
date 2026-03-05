#!/usr/bin/env python3
"""
IK Debug Tool — Interactive tool for diagnosing IK/motion planner failures.

Sends test poses to the /plan_to_target service and prints detailed diagnostics
including: position/orientation errors, condition number (singularity), joint
solution, and the specific failure reason.

Modes:
    interactive  - Enter poses manually via command line
    sweep        - Sweep XYZ grid to map the reachable workspace
    replay       - Test a specific known-failing pose

Requires:
    - Motion planner running (sim or real)
      Sim:  ros2 launch tidybot_bringup sim.launch.py
      Real: ros2 launch tidybot_bringup real.launch.py use_planner:=true

Usage:
    # Interactive mode (default):
    ros2 run tidybot_bringup ik_debug_tool.py

    # Test a specific pose:
    ros2 run tidybot_bringup ik_debug_tool.py --ros-args \
        -p mode:=replay -p x:=0.3 -p y:=-0.1 -p z:=0.05

    # Sweep workspace:
    ros2 run tidybot_bringup ik_debug_tool.py --ros-args -p mode:=sweep
"""

import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion
from scipy.spatial.transform import Rotation

from tidybot_msgs.srv import PlanToTarget


# Fingers-down quaternion (top-down grasp orientation)
FINGERS_DOWN_QUAT = Quaternion(x=0.5, y=0.5, z=0.5, w=-0.5)

# WX250s joint limits for reference
JOINT_LIMITS = {
    'waist':        (-3.14159, 3.14159),
    'shoulder':     (-1.8849,  1.9897),
    'elbow':        (-2.1468,  1.6057),
    'forearm_roll': (-3.14159, 3.14159),
    'wrist_angle':  (-1.7453,  2.1468),
    'wrist_rotate': (-3.14159, 3.14159),
}

JOINT_NAMES = list(JOINT_LIMITS.keys())


class IKDebugTool(Node):
    def __init__(self):
        super().__init__('ik_debug_tool')

        self.declare_parameter('arm', 'right')
        self.declare_parameter('mode', 'interactive')
        self.declare_parameter('x', 0.3)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.05)
        self.declare_parameter('max_cond', 100.0)

        self.arm = self.get_parameter('arm').value
        self.mode = self.get_parameter('mode').value
        self.max_cond = self.get_parameter('max_cond').value

        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')

    def wait_for_service(self):
        self.get_logger().info('Waiting for /plan_to_target service...')
        if not self.plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                'Service not available! Is the motion planner running?')
            return False
        self.get_logger().info('Service connected.')
        return True

    def test_pose(self, x, y, z, quat=None, use_orientation=True,
                  execute=False, duration=2.0):
        """Send a pose to the planner and print detailed results."""
        if quat is None:
            quat = FINGERS_DOWN_QUAT

        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation = quat

        req = PlanToTarget.Request()
        req.arm_name = self.arm
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = execute
        req.duration = duration
        req.max_condition_number = self.max_cond

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

        if not future.done() or future.exception():
            print('\n  ERROR: Service call timed out or threw exception')
            return None

        result = future.result()
        self._print_result(result, pose, use_orientation)
        return result

    def _print_result(self, result, pose, use_orientation):
        """Print comprehensive IK diagnostic output."""
        q = Rotation.from_quat([
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w])
        rpy = q.as_euler('xyz', degrees=True)

        status = '\033[92mOK\033[0m' if result.success else '\033[91mFAILED\033[0m'
        print(f'\n  === IK Result: {status} ===')
        print(f'  Target:')
        print(f'    position      : ({pose.position.x:.4f}, '
              f'{pose.position.y:.4f}, {pose.position.z:.4f}) m')
        print(f'    orientation   : quat=({pose.orientation.x:.3f}, '
              f'{pose.orientation.y:.3f}, {pose.orientation.z:.3f}, '
              f'{pose.orientation.w:.3f})')
        print(f'    orientation   : rpy=({rpy[0]:.1f}, {rpy[1]:.1f}, '
              f'{rpy[2]:.1f}) deg')
        print(f'    use_orient    : {use_orientation}')
        print(f'    arm           : {self.arm}')
        print()
        print(f'  Errors:')
        print(f'    position_error: {result.position_error:.4f} m '
              f'(threshold: ~0.01m sim / ~0.03m real)')
        print(f'    orient_error  : {result.orientation_error:.4f} rad '
              f'({np.degrees(result.orientation_error):.1f} deg, '
              f'threshold: ~0.1 rad / ~5.7 deg)')
        print(f'    cond_number   : {result.condition_number:.1f} '
              f'(threshold: {self.max_cond:.0f})')

        if result.joint_positions:
            joints = list(result.joint_positions)
            print()
            print(f'  Joint Solution:')
            print(f'    {"Joint":<15s}  {"Value (rad)":>11s}  '
                  f'{"Value (deg)":>11s}  {"Min":>8s}  {"Max":>8s}  '
                  f'{"Margin":>8s}')
            print(f'    {"-"*15}  {"-"*11}  {"-"*11}  {"-"*8}  '
                  f'{"-"*8}  {"-"*8}')
            for name, val in zip(JOINT_NAMES, joints):
                lo, hi = JOINT_LIMITS[name]
                margin = min(val - lo, hi - val)
                margin_pct = margin / (hi - lo) * 100
                flag = ' <-- NEAR LIMIT' if margin_pct < 5 else ''
                print(f'    {name:<15s}  {val:+11.4f}  '
                      f'{np.degrees(val):+11.1f}  '
                      f'{np.degrees(lo):+8.1f}  {np.degrees(hi):+8.1f}  '
                      f'{margin_pct:7.1f}%{flag}')

        if not result.success:
            print()
            print(f'  \033[91mFailure reason: {result.message}\033[0m')
            # Diagnose the specific failure
            self._diagnose_failure(result)
        print()

    def _diagnose_failure(self, result):
        """Provide actionable diagnosis of why IK failed."""
        msg = result.message.lower()
        print(f'  Diagnosis:')
        if 'convergence' in msg or 'converge' in msg:
            print(f'    -> IK solver could not reach the target pose.')
            if result.position_error > 0.03:
                print(f'    -> Position error {result.position_error:.3f}m '
                      f'is very large — pose may be out of reach.')
            else:
                print(f'    -> Position error {result.position_error:.3f}m '
                      f'is small — try use_orientation=false or adjust '
                      f'orientation.')
            if result.orientation_error > 0.5:
                print(f'    -> Orientation error is large — the requested '
                      f'orientation may be kinematically infeasible.')
        elif 'singularity' in msg or 'condition' in msg:
            print(f'    -> Arm is near a singularity at this configuration.')
            print(f'    -> Condition number {result.condition_number:.0f} '
                  f'exceeds threshold {self.max_cond:.0f}.')
            print(f'    -> Try: slightly adjust target position, or '
                  f'increase max_condition_number.')
        elif 'collision' in msg:
            print(f'    -> Arms would collide at this configuration.')
            print(f'    -> Try: move target further from robot center, '
                  f'or use a different arm.')
        elif 'joint' in msg and 'limit' in msg:
            print(f'    -> Solution requires joints beyond their limits.')
            if result.joint_positions:
                for name, val in zip(JOINT_NAMES,
                                     list(result.joint_positions)):
                    lo, hi = JOINT_LIMITS[name]
                    if val < lo or val > hi:
                        print(f'    -> {name}: {np.degrees(val):.1f} deg '
                              f'outside [{np.degrees(lo):.1f}, '
                              f'{np.degrees(hi):.1f}]')
        else:
            print(f'    -> Unknown failure type. Check planner logs for '
                  f'details.')

    def run_interactive(self):
        """Interactive mode: enter poses via command line."""
        print('\n=== IK Debug Tool (Interactive Mode) ===')
        print(f'Arm: {self.arm}')
        print(f'Max condition number: {self.max_cond}')
        print()
        print('Commands:')
        print('  x y z              - Test pose (fingers-down orientation)')
        print('  x y z yaw          - Test with yaw angle (degrees)')
        print('  x y z pos-only     - Test position-only (no orientation)')
        print('  x y z both         - Test with and without orientation')
        print('  sweep              - Run workspace sweep')
        print('  exec x y z         - Execute (actually move arm)')
        print('  q                  - Quit')
        print()

        while True:
            try:
                line = input('ik> ').strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not line or line == 'q':
                break

            if line == 'sweep':
                self.run_sweep()
                continue

            parts = line.split()
            execute = False
            if parts[0] == 'exec':
                execute = True
                parts = parts[1:]

            try:
                x = float(parts[0])
                y = float(parts[1])
                z = float(parts[2])
            except (IndexError, ValueError):
                print('  Usage: x y z [yaw|pos-only|both]')
                continue

            if len(parts) >= 4:
                flag = parts[3]
                if flag == 'pos-only':
                    self.test_pose(x, y, z, use_orientation=False,
                                   execute=execute)
                elif flag == 'both':
                    print('  --- With orientation ---')
                    self.test_pose(x, y, z, use_orientation=True,
                                   execute=False)
                    print('  --- Position-only ---')
                    self.test_pose(x, y, z, use_orientation=False,
                                   execute=False)
                else:
                    # Interpret as yaw angle in degrees
                    try:
                        yaw = float(flag)
                        from tidybot_perception.grasp_geometry import (
                            yaw_to_grasp_quaternion)
                        qw, qx, qy, qz = yaw_to_grasp_quaternion(
                            np.radians(yaw))
                        quat = Quaternion(x=qx, y=qy, z=qz, w=qw)
                        self.test_pose(x, y, z, quat=quat, execute=execute)
                    except (ValueError, ImportError):
                        print(f'  Unknown flag: {flag}')
            else:
                self.test_pose(x, y, z, execute=execute)

    def run_sweep(self):
        """Sweep XYZ grid to map reachable workspace."""
        print('\n=== Workspace Sweep ===')
        print(f'Arm: {self.arm}')
        print()

        # Sweep range (in meters, relative to base_link)
        x_range = np.arange(0.15, 0.45, 0.05)
        y_range = np.arange(-0.25, 0.05, 0.05) if self.arm == 'right' \
            else np.arange(-0.05, 0.25, 0.05)
        z_range = np.arange(0.01, 0.20, 0.03)

        total = len(x_range) * len(y_range) * len(z_range)
        ok_count = 0
        fail_count = 0
        failures = []

        print(f'Testing {total} poses...')
        print(f'  X: {x_range[0]:.2f} to {x_range[-1]:.2f} '
              f'({len(x_range)} steps)')
        print(f'  Y: {y_range[0]:.2f} to {y_range[-1]:.2f} '
              f'({len(y_range)} steps)')
        print(f'  Z: {z_range[0]:.2f} to {z_range[-1]:.2f} '
              f'({len(z_range)} steps)')
        print()

        for x in x_range:
            for y in y_range:
                for z in z_range:
                    pose = Pose()
                    pose.position.x = float(x)
                    pose.position.y = float(y)
                    pose.position.z = float(z)
                    pose.orientation = FINGERS_DOWN_QUAT

                    req = PlanToTarget.Request()
                    req.arm_name = self.arm
                    req.target_pose = pose
                    req.use_orientation = True
                    req.execute = False
                    req.duration = 2.0
                    req.max_condition_number = self.max_cond

                    future = self.plan_client.call_async(req)
                    rclpy.spin_until_future_complete(
                        self, future, timeout_sec=10.0)

                    if future.done() and not future.exception():
                        result = future.result()
                        status = '\033[92m OK \033[0m' if result.success \
                            else '\033[91mFAIL\033[0m'
                        print(f'  ({x:.2f}, {y:.2f}, {z:.2f}): {status} '
                              f'pos_err={result.position_error:.3f}m '
                              f'cond={result.condition_number:.0f}',
                              end='')
                        if result.success:
                            ok_count += 1
                        else:
                            fail_count += 1
                            failures.append((x, y, z, result.message))
                            print(f'  [{result.message}]', end='')
                        print()
                    else:
                        fail_count += 1
                        print(f'  ({x:.2f}, {y:.2f}, {z:.2f}): '
                              f'\033[91mTIMEOUT\033[0m')

        print(f'\n=== Sweep Summary ===')
        print(f'  Total:  {total}')
        print(f'  OK:     {ok_count} ({ok_count/total*100:.0f}%)')
        print(f'  Failed: {fail_count} ({fail_count/total*100:.0f}%)')

        if failures:
            # Group failures by reason
            reasons = {}
            for x, y, z, msg in failures:
                key = msg.split(':')[0] if ':' in msg else msg
                reasons.setdefault(key, []).append((x, y, z))

            print(f'\n  Failure breakdown:')
            for reason, poses in sorted(reasons.items(),
                                        key=lambda x: -len(x[1])):
                print(f'    {reason}: {len(poses)} poses')

    def run_replay(self):
        """Test a single specified pose."""
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        z = self.get_parameter('z').value

        print(f'\n=== Replay Mode ===')
        print(f'Testing pose ({x}, {y}, {z}) with fingers-down orientation')
        print()

        print('--- With orientation ---')
        self.test_pose(x, y, z, use_orientation=True)

        print('--- Position-only ---')
        self.test_pose(x, y, z, use_orientation=False)


def main(args=None):
    rclpy.init(args=args)
    node = IKDebugTool()

    if not node.wait_for_service():
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        if node.mode == 'interactive':
            node.run_interactive()
        elif node.mode == 'sweep':
            node.run_sweep()
        elif node.mode == 'replay':
            node.run_replay()
        else:
            print(f'Unknown mode: {node.mode}')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
