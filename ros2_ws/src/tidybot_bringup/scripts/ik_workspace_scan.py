#!/usr/bin/env python3
"""
IK Workspace Scanner

Sweeps X, Y, Z in base_link frame for both arms, calling /plan_to_target
to check reachability. Saves results to CSV files.

Usage:
  ros2 run tidybot_bringup ik_workspace_scan.py

  # Or with custom ranges (metres):
  ros2 run tidybot_bringup ik_workspace_scan.py \
      --x-min 0.1 --x-max 0.5 --y-min -0.3 --y-max 0.3 \
      --z-min 0.0 --z-max 0.4 --step 0.02

Requires: nav.launch.py (or sim.launch.py) running so /plan_to_target is available.
"""

import argparse
import csv
import itertools
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tidybot_msgs.srv import PlanToTarget


class IKWorkspaceScanner(Node):
    def __init__(self, args):
        super().__init__('ik_workspace_scanner')
        self.cli = self.create_client(PlanToTarget, '/plan_to_target')
        self.get_logger().info('Waiting for /plan_to_target service...')
        self.cli.wait_for_service()
        self.get_logger().info('Service ready.')
        self.args = args

    def call_ik(self, arm: str, x: float, y: float, z: float,
                use_orientation: bool = False) -> dict:
        """Call IK planner and return result dict."""
        req = PlanToTarget.Request()
        req.arm_name = arm
        req.target_pose = Pose()
        req.target_pose.position.x = x
        req.target_pose.position.y = y
        req.target_pose.position.z = z
        req.target_pose.orientation.w = 1.0
        req.use_orientation = use_orientation
        req.execute = False
        req.max_condition_number = 500.0

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            return {'success': False, 'pos_err': -1, 'ori_err': -1,
                    'cond': -1, 'joints': [], 'message': 'timeout'}

        resp = future.result()
        return {
            'success': resp.success,
            'pos_err': resp.position_error,
            'ori_err': resp.orientation_error,
            'cond': resp.condition_number,
            'joints': list(resp.joint_positions),
            'message': resp.message,
        }

    def scan_arm(self, arm: str):
        a = self.args
        xs = np.arange(a.x_min, a.x_max + 1e-9, a.step)
        ys = np.arange(a.y_min, a.y_max + 1e-9, a.step)
        zs = np.arange(a.z_min, a.z_max + 1e-9, a.step)
        total = len(xs) * len(ys) * len(zs)

        filename = f'ik_scan_{arm}_arm.csv'
        self.get_logger().info(
            f'Scanning {arm} arm: {len(xs)}x{len(ys)}x{len(zs)} = {total} points → {filename}')

        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'x', 'y', 'z', 'success', 'pos_err', 'ori_err',
                'cond', 'j0', 'j1', 'j2', 'j3', 'j4', 'j5', 'message'
            ])

            done = 0
            t0 = time.time()
            for x, y, z in itertools.product(xs, ys, zs):
                res = self.call_ik(arm, float(x), float(y), float(z))
                joints = res['joints'] if len(res['joints']) == 6 else [0]*6
                writer.writerow([
                    f'{x:.4f}', f'{y:.4f}', f'{z:.4f}',
                    int(res['success']),
                    f'{res["pos_err"]:.5f}', f'{res["ori_err"]:.5f}',
                    f'{res["cond"]:.1f}',
                    *[f'{j:.4f}' for j in joints],
                    res['message'],
                ])
                done += 1
                if done % 50 == 0:
                    elapsed = time.time() - t0
                    rate = done / elapsed
                    eta = (total - done) / rate if rate > 0 else 0
                    self.get_logger().info(
                        f'  [{arm}] {done}/{total} ({100*done/total:.1f}%) '
                        f'rate={rate:.1f}/s ETA={eta:.0f}s')

        elapsed = time.time() - t0
        self.get_logger().info(f'Done {arm} arm: {total} points in {elapsed:.1f}s → {filename}')
        return filename


def main():
    parser = argparse.ArgumentParser(description='IK Workspace Scanner')
    parser.add_argument('--x-min', type=float, default=0.05)
    parser.add_argument('--x-max', type=float, default=0.50)
    parser.add_argument('--y-min', type=float, default=-0.30)
    parser.add_argument('--y-max', type=float, default=0.30)
    parser.add_argument('--z-min', type=float, default=0.00)
    parser.add_argument('--z-max', type=float, default=0.40)
    parser.add_argument('--step', type=float, default=0.02)
    parser.add_argument('--arm', type=str, default='both',
                        choices=['left', 'right', 'both'],
                        help='Which arm(s) to scan')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = IKWorkspaceScanner(args)

    arms = ['right', 'left'] if args.arm == 'both' else [args.arm]
    files = []
    for arm in arms:
        files.append(node.scan_arm(arm))

    node.get_logger().info(f'All done! Output files: {files}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
