#!/usr/bin/env python3
"""
Map the right arm's reachable workspace for top-down grasps.

Sweeps a grid of (x, y, z, yaw) positions in base_link frame,
calls the motion planner with execute=False and use_orientation=True,
and logs which positions the IK solver can reach.

Usage (requires motion planner to be running):
    ros2 run tidybot_bringup map_arm_workspace.py

Output:
    ~/arm_workspace_map.csv — full grid results
    Terminal summary with sweet-spot centroid
"""

import csv
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Quaternion
from tidybot_msgs.srv import PlanToTarget

# Reuse the same quaternion helper used by the grasp pipeline
from tidybot_perception.grasp_geometry import yaw_to_grasp_quaternion


class WorkspaceMapper(Node):

    def __init__(self):
        super().__init__('workspace_mapper')

        self.plan_client = self.create_client(
            PlanToTarget, '/plan_to_target')

        # Grid parameters
        self.x_range = np.arange(-0.10, 0.22, 0.02)    # 16 pts
        self.y_range = np.arange(-0.45, -0.09, 0.02)    # 18 pts
        self.z_range = [0.02]                            # 2cm off floor
        self.yaw_range = [0.0, np.pi/2]                  # 2 orientations

        self.output_path = os.path.expanduser('~/arm_workspace_map.csv')

    def run(self):
        self.get_logger().info('Waiting for /plan_to_target service...')
        if not self.plan_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error(
                '/plan_to_target not available. '
                'Is the motion planner running?')
            return

        total = (len(self.x_range) * len(self.y_range)
                 * len(self.z_range) * len(self.yaw_range))
        self.get_logger().info(
            f'Starting workspace sweep: {total} IK queries')
        self.get_logger().info(f'Output: {self.output_path}')

        results = []
        successes = 0
        count = 0
        t0 = time.time()

        with open(self.output_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'x', 'y', 'z', 'yaw_deg',
                'success', 'position_error', 'orientation_error',
                'condition_number', 'message',
            ])

            for x in self.x_range:
                for y in self.y_range:
                    for z in self.z_range:
                        for yaw in self.yaw_range:
                            count += 1
                            ok, pos_err, ori_err, cond, msg = \
                                self._try_ik(x, y, z, yaw)

                            row = [
                                f'{x:.3f}', f'{y:.3f}', f'{z:.3f}',
                                f'{np.degrees(yaw):.0f}',
                                int(ok),
                                f'{pos_err:.4f}', f'{ori_err:.4f}',
                                f'{cond:.1f}', msg,
                            ]
                            writer.writerow(row)
                            if ok:
                                successes += 1
                                results.append((x, y, z, yaw, cond))

                            if count % 100 == 0:
                                elapsed = time.time() - t0
                                rate = count / elapsed
                                eta = (total - count) / rate
                                self.get_logger().info(
                                    f'  {count}/{total} '
                                    f'({successes} ok, '
                                    f'ETA {eta:.0f}s)')

        elapsed = time.time() - t0
        self.get_logger().info(
            f'Done! {count} queries in {elapsed:.1f}s '
            f'({successes}/{count} = {100*successes/max(count,1):.1f}% success)')

        self._print_summary(results)

    def _try_ik(self, x, y, z, yaw):
        """Call IK planner for a single pose. Returns (ok, pos_err, ori_err, cond, msg)."""
        qw, qx, qy, qz = yaw_to_grasp_quaternion(yaw)

        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

        req = PlanToTarget.Request()
        req.arm_name = 'right'
        req.target_pose = pose
        req.use_orientation = True
        req.execute = False
        req.duration = 2.0
        req.max_condition_number = 500.0

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done() or future.exception():
            return False, 9.99, 9.99, 999.0, 'timeout'

        r = future.result()
        return (r.success, r.position_error, r.orientation_error,
                r.condition_number, r.message)

    def _print_summary(self, results):
        if not results:
            self.get_logger().warn('No successful IK solutions found!')
            return

        xs = [r[0] for r in results]
        ys = [r[1] for r in results]
        zs = [r[2] for r in results]

        self.get_logger().info('=' * 60)
        self.get_logger().info('WORKSPACE SUMMARY (all successful poses)')
        self.get_logger().info(
            f'  X range: [{min(xs):.3f}, {max(xs):.3f}]')
        self.get_logger().info(
            f'  Y range: [{min(ys):.3f}, {max(ys):.3f}]')
        self.get_logger().info(
            f'  Z range: [{min(zs):.3f}, {max(zs):.3f}]')
        self.get_logger().info(
            f'  Centroid: ({np.mean(xs):.3f}, {np.mean(ys):.3f}, '
            f'{np.mean(zs):.3f})')

        # Sweet spot: filter to low condition number (< 100)
        sweet = [r for r in results if r[4] < 100.0]
        if sweet:
            sx = [r[0] for r in sweet]
            sy = [r[1] for r in sweet]
            sz = [r[2] for r in sweet]
            self.get_logger().info('')
            self.get_logger().info(
                f'SWEET SPOT (condition_number < 100, '
                f'{len(sweet)} poses)')
            self.get_logger().info(
                f'  X range: [{min(sx):.3f}, {max(sx):.3f}]')
            self.get_logger().info(
                f'  Y range: [{min(sy):.3f}, {max(sy):.3f}]')
            self.get_logger().info(
                f'  Z range: [{min(sz):.3f}, {max(sz):.3f}]')
            cx, cy, cz = np.mean(sx), np.mean(sy), np.mean(sz)
            self.get_logger().info(
                f'  Centroid: ({cx:.3f}, {cy:.3f}, {cz:.3f})')
            self.get_logger().info('')
            self.get_logger().info(
                '  --> Use these values in explore_and_find_object.py:')
            self.get_logger().info(
                f'      GRASP_SWEET_SPOT_X = {cx:.3f}')
            self.get_logger().info(
                f'      GRASP_SWEET_SPOT_Y = {cy:.3f}')
        else:
            self.get_logger().warn(
                'No solutions with condition_number < 100')
            # Fall back to all successes
            cx, cy = np.mean(xs), np.mean(ys)
            self.get_logger().info(
                f'  Using all-success centroid: ({cx:.3f}, {cy:.3f})')

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Full results saved to: {self.output_path}')


def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceMapper()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
