#!/usr/bin/env python3
"""
Grasp Execute — plan and execute a grasp at a known object position.

Skips camera scanning; takes object coordinates directly as parameters.

Pipeline:
  1. Call /plan_grasp → get grasp + pre-grasp poses
  2. Open gripper
  3. Move to pre-grasp via /plan_to_target
  4. Move to grasp via /plan_to_target
  5. Close gripper
  6. Lift object

Usage:
    ros2 run tidybot_bringup grasp_execute.py \
        --ros-args -p x:=0.0 -p y:=-0.45 -p z:=0.025 \
                   -p arm:=right -p frame:=base_link
"""

import time
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PointStamped
from tidybot_msgs.msg import GripperCommand
from tidybot_msgs.srv import PlanGrasp, PlanToTarget

GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.8


class GraspExecute(Node):
    def __init__(self):
        super().__init__('grasp_execute')

        # Parameters
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', -0.45)
        self.declare_parameter('z', 0.025)
        self.declare_parameter('frame', 'base_link')
        self.declare_parameter('arm', 'right')
        self.declare_parameter('object_class', 'red_cube')
        self.declare_parameter('lift_height', 0.15)
        self.declare_parameter('duration', 2.0)

        self._x = self.get_parameter('x').value
        self._y = self.get_parameter('y').value
        self._z = self.get_parameter('z').value
        self._frame = self.get_parameter('frame').value
        self._arm = self.get_parameter('arm').value
        self._obj_class = self.get_parameter('object_class').value
        self._lift_height = self.get_parameter('lift_height').value
        self._duration = self.get_parameter('duration').value

        # Clients
        self._plan_grasp_cli = self.create_client(PlanGrasp, '/plan_grasp')
        self._plan_to_target_cli = self.create_client(PlanToTarget, '/plan_to_target')

        # Gripper publishers
        self._gripper_pubs = {
            'right': self.create_publisher(GripperCommand, '/right_gripper/command', 10),
            'left': self.create_publisher(GripperCommand, '/left_gripper/command', 10),
        }

    # ── helpers ────────────────────────────────────────────────────────────

    def _set_gripper(self, position: float, effort: float = 0.5):
        msg = GripperCommand()
        msg.position = position
        msg.effort = effort
        self._gripper_pubs[self._arm].publish(msg)
        self.get_logger().info(f'Gripper → {"OPEN" if position < 0.1 else "CLOSE"} ({position:.1f})')
        time.sleep(1.0)

    def _call_service(self, client, request, label: str, timeout: float = 30.0):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'{label}: service not available')
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.exception():
            self.get_logger().error(f'{label}: {future.exception()}')
            return None
        return future.result()

    def _move_to(self, pose: Pose, label: str) -> bool:
        req = PlanToTarget.Request()
        req.arm_name = self._arm
        req.target_pose = pose
        req.use_orientation = True
        req.execute = True
        req.duration = self._duration
        req.max_condition_number = 200.0

        result = self._call_service(self._plan_to_target_cli, req, label)
        if result is None:
            return False

        if result.success:
            self.get_logger().info(
                f'{label}: OK  pos_err={result.position_error:.4f}m  '
                f'ori_err={np.degrees(result.orientation_error):.1f}°'
            )
            time.sleep(self._duration + 0.5)
            return True
        else:
            self.get_logger().warn(f'{label}: FAILED — {result.message}')
            return False

    # ── main pipeline ──────────────────────────────────────────────────────

    def run(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info('Grasp Execute')
        self.get_logger().info(f'  Object: ({self._x}, {self._y}, {self._z}) in {self._frame}')
        self.get_logger().info(f'  Arm: {self._arm}')
        self.get_logger().info('=' * 50)

        # 1. Plan grasp
        self.get_logger().info('--- Step 1: Plan grasp via /plan_grasp ---')
        obj_pt = PointStamped()
        obj_pt.header.frame_id = self._frame
        obj_pt.header.stamp = self.get_clock().now().to_msg()
        obj_pt.point.x = self._x
        obj_pt.point.y = self._y
        obj_pt.point.z = self._z

        req = PlanGrasp.Request()
        req.object_position = obj_pt
        req.object_class = self._obj_class
        req.arm_name = self._arm

        resp = self._call_service(self._plan_grasp_cli, req, 'plan_grasp')
        if resp is None or not resp.success:
            msg = resp.message if resp else 'service call failed'
            self.get_logger().error(f'Grasp planning failed: {msg}')
            return

        self.get_logger().info(f'Grasp planned: {resp.message}')
        grasp_pose = resp.grasp_pose
        pre_grasp_pose = resp.pre_grasp_pose

        self.get_logger().info(
            f'  grasp:     pos=({grasp_pose.position.x:.3f}, {grasp_pose.position.y:.3f}, {grasp_pose.position.z:.3f})')
        self.get_logger().info(
            f'  pre-grasp: pos=({pre_grasp_pose.position.x:.3f}, {pre_grasp_pose.position.y:.3f}, {pre_grasp_pose.position.z:.3f})')

        # 2. Open gripper
        self.get_logger().info('--- Step 2: Open gripper ---')
        self._set_gripper(GRIPPER_OPEN)

        # 3. Move to pre-grasp
        self.get_logger().info('--- Step 3: Move to pre-grasp ---')
        if not self._move_to(pre_grasp_pose, 'pre-grasp'):
            self.get_logger().error('Pre-grasp failed — aborting')
            return

        # 4. Move to grasp
        self.get_logger().info('--- Step 4: Move to grasp ---')
        if not self._move_to(grasp_pose, 'grasp'):
            self.get_logger().error('Grasp failed — aborting')
            return

        # 5. Close gripper
        self.get_logger().info('--- Step 5: Close gripper ---')
        self._set_gripper(GRIPPER_CLOSED, effort=0.8)

        # 6. Lift
        self.get_logger().info('--- Step 6: Lift ---')
        lift = Pose()
        lift.position.x = grasp_pose.position.x
        lift.position.y = grasp_pose.position.y
        lift.position.z = grasp_pose.position.z + self._lift_height
        lift.orientation = grasp_pose.orientation
        self._move_to(lift, 'lift')

        self.get_logger().info('=' * 50)
        self.get_logger().info('Grasp execute complete!')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = GraspExecute()

    node.get_logger().info('Waiting for /plan_to_target ...')
    while not node._plan_to_target_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('  ... still waiting')

    node.get_logger().info('Waiting for /plan_grasp ...')
    while not node._plan_grasp_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('  ... still waiting')

    node.get_logger().info('Services connected!')

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
