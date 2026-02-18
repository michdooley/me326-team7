#!/usr/bin/env python3
"""
GraspNet Grasp Demo

Full grasp pipeline for TidyBot2:
  1. Scan camera (pan-tilt) to find a coloured cube using HSV detection
  2. Back-project the cube pixel to 3D using depth (camera frame)
  3. Call /plan_grasp (GraspNet) → pre-grasp + grasp poses in base_link
  4. Execute pre-grasp via /plan_to_target
  5. Execute grasp   via /plan_to_target
  6. Close gripper
  7. Lift the object

Coordinate frames (for reference):
  camera_color_optical_frame: Z forward, X right, Y down
  base_link:                  +X = robot left, -Y = robot forward, +Z = up

Usage:
    # Terminal 1: start simulation with grasp scene
    ros2 launch tidybot_bringup sim.launch.py scene:=scene_grasp.xml

    # Terminal 2: start grasp planner (GraspNet)
    ros2 run tidybot_perception grasp_planner_node \\
        --ros-args -p model_path:=$HOME/graspnet-baseline/logs/checkpoint-rs.tar

    # Terminal 3: run this demo
    ros2 run tidybot_bringup grasp_demo.py --ros-args -p color:=red
    # or: ros2 run tidybot_bringup grasp_demo.py --ros-args -p color:=green
    # or: ros2 run tidybot_bringup grasp_demo.py --ros-args -p color:=blue
"""

import time
import numpy as np

import rclpy
from rclpy.node import Node

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped/Point TF support

from geometry_msgs.msg import Pose, PointStamped
from std_msgs.msg import Float64MultiArray
from tidybot_msgs.msg import GripperCommand
from tidybot_msgs.srv import PlanGrasp, PlanToTarget

# camera_scanner.py is installed alongside this script in lib/tidybot_bringup/
from camera_scanner import CameraScanner, Detection

# coord_converter is part of the tidybot_perception Python package
from tidybot_perception.coord_converter import CoordConverter


# ── Grasp orientation fallback (base_link frame, wxyz quaternion) ─────────────
#
# For a top-down grasp, the gripper fingers point straight down (-Z in base_link).
# This is used when GraspNet is unavailable or fails.
#
# Fingers pointing down (Ry(π/2) rotation):
#   site x-axis → base_link -Z (fingers down)
#   site z-axis → base_link +X (approach from front)
ORIENT_FINGERS_DOWN = (0.5, 0.5, 0.5, -0.5)   # (qw, qx, qy, qz)

# Gripper position: 0.0 = fully open, 1.0 = fully closed
GRIPPER_OPEN   = 0.0
GRIPPER_CLOSED = 0.8   # 80% — tune to object size


class GraspDemo(Node):
    """
    Main grasp demo node.

    Drives the full scan → GraspNet → IK → execute pipeline.
    Designed to run as a single-shot node (performs one grasp then exits).
    """

    def __init__(self):
        super().__init__('grasp_demo')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('color', 'red')
        self.declare_parameter('arm', 'right')
        self.declare_parameter('lift_height', 0.15)    # metres to lift after grasp
        self.declare_parameter('grasp_duration', 2.0)  # seconds per arm move
        self.declare_parameter('use_graspnet', True)   # False = fallback top-down

        self._color          = self.get_parameter('color').value
        self._arm            = self.get_parameter('arm').value
        self._lift_height    = self.get_parameter('lift_height').value
        self._grasp_duration = self.get_parameter('grasp_duration').value
        self._use_graspnet   = self.get_parameter('use_graspnet').value

        # ── TF2 ──────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._coord_conv  = CoordConverter(self._tf_buffer)

        # ── Service clients ───────────────────────────────────────────────
        self._plan_grasp_client = self.create_client(PlanGrasp, '/plan_grasp')
        self._plan_to_target    = self.create_client(PlanToTarget, '/plan_to_target')

        # ── Gripper publishers ────────────────────────────────────────────
        self._right_gripper_pub = self.create_publisher(
            GripperCommand, '/right_gripper/command', 10
        )
        self._left_gripper_pub = self.create_publisher(
            GripperCommand, '/left_gripper/command', 10
        )

        # ── Camera scanner helper ────────────────────────────────────────
        self._scanner = CameraScanner(self)

        # ── Announce ─────────────────────────────────────────────────────
        self.get_logger().info('=' * 55)
        self.get_logger().info('GraspNet Grasp Demo')
        self.get_logger().info('=' * 55)
        self.get_logger().info(f'  Target colour : {self._color}')
        self.get_logger().info(f'  Arm           : {self._arm}')
        self.get_logger().info(f'  Use GraspNet  : {self._use_graspnet}')
        self.get_logger().info('')

    # ── Service helpers ───────────────────────────────────────────────────────

    def _call_plan_grasp(
        self, object_pos: PointStamped
    ) -> 'PlanGrasp.Response | None':
        """Call /plan_grasp synchronously; return None on failure."""
        if not self._plan_grasp_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/plan_grasp service not available')
            return None
        req = PlanGrasp.Request()
        req.object_position = object_pos
        req.object_class = self._color + '_cube'
        req.arm_name = self._arm
        future = self._plan_grasp_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done() or future.exception():
            self.get_logger().error(f'/plan_grasp error: {future.exception()}')
            return None
        return future.result()

    def _plan_and_execute(
        self, pose: Pose, label: str, execute: bool = True
    ) -> bool:
        """Call /plan_to_target; wait for result; return success flag."""
        if not self._plan_to_target.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/plan_to_target not available')
            return False

        req = PlanToTarget.Request()
        req.arm_name        = self._arm
        req.target_pose     = pose
        req.use_orientation = True
        req.execute         = execute
        req.duration        = self._grasp_duration
        req.max_condition_number = 100.0

        future = self._plan_to_target.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        if not future.done() or future.exception():
            self.get_logger().error(f'{label}: timeout / exception')
            return False

        result = future.result()
        if result.success:
            self.get_logger().info(
                f'{label}: OK  pos_err={result.position_error:.4f} m  '
                f'ori_err={np.degrees(result.orientation_error):.1f}°'
            )
            if execute:
                time.sleep(self._grasp_duration + 0.5)
        else:
            self.get_logger().warn(f'{label}: FAILED — {result.message}')
        return result.success

    # ── Gripper ───────────────────────────────────────────────────────────────

    def _set_gripper(self, position: float, effort: float = 0.5) -> None:
        msg = GripperCommand()
        msg.position = position
        msg.effort   = effort
        pub = (
            self._right_gripper_pub
            if self._arm == 'right'
            else self._left_gripper_pub
        )
        pub.publish(msg)
        time.sleep(1.0)

    # ── Grasp execution sequence ──────────────────────────────────────────────

    def _execute_grasp_sequence(
        self, grasp_pose: Pose, pre_grasp_pose: Pose
    ) -> bool:
        """Execute: open → pre-grasp → grasp → close → lift."""
        self.get_logger().info('--- Step 1: Open gripper ---')
        self._set_gripper(GRIPPER_OPEN)

        self.get_logger().info('--- Step 2: Move to pre-grasp ---')
        if not self._plan_and_execute(pre_grasp_pose, 'pre-grasp'):
            self.get_logger().error('Pre-grasp failed — aborting')
            return False

        self.get_logger().info('--- Step 3: Move to grasp ---')
        if not self._plan_and_execute(grasp_pose, 'grasp'):
            self.get_logger().error('Grasp pose failed — aborting')
            return False

        self.get_logger().info('--- Step 4: Close gripper ---')
        self._set_gripper(GRIPPER_CLOSED, effort=0.8)

        self.get_logger().info('--- Step 5: Lift object ---')
        lift = Pose()
        lift.position.x  = grasp_pose.position.x
        lift.position.y  = grasp_pose.position.y
        lift.position.z  = grasp_pose.position.z + self._lift_height
        lift.orientation = grasp_pose.orientation
        self._plan_and_execute(lift, 'lift')  # non-fatal if lift fails

        return True

    # ── Fallback: top-down grasp without GraspNet ─────────────────────────────

    def _fallback_grasp(self, obj_cam: PointStamped) -> bool:
        """
        Simple top-down grasp when GraspNet is unavailable.
        Converts camera-frame object point to base_link, then grasps from above.
        """
        try:
            obj_base = self._coord_conv.point_camera_to_base(obj_cam)
        except Exception as exc:
            self.get_logger().error(f'TF camera→base_link failed: {exc}')
            return False

        qw, qx, qy, qz = ORIENT_FINGERS_DOWN

        grasp = Pose()
        grasp.position.x  = obj_base.point.x
        grasp.position.y  = obj_base.point.y
        grasp.position.z  = obj_base.point.z + 0.06   # above cube top surface
        grasp.orientation.w = qw
        grasp.orientation.x = qx
        grasp.orientation.y = qy
        grasp.orientation.z = qz

        pre_grasp = Pose()
        pre_grasp.position.x  = grasp.position.x
        pre_grasp.position.y  = grasp.position.y
        pre_grasp.position.z  = grasp.position.z + 0.12
        pre_grasp.orientation = grasp.orientation

        return self._execute_grasp_sequence(grasp, pre_grasp)

    # ── Main pipeline ─────────────────────────────────────────────────────────

    def run(self) -> None:
        """Execute the full grasp pipeline (blocks until done)."""

        # ── 1. Scan for target cube ───────────────────────────────────────
        self.get_logger().info(f'Scanning for {self._color!r} cube ...')
        det: Detection | None = self._scanner.scan_for_object(self._color)
        if det is None:
            self.get_logger().error(
                f'No {self._color} cube found — '
                'check that scene_grasp.xml is loaded and camera is working'
            )
            return

        # ── 2. Back-project detection pixel → 3D point in camera frame ───
        ci    = det.camera_info
        fx    = ci.k[0]
        fy    = ci.k[4]
        cx_px = ci.k[2]
        cy_px = ci.k[5]

        # Sample a small patch around centroid for a stable depth reading
        u, v  = det.pixel_cx, det.pixel_cy
        patch = det.depth_image[
            max(0, v - 5) : v + 6,
            max(0, u - 5) : u + 6,
        ]
        valid = patch[(patch > 100) & (patch < 5000)]  # 0.1 – 5 m in mm
        if len(valid) == 0:
            self.get_logger().error(
                'No valid depth near detected centroid — '
                'ensure depth camera is working and cube is in range (0.1–5 m)'
            )
            return

        depth_m = float(np.median(valid)) / 1000.0       # mm → metres
        xyz_cam = CoordConverter.depth_pixel_to_camera_point(
            u, v, depth_m, fx, fy, cx_px, cy_px
        )
        self.get_logger().info(
            f'Object in camera frame: '
            f'({xyz_cam[0]:.3f}, {xyz_cam[1]:.3f}, {xyz_cam[2]:.3f}) m'
        )

        # Build PointStamped in camera_color_optical_frame
        obj_cam = PointStamped()
        obj_cam.header.frame_id = 'camera_color_optical_frame'
        obj_cam.header.stamp    = self.get_clock().now().to_msg()
        obj_cam.point.x = float(xyz_cam[0])
        obj_cam.point.y = float(xyz_cam[1])
        obj_cam.point.z = float(xyz_cam[2])

        # ── 3. Plan grasp (GraspNet or fallback) ─────────────────────────
        graspnet_succeeded = False

        if self._use_graspnet:
            self.get_logger().info('Calling GraspNet /plan_grasp ...')
            resp = self._call_plan_grasp(obj_cam)

            if resp is not None and resp.success:
                self.get_logger().info(f'GraspNet: {resp.message}')
                graspnet_succeeded = self._execute_grasp_sequence(
                    resp.grasp_pose, resp.pre_grasp_pose
                )
            else:
                msg = resp.message if resp else 'service call failed'
                self.get_logger().warn(
                    f'GraspNet failed ({msg}) — falling back to top-down grasp'
                )

        if not graspnet_succeeded:
            self.get_logger().info('Using top-down fallback grasp ...')
            self._fallback_grasp(obj_cam)

        self.get_logger().info('')
        self.get_logger().info('=' * 55)
        self.get_logger().info('Grasp pipeline complete!')
        self.get_logger().info('=' * 55)


def main(args=None):
    rclpy.init(args=args)
    node = GraspDemo()

    # Wait for the IK planner (always required)
    node.get_logger().info('Waiting for /plan_to_target service ...')
    while not node._plan_to_target.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('  ... still waiting')
    node.get_logger().info('Service connected!')

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
