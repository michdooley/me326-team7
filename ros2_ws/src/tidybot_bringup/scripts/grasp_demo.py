#!/usr/bin/env python3
"""
GraspNet Grasp Demo

Full grasp pipeline for TidyBot2:
  1. Point camera at table, wait for object detection from classifier node
  2. Back-project detection centroid to 3D using depth (camera frame)
  3. Call /plan_grasp (GraspNet) → pre-grasp + grasp poses in base_link
  4. Execute pre-grasp via /plan_to_target
  5. Execute grasp   via /plan_to_target
  6. Close gripper
  7. Lift the object

Requires the classifier node to be running:
    ros2 run object_classification classifier

Coordinate frames (for reference):
  camera_color_optical_frame: Z forward, X right, Y down
  camera_depth_optical_frame: Z forward, X right, Y down (D435 depth)
  base_link:                  +X = robot left, -Y = robot forward, +Z = up

Usage:
    # Terminal 1: start simulation with fruit scene
    ros2 launch tidybot_bringup sim.launch.py scene:=scene_fruit_grasp.xml

    # Terminal 2: start classifier
    ros2 run object_classification classifier

    # Terminal 3: start grasp planner (GraspNet)
    ros2 run tidybot_perception grasp_planner_node \\
        --ros-args -p model_path:=$HOME/graspnet-baseline/logs/checkpoint-rs.tar

    # Terminal 4: run this demo
    ros2 run tidybot_bringup grasp_demo.py --ros-args -p object:=apple
    # or: ros2 run tidybot_bringup grasp_demo.py --ros-args -p object:=banana
    # or: ros2 run tidybot_bringup grasp_demo.py --ros-args -p object:=orange
"""

import os
import sys
import time
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped/Point TF support

from geometry_msgs.msg import Pose, PointStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray, ColorRGBA, Header
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker
from tidybot_msgs.msg import GripperCommand
from tidybot_msgs.srv import PlanGrasp, PlanToTarget

from cv_bridge import CvBridge

# coord_converter is part of the tidybot_perception Python package
from tidybot_perception.coord_converter import CoordConverter

# RGB-to-depth alignment utility — installed alongside this script in utilities/
sys.path.insert(0, str(Path(__file__).resolve().parent / 'utilities'))
from align_rgb_to_depth import align_rgb, D435_DEPTH_K  # noqa: E402


# ── YOLO COCO class ID mapping (matches classifier.py) ──────────────────────
YOLO_CLASS_IDS = {'person': '0', 'banana': '46', 'apple': '47', 'orange': '49'}

# ── Grasp orientation fallback (base_link frame, wxyz quaternion) ─────────────
ORIENT_FINGERS_DOWN = (0.5, 0.5, 0.5, -0.5)   # (qw, qx, qy, qz)

# Gripper position: 0.0 = fully open, 1.0 = fully closed
GRIPPER_OPEN   = 0.0
GRIPPER_CLOSED = 1.0   # 100% — fully closed for fruit


class GraspDemo(Node):
    """
    Main grasp demo node.

    Drives the full detect → GraspNet → IK → execute pipeline.
    Consumes detections from the external classifier node (/objbbox).
    Designed to run as a single-shot node (performs one grasp then exits).
    """

    def __init__(self):
        super().__init__('grasp_demo')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('object', 'apple')
        self.declare_parameter('arm', 'right')
        self.declare_parameter('lift_height', 0.15)    # metres to lift after grasp
        self.declare_parameter('grasp_duration', 2.0)  # seconds per arm move
        self.declare_parameter('use_graspnet', True)   # False = fallback top-down
        self.declare_parameter('sim', True)             # True = skip RGB-depth alignment

        self._object         = self.get_parameter('object').value
        self._arm            = self.get_parameter('arm').value
        self._lift_height    = self.get_parameter('lift_height').value
        self._grasp_duration = self.get_parameter('grasp_duration').value
        self._use_graspnet   = self.get_parameter('use_graspnet').value
        self._sim            = self.get_parameter('sim').value

        self._bridge = CvBridge()

        # ── TF2 ──────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._coord_conv  = CoordConverter(self._tf_buffer)

        # ── Service clients ───────────────────────────────────────────────
        self._plan_grasp_client = self.create_client(PlanGrasp, '/plan_grasp')
        self._plan_to_target    = self.create_client(PlanToTarget, '/plan_to_target')

        # ── Gripper publishers ────────────────────────────────────────────
        # MuJoCo bridge listens on /right_gripper/cmd (Float64MultiArray),
        # real hardware listens on /right_gripper/command (GripperCommand).
        if self._sim:
            self._right_gripper_pub = self.create_publisher(
                Float64MultiArray, '/right_gripper/cmd', 10
            )
            self._left_gripper_pub = self.create_publisher(
                Float64MultiArray, '/left_gripper/cmd', 10
            )
        else:
            self._right_gripper_pub = self.create_publisher(
                GripperCommand, '/right_gripper/command', 10
            )
            self._left_gripper_pub = self.create_publisher(
                GripperCommand, '/left_gripper/command', 10
            )

        # ── Camera pan-tilt publisher ─────────────────────────────────────
        self._pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt', 10)

        # ── Camera subscribers ────────────────────────────────────────────
        self._latest_rgb = None
        self._latest_depth = None
        self._camera_info = None
        self._latest_detections = None

        self.create_subscription(
            Image, '/camera/color/image_raw', self._rgb_cb,
            rclpy.qos.QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            Image, '/camera/depth/image_raw', self._depth_cb,
            rclpy.qos.QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._info_cb, 10)
        self.create_subscription(
            Detection2DArray, '/objbbox', self._detections_cb, 10)

        # ── RViz debug publisher ──────────────────────────────────────────
        self._detection_marker_pub = self.create_publisher(
            Marker, '/grasp_debug/detection', 10)

        # ── Announce ─────────────────────────────────────────────────────
        self.get_logger().info('=' * 55)
        self.get_logger().info('GraspNet Grasp Demo')
        self.get_logger().info('=' * 55)
        self.get_logger().info(f'  Target object : {self._object}')
        self.get_logger().info(f'  Arm           : {self._arm}')
        self.get_logger().info(f'  Sim mode      : {self._sim}')
        self.get_logger().info(f'  Use GraspNet  : {self._use_graspnet}')
        self.get_logger().info('')

    # ── Subscriber callbacks ─────────────────────────────────────────────────

    def _rgb_cb(self, msg):
        self._latest_rgb = self._bridge.imgmsg_to_cv2(msg, 'rgb8')

    def _depth_cb(self, msg):
        self._latest_depth = self._bridge.imgmsg_to_cv2(msg, '16UC1')

    def _info_cb(self, msg):
        self._camera_info = msg

    def _detections_cb(self, msg):
        self._latest_detections = msg

    # ── Helpers ────────────────────────────────────────────────────────────

    def _spin(self, duration: float) -> None:
        """Pump callbacks for the given duration."""
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _send_pan_tilt(self, pan: float, tilt: float) -> None:
        msg = Float64MultiArray()
        msg.data = [pan, tilt]
        self._pan_tilt_pub.publish(msg)

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
        req.object_class = self._object
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
        pub = (
            self._right_gripper_pub
            if self._arm == 'right'
            else self._left_gripper_pub
        )
        if self._sim:
            msg = Float64MultiArray()
            msg.data = [position]
        else:
            msg = GripperCommand()
            msg.position = position
            msg.effort   = effort
        pub.publish(msg)
        self.get_logger().info(f'Gripper → {position:.2f}')
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
        self._set_gripper(GRIPPER_CLOSED, effort=1.0)

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
        grasp.position.z  = obj_base.point.z + 0.06   # above object surface
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

    # ── Object detection (via external classifier node) ───────────────────────

    def _detect_object(self) -> tuple | None:
        """
        Point camera at the table, wait for detection from classifier node,
        and return (pixel_cx, pixel_cy, depth_image, camera_info) or None.
        """
        # Wait for camera data
        self._spin(3.0)
        if self._camera_info is None:
            self.get_logger().error('Camera not online after 3 s')
            return None

        # Point camera at table (pan=0, tilt=0.6 rad ≈ 34° down)
        TILT_RAD = 0.6
        self._send_pan_tilt(0.0, TILT_RAD)
        self.get_logger().info(f'Camera → pan=0.0, tilt={TILT_RAD} rad')

        # Clear stale detections and wait for fresh ones
        self._latest_detections = None
        self._spin(2.0)

        # Poll for detection of target object
        target_id = YOLO_CLASS_IDS.get(self._object)
        if target_id is None:
            self.get_logger().error(
                f'Unknown object {self._object!r} — '
                f'known classes: {list(YOLO_CLASS_IDS.keys())}')
            return None

        deadline = time.time() + 5.0
        best_cx, best_cy, best_conf = None, None, 0.0
        while time.time() < deadline:
            self._spin(0.2)
            dets = self._latest_detections
            if dets is None:
                continue
            for det in dets.detections:
                for hyp in det.results:
                    if (hyp.hypothesis.class_id == target_id
                            and hyp.hypothesis.score > best_conf):
                        best_cx = int(det.bbox.center.position.x)
                        best_cy = int(det.bbox.center.position.y)
                        best_conf = hyp.hypothesis.score
            if best_cx is not None:
                break

        if best_cx is None:
            self.get_logger().warn(
                f'Classifier did not detect {self._object!r} — '
                'ensure classifier node is running '
                '(ros2 run object_classification classifier)')
            return None

        self.get_logger().info(
            f'Detection: {self._object} conf={best_conf:.2f} '
            f'centroid=({best_cx},{best_cy})')

        depth = self._latest_depth
        ci    = self._camera_info
        if depth is None or ci is None:
            self.get_logger().error('Missing depth/camera_info')
            return None

        # Save GraspNet-compatible capture to /tmp/grasp_capture/
        import cv2 as _cv2
        capture_dir = '/tmp/grasp_capture'
        os.makedirs(capture_dir, exist_ok=True)
        if self._latest_rgb is not None:
            _cv2.imwrite(f'{capture_dir}/color.png',
                         _cv2.cvtColor(self._latest_rgb, _cv2.COLOR_RGB2BGR))
        _cv2.imwrite(f'{capture_dir}/depth.png', depth)
        _cv2.imwrite(f'{capture_dir}/workspace_mask.png',
                     np.ones(depth.shape[:2], dtype=np.uint8) * 255)
        import scipy.io as _sio
        K = np.array([[ci.k[0], ci.k[1], ci.k[2]],
                      [ci.k[3], ci.k[4], ci.k[5]],
                      [ci.k[6], ci.k[7], ci.k[8]]])
        _sio.savemat(f'{capture_dir}/meta.mat', {
            'intrinsic_matrix': K,
            'factor_depth': np.array([[1000.0]]),
        })
        self.get_logger().info(f'GraspNet capture saved to {capture_dir}/')

        return (best_cx, best_cy, depth.copy(), ci)

    # ── Main pipeline ─────────────────────────────────────────────────────────

    def run(self) -> None:
        """Execute the full grasp pipeline (blocks until done)."""

        # ── 1. Detect target object via classifier ─────────────────────────
        self.get_logger().info(
            f'Waiting for {self._object!r} detection from classifier ...')
        result = self._detect_object()
        if result is None:
            self.get_logger().error(
                f'No {self._object!r} found — '
                'check that the scene is loaded, camera is working, '
                'and classifier node is running')
            return

        cx, cy, depth, ci = result

        # ── 2. Back-project detection pixel → 3D point in camera frame ───
        if self._sim:
            # Sim: classifier ran on raw RGB, use camera_info (RGB) intrinsics
            fx, fy    = ci.k[0], ci.k[4]
            cx_px, cy_px = ci.k[2], ci.k[5]
            cam_frame = 'camera_color_optical_frame'
        else:
            # Real: classifier ran on RGB, but depth is in depth frame
            # Use D435 depth intrinsics for back-projection
            fx, fy, cx_px, cy_px = D435_DEPTH_K
            cam_frame = 'camera_depth_optical_frame'

        # Sample a small patch around centroid for a stable depth reading
        u, v = cx, cy
        patch = depth[
            max(0, v - 5) : v + 6,
            max(0, u - 5) : u + 6,
        ]
        valid = patch[(patch > 100) & (patch < 5000)]  # 0.1 – 5 m in mm
        if len(valid) == 0:
            self.get_logger().error(
                'No valid depth near detected centroid — '
                'ensure depth camera is working and object is in range (0.1–5 m)')
            return

        depth_m = float(np.median(valid)) / 1000.0       # mm → metres
        xyz_cam = CoordConverter.depth_pixel_to_camera_point(
            u, v, depth_m, fx, fy, cx_px, cy_px
        )
        self.get_logger().info(
            f'Object in {cam_frame}: '
            f'({xyz_cam[0]:.3f}, {xyz_cam[1]:.3f}, {xyz_cam[2]:.3f}) m  '
            f'[fx={fx:.1f}, fy={fy:.1f}]')

        obj_cam = PointStamped()
        obj_cam.header.frame_id = cam_frame
        obj_cam.header.stamp    = self.get_clock().now().to_msg()
        obj_cam.point.x = float(xyz_cam[0])
        obj_cam.point.y = float(xyz_cam[1])
        obj_cam.point.z = float(xyz_cam[2])

        # ── Publish detection marker to RViz ──────────────────────────────
        m = Marker()
        m.header = obj_cam.header
        m.ns = 'detection'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = obj_cam.point
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.04
        m.color = ColorRGBA(r=0.2, g=1.0, b=0.2, a=0.9)
        m.lifetime.sec = 30
        self._detection_marker_pub.publish(m)

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
                    f'GraspNet failed ({msg}) — falling back to top-down grasp')

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
