#!/usr/bin/env python3
"""
GPD Grasp Demo — 6-DOF Grasp Detection with GPD + YOLO

Detects objects using the external YOLO classifier node (/objbbox),
converts depth camera data to a point cloud, sends it to the GPD
service (/detect_grasps) for 6-DOF grasp detection, then executes
the best grasp using the motion planner (/plan_to_target).

Requires:
    - ros2 launch tidybot_bringup sim.launch.py scene:=scene_fruit_grasp.xml use_gpd:=true
    - ros2 run object_classification classifier

Usage:
    # Default: grasp a banana with right arm
    ros2 run tidybot_bringup gpd_grasp_demo.py

    # Target a specific object/arm:
    ros2 run tidybot_bringup gpd_grasp_demo.py --ros-args \
        -p target:=apple -p arm:=right
"""

import json
import os
import struct
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import tf2_ros

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from tidybot_msgs.msg import ArmCommand
from tidybot_msgs.srv import PlanToTarget
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation

# GPD service
from gpd_ros2.srv import DetectGrasps

# YOLO COCO class name -> string ID
YOLO_CLASS_IDS = {
    'person': '0',
    'banana': '46',
    'apple': '47',
    'orange': '49',
}

# Grasp geometry
PRE_GRASP_OFFSET = 0.10   # back off along approach direction
LIFT_HEIGHT = 0.15

# Timing constants for grasp sequencing
SETTLE_TIME = 1.0            # extra wait after arm reaches target (seconds)
GRIPPER_CLOSE_REPEATS = 20   # number of times to publish gripper close command
GRIPPER_CLOSE_WAIT = 2.0     # wait for gripper to exert full effort (seconds)


class GpdGraspDemo(Node):
    """Detect via YOLO, plan 6-DOF grasp via GPD, execute pick-and-place."""

    def __init__(self):
        super().__init__('gpd_grasp_demo')

        # Parameters
        self.declare_parameter('target', 'banana')
        self.declare_parameter('arm', 'right')
        self.declare_parameter('duration', 2.0)
        self.declare_parameter('crop_radius', 0.08)
        self.declare_parameter('approach_offset', 0.10)
        self.declare_parameter('save_debug', False)
        self.target_object = self.get_parameter('target').value
        self.arm_name = self.get_parameter('arm').value
        self.move_duration = self.get_parameter('duration').value
        self.crop_radius = self.get_parameter('crop_radius').value
        self.approach_offset = self.get_parameter('approach_offset').value
        self.save_debug = self.get_parameter('save_debug').value

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # CV bridge
        self._bridge = CvBridge()

        # Camera state
        self.latest_depth = None
        self.latest_rgb = None
        self.camera_info = None
        self.latest_detections = None

        # Subscribers
        qos_be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Detection2DArray, '/objbbox', self._detections_cb, 10)
        self.create_subscription(
            Image, '/camera/depth/image_raw', self._depth_cb, qos_be)
        self.create_subscription(
            Image, '/camera/color/image_raw', self._rgb_cb, qos_be)
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._info_cb, 10)

        # Debug capture state
        self._debug_session_dir = None
        self._debug_capture_index = 0
        self._debug_capture = {}
        if self.save_debug:
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            self._debug_session_dir = os.path.join(
                'gpd_debug_captures', f'session_{ts}')
            os.makedirs(self._debug_session_dir, exist_ok=True)
            self.get_logger().info(
                f'Debug capture enabled → {self._debug_session_dir}')

        # Publishers
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, f'/{self.arm_name}_gripper/cmd', 10)
        self.arm_pub = self.create_publisher(
            ArmCommand, f'/{self.arm_name}_arm/cmd', 10)
        self.marker_pub = self.create_publisher(Marker, '/grasp_debug_markers', 10)
        self.marker_id = 0

        # Service clients
        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')
        self.gpd_client = self.create_client(DetectGrasps, '/detect_grasps')

    # ── Callbacks ─────────────────────────────────────────────────────

    def _detections_cb(self, msg):
        self.latest_detections = msg

    def _depth_cb(self, msg):
        self.latest_depth = self._bridge.imgmsg_to_cv2(msg, '16UC1')

    def _rgb_cb(self, msg):
        self.latest_rgb = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _info_cb(self, msg):
        self.camera_info = msg

    # ── Utilities ─────────────────────────────────────────────────────

    def spin_for(self, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def set_gripper(self, position):
        msg = Float64MultiArray()
        msg.data = [position]
        for _ in range(10):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(0.5)

    def close_gripper_firm(self):
        """Close gripper with extended effort for a firm grasp."""
        self.get_logger().info('  Closing gripper (firm)')
        msg = Float64MultiArray()
        msg.data = [1.0]
        for _ in range(GRIPPER_CLOSE_REPEATS):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        # Wait for gripper to exert full effort on the object
        time.sleep(GRIPPER_CLOSE_WAIT)
        # Re-publish to maintain grip pressure
        for _ in range(10):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def go_home(self, duration=3.0):
        self.get_logger().info(f'Returning arm to home over {duration}s...')
        msg = ArmCommand()
        msg.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.duration = duration
        self.arm_pub.publish(msg)
        time.sleep(duration + 0.5)

    def publish_marker(self, x, y, z, r, g, b, label='', scale=0.03):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'grasp_debug'
        m.id = self.marker_id
        self.marker_id += 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.pose.orientation.w = 1.0
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        m.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=0.8)
        m.lifetime.sec = 60
        if label:
            m.text = label
        self.marker_pub.publish(m)

    # ── YOLO detection ────────────────────────────────────────────────

    def detect(self, target_class):
        """Find the best YOLO detection matching target_class."""
        if self.latest_detections is None:
            return None
        target_id = YOLO_CLASS_IDS.get(target_class.lower())
        if target_id is None:
            self.get_logger().warn(
                f'Unknown target class "{target_class}". '
                f'Available: {list(YOLO_CLASS_IDS.keys())}')
            return None

        best, best_conf = None, 0.0
        for det in self.latest_detections.detections:
            for hyp in det.results:
                if (hyp.hypothesis.class_id == target_id
                        and hyp.hypothesis.score > best_conf):
                    best = det
                    best_conf = hyp.hypothesis.score
        return best

    def pixel_to_base_link(self, u, v):
        """Back-project a depth pixel to 3D in base_link frame."""
        if self.latest_depth is None or self.camera_info is None:
            return None

        K = self.camera_info.k
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]

        # Sample 11x11 patch and take median
        h, w = self.latest_depth.shape
        half = 5
        u0, u1 = max(0, u - half), min(w, u + half + 1)
        v0, v1 = max(0, v - half), min(h, v + half + 1)
        patch = self.latest_depth[v0:v1, u0:u1].astype(np.float32)
        valid = patch[(patch > 100) & (patch < 5000)]
        if len(valid) == 0:
            return None
        z_mm = float(np.median(valid))
        z_m = z_mm / 1000.0

        # Back-project to camera frame
        x_cam = (u - cx) * z_m / fx
        y_cam = (v - cy) * z_m / fy
        z_cam = z_m

        # Transform to base_link
        tf_mat = self._get_tf_matrix('base_link', 'camera_color_optical_frame')
        if tf_mat is None:
            return None
        pt_cam = np.array([x_cam, y_cam, z_cam, 1.0], dtype=np.float32)
        pt_base = (tf_mat @ pt_cam)[:3]

        # Clamp z to floor
        pt_base[2] = max(pt_base[2], 0.005)
        return pt_base

    def _get_tf_matrix(self, target_frame, source_frame):
        try:
            import rclpy.time
            import rclpy.duration
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
            t = transform.transform.translation
            q = transform.transform.rotation
            rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
            mat = np.eye(4, dtype=np.float32)
            mat[:3, :3] = rot.as_matrix().astype(np.float32)
            mat[:3, 3] = [t.x, t.y, t.z]
            return mat
        except Exception as e:
            self.get_logger().error(f'TF lookup failed: {e}')
            return None

    # ── Point cloud generation ────────────────────────────────────────

    def depth_to_pointcloud(self, crop_center=None, crop_radius=0.08):
        """Convert depth image to numpy points in base_link frame.

        Args:
            crop_center: (x, y, z) in base_link to crop around, or None for full cloud
            crop_radius: radius in meters for cropping

        Returns:
            numpy array of shape (N, 3) in base_link frame
        """
        if self.latest_depth is None or self.camera_info is None:
            return None

        K = self.camera_info.k
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]

        depth = self.latest_depth.astype(np.float32) / 1000.0  # mm -> m
        h, w = depth.shape

        u_grid, v_grid = np.meshgrid(np.arange(w), np.arange(h))
        mask = (depth > 0.1) & (depth < 2.0)

        z = depth[mask]
        u = u_grid[mask].astype(np.float32)
        v = v_grid[mask].astype(np.float32)

        x_cam = (u - cx) * z / fx
        y_cam = (v - cy) * z / fy

        points_cam = np.stack([x_cam, y_cam, z], axis=1)  # (N, 3)

        # Transform to base_link
        tf_mat = self._get_tf_matrix('base_link', 'camera_color_optical_frame')
        if tf_mat is None:
            return None

        ones = np.ones((len(points_cam), 1), dtype=np.float32)
        pts_h = np.hstack([points_cam, ones])
        pts_base = (tf_mat @ pts_h.T).T[:, :3]

        # Crop around object if specified
        if crop_center is not None:
            center = np.array(crop_center, dtype=np.float32)
            dists = np.linalg.norm(pts_base - center, axis=1)
            pts_base = pts_base[dists < crop_radius]

        return pts_base

    def numpy_to_pointcloud2(self, points, frame_id='base_link'):
        """Convert numpy (N, 3) array to sensor_msgs/PointCloud2."""
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12  # 3 * 4 bytes
        msg.row_step = msg.point_step * msg.width

        # Pack points as binary float32
        msg.data = points.astype(np.float32).tobytes()
        return msg

    # ── GPD grasp conversion ──────────────────────────────────────────

    @staticmethod
    def gpd_grasp_to_pose(grasp):
        """Convert GPD GraspConfig to geometry_msgs/Pose.

        GPD frame: columns of rotation matrix are [approach, binormal, axis].
        """
        approach = np.array([grasp.approach.x, grasp.approach.y, grasp.approach.z])
        binormal = np.array([grasp.binormal.x, grasp.binormal.y, grasp.binormal.z])
        axis = np.array([grasp.axis.x, grasp.axis.y, grasp.axis.z])

        # Build rotation matrix from GPD frame vectors
        R = np.column_stack([approach, binormal, axis])

        # Ensure proper rotation matrix
        U, _, Vt = np.linalg.svd(R)
        R = U @ Vt
        if np.linalg.det(R) < 0:
            R[:, 2] *= -1

        rot = Rotation.from_matrix(R)
        qx, qy, qz, qw = rot.as_quat()

        pose = Pose()
        pose.position.x = float(grasp.position.x)
        pose.position.y = float(grasp.position.y)
        pose.position.z = float(grasp.position.z)
        pose.orientation.w = float(qw)
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        return pose

    @staticmethod
    def compute_pre_grasp_pose(grasp_pose, grasp_config, offset=0.10):
        """Compute pre-grasp by backing off along approach direction."""
        approach = np.array([
            grasp_config.approach.x,
            grasp_config.approach.y,
            grasp_config.approach.z,
        ])

        pre = Pose()
        pre.position.x = grasp_pose.position.x - offset * float(approach[0])
        pre.position.y = grasp_pose.position.y - offset * float(approach[1])
        pre.position.z = grasp_pose.position.z - offset * float(approach[2])
        pre.orientation = grasp_pose.orientation
        return pre

    # ── Motion planning ───────────────────────────────────────────────

    def plan_and_execute(self, pose, label, use_orientation=True):
        req = PlanToTarget.Request()
        req.arm_name = self.arm_name
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = True
        req.duration = self.move_duration
        req.max_condition_number = 100.0

        self.get_logger().info(
            f'  [{label}] pos=({pose.position.x:.3f}, {pose.position.y:.3f}, '
            f'{pose.position.z:.3f})')

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        if not future.done() or future.exception():
            self.get_logger().error(f'  [{label}] service call failed')
            return False

        result = future.result()
        if result.success:
            self.get_logger().info(
                f'  [{label}] OK  pos_err={result.position_error:.4f}m')
            if result.executed:
                time.sleep(self.move_duration + 0.5)
            return True
        else:
            self.get_logger().warn(f'  [{label}] FAILED: {result.message}')
            return False

    # ── Debug capture helpers ─────────────────────────────────────────

    def _debug_find_grasp(self, g):
        """Find debug grasp entry by matching position."""
        if not self.save_debug:
            return None
        for dg in self._debug_capture.get('grasps', []):
            if (abs(dg['position'][0] - g.position.x) < 1e-6 and
                    abs(dg['position'][1] - g.position.y) < 1e-6 and
                    abs(dg['position'][2] - g.position.z) < 1e-6):
                return dg
        return None

    def _save_debug_capture(self):
        """Save accumulated debug data to disk."""
        if not self.save_debug or not self._debug_capture:
            return

        capture_dir = os.path.join(
            self._debug_session_dir,
            f'capture_{self._debug_capture_index:04d}')
        os.makedirs(capture_dir, exist_ok=True)

        # Save RGB image
        if self.latest_rgb is not None:
            cv2.imwrite(os.path.join(capture_dir, 'rgb.png'), self.latest_rgb)

        # Save depth
        if self.latest_depth is not None:
            np.save(os.path.join(capture_dir, 'depth.npy'), self.latest_depth)

        # Save point clouds
        if 'point_cloud' in self._debug_capture:
            np.save(os.path.join(capture_dir, 'point_cloud.npy'),
                    self._debug_capture['point_cloud'])
        if 'point_cloud_full' in self._debug_capture:
            np.save(os.path.join(capture_dir, 'point_cloud_full.npy'),
                    self._debug_capture['point_cloud_full'])

        # Save metadata
        meta = {
            'target': self.target_object,
            'arm': self.arm_name,
            'crop_radius': self.crop_radius,
            'approach_offset': self.approach_offset,
        }
        for key in ['object_pos', 'intrinsics', 'major_axis', 'workspace',
                     'view_point']:
            if key in self._debug_capture:
                meta[key] = self._debug_capture[key]
        with open(os.path.join(capture_dir, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=2)

        # Save grasps with filter annotations
        if 'grasps' in self._debug_capture:
            with open(os.path.join(capture_dir, 'grasps.json'), 'w') as f:
                json.dump(self._debug_capture['grasps'], f, indent=2)

        self.get_logger().info(f'Debug data saved to {capture_dir}')
        self._debug_capture_index += 1
        self._debug_capture = {}

    # ── Main pipeline ─────────────────────────────────────────────────

    def _detect_object(self):
        """Detect target via YOLO and return (3D pos, bbox_info) in base_link."""
        self.spin_for(1.0)
        self.get_logger().info(f'Detecting "{self.target_object}" via YOLO classifier')

        for _ in range(20):
            det = self.detect(self.target_object)
            if det is not None:
                break
            self.spin_for(0.2)

        if det is None:
            self.get_logger().error(
                f'Could not find "{self.target_object}" in camera view!')
            return None, None

        u = int(det.bbox.center.position.x)
        v = int(det.bbox.center.position.y)
        bbox_w = float(det.bbox.size_x)
        bbox_h = float(det.bbox.size_y)
        self.get_logger().info(
            f'  Detected at pixel ({u}, {v}), bbox {bbox_w:.0f}x{bbox_h:.0f}')

        pt_base = self.pixel_to_base_link(u, v)
        if pt_base is None:
            self.get_logger().error('  Depth back-projection failed')
            return None, None

        ox, oy, oz = float(pt_base[0]), float(pt_base[1]), float(pt_base[2])
        self.get_logger().info(
            f'  Object in base_link: ({ox:.3f}, {oy:.3f}, {oz:.3f})')

        # Compute object major axis direction in base_link using bbox corners
        major_axis_base = None
        aspect = max(bbox_w, bbox_h) / max(min(bbox_w, bbox_h), 1.0)
        if aspect > 1.5:
            # Object is elongated — determine major axis in base_link
            # Back-project two points along the longer bbox dimension
            if bbox_w > bbox_h:
                # Wider than tall — major axis is horizontal in image
                p1 = self.pixel_to_base_link(int(u - bbox_w / 2), v)
                p2 = self.pixel_to_base_link(int(u + bbox_w / 2), v)
            else:
                # Taller than wide — major axis is vertical in image
                p1 = self.pixel_to_base_link(u, int(v - bbox_h / 2))
                p2 = self.pixel_to_base_link(u, int(v + bbox_h / 2))

            if p1 is not None and p2 is not None:
                axis = p2 - p1
                axis[2] = 0.0  # project to horizontal plane
                norm = np.linalg.norm(axis)
                if norm > 0.001:
                    major_axis_base = axis / norm
                    self.get_logger().info(
                        f'  Elongated object (aspect {aspect:.1f}), '
                        f'major axis in base_link: [{major_axis_base[0]:.2f}, '
                        f'{major_axis_base[1]:.2f}]')

        return (ox, oy, oz), major_axis_base

    def _call_gpd(self, object_pos):
        """Generate point cloud, call GPD, return list of GraspConfig."""
        ox, oy, oz = object_pos

        # Generate cropped point cloud
        self.get_logger().info('  Generating point cloud from depth...')
        points = self.depth_to_pointcloud(
            crop_center=(ox, oy, oz),
            crop_radius=self.crop_radius)

        if points is None or len(points) < 10:
            self.get_logger().error(
                f'  Insufficient points in crop region ({0 if points is None else len(points)} points)')
            return None

        self.get_logger().info(f'  Cropped point cloud: {len(points)} points')

        # Save debug: point clouds
        if self.save_debug:
            full_points = self.depth_to_pointcloud(crop_center=None)
            if full_points is not None and len(full_points) > 50000:
                full_points = full_points[::len(full_points) // 50000]
            self._debug_capture['point_cloud'] = points
            self._debug_capture['point_cloud_full'] = full_points
            self._debug_capture['object_pos'] = [ox, oy, oz]
            K = self.camera_info.k
            self._debug_capture['intrinsics'] = {
                'fx': float(K[0]), 'fy': float(K[4]),
                'cx': float(K[2]), 'cy': float(K[5]),
                'width': int(self.latest_depth.shape[1]),
                'height': int(self.latest_depth.shape[0]),
            }

        cloud_msg = self.numpy_to_pointcloud2(points, 'base_link')

        # Get camera position in base_link
        tf_mat = self._get_tf_matrix('base_link', 'camera_color_optical_frame')
        if tf_mat is None:
            return None
        cam_pos = tf_mat[:3, 3]

        # Build GPD request
        req = DetectGrasps.Request()
        req.cloud = cloud_msg
        req.view_point = Point(
            x=float(cam_pos[0]),
            y=float(cam_pos[1]),
            z=float(cam_pos[2]))

        # Workspace around the object
        margin = self.crop_radius
        req.workspace = [
            ox - margin, ox + margin,
            oy - margin, oy + margin,
            max(0.0, oz - margin), oz + margin,
        ]

        # Save debug: GPD request params
        if self.save_debug:
            self._debug_capture['view_point'] = [
                float(cam_pos[0]), float(cam_pos[1]), float(cam_pos[2])]
            self._debug_capture['workspace'] = list(req.workspace)

        # Call GPD service
        self.get_logger().info('  Calling GPD /detect_grasps service...')
        future = self.gpd_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if not future.done() or future.exception():
            self.get_logger().error('  GPD service call failed or timed out')
            return None

        result = future.result()
        if not result.success:
            self.get_logger().warn(f'  GPD: {result.message}')
            return None

        grasps = result.grasps.grasps
        self.get_logger().info(f'  GPD found {len(grasps)} grasps')

        # Log all grasps with approach direction
        for i, g in enumerate(grasps):
            self.get_logger().info(
                f'    Grasp {i}: pos=({g.position.x:.3f}, {g.position.y:.3f}, '
                f'{g.position.z:.3f}) approach=({g.approach.x:.2f}, {g.approach.y:.2f}, '
                f'{g.approach.z:.2f}) score={g.score:.3f} width={g.width*1000:.0f}mm')

        # Save debug: serialize all GPD grasps
        if self.save_debug:
            self._debug_capture['grasps'] = []
            for i, g in enumerate(grasps):
                self._debug_capture['grasps'].append({
                    'index': i,
                    'position': [float(g.position.x), float(g.position.y),
                                 float(g.position.z)],
                    'approach': [float(g.approach.x), float(g.approach.y),
                                 float(g.approach.z)],
                    'binormal': [float(g.binormal.x), float(g.binormal.y),
                                 float(g.binormal.z)],
                    'axis': [float(g.axis.x), float(g.axis.y),
                             float(g.axis.z)],
                    'width': float(g.width),
                    'score': float(g.score),
                    'sample': [float(g.sample.x), float(g.sample.y),
                               float(g.sample.z)],
                    'filter_z': None, 'filter_cone': None,
                    'filter_orient': None,
                    'ik_6dof': None, 'ik_topdown': None,
                    'ik_posonly': None, 'selected': False,
                })

        return grasps

    def _filter_grasps(self, grasps, object_pos):
        """Filter GPD grasps for feasibility.

        Removes grasps that are:
        - Below the table (z < 0.01)
        - Approach direction not within 45deg of straight down (approach_z > -0.707)
        Sorts remaining by score (highest first).
        """
        ox, oy, oz = object_pos
        filtered = []
        for i, g in enumerate(grasps):
            dg = self._debug_capture['grasps'][i] if self.save_debug else None

            # Skip grasps at/below table
            if g.position.z < 0.01:
                if dg:
                    dg['filter_z'] = {'pass': False, 'value': float(g.position.z)}
                self.get_logger().info(
                    f'    Grasp {i}: SKIP z={g.position.z:.3f} < 0.01 (below table)')
                continue
            if dg:
                dg['filter_z'] = {'pass': True, 'value': float(g.position.z)}

            # Skip grasps not within 45deg cone of straight down
            approach_z = g.approach.z
            if approach_z > -0.707:
                if dg:
                    dg['filter_cone'] = {'pass': False,
                                         'approach_z': float(approach_z)}
                self.get_logger().info(
                    f'    Grasp {i}: SKIP approach_z={approach_z:.2f} > -0.707 '
                    f'(not within 45deg of vertical)')
                continue
            if dg:
                dg['filter_cone'] = {'pass': True,
                                     'approach_z': float(approach_z)}

            filtered.append(g)
        self.get_logger().info(
            f'  Filtered: {len(filtered)}/{len(grasps)} grasps pass feasibility check')
        return filtered

    def _try_ik(self, pose, use_orientation=True):
        """Dry-run IK check. Returns (success, result_message)."""
        req = PlanToTarget.Request()
        req.arm_name = self.arm_name
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = False
        req.duration = self.move_duration
        req.max_condition_number = 200.0

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done() or future.exception():
            return False, 'service call failed'

        result = future.result()
        return result.success, result.message

    @staticmethod
    def _compute_top_down_quat(major_axis=None):
        """Compute a top-down gripper quaternion, optionally perpendicular to major_axis.

        Default (no major_axis): fingers-down along base_link convention.
        With major_axis: rotates the gripper so fingers open perpendicular to
        the object's long axis (grabs across the narrow dimension).
        """
        if major_axis is None:
            return Quaternion(w=0.5, x=0.5, y=0.5, z=-0.5)

        # Build rotation: Z points down, Y (finger opening) perpendicular to major_axis
        z_axis = np.array([0.0, 0.0, -1.0])  # approach: straight down
        # Gripper Y axis (opening direction) should be perpendicular to the object's major axis
        # Project major_axis to horizontal, then rotate 90 degrees
        perp = np.array([-major_axis[1], major_axis[0], 0.0])
        perp_norm = np.linalg.norm(perp)
        if perp_norm < 0.001:
            return Quaternion(w=0.5, x=0.5, y=0.5, z=-0.5)
        y_axis = perp / perp_norm
        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)

        R = np.column_stack([x_axis, y_axis, z_axis])
        rot = Rotation.from_matrix(R)
        qx, qy, qz, qw = rot.as_quat()
        return Quaternion(w=float(qw), x=float(qx), y=float(qy), z=float(qz))

    def _select_and_execute_grasp(self, grasps, object_pos, major_axis=None):
        """Try GPD grasps in score order until one succeeds IK.

        Strategy:
        1. Filter out infeasible grasps (below table, non-vertical approach)
        2. For elongated objects, prefer grasps that grab across the narrow dimension
        3. Try each grasp with full 6-DOF orientation, then top-down fallback
        """
        ox, oy, oz = object_pos

        # Filter grasps
        grasps = self._filter_grasps(grasps, object_pos)
        if not grasps:
            self.get_logger().error('  No feasible grasps after filtering')
            return None

        # For elongated objects, filter out grasps that try to grab along the long axis
        if major_axis is not None:
            pre_count = len(grasps)
            good_grasps = []
            for g in grasps:
                dg = self._debug_find_grasp(g)
                # GPD axis = finger opening direction (projected to horizontal)
                grip_axis = np.array([g.axis.x, g.axis.y, 0.0])
                grip_norm = np.linalg.norm(grip_axis)
                if grip_norm > 0.001:
                    grip_axis = grip_axis / grip_norm
                    # Dot product: 1.0 = parallel (bad), 0.0 = perpendicular (good)
                    alignment = abs(np.dot(grip_axis[:2], major_axis[:2]))
                    if alignment > 0.7:  # within ~45 deg of long axis
                        if dg:
                            dg['filter_orient'] = {
                                'pass': False, 'alignment': float(alignment)}
                        self.get_logger().info(
                            f'    SKIP grasp (alignment={alignment:.2f} with long axis)')
                        continue
                    if dg:
                        dg['filter_orient'] = {
                            'pass': True, 'alignment': float(alignment)}
                else:
                    if dg:
                        dg['filter_orient'] = {'pass': True, 'alignment': 0.0}
                good_grasps.append(g)
            if good_grasps:
                self.get_logger().info(
                    f'  Orientation filter: {len(good_grasps)}/{pre_count} '
                    f'grasps grab across narrow dimension')
                grasps = good_grasps
            else:
                self.get_logger().warn(
                    '  All grasps aligned with long axis — keeping all')
        elif self.save_debug:
            # No major axis — mark all as orient-passed
            for g in grasps:
                dg = self._debug_find_grasp(g)
                if dg:
                    dg['filter_orient'] = {'pass': True, 'alignment': None}

        # Compute top-down quaternion (oriented for elongated objects if applicable)
        top_down_quat = self._compute_top_down_quat(major_axis)

        for i, grasp in enumerate(grasps):
            self.get_logger().info(
                f'  Trying grasp {i} (score={grasp.score:.3f}, '
                f'approach=[{grasp.approach.x:.2f}, {grasp.approach.y:.2f}, {grasp.approach.z:.2f}])...')

            grasp_pose = self.gpd_grasp_to_pose(grasp)
            pre_grasp_pose = self.compute_pre_grasp_pose(
                grasp_pose, grasp, self.approach_offset)

            dg = self._debug_find_grasp(grasp)

            # Try 1: Full 6-DOF IK with GPD orientation
            ok, msg = self._try_ik(grasp_pose, use_orientation=True)
            use_orientation = True
            if dg:
                dg['ik_6dof'] = {'pass': ok, 'msg': msg}
            if not ok:
                self.get_logger().info(f'    6-DOF IK failed: {msg}')
                # Try 2: Top-down orientation at GPD grasp position
                fallback_pose = Pose()
                fallback_pose.position = grasp_pose.position
                fallback_pose.orientation = top_down_quat
                ok, msg = self._try_ik(fallback_pose, use_orientation=True)
                if dg:
                    dg['ik_topdown'] = {'pass': ok, 'msg': msg}
                if ok:
                    self.get_logger().info(f'    Fallback top-down IK OK!')
                    grasp_pose = fallback_pose
                    pre_grasp_pose = Pose()
                    pre_grasp_pose.position.x = grasp_pose.position.x
                    pre_grasp_pose.position.y = grasp_pose.position.y
                    pre_grasp_pose.position.z = grasp_pose.position.z + self.approach_offset
                    pre_grasp_pose.orientation = top_down_quat
                else:
                    self.get_logger().info(f'    Top-down fallback also failed: {msg}')
                    # Try 3: Position-only (no orientation constraint)
                    ok, msg = self._try_ik(grasp_pose, use_orientation=False)
                    if dg:
                        dg['ik_posonly'] = {'pass': ok, 'msg': msg}
                    if ok:
                        self.get_logger().info(f'    Position-only IK OK!')
                        use_orientation = False
                        pre_grasp_pose = Pose()
                        pre_grasp_pose.position.x = grasp_pose.position.x
                        pre_grasp_pose.position.y = grasp_pose.position.y
                        pre_grasp_pose.position.z = grasp_pose.position.z + self.approach_offset
                        pre_grasp_pose.orientation = grasp_pose.orientation
                    else:
                        self.get_logger().info(f'    Position-only also failed: {msg}')
                        continue

            if dg:
                dg['selected'] = True
            self.get_logger().info(f'    Grasp {i} IK OK! Executing...')

            # Debug markers
            gp = grasp_pose.position
            pp = pre_grasp_pose.position
            self.publish_marker(ox, oy, oz, 0.0, 1.0, 0.0,
                                label='object', scale=0.02)
            self.publish_marker(gp.x, gp.y, gp.z, 1.0, 0.0, 0.0,
                                label='grasp', scale=0.02)
            self.publish_marker(pp.x, pp.y, pp.z, 0.0, 0.0, 1.0,
                                label='pre_grasp', scale=0.02)

            # Execute: open -> pre-grasp -> settle -> grasp -> settle -> close firmly
            self.get_logger().info('  Opening gripper')
            self.set_gripper(0.0)

            self.get_logger().info('  Moving to pre-grasp')
            if not self.plan_and_execute(pre_grasp_pose, 'pre-grasp',
                                         use_orientation=use_orientation):
                self.get_logger().error('  Pre-grasp failed, trying next grasp')
                continue

            self.get_logger().info('  Settling at pre-grasp...')
            time.sleep(SETTLE_TIME)

            self.get_logger().info('  Descending to grasp')
            if not self.plan_and_execute(grasp_pose, 'grasp',
                                         use_orientation=use_orientation):
                self.get_logger().error('  Grasp motion failed')
                continue

            self.get_logger().info('  Settling at grasp position...')
            time.sleep(SETTLE_TIME)

            self.close_gripper_firm()

            return grasp_pose

        self.get_logger().error('  All GPD grasps failed IK or execution')
        return None

    def run(self):
        if self.save_debug:
            self._debug_capture = {}

        self.get_logger().info('=' * 50)
        self.get_logger().info('GPD Grasp Demo (6-DOF)')
        self.get_logger().info(f'  Target       : {self.target_object}')
        self.get_logger().info(f'  Arm          : {self.arm_name}')
        self.get_logger().info(f'  Crop radius  : {self.crop_radius}m')
        self.get_logger().info(f'  Save debug   : {self.save_debug}')
        self.get_logger().info('=' * 50)

        # Wait for services
        self.get_logger().info('Waiting for /plan_to_target service...')
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('  ...waiting')

        self.get_logger().info('Waiting for /detect_grasps service (GPD)...')
        while not self.gpd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('  ...waiting')

        self.get_logger().info('All services available.')

        # Wait for YOLO detections
        self.get_logger().info('Waiting for YOLO classifier (/objbbox)...')
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_detections is not None:
                break
        if self.latest_detections is None:
            self.get_logger().warn(
                'No YOLO detections — is classifier running? '
                '(ros2 run object_classification classifier)')

        # ── Detect ────────────────────────────────────────────────────
        detection, major_axis = self._detect_object()
        if detection is None:
            self.get_logger().error('Object detection failed — aborting')
            self._save_debug_capture()
            return
        if self.save_debug and major_axis is not None:
            self._debug_capture['major_axis'] = major_axis.tolist()

        # ── GPD grasp planning ────────────────────────────────────────
        grasps = self._call_gpd(detection)
        if grasps is None or len(grasps) == 0:
            self.get_logger().error('GPD found no grasps — aborting')
            self._save_debug_capture()
            self.go_home()
            return

        # ── Execute best grasp ────────────────────────────────────────
        grasp_pose = self._select_and_execute_grasp(grasps, detection, major_axis)
        if grasp_pose is None:
            self.get_logger().error('Grasp execution failed — aborting')
            self._save_debug_capture()
            self.go_home()
            return

        # ── Lift ──────────────────────────────────────────────────────
        ox, oy, oz = detection
        self.get_logger().info('Lifting object')
        lift_pose = Pose()
        lift_pose.position.x = grasp_pose.position.x
        lift_pose.position.y = grasp_pose.position.y
        lift_pose.position.z = grasp_pose.position.z + LIFT_HEIGHT
        lift_pose.orientation = grasp_pose.orientation
        if not self.plan_and_execute(lift_pose, 'lift'):
            self.get_logger().error('Lift failed')

        # ── Place back and release ────────────────────────────────────
        self.get_logger().info('Placing object back down')
        self.plan_and_execute(grasp_pose, 'place-down')

        self.get_logger().info('Releasing object')
        self.set_gripper(0.0)
        time.sleep(0.5)

        self.go_home()

        self._save_debug_capture()

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('GPD Grasp Demo complete!')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = GpdGraspDemo()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
