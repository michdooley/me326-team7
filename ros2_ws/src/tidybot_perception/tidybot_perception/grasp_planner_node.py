#!/usr/bin/env python3
"""
Grasp Planner Node — GraspNet-1Billion integration

Computes 6-DOF grasp poses for objects using GraspNet-1Billion.
Given a 3D object position (in any TF frame) and the live depth image,
returns a grasp pose and pre-grasp approach pose in base_link frame.

Services:
    /plan_grasp (tidybot_msgs/PlanGrasp) — plan a grasp for an object

Depends on:
    /plan_to_target (tidybot_msgs/PlanToTarget) — validates reachability
    /camera/depth/image_raw  (sensor_msgs/Image)       — live depth image
    /camera/color/camera_info (sensor_msgs/CameraInfo) — intrinsics

Parameters:
    model_path    (str):   path to GraspNet checkpoint (.tar / .pth file)
    approach_offset (float): pre-grasp retreat distance in metres (default 0.10)
    default_arm   (str):   arm to use when "auto" is requested (default "right")
    num_point     (int):   point cloud sample size for GraspNet (default 20000)
    collision_thresh (float): collision detection threshold (default 0.01 m)
    crop_radius   (float): radius around object position to crop cloud (default 0.20 m)
    min_depth     (float): minimum depth to keep in metres (default 0.10 m)
    max_depth     (float): maximum depth to keep in metres (default 1.50 m)
    remove_plane  (bool):  run RANSAC ground plane removal (default True)
    plane_distance_threshold (float): RANSAC inlier distance in metres (default 0.01 m)

Usage:
    ros2 run tidybot_perception grasp_planner_node \\
        --ros-args -p model_path:=/path/to/checkpoint-rs.tar
"""

import os
import sys
import threading
import time
import numpy as np

# ── GraspNet-baseline imports (added to sys.path at module load) ────────────
GRASPNET_ROOT = os.path.expanduser('~/graspnet-baseline')
for _subdir in ('', 'models', 'dataset', 'utils'):
    _p = os.path.join(GRASPNET_ROOT, _subdir)
    if _p not in sys.path:
        sys.path.insert(0, _p)

GRASPNET_AVAILABLE = False
try:
    import torch
    from graspnet import GraspNet, pred_decode
    from graspnetAPI import GraspGroup
    from data_utils import CameraInfo as GraspNetCameraInfo, create_point_cloud_from_depth_image
    from collision_detector import ModelFreeCollisionDetector
    import open3d as o3d
    GRASPNET_AVAILABLE = True
except ImportError as _e:
    _import_error_msg = str(_e)
# ────────────────────────────────────────────────────────────────────────────

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs  # noqa: F401

from geometry_msgs.msg import Pose, PoseStamped, PointStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from tidybot_msgs.srv import PlanGrasp, PlanToTarget
import struct
import math

from .coord_converter import CoordConverter


class GraspPlannerNode(Node):
    """GraspNet-1Billion based grasp planning node."""

    def __init__(self):
        super().__init__('grasp_planner_node')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('model_path', '')
        self.declare_parameter('approach_offset', 0.10)
        self.declare_parameter('default_arm', 'right')
        self.declare_parameter('num_point', 20000)
        self.declare_parameter('collision_thresh', 0.01)
        self.declare_parameter('crop_radius', 0.20)
        self.declare_parameter('min_depth', 0.10)
        self.declare_parameter('max_depth', 1.50)
        self.declare_parameter('remove_plane', True)
        self.declare_parameter('plane_distance_threshold', 0.01)
        self.declare_parameter('sim', True)  # True = use sim depth intrinsics (fovy=57°)

        self.model_path = self.get_parameter('model_path').value
        self.approach_offset = self.get_parameter('approach_offset').value
        self.default_arm = self.get_parameter('default_arm').value
        self.num_point = self.get_parameter('num_point').value
        self.collision_thresh = self.get_parameter('collision_thresh').value
        self.crop_radius = self.get_parameter('crop_radius').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.remove_plane = self.get_parameter('remove_plane').value
        self.plane_distance_threshold = self.get_parameter('plane_distance_threshold').value
        self._sim = self.get_parameter('sim').value

        # ── TF2 ──────────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.coord_converter = CoordConverter(self.tf_buffer)

        # ── Camera data (latest snapshots) ───────────────────────────────
        self._depth_lock = threading.Lock()
        self._depth_msg: Image | None = None
        self._camera_info: CameraInfo | None = None
        self.cv_bridge = CvBridge()

        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self._depth_cb,
            rclpy.qos.QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            ),
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._camera_info_cb, 10
        )

        # ── RViz debug publishers ─────────────────────────────────────────
        self._cloud_pub = self.create_publisher(
            PointCloud2, '/grasp_debug/cloud', 10)
        self._obj_marker_pub = self.create_publisher(
            Marker, '/grasp_debug/object_marker', 10)
        self._grasp_marker_pub = self.create_publisher(
            MarkerArray, '/grasp_debug/grasp_markers', 10)

        # ── Service: /plan_grasp ──────────────────────────────────────────
        self.grasp_srv = self.create_service(
            PlanGrasp, '/plan_grasp', self.plan_grasp_callback
        )

        # ── Client: /plan_to_target (reachability check) ─────────────────
        self.plan_to_target_client = self.create_client(
            PlanToTarget, '/plan_to_target'
        )

        # ── Load GraspNet model ───────────────────────────────────────────
        self.net = None
        self.device = None
        self._load_model()

        self.get_logger().info('GraspPlannerNode initialized')
        if not GRASPNET_AVAILABLE:
            self.get_logger().error(
                f'GraspNet dependencies not found ({_import_error_msg}). '
                'Run: uv sync  (check pyproject.toml for torch/open3d/graspnetAPI)'
            )
        elif not self.model_path:
            self.get_logger().warn(
                'No model_path set — set with: '
                '--ros-args -p model_path:=/path/to/checkpoint-rs.tar'
            )

    # ── Subscriptions ────────────────────────────────────────────────────────

    def _depth_cb(self, msg: Image) -> None:
        with self._depth_lock:
            self._depth_msg = msg

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self._camera_info = msg  # written once (intrinsics don't change)

    # ── Model loading ────────────────────────────────────────────────────────

    def _load_model(self) -> None:
        """Load GraspNet-1Billion checkpoint."""
        if not GRASPNET_AVAILABLE:
            return
        if not self.model_path:
            return

        try:
            self.device = torch.device(
                'cuda:0' if torch.cuda.is_available() else 'cpu'
            )
            self.net = GraspNet(
                input_feature_dim=0,
                num_view=300,
                num_angle=12,
                num_depth=4,
                cylinder_radius=0.05,
                hmin=-0.02,
                hmax_list=[0.01, 0.02, 0.03, 0.04],
                is_training=False,
            )
            self.net.to(self.device)
            checkpoint = torch.load(
                self.model_path, map_location=self.device, weights_only=False
            )
            self.net.load_state_dict(checkpoint['model_state_dict'])
            self.net.eval()
            epoch = checkpoint.get('epoch', '?')
            self.get_logger().info(
                f'GraspNet loaded from {self.model_path} (epoch {epoch}, device={self.device})'
            )
        except Exception as exc:
            self.get_logger().error(f'Failed to load GraspNet model: {exc}')
            self.net = None

    # ── Main service callback ─────────────────────────────────────────────────

    def plan_grasp_callback(
        self,
        request: PlanGrasp.Request,
        response: PlanGrasp.Response,
    ) -> PlanGrasp.Response:
        """Plan a grasp pose for the given object position."""
        if not GRASPNET_AVAILABLE:
            response.success = False
            response.message = (
                'GraspNet not available — install deps and set model_path'
            )
            return response

        if self.net is None:
            response.success = False
            response.message = (
                'GraspNet model not loaded — set model_path parameter'
            )
            return response

        arm = (
            request.arm_name
            if request.arm_name and request.arm_name != 'auto'
            else self.default_arm
        )
        response.arm_used = arm

        # ── 1. Wait briefly for depth + camera_info ───────────────────────
        deadline = time.time() + 3.0
        while time.time() < deadline:
            if self._camera_info is not None:
                with self._depth_lock:
                    depth_msg = self._depth_msg
                if depth_msg is not None:
                    break
            time.sleep(0.05)
        else:
            response.success = False
            response.message = 'Timed out waiting for depth image / camera_info'
            return response

        with self._depth_lock:
            depth_msg = self._depth_msg

        # ── 2. Convert depth image → metric depth array ───────────────────
        try:
            depth_raw = self.cv_bridge.imgmsg_to_cv2(depth_msg, '16UC1')
        except Exception as exc:
            response.success = False
            response.message = f'Failed to decode depth image: {exc}'
            return response

        # RealSense depth: 16-bit unsigned millimetres → float32 metres
        depth_m = depth_raw.astype(np.float32) / 1000.0

        ci = self._camera_info
        h, w = depth_m.shape[:2]

        if self._sim:
            # Sim: depth camera has fovy=57°, different from RGB (fovy=42°)
            depth_fovy_rad = math.radians(57.0)
            fx = fy = float(h) / (2.0 * math.tan(depth_fovy_rad / 2.0))
            cx, cy = float(w) / 2.0, float(h) / 2.0
            self.get_logger().info(
                f'Sim depth intrinsics: fx={fx:.1f}, fy={fy:.1f}, '
                f'cx={cx:.1f}, cy={cy:.1f}  (fovy=57°, {w}x{h})')
        else:
            # Real D435: depth is hardware-aligned to RGB, so RGB intrinsics are correct
            fx, fy = ci.k[0], ci.k[4]
            cx, cy = ci.k[2], ci.k[5]

        # ── 3. Build full-scene point cloud ───────────────────────────────
        graspnet_cam = GraspNetCameraInfo(
            width=float(w),
            height=float(h),
            fx=fx, fy=fy, cx=cx, cy=cy,
            scale=1.0,  # depth_m is already in metres
        )
        # cloud_full: (H, W, 3) organised cloud in camera optical frame
        cloud_full = create_point_cloud_from_depth_image(
            depth_m, graspnet_cam, organized=True
        )

        # ── 4. Crop cloud around the requested object position ────────────
        # Transform object_position to camera optical frame
        obj_in_camera = self._transform_to_camera(request.object_position)
        if obj_in_camera is None:
            response.success = False
            response.message = (
                f'Failed to transform object_position from '
                f'{request.object_position.header.frame_id} to camera frame'
            )
            return response

        obj_xyz = np.array([
            obj_in_camera.point.x,
            obj_in_camera.point.y,
            obj_in_camera.point.z,
        ])

        # Mask: depth range + within crop_radius of object
        valid = (depth_m > self.min_depth) & (depth_m < self.max_depth)
        dists = np.linalg.norm(cloud_full - obj_xyz, axis=-1)
        mask = valid & (dists < self.crop_radius)
        cloud_masked = cloud_full[mask]

        n_depth_valid = int(np.count_nonzero(valid))
        self.get_logger().info(
            f'Point cloud: {n_depth_valid} in depth range '
            f'[{self.min_depth:.2f}, {self.max_depth:.2f}]m, '
            f'{len(cloud_masked)} in crop radius {self.crop_radius:.2f}m'
        )

        # Publish debug cloud + object marker to RViz
        self._publish_debug_cloud(cloud_masked, CoordConverter.CAMERA_OPTICAL_FRAME)
        self._publish_object_marker(obj_in_camera)

        if len(cloud_masked) < 100:
            response.success = False
            response.message = (
                f'Too few points ({len(cloud_masked)}) near object — '
                'check camera tilt and depth topic, or adjust '
                'min_depth/max_depth/crop_radius'
            )
            return response

        # ── 4b. RANSAC ground plane removal ────────────────────────────
        if self.remove_plane and len(cloud_masked) > 500:
            try:
                plane_cloud = o3d.geometry.PointCloud()
                plane_cloud.points = o3d.utility.Vector3dVector(
                    cloud_masked.astype(np.float32)
                )
                plane_model, inlier_idxs = plane_cloud.segment_plane(
                    distance_threshold=self.plane_distance_threshold,
                    ransac_n=3,
                    num_iterations=1000,
                )
                a, b, c, d = plane_model

                inlier_set = set(inlier_idxs)
                outlier_mask = np.array(
                    [i not in inlier_set for i in range(len(cloud_masked))]
                )

                self.get_logger().info(
                    f'RANSAC plane: normal=({a:.3f}, {b:.3f}, {c:.3f}), '
                    f'inliers={len(inlier_idxs)}, '
                    f'outliers={int(outlier_mask.sum())}'
                )

                if outlier_mask.sum() >= 100:
                    cloud_masked = cloud_masked[outlier_mask]
                else:
                    self.get_logger().warn(
                        f'Plane removal would leave only '
                        f'{int(outlier_mask.sum())} points — skipping'
                    )
            except Exception as exc:
                self.get_logger().warn(
                    f'RANSAC plane removal failed (skipping): {exc}'
                )

        # ── 5. Subsample to num_point (seeded for determinism) ────────────
        rng = np.random.RandomState(42)
        n = len(cloud_masked)
        if n >= self.num_point:
            idxs = rng.choice(n, self.num_point, replace=False)
        else:
            idxs = np.concatenate([
                np.arange(n),
                rng.choice(n, self.num_point - n, replace=True),
            ])
        cloud_sampled = cloud_masked[idxs].astype(np.float32)  # (N, 3)

        # ── 6. GraspNet forward pass (seeded for determinism) ─────────────
        try:
            torch.manual_seed(42)
            if torch.cuda.is_available():
                torch.cuda.manual_seed(42)
            cloud_tensor = (
                torch.from_numpy(cloud_sampled[np.newaxis])  # (1, N, 3)
                .to(self.device)
            )
            end_points = {'point_clouds': cloud_tensor}
            with torch.no_grad():
                end_points = self.net(end_points)
                grasp_preds = pred_decode(end_points)

            gg_array = grasp_preds[0].detach().cpu().numpy()
            gg = GraspGroup(gg_array)
        except Exception as exc:
            response.success = False
            response.message = f'GraspNet inference failed: {exc}'
            return response

        if len(gg) == 0:
            response.success = False
            response.message = 'GraspNet returned no grasp candidates'
            return response

        # ── 7. Collision filtering + NMS + sort ───────────────────────────
        try:
            cloud_o3d = o3d.geometry.PointCloud()
            cloud_o3d.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
            detector = ModelFreeCollisionDetector(
                np.asarray(cloud_o3d.points), voxel_size=0.01
            )
            collision_mask = detector.detect(
                gg, approach_dist=0.05, collision_thresh=self.collision_thresh
            )
            gg = gg[~collision_mask]
        except Exception as exc:
            self.get_logger().warn(f'Collision detection failed (skipping): {exc}')

        if len(gg) == 0:
            response.success = False
            response.message = 'No collision-free grasps found'
            return response

        gg.nms()
        gg.sort_by_score()

        # ── 7b. Filter grasps by proximity to object in camera frame ────
        # GraspNet can produce grasps anywhere in the cropped cloud.
        # Keep only grasps whose translation is within a tight radius of
        # the actual object position (in camera optical frame).
        grasp_proximity_thresh = 0.08  # 8 cm
        grasp_translations = gg.translations  # (N, 3) in camera frame
        grasp_dists = np.linalg.norm(grasp_translations - obj_xyz, axis=1)
        close_mask = grasp_dists < grasp_proximity_thresh
        self.get_logger().info(
            f'Grasps after NMS: {len(gg)}, '
            f'within {grasp_proximity_thresh}m of object: {int(close_mask.sum())}'
        )
        if close_mask.sum() > 0:
            gg = gg[close_mask]
        else:
            self.get_logger().warn(
                'No grasps within proximity threshold — using closest grasp'
            )
            closest_idx = np.argmin(grasp_dists)
            gg = gg[closest_idx:closest_idx + 1]

        best = gg[0]  # highest-score grasp in camera optical frame
        self.get_logger().info(
            f'Selected grasp: score={best.score:.3f}, '
            f'translation=({best.translation[0]:.3f}, {best.translation[1]:.3f}, {best.translation[2]:.3f}), '
            f'object=({obj_xyz[0]:.3f}, {obj_xyz[1]:.3f}, {obj_xyz[2]:.3f}), '
            f'dist={np.linalg.norm(best.translation - obj_xyz):.3f}m'
        )

        # ── 8. Build grasp PoseStamped in camera frame ────────────────────
        #
        # GraspNet rotation matrix convention (camera optical frame):
        #   R[:, 0] = approach vector  (direction gripper moves toward object)
        #   R[:, 1] = closing direction (along which fingers move)
        #   R[:, 2] = lateral axis     (determined by right-hand rule)
        #
        # After transforming R to base_link, R[:, 0] in base_link gives the
        # approach direction used to compute the pre-grasp retreat offset.
        grasp_pose_cam = CoordConverter.rotation_matrix_to_pose(
            best.rotation_matrix, best.translation
        )
        grasp_ps_cam = PoseStamped()
        grasp_ps_cam.header.frame_id = CoordConverter.CAMERA_OPTICAL_FRAME
        grasp_ps_cam.header.stamp = self.get_clock().now().to_msg()
        grasp_ps_cam.pose = grasp_pose_cam

        # ── 9. Transform grasp pose to base_link ──────────────────────────
        try:
            grasp_ps_base = self.coord_converter.camera_to_base(grasp_ps_cam)
        except Exception as exc:
            response.success = False
            response.message = f'TF transform to base_link failed: {exc}'
            return response

        grasp_pose_base = grasp_ps_base.pose

        # ── 10. Compute pre-grasp pose (retreat along approach vector) ────
        pre_grasp_pose_base = CoordConverter.offset_pose_along_approach(
            grasp_pose_base,
            approach_col=0,           # GraspNet: col 0 is approach direction
            offset_m=self.approach_offset,
        )

        # ── 11. (Optional) validate reachability — DISABLED for debugging ─
        # if self.plan_to_target_client.service_is_ready():
        #     ok = self._check_reachable(arm, pre_grasp_pose_base)
        #     if not ok:
        #         self.get_logger().warn(
        #             'Pre-grasp pose not reachable — returning it anyway for '
        #             'debugging; consider running with a different crop_radius.'
        #         )

        # ── Publish grasp markers to RViz ──────────────────────────────────
        self._publish_grasp_markers(
            grasp_pose_base, pre_grasp_pose_base, 'base_link')

        # ── 12. Fill response ──────────────────────────────────────────────
        response.success = True
        response.grasp_pose = grasp_pose_base
        response.pre_grasp_pose = pre_grasp_pose_base
        response.arm_used = arm
        response.grasp_width = float(best.width)
        response.message = (
            f'Grasp planned (score={best.score:.3f}, width={best.width:.3f}m, '
            f'depth={best.depth:.3f}m, device={self.device})'
        )
        self.get_logger().info(response.message)
        return response

    # ── Internal helpers ─────────────────────────────────────────────────────

    def _transform_to_camera(self, point_stamped: PointStamped) -> PointStamped | None:
        """Transform a PointStamped into camera_color_optical_frame."""
        if point_stamped.header.frame_id == CoordConverter.CAMERA_OPTICAL_FRAME:
            return point_stamped
        try:
            transform = self.tf_buffer.lookup_transform(
                CoordConverter.CAMERA_OPTICAL_FRAME,
                point_stamped.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            return tf2_geometry_msgs.do_transform_point(point_stamped, transform)
        except Exception as exc:
            self.get_logger().error(f'TF transform to camera failed: {exc}')
            return None

    def _publish_debug_cloud(self, cloud: np.ndarray, frame_id: str) -> None:
        """Publish a (N,3) float32 point cloud as PointCloud2."""
        cloud = cloud.astype(np.float32)
        n = len(cloud)
        msg = PointCloud2()
        msg.header = Header(frame_id=frame_id, stamp=self.get_clock().now().to_msg())
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * n
        msg.is_dense = True
        msg.data = cloud.tobytes()
        self._cloud_pub.publish(msg)

    def _publish_object_marker(self, point: PointStamped) -> None:
        """Publish a sphere marker at the object position."""
        m = Marker()
        m.header = point.header
        m.ns = 'object'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = point.point
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.04
        m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9)
        m.lifetime.sec = 30
        self._obj_marker_pub.publish(m)

    def _publish_grasp_markers(
        self, grasp_pose: Pose, pre_grasp_pose: Pose, frame_id: str
    ) -> None:
        """Publish arrow markers for grasp (green) and pre-grasp (blue)."""
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()
        for i, (pose, color, label) in enumerate([
            (grasp_pose, ColorRGBA(r=0.0, g=0.8, b=0.0, a=0.9), 'grasp'),
            (pre_grasp_pose, ColorRGBA(r=0.0, g=0.4, b=1.0, a=0.9), 'pre_grasp'),
        ]):
            m = Marker()
            m.header = Header(frame_id=frame_id, stamp=now)
            m.ns = label
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose = pose
            m.scale.x = 0.06  # arrow length
            m.scale.y = 0.015  # shaft diameter
            m.scale.z = 0.02  # head diameter
            m.color = color
            m.lifetime.sec = 30
            arr.markers.append(m)
        self._grasp_marker_pub.publish(arr)

    def _check_reachable(self, arm: str, pose: Pose) -> bool:
        """Call /plan_to_target with execute=False to test reachability."""
        req = PlanToTarget.Request()
        req.arm_name = arm
        req.target_pose = pose
        req.use_orientation = True
        req.execute = False
        req.duration = 2.0
        req.max_condition_number = 100.0

        future = self.plan_to_target_client.call_async(req)
        # Cannot use spin_until_future_complete here because we are
        # already inside a spinning callback.  Busy-wait instead.
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if future.done() and not future.exception():
            return future.result().success
        return False


def main(args=None):
    rclpy.init(args=args)
    node = GraspPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
