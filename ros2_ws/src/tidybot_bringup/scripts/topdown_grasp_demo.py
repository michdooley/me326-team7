#!/usr/bin/env python3
"""
Top-Down Grasp Demo — Simple depth-PCA-based top-down grasping

Detects objects using the external YOLO classifier node (/objbbox),
uses depth point cloud + PCA to accurately localize the object and
determine its orientation, then executes a simple top-down grasp.

For elongated objects (e.g. banana), the gripper is oriented perpendicular
to the object's major axis so it grips across the narrow dimension.

Requires:
    - ros2 launch tidybot_bringup sim.launch.py
    - ros2 run object_classification classifier

Usage:
    # Default: grasp a banana with right arm
    ros2 run tidybot_bringup topdown_grasp_demo.py

    # Target a specific object/arm:
    ros2 run tidybot_bringup topdown_grasp_demo.py --ros-args \
        -p target:=apple -p arm:=right -p z_offset:=-0.03
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import tf2_ros

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from tidybot_msgs.msg import ArmCommand
from tidybot_msgs.srv import PlanToTarget
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation

from tidybot_perception.grasp_geometry import (
    yaw_to_grasp_quaternion,
    GRIPPER_MAX_OPENING_M,
)

# YOLO COCO class name -> string ID
YOLO_CLASS_IDS = {
    'person': '0',
    'banana': '46',
    'apple': '47',
    'orange': '49',
}

# Timing constants
SETTLE_TIME = 1.0
GRIPPER_CLOSE_REPEATS = 20
GRIPPER_CLOSE_WAIT = 2.0

# Point cloud filtering
MIN_DEPTH_M = 0.1
MAX_DEPTH_M = 2.0
MIN_POINTS = 20
TABLE_Z_MIN = 0.005
OUTLIER_Z_THRESH = 0.05  # keep points within ±5cm of median Z


class TopdownGraspDemo(Node):
    """Detect via YOLO, plan top-down grasp via depth PCA, execute pick."""

    def __init__(self):
        super().__init__('topdown_grasp_demo')

        # Parameters
        self.declare_parameter('target', 'banana')
        self.declare_parameter('arm', 'right')
        self.declare_parameter('duration', 2.0)
        self.declare_parameter('pre_grasp_height', 0.10)
        self.declare_parameter('lift_height', 0.15)
        self.declare_parameter('z_offset', -0.02)
        self.declare_parameter('bbox_pad', 1.3)
        self.target_object = self.get_parameter('target').value
        self.arm_name = self.get_parameter('arm').value
        self.move_duration = self.get_parameter('duration').value
        self.pre_grasp_height = self.get_parameter('pre_grasp_height').value
        self.lift_height = self.get_parameter('lift_height').value
        self.z_offset = self.get_parameter('z_offset').value
        self.bbox_pad = self.get_parameter('bbox_pad').value

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

        # Publishers
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, f'/{self.arm_name}_gripper/cmd', 10)
        self.arm_pub = self.create_publisher(
            ArmCommand, f'/{self.arm_name}_arm/cmd', 10)
        self.marker_pub = self.create_publisher(Marker, '/grasp_debug_markers', 10)
        self.marker_id = 0

        # Service clients
        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')

    # -- Callbacks ---------------------------------------------------------

    def _detections_cb(self, msg):
        self.latest_detections = msg

    def _depth_cb(self, msg):
        self.latest_depth = self._bridge.imgmsg_to_cv2(msg, '16UC1')

    def _rgb_cb(self, msg):
        self.latest_rgb = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _info_cb(self, msg):
        self.camera_info = msg

    # -- Utilities ---------------------------------------------------------

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
        time.sleep(GRIPPER_CLOSE_WAIT)
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

    # -- TF ----------------------------------------------------------------

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

    # -- YOLO detection ----------------------------------------------------

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

    # -- Motion planning ---------------------------------------------------

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

    # -- Depth PCA pipeline ------------------------------------------------

    def _crop_depth_to_bbox(self, detection):
        """Crop depth image to padded YOLO bounding box.

        Returns (depth_roi, u_offset, v_offset) or (None, None, None).
        """
        if self.latest_depth is None:
            return None, None, None

        h, w = self.latest_depth.shape
        cx = int(detection.bbox.center.position.x)
        cy = int(detection.bbox.center.position.y)
        half_w = int(detection.bbox.size_x * self.bbox_pad / 2)
        half_h = int(detection.bbox.size_y * self.bbox_pad / 2)

        u0 = max(0, cx - half_w)
        v0 = max(0, cy - half_h)
        u1 = min(w, cx + half_w)
        v1 = min(h, cy + half_h)

        depth_roi = self.latest_depth[v0:v1, u0:u1]
        return depth_roi, u0, v0

    def _depth_roi_to_points_base(self, depth_roi, u_offset, v_offset):
        """Back-project depth ROI to 3D point cloud in base_link frame.

        Returns (N, 3) numpy array or None.
        """
        if self.camera_info is None:
            return None

        K = self.camera_info.k
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]

        depth_m = depth_roi.astype(np.float32) / 1000.0
        roi_h, roi_w = depth_m.shape

        # Build pixel coordinate grids offset to full-image coordinates
        u_grid, v_grid = np.meshgrid(
            np.arange(roi_w) + u_offset,
            np.arange(roi_h) + v_offset)

        mask = (depth_m > MIN_DEPTH_M) & (depth_m < MAX_DEPTH_M)
        z = depth_m[mask]
        u = u_grid[mask].astype(np.float32)
        v = v_grid[mask].astype(np.float32)

        if len(z) < MIN_POINTS:
            return None

        # Pinhole back-projection to camera frame
        x_cam = (u - cx) * z / fx
        y_cam = (v - cy) * z / fy
        points_cam = np.stack([x_cam, y_cam, z], axis=1)

        # Transform to base_link
        tf_mat = self._get_tf_matrix('base_link', 'camera_color_optical_frame')
        if tf_mat is None:
            return None

        ones = np.ones((len(points_cam), 1), dtype=np.float32)
        pts_h = np.hstack([points_cam, ones])
        pts_base = (tf_mat @ pts_h.T).T[:, :3]

        # Filter: above table
        pts_base = pts_base[pts_base[:, 2] > TABLE_Z_MIN]

        # Filter: remove background outliers by Z
        if len(pts_base) > MIN_POINTS:
            median_z = np.median(pts_base[:, 2])
            z_mask = np.abs(pts_base[:, 2] - median_z) < OUTLIER_Z_THRESH
            pts_base = pts_base[z_mask]

        if len(pts_base) < MIN_POINTS:
            return None

        return pts_base

    def _pca_analysis(self, points_base):
        """Run 2D PCA (XY plane) on point cloud to get object geometry.

        Returns dict with centroid, major_axis, eigenvalues, extents, etc.
        """
        centroid = np.mean(points_base, axis=0)
        centered_xy = points_base[:, :2] - centroid[:2]

        # 2x2 covariance matrix in XY plane
        cov = np.cov(centered_xy.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)

        # eigh returns ascending order — flip to descending
        eigenvalues = eigenvalues[::-1]
        eigenvectors = eigenvectors[:, ::-1]

        # Major axis in base_link XY plane
        major_2d = eigenvectors[:, 0]
        major_axis = np.array([major_2d[0], major_2d[1], 0.0])

        # Elongation check
        ratio = np.sqrt(eigenvalues[0] / max(eigenvalues[1], 1e-10))
        is_elongated = ratio > 1.5

        # Extents along major and minor axes
        proj_major = centered_xy @ major_2d
        proj_minor = centered_xy @ eigenvectors[:, 1]
        extent_major = float(np.ptp(proj_major))
        extent_minor = float(np.ptp(proj_minor))

        z_top = float(np.max(points_base[:, 2]))

        return {
            'centroid': centroid,
            'major_axis': major_axis,
            'eigenvalues': eigenvalues,
            'is_elongated': is_elongated,
            'ratio': ratio,
            'extent_major': extent_major,
            'extent_minor': extent_minor,
            'z_top': z_top,
            'num_points': len(points_base),
        }

    def _compute_grasp_pose(self, pca):
        """Compute grasp and pre-grasp poses from PCA results.

        Returns (grasp_pose, pre_grasp_pose, use_orientation, info_str) or None.
        """
        centroid = pca['centroid']

        # Determine grasp yaw
        if pca['is_elongated']:
            # Grip perpendicular to major axis (across narrow dimension)
            major = pca['major_axis']
            grasp_yaw = np.arctan2(major[1], major[0]) + np.pi / 2
            grip_width = pca['extent_minor']
            self.get_logger().info(
                f'  Elongated object (ratio {pca["ratio"]:.1f}), '
                f'gripping perpendicular to major axis')
        else:
            grasp_yaw = 0.0
            grip_width = pca['extent_major']
            self.get_logger().info(
                f'  Roughly circular object (ratio {pca["ratio"]:.1f}), '
                f'using default orientation')

        # Width check (warn only — PCA extents overestimate due to point cloud spread)
        if grip_width > GRIPPER_MAX_OPENING_M:
            self.get_logger().warn(
                f'  Estimated width {grip_width*1000:.0f}mm > '
                f'{GRIPPER_MAX_OPENING_M*1000:.0f}mm gripper opening '
                f'(proceeding anyway — PCA extents are approximate)')
        else:
            self.get_logger().info(
                f'  Grip width: {grip_width*1000:.0f}mm '
                f'(max {GRIPPER_MAX_OPENING_M*1000:.0f}mm)')

        # Grasp orientation
        qw, qx, qy, qz = yaw_to_grasp_quaternion(grasp_yaw)

        # Grasp position: PCA centroid XY, slightly below top surface
        grasp_z = pca['z_top'] + self.z_offset

        grasp_pose = Pose()
        grasp_pose.position.x = float(centroid[0])
        grasp_pose.position.y = float(centroid[1])
        grasp_pose.position.z = float(grasp_z)
        grasp_pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

        # Pre-grasp: same XY, higher Z
        pre_grasp_pose = Pose()
        pre_grasp_pose.position.x = float(centroid[0])
        pre_grasp_pose.position.y = float(centroid[1])
        pre_grasp_pose.position.z = float(grasp_z + self.pre_grasp_height)
        pre_grasp_pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

        info = (f'yaw={np.degrees(grasp_yaw):.0f}deg, '
                f'z={grasp_z:.3f}m, width={grip_width*1000:.0f}mm')

        return grasp_pose, pre_grasp_pose, info

    # -- Main pipeline -----------------------------------------------------

    def run(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info('Top-Down Grasp Demo')
        self.get_logger().info(f'  Target          : {self.target_object}')
        self.get_logger().info(f'  Arm             : {self.arm_name}')
        self.get_logger().info(f'  Pre-grasp height: {self.pre_grasp_height}m')
        self.get_logger().info(f'  Z offset        : {self.z_offset}m')
        self.get_logger().info(f'  Bbox padding    : {self.bbox_pad}x')
        self.get_logger().info('=' * 50)

        # Wait for motion planner
        self.get_logger().info('Waiting for /plan_to_target service...')
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('  ...waiting')

        # Wait for YOLO detections
        self.get_logger().info('Waiting for YOLO classifier (/objbbox)...')
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_detections is not None:
                break
        if self.latest_detections is None:
            self.get_logger().warn(
                'No YOLO detections -- is classifier running? '
                '(ros2 run object_classification classifier)')

        # Wait for depth + camera info
        self.get_logger().info('Waiting for depth + camera info...')
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_depth is not None and self.camera_info is not None:
                break
        if self.latest_depth is None or self.camera_info is None:
            self.get_logger().error('No depth/camera_info received -- aborting')
            return

        # -- Step 1: YOLO detection ----------------------------------------
        self.spin_for(1.0)  # let data flow
        self.get_logger().info(f'Detecting "{self.target_object}" via YOLO...')

        det = None
        for _ in range(20):
            det = self.detect(self.target_object)
            if det is not None:
                break
            self.spin_for(0.2)

        if det is None:
            self.get_logger().error(
                f'Could not find "{self.target_object}" in camera view!')
            return

        u = int(det.bbox.center.position.x)
        v = int(det.bbox.center.position.y)
        bbox_w = float(det.bbox.size_x)
        bbox_h = float(det.bbox.size_y)
        self.get_logger().info(
            f'  YOLO detection at pixel ({u}, {v}), '
            f'bbox {bbox_w:.0f}x{bbox_h:.0f}')

        # -- Step 2: Depth point cloud from bbox ROI -----------------------
        self.get_logger().info('Generating point cloud from depth ROI...')
        depth_roi, u_off, v_off = self._crop_depth_to_bbox(det)
        if depth_roi is None:
            self.get_logger().error('Failed to crop depth image')
            return

        pts_base = self._depth_roi_to_points_base(depth_roi, u_off, v_off)
        if pts_base is None:
            self.get_logger().error(
                f'Insufficient points in depth ROI (need >= {MIN_POINTS})')
            return

        self.get_logger().info(f'  Point cloud: {len(pts_base)} points')

        # -- Step 3: PCA analysis ------------------------------------------
        self.get_logger().info('Running PCA analysis...')
        pca = self._pca_analysis(pts_base)
        centroid = pca['centroid']
        self.get_logger().info(
            f'  Centroid: ({centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f})')
        self.get_logger().info(
            f'  Extents: major={pca["extent_major"]*1000:.0f}mm, '
            f'minor={pca["extent_minor"]*1000:.0f}mm')
        self.get_logger().info(
            f'  Z top: {pca["z_top"]:.3f}m')

        # -- Step 4: Compute grasp pose ------------------------------------
        result = self._compute_grasp_pose(pca)
        if result is None:
            self.get_logger().error('Grasp pose computation failed -- aborting')
            self.go_home()
            return
        grasp_pose, pre_grasp_pose, info = result
        self.get_logger().info(f'  Grasp params: {info}')

        # Debug markers
        self.publish_marker(
            centroid[0], centroid[1], centroid[2],
            0.0, 1.0, 0.0, label='object', scale=0.02)
        gp = grasp_pose.position
        self.publish_marker(
            gp.x, gp.y, gp.z,
            1.0, 0.0, 0.0, label='grasp', scale=0.02)
        pp = pre_grasp_pose.position
        self.publish_marker(
            pp.x, pp.y, pp.z,
            0.0, 0.0, 1.0, label='pre_grasp', scale=0.02)

        # -- Step 5: IK check ----------------------------------------------
        self.get_logger().info('Checking IK feasibility...')
        use_orientation = True
        ok, msg = self._try_ik(grasp_pose, use_orientation=True)
        if not ok:
            self.get_logger().warn(f'  IK with orientation failed: {msg}')
            self.get_logger().info('  Trying position-only IK...')
            ok, msg = self._try_ik(grasp_pose, use_orientation=False)
            if ok:
                use_orientation = False
                self.get_logger().info('  Position-only IK OK')
            else:
                self.get_logger().error(
                    f'  Position-only IK also failed: {msg} -- aborting')
                self.go_home()
                return
        else:
            self.get_logger().info('  IK OK')

        # -- Step 6: Execute grasp -----------------------------------------
        self.get_logger().info('Opening gripper')
        self.set_gripper(0.0)

        self.get_logger().info('Moving to pre-grasp')
        if not self.plan_and_execute(
                pre_grasp_pose, 'pre-grasp', use_orientation=use_orientation):
            self.get_logger().error('Pre-grasp failed -- aborting')
            self.go_home()
            return

        self.get_logger().info('Settling at pre-grasp...')
        time.sleep(SETTLE_TIME)

        self.get_logger().info('Descending to grasp')
        if not self.plan_and_execute(
                grasp_pose, 'grasp', use_orientation=use_orientation):
            self.get_logger().error('Grasp descent failed -- aborting')
            self.go_home()
            return

        self.get_logger().info('Settling at grasp position...')
        time.sleep(SETTLE_TIME)

        self.close_gripper_firm()

        # -- Step 7: Lift --------------------------------------------------
        self.get_logger().info('Lifting object')
        lift_pose = Pose()
        lift_pose.position.x = grasp_pose.position.x
        lift_pose.position.y = grasp_pose.position.y
        lift_pose.position.z = grasp_pose.position.z + self.lift_height
        lift_pose.orientation = grasp_pose.orientation
        if not self.plan_and_execute(
                lift_pose, 'lift', use_orientation=use_orientation):
            self.get_logger().warn('Lift failed, but continuing...')

        # -- Done ----------------------------------------------------------
        self.get_logger().info('Returning home and releasing')
        self.go_home()
        self.set_gripper(0.0)

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Top-Down Grasp Demo complete!')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = TopdownGraspDemo()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
