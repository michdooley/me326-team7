#!/usr/bin/env python3
"""
YOLO Object Detector

Drop-in replacement for RGBDObjectDetector that uses YOLOv8 instead of
HSV color segmentation. Detects objects by class name (e.g., 'banana',
'apple', 'sports ball') and estimates their 3D position in base_link
using the depth image, camera intrinsics, and TF2.

Usage:
    from tidybot_perception.yolo_object_detector import YOLOObjectDetector

    # Inside a ROS2 node:
    detector = YOLOObjectDetector(node)
    result = detector.detect_and_localize('banana')
    if result is not None:
        pixel, base_pt = result
        print(f'Object at base_link: {base_pt.point}')
"""

import numpy as np
import cv2

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import tf2_ros
import tf2_geometry_msgs  # noqa: F401

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge

from tidybot_perception.coord_converter import CoordConverter

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class YOLOObjectDetector:
    """
    Reusable YOLO-based RGB-D perception utility.

    Subscribes to camera topics and provides methods to detect objects
    by class name and estimate their 3D position in base_link.

    Args:
        node: A ROS2 Node to attach subscriptions and TF listener to.
        model_path: Path to YOLO weights (default: 'yolov8n.pt', auto-downloaded).
        confidence: Minimum detection confidence threshold (default: 0.5).
        depth_patch_radius: Half-size of the patch sampled around the
            centroid for robust depth estimation (default 5 -> 11x11 patch).
        floor_z_min: Minimum allowed z in base_link (metres). Points below
            this are clamped up to prevent underground coordinates from
            depth noise. Default: 0.005 m (5 mm above floor).
    """

    FLOOR_Z_MIN = 0.005

    def __init__(self, node: Node, model_path: str = 'yolov8n.pt',
                 confidence: float = 0.5, depth_patch_radius: int = 5,
                 floor_z_min: float = None):
        self._node = node
        self._confidence = confidence
        self._patch_r = depth_patch_radius
        self._floor_z = floor_z_min if floor_z_min is not None else self.FLOOR_Z_MIN

        # Load YOLO model
        if YOLO is None:
            node.get_logger().error(
                'ultralytics not installed — run: pip install ultralytics')
            self.model = None
            self._class_name_to_id = {}
        else:
            node.get_logger().info(f'Loading YOLO model: {model_path}')
            self.model = YOLO(model_path)
            # Build reverse lookup: class name -> class id
            self._class_name_to_id = {
                name.lower(): cid for cid, name in self.model.names.items()
            }
            node.get_logger().info(
                f'YOLO loaded — {len(self.model.names)} classes available')

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        self.coord_conv = CoordConverter(self.tf_buffer)

        # CV bridge
        self._bridge = CvBridge()

        # Camera state
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_info = None

        # Subscribers
        qos_be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(
            Image, '/camera/color/image_raw', self._rgb_cb, qos_be)
        node.create_subscription(
            Image, '/camera/depth/image_raw', self._depth_cb, qos_be)
        node.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._info_cb, 10)

    # -- Callbacks ---------------------------------------------------------

    def _rgb_cb(self, msg):
        self.latest_rgb = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _depth_cb(self, msg):
        self.latest_depth = self._bridge.imgmsg_to_cv2(msg, '16UC1')

    def _info_cb(self, msg):
        self.camera_info = msg

    # -- YOLO detection ----------------------------------------------------

    def _find_target(self, target_class):
        """Run YOLO and return the best detection matching target_class.

        Args:
            target_class: Object class name (e.g., 'banana', 'apple',
                'sports ball'). Case-insensitive.

        Returns:
            (cx, cy, confidence) of the highest-confidence matching
            detection, or None if not found.
        """
        if self.model is None or self.latest_rgb is None:
            return None

        target_lower = target_class.lower()

        # Run inference (no class filter — detect everything)
        results = self.model(
            self.latest_rgb, conf=self._confidence, verbose=False)

        best = None
        best_conf = 0.0

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id].lower()
                conf = float(box.conf[0])

                if cls_name == target_lower and conf > best_conf:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx = int((x1 + x2) / 2.0)
                    cy = int((y1 + y2) / 2.0)
                    best = (cx, cy, conf)
                    best_conf = conf

        return best

    def detect(self, target_class, min_area=None):
        """
        Detect an object by class name in the latest RGB image.

        Args:
            target_class: COCO class name (e.g., 'banana', 'apple',
                'sports ball', 'cup'). Case-insensitive.
            min_area: Unused — kept for API compatibility with
                RGBDObjectDetector.detect_color().

        Returns:
            (u, v) pixel centroid of the best matching detection, or None.
        """
        result = self._find_target(target_class)
        if result is None:
            return None
        cx, cy, _conf = result
        return (cx, cy)

    # -- Depth back-projection ---------------------------------------------

    def pixel_to_base_link(self, u, v):
        """
        Back-project a pixel coordinate to a 3D point in base_link.

        Uses the depth image, camera intrinsics, and TF2 to convert
        pixel (u, v) -> camera-frame 3D point -> base_link 3D point.

        Args:
            u, v: Pixel column and row.

        Returns:
            PointStamped in base_link frame, or None on failure.
        """
        if self.latest_depth is None or self.camera_info is None:
            return None

        r = self._patch_r
        y_lo = max(0, v - r)
        y_hi = min(self.latest_depth.shape[0], v + r + 1)
        x_lo = max(0, u - r)
        x_hi = min(self.latest_depth.shape[1], u + r + 1)
        patch = self.latest_depth[y_lo:y_hi, x_lo:x_hi]
        valid = patch[(patch > 100) & (patch < 5000)]  # 0.1 m - 5 m
        if len(valid) == 0:
            return None

        depth_m = float(np.median(valid)) / 1000.0

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        xyz_cam = CoordConverter.depth_pixel_to_camera_point(
            u, v, depth_m, fx, fy, cx, cy)

        pt = PointStamped()
        pt.header.frame_id = 'camera_color_optical_frame'
        pt.header.stamp = self._node.get_clock().now().to_msg()
        pt.point.x = float(xyz_cam[0])
        pt.point.y = float(xyz_cam[1])
        pt.point.z = float(xyz_cam[2])

        try:
            base_pt = self.coord_conv.point_camera_to_base(pt)
        except Exception as e:
            self._node.get_logger().error(f'TF transform failed: {e}')
            return None

        # Clamp z to floor level
        if base_pt.point.z < self._floor_z:
            self._node.get_logger().warn(
                f'Clamping z from {base_pt.point.z:.4f} to {self._floor_z:.4f} '
                f'(below floor)')
            base_pt.point.z = self._floor_z

        return base_pt

    # -- Combined detect + localize ----------------------------------------

    def detect_and_localize(self, target_class):
        """
        Detect an object and return its 3D position in base_link.

        Args:
            target_class: COCO class name (e.g., 'banana', 'apple').

        Returns:
            Tuple of ((u, v), PointStamped_in_base_link), or None if
            the object was not found or localization failed.
        """
        centroid = self.detect(target_class)
        if centroid is None:
            return None

        u, v = centroid
        base_pt = self.pixel_to_base_link(u, v)
        if base_pt is None:
            return None

        return (centroid, base_pt)
