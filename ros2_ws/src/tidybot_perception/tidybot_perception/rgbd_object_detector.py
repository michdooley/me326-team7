#!/usr/bin/env python3
"""
RGB-D Object Detector

Detects colored objects using HSV segmentation on the RGB stream,
then estimates their 3D position in base_link using the depth image,
camera intrinsics, and TF2.

Usage:
    from tidybot_perception.rgbd_object_detector import RGBDObjectDetector

    # Inside a ROS2 node:
    detector = RGBDObjectDetector(node)
    result = detector.detect_and_localize('red')
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


# HSV color ranges (H: 0-180, S: 0-255, V: 0-255)
COLOR_RANGES = {
    'red':    [((0, 100, 80), (10, 255, 255)),
               ((165, 100, 80), (180, 255, 255))],
    'green':  [((40, 60, 60), (85, 255, 255))],
    'orange': [((10, 100, 80), (25, 255, 255))],
    'yellow': [((25, 80, 80), (40, 255, 255))],
}

MIN_BLOB_AREA = 200


class RGBDObjectDetector:
    """
    Reusable RGB-D perception utility.

    Subscribes to camera topics and provides methods to detect colored
    objects and estimate their 3D position in base_link.

    Args:
        node: A ROS2 Node to attach subscriptions and TF listener to.
        depth_patch_radius: Half-size of the patch sampled around the
            centroid for robust depth estimation (default 5 → 11x11 patch).
        floor_z_min: Minimum allowed z in base_link (metres). Points below
            this are clamped up to prevent underground coordinates from
            depth noise. Default: 0.005 m (5 mm above floor).
    """

    # In base_link, z=0 is approximately floor level.
    # Depth noise can produce points slightly below the floor.
    FLOOR_Z_MIN = 0.005  # 5 mm above floor — minimum allowed z in base_link

    def __init__(self, node: Node, depth_patch_radius: int = 5,
                 floor_z_min: float = None):
        self._node = node
        self._patch_r = depth_patch_radius
        self._floor_z = floor_z_min if floor_z_min is not None else self.FLOOR_Z_MIN

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

    # ── Callbacks ───────────────────────────────────────────────────

    def _rgb_cb(self, msg):
        self.latest_rgb = self._bridge.imgmsg_to_cv2(msg, 'rgb8')

    def _depth_cb(self, msg):
        self.latest_depth = self._bridge.imgmsg_to_cv2(msg, '16UC1')

    def _info_cb(self, msg):
        self.camera_info = msg

    # ── Color detection ─────────────────────────────────────────────

    def detect_color(self, color, min_area=MIN_BLOB_AREA):
        """
        Detect a colored object in the latest RGB image.

        Args:
            color: Color name ('red', 'green', 'orange', 'yellow').
            min_area: Minimum contour area in pixels to accept.

        Returns:
            (u, v) pixel centroid of the largest matching blob, or None.
        """
        if self.latest_rgb is None:
            return None

        if color not in COLOR_RANGES:
            raise ValueError(
                f'Unknown color {color!r}. '
                f'Available: {list(COLOR_RANGES.keys())}')

        bgr = cv2.cvtColor(self.latest_rgb, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in COLOR_RANGES[color]:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < min_area:
            return None

        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return (cx, cy)

    # ── Depth back-projection ───────────────────────────────────────

    def pixel_to_base_link(self, u, v):
        """
        Back-project a pixel coordinate to a 3D point in base_link.

        Uses the depth image, camera intrinsics, and TF2 to convert
        pixel (u, v) → camera-frame 3D point → base_link 3D point.

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
        valid = patch[(patch > 100) & (patch < 5000)]  # 0.1 m – 5 m
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

        # Clamp z to floor level — depth noise can place points underground
        if base_pt.point.z < self._floor_z:
            self._node.get_logger().warn(
                f'Clamping z from {base_pt.point.z:.4f} to {self._floor_z:.4f} '
                f'(below floor)')
            base_pt.point.z = self._floor_z

        return base_pt

    # ── Inverse projection (3D → pixel) ─────────────────────────────

    def base_link_to_pixel(self, x, y, z):
        """
        Project a 3D point in base_link back to image pixel (u, v).

        Inverse of pixel_to_base_link(). Transforms the point from
        base_link to camera optical frame, then projects using intrinsics.

        Args:
            x, y, z: Coordinates in base_link frame.

        Returns:
            (u, v) pixel coordinates, or None on failure.
        """
        if self.camera_info is None:
            return None

        # Build a PointStamped in base_link
        pt = PointStamped()
        pt.header.frame_id = 'base_link'
        pt.header.stamp = self._node.get_clock().now().to_msg()
        pt.point.x = float(x)
        pt.point.y = float(y)
        pt.point.z = float(z)

        # Transform base_link → camera_color_optical_frame
        try:
            import rclpy.time
            import rclpy.duration
            transform = self.tf_buffer.lookup_transform(
                'camera_color_optical_frame',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            cam_pt = tf2_geometry_msgs.do_transform_point(pt, transform)
        except Exception as e:
            self._node.get_logger().error(f'TF transform failed: {e}')
            return None

        # Camera optical frame: +Z forward, +X right, +Y down
        cx_cam = cam_pt.point.x
        cy_cam = cam_pt.point.y
        cz_cam = cam_pt.point.z

        if cz_cam <= 0.01:
            return None  # Behind camera

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        u = int(round(fx * cx_cam / cz_cam + cx))
        v = int(round(fy * cy_cam / cz_cam + cy))

        return (u, v)

    # ── Combined detect + localize ──────────────────────────────────

    def detect_and_localize(self, color):
        """
        Detect a colored object and return its 3D position in base_link.

        Args:
            color: Color name ('red', 'green', 'orange', 'yellow').

        Returns:
            Tuple of ((u, v), PointStamped_in_base_link), or None if
            the object was not found or localization failed.
        """
        centroid = self.detect_color(color)
        if centroid is None:
            return None

        u, v = centroid
        base_pt = self.pixel_to_base_link(u, v)
        if base_pt is None:
            return None

        return (centroid, base_pt)
