#!/usr/bin/env python3
"""
Camera Scanner — HSV-based colored cube detection

Sweeps the pan-tilt camera downward in 5-degree increments across a range
of pan angles, returning the first frame in which the requested colour is
detected with sufficient blob area.

Sign conventions (empirically verified):
  - positive tilt = look DOWN
  - positive pan  = look LEFT
  - tilt hard limit = 1.309 rad; commanded values clamped to <= 1.2

Topics:
    /camera/pan_tilt_cmd        Float64MultiArray [pan, tilt]
    /camera/color/image_raw     sensor_msgs/Image  (RGB)
    /camera/depth/image_raw     sensor_msgs/Image  (uint16 mm)
    /camera/color/camera_info   sensor_msgs/CameraInfo

Usage (standalone):
    ros2 run tidybot_bringup camera_scanner.py --ros-args -p color:=red

Usage (as a library):
    scanner = CameraScanner(node)
    det = scanner.scan_for_object('red')
    if det:
        cx, cy = det.pixel_cx, det.pixel_cy
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray

try:
    from cv_bridge import CvBridge
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


# ── Detection result ──────────────────────────────────────────────────────────

@dataclass
class Detection:
    pixel_cx:    int
    pixel_cy:    int
    bbox:        Tuple[int, int, int, int]   # (x, y, w, h) in pixels
    depth_image: np.ndarray                  # uint16, mm
    camera_info: CameraInfo
    color:       str


# ── HSV colour ranges (H 0-179, S 0-255, V 0-255) ────────────────────────────

COLOR_RANGES = {
    'red': [
        ((0,   100, 80),  (10,  255, 255)),
        ((165, 100, 80),  (180, 255, 255)),
    ],
    'green': [
        ((40,  60,  60),  (85,  255, 255)),
    ],
    'blue': [
        ((100, 80,  60),  (135, 255, 255)),
    ],
}

# Minimum contour area to count as a real detection.
MIN_BLOB_AREA_PX = 150

# Sweep parameters
TILT_START_DEG  = 20.0   # starting tilt (degrees) — slight downward angle
TILT_END_DEG    = 68.0   # max tilt (degrees); hard limit is 1.309 rad = 75 deg
TILT_STEP_DEG   =  5.0   # tilt increment per step

# Pan positions to sweep at (degrees). 0 = forward, + = left, - = right.
PAN_POSITIONS_DEG = [0.0, 17.0, -17.0]

# Time (s) to wait after each pan/tilt command before sampling the image.
STEP_SETTLE = 0.5


# ── Main scanner class ────────────────────────────────────────────────────────

class CameraScanner:
    """
    Incremental tilt-sweep camera scanner.

    For each pan position, tilts downward in TILT_STEP_DEG increments and
    checks for the target colour after each step.  Stops and returns on the
    first successful detection.
    """

    def __init__(self, node: Node):
        if not CV_AVAILABLE:
            raise RuntimeError(
                'cv_bridge / opencv not available — install opencv-python'
            )

        self.node   = node
        self.logger = node.get_logger()
        self.bridge = CvBridge()

        self._latest_rgb:   Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None
        self._camera_info:  Optional[CameraInfo] = None

        qos_be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(Image, '/camera/color/image_raw',
                                 self._rgb_cb, qos_be)
        node.create_subscription(Image, '/camera/depth/image_raw',
                                 self._depth_cb, qos_be)
        node.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 self._camera_info_cb, 10)

        self._pan_tilt_pub = node.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _rgb_cb(self, msg: Image) -> None:
        try:
            self._latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        except Exception as e:
            self.logger.warn(f'RGB decode: {e}')

    def _depth_cb(self, msg: Image) -> None:
        try:
            self._latest_depth = self.bridge.imgmsg_to_cv2(msg, '16UC1')
        except Exception as e:
            self.logger.warn(f'Depth decode: {e}')

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    # ── Utilities ─────────────────────────────────────────────────────────────

    def _spin(self, duration: float) -> None:
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _cmd(self, pan_deg: float, tilt_deg: float) -> None:
        """Send a pan/tilt command (degrees) and wait STEP_SETTLE seconds."""
        pan  = float(np.radians(pan_deg))
        tilt = float(np.radians(tilt_deg))
        # Safety clamp
        pan  = max(-0.6, min(0.6,  pan))
        tilt = max(0.0,  min(1.2,  tilt))
        msg = Float64MultiArray()
        msg.data = [pan, tilt]
        self._pan_tilt_pub.publish(msg)
        self._spin(STEP_SETTLE)

    # ── Detection ─────────────────────────────────────────────────────────────

    def _detect_color(
        self, rgb: np.ndarray, color: str
    ) -> Optional[Tuple[int, int, Tuple[int, int, int, int]]]:
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in COLOR_RANGES[color]:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < MIN_BLOB_AREA_PX:
            return None

        x, y, w, h = cv2.boundingRect(largest)
        return int(x + w / 2), int(y + h / 2), (x, y, w, h)

    # ── Main API ──────────────────────────────────────────────────────────────

    def scan_for_object(
        self, color: str, **_kwargs
    ) -> Optional[Detection]:
        """
        Point camera at pan=0, tilt=0.6 rad (~34 deg down), then detect
        the requested colour in the current frame.
        """
        if color not in COLOR_RANGES:
            raise ValueError(f'Unknown color {color!r}; choose {list(COLOR_RANGES)}')

        self.logger.info(f'Looking for {color} cube ...')

        # Wait for camera to come online
        self._spin(3.0)
        if self._latest_rgb is None or self._camera_info is None:
            self.logger.error('Camera not online after 3 s')
            return None

        # Point camera: pan=0 (forward), tilt=0.6 rad (~34 deg down)
        TILT_RAD = 0.6
        msg = Float64MultiArray()
        msg.data = [0.0, TILT_RAD]
        self._pan_tilt_pub.publish(msg)
        self.logger.info(f'Camera → pan=0.0, tilt={TILT_RAD} rad')
        self._spin(1.0)  # wait for camera to settle

        rgb   = self._latest_rgb
        depth = self._latest_depth
        ci    = self._camera_info
        if rgb is None or depth is None or ci is None:
            self.logger.error('Missing RGB/depth/camera_info after settling')
            return None

        result = self._detect_color(rgb, color)
        if result is None:
            self.logger.warn(f'Could not find {color} in current frame.')
            return None

        cx, cy, bbox = result
        self.logger.info(
            f'FOUND {color}!  centroid=({cx},{cy}), bbox={bbox}'
        )

        return Detection(
            pixel_cx=cx,
            pixel_cy=cy,
            bbox=bbox,
            depth_image=depth.copy(),
            camera_info=ci,
            color=color,
        )

    
    def tilt_simple(self):
        # Wait for camera to come online
        self._spin(3.0)
        if self._latest_rgb is None or self._camera_info is None:
            self.logger.error('Camera not online after 3 s')
            return None

        # Point camera: pan=0 (forward), tilt=0.6 rad (~34 deg down)
        TILT_RAD = 0.55
        msg = Float64MultiArray()
        msg.data = [0.0, TILT_RAD]
        self._pan_tilt_pub.publish(msg)
        self.logger.info(f'Camera → pan=0.0, tilt={TILT_RAD} rad')
        self._spin(1.0)  # wait for camera to settle

# ── Standalone node ───────────────────────────────────────────────────────────

class CameraScannerNode(Node):
    def __init__(self):
        super().__init__('camera_scanner_node')
        self.declare_parameter('color', 'red')
        self._color   = self.get_parameter('color').value
        self._scanner = CameraScanner(self)
        self.get_logger().info(
            f'CameraScanner: scanning for {self._color!r} ...'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CameraScannerNode()
    try:
        node._scanner.tilt_simple()

        # det = node._scanner.scan_for_object(node._color)
        # if det:
        #     node.get_logger().info(
        #         f'Detection: pixel=({det.pixel_cx},{det.pixel_cy}), '
        #         f'bbox={det.bbox}, color={det.color}'
        #     )
        # else:
        #     node.get_logger().warn('No detection found.')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
