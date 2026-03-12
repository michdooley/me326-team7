#!/usr/bin/env python3
"""
Perception Overlay Node — composites YOLO, face, and AprilTag detections
onto the raw camera feed and publishes a single annotated image.

Subscribes:
  /camera/color/image_raw      (sensor_msgs/Image)
  /objbbox                     (vision_msgs/Detection2DArray)
  /face_detections             (vision_msgs/Detection2DArray)
  /apriltag_detections         (vision_msgs/Detection2DArray)

Publishes:
  /camera/color/image_annotated (sensor_msgs/Image)
"""

import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge

# COCO class IDs used by the YOLO classifier node
COCO_NAMES = {
    '0': 'person', '39': 'bottle', '41': 'cup',
    '46': 'banana', '47': 'apple', '49': 'orange',
}

# Colors (BGR)
COLOR_YOLO = (0, 255, 0)       # green
COLOR_FACE = (255, 160, 0)     # blue-ish
COLOR_APRILTAG = (0, 0, 255)   # red


class PerceptionOverlayNode(Node):

    def __init__(self):
        super().__init__('perception_overlay_node')

        self.declare_parameter('staleness_timeout', 2.0)
        self.staleness_timeout = self.get_parameter('staleness_timeout').value

        self.bridge = CvBridge()

        # Cached detections
        self._yolo = None
        self._yolo_time = 0.0
        self._faces = None
        self._faces_time = 0.0
        self._tags = None
        self._tags_time = 0.0

        # Subscribers
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Image, '/camera/color/image_raw', self._image_cb, qos)
        self.create_subscription(
            Detection2DArray, '/objbbox', self._yolo_cb, 10)
        self.create_subscription(
            Detection2DArray, '/face_detections', self._face_cb, 10)
        self.create_subscription(
            Detection2DArray, '/apriltag_detections', self._tag_cb, 10)

        # Publisher
        self.image_pub = self.create_publisher(
            Image, '/camera/color/image_annotated', 10)

        self.get_logger().info('Perception overlay node started')

    # -- Detection callbacks (just cache) --
    def _yolo_cb(self, msg):
        self._yolo = msg
        self._yolo_time = time.monotonic()

    def _face_cb(self, msg):
        self._faces = msg
        self._faces_time = time.monotonic()

    def _tag_cb(self, msg):
        self._tags = msg
        self._tags_time = time.monotonic()

    # -- Image callback --
    def _image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        now = time.monotonic()

        if self._yolo and (now - self._yolo_time) < self.staleness_timeout:
            self._draw(frame, self._yolo, COLOR_YOLO, self._yolo_label)
        if self._faces and (now - self._faces_time) < self.staleness_timeout:
            self._draw(frame, self._faces, COLOR_FACE, self._face_label)
        if self._tags and (now - self._tags_time) < self.staleness_timeout:
            self._draw(frame, self._tags, COLOR_APRILTAG, self._tag_label)

        out = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        out.header = msg.header
        self.image_pub.publish(out)

    # -- Drawing --
    def _draw(self, frame, det_array, color, label_fn):
        for det in det_array.detections:
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            w = det.bbox.size_x
            h = det.bbox.size_y
            x1, y1 = int(cx - w / 2), int(cy - h / 2)
            x2, y2 = int(cx + w / 2), int(cy + h / 2)

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            label = label_fn(det)
            # Background for text readability
            (tw, th), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(
                frame, (x1, y1 - th - 6), (x1 + tw, y1), color, -1)
            cv2.putText(
                frame, label, (x1, y1 - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    # -- Label formatters --
    @staticmethod
    def _yolo_label(det):
        hyp = det.results[0].hypothesis if det.results else None
        if not hyp:
            return '?'
        class_id = hyp.class_id
        name = COCO_NAMES.get(class_id, f'cls{class_id}')
        return f'{name} {hyp.score:.2f}'

    @staticmethod
    def _face_label(det):
        hyp = det.results[0].hypothesis if det.results else None
        if not hyp:
            return 'face'
        return f'{hyp.class_id} {hyp.score:.2f}'

    @staticmethod
    def _tag_label(det):
        hyp = det.results[0].hypothesis if det.results else None
        if not hyp:
            return 'tag'
        return f'Tag {hyp.class_id}'


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionOverlayNode()
    rclpy.spin(node)
    rclpy.shutdown()
