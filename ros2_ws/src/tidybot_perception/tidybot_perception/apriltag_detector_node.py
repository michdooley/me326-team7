#!/usr/bin/env python3
"""
AprilTag Detector Node for TidyBot2.

Detects AprilTags (tag36h11 family) in camera frames and publishes
detections as Detection2DArray messages on /apriltag_detections.

Each detection includes:
  - class_id: tag ID as string (e.g., "0", "1", "2")
  - score: detection decision margin (confidence proxy)
  - bbox: bounding box from tag corners (center + size)

Usage:
  ros2 run tidybot_perception apriltag_detector_node
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from cv_bridge import CvBridge
from pupil_apriltags import Detector


class AprilTagDetectorNode(Node):
    INFERENCE_RATE = 10.0  # Hz — max detections per second

    def __init__(self):
        super().__init__('apriltag_detector')

        self.bridge = CvBridge()
        self.last_inference_time = 0.0

        # Initialize AprilTag detector (tag36h11 family)
        self.detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,    # No decimation for better detection at range
            quad_sigma=0.0,       # No blur
            decode_sharpening=0.25,
        )

        # Publisher for detections
        self.det_pub = self.create_publisher(
            Detection2DArray, '/apriltag_detections', 10)

        # Subscribe to camera with RELIABLE QoS to match MuJoCo bridge
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self._image_cb, qos)

        self.get_logger().info(
            'AprilTag detector started (tag36h11), publishing to /apriltag_detections')

    def _image_cb(self, msg: Image):
        # Rate-limit to avoid excessive CPU usage
        now = time.monotonic()
        if now - self.last_inference_time < 1.0 / self.INFERENCE_RATE:
            return
        self.last_inference_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        detections = self.detector.detect(gray)

        # Build Detection2DArray message
        det_array = Detection2DArray()
        det_array.header = msg.header

        for det in detections:
            d = Detection2D()

            # Bounding box from tag corners
            corners = det.corners  # 4x2 array of corner pixels
            xs = corners[:, 0]
            ys = corners[:, 1]
            x_min, x_max = float(np.min(xs)), float(np.max(xs))
            y_min, y_max = float(np.min(ys)), float(np.max(ys))

            d.bbox.center.position.x = float(det.center[0])
            d.bbox.center.position.y = float(det.center[1])
            d.bbox.size_x = x_max - x_min
            d.bbox.size_y = y_max - y_min

            # Tag ID as class_id, decision margin as score
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(det.tag_id)
            hyp.hypothesis.score = float(det.decision_margin)
            d.results.append(hyp)

            det_array.detections.append(d)

        self.det_pub.publish(det_array)

        if detections:
            tag_ids = [d.tag_id for d in detections]
            self.get_logger().debug(f'Detected tags: {tag_ids}')


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
