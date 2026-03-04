#!/usr/bin/env python3
import os
import time

# Limit PyTorch/OpenMP to 1 thread so YOLO inference doesn't
# saturate all CPU cores and starve the MuJoCo bridge process.
os.environ.setdefault('OMP_NUM_THREADS', '1')
os.environ.setdefault('MKL_NUM_THREADS', '1')

import torch
torch.set_num_threads(1)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class ObjectClassifier(Node):
    INFERENCE_RATE = 2.0  # Hz — max YOLO inferences per second

    def __init__(self):
        super().__init__('object_classifier')
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()
        self.last_inference_time = 0.0

        # Publisher for bounding boxes
        self.bbox_pub = self.create_publisher(Detection2DArray, '/objbbox', 10)

        # Publisher for annotated image (viewable in RViz)
        self.yolo_image_pub = self.create_publisher(Image, '/camera/color/image_yolo', 10)

        # Subscribe with depth=1 to only keep latest frame
        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.listener_callback, be)

    def listener_callback(self, msg):
        # Rate-limit inference to avoid starving other nodes of CPU
        now = time.monotonic()
        if now - self.last_inference_time < 1.0 / self.INFERENCE_RATE:
            return
        self.last_inference_time = now

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(frame, classes=[0, 46, 47], conf=0.5, verbose=False)

        # Initialize the array message
        detection_array = Detection2DArray()
        detection_array.header = msg.header # Sync timestamp and frame_id with the image

        for r in results:
            for box in r.boxes:
                # 1. Create individual detection
                detection = Detection2D()
                
                # 2. Set Bounding Box (YOLO uses [x1, y1, x2, y2], vision_msgs uses center + size)
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                detection.bbox.center.position.x = float((x1 + x2) / 2.0)
                detection.bbox.center.position.y = float((y1 + y2) / 2.0)
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)

                # 3. Set Class ID and Confidence
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(int(box.cls[0]))
                hypothesis.hypothesis.score = float(box.conf[0])
                detection.results.append(hypothesis)

                # Add to the array
                detection_array.detections.append(detection)

        # Publish the final list of boxes
        self.bbox_pub.publish(detection_array)

        # Publish YOLO-annotated image for RViz
        annotated_frame = results[0].plot()
        yolo_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        yolo_msg.header = msg.header
        self.yolo_image_pub.publish(yolo_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectClassifier()
    rclpy.spin(node)
    rclpy.shutdown()

