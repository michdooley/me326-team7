#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

# ── Configuration ──────────────────────────────────────────────────────
YOLO_CONFIDENCE = 0.01          # minimum detection confidence (0.0–1.0)
YOLO_CLASSES = [0, 46, 47, 52, 53, 55]     # COCO class IDs: person=0, banana=46, apple=47

YOLO_MODEL_HEAVY = 'yolov8m.pt'  # used when scanning (stationary, need accuracy)
YOLO_MODEL_LIGHT = 'yolov8n.pt'  # used when moving / object already found
YOLO_IMGSZ_HEAVY = 1280          # inference resolution for heavy model
YOLO_IMGSZ_LIGHT = 640           # inference resolution for light model

class ObjectClassifier(Node):
    def __init__(self):
        super().__init__('object_classifier')
        self.bridge = CvBridge()

        # Load both models at startup
        self.get_logger().info(f'Loading heavy model: {YOLO_MODEL_HEAVY}')
        self.model_heavy = YOLO(YOLO_MODEL_HEAVY)
        self.get_logger().info(f'Loading light model: {YOLO_MODEL_LIGHT}')
        self.model_light = YOLO(YOLO_MODEL_LIGHT)

        # Default to heavy model
        self.active_model = self.model_heavy
        self.active_imgsz = YOLO_IMGSZ_HEAVY
        self.active_mode = 'heavy'

        # Subscribe to mode selector: publish "heavy" or "light" to /yolo_mode
        self.create_subscription(String, '/yolo_mode', self._mode_cb, 10)

        # Publisher for bounding boxes
        self.bbox_pub = self.create_publisher(Detection2DArray, '/objbbox', 10)

        # Publisher for annotated image (viewable in RViz)
        self.yolo_image_pub = self.create_publisher(Image, '/camera/color/image_yolo', 10)

        self.subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.listener_callback, 10)

    def _mode_cb(self, msg):
        mode = msg.data.strip().lower()
        if mode == self.active_mode:
            return
        if mode == 'heavy':
            self.active_model = self.model_heavy
            self.active_imgsz = YOLO_IMGSZ_HEAVY
            self.active_mode = 'heavy'
            self.get_logger().info('Switched to heavy model (yolov8m)')
        elif mode == 'light':
            self.active_model = self.model_light
            self.active_imgsz = YOLO_IMGSZ_LIGHT
            self.active_mode = 'light'
            self.get_logger().info('Switched to light model (yolov8n)')

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.active_model(frame, classes=YOLO_CLASSES, conf=YOLO_CONFIDENCE,
                                    imgsz=self.active_imgsz, verbose=False)

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
