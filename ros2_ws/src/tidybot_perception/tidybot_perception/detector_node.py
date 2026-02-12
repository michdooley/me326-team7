#!/usr/bin/env python3
"""
YOLO Object Detection Node

Runs YOLO inference on camera frames and publishes detections.

Subscribes:
    /camera/color/image_raw (sensor_msgs/Image) - RGB camera feed

Publishes:
    /detections (tidybot_msgs/Detection2DArray) - continuous detections @ publish_rate Hz

Services:
    /detect (tidybot_msgs/Detect) - on-demand detection with optional class filter

Parameters:
    model_path (str): path to YOLO weights file
    confidence_threshold (float): minimum detection confidence (default 0.5)
    publish_rate (float): max publishing rate in Hz (default 10.0)
    device (str): inference device, "cpu" or "cuda:0" (default "cpu")

Usage:
    ros2 run tidybot_perception detector_node
    ros2 run tidybot_perception detector_node --ros-args -p model_path:=/path/to/yolov8n.pt
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from tidybot_msgs.msg import Detection2D, Detection2DArray
from tidybot_msgs.srv import Detect


class DetectorNode(Node):
    """YOLO-based object detection node."""

    def __init__(self):
        super().__init__('detector_node')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('device', 'cpu')

        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.device = self.get_parameter('device').value

        # Subscriber for camera images
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, qos
        )

        # Publisher for continuous detections
        self.detections_pub = self.create_publisher(Detection2DArray, '/detections', 10)

        # Service for on-demand detection
        self.detect_srv = self.create_service(Detect, '/detect', self.detect_callback)

        # Internal state
        self.latest_image = None
        self.model = None

        # Load model
        self._load_model()

        self.get_logger().info('DetectorNode initialized')
        if not self.model_path:
            self.get_logger().warn('No model_path set — detection will not run until configured')

    def _load_model(self):
        """Load the YOLO model. Override this with actual YOLO loading."""
        if not self.model_path:
            return
        # TODO: Load YOLO model
        # from ultralytics import YOLO
        # self.model = YOLO(self.model_path)
        # self.model.to(self.device)
        self.get_logger().info(f'Model loaded from {self.model_path} on {self.device}')

    def image_callback(self, msg: Image):
        """Store latest image and run detection."""
        self.latest_image = msg
        # TODO: Run YOLO inference on the image
        # TODO: Publish Detection2DArray to /detections

    def detect_callback(self, request: Detect.Request, response: Detect.Response):
        """Handle on-demand detection service call."""
        if self.latest_image is None:
            response.success = False
            response.message = 'No image received yet'
            return response

        if self.model is None:
            response.success = False
            response.message = 'Model not loaded — set model_path parameter'
            return response

        # TODO: Run inference on latest_image
        # TODO: Filter by request.target_class if non-empty
        # TODO: Populate response.detections

        response.success = True
        response.detections = []
        response.message = 'OK'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
