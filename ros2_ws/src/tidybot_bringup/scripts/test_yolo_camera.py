#!/usr/bin/env python3
"""
YOLO Camera Test

Tilts the camera down 40 degrees, waits for the image to settle,
then runs YOLO detection and prints what it sees.

Usage:
    # Terminal 1: Start simulation
    ros2 launch tidybot_bringup sim.launch.py scene:=scene_rgbd_grasp.xml

    # Terminal 2: Run this test
    python3 ros2_ws/src/tidybot_bringup/scripts/test_yolo_camera.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import numpy as np
import time
from pathlib import Path

from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

from cv_bridge import CvBridge
import cv2

from tidybot_perception.yolo_object_detector import YOLOObjectDetector


TILT_DOWN_RAD = np.radians(40) 
SETTLE_TIME = 2.0  # seconds to wait after tilting


class TestYoloCamera(Node):
    """Tilt camera down and run YOLO detection."""

    def __init__(self):
        super().__init__('test_yolo_camera')

        # Pan-tilt publisher
        self.pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10)

        # YOLO detector (subscribes to camera topics internally)
        self.detector = YOLOObjectDetector(self)

        # For saving annotated image
        self.cv_bridge = CvBridge()
        self.output_dir = Path(__file__).parent.parent / 'captures'
        self.output_dir.mkdir(exist_ok=True)

        # State
        self.tilt_sent_time = None
        self.done = False

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('=' * 50)
        self.get_logger().info('YOLO Camera Test')
        self.get_logger().info(f'Tilting camera down {abs(np.degrees(TILT_DOWN_RAD)):.0f} degrees')
        self.get_logger().info('=' * 50)

    def control_loop(self):
        if self.done:
            return

        # Wait for camera frames
        if self.detector.latest_rgb is None:
            return

        # Send tilt command
        if self.tilt_sent_time is None:
            msg = Float64MultiArray()
            msg.data = [0.0, float(TILT_DOWN_RAD)]
            self.pan_tilt_pub.publish(msg)
            self.tilt_sent_time = time.time()
            self.get_logger().info('Camera tilt command sent, waiting to settle...')
            return

        # Keep publishing tilt to make sure it takes
        msg = Float64MultiArray()
        msg.data = [0.0, float(TILT_DOWN_RAD)]
        self.pan_tilt_pub.publish(msg)

        # Wait for settle
        if time.time() - self.tilt_sent_time < SETTLE_TIME:
            return

        self.get_logger().info('Running YOLO on current camera frame...')
        self.get_logger().info('')

        # Run YOLO on the raw frame to see ALL detections
        if self.detector.model is not None and self.detector.latest_rgb is not None:
            results = self.detector.model(
                self.detector.latest_rgb, conf=0.3, verbose=False)

            detection_count = 0
            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    cls_name = self.detector.model.names[cls_id]
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                    self.get_logger().info(
                        f'  [{conf:.2f}] {cls_name:20s} at pixel ({cx}, {cy})')
                    detection_count += 1

            if detection_count == 0:
                self.get_logger().info('  No COCO objects detected.')
                self.get_logger().info('  (MuJoCo geometric primitives are not in COCO)')

            self.get_logger().info(f'\nTotal detections: {detection_count}')

            # Save annotated image
            annotated = results[0].plot() if results else self.detector.latest_rgb
            filename = self.output_dir / 'yolo_test.png'
            cv2.imwrite(str(filename), annotated)
            self.get_logger().info(f'Annotated image saved to: {filename}')

        # Also test the detect() API for specific classes
        self.get_logger().info('')
        self.get_logger().info('Testing detect() API for specific classes:')
        for cls in ['sports ball', 'banana', 'apple', 'cup', 'orange', 'bottle']:
            result = self.detector.detect(cls)
            status = f'pixel ({result[0]}, {result[1]})' if result else 'not found'
            self.get_logger().info(f'  {cls:15s} -> {status}')

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Test complete!')
        self.get_logger().info('=' * 50)
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = TestYoloCamera()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
