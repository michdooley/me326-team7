#!/usr/bin/env python3
"""
Face Recognition Node for TidyBot2.

Detects and identifies faces in camera frames and publishes results as
Detection2DArray messages on /face_detections.

Each detection includes:
  - class_id: person name (or "unknown" if face not recognized)
  - score: confidence (1.0 - face distance, so higher = more confident)
  - bbox: bounding box around detected face

Known faces are loaded from `known_faces_dir` at startup:
  - Subdirectories: each subdirectory name is a person's name, images inside
    are their reference photos (supports multiple people).
  - Root-level images: labeled with the `person_name` param (default "unknown_person").

Example usage:
  # Single person, flat directory of their photos:
  ros2 run tidybot_perception face_recognition_node --ros-args \\
    -p known_faces_dir:=/path/to/face_photos \\
    -p person_name:=michael

  # Multiple people, subdirectory structure:
  #   known_faces/michael/img1.jpg
  #   known_faces/alice/img1.jpg
  ros2 run tidybot_perception face_recognition_node --ros-args \\
    -p known_faces_dir:=/path/to/known_faces
"""

import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from cv_bridge import CvBridge

try:
    import face_recognition
    _FACE_RECOGNITION_AVAILABLE = True
except ImportError:
    _FACE_RECOGNITION_AVAILABLE = False

_IMAGE_EXTS = {'.jpg', '.jpeg', '.png', '.bmp'}


class FaceRecognitionNode(Node):
    INFERENCE_RATE = 10.0  # Hz — max detections per second

    def __init__(self):
        super().__init__('face_recognition_node')

        self.bridge = CvBridge()
        self.last_inference_time = 0.0

        if not _FACE_RECOGNITION_AVAILABLE:
            self.get_logger().fatal(
                'face_recognition library not found. Install with: pip install face-recognition')
            raise RuntimeError('face_recognition not installed')

        # Parameters
        self.declare_parameter('known_faces_dir', '')
        self.declare_parameter('person_name', 'unknown_person')
        self.declare_parameter('recognition_tolerance', 0.6)
        self.declare_parameter('inference_rate', self.INFERENCE_RATE)
        self.declare_parameter('model', 'hog')  # 'hog' (CPU) or 'cnn' (GPU)

        known_faces_dir = self.get_parameter('known_faces_dir').value
        self.person_name = self.get_parameter('person_name').value
        self.tolerance = self.get_parameter('recognition_tolerance').value
        self.INFERENCE_RATE = self.get_parameter('inference_rate').value
        self.model = self.get_parameter('model').value

        # Load known face encodings
        self.known_encodings: list[np.ndarray] = []
        self.known_names: list[str] = []
        if known_faces_dir:
            self._load_known_faces(known_faces_dir)
        else:
            self.get_logger().warn(
                'No known_faces_dir set — running in detection-only mode (class_id="face")')

        # Publisher for detections
        self.det_pub = self.create_publisher(
            Detection2DArray, '/face_detections', 10)

        # Subscribe to camera with RELIABLE QoS to match MuJoCo bridge
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self._image_cb, qos)

        mode_str = 'recognition' if self.known_encodings else 'detection-only'
        self.get_logger().info(
            f'Face recognition node started ({mode_str}, {len(self.known_encodings)} known faces), '
            f'publishing to /face_detections')

    def _load_known_faces(self, directory: str):
        """Load face encodings from a directory.

        Supports two layouts:
          1. Subdirectories: each subdir name = person name, images inside are refs.
          2. Flat: all images in root are labeled with `person_name` param.
        """
        if not os.path.isdir(directory):
            self.get_logger().error(f'known_faces_dir does not exist: {directory}')
            return

        loaded = 0

        # Check for subdirectories (multi-person layout)
        subdirs = [
            e for e in os.scandir(directory)
            if e.is_dir() and not e.name.startswith('.')
        ]
        if subdirs:
            for subdir in subdirs:
                person = subdir.name
                for entry in os.scandir(subdir.path):
                    if not entry.is_file():
                        continue
                    if os.path.splitext(entry.name)[1].lower() not in _IMAGE_EXTS:
                        continue
                    enc = self._encode_image(entry.path, person)
                    if enc is not None:
                        self.known_encodings.append(enc)
                        self.known_names.append(person)
                        loaded += 1

        # Also load root-level images, labeled with person_name param
        for entry in os.scandir(directory):
            if not entry.is_file():
                continue
            if os.path.splitext(entry.name)[1].lower() not in _IMAGE_EXTS:
                continue
            enc = self._encode_image(entry.path, self.person_name)
            if enc is not None:
                self.known_encodings.append(enc)
                self.known_names.append(self.person_name)
                loaded += 1

        self.get_logger().info(
            f'Loaded {loaded} reference face(s) for {len(set(self.known_names))} person(s): '
            f'{sorted(set(self.known_names))}')

    def _encode_image(self, path: str, person: str) -> 'np.ndarray | None':
        """Load an image file and return its first face encoding, or None."""
        img = face_recognition.load_image_file(path)
        encodings = face_recognition.face_encodings(img)
        if not encodings:
            self.get_logger().warn(f'No face found in reference image: {path} ({person})')
            return None
        return encodings[0]

    def _image_cb(self, msg: Image):
        # Rate-limit to avoid excessive CPU usage
        now = time.monotonic()
        if now - self.last_inference_time < 1.0 / self.INFERENCE_RATE:
            return
        self.last_inference_time = now

        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # face_recognition uses RGB
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        # Detect face locations (bounding boxes)
        locations = face_recognition.face_locations(frame_rgb, model=self.model)

        det_array = Detection2DArray()
        det_array.header = msg.header

        if not locations:
            self.det_pub.publish(det_array)
            return

        # Compute encodings only if we have known faces to compare against
        if self.known_encodings:
            encodings = face_recognition.face_encodings(frame_rgb, locations)
        else:
            encodings = [None] * len(locations)

        for (top, right, bottom, left), encoding in zip(locations, encodings):
            name, score = self._identify(encoding)

            d = Detection2D()
            d.header = msg.header

            d.bbox.center.position.x = float((left + right) / 2)
            d.bbox.center.position.y = float((top + bottom) / 2)
            d.bbox.size_x = float(right - left)
            d.bbox.size_y = float(bottom - top)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = score
            d.results.append(hyp)

            det_array.detections.append(d)

        self.det_pub.publish(det_array)

        if det_array.detections:
            names = [d.results[0].hypothesis.class_id for d in det_array.detections]
            self.get_logger().debug(f'Detected faces: {names}')

    def _identify(self, encoding) -> tuple[str, float]:
        """Match an encoding against known faces. Returns (name, confidence)."""
        if encoding is None or not self.known_encodings:
            return 'face', 1.0

        distances = face_recognition.face_distance(self.known_encodings, encoding)
        best_idx = int(np.argmin(distances))
        best_dist = float(distances[best_idx])

        if best_dist <= self.tolerance:
            return self.known_names[best_idx], round(1.0 - best_dist, 3)
        return 'unknown', round(1.0 - best_dist, 3)


def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
