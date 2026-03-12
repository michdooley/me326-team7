#!/usr/bin/env python3
"""
Grasp calibration test — verify depth back-projection accuracy.

Place an object at a known measured position relative to the robot,
run this script, and compare computed vs actual position.

Usage:
    ros2 run tidybot_bringup test_grasp_calibration.py

Requires: classifier node (for /objbbox), camera, and TF to be running.
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration

import tf2_ros
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge

# YOLO class IDs
YOLO_CLASSES = {
    'banana': 46, 'apple': 47, 'cup': 41, 'bottle': 39,
    'sports ball': 32, 'cell phone': 67, 'book': 73,
}


class GraspCalibrationTest(Node):

    def __init__(self):
        super().__init__('grasp_calibration_test')

        self.camera_K = None
        self.latest_depth = None
        self.latest_det = None
        self.cv_bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._info_cb, qos)
        self.create_subscription(
            Image, '/camera/depth/image_raw', self._depth_cb, qos)
        self.create_subscription(
            Detection2DArray, '/objbbox', self._det_cb, 10)

        self.get_logger().info('Grasp calibration test ready.')
        self.get_logger().info('Place an object in view and press Enter...')

    def _info_cb(self, msg):
        self.camera_K = np.array(msg.k).reshape(3, 3)

    def _depth_cb(self, msg):
        self.latest_depth = self.cv_bridge.imgmsg_to_cv2(msg, '16UC1')

    def _det_cb(self, msg):
        self.latest_det = msg

    def run(self):
        target = 'banana'
        target_id = str(YOLO_CLASSES.get(target, -1))

        self.get_logger().info(f'Looking for "{target}" (class {target_id})...')
        self.get_logger().info('Will sample 10 readings over 10 seconds.\n')

        readings = []
        for i in range(10):
            # Spin to get fresh data
            deadline = time.time() + 1.0
            while time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)

            if self.camera_K is None:
                self.get_logger().warn(f'  [{i+1}] No camera_info yet')
                continue
            if self.latest_depth is None:
                self.get_logger().warn(f'  [{i+1}] No depth image yet')
                continue
            if self.latest_det is None:
                self.get_logger().warn(f'  [{i+1}] No YOLO detections yet')
                continue

            # Find target detection
            det = None
            for d in self.latest_det.detections:
                for r in d.results:
                    if r.hypothesis.class_id == target_id:
                        if det is None or r.hypothesis.score > det[1]:
                            det = (d, r.hypothesis.score)
            if det is None:
                self.get_logger().warn(f'  [{i+1}] No "{target}" detected')
                continue

            d = det[0]
            u = int(d.bbox.center.position.x)
            v = int(d.bbox.center.position.y)

            # Sample depth (11x11 patch median)
            h, w = self.latest_depth.shape
            r = 5
            u0, u1 = max(0, u - r), min(w, u + r + 1)
            v0, v1 = max(0, v - r), min(h, v + r + 1)
            patch = self.latest_depth[v0:v1, u0:u1].astype(np.float32)
            valid = patch[patch > 100]  # >100mm
            if len(valid) == 0:
                self.get_logger().warn(f'  [{i+1}] No valid depth at ({u}, {v})')
                continue
            depth_m = float(np.median(valid)) / 1000.0

            # Back-project to camera frame
            fx = self.camera_K[0, 0]
            fy = self.camera_K[1, 1]
            cx = self.camera_K[0, 2]
            cy = self.camera_K[1, 2]
            x_cam = (u - cx) * depth_m / fx
            y_cam = (v - cy) * depth_m / fy
            z_cam = depth_m

            # Transform to base_link
            try:
                tf = self.tf_buffer.lookup_transform(
                    'base_link', 'camera_color_optical_frame',
                    rclpy.time.Time(), timeout=Duration(seconds=0.5))
            except Exception as e:
                self.get_logger().warn(f'  [{i+1}] TF error: {e}')
                continue

            q = tf.transform.rotation
            t = tf.transform.translation
            # Quaternion to rotation matrix
            qw, qx, qy, qz = q.w, q.x, q.y, q.z
            R = np.array([
                [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)],
            ])
            pt_cam = np.array([x_cam, y_cam, z_cam])
            pt_base = R @ pt_cam + np.array([t.x, t.y, t.z])

            readings.append(pt_base)
            self.get_logger().info(
                f'  [{i+1}] pixel=({u},{v}) depth={depth_m:.3f}m '
                f'-> base_link=({pt_base[0]:.4f}, {pt_base[1]:.4f}, {pt_base[2]:.4f})')

        if not readings:
            self.get_logger().error('No valid readings collected!')
            return

        pts = np.array(readings)
        mean = pts.mean(axis=0)
        std = pts.std(axis=0)

        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'CALIBRATION RESULTS ({len(readings)} readings)')
        self.get_logger().info(f'  Mean position in base_link:')
        self.get_logger().info(f'    X = {mean[0]:.4f}m  (std {std[0]:.4f})')
        self.get_logger().info(f'    Y = {mean[1]:.4f}m  (std {std[1]:.4f})')
        self.get_logger().info(f'    Z = {mean[2]:.4f}m  (std {std[2]:.4f})')
        self.get_logger().info('')
        self.get_logger().info(
            f'  Camera intrinsics (color): '
            f'fx={self.camera_K[0,0]:.1f} fy={self.camera_K[1,1]:.1f} '
            f'cx={self.camera_K[0,2]:.1f} cy={self.camera_K[1,2]:.1f}')
        self.get_logger().info('')
        self.get_logger().info(
            '  Compare these values with a tape measure from')
        self.get_logger().info(
            '  the robot base center to the object.')
        self.get_logger().info(
            '  X = left/right, Y = forward (negative), Z = height')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = GraspCalibrationTest()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
