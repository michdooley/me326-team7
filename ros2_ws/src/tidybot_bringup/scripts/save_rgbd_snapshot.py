#!/usr/bin/env python3
"""
Save a single RGB-D snapshot from the running simulation in GraspNet format.

Captures one frame from the camera topics and writes:
    color.png          — 8-bit RGB image
    depth.png          — 16-bit depth (millimetres)
    workspace_mask.png — binary mask (255 where depth > 0)
    meta.mat           — intrinsic_matrix (3x3) + factor_depth

Usage:
    source ros2_ws/setup_env.bash
    python ros2_ws/src/tidybot_bringup/scripts/save_rgbd_snapshot.py

    # Custom output directory:
    python save_rgbd_snapshot.py --output_dir /tmp/my_scene
"""

import os
import argparse
import numpy as np
import scipy.io as scio
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image as RosImage, CameraInfo
from cv_bridge import CvBridge


class SnapshotNode(Node):
    def __init__(self, output_dir):
        super().__init__('rgbd_snapshot')
        self.output_dir = output_dir
        self.bridge = CvBridge()

        self._color = None
        self._depth = None
        self._camera_info = None

        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(
            RosImage, '/camera/color/image_raw', self._color_cb, best_effort)
        self.create_subscription(
            RosImage, '/camera/depth/image_raw', self._depth_cb, best_effort)
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._info_cb, 10)

        self.timer = self.create_timer(0.1, self._check_done)
        self.get_logger().info('Waiting for color + depth + camera_info ...')

    def _color_cb(self, msg):
        if self._color is None:
            self._color = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            self.get_logger().info(
                f'Got color: {self._color.shape} {self._color.dtype}')

    def _depth_cb(self, msg):
        if self._depth is None:
            self._depth = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            self.get_logger().info(
                f'Got depth: {self._depth.shape} {self._depth.dtype} '
                f'range=[{self._depth.min()}, {self._depth.max()}]')

    def _info_cb(self, msg):
        if self._camera_info is None:
            self._camera_info = msg
            K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f'Got camera_info: fx={K[0,0]:.1f} fy={K[1,1]:.1f}')

    def _check_done(self):
        if self._color is None or self._depth is None or self._camera_info is None:
            return

        self.timer.cancel()
        self._save()
        raise SystemExit(0)

    def _save(self):
        os.makedirs(self.output_dir, exist_ok=True)

        # color.png — 8-bit RGB
        color_path = os.path.join(self.output_dir, 'color.png')
        Image.fromarray(self._color).save(color_path)

        # depth.png — 16-bit uint (millimetres)
        depth_path = os.path.join(self.output_dir, 'depth.png')
        Image.fromarray(self._depth).save(depth_path)

        # workspace_mask.png — 255 wherever depth is valid
        mask = (self._depth > 0).astype(np.uint8) * 255
        mask_path = os.path.join(self.output_dir, 'workspace_mask.png')
        Image.fromarray(mask).save(mask_path)

        # meta.mat — intrinsic matrix + depth scale
        K = np.array(self._camera_info.k).reshape(3, 3)
        meta_path = os.path.join(self.output_dir, 'meta.mat')
        scio.savemat(meta_path, {
            'intrinsic_matrix': K,
            'factor_depth': np.array([[1000.0]]),  # depth is in mm
        })

        self.get_logger().info(f'Saved to {self.output_dir}/')
        self.get_logger().info(f'  color.png          {self._color.shape}')
        self.get_logger().info(f'  depth.png          {self._depth.shape}  '
                               f'[{self._depth.min()}-{self._depth.max()}] mm')
        self.get_logger().info(f'  workspace_mask.png {mask.shape}  '
                               f'{np.count_nonzero(mask):,} valid px')
        self.get_logger().info(f'  meta.mat           fx={K[0,0]:.1f} fy={K[1,1]:.1f}')
        self.get_logger().info('')
        self.get_logger().info('Run GraspNet debug:')
        self.get_logger().info(
            f'  python ros2_ws/src/tidybot_bringup/scripts/debug_graspnet.py '
            f'--data_dir {os.path.abspath(self.output_dir)}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output_dir', default='graspnet_snapshot',
                        help='Directory to save the snapshot files')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = SnapshotNode(args.output_dir)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
