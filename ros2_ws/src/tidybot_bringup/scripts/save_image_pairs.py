#!/usr/bin/env python3
"""
Save synchronized RGB + Depth image pairs from the TidyBot2 camera.

Runs alongside the simulation (or real hardware). Press ENTER to capture
a pair, or use --auto to capture at a fixed interval.

Saves to: tidybot_bringup/captures/pair_NNNN/
  - rgb.png    (8-bit BGR PNG)
  - depth.png  (16-bit single-channel PNG, values in mm)
  - depth.npy  (raw uint16 numpy array for programmatic use)

Topics:
  /camera/color/image_raw  (sensor_msgs/Image, rgb8)
  /camera/depth/image_raw  (sensor_msgs/Image, 16UC1)

Usage:
    # Terminal 1: launch sim + rviz
    ros2 launch tidybot_bringup sim.launch.py

    # Terminal 2: run this script
    ros2 run tidybot_bringup save_image_pairs.py              # manual (ENTER to capture)
    ros2 run tidybot_bringup save_image_pairs.py --auto 2.0   # auto every 2 seconds
"""

import argparse
import threading
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

import numpy as np
import cv2
from cv_bridge import CvBridge


class ImagePairSaver(Node):
    def __init__(self, output_dir: Path, auto_interval: float | None):
        super().__init__('save_image_pairs')

        self.bridge = CvBridge()
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.pair_count = 0

        # Latest images (protected by lock)
        self._lock = threading.Lock()
        self._rgb = None
        self._depth = None

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Image, '/camera/color/image_raw', self._rgb_cb, qos)
        self.create_subscription(Image, '/camera/depth/image_raw', self._depth_cb, qos)

        if auto_interval is not None:
            self.create_timer(auto_interval, self.save_pair)
            self.get_logger().info(f'Auto-capture every {auto_interval:.1f}s')
        else:
            # Manual mode: listen for ENTER on a background thread
            self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
            self._input_thread.start()
            self.get_logger().info('Press ENTER to capture a pair (Ctrl-C to quit)')

        self.get_logger().info(f'Saving to: {self.output_dir}')
        self.get_logger().info('Waiting for RGB + Depth topics...')

    # ── subscribers ──────────────────────────────────────────────

    def _rgb_cb(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            with self._lock:
                self._rgb = img
        except Exception as e:
            self.get_logger().warn(f'RGB conversion failed: {e}')

    def _depth_cb(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')  # uint16
            with self._lock:
                self._depth = img
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')

    # ── capture ──────────────────────────────────────────────────

    def save_pair(self):
        with self._lock:
            rgb = self._rgb
            depth = self._depth

        if rgb is None or depth is None:
            missing = []
            if rgb is None:
                missing.append('RGB')
            if depth is None:
                missing.append('Depth')
            self.get_logger().warn(f'Skipping capture — missing: {", ".join(missing)}')
            return

        pair_dir = self.output_dir / f'pair_{self.pair_count:04d}'
        pair_dir.mkdir(exist_ok=True)

        # Save RGB as BGR PNG (standard OpenCV format)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(pair_dir / 'rgb.png'), bgr)

        # Save depth as 16-bit PNG (lossless, mm)
        cv2.imwrite(str(pair_dir / 'depth.png'), depth)

        # Save raw depth as .npy for easy programmatic access
        np.save(str(pair_dir / 'depth.npy'), depth)

        self.get_logger().info(
            f'[{self.pair_count:04d}] Saved to {pair_dir.name}/  '
            f'(rgb {rgb.shape[1]}x{rgb.shape[0]}, '
            f'depth {depth.shape[1]}x{depth.shape[0]} '
            f'range {depth.min()}-{depth.max()} mm)'
        )
        self.pair_count += 1

    # ── manual input ─────────────────────────────────────────────

    def _input_loop(self):
        try:
            while True:
                input()  # blocks until ENTER
                self.save_pair()
        except EOFError:
            pass


def main():
    parser = argparse.ArgumentParser(description='Save RGB + Depth image pairs')
    parser.add_argument('--auto', type=float, default=None, metavar='SEC',
                        help='Auto-capture interval in seconds (omit for manual ENTER mode)')
    parser.add_argument('--output', type=str, default=None,
                        help='Output directory (default: tidybot_bringup/captures)')
    # ROS2 passes extra args; ignore unknown
    args, _ = parser.parse_known_args()

    output_dir = Path(args.output) if args.output else (
        Path(__file__).resolve().parent.parent / 'captures'
    )

    rclpy.init()
    node = ImagePairSaver(output_dir, args.auto)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'Saved {node.pair_count} pairs total.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
