#!/usr/bin/env python3
"""Capture synchronized RGB-D pairs from the running simulation.

Saves each pair into captures_sim/pair_XXXX/ with the same layout expected
by GraspNet (and matching the captures_real/ structure):
    pair_XXXX/
        rgb.png            – 640x480 BGR-encoded color image
        depth.png          – 640x480 16-bit depth (mm)
        depth.npy          – same depth as numpy array
        workspace_mask.png – pure-white 480x640 single-channel mask
        meta.mat           – camera intrinsics + factor_depth

Controls:
    Enter  – capture a pair
    q      – quit

Prerequisites:
    Terminal 1:  ros2 launch tidybot_bringup sim.launch.py
    Terminal 2:  source ros2_ws/setup_env.bash
                 python3 ros2_ws/src/tidybot_perception/test/capture_sim_pairs.py

Usage:
    python3 capture_sim_pairs.py [--output-dir PATH]
"""

import argparse
import sys
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

try:
    import scipy.io as sio
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

try:
    from cv_bridge import CvBridge
except ImportError:
    print("ERROR: cv_bridge not found. Source setup_env.bash first.")
    sys.exit(1)


class PairCapture(Node):
    def __init__(self, output_dir: Path):
        super().__init__("pair_capture")
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.cv_bridge = CvBridge()

        # Latest frames (guarded by lock)
        self.lock = threading.Lock()
        self.latest_rgb = None
        self.latest_depth = None
        self.intrinsics = None  # 3x3 numpy array

        # Figure out starting index from existing pair_XXXX dirs
        existing = [
            int(p.name.split("_")[1])
            for p in self.output_dir.iterdir()
            if p.is_dir() and p.name.startswith("pair_")
        ]
        self.pair_index = max(existing) + 1 if existing else 0

        # Subscribers
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, "/camera/color/image_raw", self._rgb_cb, qos)
        self.create_subscription(Image, "/camera/depth/image_raw", self._depth_cb, qos)
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self._info_cb, qos)

        # Pre-generate the workspace mask once
        self.mask = np.full((480, 640), 255, dtype=np.uint8)

        # Keyboard thread
        self._quit = False
        self._capture_requested = threading.Event()
        threading.Thread(target=self._keyboard_thread, daemon=True).start()

        # Process capture requests at 10 Hz
        self.create_timer(0.1, self._timer_cb)

        self.get_logger().info("=" * 50)
        self.get_logger().info("RGB-D Pair Capture (Simulation)")
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"Output: {self.output_dir}")
        self.get_logger().info(f"Next pair index: {self.pair_index}")
        self.get_logger().info("Press ENTER to capture, 'q' to quit.")
        self.get_logger().info("Waiting for camera topics...")

    # ---- ROS callbacks ----
    def _rgb_cb(self, msg: Image):
        try:
            img = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")
            with self.lock:
                self.latest_rgb = img
        except Exception as e:
            self.get_logger().warn(f"RGB conversion error: {e}")

    def _depth_cb(self, msg: Image):
        try:
            img = self.cv_bridge.imgmsg_to_cv2(msg, "16UC1")
            with self.lock:
                self.latest_depth = img
        except Exception as e:
            self.get_logger().warn(f"Depth conversion error: {e}")

    def _info_cb(self, msg: CameraInfo):
        K = np.array(msg.k).reshape(3, 3)
        with self.lock:
            self.intrinsics = K

    # ---- Keyboard thread ----
    def _keyboard_thread(self):
        while not self._quit:
            try:
                line = input()
            except EOFError:
                break
            if line.strip().lower() == "q":
                self._quit = True
                rclpy.shutdown()
                return
            self._capture_requested.set()

    # ---- Timer callback ----
    def _timer_cb(self):
        if self._quit:
            raise SystemExit

        if not self._capture_requested.is_set():
            return
        self._capture_requested.clear()

        with self.lock:
            rgb = self.latest_rgb
            depth = self.latest_depth
            K = self.intrinsics

        if rgb is None or depth is None:
            self.get_logger().warn("No images received yet — skipping capture.")
            return
        if K is None:
            self.get_logger().warn("No camera_info received yet — skipping capture.")
            return

        self._save_pair(rgb, depth, K)

    # ---- Save logic ----
    def _save_pair(self, rgb: np.ndarray, depth: np.ndarray, K: np.ndarray):
        pair_dir = self.output_dir / f"pair_{self.pair_index:04d}"
        pair_dir.mkdir(exist_ok=True)

        # rgb.png (OpenCV expects BGR)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(pair_dir / "rgb.png"), bgr)

        # depth.png + depth.npy
        cv2.imwrite(str(pair_dir / "depth.png"), depth)
        np.save(str(pair_dir / "depth.npy"), depth)

        # workspace_mask.png
        cv2.imwrite(str(pair_dir / "workspace_mask.png"), self.mask)

        # meta.mat
        if HAS_SCIPY:
            meta = {
                "intrinsic_matrix": K,
                "factor_depth": np.array([[1000]], dtype=np.float64),
            }
            sio.savemat(str(pair_dir / "meta.mat"), meta)
        else:
            # Fallback: save as .npz
            np.savez(
                str(pair_dir / "meta.npz"),
                intrinsic_matrix=K,
                factor_depth=np.array([[1000]], dtype=np.float64),
            )
            self.get_logger().warn("scipy not available — saved meta.npz instead of meta.mat")

        self.get_logger().info(
            f"Saved pair_{self.pair_index:04d}  "
            f"(rgb {rgb.shape[1]}x{rgb.shape[0]}, depth {depth.shape[1]}x{depth.shape[0]})"
        )
        self.pair_index += 1


def main():
    parser = argparse.ArgumentParser(description="Capture RGB-D pairs from sim")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "captures_sim",
        help="Root output directory (default: captures_sim/ next to this script)",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = PairCapture(args.output_dir)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.get_logger().info(f"Captured {node.pair_index} pairs total.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
