#!/usr/bin/env python3
"""
Depth-to-RGB alignment utility for RealSense cameras.

Aligns a depth image to an RGB image using camera intrinsics and an
extrinsic transform between the two sensors. Tested on the RealSense
D435 and L515. May need intrinsic adjustments for other cameras.

Usage as a library (from another script):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from align_depth_to_rgb import align_depth

    aligned = align_depth(depth, rgb)

Usage from the command line:
    python3 align_depth_to_rgb.py depth.png rgb.png aligned_depth.png

Based on reference code from ARM Lab @ Stanford.
"""

import numpy as np
import cv2
from typing import Tuple, Optional
import argparse

# Default D435 intrinsics (fx, fy, cx, cy) at 640x480
D435_DEPTH_K = (380.4629821777344, 380.4629821777344, 324.5294494628906, 241.70526123046875)
D435_RGB_K = (609.7716674804688, 610.3194580078125, 331.6385192871094, 247.25904846191406)


def convert_intrinsics(img: np.ndarray,
                       K_old: np.ndarray,
                       K_new: np.ndarray,
                       new_size: Tuple[int, int] = (1280, 720)) -> np.ndarray:
    """
    Re-project an image from one set of camera intrinsics to another.

    Parameters:
        img: Input image (H, W) or (H, W, C).
        K_old: 3x3 intrinsics matrix of the source camera.
        K_new: 3x3 intrinsics matrix of the target camera.
        new_size: (width, height) of the output image.

    Returns:
        Image remapped to the new intrinsics.
    """
    width, height = new_size

    K_new_inv = np.linalg.inv(K_new)

    x, y = np.meshgrid(np.arange(width), np.arange(height))
    homogenous_coords = np.stack(
        [x.ravel(), y.ravel(), np.ones_like(x).ravel()], axis=-1
    ).T

    old_coords = K_old @ K_new_inv @ homogenous_coords
    old_coords /= old_coords[2, :]

    map_x = old_coords[0, :].reshape(height, width).astype(np.float32)
    map_y = old_coords[1, :].reshape(height, width).astype(np.float32)

    return cv2.remap(img, map_x, map_y, interpolation=cv2.INTER_LINEAR)


def compute_homography(K: np.ndarray,
                       R: np.ndarray,
                       t: np.ndarray) -> np.ndarray:
    """
    Compute a homography from rotation R and translation t.

    Parameters:
        K: 3x3 intrinsics matrix (shared by both views after conversion).
        R: 3x3 rotation matrix from depth camera to RGB camera.
        t: 3x1 translation vector from depth camera to RGB camera.

    Returns:
        3x3 homography matrix.
    """
    K_inv = np.linalg.inv(K)
    H = K @ (R - t.reshape(-1, 1) @ K_inv[-1, :].reshape(1, -1)) @ K_inv
    return H


def warp_image(image: np.ndarray,
               K: np.ndarray,
               R: np.ndarray,
               t: np.ndarray) -> np.ndarray:
    """
    Warp an image using a homography derived from K, R, and t.

    Parameters:
        image: Input image.
        K: 3x3 intrinsics matrix.
        R: 3x3 rotation matrix (depth -> RGB).
        t: 3x1 translation vector (depth -> RGB).

    Returns:
        Warped image.
    """
    H = compute_homography(K, R, t)
    height, width = image.shape[:2]
    return cv2.warpPerspective(image, H, (width, height))


def align_depth(depth: np.ndarray,
                rgb: np.ndarray,
                depth_K: Tuple[float, float, float, float] = D435_DEPTH_K,
                rgb_K: Tuple[float, float, float, float] = D435_RGB_K,
                cam2cam_transform: Optional[np.ndarray] = None) -> np.ndarray:
    """
    Align a depth image to an RGB image.

    Parameters:
        depth: Depth image (H, W), typically uint16 or float32.
        rgb: RGB image, used only for its dimensions.
        depth_K: Depth camera intrinsics as (fx, fy, cx, cy). Defaults to D435.
        rgb_K: RGB camera intrinsics as (fx, fy, cx, cy). Defaults to D435.
        cam2cam_transform: 4x4 extrinsic transform from depth to RGB camera.
                           Defaults to identity.

    Returns:
        Depth image aligned to the RGB camera frame.
    """
    if cam2cam_transform is None:
        cam2cam_transform = np.eye(4)

    old_fx, old_fy, old_cx, old_cy = depth_K
    new_fx, new_fy, new_cx, new_cy = rgb_K

    K_old = np.array([[old_fx, 0, old_cx],
                       [0, old_fy, old_cy],
                       [0, 0, 1]])
    K_new = np.array([[new_fx, 0, new_cx],
                       [0, new_fy, new_cy],
                       [0, 0, 1]])

    # Re-project depth to RGB intrinsics
    depth = convert_intrinsics(depth, K_old, K_new,
                               new_size=(rgb.shape[1], rgb.shape[0]))

    # Warp using the extrinsic transform between cameras
    depth = warp_image(depth, K_new,
                       cam2cam_transform[:3, :3],
                       cam2cam_transform[:3, 3])
    return depth


def main():
    parser = argparse.ArgumentParser(
        description="Align a depth image to an RGB image."
    )
    parser.add_argument("depth", help="Path to depth image (16-bit PNG)")
    parser.add_argument("rgb", help="Path to RGB image")
    parser.add_argument("output", help="Path for aligned depth output")
    parser.add_argument("--depth-intrinsics", type=float, nargs=4,
                        metavar=("FX", "FY", "CX", "CY"),
                        default=list(D435_DEPTH_K),
                        help="Depth camera intrinsics (fx fy cx cy)")
    parser.add_argument("--rgb-intrinsics", type=float, nargs=4,
                        metavar=("FX", "FY", "CX", "CY"),
                        default=list(D435_RGB_K),
                        help="RGB camera intrinsics (fx fy cx cy)")
    args = parser.parse_args()

    depth = cv2.imread(args.depth, cv2.IMREAD_UNCHANGED)
    rgb = cv2.imread(args.rgb)

    if depth is None:
        print(f"Error: cannot read depth image '{args.depth}'")
        return
    if rgb is None:
        print(f"Error: cannot read RGB image '{args.rgb}'")
        return

    aligned = align_depth(depth, rgb,
                          depth_K=tuple(args.depth_intrinsics),
                          rgb_K=tuple(args.rgb_intrinsics))
    cv2.imwrite(args.output, aligned)
    print(f"Aligned depth saved to {args.output}")


if __name__ == "__main__":
    main()
