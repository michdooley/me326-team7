#!/usr/bin/env python3
"""Prepare captured RGB-D pairs for GraspNet inference.

For each pair_XXXX directory in captures_real/:
  1. Rename 'color.png' -> 'rgb.png' (if needed)
  2. Copy in workspace_mask.png (pure-white 480x640 single-channel)
  3. Copy in meta.mat (camera intrinsics + depth scale factor)

Usage:
    python3 prepare_graspnet.py [--captures-dir PATH]
"""

import argparse
import shutil
from pathlib import Path

import cv2
import numpy as np
import scipy.io as sio

# RealSense D435 color camera intrinsics (from /camera/color/camera_info)
COLOR_INTRINSICS = np.array([
    [609.7716674804688, 0.0,               331.6385192871094],
    [0.0,               610.3194580078125,  247.25904846191406],
    [0.0,               0.0,               1.0],
])

# Depth is uint16 in millimeters; GraspNet divides by factor_depth to get meters
FACTOR_DEPTH = 1000


def main():
    parser = argparse.ArgumentParser(description="Prepare captures for GraspNet")
    parser.add_argument(
        "--captures-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "captures_real",
        help="Root directory containing pair_XXXX folders",
    )
    args = parser.parse_args()

    captures_dir: Path = args.captures_dir
    if not captures_dir.is_dir():
        raise FileNotFoundError(f"Captures directory not found: {captures_dir}")

    # --- Create master workspace_mask.png (pure white, single channel) ---
    mask = np.full((480, 640), 255, dtype=np.uint8)
    mask_path = captures_dir / "workspace_mask.png"
    cv2.imwrite(str(mask_path), mask)
    print(f"Created {mask_path}")

    # --- Create master meta.mat ---
    meta = {
        "intrinsic_matrix": COLOR_INTRINSICS,
        "factor_depth": np.array([[FACTOR_DEPTH]], dtype=np.float64),
    }
    meta_path = captures_dir / "meta.mat"
    sio.savemat(str(meta_path), meta)
    print(f"Created {meta_path}")

    # --- Process each pair directory ---
    pair_dirs = sorted(p for p in captures_dir.iterdir() if p.is_dir() and p.name.startswith("pair_"))
    print(f"\nFound {len(pair_dirs)} pair directories")

    for pair_dir in pair_dirs:
        # Rename color.png -> rgb.png if needed
        color_file = pair_dir / "color.png"
        rgb_file = pair_dir / "rgb.png"
        if color_file.exists() and not rgb_file.exists():
            color_file.rename(rgb_file)
            print(f"  {pair_dir.name}: renamed color.png -> rgb.png")

        # Copy workspace_mask.png
        shutil.copy2(mask_path, pair_dir / "workspace_mask.png")

        # Copy meta.mat
        shutil.copy2(meta_path, pair_dir / "meta.mat")

        print(f"  {pair_dir.name}: copied workspace_mask.png + meta.mat")

    print(f"\nDone! Processed {len(pair_dirs)} directories.")


if __name__ == "__main__":
    main()
