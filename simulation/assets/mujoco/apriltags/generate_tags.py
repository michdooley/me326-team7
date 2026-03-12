#!/usr/bin/env python3
"""
Generate AprilTag PNGs for MuJoCo textures.

Uses OpenCV's aruco module to generate correct tag36h11 markers,
then adds a white border for reliable detection.

Tag mapping:
  tag 0 = banana bin
  tag 1 = apple bin
  tag 2 = orange bin

Usage:
  cd simulation/assets/mujoco/apriltags
  python generate_tags.py
"""

import cv2
import numpy as np
from PIL import Image


def generate_tag_image(tag_id, size=200):
    """Generate a single AprilTag image using OpenCV's aruco module.

    The marker is generated at a smaller size and centered in a white
    image to provide the required quiet zone border.
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

    # Generate the core marker (smaller than final size to leave room for border)
    marker_size = int(size * 0.6)  # 60% of image is marker, 20% border each side
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, tag_id, marker_size)

    # Create white canvas and center the marker
    canvas = np.ones((size, size), dtype=np.uint8) * 255
    offset = (size - marker_size) // 2
    canvas[offset:offset + marker_size, offset:offset + marker_size] = marker_img

    return Image.fromarray(canvas, mode='L')


def main():
    tag_names = {0: "banana", 1: "apple", 2: "orange"}

    for tag_id in [0, 1, 2]:
        filename = f"tag36h11_{tag_id}.png"
        img = generate_tag_image(tag_id, size=200)
        img.save(filename)
        print(f"Generated {filename} (tag {tag_id} = {tag_names[tag_id]} bin)")

    # Verify detection
    try:
        from pupil_apriltags import Detector
        detector = Detector(families='tag36h11', nthreads=1)
        print("\nVerifying detection:")
        for tag_id in [0, 1, 2]:
            img = np.array(Image.open(f"tag36h11_{tag_id}.png"))
            results = detector.detect(img)
            if results:
                for r in results:
                    print(f"  tag36h11_{tag_id}.png: OK "
                          f"(id={r.tag_id}, margin={r.decision_margin:.1f})")
            else:
                print(f"  tag36h11_{tag_id}.png: FAILED — not detected!")
    except ImportError:
        print("\nSkipping verification (pupil-apriltags not installed)")

    print("\nDone!")


if __name__ == "__main__":
    main()
