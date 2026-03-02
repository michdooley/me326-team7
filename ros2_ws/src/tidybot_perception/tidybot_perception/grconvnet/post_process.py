"""
Post-processing for GR-ConvNet output.

Converts raw network output tensors to numpy arrays with grasp quality,
angle, and width maps. Includes grasp detection via local peak finding.

Adapted from: https://github.com/skumra/robotic-grasping
"""

import numpy as np
import torch
from skimage.filters import gaussian
from skimage.feature import peak_local_max


def post_process_output(q_img, cos_img, sin_img, width_img):
    """
    Post-process raw network output into filtered grasp maps.

    Args:
        q_img: Quality output tensor (B, 1, H, W)
        cos_img: Cosine of 2*angle output tensor
        sin_img: Sine of 2*angle output tensor
        width_img: Normalized width output tensor

    Returns:
        q_img: (H, W) grasp quality map, gaussian-filtered
        ang_img: (H, W) grasp angle in radians, gaussian-filtered
        width_img: (H, W) grasp width in pixels, gaussian-filtered
    """
    q_img = q_img.cpu().numpy().squeeze()
    ang_img = (torch.atan2(sin_img, cos_img) / 2.0).cpu().numpy().squeeze()
    width_img = width_img.cpu().numpy().squeeze() * 150.0

    q_img = gaussian(q_img, 2.0, preserve_range=True)
    ang_img = gaussian(ang_img, 2.0, preserve_range=True)
    width_img = gaussian(width_img, 1.0, preserve_range=True)

    return q_img, ang_img, width_img


class Grasp:
    """
    A grasp represented by center pixel, rotation angle, and gripper width.

    Attributes:
        center: (row, col) pixel coordinates of grasp center
        angle: Rotation angle in radians (positive = CCW from horizontal)
        length: Gripper opening in pixels
        width: Gripper depth in pixels
    """

    def __init__(self, center, angle, length=60, width=30):
        self.center = center
        self.angle = angle
        self.length = length
        self.width = width

    def __repr__(self):
        return (f'Grasp(center={self.center}, angle={np.degrees(self.angle):.1f}deg, '
                f'length={self.length:.1f}px, width={self.width:.1f}px)')


def detect_grasps(q_img, ang_img, width_img=None, no_grasps=1):
    """
    Detect grasps from network output maps.

    Finds local peaks in the quality map and extracts the corresponding
    angle and width at each peak location.

    Args:
        q_img: (H, W) quality map (higher = better grasp)
        ang_img: (H, W) angle map in radians
        width_img: (H, W) width map in pixels (optional)
        no_grasps: Maximum number of grasps to return

    Returns:
        List of Grasp objects, sorted by quality (highest first)
    """
    local_max = peak_local_max(q_img, min_distance=20, threshold_abs=0.2,
                               num_peaks=no_grasps)

    grasps = []
    for grasp_point_array in local_max:
        grasp_point = tuple(grasp_point_array)
        grasp_angle = ang_img[grasp_point]

        g = Grasp(grasp_point, grasp_angle)
        if width_img is not None:
            g.length = width_img[grasp_point]
            g.width = g.length / 2

        grasps.append(g)

    return grasps
