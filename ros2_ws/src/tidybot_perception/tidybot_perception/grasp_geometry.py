"""
Grasp Geometry Utilities

Pure-computation module for converting GR-ConvNet pixel-space grasps
into 3D base_link poses for a top-down parallel jaw gripper.

No ROS dependencies — all functions take numpy arrays or callables.
"""

import numpy as np
from scipy.spatial.transform import Rotation


# WX250s gripper: each finger slides [0.015, 0.037] m
# Total jaw opening: 2 * 0.037 = 0.074 m (74 mm)
GRIPPER_MAX_OPENING_M = 0.074


def yaw_to_grasp_quaternion(yaw_rad):
    """
    Compute a fingers-down quaternion with additional yaw around base_link Z.

    The base fingers-down orientation is ORIENT_FINGERS_DOWN = (0.5, 0.5, 0.5, -0.5)
    in (w,x,y,z). This function composes a Z-rotation of yaw_rad with that base.

    Args:
        yaw_rad: Rotation around base_link Z-axis in radians.

    Returns:
        (qw, qx, qy, qz) quaternion in wxyz format (ROS convention).
    """
    # ORIENT_FINGERS_DOWN = (w=0.5, x=0.5, y=0.5, z=-0.5)
    # In scipy xyzw order: (x=0.5, y=0.5, z=-0.5, w=0.5)
    R_down = Rotation.from_quat([0.5, 0.5, -0.5, 0.5])
    R_yaw = Rotation.from_euler('z', yaw_rad)
    R_grasp = R_yaw * R_down
    qx, qy, qz, qw = R_grasp.as_quat()
    return (float(qw), float(qx), float(qy), float(qz))


def image_angle_to_base_link_yaw(angle_image_rad, centroid_uv, pixel_to_base_fn):
    """
    Convert an angle in image coordinates to a yaw angle in base_link XY plane.

    Projects two points along the image-space direction to 3D base_link
    coordinates, then computes the angle from their XY displacement.

    Args:
        angle_image_rad: Angle in image space (radians, CCW from +u axis).
        centroid_uv: (u, v) pixel centroid of the object.
        pixel_to_base_fn: Callable(u, v) -> PointStamped in base_link, or None.

    Returns:
        Yaw angle in base_link frame (radians), or None on failure.
    """
    cu, cv = centroid_uv
    delta = 30  # pixels
    du = delta * np.cos(angle_image_rad)
    dv = delta * np.sin(angle_image_rad)

    u1, v1 = int(cu - du), int(cv - dv)
    u2, v2 = int(cu + du), int(cv + dv)

    pt1 = pixel_to_base_fn(u1, v1)
    pt2 = pixel_to_base_fn(u2, v2)

    if pt1 is None or pt2 is None:
        return None

    dx = pt2.point.x - pt1.point.x
    dy = pt2.point.y - pt1.point.y

    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        return None

    return float(np.arctan2(dy, dx))


def grasp_width_pixels_to_meters(width_pixels, depth_m, fx):
    """
    Convert grasp width from pixels to meters using depth and focal length.

    Args:
        width_pixels: Grasp width in image pixels.
        depth_m: Depth at the grasp center in meters.
        fx: Focal length in pixels (from camera intrinsics K[0,0]).

    Returns:
        Width in meters.
    """
    return (width_pixels / fx) * depth_m
