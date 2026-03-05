"""Grasp geometry utilities for top-down grasping."""

import numpy as np
from scipy.spatial.transform import Rotation

# WX250s Interbotix gripper max finger separation
GRIPPER_MAX_OPENING_M = 0.048


def yaw_to_grasp_quaternion(yaw: float) -> tuple:
    """Convert a yaw angle to a quaternion for a top-down grasp.

    The gripper approaches from above (-Z direction in base_link).
    yaw controls the finger alignment angle in the XY plane:
      yaw=0    -> fingers aligned along X axis
      yaw=pi/2 -> fingers aligned along Y axis

    Args:
        yaw: Rotation around Z axis in radians.

    Returns:
        (w, x, y, z) quaternion tuple.
    """
    # 180 deg about X flips Z downward; then rotate by yaw about Z.
    R = Rotation.from_euler('xz', [np.pi, yaw])
    q = R.as_quat()  # scipy returns [x, y, z, w]
    return (float(q[3]), float(q[0]), float(q[1]), float(q[2]))
