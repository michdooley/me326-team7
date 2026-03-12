"""Grasp geometry utilities for top-down grasping."""

import numpy as np
from scipy.spatial.transform import Rotation

# WX250s Interbotix gripper max finger separation
GRIPPER_MAX_OPENING_M = 0.048


def yaw_to_grasp_quaternion(yaw: float) -> tuple:
    """Convert a yaw angle to a quaternion for a top-down grasp.

    The gripper approaches from above (-Z direction in base_link).
    Uses the correct fingers-down orientation for the WX250s arm
    mounted at yaw=-90deg on the TidyBot2 base.

    yaw controls the finger alignment angle in the XY plane:
      yaw=0    -> default fingers-down orientation
      yaw=pi/2 -> fingers rotated 90deg about Z

    Args:
        yaw: Rotation around Z axis in radians.

    Returns:
        (w, x, y, z) quaternion tuple.
    """
    # Base fingers-down orientation for the WX250s arm (verified in test_planner_real.py)
    # This quaternion maps the ee's local axes to point the gripper straight down (-Z).
    R_fingers_down = Rotation.from_quat([0.5, 0.5, -0.5, 0.5])  # scipy [x,y,z,w]

    # Apply yaw rotation about base_link Z axis
    R_yaw = Rotation.from_euler('z', yaw)
    R = R_yaw * R_fingers_down

    q = R.as_quat()  # scipy returns [x, y, z, w]
    return (float(q[3]), float(q[0]), float(q[1]), float(q[2]))
