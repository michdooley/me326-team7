#!/usr/bin/env python3
"""
Coordinate Converter Utility

Converts poses between camera frame and base_link frame using TF2.
Used by the GraspNet grasp planner to convert camera-frame grasp poses
into base_link-frame poses for the IK motion planner.

Coordinate frame conventions:
  camera_color_optical_frame (ROS optical convention):
    +Z = forward (depth / distance from camera)
    +X = right (image column direction)
    +Y = down  (image row direction)

  base_link frame (TidyBot2 convention, from test_planner_sim.py):
    +X = left  (robot lateral, positive = robot's left)
    -Y = forward (robot heading, negative Y = in front of robot)
    +Z = up

  The IK planner (PlanToTarget service) expects poses in base_link frame
  with quaternion in wxyz format (w, x, y, z).
"""

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
import rclpy.time

from geometry_msgs.msg import Pose, PoseStamped, PointStamped

try:
    import tf2_ros
    import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform support
    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False


class CoordConverter:
    """
    Utility class for converting coordinates between camera and robot frames.

    Wraps TF2 lookups so that callers don't need to know the frame names or
    the TF API directly.
    """

    TARGET_FRAME = 'base_link'
    CAMERA_OPTICAL_FRAME = 'camera_color_optical_frame'

    def __init__(self, tf_buffer: 'tf2_ros.Buffer'):
        """
        Args:
            tf_buffer: A TF2 buffer that is being updated (attached to a
                       running Node's tf2_ros.TransformListener).
        """
        self.tf_buffer = tf_buffer

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------

    def camera_to_base(
        self,
        pose_stamped: PoseStamped,
        timeout_sec: float = 1.0,
    ) -> PoseStamped:
        """
        Transform a PoseStamped from any camera frame into base_link.

        Args:
            pose_stamped: Pose in camera frame (header.frame_id must be set).
            timeout_sec:  How long to wait for the TF transform.

        Returns:
            PoseStamped in base_link frame.

        Raises:
            tf2_ros.LookupException / ConnectivityException / ExtrapolationException
            on TF lookup failure.
        """
        transform = self.tf_buffer.lookup_transform(
            self.TARGET_FRAME,
            pose_stamped.header.frame_id,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=timeout_sec),
        )
        return tf2_geometry_msgs.do_transform_pose_stamped(pose_stamped, transform)

    def point_camera_to_base(
        self,
        point_stamped: PointStamped,
        timeout_sec: float = 1.0,
    ) -> PointStamped:
        """
        Transform a PointStamped from any camera frame into base_link.
        """
        transform = self.tf_buffer.lookup_transform(
            self.TARGET_FRAME,
            point_stamped.header.frame_id,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=timeout_sec),
        )
        return tf2_geometry_msgs.do_transform_point(point_stamped, transform)

    @staticmethod
    def rotation_matrix_to_pose(R: np.ndarray, translation: np.ndarray) -> Pose:
        """
        Build a geometry_msgs/Pose from a 3x3 rotation matrix and translation vector.

        GraspNet returns grasps as (rotation_matrix 3x3, translation 3-vector) in
        camera frame.  This helper converts them to a Pose message so they can be
        used with TF2 transformations and then passed to PlanToTarget.

        Args:
            R:           (3, 3) rotation matrix.  In GraspNet convention the grasp
                         frame is defined as:
                           col 0 (R[:, 0]): grasp approach direction
                           col 1 (R[:, 1]): closing direction (along which fingers move)
                           col 2 (R[:, 2]): lateral axis (right-hand rule)
            translation: (3,) position in the same frame as R.

        Returns:
            geometry_msgs/Pose with position and orientation populated.
            Quaternion is in (w, x, y, z) order as required by ROS messages.
        """
        # scipy returns (x, y, z, w) — convert to ROS (w, x, y, z)
        rot = Rotation.from_matrix(R)
        x, y, z, w = rot.as_quat()

        pose = Pose()
        pose.position.x = float(translation[0])
        pose.position.y = float(translation[1])
        pose.position.z = float(translation[2])
        pose.orientation.w = float(w)
        pose.orientation.x = float(x)
        pose.orientation.y = float(y)
        pose.orientation.z = float(z)
        return pose

    @staticmethod
    def offset_pose_along_approach(
        pose: Pose,
        approach_col: int = 0,
        offset_m: float = 0.10,
    ) -> Pose:
        """
        Compute a pre-grasp pose by backing off along the approach direction.

        The approach direction is extracted from the rotation matrix encoded in
        the quaternion.  In GraspNet's convention, column 0 of the rotation matrix
        (R[:, 0]) is the approach vector — i.e., the direction the gripper travels
        *toward* the object.  The pre-grasp pose is offset *opposite* to that
        direction.

        Args:
            pose:         Grasp pose (position + quaternion, in any frame).
            approach_col: Which column of the rotation matrix holds the approach
                          vector (0 for GraspNet convention).
            offset_m:     Distance to back off in metres.

        Returns:
            Pre-grasp Pose with position offset along the retreat direction.
        """
        q = np.array([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ])  # scipy xyzw
        R = Rotation.from_quat(q).as_matrix()  # (3, 3)

        # Approach vector points toward object; retreat by negating it
        approach_vec = R[:, approach_col]
        retreat_vec = -approach_vec

        pre_grasp = Pose()
        pre_grasp.position.x = pose.position.x + offset_m * retreat_vec[0]
        pre_grasp.position.y = pose.position.y + offset_m * retreat_vec[1]
        pre_grasp.position.z = pose.position.z + offset_m * retreat_vec[2]
        pre_grasp.orientation = pose.orientation  # same orientation as grasp
        return pre_grasp

    @staticmethod
    def depth_pixel_to_camera_point(
        u: int,
        v: int,
        depth_m: float,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
    ) -> np.ndarray:
        """
        Back-project a depth pixel to a 3-D point in camera optical frame.

        Camera optical frame: +Z forward, +X right, +Y down.

        Args:
            u, v:    Pixel column and row (image coordinates).
            depth_m: Depth value in metres.
            fx, fy:  Focal lengths in pixels.
            cx, cy:  Principal point in pixels.

        Returns:
            (3,) numpy array [x, y, z] in camera optical frame (metres).
        """
        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        z = depth_m
        return np.array([x, y, z], dtype=np.float32)
