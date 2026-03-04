#!/usr/bin/env python3
"""
Geometric Grasp Planner Node — Antipodal Point Cloud Analysis

CPU-only grasp planning using geometric antipodal sampling on point clouds.
No neural networks, no GPU required. Drop-in replacement for
grasp_planner_node.py (GR-ConvNet based).

Pipeline:
    1. Receive /plan_grasp request with object 3D position
    2. Get depth image + camera intrinsics from RGBDObjectDetector
    3. Look up TF: camera_color_optical_frame <-> base_link
    4. Convert depth to point cloud, crop around object
    5. Estimate surface normals (scipy cKDTree + PCA)
    6. Sample antipodal + top-surface grasp candidates
    7. Score, sort, return best candidate

Services provided:
    /plan_grasp (tidybot_msgs/PlanGrasp)

Parameters:
    approach_offset (float): Pre-grasp height above grasp (default 0.10m)
    default_arm (str): Arm when "auto" requested (default "right")
    grasp_z_offset (float): Z offset from detected surface (default 0.0)
    crop_radius (float): Point cloud crop radius around object (default 0.06m)
    num_samples (int): Number of antipodal samples (default 200)

Usage:
    ros2 run tidybot_perception grasp_planner_node
"""

import numpy as np

import rclpy
from rclpy.node import Node

import tf2_ros
import tf2_geometry_msgs  # noqa: F401

from geometry_msgs.msg import Pose, Point, Quaternion
from scipy.spatial.transform import Rotation

from tidybot_msgs.srv import PlanGrasp

from tidybot_perception.rgbd_object_detector import RGBDObjectDetector
from tidybot_perception.grasp_geometry import (
    yaw_to_grasp_quaternion,
    GRIPPER_MAX_OPENING_M,
)
from tidybot_perception.geometric_grasp_sampler import GeometricGraspSampler


class GeometricGraspPlannerNode(Node):
    """Geometric antipodal grasp planner — no neural network, CPU only."""

    def __init__(self):
        super().__init__('grasp_planner_node')

        # Parameters
        self.declare_parameter('approach_offset', 0.10)
        self.declare_parameter('default_arm', 'right')
        self.declare_parameter('grasp_z_offset', 0.0)
        self.declare_parameter('crop_radius', 0.06)
        self.declare_parameter('num_samples', 200)
        # Accept model_path to avoid warnings from launch files that pass it
        self.declare_parameter('model_path', '')

        self.approach_offset = self.get_parameter('approach_offset').value
        self.default_arm = self.get_parameter('default_arm').value
        self.grasp_z_offset = self.get_parameter('grasp_z_offset').value

        # Perception (subscribes to camera topics + TF2)
        self.detector = RGBDObjectDetector(self)

        # Geometric sampler
        self.sampler = GeometricGraspSampler(
            gripper_max_width=GRIPPER_MAX_OPENING_M,
            crop_radius=self.get_parameter('crop_radius').value,
            num_antipodal_samples=self.get_parameter('num_samples').value,
        )

        # Service
        self.grasp_srv = self.create_service(
            PlanGrasp, '/plan_grasp', self.plan_grasp_callback)

        self.get_logger().info('GeometricGraspPlannerNode initialized (CPU-only)')
        self.get_logger().info(f'  approach_offset: {self.approach_offset}m')
        self.get_logger().info(f'  default_arm: {self.default_arm}')
        self.get_logger().info(
            f'  crop_radius: {self.get_parameter("crop_radius").value}m')
        self.get_logger().info(
            f'  num_samples: {self.get_parameter("num_samples").value}')

    # ── TF helpers ────────────────────────────────────────────────────

    def _get_tf_matrix(self, target_frame, source_frame):
        """
        Look up TF and return as a 4x4 homogeneous transform matrix.

        Returns:
            (4, 4) numpy array, or None on failure.
        """
        try:
            import rclpy.time
            import rclpy.duration
            transform = self.detector.tf_buffer.lookup_transform(
                target_frame, source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))

            t = transform.transform.translation
            q = transform.transform.rotation
            # scipy uses xyzw order
            rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
            mat = np.eye(4, dtype=np.float32)
            mat[:3, :3] = rot.as_matrix().astype(np.float32)
            mat[:3, 3] = [t.x, t.y, t.z]
            return mat
        except Exception as e:
            self.get_logger().error(f'TF lookup failed ({source_frame} → {target_frame}): {e}')
            return None

    def _transform_point_to_camera(self, x, y, z):
        """
        Transform a base_link point to camera_color_optical_frame.

        Returns:
            (3,) numpy array in camera frame, or None.
        """
        tf_base_to_cam = self._get_tf_matrix(
            'camera_color_optical_frame', 'base_link')
        if tf_base_to_cam is None:
            return None

        pt = np.array([x, y, z, 1.0], dtype=np.float32)
        pt_cam = tf_base_to_cam @ pt
        return pt_cam[:3]

    # ── Main service callback ─────────────────────────────────────────

    def plan_grasp_callback(self, request, response):
        """Plan a grasp for the given object position."""
        arm = request.arm_name
        if not arm or arm == 'auto':
            arm = self.default_arm

        obj_pos = request.object_position
        ox = obj_pos.point.x
        oy = obj_pos.point.y
        oz = obj_pos.point.z

        self.get_logger().info(
            f'Planning grasp for "{request.object_class}" at '
            f'({ox:.3f}, {oy:.3f}, {oz:.3f}), arm={arm}')

        # Check camera data
        if self.detector.latest_depth is None or self.detector.camera_info is None:
            self.get_logger().warn('No depth/camera_info available')
            response.success = False
            response.message = 'No depth/camera_info available'
            return response

        # Get camera intrinsics
        K = self.detector.camera_info.k
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]

        # Get TF: camera → base_link
        tf_cam_to_base = self._get_tf_matrix(
            'base_link', 'camera_color_optical_frame')
        if tf_cam_to_base is None:
            self.get_logger().warn('TF lookup failed')
            response.success = False
            response.message = 'TF lookup failed (camera → base_link)'
            return response

        # Transform object center to camera frame for cropping
        obj_cam = self._transform_point_to_camera(ox, oy, oz)
        if obj_cam is None:
            self.get_logger().warn('Could not transform object to camera frame')
            response.success = False
            response.message = 'TF lookup failed (base_link → camera)'
            return response

        # Run geometric grasp sampler
        candidates = self.sampler.plan_grasp(
            self.detector.latest_depth,
            (fx, fy, cx, cy),
            obj_cam,
            tf_cam_to_base,
        )

        if not candidates:
            self.get_logger().warn('No geometric grasp candidates found')
            response.success = False
            response.message = 'No geometric grasp candidates found'
            return response

        self.get_logger().info(
            f'  Geometric sampler found {len(candidates)} candidates')

        # Return best candidate (highest score — no IK validation here,
        # the caller handles IK via plan_and_execute)
        best = candidates[0]
        gx, gy = float(best.center[0]), float(best.center[1])
        gz = float(best.center[2]) + self.grasp_z_offset

        # Build grasp orientation from yaw
        qw, qx, qy, qz = yaw_to_grasp_quaternion(best.yaw)

        self.get_logger().info(
            f'  Best: pos=({gx:.3f}, {gy:.3f}, {gz:.3f}) '
            f'yaw={np.degrees(best.yaw):.1f}deg '
            f'width={best.width*1000:.0f}mm '
            f'score={best.total_score:.3f}')

        # Build grasp pose
        grasp_pose = Pose()
        grasp_pose.position = Point(x=gx, y=gy, z=gz)
        grasp_pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

        # Build pre-grasp pose (above grasp)
        pre_grasp_pose = Pose()
        pre_grasp_pose.position = Point(
            x=gx, y=gy, z=gz + self.approach_offset)
        pre_grasp_pose.orientation = grasp_pose.orientation

        response.success = True
        response.grasp_pose = grasp_pose
        response.pre_grasp_pose = pre_grasp_pose
        response.arm_used = arm
        response.grasp_width = float(best.width)
        response.message = (
            f'Geometric grasp: yaw={np.degrees(best.yaw):.1f}deg, '
            f'width={best.width*1000:.0f}mm, '
            f'score={best.total_score:.3f}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GeometricGraspPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
