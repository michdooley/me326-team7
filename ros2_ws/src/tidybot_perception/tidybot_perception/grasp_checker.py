#!/usr/bin/env python3
"""
Grasp Checker Utility - Geometric Heuristic-based Grasp Detection

Provides simple antipodal grasp sampling from RGB-D point clouds using
surface normals and geometric scoring.
"""

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
from typing import List, Tuple


class GraspPose:
    """Represents a 6-DoF grasp pose with quality score."""

    def __init__(self, position: np.ndarray, orientation: np.ndarray, score: float):
        """
        Args:
            position: (3,) XYZ position in meters
            orientation: (4,) quaternion [x, y, z, w] or (3, 3) rotation matrix
            score: Grasp quality score (0-1, higher is better)
        """
        self.position = np.asarray(position, dtype=np.float64)

        # Convert to rotation matrix if quaternion provided
        if orientation.shape == (4,):
            self.orientation = Rotation.from_quat(orientation).as_matrix()
        elif orientation.shape == (3, 3):
            self.orientation = orientation.astype(np.float64)
        else:
            raise ValueError(f"Orientation must be (4,) quaternion or (3, 3) matrix, got {orientation.shape}")

        self.score = float(score)

    def to_quaternion(self, format='xyzw') -> np.ndarray:
        """
        Convert orientation to quaternion.

        Args:
            format: 'xyzw' (scipy/ROS default) or 'wxyz' (MuJoCo/some robotics libs)

        Returns:
            (4,) quaternion array
        """
        quat = Rotation.from_matrix(self.orientation).as_quat()  # Returns [x,y,z,w]
        if format == 'wxyz':
            return np.roll(quat, 1)  # Convert to [w,x,y,z]
        return quat


class GraspChecker:
    """Geometric heuristic-based grasp detection from RGB-D images."""

    def __init__(self,
                 score_threshold: float = 0.3,
                 gripper_width: Tuple[float, float] = (0.03, 0.07),
                 sample_ratio: float = 0.1):
        """
        Initialize grasp checker.

        Args:
            score_threshold: Minimum grasp quality score (0-1)
            gripper_width: (min, max) gripper opening in meters (default: 3-7cm for WX250s)
            sample_ratio: Fraction of point cloud to sample for grasp candidates (0-1)
        """
        self.score_threshold = score_threshold
        self.gripper_width = gripper_width
        self.sample_ratio = sample_ratio

    def predict(self,
                rgb: np.ndarray,
                depth: np.ndarray,
                camera_intrinsics: np.ndarray,
                num_grasps: int = 10) -> List[GraspPose]:
        """
        Detect grasps from RGB-D image.

        Args:
            rgb: (H, W, 3) RGB image [0-255] uint8
            depth: (H, W) depth in meters, float32
            camera_intrinsics: (3, 3) camera matrix K
            num_grasps: Maximum number of top grasps to return

        Returns:
            List of GraspPose objects sorted by score (descending)
        """
        # Convert RGB-D to point cloud
        point_cloud = self._rgbd_to_pointcloud(rgb, depth, camera_intrinsics)

        # Run grasp sampling
        grasp_candidates = self._sample_antipodal_grasps(point_cloud)

        # Filter by score threshold and sort
        grasps = [
            GraspPose(pos, orient, score)
            for pos, orient, score in grasp_candidates
            if score >= self.score_threshold
        ]
        grasps.sort(key=lambda g: g.score, reverse=True)

        return grasps[:num_grasps]

    def _rgbd_to_pointcloud(self,
                            rgb: np.ndarray,
                            depth: np.ndarray,
                            intrinsics: np.ndarray) -> o3d.geometry.PointCloud:
        """Convert RGB-D image to Open3D point cloud."""
        h, w = depth.shape
        fx, fy = intrinsics[0, 0], intrinsics[1, 1]
        cx, cy = intrinsics[0, 2], intrinsics[1, 2]

        # Create Open3D camera intrinsic
        o3d_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=w, height=h, fx=fx, fy=fy, cx=cx, cy=cy
        )

        # Convert to Open3D images
        rgb_o3d = o3d.geometry.Image(rgb.astype(np.uint8))
        depth_o3d = o3d.geometry.Image((depth * 1000).astype(np.uint16))

        # Create RGBD image and point cloud
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            rgb_o3d, depth_o3d,
            depth_scale=1000.0,
            convert_rgb_to_intensity=False,
            depth_trunc=2.0
        )
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d_intrinsic)

        return pcd

    def _sample_antipodal_grasps(self,
                                  point_cloud: o3d.geometry.PointCloud) -> List[Tuple[np.ndarray, np.ndarray, float]]:
        """
        Sample antipodal grasp candidates from point cloud using surface normals.

        Returns:
            List of (position, orientation_matrix, score) tuples
        """
        points = np.asarray(point_cloud.points)

        if len(points) < 10:
            return []

        # Estimate surface normals
        point_cloud.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.03, max_nn=30)
        )
        normals = np.asarray(point_cloud.normals)

        # Orient normals toward camera
        for i in range(len(normals)):
            if np.dot(normals[i], -points[i]) < 0:
                normals[i] = -normals[i]

        # Sample points for grasp candidates
        num_samples = max(10, int(len(points) * self.sample_ratio))
        indices = np.random.choice(len(points), size=min(num_samples, len(points)), replace=False)

        grasp_candidates = []

        for idx in indices:
            point = points[idx]
            normal = normals[idx]

            if np.linalg.norm(normal) < 0.1:
                continue

            # Grasp approach direction: along negative normal
            approach = -normal / np.linalg.norm(normal)

            # Create gripper orientation frame
            if abs(approach[2]) < 0.9:
                up = np.array([0, 0, 1])
            else:
                up = np.array([1, 0, 0])

            x_axis = np.cross(approach, up)
            x_axis = x_axis / (np.linalg.norm(x_axis) + 1e-8)
            y_axis = np.cross(approach, x_axis)
            y_axis = y_axis / (np.linalg.norm(y_axis) + 1e-8)

            orientation = np.column_stack([x_axis, y_axis, approach])

            # Score grasp based on local point density, distance, and normal consistency
            distances = np.linalg.norm(points - point, axis=1)
            neighbors_mask = distances < self.gripper_width[1]
            num_neighbors = np.sum(neighbors_mask)

            density_score = min(1.0, num_neighbors / 50.0)

            point_dist = np.linalg.norm(point)
            if point_dist < 0.2 or point_dist > 1.5:
                distance_score = 0.0
            else:
                distance_score = 1.0 - min(1.0, (point_dist - 0.2) / 1.3)

            if num_neighbors > 5:
                neighbor_normals = normals[neighbors_mask]
                normal_dots = np.abs(np.dot(neighbor_normals, normal))
                consistency_score = np.mean(normal_dots)
            else:
                consistency_score = 0.5

            score = (0.4 * density_score +
                    0.3 * distance_score +
                    0.3 * consistency_score)

            grasp_candidates.append((point, orientation, score))

        return grasp_candidates
