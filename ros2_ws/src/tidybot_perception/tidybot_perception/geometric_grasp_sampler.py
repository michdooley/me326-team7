"""
Geometric Antipodal Grasp Sampler

CPU-only grasp candidate generation from depth images using geometric
antipodal analysis. No neural networks, no GPU required.

Algorithm:
    1. Convert depth image to point cloud (vectorized numpy)
    2. Crop around known object position
    3. Estimate surface normals (scipy cKDTree + PCA)
    4. Sample antipodal grasp candidates:
       a. Random-point antipodal search (opposing normals)
       b. Top-surface yaw sweep (for spheres/flat objects)
    5. Score candidates (heavily favoring top-down approach)
    6. Deduplicate and return sorted list

Dependencies: numpy, scipy (already installed)
No ROS dependencies — all functions take numpy arrays.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
from scipy.spatial import cKDTree

from tidybot_perception.grasp_geometry import GRIPPER_MAX_OPENING_M


# ── Data structures ──────────────────────────────────────────────────────────

@dataclass
class GraspCandidate:
    """A single grasp candidate in base_link frame."""
    center: np.ndarray          # (3,) xyz in base_link
    yaw: float                  # finger-axis angle in base_link XY (rad)
    width: float                # distance between contact points (m)
    antipodal_score: float      # 0-1, how antipodal the contact normals are
    total_score: float = 0.0    # computed by score()

    def score(self, weights=None):
        """
        Compute total score. Top-down alignment is enforced by construction
        (all grasps approach from above), so we score on:
          - antipodal quality (how opposing the contact normals are)
          - width tightness (prefer object filling more of gripper)
          - height (penalize grasps too close to table)
        """
        if weights is None:
            weights = {
                'antipodal': 2.0,
                'width': 1.0,
                'height': 1.0,
            }

        # Antipodal quality: already in [0, 1]
        s_antipodal = self.antipodal_score

        # Width: prefer tighter grasps (object fills more of gripper)
        s_width = max(0.0, 1.0 - self.width / GRIPPER_MAX_OPENING_M)

        # Height: penalize grasps too close to table (z=0)
        z = self.center[2]
        if z < 0.005:
            s_height = 0.0
        elif z < 0.06:
            s_height = 1.0
        else:
            s_height = max(0.0, 1.0 - (z - 0.06) / 0.10)

        self.total_score = (
            weights['antipodal'] * s_antipodal
            + weights['width'] * s_width
            + weights['height'] * s_height
        )
        return self.total_score


# ── Point cloud utilities ────────────────────────────────────────────────────

def depth_to_point_cloud(
    depth_img: np.ndarray,
    fx: float, fy: float, cx: float, cy: float,
    min_depth: float = 0.1,
    max_depth: float = 2.0,
) -> np.ndarray:
    """
    Convert a uint16 depth image (mm) to an Nx3 point cloud in camera
    optical frame (+Z forward, +X right, +Y down).

    Args:
        depth_img: (H, W) uint16, depth in millimeters.
        fx, fy, cx, cy: Camera intrinsics.
        min_depth, max_depth: Valid depth range in meters.

    Returns:
        (N, 3) float32 array of 3D points in camera frame.
    """
    h, w = depth_img.shape
    z = depth_img.astype(np.float32) / 1000.0  # mm → m

    # Build pixel coordinate grids
    u_grid, v_grid = np.meshgrid(np.arange(w), np.arange(h))

    # Mask valid depths
    mask = (z > min_depth) & (z < max_depth)

    z_valid = z[mask]
    u_valid = u_grid[mask].astype(np.float32)
    v_valid = v_grid[mask].astype(np.float32)

    x = (u_valid - cx) * z_valid / fx
    y = (v_valid - cy) * z_valid / fy

    return np.stack([x, y, z_valid], axis=1)


def crop_around_point(
    points: np.ndarray,
    center: np.ndarray,
    radius: float = 0.06,
) -> np.ndarray:
    """
    Extract points within a sphere of given radius around center.

    Args:
        points: (N, 3) array.
        center: (3,) center position.
        radius: Crop sphere radius in meters.

    Returns:
        (M, 3) subset of points within the sphere.
    """
    dists = np.linalg.norm(points - center, axis=1)
    return points[dists < radius]


def estimate_normals(
    points: np.ndarray,
    search_radius: float = 0.015,
    max_nn: int = 30,
    orient_toward: Optional[np.ndarray] = None,
) -> np.ndarray:
    """
    Estimate surface normals using scipy cKDTree + PCA on local neighborhoods.

    For each point, finds neighbors within search_radius (up to max_nn),
    fits a plane via SVD (smallest singular vector = normal), then orients
    all normals toward orient_toward.

    Args:
        points: (M, 3) numpy array.
        search_radius: Radius for neighbor search (meters).
        max_nn: Max neighbors per point.
        orient_toward: (3,) point to orient normals toward.
            Default orients upward (toward [0, 0, 10]).

    Returns:
        (M, 3) unit normal vectors.
    """
    if orient_toward is None:
        orient_toward = np.array([0.0, 0.0, 10.0])

    n_pts = len(points)
    normals = np.zeros((n_pts, 3), dtype=np.float64)

    # Build KD-tree
    tree = cKDTree(points)

    # Query all neighbors within radius (batch query)
    neighbor_lists = tree.query_ball_point(points, search_radius)

    for i in range(n_pts):
        neighbors = neighbor_lists[i]

        # Limit to max_nn closest neighbors
        if len(neighbors) > max_nn:
            # Sort by distance to point i
            dists = np.linalg.norm(points[neighbors] - points[i], axis=1)
            closest = np.argsort(dists)[:max_nn]
            neighbors = [neighbors[j] for j in closest]

        if len(neighbors) < 3:
            # Not enough neighbors — default to upward normal
            normals[i] = [0.0, 0.0, 1.0]
            continue

        # PCA: center the neighborhood, compute SVD
        neighborhood = points[neighbors]
        centered = neighborhood - neighborhood.mean(axis=0)
        # SVD: smallest singular value's right singular vector = normal
        try:
            _, _, Vt = np.linalg.svd(centered, full_matrices=False)
            normal = Vt[2]  # last row = smallest component direction
        except np.linalg.LinAlgError:
            normals[i] = [0.0, 0.0, 1.0]
            continue

        # Orient toward the reference point
        to_ref = orient_toward - points[i]
        if np.dot(normal, to_ref) < 0:
            normal = -normal

        normals[i] = normal

    # Ensure unit length
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    lengths = np.maximum(lengths, 1e-10)
    normals = normals / lengths

    return normals.astype(np.float32)


def transform_points(points: np.ndarray, tf_4x4: np.ndarray) -> np.ndarray:
    """Apply a 4x4 homogeneous transform to (N,3) points."""
    ones = np.ones((len(points), 1), dtype=np.float32)
    pts_h = np.hstack([points, ones])
    transformed = (tf_4x4 @ pts_h.T).T
    return transformed[:, :3]


# ── Antipodal grasp sampling ─────────────────────────────────────────────────

def sample_antipodal_grasps(
    points: np.ndarray,
    normals: np.ndarray,
    gripper_max_width: float = GRIPPER_MAX_OPENING_M,
    num_samples: int = 200,
    antipodal_threshold: float = 0.80,
    rng: Optional[np.random.Generator] = None,
) -> List[GraspCandidate]:
    """
    Sample antipodal grasp candidates from a point cloud with normals.

    For each sample:
      1. Pick random point p1, get normal n1
      2. Determine finger direction (horizontal projection of n1, or random
         yaw if n1 is mostly vertical)
      3. Find points on the opposite side along finger direction
      4. Check antipodality: n1 · n2 < -threshold
      5. Check width fits gripper
      6. Record grasp candidate
    """
    if rng is None:
        rng = np.random.default_rng()

    n_pts = len(points)
    if n_pts < 10:
        return []

    candidates = []

    for _ in range(num_samples):
        idx1 = rng.integers(0, n_pts)
        p1 = points[idx1]
        n1 = normals[idx1]

        # Project normal to XY plane for finger direction
        n1_horiz = np.array([n1[0], n1[1], 0.0])
        n1_horiz_len = np.linalg.norm(n1_horiz)

        if n1_horiz_len < 0.1:
            # Normal is nearly vertical (top surface point) — random yaw
            yaw = rng.uniform(-np.pi, np.pi)
            finger_dir = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        else:
            finger_dir = n1_horiz / n1_horiz_len

        # Find points on the opposite side
        displacements = points - p1
        distances = np.linalg.norm(displacements, axis=1)

        # Filter: within gripper width and not self
        mask = (distances > 0.005) & (distances < gripper_max_width)
        if mask.sum() == 0:
            continue

        cand_dirs = displacements[mask] / distances[mask, None]

        # Want points on opposite side: displacement · finger_dir < -0.5
        alignment = cand_dirs @ finger_dir
        opp_mask = alignment < -0.5
        if opp_mask.sum() == 0:
            continue

        # Check antipodality of normals at opposing points
        opp_normals = normals[mask][opp_mask]
        antipodal_scores = -(n1 @ opp_normals.T)

        best_idx = np.argmax(antipodal_scores)
        if antipodal_scores[best_idx] < antipodal_threshold:
            continue

        opp_points = points[mask][opp_mask]
        p2 = opp_points[best_idx]

        grasp_center = (p1 + p2) / 2.0
        grasp_width = float(np.linalg.norm(p2 - p1))
        finger_vec = p2 - p1
        grasp_yaw = float(np.arctan2(finger_vec[1], finger_vec[0]))

        candidates.append(GraspCandidate(
            center=grasp_center,
            yaw=grasp_yaw,
            width=grasp_width,
            antipodal_score=float(antipodal_scores[best_idx]),
        ))

    return candidates


def sample_top_grasps(
    points: np.ndarray,
    normals: np.ndarray,
    gripper_max_width: float = GRIPPER_MAX_OPENING_M,
    num_yaw_samples: int = 12,
) -> List[GraspCandidate]:
    """
    Complementary strategy for top-surface-dominated objects (spheres, flat).

    Sweeps yaw angles, projects points onto finger axis, finds extent width.
    Good for objects where most normals point upward.
    """
    if len(points) < 10:
        return []

    centroid = np.mean(points, axis=0)
    candidates = []

    for i in range(num_yaw_samples):
        yaw = i * np.pi / num_yaw_samples  # [0, pi)
        finger_dir = np.array([np.cos(yaw), np.sin(yaw), 0.0])

        # Project all points onto finger axis
        projections = points @ finger_dir

        p_min_idx = np.argmin(projections)
        p_max_idx = np.argmax(projections)

        width = float(projections[p_max_idx] - projections[p_min_idx])

        if width > gripper_max_width or width < 0.005:
            continue

        # Check there are points on both sides
        median_proj = np.median(projections)
        n_left = np.sum(projections < median_proj)
        n_right = np.sum(projections >= median_proj)
        if min(n_left, n_right) < 3:
            continue

        # Antipodal score from normals at extreme points
        n1 = normals[p_min_idx]
        n2 = normals[p_max_idx]
        antipodal = float(max(0.0, -(n1 @ n2)))

        candidates.append(GraspCandidate(
            center=centroid.copy(),
            yaw=float(yaw),
            width=width,
            antipodal_score=antipodal,
        ))

    return candidates


# ── Deduplication ────────────────────────────────────────────────────────────

def _normalize_angle(a: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while a > np.pi:
        a -= 2 * np.pi
    while a < -np.pi:
        a += 2 * np.pi
    return a


def deduplicate_grasps(
    grasps: List[GraspCandidate],
    pos_threshold: float = 0.01,
    yaw_threshold: float = 0.2,
) -> List[GraspCandidate]:
    """Remove grasps whose center and yaw are too similar."""
    result: List[GraspCandidate] = []
    for g in grasps:
        is_dup = False
        for existing in result:
            d_pos = np.linalg.norm(g.center - existing.center)
            d_yaw = abs(_normalize_angle(g.yaw - existing.yaw))
            if d_pos < pos_threshold and d_yaw < yaw_threshold:
                is_dup = True
                break
        if not is_dup:
            result.append(g)
    return result


# ── Main sampler class ───────────────────────────────────────────────────────

class GeometricGraspSampler:
    """
    Geometric antipodal grasp sampler.

    Produces top-down grasp candidates for a parallel-jaw gripper
    from depth images, using purely geometric analysis. No GPU required.

    Usage (standalone, no ROS):
        sampler = GeometricGraspSampler()
        candidates = sampler.plan_grasp(
            depth_img, (fx, fy, cx, cy),
            object_center_cam, tf_cam_to_base)

    Usage (from point cloud directly):
        candidates = sampler.plan_grasp_from_points(points_base)
    """

    def __init__(
        self,
        gripper_max_width: float = GRIPPER_MAX_OPENING_M,
        crop_radius: float = 0.06,
        num_antipodal_samples: int = 200,
        num_yaw_samples: int = 12,
        normal_search_radius: float = 0.015,
        antipodal_threshold: float = 0.80,
        min_points: int = 20,
        max_candidates: int = 10,
    ):
        self.gripper_max_width = gripper_max_width
        self.crop_radius = crop_radius
        self.num_antipodal_samples = num_antipodal_samples
        self.num_yaw_samples = num_yaw_samples
        self.normal_search_radius = normal_search_radius
        self.antipodal_threshold = antipodal_threshold
        self.min_points = min_points
        self.max_candidates = max_candidates

    def plan_grasp(
        self,
        depth_img: np.ndarray,
        camera_intrinsics: Tuple[float, float, float, float],
        object_center_cam: np.ndarray,
        tf_cam_to_base: np.ndarray,
    ) -> List[GraspCandidate]:
        """
        Full pipeline: depth image → scored grasp candidates.

        Args:
            depth_img: (H, W) uint16 depth in mm.
            camera_intrinsics: (fx, fy, cx, cy).
            object_center_cam: (3,) object center in camera optical frame.
            tf_cam_to_base: (4, 4) homogeneous transform from camera to base_link.

        Returns:
            List of GraspCandidate sorted by score (best first).
        """
        fx, fy, cx, cy = camera_intrinsics

        # 1. Depth → point cloud (camera frame)
        points_cam = depth_to_point_cloud(depth_img, fx, fy, cx, cy)
        if len(points_cam) < self.min_points:
            return []

        # 2. Crop around object (in camera frame for speed)
        cropped_cam = crop_around_point(
            points_cam, object_center_cam, self.crop_radius)
        if len(cropped_cam) < self.min_points:
            return []

        # 3. Transform cropped points to base_link
        pts_base = transform_points(cropped_cam, tf_cam_to_base)

        return self.plan_grasp_from_points(pts_base)

    def plan_grasp_from_points(
        self,
        points_base: np.ndarray,
    ) -> List[GraspCandidate]:
        """
        Generate grasp candidates from a point cloud already in base_link.

        Useful for standalone testing without camera data.
        """
        if len(points_base) < self.min_points:
            return []

        # 4. Estimate normals (orient upward in base_link)
        normals = estimate_normals(
            points_base,
            search_radius=self.normal_search_radius,
            orient_toward=np.array([0.0, 0.0, 10.0]),
        )

        # 5a. Antipodal sampling
        antipodal_grasps = sample_antipodal_grasps(
            points_base, normals,
            gripper_max_width=self.gripper_max_width,
            num_samples=self.num_antipodal_samples,
            antipodal_threshold=self.antipodal_threshold,
        )

        # 5b. Top-surface yaw sweep
        top_grasps = sample_top_grasps(
            points_base, normals,
            gripper_max_width=self.gripper_max_width,
            num_yaw_samples=self.num_yaw_samples,
        )

        # 6. Combine, score, sort
        all_grasps = antipodal_grasps + top_grasps
        for g in all_grasps:
            g.score()

        all_grasps.sort(key=lambda g: g.total_score, reverse=True)

        # 7. Deduplicate
        deduped = deduplicate_grasps(all_grasps)

        return deduped[:self.max_candidates]
