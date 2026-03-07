#!/usr/bin/env python3
"""
Viser-based Grasp Pose Visualizer

Visualize point clouds and grasp poses from topdown_grasp_demo.py snapshots
or live ROS data. Compare multiple grasp methods interactively.

Usage:
    # From latest snapshot (no ROS needed):
    python3 grasp_visualizer.py

    # From specific snapshot:
    python3 grasp_visualizer.py --snapshot ~/.tidybot_grasp_snapshots/2026-03-05_143022.npz

    # Live from ROS topics:
    python3 grasp_visualizer.py --live

Open http://localhost:8080 in your browser.
"""

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional

import numpy as np
import trimesh
import viser
from scipy.spatial.transform import Rotation

# Add the perception package to path for imports
_install_dir = (
    Path(__file__).resolve().parents[3]
    / 'install/tidybot_perception/lib/python3.10/site-packages'
)
if _install_dir.is_dir():
    sys.path.insert(0, str(_install_dir))

from tidybot_perception.grasp_geometry import (
    GRIPPER_MAX_OPENING_M,
    yaw_to_grasp_quaternion,
)
from tidybot_perception.geometric_grasp_sampler import (
    GeometricGraspSampler,
    estimate_normals,
)


# ── Data structures ──────────────────────────────────────────────────────────

@dataclass
class GraspResult:
    """A grasp pose for visualization."""
    center: np.ndarray   # (3,) xyz in base_link
    yaw: float           # finger-axis angle in XY (rad)
    width: float         # grip width (m)
    score: float         # 0-1 quality score
    label: str = ''      # optional text label

    @property
    def z_top(self):
        return float(self.center[2])


# Method type: takes points_base + keyword params, returns list of GraspResult
GraspMethod = Callable[..., List[GraspResult]]


# ── Grasp methods ────────────────────────────────────────────────────────────

def method_minwidth_sweep(
    points_base: np.ndarray,
    angle_step: int = 5,
    z_offset: float = -0.02,
    **kwargs,
) -> List[GraspResult]:
    """Min-width sweep (same as topdown_grasp_demo._analyze_object).

    Always returns the best grasp (minimum width angle), even if width
    exceeds gripper max — score reflects feasibility.
    """
    if len(points_base) < 20:
        return []

    centroid = np.mean(points_base, axis=0)
    z_top = float(np.max(points_base[:, 2]))
    pts_xy = points_base[:, :2]
    centered = pts_xy - centroid[:2]

    results = []
    for angle_deg in range(0, 180, angle_step):
        angle = np.radians(angle_deg)
        direction = np.array([np.cos(angle), np.sin(angle)])
        projections = centered @ direction
        width = float(np.ptp(projections))

        feasible = width <= GRIPPER_MAX_OPENING_M
        score = max(0.0, 1.0 - width / GRIPPER_MAX_OPENING_M) if feasible else 0.0

        grasp_center = np.array([centroid[0], centroid[1], z_top + z_offset])
        results.append(GraspResult(
            center=grasp_center,
            yaw=angle,
            width=width,
            score=score,
            label=f'{angle_deg}deg w={width*1000:.0f}mm{"" if feasible else " [WIDE]"}',
        ))

    results.sort(key=lambda g: g.score, reverse=True)
    return results


def method_antipodal(
    points_base: np.ndarray,
    num_samples: int = 200,
    antipodal_threshold: float = 0.80,
    **kwargs,
) -> List[GraspResult]:
    """Geometric antipodal sampling via GeometricGraspSampler."""
    sampler = GeometricGraspSampler(
        num_antipodal_samples=num_samples,
        antipodal_threshold=antipodal_threshold,
    )
    candidates = sampler.plan_grasp_from_points(points_base)
    return [
        GraspResult(
            center=c.center.copy(),
            yaw=c.yaw,
            width=c.width,
            score=c.total_score / 4.0,  # normalize to ~[0,1]
            label=f'antipodal s={c.total_score:.2f} w={c.width*1000:.0f}mm',
        )
        for c in candidates
    ]


def separate_floor(points: np.ndarray, distance_thresh: float = 0.008,
                    max_iterations: int = 200, min_pts: int = 20):
    """RANSAC plane fit to find dominant surface (floor/table).

    Returns (floor_z, object_points). floor_z is median Z of inliers.
    """
    n = len(points)
    if n < min_pts:
        return float(np.median(points[:, 2])), points

    best_inlier_mask = None
    best_count = 0

    rng = np.random.default_rng(42)
    for _ in range(max_iterations):
        idx = rng.choice(n, 3, replace=False)
        p0, p1, p2 = points[idx]

        v1 = p1 - p0
        v2 = p2 - p0
        normal = np.cross(v1, v2)
        norm_len = np.linalg.norm(normal)
        if norm_len < 1e-10:
            continue
        normal /= norm_len

        dists = np.abs((points - p0) @ normal)
        inlier_mask = dists < distance_thresh
        count = inlier_mask.sum()

        if count > best_count:
            best_count = count
            best_inlier_mask = inlier_mask

    if best_inlier_mask is None or best_count < 0.20 * n:
        return float(np.min(points[:, 2])), points

    floor_z = float(np.median(points[best_inlier_mask, 2]))
    outlier_mask = ~best_inlier_mask

    if outlier_mask.sum() < min_pts:
        return floor_z, points

    return floor_z, points[outlier_mask]


# Registry of available methods
METHODS: Dict[str, GraspMethod] = {
    'minwidth_sweep': method_minwidth_sweep,
    'antipodal': method_antipodal,
}

METHOD_COLORS = {
    'minwidth_sweep': (0, 200, 0),      # green
    'antipodal': (50, 100, 255),         # blue
}


# ── Gripper mesh ─────────────────────────────────────────────────────────────

def create_wx250s_gripper_mesh(
    center: np.ndarray,
    yaw: float,
    width: float,
    color: tuple = (0, 200, 0),
    alpha: int = 200,
) -> trimesh.Trimesh:
    """Create a WX250s-scale gripper mesh at the given grasp pose.

    The gripper approaches from above (fingers-down) with finger axis
    rotated by `yaw` in the XY plane.

    yaw_to_grasp_quaternion EE frame convention:
        local X -> approach direction (maps to world -Z, i.e. downward)
        local Y -> finger axis (maps to world yaw direction)
        local Z -> wrist up (maps to world perpendicular)

    So we build the mesh with:
        - fingers spread along local Y (at ±half_w)
        - fingers extend along local +X (approach/closing direction)
        - stem extends along local -X (away from object)
    """
    finger_length = 0.04
    finger_thickness = 0.006
    palm_thickness = 0.006
    stem_length = 0.04

    half_w = width / 2

    # Left finger — offset along +Y, extends along +X
    lf = trimesh.creation.box([finger_length, finger_thickness, finger_thickness])
    lf.apply_translation([finger_length / 2, half_w + finger_thickness / 2, 0])

    # Right finger — offset along -Y, extends along +X
    rf = trimesh.creation.box([finger_length, finger_thickness, finger_thickness])
    rf.apply_translation([finger_length / 2, -(half_w + finger_thickness / 2), 0])

    # Palm — connects fingers, along Y at X=0
    palm = trimesh.creation.box([palm_thickness, width + 2 * finger_thickness, finger_thickness])
    palm.apply_translation([0, 0, 0])

    # Stem — extends along -X (away from grasp point)
    stem = trimesh.creation.box([stem_length, finger_thickness, finger_thickness])
    stem.apply_translation([-stem_length / 2, 0, 0])

    gripper = trimesh.util.concatenate([lf, rf, palm, stem])

    # Apply the grasp orientation + position
    qw, qx, qy, qz = yaw_to_grasp_quaternion(yaw)
    rot = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()

    transform = np.eye(4)
    transform[:3, :3] = rot
    transform[:3, 3] = center
    gripper.apply_transform(transform)

    gripper.visual.face_colors = [color[0], color[1], color[2], alpha]
    return gripper


# ── Snapshot loading ─────────────────────────────────────────────────────────

def load_snapshot(path: Path) -> dict:
    """Load a snapshot NPZ file."""
    data = np.load(str(path), allow_pickle=True)
    result = {'points_base': data['points_base']}
    if 'points_base_raw' in data:
        result['points_base_raw'] = data['points_base_raw']
    if 'points_full_scene' in data:
        result['points_full_scene'] = data['points_full_scene']
    for key in ['z_offset', 'z_bias', 'pre_grasp_height', 'target_object', 'floor_z']:
        if key in data:
            result[key] = data[key].item() if data[key].ndim == 0 else data[key]
    return result


def find_snapshots() -> List[Path]:
    """Find all snapshot files, newest first."""
    snap_dir = Path.home() / '.tidybot_grasp_snapshots'
    if not snap_dir.exists():
        return []
    files = sorted(snap_dir.glob('*.npz'), key=lambda f: f.stat().st_mtime, reverse=True)
    return files


# ── Live ROS data ────────────────────────────────────────────────────────────

def grab_live_pointcloud() -> Optional[np.ndarray]:
    """Grab a point cloud from live ROS topics. Returns (N,3) or None."""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image, CameraInfo
        import tf2_ros
    except ImportError:
        print('ROS2 not available for live mode')
        return None

    rclpy.init()
    node = Node('grasp_viz_grabber')

    depth_data = {'img': None, 'info': None, 'tf': None}
    qos_be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

    from cv_bridge import CvBridge
    bridge = CvBridge()

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)

    def depth_cb(msg):
        depth_data['img'] = bridge.imgmsg_to_cv2(msg, '16UC1')

    def info_cb(msg):
        depth_data['info'] = msg

    node.create_subscription(Image, '/camera/depth/image_raw', depth_cb, qos_be)
    node.create_subscription(CameraInfo, '/camera/depth/camera_info', info_cb, 10)

    # Spin until we have data
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        if depth_data['img'] is not None and depth_data['info'] is not None:
            break

    if depth_data['img'] is None or depth_data['info'] is None:
        print('Could not get depth data from ROS topics')
        node.destroy_node()
        rclpy.shutdown()
        return None

    # Get TF
    try:
        import rclpy.time, rclpy.duration
        transform = tf_buffer.lookup_transform(
            'base_link', 'camera_depth_optical_frame',
            rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
        t = transform.transform.translation
        q = transform.transform.rotation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        tf_mat = np.eye(4, dtype=np.float32)
        tf_mat[:3, :3] = rot.as_matrix().astype(np.float32)
        tf_mat[:3, 3] = [t.x, t.y, t.z]
    except Exception as e:
        print(f'TF lookup failed: {e}')
        node.destroy_node()
        rclpy.shutdown()
        return None

    K = depth_data['info'].k
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    depth = depth_data['img']

    from tidybot_perception.geometric_grasp_sampler import depth_to_point_cloud, transform_points
    pts_cam = depth_to_point_cloud(depth, fx, fy, cx, cy)
    pts_base = transform_points(pts_cam, tf_mat)

    # Filter: above table
    pts_base = pts_base[pts_base[:, 2] > 0.005]

    node.destroy_node()
    rclpy.shutdown()
    return pts_base


# ── Main visualizer ──────────────────────────────────────────────────────────

class GraspVisualizer:
    def __init__(self, points_base: np.ndarray, params: dict = None):
        self.params = params or {}
        self.server = viser.ViserServer()
        self._grasp_handles: list = []
        self._method_results: Dict[str, List[GraspResult]] = {}

        # Store raw (pre-floor-filtered) points if available
        self.points_base_raw = self.params.pop('points_base_raw', None)
        if self.points_base_raw is None:
            self.points_base_raw = points_base.copy()

        # Store full scene cloud for TF debugging
        self.points_full_scene = self.params.pop('points_full_scene', None)

        # Apply floor separation if not already done
        self.points_base, self.params = self._ensure_floor_filtered(
            points_base, self.params)

    @staticmethod
    def _ensure_floor_filtered(points: np.ndarray, params: dict):
        """Apply floor filtering if floor_z not already set in params."""
        params = dict(params)
        if 'floor_z' not in params:
            floor_z, obj_pts = separate_floor(points)
            params['floor_z'] = floor_z
            n_before = len(points)
            points = obj_pts
            print(f'Floor detected at Z={floor_z:.3f}m, '
                  f'filtered {n_before} -> {len(points)} points')
        return points, params

    def _color_by_height(self, points: np.ndarray) -> np.ndarray:
        """Color points by Z height: blue (low) -> green -> red (high)."""
        z = points[:, 2]
        z_min, z_max = z.min(), z.max()
        if z_max - z_min < 1e-6:
            return np.full((len(points), 3), 128, dtype=np.uint8)

        t = (z - z_min) / (z_max - z_min)
        colors = np.zeros((len(points), 3), dtype=np.uint8)
        # blue -> green -> red
        colors[:, 0] = (np.clip(2 * t - 1, 0, 1) * 255).astype(np.uint8)
        colors[:, 1] = (np.clip(1 - np.abs(2 * t - 1), 0, 1) * 255).astype(np.uint8)
        colors[:, 2] = (np.clip(1 - 2 * t, 0, 1) * 255).astype(np.uint8)
        return colors

    def _add_floor_plane(self, floor_z: float, centroid: np.ndarray):
        """Add a translucent plane at floor_z for reference."""
        plane_size = 0.20
        plane = trimesh.creation.box([plane_size, plane_size, 0.001])
        plane.apply_translation([centroid[0], centroid[1], floor_z])
        plane.visual.face_colors = [255, 165, 0, 60]  # orange, translucent
        self.server.scene.add_mesh_trimesh(name='/scene/floor_plane', mesh=plane)

    def _clear_grasps(self):
        for h in self._grasp_handles:
            h.remove()
        self._grasp_handles.clear()

    @staticmethod
    def _score_to_color(score: float) -> tuple:
        """Map score [0,1] to color: red (0) -> yellow (0.5) -> green (1)."""
        s = max(0.0, min(1.0, score))
        if s < 0.5:
            # red -> yellow
            t = s * 2
            return (int(220), int(t * 200), int(20))
        else:
            # yellow -> green
            t = (s - 0.5) * 2
            return (int(220 * (1 - t)), int(200), int(20))

    def _show_grasps(self, method_name: str, grasps: List[GraspResult],
                     color: tuple, show_pregrasp: bool, pre_grasp_height: float,
                     max_show: int = 20):
        """Add gripper meshes for a method's grasps, colored by score."""
        for i, g in enumerate(grasps[:max_show]):
            c = self._score_to_color(g.score)
            mesh = create_wx250s_gripper_mesh(g.center, g.yaw, g.width, color=c)
            h = self.server.scene.add_mesh_trimesh(
                name=f'/grasps/{method_name}/grasp_{i}',
                mesh=mesh,
            )
            self._grasp_handles.append(h)

            if show_pregrasp:
                pg_center = g.center.copy()
                pg_center[2] += pre_grasp_height
                pg_mesh = create_wx250s_gripper_mesh(
                    pg_center, g.yaw, g.width,
                    color=c, alpha=80,
                )
                h2 = self.server.scene.add_mesh_trimesh(
                    name=f'/grasps/{method_name}/pregrasp_{i}',
                    mesh=pg_mesh,
                )
                self._grasp_handles.append(h2)

    @staticmethod
    def _crop_xy(points: np.ndarray, radius: float) -> np.ndarray:
        """Crop points to within `radius` of centroid in XY plane."""
        if radius <= 0:
            return points
        centroid_xy = np.mean(points[:, :2], axis=0)
        dists = np.linalg.norm(points[:, :2] - centroid_xy, axis=1)
        return points[dists < radius]

    def _recompute_and_show(self, enabled_methods, gui_params,
                            show_pregrasp, pre_grasp_height, max_grasps,
                            crop_radius=0.0):
        """Recompute all enabled methods and update the scene."""
        self._clear_grasps()
        self._method_results.clear()

        # Optionally crop XY around centroid before running methods
        pts = self.points_base
        if crop_radius > 0:
            pts = self._crop_xy(pts, crop_radius)
            if len(pts) < 20:
                pts = self.points_base  # fallback if crop too aggressive

        for name in enabled_methods:
            if name not in METHODS:
                continue
            method_fn = METHODS[name]
            grasps = method_fn(pts, **gui_params)
            self._method_results[name] = grasps
            color = METHOD_COLORS.get(name, (200, 200, 200))
            self._show_grasps(name, grasps, color, show_pregrasp,
                              pre_grasp_height, max_show=max_grasps)

    def _show_raw_cloud(self, visible: bool):
        """Show or hide the raw (pre-floor-filtered) point cloud."""
        if visible and self.points_base_raw is not None:
            raw_colors = np.full((len(self.points_base_raw), 3), 100, dtype=np.uint8)
            raw_colors[:, 0] = 160  # grayish-red tint for raw
            self.server.scene.add_point_cloud(
                name='/scene/pointcloud_raw',
                points=self.points_base_raw.astype(np.float32),
                colors=raw_colors,
                point_size=0.002,
            )
        else:
            self.server.scene.add_point_cloud(
                name='/scene/pointcloud_raw',
                points=np.zeros((0, 3), dtype=np.float32),
                colors=np.zeros((0, 3), dtype=np.uint8),
                point_size=0.002,
            )

    def _show_full_scene(self, visible: bool):
        """Show or hide the full (uncropped) scene point cloud."""
        if visible and self.points_full_scene is not None:
            colors = np.full((len(self.points_full_scene), 3), 140, dtype=np.uint8)
            self.server.scene.add_point_cloud(
                name='/scene/pointcloud_full',
                points=self.points_full_scene.astype(np.float32),
                colors=colors,
                point_size=0.002,
            )
        else:
            self.server.scene.add_point_cloud(
                name='/scene/pointcloud_full',
                points=np.zeros((0, 3), dtype=np.float32),
                colors=np.zeros((0, 3), dtype=np.uint8),
                point_size=0.002,
            )

    def run(self):
        print(f'Viser server starting at http://localhost:8080')
        print(f'Point cloud: {len(self.points_base)} points'
              f' (raw: {len(self.points_base_raw)})')

        # Add point cloud
        colors = self._color_by_height(self.points_base)
        self.server.scene.add_point_cloud(
            name='/scene/pointcloud',
            points=self.points_base.astype(np.float32),
            colors=colors,
            point_size=0.003,
        )

        # Add origin frame
        self.server.scene.add_frame(
            name='/scene/origin',
            axes_length=0.1,
            axes_radius=0.003,
        )

        # Add centroid marker
        centroid = np.mean(self.points_base, axis=0)
        self.server.scene.add_frame(
            name='/scene/centroid',
            axes_length=0.03,
            axes_radius=0.002,
            position=centroid.astype(np.float32),
        )

        # Add floor plane if known
        floor_z = self.params.get('floor_z')
        if floor_z is not None:
            self._add_floor_plane(floor_z, centroid)
            print(f'Floor Z: {floor_z:.3f}m')

        # GUI controls
        with self.server.gui.add_folder('Methods'):
            method_toggles = {}
            for name in METHODS:
                method_toggles[name] = self.server.gui.add_checkbox(
                    name, initial_value=True)

        with self.server.gui.add_folder('Point Cloud'):
            show_raw_cb = self.server.gui.add_checkbox(
                'Show raw (pre-floor-filter)', initial_value=False)
            show_raw_cb.on_update(lambda _: self._show_raw_cloud(show_raw_cb.value))
            show_full_cb = self.server.gui.add_checkbox(
                'Show full scene (TF debug)', initial_value=False)
            show_full_cb.on_update(lambda _: self._show_full_scene(show_full_cb.value))
            if self.points_full_scene is None:
                show_full_cb.disabled = True

        with self.server.gui.add_folder('Parameters'):
            z_offset_slider = self.server.gui.add_slider(
                'z_offset (mm)', min=-50, max=20, step=1,
                initial_value=self.params.get('z_offset', -0.02) * 1000)
            pre_grasp_slider = self.server.gui.add_slider(
                'pre_grasp_height (mm)', min=20, max=200, step=5,
                initial_value=self.params.get('pre_grasp_height', 0.10) * 1000)
            angle_step_slider = self.server.gui.add_slider(
                'angle_step (deg)', min=1, max=30, step=1,
                initial_value=5)
            crop_radius_slider = self.server.gui.add_slider(
                'crop_radius (mm)', min=0, max=150, step=5,
                initial_value=40)
            max_grasps_slider = self.server.gui.add_slider(
                'max_grasps', min=1, max=50, step=1,
                initial_value=10)
            show_pregrasp_cb = self.server.gui.add_checkbox(
                'Show pre-grasp poses', initial_value=True)

        with self.server.gui.add_folder('Actions'):
            recompute_btn = self.server.gui.add_button('Recompute Grasps')

            # Snapshot selector
            snapshots = find_snapshots()
            if snapshots:
                snap_names = [f.stem for f in snapshots]
                snap_dropdown = self.server.gui.add_dropdown(
                    'Load snapshot', options=snap_names,
                    initial_value=snap_names[0])
                load_snap_btn = self.server.gui.add_button('Load Snapshot')

        with self.server.gui.add_folder('Info'):
            info_text = self.server.gui.add_text(
                'Status', initial_value=f'{len(self.points_base)} points loaded')

        def do_recompute(_=None):
            enabled = [n for n, cb in method_toggles.items() if cb.value]
            params = {
                'z_offset': z_offset_slider.value / 1000.0,
                'angle_step': int(angle_step_slider.value),
            }
            crop_r = crop_radius_slider.value / 1000.0
            self._recompute_and_show(
                enabled, params,
                show_pregrasp_cb.value,
                pre_grasp_slider.value / 1000.0,
                int(max_grasps_slider.value),
                crop_radius=crop_r,
            )
            total = sum(len(v) for v in self._method_results.values())
            details = ', '.join(
                f'{k}: {len(v)}' for k, v in self._method_results.items())
            info_text.value = f'{total} grasps ({details})'

        recompute_btn.on_click(do_recompute)

        # Wire checkboxes and sliders to auto-recompute
        for cb in method_toggles.values():
            cb.on_update(lambda _: do_recompute())
        show_pregrasp_cb.on_update(lambda _: do_recompute())

        if snapshots:
            def do_load_snapshot(_=None):
                idx = snap_names.index(snap_dropdown.value)
                snap = load_snapshot(snapshots[idx])
                new_params = {k: v for k, v in snap.items()
                              if k not in ('points_base', 'points_base_raw',
                                           'points_full_scene')}
                self.points_base_raw = snap.get('points_base_raw', snap['points_base'].copy())
                self.points_full_scene = snap.get('points_full_scene')
                self.points_base, new_params = self._ensure_floor_filtered(
                    snap['points_base'], new_params)
                self.params.update(new_params)
                # Update auxiliary clouds if visible
                self._show_raw_cloud(show_raw_cb.value)
                self._show_full_scene(show_full_cb.value)
                show_full_cb.disabled = self.points_full_scene is None

                # Update point cloud
                new_colors = self._color_by_height(self.points_base)
                self.server.scene.add_point_cloud(
                    name='/scene/pointcloud',
                    points=self.points_base.astype(np.float32),
                    colors=new_colors,
                    point_size=0.003,
                )

                # Update centroid
                new_centroid = np.mean(self.points_base, axis=0)
                self.server.scene.add_frame(
                    name='/scene/centroid',
                    axes_length=0.03,
                    axes_radius=0.002,
                    position=new_centroid.astype(np.float32),
                )

                if 'z_offset' in snap:
                    z_offset_slider.value = snap['z_offset'] * 1000
                if 'pre_grasp_height' in snap:
                    pre_grasp_slider.value = snap['pre_grasp_height'] * 1000

                info_text.value = f'Loaded {snap_dropdown.value} ({len(self.points_base)} pts)'
                do_recompute()

            load_snap_btn.on_click(do_load_snapshot)

        # Initial computation
        do_recompute()

        print('Visualizer ready. Open http://localhost:8080')
        print('Press Ctrl+C to exit.')
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print('\nShutting down.')


# ── CLI entry point ──────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Viser Grasp Pose Visualizer')
    parser.add_argument('--snapshot', type=str, default=None,
                        help='Path to snapshot NPZ file')
    parser.add_argument('--live', action='store_true',
                        help='Grab point cloud from live ROS topics')
    args = parser.parse_args()

    points_base = None
    params = {}

    if args.live:
        print('Grabbing point cloud from ROS topics...')
        points_base = grab_live_pointcloud()
        if points_base is None:
            print('Failed to grab live data. Falling back to snapshots.')

    if points_base is None and args.snapshot:
        snap_path = Path(args.snapshot)
        if not snap_path.exists():
            print(f'Snapshot not found: {snap_path}')
            sys.exit(1)
        snap = load_snapshot(snap_path)
        points_base = snap['points_base']
        params = {k: v for k, v in snap.items() if k not in ('points_base',)}
        print(f'Loaded snapshot: {snap_path.name} ({len(points_base)} points)')

    if points_base is None:
        # Try latest snapshot
        snapshots = find_snapshots()
        if snapshots:
            snap = load_snapshot(snapshots[0])
            points_base = snap['points_base']
            params = {k: v for k, v in snap.items() if k not in ('points_base',)}
            print(f'Loaded latest snapshot: {snapshots[0].name} ({len(points_base)} points)')
        else:
            print('No snapshot found and --live not specified.')
            print('Run topdown_grasp_demo.py first to generate a snapshot,')
            print('or use --live to grab from ROS topics.')
            sys.exit(1)

    viz = GraspVisualizer(points_base, params)
    viz.run()


if __name__ == '__main__':
    main()
