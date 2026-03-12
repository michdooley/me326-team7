#!/usr/bin/env python3
"""
Grasp Pipeline Step-by-Step Visualizer

Interactive Viser visualization showing 7 stages of the grasp pipeline,
all perfectly aligned in 3D so screenshots overlay without moving the camera.

Stages:
    1. Normalized depth image
    2. RGB image + YOLO bounding box
    3. Cropped depth image (YOLO region)
    4. Cropped point cloud from depth
    5. RANSAC-filtered point cloud (floor removed)
    6. Min-sweep grasps (all candidates)
    7. Selected best grasp

Usage:
    python3 grasp_pipeline_visualizer.py --pair pair_0008
    python3 grasp_pipeline_visualizer.py --pair pair_0008 --target banana

Open http://localhost:8080 in your browser.
"""

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import trimesh
import viser
from scipy.io import loadmat
from scipy.spatial.transform import Rotation

# ── Grasp constants (matching pick_and_bring.py) ────────────────────────────

GRASP_BBOX_PAD = 1.3
GRASP_MIN_DEPTH_M = 0.15
GRASP_MAX_DEPTH_M = 5.0
GRIPPER_MAX_OPENING_M = 0.048

# D435 depth sensor intrinsics (wider FOV than RGB)
D435_DEPTH_K = (380.4629821777344, 380.4629821777344,
                324.5294494628906, 241.70526123046875)

STAGE_LABELS = {
    1: 'Depth Image (normalized)',
    2: 'RGB + YOLO Bounding Box',
    3: 'Cropped Depth (YOLO region)',
    4: 'Cropped Point Cloud',
    5: 'Filtered Point Cloud (RANSAC)',
    6: 'Min-Sweep Grasps (all)',
    7: 'Selected Grasp',
}


# ── Data structures ─────────────────────────────────────────────────────────

@dataclass
class GraspResult:
    """A grasp pose for visualization."""
    center: np.ndarray   # (3,) xyz
    yaw: float           # finger-axis angle in XY (rad)
    width: float         # grip width (m)
    score: float         # 0-1 quality score
    label: str = ''


# ── Pipeline functions ──────────────────────────────────────────────────────

def normalize_depth_colormap(depth_mm: np.ndarray) -> np.ndarray:
    """Normalize uint16 depth to colorized uint8 BGR image."""
    valid = depth_mm[depth_mm > 0]
    if len(valid) == 0:
        return np.zeros((*depth_mm.shape, 3), dtype=np.uint8)
    vmin, vmax = float(valid.min()), float(valid.max())
    normalized = np.zeros_like(depth_mm, dtype=np.uint8)
    mask = depth_mm > 0
    normalized[mask] = ((depth_mm[mask].astype(np.float32) - vmin)
                        / max(vmax - vmin, 1) * 255).astype(np.uint8)
    return cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)


def crop_depth_roi(depth_mm: np.ndarray, cx: int, cy: int,
                   bbox_w: float, bbox_h: float, pad: float = GRASP_BBOX_PAD
                   ) -> Tuple[np.ndarray, int, int, int, int]:
    """Crop depth to padded YOLO bbox. Returns (roi, u0, v0, u1, v1)."""
    h, w = depth_mm.shape
    half_w = int(bbox_w * pad / 2)
    half_h = int(bbox_h * pad / 2)
    u0 = max(0, cx - half_w)
    v0 = max(0, cy - half_h)
    u1 = min(w, cx + half_w)
    v1 = min(h, cy + half_h)
    return depth_mm[v0:v1, u0:u1], u0, v0, u1, v1


def depth_roi_to_points(depth_roi: np.ndarray, u_offset: int, v_offset: int,
                        fx: float, fy: float, cx: float, cy: float
                        ) -> np.ndarray:
    """Back-project cropped depth ROI to point cloud in camera frame."""
    depth_m = depth_roi.astype(np.float32) / 1000.0
    roi_h, roi_w = depth_m.shape

    u_grid, v_grid = np.meshgrid(
        np.arange(roi_w) + u_offset,
        np.arange(roi_h) + v_offset)

    mask = (depth_m > GRASP_MIN_DEPTH_M) & (depth_m < GRASP_MAX_DEPTH_M)
    z = depth_m[mask]
    u = u_grid[mask].astype(np.float32)
    v = v_grid[mask].astype(np.float32)

    if len(z) < 20:
        return np.zeros((0, 3), dtype=np.float32)

    x_cam = (u - cx) * z / fx
    y_cam = (v - cy) * z / fy
    return np.stack([x_cam, y_cam, z], axis=1)


def separate_floor(points: np.ndarray, distance_thresh: float = 0.008,
                   max_iterations: int = 200, min_pts: int = 20):
    """RANSAC plane fit. Returns (plane_info, object_points).

    plane_info is a dict with:
        'normal': unit normal of the dominant plane
        'point':  centroid of plane inliers
        'inlier_points': the inlier points themselves
    """
    n = len(points)
    if n < min_pts:
        return None, points

    best_inlier_mask = None
    best_count = 0
    best_normal = None
    best_p0 = None

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
            best_normal = normal
            best_p0 = p0

    if best_inlier_mask is None or best_count < 0.20 * n:
        return None, points

    inlier_pts = points[best_inlier_mask]
    plane_center = np.mean(inlier_pts, axis=0)
    outlier_mask = ~best_inlier_mask

    plane_info = {
        'normal': best_normal,
        'point': plane_center,
        'inlier_points': inlier_pts,
    }

    if outlier_mask.sum() < min_pts:
        return plane_info, points

    return plane_info, points[outlier_mask]


def build_plane_basis(normal: np.ndarray):
    """Build an orthonormal basis (u, v, n) from a plane normal.

    Returns (u, v) tangent vectors in the plane. u is chosen to be as
    close to the world X-axis as possible for a stable reference.
    """
    n = normal / np.linalg.norm(normal)
    # Pick a reference that isn't parallel to the normal
    ref = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(n, ref)) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    u = np.cross(n, ref)
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    v /= np.linalg.norm(v)
    return u, v


def method_minwidth_sweep(points: np.ndarray, plane_normal: np.ndarray,
                          angle_step: int = 5) -> List[GraspResult]:
    """Max-clearance yaw sweep aligned to the detected floor plane.

    The sweep is done in the plane defined by plane_normal (the table
    surface). Fingers spread in the tangent plane; the gripper approaches
    along the plane normal (perpendicular to the table).
    """
    if len(points) < 20:
        return []

    # Build tangent basis in the floor plane
    u_axis, v_axis = build_plane_basis(plane_normal)

    centroid = np.mean(points, axis=0)
    # Project points into the tangent plane (2D)
    centered_3d = points - centroid
    proj_u = centered_3d @ u_axis
    proj_v = centered_3d @ v_axis

    results = []
    for angle_deg in range(0, 180, angle_step):
        angle = np.radians(angle_deg)
        # Finger direction in the tangent plane
        finger_2d = np.array([np.cos(angle), np.sin(angle)])
        projections = proj_u * finger_2d[0] + proj_v * finger_2d[1]
        proj_min = float(projections.min())
        proj_max = float(projections.max())
        width = proj_max - proj_min

        center_offset = (proj_min + proj_max) / 2.0
        clearance = (GRIPPER_MAX_OPENING_M - width) / 2.0
        feasible = width <= GRIPPER_MAX_OPENING_M
        # Positive score for feasible grasps; for infeasible, use negative
        # clearance so the narrowest (least infeasible) still ranks highest
        if feasible:
            score = clearance / (GRIPPER_MAX_OPENING_M / 2.0)
        else:
            score = clearance / (GRIPPER_MAX_OPENING_M / 2.0)  # negative

        # Grasp center in 3D: centroid + offset along finger direction in plane
        finger_3d = finger_2d[0] * u_axis + finger_2d[1] * v_axis
        grasp_center = centroid + center_offset * finger_3d

        results.append(GraspResult(
            center=grasp_center,
            yaw=angle,
            width=width,
            score=score,
            label=f'{angle_deg}deg w={width*1000:.0f}mm clr={clearance*1000:.1f}mm'
                  f'{"" if feasible else " [WIDE]"}',
        ))

    results.sort(key=lambda g: g.score, reverse=True)
    return results


def create_gripper_mesh(center: np.ndarray, yaw: float, width: float,
                        plane_normal: np.ndarray,
                        color: tuple = (0, 200, 0), alpha: int = 200
                        ) -> trimesh.Trimesh:
    """Create a WX250s gripper mesh approaching along the plane normal.

    The gripper approaches perpendicular to the table (along plane_normal),
    with fingers spread in the tangent plane at the given yaw angle.
    """
    finger_length = 0.04
    finger_thickness = 0.006
    palm_thickness = 0.006
    stem_length = 0.04

    half_w = width / 2

    # Build gripper in local frame:
    #   local X = finger spread axis
    #   local Z = approach direction (fingers extend along +Z, stem along -Z)
    lf = trimesh.creation.box([finger_thickness, finger_thickness, finger_length])
    lf.apply_translation([half_w + finger_thickness / 2, 0, finger_length / 2])

    rf = trimesh.creation.box([finger_thickness, finger_thickness, finger_length])
    rf.apply_translation([-(half_w + finger_thickness / 2), 0, finger_length / 2])

    palm = trimesh.creation.box([width + 2 * finger_thickness, finger_thickness, palm_thickness])
    palm.apply_translation([0, 0, 0])

    stem = trimesh.creation.box([finger_thickness, finger_thickness, stem_length])
    stem.apply_translation([0, 0, -stem_length / 2])

    gripper = trimesh.util.concatenate([lf, rf, palm, stem])

    # Build rotation: local +Z maps to -plane_normal (approach INTO table),
    # local X maps to finger direction in the tangent plane
    u_axis, v_axis = build_plane_basis(plane_normal)
    finger_3d = np.cos(yaw) * u_axis + np.sin(yaw) * v_axis
    perp_3d = np.cross(-plane_normal, finger_3d)  # local Y

    rot = np.column_stack([finger_3d, perp_3d, -plane_normal])

    transform = np.eye(4)
    transform[:3, :3] = rot
    transform[:3, 3] = center
    gripper.apply_transform(transform)

    gripper.visual.face_colors = [color[0], color[1], color[2], alpha]
    return gripper


def score_to_color(score: float) -> tuple:
    """Map score to color: red (<=0) -> yellow (0.5) -> green (1)."""
    s = max(0.0, min(1.0, score))  # clamp negatives to red
    if s < 0.5:
        t = s * 2
        return (int(220), int(t * 200), int(20))
    else:
        t = (s - 0.5) * 2
        return (int(220 * (1 - t)), int(200), int(20))


def color_by_depth(points: np.ndarray) -> np.ndarray:
    """Color points by Z depth: blue (near) -> green -> red (far)."""
    z = points[:, 2]
    z_min, z_max = z.min(), z.max()
    if z_max - z_min < 1e-6:
        return np.full((len(points), 3), 128, dtype=np.uint8)
    t = (z - z_min) / (z_max - z_min)
    colors = np.zeros((len(points), 3), dtype=np.uint8)
    colors[:, 0] = (np.clip(2 * t - 1, 0, 1) * 255).astype(np.uint8)
    colors[:, 1] = (np.clip(1 - np.abs(2 * t - 1), 0, 1) * 255).astype(np.uint8)
    colors[:, 2] = (np.clip(1 - 2 * t, 0, 1) * 255).astype(np.uint8)
    return colors


# ── Image-as-plane rendering ───────────────────────────────────────────────

def add_image_plane(server: viser.ViserServer, name: str,
                    image_bgr: np.ndarray, z_depth: float,
                    fx: float, fy: float, cx: float, cy: float,
                    full_w: int, full_h: int,
                    u0: int = 0, v0: int = 0,
                    u1: Optional[int] = None, v1: Optional[int] = None,
                    visible: bool = True):
    """Place an image in the 3D scene using viser's add_image.

    Computes world-space dimensions from the pinhole model so that the image
    pixels align exactly with back-projected 3D points at the same depth.

    u0,v0,u1,v1 specify the pixel region this image covers in the full frame.
    The image is positioned so its center matches the correct 3D location.
    """
    if u1 is None:
        u1 = full_w
    if v1 is None:
        v1 = full_h

    # World-space width and height of the pixel region at z_depth
    render_width = (u1 - u0) * z_depth / fx
    render_height = (v1 - v0) * z_depth / fy

    # Center of the pixel region in world coords
    u_center = (u0 + u1) / 2.0
    v_center = (v0 + v1) / 2.0
    x_center = (u_center - cx) * z_depth / fx
    y_center = (v_center - cy) * z_depth / fy

    # Convert BGR to RGB for viser
    rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

    return server.scene.add_image(
        name, image=rgb,
        render_width=render_width, render_height=render_height,
        position=(x_center, y_center, z_depth),
        visible=visible,
    )


# ── YOLO detection ──────────────────────────────────────────────────────────

def run_yolo(rgb_bgr: np.ndarray, target: Optional[str] = None
             ) -> Optional[dict]:
    """Run YOLOv8 on an image. Returns best detection dict or None."""
    try:
        from ultralytics import YOLO
    except ImportError:
        print('ultralytics not installed. Run: pip install ultralytics')
        return None

    model = YOLO('yolov8n.pt')
    results = model(rgb_bgr, verbose=False)

    if not results or len(results[0].boxes) == 0:
        print('YOLO: no detections')
        return None

    boxes = results[0].boxes
    names = results[0].names

    best = None
    best_conf = -1.0

    for i in range(len(boxes)):
        cls_id = int(boxes.cls[i])
        cls_name = names[cls_id]
        conf = float(boxes.conf[i])

        if target and target.lower() not in cls_name.lower():
            continue

        if conf > best_conf:
            best_conf = conf
            x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)
            best = {
                'class': cls_name,
                'confidence': conf,
                'x1': int(x1), 'y1': int(y1),
                'x2': int(x2), 'y2': int(y2),
                'cx': int((x1 + x2) / 2),
                'cy': int((y1 + y2) / 2),
                'w': int(x2 - x1),
                'h': int(y2 - y1),
            }

    if best:
        print(f'YOLO: detected "{best["class"]}" ({best["confidence"]:.2f}) '
              f'at ({best["cx"]}, {best["cy"]}) size {best["w"]}x{best["h"]}')
    else:
        print(f'YOLO: no detections matching target="{target}"')

    return best


# ── Pipeline computation ───────────────────────────────────────────────────

def align_depth_to_rgb(depth_mm: np.ndarray, rgb_bgr: np.ndarray,
                       rgb_K: Tuple[float, float, float, float],
                       depth_K: Tuple[float, float, float, float] = D435_DEPTH_K
                       ) -> np.ndarray:
    """Align depth image from depth-sensor FOV to RGB-sensor FOV.

    The D435 depth sensor has a wider FOV (lower focal length) than the
    RGB sensor. This remaps depth pixels so they match the RGB image.
    """
    old_fx, old_fy, old_cx, old_cy = depth_K
    new_fx, new_fy, new_cx, new_cy = rgb_K

    K_old = np.array([[old_fx, 0, old_cx],
                       [0, old_fy, old_cy],
                       [0, 0, 1]])
    K_new = np.array([[new_fx, 0, new_cx],
                       [0, new_fy, new_cy],
                       [0, 0, 1]])

    h, w = rgb_bgr.shape[:2]
    K_new_inv = np.linalg.inv(K_new)

    x, y = np.meshgrid(np.arange(w), np.arange(h))
    homogeneous = np.stack([x.ravel(), y.ravel(), np.ones_like(x).ravel()], axis=-1).T
    old_coords = K_old @ K_new_inv @ homogeneous
    old_coords /= old_coords[2, :]

    map_x = old_coords[0, :].reshape(h, w).astype(np.float32)
    map_y = old_coords[1, :].reshape(h, w).astype(np.float32)

    return cv2.remap(depth_mm, map_x, map_y,
                     interpolation=cv2.INTER_NEAREST)


def compute_pipeline(depth_mm_raw: np.ndarray, rgb_bgr: np.ndarray,
                     fx: float, fy: float, cx: float, cy: float,
                     detection: dict) -> dict:
    """Compute all 7 pipeline stages from raw data. Returns dict of results."""
    results = {}

    # Align depth to RGB intrinsics (D435 depth has wider FOV than RGB)
    depth_mm = align_depth_to_rgb(
        depth_mm_raw, rgb_bgr, rgb_K=(fx, fy, cx, cy))
    print(f'Depth aligned to RGB intrinsics '
          f'(depth FOV: f={D435_DEPTH_K[0]:.0f}, '
          f'RGB FOV: f={fx:.0f})')

    # Stage 1: Normalized depth (aligned)
    results['depth_colored'] = normalize_depth_colormap(depth_mm)

    # Stage 2: RGB + YOLO bbox
    rgb_bbox = rgb_bgr.copy()
    cv2.rectangle(rgb_bbox,
                  (detection['x1'], detection['y1']),
                  (detection['x2'], detection['y2']),
                  (0, 255, 0), 3)
    cv2.putText(rgb_bbox,
                f'{detection["class"]} {detection["confidence"]:.2f}',
                (detection['x1'], detection['y1'] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    results['rgb_bbox'] = rgb_bbox

    # Stage 3: Cropped depth
    roi, u0, v0, u1, v1 = crop_depth_roi(
        depth_mm, detection['cx'], detection['cy'],
        detection['w'], detection['h'])
    results['depth_roi'] = roi
    results['crop_bounds'] = (u0, v0, u1, v1)
    results['depth_roi_colored'] = normalize_depth_colormap(roi)

    # Also draw crop region on the RGB bbox image
    cv2.rectangle(rgb_bbox,
                  (u0, v0), (u1, v1),
                  (0, 165, 255), 2)  # orange for padded crop

    # Stage 4: Cropped point cloud (using RGB intrinsics since depth is aligned)
    points_cam = depth_roi_to_points(roi, u0, v0, fx, fy, cx, cy)
    results['points_raw'] = points_cam
    print(f'Stage 4: {len(points_cam)} points from cropped depth')

    # Stage 5: RANSAC floor separation
    if len(points_cam) >= 20:
        plane_info, obj_pts = separate_floor(points_cam)
        results['plane_info'] = plane_info
        results['points_filtered'] = obj_pts
        if plane_info is not None:
            n = plane_info['normal']
            p = plane_info['point']
            print(f'Stage 5: RANSAC {len(points_cam)} -> {len(obj_pts)} points, '
                  f'plane normal=({n[0]:.2f},{n[1]:.2f},{n[2]:.2f}), '
                  f'center=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})')
        else:
            print(f'Stage 5: RANSAC found no dominant plane, '
                  f'keeping all {len(obj_pts)} points')
    else:
        results['plane_info'] = None
        results['points_filtered'] = points_cam

    # Stage 6: Min-sweep grasps (in the floor plane coordinate system)
    plane_info = results.get('plane_info')
    plane_normal = plane_info['normal'] if plane_info is not None else np.array([0., -1., 0.])
    results['plane_normal'] = plane_normal
    grasps = method_minwidth_sweep(results['points_filtered'], plane_normal)
    results['all_grasps'] = grasps
    print(f'Stage 6: {len(grasps)} grasp candidates')

    # Stage 7: Best grasp
    if grasps:
        results['best_grasp'] = grasps[0]
        g = grasps[0]
        print(f'Stage 7: best grasp score={g.score:.2f}, '
              f'width={g.width*1000:.0f}mm, {g.label}')
    else:
        results['best_grasp'] = None

    return results


# ── Viser visualization ────────────────────────────────────────────────────

class PipelineVisualizer:
    def __init__(self, depth_mm: np.ndarray, rgb_bgr: np.ndarray,
                 fx: float, fy: float, cx: float, cy: float,
                 pipeline: dict):
        self.fx, self.fy = fx, fy
        self.cx, self.cy = cx, cy
        self.pipeline = pipeline
        self.img_h, self.img_w = depth_mm.shape

        # Compute image plane depth: use median Z of filtered object points
        pts = pipeline.get('points_filtered', pipeline.get('points_raw'))
        if pts is not None and len(pts) > 0:
            self.plane_z = float(np.median(pts[:, 2]))
        else:
            valid = depth_mm[depth_mm > 0]
            self.plane_z = float(np.median(valid)) / 1000.0 if len(valid) > 0 else 0.5

        self.server = viser.ViserServer()
        self._scene_handles = {}
        self._build_scene()
        self._build_gui()
        self._setup_camera()

        print(f'\nVisualizer ready at http://localhost:8080')
        print(f'Image plane at z={self.plane_z:.3f}m')

    def _build_scene(self):
        p = self.pipeline
        z = self.plane_z

        # Stage 1: Depth image as plane in 3D scene
        h = add_image_plane(
            self.server, '/stage1/depth_plane',
            p['depth_colored'], z, self.fx, self.fy, self.cx, self.cy,
            self.img_w, self.img_h)
        self._scene_handles['stage1'] = [h]

        # Stage 2: RGB + YOLO bbox as plane in 3D scene
        h = add_image_plane(
            self.server, '/stage2/rgb_plane',
            p['rgb_bbox'], z, self.fx, self.fy, self.cx, self.cy,
            self.img_w, self.img_h)
        self._scene_handles['stage2'] = [h]

        # Stage 3: Cropped depth as plane (positioned at crop region)
        u0, v0, u1, v1 = p['crop_bounds']
        h = add_image_plane(
            self.server, '/stage3/crop_plane',
            p['depth_roi_colored'], z, self.fx, self.fy, self.cx, self.cy,
            self.img_w, self.img_h,
            u0=u0, v0=v0, u1=u1, v1=v1)
        self._scene_handles['stage3'] = [h]

        # Stage 4: Raw point cloud
        pts_raw = p['points_raw']
        handles4 = []
        if len(pts_raw) > 0:
            colors4 = color_by_depth(pts_raw)
            h = self.server.scene.add_point_cloud(
                '/stage4/points', points=pts_raw.astype(np.float32),
                colors=colors4, point_size=0.002)
            handles4.append(h)
        self._scene_handles['stage4'] = handles4

        # Stage 5: Filtered point cloud + floor plane
        pts_filt = p['points_filtered']
        handles5 = []
        if len(pts_filt) > 0:
            colors5 = color_by_depth(pts_filt)
            h = self.server.scene.add_point_cloud(
                '/stage5/points', points=pts_filt.astype(np.float32),
                colors=colors5, point_size=0.002)
            handles5.append(h)

            # Floor plane oriented by RANSAC normal
            plane_info = p.get('plane_info')
            if plane_info is not None:
                plane_center = plane_info['point']
                plane_normal = plane_info['normal']

                # Build a rotation that aligns +Y with the plane normal
                # (trimesh box has thin dimension along Y when created as [w, thin, h])
                up = np.array([0.0, 1.0, 0.0])
                if abs(np.dot(up, plane_normal)) < 0.999:
                    axis = np.cross(up, plane_normal)
                    axis /= np.linalg.norm(axis)
                    angle = np.arccos(np.clip(np.dot(up, plane_normal), -1, 1))
                    rot = Rotation.from_rotvec(axis * angle).as_matrix()
                else:
                    rot = np.eye(3) if np.dot(up, plane_normal) > 0 else \
                        Rotation.from_euler('x', 180, degrees=True).as_matrix()

                plane_mesh = trimesh.creation.box([0.20, 0.001, 0.20])
                xform = np.eye(4)
                xform[:3, :3] = rot
                xform[:3, 3] = plane_center
                plane_mesh.apply_transform(xform)
                plane_mesh.visual.face_colors = [255, 165, 0, 60]
                h = self.server.scene.add_mesh_trimesh('/stage5/floor', plane_mesh)
                handles5.append(h)
        self._scene_handles['stage5'] = handles5

        # Stage 6: All grasps + point cloud
        plane_normal = p['plane_normal']
        handles6 = []
        if len(pts_filt) > 0:
            h = self.server.scene.add_point_cloud(
                '/stage6/points', points=pts_filt.astype(np.float32),
                colors=colors5, point_size=0.002)
            handles6.append(h)

        for i, g in enumerate(p['all_grasps'][:36]):  # show up to 36
            c = score_to_color(g.score)
            mesh = create_gripper_mesh(g.center, g.yaw, g.width,
                                       plane_normal, color=c)
            h = self.server.scene.add_mesh_trimesh(f'/stage6/gripper_{i:02d}', mesh)
            handles6.append(h)
        self._scene_handles['stage6'] = handles6

        # Stage 7: Best grasp + point cloud
        handles7 = []
        if len(pts_filt) > 0:
            h = self.server.scene.add_point_cloud(
                '/stage7/points', points=pts_filt.astype(np.float32),
                colors=colors5, point_size=0.002)
            handles7.append(h)

        if p['best_grasp'] is not None:
            g = p['best_grasp']
            mesh = create_gripper_mesh(g.center, g.yaw, g.width,
                                       plane_normal,
                                       color=(0, 220, 0), alpha=230)
            h = self.server.scene.add_mesh_trimesh('/stage7/gripper_best', mesh)
            handles7.append(h)
        self._scene_handles['stage7'] = handles7

        # Initially show only stage 1
        self._show_stage(1)

    def _build_gui(self):
        self.stage_slider = self.server.gui.add_slider(
            'Stage', min=1, max=7, step=1, initial_value=1)
        self.stage_label = self.server.gui.add_text(
            'Stage Info', initial_value='1/7: Depth Image (normalized)')

        # Info folder
        with self.server.gui.add_folder('Info'):
            pts_raw = self.pipeline['points_raw']
            pts_filt = self.pipeline['points_filtered']
            n_grasps = len(self.pipeline['all_grasps'])
            best = self.pipeline['best_grasp']

            self.server.gui.add_text(
                'Points (raw)', initial_value=str(len(pts_raw)))
            self.server.gui.add_text(
                'Points (filtered)', initial_value=str(len(pts_filt)))
            self.server.gui.add_text(
                'Grasp candidates', initial_value=str(n_grasps))
            if best:
                self.server.gui.add_text(
                    'Best grasp',
                    initial_value=f'score={best.score:.2f} w={best.width*1000:.0f}mm')

        @self.stage_slider.on_update
        def _on_stage_change(event: viser.GuiEvent) -> None:
            stage = int(self.stage_slider.value)
            self._show_stage(stage)
            self.stage_label.value = f'{stage}/7: {STAGE_LABELS[stage]}'

    def _show_stage(self, stage: int):
        """Show only the scene elements for the given stage."""
        for s in range(1, 8):
            key = f'stage{s}'
            visible = (s == stage)
            for handle in self._scene_handles.get(key, []):
                handle.visible = visible

    def _setup_camera(self):
        @self.server.on_client_connect
        def on_connect(client: viser.ClientHandle) -> None:
            client.camera.position = (0.0, 0.0, -0.05)
            client.camera.look_at = (0.0, 0.0, self.plane_z)
            client.camera.up_direction = (0.0, -1.0, 0.0)

    def run(self):
        """Block forever, serving the visualizer."""
        try:
            while True:
                import time
                time.sleep(1.0)
        except KeyboardInterrupt:
            print('\nShutting down.')


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Grasp pipeline step-by-step visualizer')
    parser.add_argument(
        '--captures-dir', type=Path,
        default=Path(__file__).resolve().parents[2]
        / 'tidybot_perception/test/captures_real',
        help='Root captures directory')
    parser.add_argument(
        '--pair', type=str, default='pair_0008',
        help='Pair directory name (e.g., pair_0008)')
    parser.add_argument(
        '--target', type=str, default=None,
        help='YOLO target class filter (e.g., banana)')
    args = parser.parse_args()

    pair_dir = args.captures_dir / args.pair
    if not pair_dir.is_dir():
        print(f'Error: {pair_dir} not found')
        sys.exit(1)

    # Load data
    print(f'Loading data from {pair_dir}')
    depth_mm = np.load(str(pair_dir / 'depth.npy'))
    rgb_bgr = cv2.imread(str(pair_dir / 'rgb.png'))
    meta = loadmat(str(pair_dir / 'meta.mat'))
    K = meta['intrinsic_matrix']
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    print(f'Depth: {depth_mm.shape}, RGB: {rgb_bgr.shape}')
    print(f'Intrinsics: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}')

    # Run YOLO
    detection = run_yolo(rgb_bgr, target=args.target)
    if detection is None:
        print('No YOLO detection found. Cannot proceed.')
        sys.exit(1)

    # Compute pipeline
    print('\nComputing pipeline stages...')
    pipeline = compute_pipeline(depth_mm, rgb_bgr, fx, fy, cx, cy, detection)

    # Launch visualizer
    print('\nLaunching Viser visualizer...')
    viz = PipelineVisualizer(depth_mm, rgb_bgr, fx, fy, cx, cy, pipeline)
    viz.run()


if __name__ == '__main__':
    main()
