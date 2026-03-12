"""Shared constants and pure functions for TidyBot2 task scripts."""

import numpy as np


# ── YOLO class name -> ID mapping (COCO dataset, 80 classes) ─────────────────

YOLO_CLASS_MAP = {
    'person': 0, 'bicycle': 1, 'car': 2, 'motorcycle': 3, 'airplane': 4,
    'bus': 5, 'train': 6, 'truck': 7, 'boat': 8, 'traffic light': 9,
    'fire hydrant': 10, 'stop sign': 11, 'parking meter': 12, 'bench': 13,
    'bird': 14, 'cat': 15, 'dog': 16, 'horse': 17, 'sheep': 18, 'cow': 19,
    'elephant': 20, 'bear': 21, 'zebra': 22, 'giraffe': 23,
    'backpack': 24, 'umbrella': 25, 'handbag': 26, 'tie': 27, 'suitcase': 28,
    'frisbee': 29, 'skis': 30, 'snowboard': 31, 'sports ball': 32,
    'kite': 33, 'baseball bat': 34, 'baseball glove': 35, 'skateboard': 36,
    'surfboard': 37, 'tennis racket': 38,
    'bottle': 39, 'wine glass': 40, 'cup': 41, 'fork': 42, 'knife': 43,
    'spoon': 44, 'bowl': 45, 'banana': 46, 'apple': 47, 'sandwich': 48,
    'orange': 49, 'broccoli': 50, 'carrot': 51, 'hot dog': 52, 'pizza': 53,
    'donut': 54, 'cake': 55,
    'chair': 56, 'couch': 57, 'potted plant': 58, 'bed': 59,
    'dining table': 60, 'toilet': 61, 'tv': 62, 'laptop': 63, 'mouse': 64,
    'remote': 65, 'keyboard': 66, 'cell phone': 67, 'microwave': 68,
    'oven': 69, 'toaster': 70, 'sink': 71, 'refrigerator': 72, 'book': 73,
    'clock': 74, 'vase': 75, 'scissors': 76, 'teddy bear': 77,
    'hair drier': 78, 'toothbrush': 79,
}

# ── Object name -> AprilTag ID mapping (for bin placement) ───────────────────

TAG_MAP = {"banana": "0", "apple": "1", "orange": "2"}

# ── RANSAC / depth constants ─────────────────────────────────────────────────

FLOOR_MARGIN = 0.008   # RANSAC inlier distance threshold for floor plane
GRIPPER_OPEN_POS = 0.10  # gripper "open" command value

# ── Saved joint poses [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate] ──

GRASP_VALIDATION_POS = [0.0, -1.0, 0.6, 0.0, 0.4, 0.0]
RETRACT_HOLDING_POS  = [0.0, -1.35, 0.6, 0.0, 0.75, 0.0]
PRESENT_POSE         = [0.0, -0.5, 0.5, 0.0, -0.5, 0.0]


# ── Pure functions ───────────────────────────────────────────────────────────

def normalize_angle(angle):
    """Wrap angle to [-pi, pi]."""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def ransac_floor_separate(pts, distance_thresh=0.008, max_iterations=200,
                          min_inlier_ratio=0.20, min_pts=20):
    """RANSAC plane fit to find the dominant planar surface (floor/table).

    Returns (floor_z, object_points).  floor_z is the median Z of inliers.
    """
    n = len(pts)
    if n < min_pts:
        return float(np.median(pts[:, 2])), pts

    best_inlier_mask = None
    best_count = 0

    rng = np.random.default_rng(42)
    for _ in range(max_iterations):
        idx = rng.choice(n, 3, replace=False)
        p0, p1, p2 = pts[idx]

        v1 = p1 - p0
        v2 = p2 - p0
        normal = np.cross(v1, v2)
        norm_len = np.linalg.norm(normal)
        if norm_len < 1e-10:
            continue
        normal /= norm_len

        dists = np.abs((pts - p0) @ normal)
        inlier_mask = dists < distance_thresh
        count = inlier_mask.sum()

        if count > best_count:
            best_count = count
            best_inlier_mask = inlier_mask

    if best_inlier_mask is None or best_count < min_inlier_ratio * n:
        return float(np.min(pts[:, 2])), pts

    floor_z = float(np.median(pts[best_inlier_mask, 2]))
    outlier_mask = ~best_inlier_mask

    if outlier_mask.sum() < min_pts:
        return floor_z, pts

    return floor_z, pts[outlier_mask]
