#!/usr/bin/env python3
"""
Navigate to Object Module

Searches for a target object (e.g., 'banana', 'apple') by scanning the environment,
explores with frontier-based navigation and obstacle avoidance using a persistent
log-odds occupancy grid, and positions the robot within grasping range.

Uses YOLO detections from the external classifier node (object_classification/classifier.py)
via the /objbbox topic, rather than running its own YOLO model internally.

Mapping is based on the robust log-odds occupancy grid from frontier_exploration.py
(alexander branch), with angular-diversity obstacle confirmation, height-band filtering,
robot footprint clearing, and information-gain frontier scoring.

Internal sub-states: INIT → SCAN → EXPLORE → APPROACH → ALIGN → POSITIONED

Subscribes:
    /objbbox (vision_msgs/Detection2DArray) - YOLO detections from classifier node
    /camera/depth/image_raw (Image) - depth for obstacle avoidance + 3D localization
    /camera/color/camera_info (CameraInfo) - intrinsics

Publishes:
    /cmd_vel (Twist) - base velocity
    /camera/pan_tilt_cmd (Float64MultiArray) - camera pointing
    /map (OccupancyGrid) - continuous grayscale occupancy grid
    /map_image (Image) - RGB debug view of the map
    /frontier_markers (MarkerArray) - frontier cluster visualization

Requires:
    The classifier node must be running: ros2 run object_classification classifier

Usage:
    # As a standalone test:
    ros2 run tidybot_bringup navigate_to_object.py --ros-args -p target_object:=banana

    # Or import in the state machine:
    from navigate_to_object import NavigateToObject
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import numpy as np
import time
from enum import Enum, auto
from collections import deque

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, ColorRGBA, String
from nav_msgs.msg import OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy

from tidybot_perception.coord_converter import CoordConverter


# ═══════════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════════

# Scanning
SCAN_ROTATION_SPEED = 0.5      # rad/s while rotating between scan stops  (bridge limit: 1.5)
SCAN_NUM_STOPS = 16             # number of stops per 360° scan (6 = every 60°)
SCAN_PAUSE_SEC = 10.0           # seconds to pause at each stop for YOLO detection
CAMERA_TILT_NAV = 0.3          # slight downward tilt for navigation

# Exploration
EXPLORE_SPEED = 0.35           # m/s while exploring  (bridge limit: 0.5)
EXPLORE_SCAN_INTERVAL = 8.0    # seconds between scans during exploration
EXPLORE_SCAN_DISTANCE = 3.0    # meters between scans during exploration
MAX_EXPLORE_SCANS = 15         # give up after this many full scans with no detection

# Approach
APPROACH_SPEED = 0.30          # m/s while approaching
GRASP_DISTANCE = 0.40          # meters — stop this far from object
OBJECT_LOST_TIMEOUT = 5.0      # seconds before giving up on lost object

# Align
ALIGN_PIXEL_TOLERANCE = 25     # pixels from image center
ALIGN_DISTANCE_TOLERANCE = 0.05  # meters from GRASP_DISTANCE

# Obstacle avoidance
OBSTACLE_THRESHOLD = 0.5       # meters — trigger avoidance
EMERGENCY_STOP_DISTANCE = 0.25 # meters — hard stop

# ═══════════════════════════════════════════════════════════════════════
# Occupancy Grid  (15m × 15m world)
# ═══════════════════════════════════════════════════════════════════════
GRID_RESOLUTION = 0.05         # meters per cell
GRID_SIZE = 300                # cells per side (300 * 0.05 = 15m)
GRID_ORIGIN_X = -7.5           # bottom-left corner in odom (meters)
GRID_ORIGIN_Y = -7.5

# ── Depth filtering ──────────────────────────────────────────────────
MIN_DEPTH_M = 0.30             # ignore returns closer than this (robot body)
MAX_DEPTH_M = 5.00             # ignore returns farther than this
DEPTH_SUBSAMPLE = 4            # process every Nth pixel for speed
ROBOT_BODY_RADIUS = 0.60       # m — XY radius around camera to exclude self-hits
FREE_TRACE_MAX_M = 3.00        # m — max free-ray length for floor/ceiling returns

# ── Height-band filtering ────────────────────────────────────────────
MIN_OBSTACLE_Z = 0.10          # m — clears floor returns (z ≈ 0)
MAX_OBSTACLE_Z = 0.80          # m — ignores camera mount / ceilings

# ── Log-odds belief (Thrun, Burgard & Fox, Ch. 9) ────────────────────
LOG_ODDS_HIT = 0.85            # evidence of occupancy per observation
LOG_ODDS_MISS = -0.40          # evidence of free space per ray cell
LOG_ODDS_MIN = -10.0           # clamp floor
LOG_ODDS_MAX = 10.0            # clamp ceiling
LOG_ODDS_FREE_THRESH = -0.50   # frontier detection: below this → FREE cell
LOG_ODDS_OCC_THRESH = 1.50     # raytrace early-exit threshold (≈ 2 net HITs)

# ── Angular-diversity obstacle confirmation ──────────────────────────
MIN_HIT_ANGLE_SEP = np.radians(20)  # min gap between accepted hits
MIN_HIT_COUNT = 2              # distinct-angle hits to confirm obstacle

# ── Frontier selection ───────────────────────────────────────────────
MIN_FRONTIER_SIZE = 8          # minimum cells in a frontier cluster
FRONTIER_NAV_OFFSET = 0.4      # navigate slightly in front of frontier (meters)

# YOLO COCO class name → string ID (must match classifier.py output)
YOLO_CLASS_IDS = {'person': '0', 'banana': '46', 'apple': '47', 'orange': '49'}

# Depth back-projection
DEPTH_PATCH_RADIUS = 5         # half-size of patch for robust depth estimation (11x11)
FLOOR_Z_MIN = 0.005            # metres — clamp z above floor


class NavState(Enum):
    """Internal navigation sub-states."""
    INIT       = auto()
    SCAN       = auto()
    EXPLORE    = auto()
    APPROACH   = auto()
    ALIGN      = auto()
    POSITIONED = auto()


# ═══════════════════════════════════════════════════════════════════════
# Log-Odds Occupancy Grid
# ═══════════════════════════════════════════════════════════════════════

class LogOddsOccupancyGrid:
    """Persistent 2D log-odds occupancy grid with angular-diversity confirmation.

    Key features (ported from frontier_exploration.py):
    - Bayesian log-odds belief accumulation (never cleared between scans)
    - Height-band filtering: only Z ∈ [MIN_OBSTACLE_Z, MAX_OBSTACLE_Z] → obstacle HITs
    - Angular-diversity confirmation: cells need hits from 2+ distinct camera angles
    - Robot footprint clearing: prevents false frontiers under the robot
    - Early-exit raytrace: confirmed obstacles stop free-space rays
    - Information-gain frontier scoring: unknown_cells_within_range / distance
    """

    def __init__(self):
        self.log_odds = np.zeros(
            (GRID_SIZE, GRID_SIZE), dtype=np.float32)
        self.hit_count = np.zeros(
            (GRID_SIZE, GRID_SIZE), dtype=np.uint8)
        self.hit_angle = np.full(
            (GRID_SIZE, GRID_SIZE), np.nan, dtype=np.float32)

    def world_to_grid(self, wx, wy):
        gx = int((wx - GRID_ORIGIN_X) / GRID_RESOLUTION)
        gy = int((wy - GRID_ORIGIN_Y) / GRID_RESOLUTION)
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = GRID_ORIGIN_X + (gx + 0.5) * GRID_RESOLUTION
        wy = GRID_ORIGIN_Y + (gy + 0.5) * GRID_RESOLUTION
        return wx, wy

    def in_bounds(self, gx, gy):
        return 0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE

    def update_from_depth(self, depth_image, camera_K, R_cam_to_odom,
                          t_cam_to_odom, cam_heading):
        """Project depth image into odom frame and update log-odds grid.

        Args:
            depth_image: 16UC1 depth in mm (480x640)
            camera_K: 3x3 intrinsic matrix
            R_cam_to_odom: 3x3 rotation from camera optical frame to odom
            t_cam_to_odom: (3,) translation from camera optical frame to odom
            cam_heading: Robot heading angle (rad) for angular diversity tracking
        """
        h, w = depth_image.shape
        step = DEPTH_SUBSAMPLE

        fx, fy = camera_K[0, 0], camera_K[1, 1]
        cx, cy = camera_K[0, 2], camera_K[1, 2]

        cam_origin = t_cam_to_odom
        cam_gx, cam_gy = self.world_to_grid(cam_origin[0], cam_origin[1])

        # ── 1. Vectorized depth projection (subsampled) ─────────────────
        vs, us = np.mgrid[0:h:step, 0:w:step]
        us = us.ravel()
        vs = vs.ravel()
        depths_mm = depth_image[vs, us].astype(np.float64)

        min_mm = MIN_DEPTH_M * 1000.0
        max_mm = MAX_DEPTH_M * 1000.0
        valid = (depths_mm > min_mm) & (depths_mm < max_mm)
        us = us[valid]
        vs = vs[valid]
        depths_m = depths_mm[valid] / 1000.0

        if len(depths_m) == 0:
            return

        # Project to camera optical frame and transform to odom
        x_cam = (us - cx) * depths_m / fx
        y_cam = (vs - cy) * depths_m / fy
        z_cam = depths_m
        pts_odom = (R_cam_to_odom @ np.stack(
            [x_cam, y_cam, z_cam], axis=1).T).T + cam_origin

        # ── 2. Grid coordinates ──────────────────────────────────────────
        gxs = ((pts_odom[:, 0] - GRID_ORIGIN_X) / GRID_RESOLUTION).astype(int)
        gys = ((pts_odom[:, 1] - GRID_ORIGIN_Y) / GRID_RESOLUTION).astype(int)

        # ── 3. Build masks ───────────────────────────────────────────────
        # Self-exclusion: ignore returns near camera origin (robot body)
        dist_xy = np.hypot(pts_odom[:, 0] - cam_origin[0],
                           pts_odom[:, 1] - cam_origin[1])
        not_self = dist_xy > ROBOT_BODY_RADIUS

        in_bounds = ((gxs >= 0) & (gxs < GRID_SIZE) &
                     (gys >= 0) & (gys < GRID_SIZE))
        in_height = ((pts_odom[:, 2] > MIN_OBSTACLE_Z) &
                     (pts_odom[:, 2] < MAX_OBSTACLE_Z))

        # Obstacle returns → free raytrace + HIT
        obs_mask = not_self & in_bounds & in_height

        # Floor/ceiling returns → free raytrace only (no HIT), limited depth
        floor_mask = (not_self & in_bounds & ~in_height &
                      (depths_m < FREE_TRACE_MAX_M))

        obs_idx = np.where(obs_mask)[0]
        floor_idx = np.where(floor_mask)[0]

        # ── 4. Robot footprint clearing ──────────────────────────────────
        r_cells = int(ROBOT_BODY_RADIUS / GRID_RESOLUTION) + 1
        fy_min = max(0, cam_gy - r_cells)
        fy_max = min(GRID_SIZE, cam_gy + r_cells + 1)
        fx_min = max(0, cam_gx - r_cells)
        fx_max = min(GRID_SIZE, cam_gx + r_cells + 1)
        ffy, ffx = np.meshgrid(
            np.arange(fy_min, fy_max), np.arange(fx_min, fx_max), indexing='ij')
        in_fp = ((ffy - cam_gy)**2 + (ffx - cam_gx)**2) <= r_cells**2
        fp_gy = ffy[in_fp]
        fp_gx = ffx[in_fp]
        # Don't clear cells already confirmed as real obstacles
        clearable = self.hit_count[fp_gy, fp_gx] < MIN_HIT_COUNT
        self.log_odds[fp_gy[clearable], fp_gx[clearable]] = np.maximum(
            LOG_ODDS_MIN,
            self.log_odds[fp_gy[clearable], fp_gx[clearable]] + LOG_ODDS_MISS)

        if len(obs_idx) == 0 and len(floor_idx) == 0:
            return

        # ── 5a. Free-space raytrace: obstacle returns ────────────────────
        if len(obs_idx) > 0:
            ray_step = max(1, len(obs_idx) // 500)
            for i in obs_idx[::ray_step]:
                self._raytrace_free(cam_gx, cam_gy, int(gxs[i]), int(gys[i]))

        # ── 5b. Free-space raytrace: floor/ceiling returns ───────────────
        if len(floor_idx) > 0:
            ray_step2 = max(1, len(floor_idx) // 300)
            for i in floor_idx[::ray_step2]:
                self._raytrace_free(cam_gx, cam_gy, int(gxs[i]), int(gys[i]))

        # ── 6. Mark obstacle cells with HIT ──────────────────────────────
        if len(obs_idx) > 0:
            np.add.at(self.log_odds, (gys[obs_mask], gxs[obs_mask]),
                      LOG_ODDS_HIT)
            np.clip(self.log_odds, LOG_ODDS_MIN, LOG_ODDS_MAX,
                    out=self.log_odds)

        # ── 7. Update angular-diversity hit count ────────────────────────
        if len(obs_idx) > 0 and cam_heading is not None:
            cam_theta = float(cam_heading)

            # Deduplicate: many pixels may project to the same cell
            u_yx = np.unique(
                np.stack([gys[obs_mask], gxs[obs_mask]], axis=1), axis=0)
            u_gy, u_gx = u_yx[:, 0], u_yx[:, 1]

            # First-hit cells
            no_prior = np.isnan(self.hit_angle[u_gy, u_gx])
            if np.any(no_prior):
                self.hit_angle[u_gy[no_prior], u_gx[no_prior]] = cam_theta
                self.hit_count[u_gy[no_prior], u_gx[no_prior]] = 1

            # Subsequent hits — only count if angle is sufficiently different
            has_prior = ~no_prior
            if np.any(has_prior):
                hp_gy = u_gy[has_prior]
                hp_gx = u_gx[has_prior]
                prior = self.hit_angle[hp_gy, hp_gx]
                diff = np.abs(cam_theta - prior)
                diff = np.minimum(diff, 2 * np.pi - diff)
                diverse = diff >= MIN_HIT_ANGLE_SEP
                if np.any(diverse):
                    d_gy = hp_gy[diverse]
                    d_gx = hp_gx[diverse]
                    self.hit_count[d_gy, d_gx] = np.minimum(
                        self.hit_count[d_gy, d_gx].astype(np.uint16) + 1,
                        255).astype(np.uint8)
                    self.hit_angle[d_gy, d_gx] = cam_theta

    def _raytrace_free(self, x0, y0, x1, y1):
        """Bresenham ray from (x0,y0) to (x1,y1) exclusive.

        Applies LOG_ODDS_MISS to each traversed cell. Stops early at cells
        confirmed from multiple angles (hit_count >= MIN_HIT_COUNT AND
        log_odds >= LOG_ODDS_OCC_THRESH).
        """
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0

        while not (x == x1 and y == y1):
            if self.in_bounds(x, y):
                # Stop at confirmed obstacles (multi-angle)
                if (self.hit_count[y, x] >= MIN_HIT_COUNT and
                        self.log_odds[y, x] >= LOG_ODDS_OCC_THRESH):
                    break
                self.log_odds[y, x] = max(
                    LOG_ODDS_MIN,
                    self.log_odds[y, x] + LOG_ODDS_MISS)
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def find_best_frontier(self, robot_x, robot_y, robot_heading):
        """Find the best frontier to explore using information-gain scoring.

        A frontier cell is FREE (log_odds <= FREE_THRESH) and 4-adjacent to
        UNKNOWN. Connected components (8-connected) smaller than
        MIN_FRONTIER_SIZE are discarded.

        Score = unknown_cells_within_sensor_range / distance, with heading bonus.

        Args:
            robot_x, robot_y: Robot position in odom frame.
            robot_heading: Robot heading (actual world heading, not MuJoCo theta).

        Returns:
            (goal_x, goal_y) in odom frame, or None if no frontiers.
        """
        free_mask = self.log_odds <= LOG_ODDS_FREE_THRESH
        confirmed_occ = ((self.log_odds >= LOG_ODDS_OCC_THRESH) &
                         (self.hit_count >= MIN_HIT_COUNT))
        unknown_mask = ~free_mask & ~confirmed_occ

        frontier_mask = np.zeros_like(self.log_odds, dtype=bool)
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            frontier_mask |= (free_mask &
                              np.roll(np.roll(unknown_mask, dy, axis=0),
                                      dx, axis=1))

        fys, fxs = np.where(frontier_mask)
        if len(fxs) == 0:
            return None

        # Cluster via BFS (8-connected)
        frontier_set = set(zip(fxs.tolist(), fys.tolist()))
        visited = set()
        clusters = []

        for seed in frontier_set:
            if seed in visited:
                continue
            cluster = []
            queue = deque([seed])
            visited.add(seed)
            while queue:
                cx_c, cy_c = queue.popleft()
                cluster.append((cx_c, cy_c))
                for ddx, ddy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                                 (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                    nb = (cx_c + ddx, cy_c + ddy)
                    if nb in frontier_set and nb not in visited:
                        visited.add(nb)
                        queue.append(nb)

            if len(cluster) >= MIN_FRONTIER_SIZE:
                clusters.append(cluster)

        if not clusters:
            return None

        # Pre-build disc mask for counting unknown cells within sensor range
        r_cells = int(MAX_DEPTH_M / GRID_RESOLUTION)

        # Score clusters: information gain / distance, with heading bonus
        best_score = -1
        best_goal = None

        for cluster in clusters:
            mgx = int(np.mean([c[0] for c in cluster]))
            mgy = int(np.mean([c[1] for c in cluster]))
            wx, wy = self.grid_to_world(mgx, mgy)

            dx = wx - robot_x
            dy = wy - robot_y
            dist = np.sqrt(dx**2 + dy**2)
            if dist < 0.3:
                continue  # too close

            # Count UNKNOWN cells within sensor range of frontier centroid
            y_lo = max(0, mgy - r_cells)
            y_hi = min(GRID_SIZE, mgy + r_cells + 1)
            x_lo = max(0, mgx - r_cells)
            x_hi = min(GRID_SIZE, mgx + r_cells + 1)

            # Disc kernel slice
            ky_lo = y_lo - (mgy - r_cells)
            ky_hi = ky_lo + (y_hi - y_lo)
            kx_lo = x_lo - (mgx - r_cells)
            kx_hi = kx_lo + (x_hi - x_lo)

            ky, kx = np.ogrid[-r_cells:r_cells + 1, -r_cells:r_cells + 1]
            disc = (ky**2 + kx**2) <= r_cells**2

            patch = unknown_mask[y_lo:y_hi, x_lo:x_hi]
            nearby_unknown = int(np.sum(
                patch & disc[ky_lo:ky_hi, kx_lo:kx_hi]))

            # Heading bonus: prefer frontiers in the direction we're facing
            angle_to_frontier = np.arctan2(dy, dx)
            angle_diff = abs(_normalize_angle(angle_to_frontier - robot_heading))
            heading_bonus = 1.0 + 0.5 * (1.0 - angle_diff / np.pi)

            score = (nearby_unknown * heading_bonus) / max(dist, 0.5)
            if score > best_score:
                best_score = score
                # Navigate to a point offset back toward the robot
                if dist > FRONTIER_NAV_OFFSET * 2:
                    ratio = (dist - FRONTIER_NAV_OFFSET) / dist
                    best_goal = (robot_x + dx * ratio, robot_y + dy * ratio)
                else:
                    best_goal = (wx, wy)

        return best_goal

    def find_frontiers_for_viz(self, robot_x, robot_y):
        """Return scored frontier clusters for visualization.

        Returns list of (wx, wy, cluster_size, score) sorted by score descending.
        """
        free_mask = self.log_odds <= LOG_ODDS_FREE_THRESH
        confirmed_occ = ((self.log_odds >= LOG_ODDS_OCC_THRESH) &
                         (self.hit_count >= MIN_HIT_COUNT))
        unknown_mask = ~free_mask & ~confirmed_occ

        frontier_mask = np.zeros_like(self.log_odds, dtype=bool)
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            frontier_mask |= (free_mask &
                              np.roll(np.roll(unknown_mask, dy, axis=0),
                                      dx, axis=1))

        fys, fxs = np.where(frontier_mask)
        if len(fxs) == 0:
            return []

        frontier_set = set(zip(fxs.tolist(), fys.tolist()))
        visited = set()
        clusters = []

        for seed in frontier_set:
            if seed in visited:
                continue
            cluster = []
            queue = deque([seed])
            visited.add(seed)
            while queue:
                cx_c, cy_c = queue.popleft()
                cluster.append((cx_c, cy_c))
                for ddx, ddy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                                 (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                    nb = (cx_c + ddx, cy_c + ddy)
                    if nb in frontier_set and nb not in visited:
                        visited.add(nb)
                        queue.append(nb)
            if len(cluster) >= MIN_FRONTIER_SIZE:
                clusters.append(cluster)

        if not clusters:
            return []

        r_cells = int(MAX_DEPTH_M / GRID_RESOLUTION)

        results = []
        for cluster in clusters:
            mgx = int(np.mean([c[0] for c in cluster]))
            mgy = int(np.mean([c[1] for c in cluster]))
            wx, wy = self.grid_to_world(mgx, mgy)
            dist = np.hypot(wx - robot_x, wy - robot_y)

            y_lo = max(0, mgy - r_cells)
            y_hi = min(GRID_SIZE, mgy + r_cells + 1)
            x_lo = max(0, mgx - r_cells)
            x_hi = min(GRID_SIZE, mgx + r_cells + 1)

            ky_lo = y_lo - (mgy - r_cells)
            ky_hi = ky_lo + (y_hi - y_lo)
            kx_lo = x_lo - (mgx - r_cells)
            kx_hi = kx_lo + (x_hi - x_lo)

            ky, kx = np.ogrid[-r_cells:r_cells + 1, -r_cells:r_cells + 1]
            disc = (ky**2 + kx**2) <= r_cells**2

            patch = unknown_mask[y_lo:y_hi, x_lo:x_hi]
            nearby_unknown = int(np.sum(
                patch & disc[ky_lo:ky_hi, kx_lo:kx_hi]))

            score = nearby_unknown / max(dist, 0.5)
            results.append((wx, wy, len(cluster), score))

        results.sort(key=lambda r: r[3], reverse=True)
        return results

    def get_map_data(self):
        """Return occupancy grid data as int8 array for ROS publishing.

        Continuous grayscale: log_odds mapped linearly to [0, 100].
        Unconfirmed cells capped at 80. Never-updated cells → -1 (unknown).
        """
        span = float(LOG_ODDS_MAX - LOG_ODDS_MIN)
        ratio = (self.log_odds - LOG_ODDS_MIN) / span
        occ = np.clip(ratio * 100.0, 0.0, 100.0)

        # Cap unconfirmed cells so they're visually distinct from confirmed
        occ[self.hit_count < MIN_HIT_COUNT] = np.minimum(
            occ[self.hit_count < MIN_HIT_COUNT], 80.0)

        grid = occ.astype(np.int8)

        # Never-updated cells published as -1 (unknown)
        grid[(self.hit_count == 0) & (self.log_odds == 0.0)] = -1

        return grid


# ═══════════════════════════════════════════════════════════════════════
# Helper functions
# ═══════════════════════════════════════════════════════════════════════

def _normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


# ═══════════════════════════════════════════════════════════════════════
# NavigateToObject Node
# ═══════════════════════════════════════════════════════════════════════

class NavigateToObject(Node):
    """
    Navigate to a target object using external YOLO classifier detections,
    log-odds frontier exploration, and reactive obstacle avoidance.

    Requires the classifier node to be running for /objbbox detections.
    Can be run standalone or used by the state machine.
    """

    def __init__(self, target_object: str = 'banana'):
        super().__init__('navigate_to_object')

        self.target_object = target_object

        # ── TF2 (for odom→base_link lookups) ──
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.coord_conv = CoordConverter(self.tf_buffer)

        # ── Perception: subscribe to external classifier + depth ──
        self._bridge = CvBridge()
        self.latest_depth = None
        self.latest_depth_stamp = None
        self.camera_info = None
        self.latest_detections = None

        qos_be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Detection2DArray, '/objbbox', self._detections_cb, 10)
        self.create_subscription(
            Image, '/camera/depth/image_raw', self._depth_cb, qos_be)
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._info_cb, 10)

        # ── Publishers ──
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.map_image_pub = self.create_publisher(Image, '/map_image', 10)
        self.frontier_pub = self.create_publisher(
            MarkerArray, '/frontier_markers', 10)
        self.yolo_mode_pub = self.create_publisher(String, '/yolo_mode', 10)

        # ── Occupancy grid ──
        self.occ_grid = LogOddsOccupancyGrid()

        # ── State machine ──
        self.nav_state = NavState.INIT
        self.state_start_time = time.time()

        # Scan tracking
        self.prev_yaw = None
        self.total_rotated = 0.0
        self.scan_count = 0

        # Explore tracking
        self.explore_start_time = 0.0
        self.explore_start_pos = None
        self.explore_goal = None

        # Approach/align tracking
        self.target_base_pt = None       # PointStamped in base_link
        self.target_pixel = None         # (u, v) pixel centroid for visual servoing
        self.last_detection_time = 0.0

        # Map publishing
        self.last_map_publish_time = 0.0
        MAP_PUBLISH_RATE = 2.0  # Hz
        self.map_publish_interval = 1.0 / MAP_PUBLISH_RATE

        # ── Control loop at 10Hz ──
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f'NavigateToObject: searching for "{target_object}"')

    # ═══════════════════════════════════════════════════════════════════
    # Public interface
    # ═══════════════════════════════════════════════════════════════════

    def is_positioned(self) -> bool:
        """Returns True when the robot is positioned within grasping range."""
        return self.nav_state == NavState.POSITIONED

    def get_object_position(self):
        """Returns the latest PointStamped of the target in base_link, or None."""
        return self.target_base_pt

    def set_target(self, target_class: str):
        """Set a new target object class and reset to INIT state."""
        self.target_object = target_class
        self.reset()
        self.get_logger().info(f'New target object: "{target_class}"')

    def reset(self):
        """Reset the full state machine."""
        self.nav_state = NavState.INIT
        self.state_start_time = time.time()
        self.occ_grid = LogOddsOccupancyGrid()
        self.prev_yaw = None
        self.total_rotated = 0.0
        self.scan_count = 0
        self.explore_goal = None
        self.target_base_pt = None
        self.target_pixel = None
        self._stop_base()

    # ═══════════════════════════════════════════════════════════════════
    # Utility methods
    # ═══════════════════════════════════════════════════════════════════

    def _get_base_pose_in_odom(self):
        """Get (x, y, theta) from odom→base_link TF.

        theta is in MuJoCo convention. Actual heading = theta - pi/2.
        Returns None if TF lookup fails.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

        t = transform.transform.translation
        q = transform.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = np.arctan2(siny, cosy)
        return (t.x, t.y, theta)

    def _get_current_yaw(self):
        """Get current yaw (MuJoCo theta) from TF."""
        pose = self._get_base_pose_in_odom()
        if pose is None:
            return None
        return pose[2]

    def _get_camera_transform(self, stamp=None):
        """Get rotation matrix and translation for camera→odom transform.

        Uses stamped TF lookup matching the depth frame timestamp for
        synchronized transforms (eliminates TF timing lag artifacts).
        Falls back to latest TF if stamped lookup fails.

        Args:
            stamp: builtin_interfaces/Time from depth image header, or None.

        Returns (R, t) or (None, None) on failure.
        """
        # Try stamped lookup first (synchronized with depth frame)
        if stamp is not None:
            try:
                tf_stamp = rclpy.time.Time.from_msg(stamp)
                transform = self.tf_buffer.lookup_transform(
                    'odom', 'camera_depth_optical_frame',
                    tf_stamp, timeout=Duration(seconds=0.1))
                return self._extract_transform(transform)
            except Exception:
                pass  # fall through to latest

        # Fallback to latest TF
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'camera_depth_optical_frame',
                rclpy.time.Time(), timeout=Duration(seconds=0.5))
            return self._extract_transform(transform)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None, None

    def _extract_transform(self, transform):
        """Extract R, t from a TransformStamped."""
        t = transform.transform.translation
        q = transform.transform.rotation
        cam_origin = np.array([t.x, t.y, t.z])

        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
        ])

        return R, cam_origin

    def _stop_base(self):
        """Publish zero velocity."""
        self.cmd_vel_pub.publish(Twist())

    def _send_pan_tilt(self, pan, tilt):
        """Send camera pan/tilt command."""
        msg = Float64MultiArray()
        msg.data = [float(pan), float(tilt)]
        self.pan_tilt_pub.publish(msg)

    # ── Classifier subscription callbacks ──

    def _detections_cb(self, msg):
        self.latest_detections = msg

    def _depth_cb(self, msg):
        self.latest_depth = self._bridge.imgmsg_to_cv2(msg, '16UC1')
        self.latest_depth_stamp = msg.header.stamp

    def _info_cb(self, msg):
        self.camera_info = msg

    # ── Detection helpers (consume external classifier) ──

    def _detect(self, target_class):
        """Find the best detection matching target_class from /objbbox.

        Args:
            target_class: COCO class name (e.g. 'banana'). Case-insensitive.

        Returns:
            (u, v) pixel centroid of highest-confidence match, or None.
        """
        if self.latest_detections is None:
            return None
        target_id = YOLO_CLASS_IDS.get(target_class.lower())
        if target_id is None:
            return None

        best, best_conf = None, 0.0
        for det in self.latest_detections.detections:
            for hyp in det.results:
                if hyp.hypothesis.class_id == target_id and hyp.hypothesis.score > best_conf:
                    best = (int(det.bbox.center.position.x),
                            int(det.bbox.center.position.y))
                    best_conf = hyp.hypothesis.score
        return best

    def _detect_and_localize(self, target_class):
        """Detect an object and return its 3D position in base_link.

        Args:
            target_class: COCO class name (e.g. 'banana').

        Returns:
            ((u, v), PointStamped_in_base_link), or None.
        """
        centroid = self._detect(target_class)
        if centroid is None:
            return None

        base_pt = self.coord_conv.pixel_to_base_link(
            centroid[0], centroid[1],
            self.latest_depth, self.camera_info, self.get_clock(),
            patch_radius=DEPTH_PATCH_RADIUS, floor_z_min=FLOOR_Z_MIN)
        if base_pt is None:
            return None

        return (centroid, base_pt)

    def _analyze_depth_obstacles(self):
        """Analyze depth image for obstacles in left/center/right sectors.

        Returns:
            (clearance, distances) where each is (left, center, right).
            clearance: tuple of bools (True = clear).
            distances: tuple of floats (meters to nearest obstacle).
        """
        depth = self.latest_depth
        if depth is None:
            return (True, True, True), (999.0, 999.0, 999.0)

        h, w = depth.shape

        # Vertical band: ignore top 30% (sky/ceiling) and bottom 20% (floor)
        v_start = int(h * 0.3)
        v_end = int(h * 0.8)

        left_region = depth[v_start:v_end, 0:w // 3]
        center_region = depth[v_start:v_end, w // 3:2 * w // 3]
        right_region = depth[v_start:v_end, 2 * w // 3:w]

        def sector_min_dist(region):
            valid = region[region > 0].astype(np.float64)
            if len(valid) == 0:
                return 999.0
            return np.percentile(valid, 10) / 1000.0  # mm → meters

        left_dist = sector_min_dist(left_region)
        center_dist = sector_min_dist(center_region)
        right_dist = sector_min_dist(right_region)

        clearance = (
            left_dist > OBSTACLE_THRESHOLD,
            center_dist > OBSTACLE_THRESHOLD,
            right_dist > OBSTACLE_THRESHOLD,
        )
        return clearance, (left_dist, center_dist, right_dist)

    def _drive_toward_goal_with_avoidance(self, goal_x, goal_y, speed=EXPLORE_SPEED):
        """Drive toward an odom-frame goal with reactive obstacle avoidance.

        Args:
            goal_x, goal_y: Target position in odom frame.
            speed: Maximum forward speed.
        """
        pose = self._get_base_pose_in_odom()
        if pose is None:
            return

        cx, cy, ctheta = pose
        actual_heading = ctheta - np.pi / 2

        dx = goal_x - cx
        dy = goal_y - cy
        distance = np.sqrt(dx**2 + dy**2)
        desired_heading = np.arctan2(dy, dx)
        heading_error = _normalize_angle(desired_heading - actual_heading)

        clearance, distances = self._analyze_depth_obstacles()
        left_clear, center_clear, right_clear = clearance
        left_dist, center_dist, right_dist = distances

        twist = Twist()

        if center_clear:
            if abs(heading_error) > 0.4:
                # Large heading error: rotate first
                twist.angular.z = np.clip(2.0 * heading_error, -1.0, 1.0)
            else:
                # Drive forward with heading correction
                twist.linear.x = min(speed, distance * 0.5)
                twist.angular.z = np.clip(1.5 * heading_error, -0.8, 0.8)
        else:
            # Center blocked — steer toward clearer side
            if left_dist > right_dist:
                twist.angular.z = 0.5
                twist.linear.x = 0.05
            elif right_dist > left_dist:
                twist.angular.z = -0.5
                twist.linear.x = 0.05
            else:
                # All blocked — rotate in place
                twist.angular.z = 0.6

        # Emergency stop if very close to anything
        if min(left_dist, center_dist, right_dist) < EMERGENCY_STOP_DISTANCE:
            twist.linear.x = 0.0

        self.cmd_vel_pub.publish(twist)

    def _update_occupancy_grid(self):
        """Update the log-odds occupancy grid from the latest depth image."""
        depth = self.latest_depth
        if depth is None or self.camera_info is None:
            return

        R, t = self._get_camera_transform(stamp=self.latest_depth_stamp)
        if R is None:
            return

        # Get robot heading for angular diversity tracking
        pose = self._get_base_pose_in_odom()
        cam_heading = pose[2] if pose is not None else None

        camera_K = np.array(self.camera_info.k).reshape(3, 3)
        self.occ_grid.update_from_depth(depth, camera_K, R, t, cam_heading)

    def _publish_map(self):
        """Publish the occupancy grid and visualization to RViz."""
        now = time.time()
        if now - self.last_map_publish_time < self.map_publish_interval:
            return
        self.last_map_publish_time = now

        stamp = self.get_clock().now().to_msg()

        # ── OccupancyGrid message ──
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'

        msg.info = MapMetaData()
        msg.info.resolution = GRID_RESOLUTION
        msg.info.width = GRID_SIZE
        msg.info.height = GRID_SIZE
        msg.info.origin.position.x = float(GRID_ORIGIN_X)
        msg.info.origin.position.y = float(GRID_ORIGIN_Y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        grid = self.occ_grid.get_map_data()
        msg.data = grid.ravel().tolist()
        self.map_pub.publish(msg)

        # ── RGB debug image ──
        brightness = np.where(
            grid < 0,
            128,
            255 - (grid.astype(np.int16) * 255 // 100)
        ).astype(np.uint8)
        img = np.stack([brightness, brightness, brightness], axis=-1)
        img_msg = self._bridge.cv2_to_imgmsg(img, encoding='rgb8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = 'odom'
        self.map_image_pub.publish(img_msg)

        # ── Frontier markers ──
        pose = self._get_base_pose_in_odom()
        if pose is not None:
            rx, ry, _ = pose
            frontiers = self.occ_grid.find_frontiers_for_viz(rx, ry)
            self._publish_frontier_markers(frontiers, stamp)

    def _publish_frontier_markers(self, frontiers, stamp):
        """Publish frontier clusters as colored cylinder markers."""
        ma = MarkerArray()
        for i, (wx, wy, size, score) in enumerate(frontiers):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = 'odom'
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.3
            bright = 1.0 if i == 0 else 0.4
            m.color = ColorRGBA(r=0.0, g=bright, b=0.3, a=0.8)
            m.lifetime.sec = 5
            ma.markers.append(m)
        self.frontier_pub.publish(ma)

    def _transition_to(self, new_state):
        """Transition to a new state with logging."""
        old = self.nav_state.name
        self.nav_state = new_state
        self.state_start_time = time.time()
        self.get_logger().info(f'[NAV] {old} → {new_state.name}')

        # Switch YOLO model: heavy when scanning (stationary), light otherwise
        mode = 'heavy' if new_state == NavState.SCAN else 'light'
        self.yolo_mode_pub.publish(String(data=mode))

    # ═══════════════════════════════════════════════════════════════════
    # State handlers
    # ═══════════════════════════════════════════════════════════════════

    def _handle_init(self):
        """Wait for sensor data and TF to be available."""
        if (self.latest_detections is None
                or self.latest_depth is None
                or self.camera_info is None):
            return

        try:
            self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return

        # Set camera to forward with slight downward tilt
        self._send_pan_tilt(0.0, CAMERA_TILT_NAV)

        # Initialize scan tracking
        self.prev_yaw = self._get_current_yaw()
        self.total_rotated = 0.0
        self.scan_count = 0
        self.scan_stop_index = 0              # which stop we're heading to (0..NUM_STOPS-1)
        self.scan_paused = False               # True while paused at a stop
        self.scan_pause_start = 0.0            # when the current pause began
        self.scan_step_angle = 2.0 * np.pi / SCAN_NUM_STOPS  # radians per step

        self._transition_to(NavState.SCAN)

    def _handle_scan(self):
        """Rotate 360° in discrete steps, pausing at each stop to check YOLO."""
        # Update map while scanning (captures depth from all directions)
        self._update_occupancy_grid()

        # ── Paused at a stop: wait, then check YOLO ──────────────────
        if self.scan_paused:
            elapsed = time.time() - self.scan_pause_start

            # While paused, keep checking for detection every tick
            detection = self._detect_and_localize(self.target_object)
            if detection is not None:
                pixel, base_pt = detection
                self.target_base_pt = base_pt
                self.last_detection_time = time.time()
                self._stop_base()
                self.get_logger().info(
                    f'[SCAN] Detected + localized {self.target_object} at stop '
                    f'{self.scan_stop_index}/{SCAN_NUM_STOPS}, pixel {pixel}, '
                    f'base_link=({base_pt.point.x:.2f}, {base_pt.point.y:.2f}, '
                    f'{base_pt.point.z:.2f})')
                self._transition_to(NavState.APPROACH)
                return

            # 2D-only fallback: YOLO sees it but depth localization failed
            pixel_only = self._detect(self.target_object)
            if pixel_only is not None:
                self.target_pixel = pixel_only
                self.target_base_pt = None  # no 3D yet — approach will visual-servo
                self.last_detection_time = time.time()
                self._stop_base()
                self.get_logger().info(
                    f'[SCAN] Detected {self.target_object} (2D only) at pixel '
                    f'{pixel_only}, no depth — switching to APPROACH')
                self._transition_to(NavState.APPROACH)
                return

            if elapsed < SCAN_PAUSE_SEC:
                return  # still waiting

            # Pause done, no detection — move to next step
            self.scan_paused = False
            self.scan_stop_index += 1

            # Check if full rotation is complete
            if self.scan_stop_index >= SCAN_NUM_STOPS:
                self.scan_count += 1
                self.get_logger().info(
                    f'[SCAN] Full rotation #{self.scan_count}, '
                    f'no {self.target_object} found')

                if self.scan_count >= MAX_EXPLORE_SCANS:
                    self.get_logger().warn(
                        f'[SCAN] Gave up after {self.scan_count} scans')
                    self._stop_base()
                    return

                # Reset and transition to explore
                self.total_rotated = 0.0
                self.prev_yaw = self._get_current_yaw()
                self.explore_start_time = time.time()
                pose = self._get_base_pose_in_odom()
                self.explore_start_pos = (pose[0], pose[1]) if pose else None
                self.explore_goal = None
                self._transition_to(NavState.EXPLORE)
                return

            # Reset rotation tracking for the next step
            self.total_rotated = 0.0
            self.prev_yaw = self._get_current_yaw()
            return

        # ── Rotating toward the next stop ─────────────────────────────
        # Also check YOLO while rotating (cheap, might catch something)
        detection = self._detect_and_localize(self.target_object)
        if detection is not None:
            pixel, base_pt = detection
            self.target_base_pt = base_pt
            self.last_detection_time = time.time()
            self._stop_base()
            self.get_logger().info(
                f'[SCAN] Detected {self.target_object} while rotating, '
                f'pixel {pixel}, base_link=({base_pt.point.x:.2f}, '
                f'{base_pt.point.y:.2f}, {base_pt.point.z:.2f})')
            self._transition_to(NavState.APPROACH)
            return

        # 2D-only fallback while rotating
        pixel_only = self._detect(self.target_object)
        if pixel_only is not None:
            self.target_pixel = pixel_only
            self.target_base_pt = None
            self.last_detection_time = time.time()
            self._stop_base()
            self.get_logger().info(
                f'[SCAN] Detected {self.target_object} (2D only) while rotating, '
                f'pixel {pixel_only} — switching to APPROACH')
            self._transition_to(NavState.APPROACH)
            return

        twist = Twist()
        twist.angular.z = SCAN_ROTATION_SPEED
        self.cmd_vel_pub.publish(twist)

        # Track rotation via yaw
        current_yaw = self._get_current_yaw()
        if current_yaw is not None and self.prev_yaw is not None:
            delta = _normalize_angle(current_yaw - self.prev_yaw)
            self.total_rotated += abs(delta)
        self.prev_yaw = current_yaw

        # Reached the next stop — pause
        if self.total_rotated >= self.scan_step_angle:
            self._stop_base()
            self.scan_paused = True
            self.scan_pause_start = time.time()
            self.get_logger().info(
                f'[SCAN] Paused at stop {self.scan_stop_index + 1}/'
                f'{SCAN_NUM_STOPS} — checking YOLO')

    def _handle_explore(self):
        """Explore the environment, building an occupancy grid and looking for the target."""
        # Always update the map
        self._update_occupancy_grid()

        # Check for target continuously while exploring
        detection = self._detect(self.target_object)
        if detection is not None:
            full = self._detect_and_localize(self.target_object)
            if full is not None:
                pixel, base_pt = full
                self.target_base_pt = base_pt
                self.last_detection_time = time.time()
                self._stop_base()
                self.get_logger().info(
                    f'[EXPLORE] Found {self.target_object} while exploring!')
                self._transition_to(NavState.APPROACH)
                return

        # Check if it's time to stop and do a full scan
        elapsed = time.time() - self.explore_start_time
        dist_traveled = 0.0
        if self.explore_start_pos is not None:
            pose = self._get_base_pose_in_odom()
            if pose is not None:
                dx = pose[0] - self.explore_start_pos[0]
                dy = pose[1] - self.explore_start_pos[1]
                dist_traveled = np.sqrt(dx**2 + dy**2)

        if elapsed > EXPLORE_SCAN_INTERVAL or dist_traveled > EXPLORE_SCAN_DISTANCE:
            self._stop_base()
            self._start_scan()
            return

        # Pick an exploration goal if we don't have one (or reached it)
        if self.explore_goal is not None:
            pose = self._get_base_pose_in_odom()
            if pose is not None:
                dx = self.explore_goal[0] - pose[0]
                dy = self.explore_goal[1] - pose[1]
                if np.sqrt(dx**2 + dy**2) < 0.3:
                    self.explore_goal = None  # reached goal, pick new one

        if self.explore_goal is None:
            pose = self._get_base_pose_in_odom()
            if pose is None:
                return
            rx, ry, rtheta = pose
            actual_heading = rtheta - np.pi / 2

            goal = self.occ_grid.find_best_frontier(rx, ry, actual_heading)
            if goal is not None:
                self.explore_goal = goal
                self.get_logger().info(
                    f'[EXPLORE] New frontier goal: ({goal[0]:.2f}, {goal[1]:.2f})')
            else:
                # No frontiers found — pick a random direction
                angle = actual_heading + np.random.uniform(-np.pi / 2, np.pi / 2)
                dist = 2.0
                self.explore_goal = (rx + dist * np.cos(angle),
                                     ry + dist * np.sin(angle))
                self.get_logger().info(
                    f'[EXPLORE] No frontiers, random goal: '
                    f'({self.explore_goal[0]:.2f}, {self.explore_goal[1]:.2f})')

        # Drive toward exploration goal with obstacle avoidance
        self._drive_toward_goal_with_avoidance(
            self.explore_goal[0], self.explore_goal[1], speed=EXPLORE_SPEED)

    def _handle_approach(self):
        """Drive toward the detected object while avoiding obstacles."""
        # Re-detect on every tick for live tracking
        detection = self._detect_and_localize(self.target_object)
        if detection is not None:
            pixel, base_pt = detection
            self.target_base_pt = base_pt
            self.target_pixel = pixel
            self.last_detection_time = time.time()
        else:
            # Track pixel even when depth fails
            pixel_only = self._detect(self.target_object)
            if pixel_only is not None:
                self.target_pixel = pixel_only
                self.last_detection_time = time.time()

        # Check if we've lost the object entirely (no 2D detection either)
        if time.time() - self.last_detection_time > OBJECT_LOST_TIMEOUT:
            self._stop_base()
            self.get_logger().warn('[APPROACH] Lost object, returning to SCAN')
            self._start_scan()
            return

        # ── Visual servoing mode: no 3D yet, steer toward pixel ──────
        if self.target_base_pt is None:
            if self.target_pixel is not None and self.camera_info is not None:
                img_cx = self.camera_info.width / 2.0
                pixel_error = self.target_pixel[0] - img_cx  # positive = object right of center
                twist = Twist()
                twist.linear.x = APPROACH_SPEED * 0.5  # approach cautiously
                twist.angular.z = np.clip(-0.003 * pixel_error, -0.5, 0.5)
                # Emergency stop check
                clearance, distances = self._analyze_depth_obstacles()
                if min(distances) < EMERGENCY_STOP_DISTANCE:
                    twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(
                    f'[APPROACH] Visual servo: pixel_err={pixel_error:.0f}, '
                    f'angular_z={twist.angular.z:.2f}', throttle_duration_sec=2.0)
            return

        # ── Normal 3D approach ────────────────────────────────────────
        # Object position in base_link: +X=left, -Y=forward
        obj_x = self.target_base_pt.point.x
        obj_y = self.target_base_pt.point.y
        distance = np.sqrt(obj_x**2 + obj_y**2)

        # Heading error: angle from forward (-Y) to object
        heading_error = np.arctan2(obj_x, -obj_y)

        # Close enough to align
        if distance < GRASP_DISTANCE + 0.05:
            self._stop_base()
            self._transition_to(NavState.ALIGN)
            return

        # Drive toward with obstacle avoidance
        clearance, distances = self._analyze_depth_obstacles()
        left_clear, center_clear, right_clear = clearance
        left_dist, center_dist, right_dist = distances

        twist = Twist()

        # Allow driving even if center depth is "blocked" when the obstacle
        # IS the target object (object is closer than threshold)
        center_is_target = (center_dist > distance * 0.7)

        if center_clear or center_is_target:
            speed = min(APPROACH_SPEED, (distance - GRASP_DISTANCE) * 0.5)
            twist.linear.x = max(0.05, speed)
            twist.angular.z = np.clip(1.5 * heading_error, -0.8, 0.8)
        else:
            # Real obstacle in the way — steer around it
            self.get_logger().info(
                f'[APPROACH] Obstacle! L={left_dist:.2f} C={center_dist:.2f} '
                f'R={right_dist:.2f}')
            if left_dist > right_dist:
                twist.angular.z = 0.5
                twist.linear.x = 0.05
            elif right_dist > left_dist:
                twist.angular.z = -0.5
                twist.linear.x = 0.05
            else:
                twist.angular.z = 0.6

        # Emergency stop
        if min(left_dist, center_dist, right_dist) < EMERGENCY_STOP_DISTANCE:
            twist.linear.x = 0.0

        self.cmd_vel_pub.publish(twist)

    def _handle_align(self):
        """Fine-tune position: center the object in view and adjust distance."""
        # Detect pixel position
        pixel = self._detect(self.target_object)
        if pixel is None:
            # Lost during alignment — go back to approach
            self.get_logger().warn('[ALIGN] Lost object, returning to APPROACH')
            self._transition_to(NavState.APPROACH)
            return

        u, v = pixel
        image_center_u = 320  # 640 / 2

        # Pixel error: positive = object is to the right in the image
        # Camera optical: +X = right. To center object, turn right (negative angular.z)
        pixel_error = u - image_center_u
        angular_correction = -0.003 * pixel_error

        # Get 3D position for distance check
        full = self._detect_and_localize(self.target_object)
        if full is not None:
            _, base_pt = full
            self.target_base_pt = base_pt
            self.last_detection_time = time.time()

            distance = np.sqrt(base_pt.point.x**2 + base_pt.point.y**2)

            linear_correction = 0.0
            if distance > GRASP_DISTANCE + ALIGN_DISTANCE_TOLERANCE:
                linear_correction = 0.05
            elif distance < GRASP_DISTANCE - ALIGN_DISTANCE_TOLERANCE:
                linear_correction = -0.05

            centered = abs(pixel_error) < ALIGN_PIXEL_TOLERANCE
            at_distance = abs(distance - GRASP_DISTANCE) < ALIGN_DISTANCE_TOLERANCE

            if centered and at_distance:
                self._stop_base()
                self.get_logger().info(
                    f'[ALIGN] Positioned! dist={distance:.3f}m, '
                    f'pixel_err={pixel_error}')
                self._transition_to(NavState.POSITIONED)
                return

            twist = Twist()
            twist.linear.x = linear_correction
            twist.angular.z = np.clip(angular_correction, -0.3, 0.3)
            self.cmd_vel_pub.publish(twist)
        else:
            # Have pixel but depth failed — just do angular correction
            twist = Twist()
            twist.angular.z = np.clip(angular_correction, -0.3, 0.3)
            self.cmd_vel_pub.publish(twist)

    def _handle_positioned(self):
        """Stay stopped — we're in position."""
        self.cmd_vel_pub.publish(Twist())

    # ═══════════════════════════════════════════════════════════════════
    # Control loop
    # ═══════════════════════════════════════════════════════════════════

    def control_loop(self):
        """Navigation state machine at 10Hz."""
        if not self.target_object:
            return

        # Publish map to RViz at 2Hz
        self._publish_map()

        if self.nav_state == NavState.INIT:
            self._handle_init()
        elif self.nav_state == NavState.SCAN:
            self._handle_scan()
        elif self.nav_state == NavState.EXPLORE:
            self._handle_explore()
        elif self.nav_state == NavState.APPROACH:
            self._handle_approach()
        elif self.nav_state == NavState.ALIGN:
            self._handle_align()
        elif self.nav_state == NavState.POSITIONED:
            self._handle_positioned()


# ═══════════════════════════════════════════════════════════════════════
# Standalone entry point
# ═══════════════════════════════════════════════════════════════════════

def main(args=None):
    """Standalone test: search for an object by class name."""
    rclpy.init(args=args)

    node = NavigateToObject(target_object='banana')
    node.declare_parameter('target_object', 'banana')
    target = node.get_parameter('target_object').value
    if target:
        node.set_target(target)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
