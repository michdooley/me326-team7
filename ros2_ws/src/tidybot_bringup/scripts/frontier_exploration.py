#!/usr/bin/env python3
"""
TidyBot2 Frontier Explorer

Maintains a persistent 2-D log-odds occupancy grid that accumulates over
the full exploration run.  The exploration loop repeats:

  1. SCANNING   — rotates the base 360° in place, integrating every depth
                  frame into the map.
  2. SELECTING  — clusters frontier cells (FREE↔UNKNOWN boundary) and picks
                  the highest-scoring one (score = cluster_size / distance).
  3. NAVIGATING — drives to the selected frontier, mapping along the way.

Key design choices
──────────────────
• Persistent map — the occupancy grid is NEVER cleared between scans.
  Evidence accumulates correctly because the MuJoCo bridge co-publishes a
  synchronised odom→base_link TF with each depth frame (same timestamp,
  same sim state), eliminating TF timing lag.

• Obstacle-only raytrace — free-space rays are traced ONLY for depth
  returns that fall in the obstacle height band [MIN_OBSTACLE_Z,
  MAX_OBSTACLE_Z].  Floor and ceiling returns are silently dropped.
  This prevents a 3-D→2-D artefact where a floor point visible beside a
  wall in 3-D produces a 2-D ray that crosses the wall's grid footprint
  and erroneously marks it as free ("see-through" walls).

• Early-exit raytrace — each free-space ray stops when it reaches a cell
  already above LOG_ODDS_OCC_THRESH (≥ 2 confirmed hits).  Cells below the
  threshold (single-frame noise) are still correctable by MISS updates.

Publishes:
  /map               nav_msgs/OccupancyGrid   — standard RViz map
  /map_image         sensor_msgs/Image        — RGB debug view
  /frontier_markers  visualization_msgs/MarkerArray

Subscribes:
  /camera/depth/image_raw    — depth frames (16UC1, mm)
  /camera/color/camera_info  — camera intrinsics
  /base/goal_reached         — Bool, fired when navigation completes

Commands:
  /cmd_vel           geometry_msgs/Twist   — in-place rotation scans
  /base/target_pose  geometry_msgs/Pose2D  — navigation goal

Prerequisites:
    ros2 launch tidybot_bringup sim.launch.py scene:=scene_obstacles.xml

Usage:
    ros2 run tidybot_bringup frontier_exploration.py
"""

import time
from collections import deque
from enum import Enum

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import MapMetaData, OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

try:
    from cv_bridge import CvBridge
except ImportError:
    print("ERROR: cv_bridge is required.")
    raise


class ExploreState(Enum):
    SCANNING   = 0
    SELECTING  = 1
    NAVIGATING = 2
    COMPLETE   = 3


class FrontierExplorer(Node):

    # ── Grid ─────────────────────────────────────────────────────────────────
    GRID_RESOLUTION = 0.05   # m / cell
    GRID_SIZE       = 300    # cells per axis  →  15 m × 15 m world
    GRID_ORIGIN_X   = -7.5   # world X of grid column 0  (m)
    GRID_ORIGIN_Y   = -7.5   # world Y of grid row    0  (m)

    # ── Depth filtering ───────────────────────────────────────────────────────
    MIN_DEPTH_M       = 0.30   # ignore returns closer than this (robot body)
    MAX_DEPTH_M       = 5.00   # ignore returns farther  than this
    DEPTH_SUBSAMPLE   = 4      # process every Nth pixel (speed vs. quality)
    ROBOT_BODY_RADIUS = 0.60   # m — XY radius around camera to exclude self-hits
    FREE_TRACE_MAX_M  = 3.00   # m — max free-ray length for floor/ceiling returns

    # ── Height filter ─────────────────────────────────────────────────────────
    # Only 3-D points in this Z band vote for OCCUPIED — floor and ceiling
    # are excluded without requiring a full 3-D voxel representation.
    MIN_OBSTACLE_Z = 0.10    # m — clears floor returns (z ≈ 0)
    MAX_OBSTACLE_Z = 0.80    # m — ignores camera mount / ceilings

    # ── Log-odds belief (Thrun, Burgard & Fox, Ch. 9) ────────────────────────
    LOG_ODDS_HIT         =  0.85   # evidence of occupancy per observation
    LOG_ODDS_MISS        = -0.40   # evidence of free space per ray cell
    LOG_ODDS_MIN         = -3.00   # clamp floor  (P ≈ 0.05  very free)
    LOG_ODDS_MAX         =  3.00   # clamp ceiling (P ≈ 0.95  very occupied)
    LOG_ODDS_FREE_THRESH = -0.50   # below → publish   0 (FREE)
    LOG_ODDS_OCC_THRESH  =  1.50   # log-odds threshold for raytrace early-exit

    # ── Angular-diversity obstacle confirmation ───────────────────────────────
    # A smeared cell is only ever seen from ONE camera angle (the angle at
    # which TF lag placed it incorrectly).  A real obstacle is seen from
    # many angles during the 360° sweep.  We require hits from at least
    # MIN_HIT_COUNT distinct directions (separated by MIN_HIT_ANGLE_SEP) before
    # a cell is published as OCCUPIED or used to stop free-space rays.
    # The raytrace early-exit is also gated on hit_count so smeared cells
    # remain correctable by MISS updates from other directions.
    MIN_HIT_ANGLE_SEP = np.radians(20)  # rad — min gap between accepted hits
    MIN_HIT_COUNT     = 2               # distinct-angle hits to confirm obstacle

    # ── Map publishing ────────────────────────────────────────────────────────
    MAP_PUBLISH_RATE = 2.0   # Hz

    # ── 360° scan ─────────────────────────────────────────────────────────────
    SCAN_ANGULAR_VEL = 0.8   # rad/s  (~8 s per revolution, slower = less TF lag error)
    SCAN_SETTLE_TIME = 1.0   # s — wait after stopping for vibration to die

    # ── Frontier selection ────────────────────────────────────────────────────
    MIN_FRONTIER_SIZE   = 8    # cells — ignore tiny frontier clusters
    FRONTIER_NAV_OFFSET = 0.4  # m — goal set this far in front of centroid

    # ── Navigation ────────────────────────────────────────────────────────────
    NAV_TIMEOUT       = 30.0  # s before giving up on a navigation goal
    NO_FRONTIER_LIMIT = 3     # consecutive empty scans → declare COMPLETE

    # ─────────────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('frontier_explorer')

        # Persistent log-odds grid (never cleared between scans)
        self.log_odds  = np.zeros(
            (self.GRID_SIZE, self.GRID_SIZE), dtype=np.float32)

        # Angular-diversity confirmation maps.
        # hit_count[y,x] = number of distinct-angle HITs received so far.
        # hit_angle[y,x] = camera heading (rad) of the most recently accepted
        #                  HIT (NaN = no hit yet).  A new HIT is "accepted" only
        #                  if the current heading differs from hit_angle by at
        #                  least MIN_HIT_ANGLE_SEP, preventing the same scan arc
        #                  from incrementing the counter on every frame.
        self.hit_count = np.zeros(
            (self.GRID_SIZE, self.GRID_SIZE), dtype=np.uint8)
        self.hit_angle = np.full(
            (self.GRID_SIZE, self.GRID_SIZE), np.nan, dtype=np.float32)

        # Camera state
        self.latest_depth       = None   # np.ndarray (H×W, uint16, mm)
        self.latest_depth_stamp = None   # builtin_interfaces/Time
        self.camera_K           = None   # 3×3 intrinsic matrix
        self.cv_bridge          = CvBridge()

        # State machine
        self.state             = ExploreState.SCANNING
        self.scan_accumulated  = 0.0
        self.scan_last_heading = None
        self.scan_settle_start = None
        self.nav_start_time    = None
        self.no_frontier_count = 0
        self.base_goal_reached = False
        self.last_map_publish  = 0.0

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image,      '/camera/depth/image_raw',
                                 self._depth_cb,       be)
        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 self._camera_info_cb, be)
        self.create_subscription(Bool, '/base/goal_reached',
                                 self._goal_reached_cb, 10)

        # Publishers
        self.map_pub       = self.create_publisher(OccupancyGrid, '/map',              10)
        self.map_image_pub = self.create_publisher(Image,         '/map_image',        10)
        self.frontier_pub  = self.create_publisher(MarkerArray,   '/frontier_markers', 10)
        self.cmd_vel_pub   = self.create_publisher(Twist,         '/cmd_vel',          10)
        self.base_goal_pub = self.create_publisher(Pose2D,        '/base/target_pose', 10)

        self.get_logger().info('=' * 55)
        self.get_logger().info('Frontier Explorer  (persistent map)')
        self.get_logger().info(
            f'  Grid   : {self.GRID_SIZE}×{self.GRID_SIZE} '
            f'@ {self.GRID_RESOLUTION} m/cell '
            f'({self.GRID_SIZE * self.GRID_RESOLUTION:.0f} m × '
            f'{self.GRID_SIZE * self.GRID_RESOLUTION:.0f} m)')
        self.get_logger().info(
            f'  Height : {self.MIN_OBSTACLE_Z}–{self.MAX_OBSTACLE_Z} m '
            f'(floor excluded)')
        self.get_logger().info(
            f'  Scan   : {self.SCAN_ANGULAR_VEL} rad/s '
            f'(≈{2*np.pi/self.SCAN_ANGULAR_VEL:.0f} s per 360°)')
        self.get_logger().info('=' * 55)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _depth_cb(self, msg: Image):
        """Store the latest depth frame and immediately integrate it."""
        try:
            depth = self.cv_bridge.imgmsg_to_cv2(msg, '16UC1')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')
            return
        self.latest_depth       = depth
        self.latest_depth_stamp = msg.header.stamp
        if self.camera_K is not None:
            self._integrate_depth()

    def _camera_info_cb(self, msg: CameraInfo):
        self.camera_K = np.array(msg.k).reshape(3, 3)

    def _goal_reached_cb(self, msg: Bool):
        if msg.data:
            self.base_goal_reached = True

    # ── Grid helpers ──────────────────────────────────────────────────────────

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.GRID_ORIGIN_X) / self.GRID_RESOLUTION)
        gy = int((wy - self.GRID_ORIGIN_Y) / self.GRID_RESOLUTION)
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = self.GRID_ORIGIN_X + (gx + 0.5) * self.GRID_RESOLUTION
        wy = self.GRID_ORIGIN_Y + (gy + 0.5) * self.GRID_RESOLUTION
        return wx, wy

    def in_grid(self, gx, gy):
        return 0 <= gx < self.GRID_SIZE and 0 <= gy < self.GRID_SIZE

    # ── Robot pose ────────────────────────────────────────────────────────────

    def get_base_pose(self):
        """Return (x, y, theta) of base_link in odom, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(f'TF odom→base_link: {e}')
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return t.x, t.y, np.arctan2(siny, cosy)

    def get_heading(self):
        p = self.get_base_pose()
        return p[2] if p is not None else None

    # ── Mapping ───────────────────────────────────────────────────────────────

    def _integrate_depth(self):
        """
        Project the current depth frame into the persistent log-odds grid.

        Only depth returns whose reprojected world-Z falls in the obstacle
        height band [MIN_OBSTACLE_Z, MAX_OBSTACLE_Z] are used:

          • HIT  (+0.85) is added to the cell containing the 3-D point.
          • A free-space ray is traced from the camera origin to that cell
            (exclusive), applying MISS (−0.40) to every intermediate cell
            and stopping early if a confirmed obstacle (log_odds ≥ 1.50)
            is encountered.

        Floor and ceiling returns are completely ignored.  This avoids the
        3-D→2-D projection artefact where a floor ray visible beside a wall
        in 3-D produces a 2-D line that crosses the wall's grid footprint
        and would erroneously mark it as free space.
        """
        if self.latest_depth is None or self.camera_K is None:
            return

        # ── 1. Look up camera pose in odom ───────────────────────────────────
        # The MuJoCo bridge broadcasts a TF with the exact same timestamp as
        # the depth image, so this stamped lookup has zero lag.
        try:
            stamp = rclpy.time.Time.from_msg(self.latest_depth_stamp)
            tf = self.tf_buffer.lookup_transform(
                'odom', 'camera_depth_optical_frame',
                stamp, timeout=Duration(seconds=0.1))
        except Exception:
            # Fallback to latest if stamped lookup fails (e.g. on startup)
            try:
                tf = self.tf_buffer.lookup_transform(
                    'odom', 'camera_depth_optical_frame',
                    rclpy.time.Time(), timeout=Duration(seconds=0.05))
            except Exception:
                return

        t  = tf.transform.translation
        q  = tf.transform.rotation
        cam_origin = np.array([t.x, t.y, t.z])

        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        R = np.array([
            [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
            [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
            [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)],
        ])

        # ── 2. Back-project depth pixels to odom frame ───────────────────────
        fx = self.camera_K[0, 0];  fy = self.camera_K[1, 1]
        cx = self.camera_K[0, 2];  cy = self.camera_K[1, 2]

        depth = self.latest_depth
        h, w  = depth.shape
        step  = self.DEPTH_SUBSAMPLE

        vs, us    = np.mgrid[0:h:step, 0:w:step]
        us        = us.ravel();  vs = vs.ravel()
        depths_mm = depth[vs, us].astype(np.float64)

        min_mm = self.MIN_DEPTH_M * 1000.0
        max_mm = self.MAX_DEPTH_M * 1000.0
        valid  = (depths_mm > min_mm) & (depths_mm < max_mm)
        us     = us[valid];  vs     = vs[valid]
        depths_m = depths_mm[valid] / 1000.0

        if len(depths_m) == 0:
            return

        x_cam = (us - cx) * depths_m / fx
        y_cam = (vs - cy) * depths_m / fy
        z_cam = depths_m
        pts_odom = (R @ np.stack([x_cam, y_cam, z_cam], axis=1).T).T + cam_origin

        # ── 3. Compute grid coordinates ───────────────────────────────────────
        gxs = ((pts_odom[:, 0] - self.GRID_ORIGIN_X) / self.GRID_RESOLUTION).astype(int)
        gys = ((pts_odom[:, 1] - self.GRID_ORIGIN_Y) / self.GRID_RESOLUTION).astype(int)

        # ── 4. Build masks ────────────────────────────────────────────────────
        # 4a. Self-exclusion: ignore returns whose XY is within ROBOT_BODY_RADIUS
        #     of the camera origin — these are the robot's own body / TF-axis
        #     visualizations that appear at the base frame origin.
        dist_xy  = np.hypot(pts_odom[:, 0] - cam_origin[0],
                            pts_odom[:, 1] - cam_origin[1])
        not_self = dist_xy > self.ROBOT_BODY_RADIUS

        in_bounds = ((gxs >= 0) & (gxs < self.GRID_SIZE) &
                     (gys >= 0) & (gys < self.GRID_SIZE))
        in_height = ((pts_odom[:, 2] > self.MIN_OBSTACLE_Z) &
                     (pts_odom[:, 2] < self.MAX_OBSTACLE_Z))

        # Obstacle returns → free raytrace + HIT
        obs_mask = not_self & in_bounds & in_height

        # Floor/ceiling returns → free raytrace only (no HIT), limited depth so
        # rays don't cross distant walls they can't physically see in 3-D.
        # The early exit in _raytrace_free stops any ray at a confirmed obstacle
        # (log_odds ≥ OCC_THRESH), protecting established wall cells.
        floor_mask = (not_self & in_bounds & ~in_height &
                      (depths_m < self.FREE_TRACE_MAX_M))

        obs_idx   = np.where(obs_mask)[0]
        floor_idx = np.where(floor_mask)[0]

        if len(obs_idx) == 0 and len(floor_idx) == 0:
            return

        cam_gx, cam_gy = self.world_to_grid(cam_origin[0], cam_origin[1])

        # ── 5a. Free-space raytrace: obstacle returns ─────────────────────────
        if len(obs_idx) > 0:
            ray_step = max(1, len(obs_idx) // 500)
            for i in obs_idx[::ray_step]:
                self._raytrace_free(cam_gx, cam_gy, int(gxs[i]), int(gys[i]))

        # ── 5b. Free-space raytrace: floor/ceiling returns ────────────────────
        # These rays mark empty space in directions where no obstacle-height
        # return was seen (e.g. the robot faces an open corridor).  They do NOT
        # add any HIT, so they cannot create false obstacles.
        if len(floor_idx) > 0:
            ray_step2 = max(1, len(floor_idx) // 300)
            for i in floor_idx[::ray_step2]:
                self._raytrace_free(cam_gx, cam_gy, int(gxs[i]), int(gys[i]))

        # ── 6. Mark obstacle cells with HIT ───────────────────────────────────
        if len(obs_idx) > 0:
            np.add.at(self.log_odds, (gys[obs_mask], gxs[obs_mask]), self.LOG_ODDS_HIT)
            np.clip(self.log_odds, self.LOG_ODDS_MIN, self.LOG_ODDS_MAX, out=self.log_odds)

        # ── 7. Update angular-diversity hit count ──────────────────────────────
        # For each unique grid cell that received a HIT this frame, check
        # whether the current camera heading differs from the cell's stored
        # heading by at least MIN_HIT_ANGLE_SEP.  If so (or if this is the
        # first hit), record the new heading and increment hit_count.
        # Multiple depth pixels mapping to the same cell in one frame count
        # only once (deduplicated via np.unique).
        if len(obs_idx) > 0:
            base_pose = self.get_base_pose()
            if base_pose is not None:
                cam_theta = float(base_pose[2])

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
                    hp_gy     = u_gy[has_prior]
                    hp_gx     = u_gx[has_prior]
                    prior     = self.hit_angle[hp_gy, hp_gx]
                    diff      = np.abs(cam_theta - prior)
                    diff      = np.minimum(diff, 2 * np.pi - diff)  # wrap to [0, π]
                    diverse   = diff >= self.MIN_HIT_ANGLE_SEP
                    if np.any(diverse):
                        d_gy = hp_gy[diverse];  d_gx = hp_gx[diverse]
                        self.hit_count[d_gy, d_gx] = np.minimum(
                            self.hit_count[d_gy, d_gx].astype(np.uint16) + 1,
                            255).astype(np.uint8)
                        self.hit_angle[d_gy, d_gx] = cam_theta

    def _raytrace_free(self, x0: int, y0: int, x1: int, y1: int):
        """
        Bresenham ray from (x0,y0) to (x1,y1) exclusive, applying
        LOG_ODDS_MISS to each traversed cell.

        Stops early when a cell's log_odds ≥ LOG_ODDS_OCC_THRESH (i.e. the
        cell has been confirmed by ≥ 2 independent hits).  This preserves
        established obstacles from being eroded by later free-space rays
        that cross their 2-D footprint due to projection geometry.

        Cells below OCC_THRESH (< 2 hits) are still correctable.
        """
        dx = abs(x1 - x0);  sx = 1 if x0 < x1 else -1
        dy = abs(y1 - y0);  sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0

        while not (x == x1 and y == y1):
            if self.in_grid(x, y):
                # Stop only at cells confirmed from multiple distinct angles.
                # Smeared cells (hit_count < MIN_HIT_COUNT) are NOT protected
                # so MISS updates from other directions can clear them.
                if (self.hit_count[y, x] >= self.MIN_HIT_COUNT and
                        self.log_odds[y, x] >= self.LOG_ODDS_OCC_THRESH):
                    break
                self.log_odds[y, x] = max(
                    self.LOG_ODDS_MIN,
                    self.log_odds[y, x] + self.LOG_ODDS_MISS)
            e2 = 2 * err
            if e2 > -dy:
                err -= dy;  x += sx
            if e2 < dx:
                err += dx;  y += sy

    # ── Frontier detection ────────────────────────────────────────────────────

    def find_frontiers(self):
        """
        Return frontier clusters sorted by score (size / distance).

        A frontier cell is FREE (log_odds ≤ FREE_THRESH) and 4-adjacent to
        at least one UNKNOWN cell (FREE_THRESH < log_odds < OCC_THRESH).
        Connected components (8-connected) smaller than MIN_FRONTIER_SIZE
        are discarded.
        """
        free_mask    = self.log_odds <= self.LOG_ODDS_FREE_THRESH
        # Use the same confirmed-obstacle definition as publish_map so that
        # smeared cells (hit_count < MIN_HIT_COUNT) count as UNKNOWN, not walls.
        confirmed_occ = ((self.log_odds  >= self.LOG_ODDS_OCC_THRESH) &
                         (self.hit_count >= self.MIN_HIT_COUNT))
        unknown_mask  = ~free_mask & ~confirmed_occ

        frontier_mask = np.zeros_like(self.log_odds, dtype=bool)
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            frontier_mask |= (free_mask &
                              np.roll(np.roll(unknown_mask, dy, axis=0), dx, axis=1))

        fys, fxs = np.where(frontier_mask)
        if len(fxs) == 0:
            return []

        frontier_set = set(zip(fxs.tolist(), fys.tolist()))
        visited      = set()
        clusters     = []

        for seed in frontier_set:
            if seed in visited:
                continue
            cluster = []
            queue   = deque([seed])
            visited.add(seed)
            while queue:
                cx, cy = queue.popleft()
                cluster.append((cx, cy))
                for dy, dx in [(-1,0),(1,0),(0,-1),(0,1),
                                (-1,-1),(-1,1),(1,-1),(1,1)]:
                    nb = (cx + dx, cy + dy)
                    if nb in frontier_set and nb not in visited:
                        visited.add(nb)
                        queue.append(nb)
            if len(cluster) >= self.MIN_FRONTIER_SIZE:
                clusters.append(cluster)

        if not clusters:
            return []

        base_pose = self.get_base_pose()
        if base_pose is None:
            return []
        bx, by, _ = base_pose

        results = []
        for cluster in clusters:
            mgx = np.mean([c[0] for c in cluster])
            mgy = np.mean([c[1] for c in cluster])
            wx, wy = self.grid_to_world(int(mgx), int(mgy))
            dist   = np.hypot(wx - bx, wy - by)
            score  = len(cluster) / max(dist, 0.5)
            results.append((wx, wy, len(cluster), score))

        results.sort(key=lambda r: r[3], reverse=True)
        return results

    # ── Publishing ────────────────────────────────────────────────────────────

    def publish_map(self):
        grid = np.full((self.GRID_SIZE, self.GRID_SIZE), -1, dtype=np.int8)
        grid[self.log_odds <= self.LOG_ODDS_FREE_THRESH] = 0
        # OCCUPIED only when confirmed from ≥ MIN_HIT_COUNT distinct angles.
        # Cells with high log_odds but hit_count < 2 are smear artifacts —
        # they stay UNKNOWN (gray) and remain correctable by MISS updates.
        confirmed = ((self.log_odds  >= self.LOG_ODDS_OCC_THRESH) &
                     (self.hit_count >= self.MIN_HIT_COUNT))
        grid[confirmed] = 100

        msg = OccupancyGrid()
        msg.header.stamp              = self.get_clock().now().to_msg()
        msg.header.frame_id           = 'odom'
        msg.info                      = MapMetaData()
        msg.info.resolution           = self.GRID_RESOLUTION
        msg.info.width                = self.GRID_SIZE
        msg.info.height               = self.GRID_SIZE
        msg.info.origin.position.x    = float(self.GRID_ORIGIN_X)
        msg.info.origin.position.y    = float(self.GRID_ORIGIN_Y)
        msg.info.origin.position.z    = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.ravel().tolist()
        self.map_pub.publish(msg)

        # RGB debug image: gray=unknown, white=free, black=occupied
        img = np.full((self.GRID_SIZE, self.GRID_SIZE, 3), 128, dtype=np.uint8)
        img[grid == 0]   = [255, 255, 255]
        img[grid == 100] = [  0,   0,   0]
        img_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding='rgb8')
        img_msg.header.stamp    = msg.header.stamp
        img_msg.header.frame_id = 'odom'
        self.map_image_pub.publish(img_msg)

    def publish_frontiers(self, frontiers):
        ma = MarkerArray()
        for i, (wx, wy, size, score) in enumerate(frontiers):
            m = Marker()
            m.header.stamp       = self.get_clock().now().to_msg()
            m.header.frame_id    = 'odom'
            m.ns                 = 'frontiers'
            m.id                 = i
            m.type               = Marker.CYLINDER
            m.action             = Marker.ADD
            m.pose.position.x    = wx
            m.pose.position.y    = wy
            m.pose.position.z    = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = 0.2;  m.scale.y = 0.2;  m.scale.z = 0.3
            brightness = 1.0 if i == 0 else 0.4
            m.color    = ColorRGBA(r=0.0, g=brightness, b=0.3, a=0.8)
            m.lifetime.sec = 5
            ma.markers.append(m)
        self.frontier_pub.publish(ma)

    # ── State machine ─────────────────────────────────────────────────────────

    def _start_scan(self):
        """Reset rotation counters and enter SCANNING state."""
        self.scan_accumulated  = 0.0
        self.scan_last_heading = None
        self.scan_settle_start = None
        self.state = ExploreState.SCANNING
        self.get_logger().info('[SCAN] Starting 360° rotation scan...')

    def _stop_base(self):
        self.cmd_vel_pub.publish(Twist())

    def _state_scanning(self):
        if self.scan_settle_start is not None:
            if time.time() - self.scan_settle_start >= self.SCAN_SETTLE_TIME:
                self.get_logger().info('[SCAN] Settled — selecting frontier.')
                self.state = ExploreState.SELECTING
            return

        heading = self.get_heading()
        if heading is None:
            return

        if self.scan_last_heading is not None:
            delta = (heading - self.scan_last_heading + np.pi) % (2*np.pi) - np.pi
            self.scan_accumulated += abs(delta)

        self.scan_last_heading = heading

        if self.scan_accumulated >= 2 * np.pi:
            self._stop_base()
            self.scan_settle_start = time.time()
            self.get_logger().info(
                f'[SCAN] 360° done '
                f'({np.degrees(self.scan_accumulated):.0f}° accumulated). '
                f'Settling {self.SCAN_SETTLE_TIME} s...')
        else:
            cmd = Twist()
            cmd.angular.z = self.SCAN_ANGULAR_VEL
            self.cmd_vel_pub.publish(cmd)

    def _state_selecting(self):
        frontiers = self.find_frontiers()
        self.publish_frontiers(frontiers)

        if not frontiers:
            self.no_frontier_count += 1
            if self.no_frontier_count >= self.NO_FRONTIER_LIMIT:
                self.get_logger().info('[SELECT] Exploration complete.')
                self.state = ExploreState.COMPLETE
            else:
                self.get_logger().info(
                    f'[SELECT] No frontiers '
                    f'({self.no_frontier_count}/{self.NO_FRONTIER_LIMIT}) '
                    f'— re-scanning.')
                self._start_scan()
            return

        self.no_frontier_count = 0
        wx, wy, size, score = frontiers[0]
        self.get_logger().info(
            f'[SELECT] Best frontier ({wx:.2f}, {wy:.2f}) '
            f'size={size} score={score:.1f}')

        base_pose = self.get_base_pose()
        if base_pose is None:
            self._start_scan()
            return
        bx, by, _ = base_pose
        dx, dy = wx - bx, wy - by
        dist   = np.hypot(dx, dy)

        if dist > self.FRONTIER_NAV_OFFSET * 2:
            ratio  = (dist - self.FRONTIER_NAV_OFFSET) / dist
            goal_x = bx + dx * ratio
            goal_y = by + dy * ratio
        else:
            goal_x, goal_y = wx, wy

        goal_theta = np.arctan2(dy, dx)
        user_theta = goal_theta - np.pi / 2   # bridge frame convention

        self.get_logger().info(
            f'[SELECT] Goal ({goal_x:.2f}, {goal_y:.2f}) '
            f'heading={np.degrees(goal_theta):.0f}°')

        goal = Pose2D()
        goal.x = goal_x;  goal.y = goal_y;  goal.theta = user_theta
        self.base_goal_reached = False
        self.base_goal_pub.publish(goal)
        self.nav_start_time = time.time()
        self.state = ExploreState.NAVIGATING

    def _state_navigating(self):
        if self.base_goal_reached:
            self.get_logger().info('[NAV] Goal reached — starting scan.')
            self._start_scan()
            return
        if time.time() - self.nav_start_time > self.NAV_TIMEOUT:
            self.get_logger().warn('[NAV] Timeout — scanning from here.')
            self._start_scan()

    def _state_complete(self):
        self._stop_base()
        self.publish_map()

        free     = int(np.sum(self.log_odds <= self.LOG_ODDS_FREE_THRESH))
        occupied = int(np.sum(self.log_odds >= self.LOG_ODDS_OCC_THRESH))
        total    = self.GRID_SIZE ** 2

        self.get_logger().info('=' * 55)
        self.get_logger().info('EXPLORATION COMPLETE')
        self.get_logger().info(f'  Coverage : {(free+occupied)/total*100:.1f}%')
        self.get_logger().info(f'  Free     : {free}')
        self.get_logger().info(f'  Occupied : {occupied}')
        self.get_logger().info(f'  Unknown  : {total - free - occupied}')
        self.get_logger().info('=' * 55)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1.0)
            self.publish_map()

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        self.get_logger().info('Waiting for depth data and TF...')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_depth is not None and self.camera_K is not None:
                try:
                    self.tf_buffer.lookup_transform(
                        'odom', 'camera_depth_optical_frame',
                        rclpy.time.Time(), timeout=Duration(seconds=0.1))
                    break
                except Exception:
                    pass

        self.get_logger().info('Sensors ready — starting exploration!')
        self._start_scan()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            now = time.time()
            if now - self.last_map_publish >= 1.0 / self.MAP_PUBLISH_RATE:
                self.publish_map()
                self.last_map_publish = now

            if   self.state == ExploreState.SCANNING:
                self._state_scanning()
            elif self.state == ExploreState.SELECTING:
                self._state_selecting()
            elif self.state == ExploreState.NAVIGATING:
                self._state_navigating()
            elif self.state == ExploreState.COMPLETE:
                self._state_complete()
                break


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
