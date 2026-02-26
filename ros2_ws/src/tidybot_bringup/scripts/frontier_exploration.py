#!/usr/bin/env python3
"""
TidyBot2 Frontier Explorer

Maintains a persistent 2-D log-odds occupancy grid that accumulates over
the full exploration run.  The exploration loop repeats:

  1. SCANNING   — rotates the base 360° in place, integrating every depth
                  frame into the map.
  2. SELECTING  — clusters frontier cells (FREE↔UNKNOWN boundary) and picks
                  the highest-scoring one (score = nearby_unknown / distance).
  3. NAVIGATING — follows A* waypoints to the selected frontier with
                  depth-based reactive obstacle avoidance.

Key design choices
──────────────────
• Persistent map — the occupancy grid is NEVER cleared between scans.
  Evidence accumulates correctly because the MuJoCo bridge co-publishes a
  synchronised odom→base_link TF with each depth frame (same timestamp,
  same sim state), eliminating TF timing lag.

• Obstacle-only raytrace — free-space rays are traced ONLY for depth
  returns that fall in the obstacle height band [MIN_OBSTACLE_Z,
  MAX_OBSTACLE_Z].  Floor returns below the band also generate free-space
  rays (capped at FREE_TRACE_MAX_M) to clear open corridors, but never
  mark HIT.  This prevents "see-through" walls from 3-D→2-D projection.

• Early-exit raytrace — each free-space ray stops at a cell confirmed from
  ≥ MIN_HIT_COUNT distinct angles (hit_count ≥ 2 AND log_odds ≥ OCC_THRESH).
  Smeared cells (hit_count < 2) are not protected and can be corrected.

• Angular-diversity confirmation — a grid cell is only published as OCCUPIED
  when it has been hit from ≥ 2 directions separated by ≥ MIN_HIT_ANGLE_SEP.
  This suppresses smear artefacts that are only seen from a single angle.

• A* path planning — navigation goals are validated by computing an A* path
  on the inflated occupancy grid.  Waypoints are extracted and followed via
  cmd_vel with reactive depth-based obstacle avoidance.

Publishes:
  /map               nav_msgs/OccupancyGrid   — grayscale confidence map
  /frontier_markers  visualization_msgs/MarkerArray
  /cmd_vel           geometry_msgs/Twist       — base velocity commands

Subscribes:
  /camera/depth/image_raw    — depth frames (16UC1, mm)
  /camera/depth/camera_info  — depth camera intrinsics (fovy=57°)

Prerequisites:
    ros2 launch tidybot_bringup sim.launch.py scene:=scene_obstacles.xml

Usage:
    ros2 run tidybot_bringup frontier_exploration.py
"""

import heapq
import time
from collections import deque
from enum import Enum

import numpy as np
from scipy.ndimage import binary_dilation, label as ndimage_label
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import MapMetaData, OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
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
    MIN_DEPTH_M       = 0.40   # ignore returns closer than this (robot body)
    MAX_DEPTH_M       = 5.00   # ignore returns farther  than this
    DEPTH_SUBSAMPLE   = 8      # process every Nth pixel (speed vs. quality)
    DEPTH_H_CROP      = 0.20   # fraction of image width to crop from each side
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
    LOG_ODDS_MIN         = -10.0   # clamp floor  — needs ~25 net MISSes to reach
    LOG_ODDS_MAX         =  10.0   # clamp ceiling — needs ~12 net HITs to reach
    LOG_ODDS_FREE_THRESH = -0.50   # frontier detection: below this → FREE cell
    LOG_ODDS_OCC_THRESH  =  1.50   # raytrace early-exit threshold (≈ 2 net HITs)

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
    MAP_PUBLISH_RATE = 1.0   # Hz

    # ── 360° scan ─────────────────────────────────────────────────────────────
    SCAN_ANGULAR_VEL = 0.8   # rad/s  (~8 s per revolution, slower = less TF lag error)
    SCAN_SETTLE_TIME = 1.0   # s — wait after stopping for vibration to die

    # ── Frontier selection ────────────────────────────────────────────────────
    MIN_FRONTIER_SIZE   = 8    # cells — ignore tiny frontier clusters
    MIN_FRONTIER_DIST   = 1.0  # m — ignore frontiers closer than this to the robot
    ROBOT_CLEARANCE     = 0.30 # m — inflation radius for A* and goal validation

    # ── Navigation (cmd_vel waypoint following) ─────────────────────────────
    NAV_TIMEOUT        = 120.0  # s per waypoint — resets on each waypoint advance
    NO_FRONTIER_LIMIT  = 3     # consecutive empty scans → declare COMPLETE
    WAYPOINT_SPACING   = 20    # cells (~1.0 m) between A* waypoints
    WAYPOINT_TOLERANCE = 0.3   # m — distance to accept waypoint arrival
    NAV_LINEAR_SPEED   = 0.25  # m/s — forward speed during waypoint following
    OBSTACLE_THRESHOLD = 0.5   # m — depth distance to trigger avoidance

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
        self.last_map_publish  = 0.0

        # Waypoint navigation state
        self.nav_waypoints     = []    # list of (gx, gy)
        self.nav_waypoint_idx  = 0

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image,      '/camera/depth/image_raw',
                                 self._depth_cb,       be)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info',
                                 self._camera_info_cb, be)
        # Publishers
        self.map_pub      = self.create_publisher(OccupancyGrid, '/map',              10)
        self.frontier_pub = self.create_publisher(MarkerArray,   '/frontier_markers', 10)
        self.cmd_vel_pub  = self.create_publisher(Twist,         '/cmd_vel',          10)

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
        """Store the latest depth frame and integrate it into the map.

        Depth is integrated in all states (including NAVIGATING) so the
        map stays current as the robot drives.  At 0.25 m/s and ~30 Hz
        depth, positional error is ~8 mm/frame — well within the 5 cm
        grid resolution.
        """
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
        # Compose two independent lookups instead of one chained lookup:
        #
        #   odom → base_link           stamped to depth image timestamp
        #   base_link → camera_optical  latest (effectively static: pan=tilt=0)
        #
        # This avoids the fallback race condition where a full chained lookup
        # of odom→camera_depth_optical_frame falls through to rclpy.time.Time()
        # (latest TF) because robot_state_publisher's joint TFs don't have an
        # entry at exactly the image timestamp.  During rotation, the "latest"
        # odom→base_link in the buffer is from a publish_callback that ran
        # AFTER the image was rendered, placing obstacles at the wrong angle.
        #
        # With this split: odom→base_link is stamped (zero-lag, bridge synced),
        # and base_link→camera_optical only changes when pan/tilt move so its
        # latest-time lookup is always correct for exploration.
        try:
            stamp  = rclpy.time.Time.from_msg(self.latest_depth_stamp)
            tf_ob  = self.tf_buffer.lookup_transform(
                'odom', 'base_link', stamp, timeout=Duration(seconds=0.1))
        except Exception:
            # Startup: synced TF not yet in buffer — accept a small lag.
            try:
                tf_ob = self.tf_buffer.lookup_transform(
                    'odom', 'base_link',
                    rclpy.time.Time(), timeout=Duration(seconds=0.05))
            except Exception:
                return

        try:
            tf_bc = self.tf_buffer.lookup_transform(
                'base_link', 'camera_depth_optical_frame',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return

        def _q2m(q):
            """Quaternion ROS msg → 3×3 rotation matrix (frame → parent)."""
            qw, qx, qy, qz = q.w, q.x, q.y, q.z
            return np.array([
                [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)],
            ])

        R_ob = _q2m(tf_ob.transform.rotation)
        R_bc = _q2m(tf_bc.transform.rotation)
        to   = tf_ob.transform.translation
        tc   = tf_bc.transform.translation

        # Composed odom→camera: t_oc = R_ob @ t_bc + t_ob,  R_oc = R_ob @ R_bc
        cam_origin = R_ob @ np.array([tc.x, tc.y, tc.z]) + np.array([to.x, to.y, to.z])
        R          = R_ob @ R_bc

        # ── 2. Back-project depth pixels to odom frame ───────────────────────
        fx = self.camera_K[0, 0];  fy = self.camera_K[1, 1]
        cx = self.camera_K[0, 2];  cy = self.camera_K[1, 2]

        depth = self.latest_depth
        h, w  = depth.shape
        step  = self.DEPTH_SUBSAMPLE

        # Crop horizontal edges to reduce rotational smearing — edge pixels
        # have larger angular offsets and any TF timing lag produces arcs
        # instead of straight lines in the occupancy grid.
        margin = int(w * self.DEPTH_H_CROP)
        vs, us    = np.mgrid[0:h:step, margin:w - margin:step]
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

        cam_gx, cam_gy = self.world_to_grid(cam_origin[0], cam_origin[1])

        # ── 4b. Mark robot footprint as free ──────────────────────────────────
        # The depth camera looks outward and cannot see the floor directly
        # beneath the robot.  Those cells never receive MISS updates so they
        # stay UNKNOWN (log_odds = 0).  The frontier detector then scores the
        # robot's own position as a high-value frontier (close + large cluster
        # of adjacent unknown cells), causing the robot to navigate back to
        # where it started.  Clearing the footprint on every depth frame
        # prevents this false frontier from forming.
        r_cells = int(self.ROBOT_BODY_RADIUS / self.GRID_RESOLUTION) + 1
        fy_min  = max(0, cam_gy - r_cells)
        fy_max  = min(self.GRID_SIZE, cam_gy + r_cells + 1)
        fx_min  = max(0, cam_gx - r_cells)
        fx_max  = min(self.GRID_SIZE, cam_gx + r_cells + 1)
        ffy, ffx = np.meshgrid(
            np.arange(fy_min, fy_max), np.arange(fx_min, fx_max), indexing='ij')
        in_fp   = ((ffy - cam_gy)**2 + (ffx - cam_gx)**2) <= r_cells ** 2
        fp_gy   = ffy[in_fp];  fp_gx = ffx[in_fp]
        # Don't clear cells already confirmed as real obstacles (hit_count ≥ 2)
        clearable = self.hit_count[fp_gy, fp_gx] < self.MIN_HIT_COUNT
        self.log_odds[fp_gy[clearable], fp_gx[clearable]] = np.maximum(
            self.LOG_ODDS_MIN,
            self.log_odds[fp_gy[clearable], fp_gx[clearable]] + self.LOG_ODDS_MISS)

        if len(obs_idx) == 0 and len(floor_idx) == 0:
            return

        # ── 5a. Free-space raytrace: obstacle returns ─────────────────────────
        if len(obs_idx) > 0:
            ray_step = max(1, len(obs_idx) // 200)
            for i in obs_idx[::ray_step]:
                self._raytrace_free(cam_gx, cam_gy, int(gxs[i]), int(gys[i]))

        # ── 5b. Free-space raytrace: floor/ceiling returns ────────────────────
        # These rays mark empty space in directions where no obstacle-height
        # return was seen (e.g. the robot faces an open corridor).  They do NOT
        # add any HIT, so they cannot create false obstacles.
        if len(floor_idx) > 0:
            ray_step2 = max(1, len(floor_idx) // 100)
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

    # ── Path planning ──────────────────────────────────────────────────────────

    def _plan_path(self, start_gx, start_gy, goal_gx, goal_gy, clearance_m=None):
        """A* on the inflated occupancy grid.

        Returns a list of (gx, gy) waypoints subsampled every WAYPOINT_SPACING
        cells, or None if no path exists.

        *clearance_m* overrides ROBOT_CLEARANCE for this call (used for
        reduced-inflation fallback when full inflation blocks all paths).
        """
        if clearance_m is None:
            clearance_m = self.ROBOT_CLEARANCE
        free_mask = self.log_odds <= self.LOG_ODDS_FREE_THRESH
        confirmed_occ = ((self.log_odds  >= self.LOG_ODDS_OCC_THRESH) &
                         (self.hit_count >= self.MIN_HIT_COUNT))
        if clearance_m > 0:
            r_clear = int(np.ceil(clearance_m / self.GRID_RESOLUTION))
            ky_c, kx_c = np.ogrid[-r_clear:r_clear + 1, -r_clear:r_clear + 1]
            clear_disc = (ky_c ** 2 + kx_c ** 2) <= r_clear ** 2
            inflated = binary_dilation(confirmed_occ, structure=clear_disc)
            traversable = free_mask & ~inflated
        else:
            traversable = free_mask & ~confirmed_occ

        # Ensure start and goal are in-bounds
        if not (self.in_grid(start_gx, start_gy) and
                self.in_grid(goal_gx, goal_gy)):
            return None

        # If start is inside inflated zone (robot got pushed into obstacle
        # proximity), relax to free_mask for the start cell only
        if not traversable[start_gy, start_gx]:
            if not free_mask[start_gy, start_gx]:
                return None
            traversable[start_gy, start_gx] = True

        if not traversable[goal_gy, goal_gx]:
            return None

        SQRT2 = 1.414
        DIRS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]

        def heuristic(gx, gy):
            return np.hypot(gx - goal_gx, gy - goal_gy)

        start = (start_gx, start_gy)
        goal  = (goal_gx, goal_gy)

        open_set = [(heuristic(*start), 0.0, start)]
        g_cost   = {start: 0.0}
        came_from = {}
        expansions = 0
        MAX_EXPANSIONS = 50000

        while open_set and expansions < MAX_EXPANSIONS:
            _, g, (cx, cy) = heapq.heappop(open_set)
            expansions += 1

            if (cx, cy) == goal:
                # Reconstruct path
                path = [(cx, cy)]
                while (cx, cy) in came_from:
                    cx, cy = came_from[(cx, cy)]
                    path.append((cx, cy))
                path.reverse()
                return self._subsample_path(path)

            if g > g_cost.get((cx, cy), float('inf')):
                continue

            for dx, dy, cost in DIRS:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < self.GRID_SIZE and 0 <= ny < self.GRID_SIZE):
                    continue
                if not traversable[ny, nx]:
                    continue
                ng = g + cost
                if ng < g_cost.get((nx, ny), float('inf')):
                    g_cost[(nx, ny)] = ng
                    came_from[(nx, ny)] = (cx, cy)
                    heapq.heappush(open_set, (ng + heuristic(nx, ny), ng, (nx, ny)))

        return None  # no path found within expansion budget

    def _subsample_path(self, path):
        """Extract waypoints every WAYPOINT_SPACING cells, skipping start cell."""
        if len(path) <= 1:
            return path
        # Skip path[0] (robot's current position) — start from first intermediate
        waypoints = []
        for i in range(self.WAYPOINT_SPACING, len(path) - 1, self.WAYPOINT_SPACING):
            waypoints.append(path[i])
        waypoints.append(path[-1])  # always include goal
        return waypoints

    def _compute_inflated(self, clearance_m=None):
        """Compute an inflated obstacle mask for the given clearance."""
        if clearance_m is None:
            clearance_m = self.ROBOT_CLEARANCE
        confirmed_occ = ((self.log_odds  >= self.LOG_ODDS_OCC_THRESH) &
                         (self.hit_count >= self.MIN_HIT_COUNT))
        if clearance_m > 0:
            r = int(np.ceil(clearance_m / self.GRID_RESOLUTION))
            ky, kx = np.ogrid[-r:r + 1, -r:r + 1]
            disc = (ky ** 2 + kx ** 2) <= r ** 2
            return binary_dilation(confirmed_occ, structure=disc)
        return confirmed_occ

    def _is_path_blocked(self):
        """Check remaining waypoints against obstacles at the nav clearance."""
        nav_clear = getattr(self, 'nav_clearance', None)
        inflated = self._compute_inflated(nav_clear)
        for i in range(self.nav_waypoint_idx, len(self.nav_waypoints)):
            gx, gy = self.nav_waypoints[i]
            if self.in_grid(gx, gy) and inflated[gy, gx]:
                return True
        return False

    def _replan_from_here(self):
        """Re-plan A* from current position to the final waypoint goal.

        Uses the same clearance that produced the original path.
        Returns True if a new path was found and waypoints were updated.
        """
        pose = self.get_base_pose()
        if pose is None or len(self.nav_waypoints) == 0:
            return False
        bx, by, _ = pose
        robot_gx, robot_gy = self.world_to_grid(bx, by)
        goal_gx, goal_gy = self.nav_waypoints[-1]

        nav_clear = getattr(self, 'nav_clearance', None)
        path = self._plan_path(robot_gx, robot_gy, goal_gx, goal_gy,
                               clearance_m=nav_clear)
        if path is None:
            return False

        self.nav_waypoints = path
        self.nav_waypoint_idx = 0
        self.get_logger().info(
            f'[NAV] Re-planned: {len(path)} waypoints from '
            f'({bx:.2f},{by:.2f})')
        return True

    # ── Depth obstacle avoidance ─────────────────────────────────────────────

    def _analyze_depth_obstacles(self):
        """Analyze depth image for obstacles in left/center/right sectors.

        Returns (clearance, distances) where each is (left, center, right).
        clearance[i] is True if the sector is clear of obstacles.
        distances[i] is the 10th-percentile depth in metres.
        """
        depth = self.latest_depth
        if depth is None:
            return (True, True, True), (999.0, 999.0, 999.0)

        h, w = depth.shape
        # Ignore top 30% (ceiling) and bottom 20% (ground)
        v_start = int(h * 0.3)
        v_end   = int(h * 0.8)
        # Crop horizontal edges (same as mapping FOV)
        margin = int(w * self.DEPTH_H_CROP)
        u_start = margin
        u_end   = w - margin
        third   = (u_end - u_start) // 3

        left_region   = depth[v_start:v_end, u_start:u_start + third]
        center_region = depth[v_start:v_end, u_start + third:u_start + 2 * third]
        right_region  = depth[v_start:v_end, u_start + 2 * third:u_end]

        def sector_min_dist(region):
            valid = region[region > 0].astype(np.float64)
            if len(valid) == 0:
                return 999.0
            return float(np.percentile(valid, 10)) / 1000.0  # mm → m

        left_dist   = sector_min_dist(left_region)
        center_dist = sector_min_dist(center_region)
        right_dist  = sector_min_dist(right_region)

        threshold = self.OBSTACLE_THRESHOLD
        clearance = (left_dist > threshold,
                     center_dist > threshold,
                     right_dist > threshold)
        return clearance, (left_dist, center_dist, right_dist)

    @staticmethod
    def _normalize_angle(angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    # ── Frontier detection ────────────────────────────────────────────────────

    def find_frontiers(self):
        """
        Return frontier clusters sorted by information-gain score.

        A frontier cell is FREE (log_odds ≤ FREE_THRESH) and 4-adjacent to
        at least one UNKNOWN cell.  Connected components (8-connected) smaller
        than MIN_FRONTIER_SIZE are discarded.

        Clusters are additionally filtered by:
          • MIN_FRONTIER_DIST  — centroid must be > MIN_FRONTIER_DIST from the robot.
          • Reachability       — at least one cell in the cluster must be in the
                                 same free-space connected component as the robot
                                 (scipy label flood-fill).
          • Centroid snapping  — if the raw centroid (mean position) falls on an
                                 invalid cell (unknown, inflated-obstacle, or
                                 unreachable), it is snapped to the nearest valid
                                 cluster cell.  If no valid cell exists, the
                                 cluster is discarded.

        Score = unknown_cells_within_MAX_DEPTH_M / distance

        Counts every UNKNOWN cell within the sensor range disc centred on the
        frontier — an estimate of information gain — then divides by travel
        distance to balance exploration value against travel cost.
        """
        free_mask    = self.log_odds <= self.LOG_ODDS_FREE_THRESH
        # Use the same confirmed-obstacle definition as publish_map so that
        # smeared cells (hit_count < MIN_HIT_COUNT) count as UNKNOWN, not walls.
        confirmed_occ = ((self.log_odds  >= self.LOG_ODDS_OCC_THRESH) &
                         (self.hit_count >= self.MIN_HIT_COUNT))
        unknown_mask  = ~free_mask & ~confirmed_occ

        # Inflate confirmed obstacles by ROBOT_CLEARANCE.  Frontier centroids
        # and nav goals inside this inflated zone are within the robot's body
        # radius of a wall and unsafe to navigate to.  Cache the mask so that
        # _state_select() can also validate the final goal position.
        r_clear = int(np.ceil(self.ROBOT_CLEARANCE / self.GRID_RESOLUTION))
        ky_c, kx_c = np.ogrid[-r_clear:r_clear + 1, -r_clear:r_clear + 1]
        clear_disc = (ky_c ** 2 + kx_c ** 2) <= r_clear ** 2
        self._inflated_occ = binary_dilation(confirmed_occ, structure=clear_disc)

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
        robot_gx, robot_gy = self.world_to_grid(bx, by)

        # ── Reachability: connected-component labeling on free cells ─────────
        # Flood-fill on uninflated free space to check basic connectivity.
        # Inflation (robot width) is enforced by A* path planning, not here.
        _labeled, _ = ndimage_label(free_mask, structure=np.ones((3, 3), dtype=bool))
        if (self.in_grid(robot_gx, robot_gy) and
                _labeled[robot_gy, robot_gx] > 0):
            reachable = (_labeled == _labeled[robot_gy, robot_gx])
        else:
            reachable = None

        # Pre-build a boolean disc mask of radius MAX_DEPTH_M in grid cells.
        # Reused for every cluster so we only allocate it once.
        r_cells = int(self.MAX_DEPTH_M / self.GRID_RESOLUTION)
        side    = 2 * r_cells + 1
        ky, kx  = np.ogrid[-r_cells:r_cells + 1, -r_cells:r_cells + 1]
        disc    = (ky ** 2 + kx ** 2) <= r_cells ** 2   # (side × side) bool

        results = []
        n_dist = n_reach = n_clear = 0   # diagnostic counters

        for cluster in clusters:
            cluster_cells = np.array(cluster)          # (N, 2) — columns: gx, gy
            cxs, cys = cluster_cells[:, 0], cluster_cells[:, 1]

            # ── 1. Distance filter (coarse) ────────────────────────────────
            mgx = int(np.mean(cxs))
            mgy = int(np.mean(cys))
            wx, wy = self.grid_to_world(mgx, mgy)
            dist   = np.hypot(wx - bx, wy - by)

            if dist < self.MIN_FRONTIER_DIST:
                n_dist += 1
                continue

            # ── 2. Reachability: same free-space component as robot ────
            # Frontier cells are free by definition, so checking them
            # directly against the uninflated reachable set works.
            # A* enforces the inflation (robot-width) constraint later.
            if reachable is not None:
                if not np.any(reachable[cys, cxs]):
                    n_reach += 1
                    continue

            # ── 3. Snap centroid to nearest cluster cell outside inflation ──
            # The raw centroid (mean position) often lands on an UNKNOWN or
            # inflated-obstacle cell because frontier cells border unknown
            # space.  Snap to the nearest cluster cell outside the inflated
            # zone so A* can accept it as a goal.
            centroid_valid = (
                self.in_grid(mgx, mgy) and
                not self._inflated_occ[mgy, mgx])

            if not centroid_valid:
                valid = ~self._inflated_occ[cys, cxs]
                if not np.any(valid):
                    n_clear += 1
                    continue
                valid_idx = np.where(valid)[0]
                dists2 = (cxs[valid_idx] - mgx) ** 2 + (cys[valid_idx] - mgy) ** 2
                best = valid_idx[np.argmin(dists2)]
                mgx, mgy = int(cxs[best]), int(cys[best])
                wx, wy = self.grid_to_world(mgx, mgy)
                dist   = np.hypot(wx - bx, wy - by)

            # ── 4. Information-gain score ──────────────────────────────────
            y_lo = max(0,             mgy - r_cells)
            y_hi = min(self.GRID_SIZE, mgy + r_cells + 1)
            x_lo = max(0,             mgx - r_cells)
            x_hi = min(self.GRID_SIZE, mgx + r_cells + 1)

            ky_lo = y_lo - (mgy - r_cells)
            ky_hi = ky_lo + (y_hi - y_lo)
            kx_lo = x_lo - (mgx - r_cells)
            kx_hi = kx_lo + (x_hi - x_lo)

            patch = unknown_mask[y_lo:y_hi, x_lo:x_hi]
            nearby_unknown = int(np.sum(patch & disc[ky_lo:ky_hi, kx_lo:kx_hi]))

            score = nearby_unknown / max(dist, 0.5)
            results.append((wx, wy, len(cluster), score))

        self.get_logger().info(
            f'[FRONTIER] {len(clusters)} clusters → {len(results)} valid '
            f'(filtered: dist={n_dist}, reach={n_reach}, clear={n_clear})')

        results.sort(key=lambda r: r[3], reverse=True)
        return results

    # ── Publishing ────────────────────────────────────────────────────────────

    def publish_map(self):
        # ── Continuous grayscale occupancy ────────────────────────────────────
        # Map log_odds linearly to [0, 100]:
        #   LOG_ODDS_MIN (−10) → 0   (pure white  — strongly free)
        #   log_odds = 0        → 50  (mid-gray    — neutral / uncertain)
        #   LOG_ODDS_MAX (+10) → 100  (pure black  — strongly occupied)
        #
        # Saturation now requires ~12 net HITs (10 / 0.85) or ~25 net MISSes,
        # giving a gradual buildup rather than snapping after just 2 hits.
        span  = float(self.LOG_ODDS_MAX - self.LOG_ODDS_MIN)
        ratio = (self.log_odds - self.LOG_ODDS_MIN) / span
        occ   = np.clip(ratio * 100.0, 0.0, 100.0)

        # Cells not yet confirmed from 2+ distinct angles (hit_count < 2) are
        # capped at 80 so fully-confirmed obstacles remain visually distinct
        # from single-direction detections (e.g. smear artifacts).
        occ[self.hit_count < self.MIN_HIT_COUNT] = np.minimum(
            occ[self.hit_count < self.MIN_HIT_COUNT], 80.0)

        grid = occ.astype(np.int8)

        # Cells that have never been updated (no HIT, log_odds still exactly 0)
        # are published as −1 (unknown / gray in RViz).
        grid[(self.hit_count == 0) & (self.log_odds == 0.0)] = -1

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
                # No frontiers visible — a 360° scan is warranted to
                # discover new unknown boundaries.
                self.get_logger().info(
                    f'[SELECT] No frontiers '
                    f'({self.no_frontier_count}/{self.NO_FRONTIER_LIMIT}) '
                    f'— re-scanning.')
                self._start_scan()
            return

        self.no_frontier_count = 0

        base_pose = self.get_base_pose()
        if base_pose is None:
            return  # retry next tick
        bx, by, _ = base_pose
        robot_gx, robot_gy = self.world_to_grid(bx, by)

        # Try each frontier with progressively reduced inflation.
        # Full clearance first (safest), then half, then zero (rely on
        # depth obstacle avoidance for real-time safety).
        clearances = [None, self.ROBOT_CLEARANCE * 0.5, 0.0]
        labels     = ['full', 'half', 'zero']

        for clearance, label in zip(clearances, labels):
            for wx, wy, size, score in frontiers:
                if clearance is None:
                    self.get_logger().info(
                        f'[SELECT] Trying frontier ({wx:.2f}, {wy:.2f}) '
                        f'cluster={size} score={score:.0f}')

                goal_gx, goal_gy = self.world_to_grid(wx, wy)
                path = self._plan_path(robot_gx, robot_gy, goal_gx, goal_gy,
                                       clearance_m=clearance)

                if path is None:
                    continue

                self.get_logger().info(
                    f'[SELECT] A* path ({label} inflation): {len(path)} '
                    f'waypoints to ({wx:.2f}, {wy:.2f})')

                self.nav_waypoints    = path
                self.nav_waypoint_idx = 0
                self.nav_start_time   = time.time()
                self.nav_clearance    = clearance  # for re-plan consistency
                self.state = ExploreState.NAVIGATING
                return

        # Truly no path at any inflation level — scan for new data
        self.get_logger().warn(
            '[SELECT] No path to any frontier at any clearance — re-scanning.')
        self._start_scan()

    def _state_navigating(self):
        """Follow A* waypoints via cmd_vel with map-based re-planning."""
        now = time.time()

        # ── Safety timeout ─────────────────────────────────────────────
        if now - self.nav_start_time > self.NAV_TIMEOUT:
            self._stop_base()
            self.get_logger().warn('[NAV] Timeout — re-selecting.')
            self.state = ExploreState.SELECTING
            return

        # ── Get current pose ───────────────────────────────────────────
        pose = self.get_base_pose()
        if pose is None:
            return
        bx, by, btheta = pose
        actual_heading = btheta - np.pi / 2  # MuJoCo → world heading

        # ── Check if path is blocked on the mapped obstacles ───────────
        if self._is_path_blocked():
            self.get_logger().info('[NAV] Path blocked on map — re-planning...')
            if self._replan_from_here():
                return  # re-planned successfully, follow new path next tick
            else:
                self._stop_base()
                self.get_logger().warn(
                    '[NAV] Re-plan failed — re-selecting.')
                self.state = ExploreState.SELECTING
                return

        # ── Current waypoint ───────────────────────────────────────────
        if self.nav_waypoint_idx >= len(self.nav_waypoints):
            self._stop_base()
            self.get_logger().info('[NAV] All waypoints reached — starting scan.')
            self._start_scan()
            return

        wp_gx, wp_gy = self.nav_waypoints[self.nav_waypoint_idx]
        wp_wx, wp_wy = self.grid_to_world(wp_gx, wp_gy)
        dx, dy = wp_wx - bx, wp_wy - by
        dist = np.hypot(dx, dy)

        # Advance waypoint if close enough
        if dist < self.WAYPOINT_TOLERANCE:
            self.nav_waypoint_idx += 1
            self.nav_start_time = time.time()  # reset per-waypoint timeout
            if self.nav_waypoint_idx >= len(self.nav_waypoints):
                self._stop_base()
                self.get_logger().info('[NAV] Goal reached — starting scan.')
                self._start_scan()
                return
            wp_gx, wp_gy = self.nav_waypoints[self.nav_waypoint_idx]
            wp_wx, wp_wy = self.grid_to_world(wp_gx, wp_gy)
            dx, dy = wp_wx - bx, wp_wy - by
            dist = np.hypot(dx, dy)

        desired_heading = np.arctan2(dy, dx)
        heading_error = self._normalize_angle(desired_heading - actual_heading)

        # ── Periodic status log (every ~2 s) ──────────────────────────
        if not hasattr(self, '_nav_last_log') or now - self._nav_last_log > 2.0:
            self._nav_last_log = now
            self.get_logger().info(
                f'[NAV] wp {self.nav_waypoint_idx}/{len(self.nav_waypoints)} '
                f'dist={dist:.2f} hdg_err={np.degrees(heading_error):.0f}° '
                f'pos=({bx:.2f},{by:.2f})')

        cmd = Twist()

        if abs(heading_error) > 0.6:
            # ── Large heading error: rotate in place ──────────────────
            # No forward motion → no collision risk → skip depth avoidance.
            # This prevents obstacle avoidance from fighting the needed turn.
            cmd.angular.z = float(np.clip(2.0 * heading_error, -1.0, 1.0))
        else:
            # ── Heading roughly aligned: drive forward with avoidance ─
            clearance, distances = self._analyze_depth_obstacles()
            left_clear, center_clear, right_clear = clearance
            left_dist, center_dist, right_dist = distances

            if center_clear:
                cmd.linear.x = float(min(self.NAV_LINEAR_SPEED, dist * 0.5))
                cmd.angular.z = float(np.clip(1.5 * heading_error, -0.8, 0.8))
            else:
                self.get_logger().info(
                    f'[NAV] Depth obstacle: L={left_dist:.2f} C={center_dist:.2f} '
                    f'R={right_dist:.2f} — steering around')
                if left_dist > right_dist:
                    cmd.angular.z = 0.5
                    cmd.linear.x = 0.05
                elif right_dist > left_dist:
                    cmd.angular.z = -0.5
                    cmd.linear.x = 0.05
                else:
                    cmd.angular.z = 0.6

            # Safety: stop forward if very close to anything
            if min(left_dist, center_dist, right_dist) < 0.25:
                cmd.linear.x = 0.0

        self.cmd_vel_pub.publish(cmd)

    def _state_complete(self):
        self._stop_base()
        self.publish_map()

        free     = int(np.sum(self.log_odds <= self.LOG_ODDS_FREE_THRESH))
        occupied = int(np.sum(
            (self.log_odds  >= self.LOG_ODDS_OCC_THRESH) &
            (self.hit_count >= self.MIN_HIT_COUNT)))
        total    = self.GRID_SIZE ** 2

        self.get_logger().info('=' * 55)
        self.get_logger().info('EXPLORATION COMPLETE')
        self.get_logger().info(f'  Coverage : {(free+occupied)/total*100:.1f}%')
        self.get_logger().info(f'  Free     : {free}')
        self.get_logger().info(f'  Confirmed occupied : {occupied}')
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
