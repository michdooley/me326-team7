#!/usr/bin/env python3
"""
TidyBot2 Frontier Explorer

Builds a 2D log-odds occupancy grid from the depth camera, then
autonomously explores by repeating:

  1. SCANNING   — rotates the base 360° in place while continuously
                  updating the map from every depth frame
  2. SELECTING  — finds frontier cells (FREE↔UNKNOWN boundary), clusters
                  them, and navigates to the highest-scoring cluster
                  (score = cluster_size / distance_to_robot)
  3. NAVIGATING — drives to the chosen goal; on arrival (or timeout)
                  returns to step 1 at the new position

The log-odds grid is key to correctness:
  • Every depth ray clears FREE belief along its entire path, so a
    noisy OCCUPIED reading can be overwritten by later free observations.
  • Only points whose world-z falls in [MIN_OBSTACLE_Z, MAX_OBSTACLE_Z]
    vote for OCCUPIED — the floor (z≈0) is excluded without needing a
    full 3-D voxel representation.

Publishes:
  /map               nav_msgs/OccupancyGrid   — standard RViz map
  /map_image         sensor_msgs/Image        — RGB debug view
  /frontier_markers  visualization_msgs/MarkerArray

Subscribes:
  /camera/depth/image_raw    — depth frames
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
    SCANNING   = 0   # rotating 360° in place, building the map
    SELECTING  = 1   # choosing the best frontier to drive toward
    NAVIGATING = 2   # driving to the selected frontier
    COMPLETE   = 3   # no frontiers remain


class FrontierExplorer(Node):
    """Log-odds occupancy mapper with 360° scan + frontier-based navigation."""

    # ── Grid ─────────────────────────────────────────────────────────────────
    GRID_RESOLUTION = 0.05   # m/cell
    GRID_SIZE       = 300    # cells per axis  (15 m × 15 m)
    GRID_ORIGIN_X   = -7.5   # world X at grid column 0 (m)
    GRID_ORIGIN_Y   = -7.5   # world Y at grid row    0 (m)

    # ── Depth processing ─────────────────────────────────────────────────────
    MIN_DEPTH_M     = 0.30   # ignore returns closer than this (robot body)
    MAX_DEPTH_M     = 5.00   # ignore returns farther than this
    DEPTH_SUBSAMPLE = 4      # process every Nth pixel

    # ── Height filter ─────────────────────────────────────────────────────────
    # Only 3-D points whose odom-Z is in this band vote for OCCUPIED.
    # Floor (z≈0) and overhead surfaces are excluded without a voxel grid.
    MIN_OBSTACLE_Z = 0.10    # m — 10 cm clears floor noise
    MAX_OBSTACLE_Z = 0.80    # m — ignore camera mounts / ceilings

    # ── Log-odds belief (Thrun, Burgard & Fox Ch. 9) ─────────────────────────
    LOG_ODDS_HIT         =  0.85
    LOG_ODDS_MISS        = -0.40
    LOG_ODDS_MIN         = -3.00   # very confident FREE   (P ≈ 0.05)
    LOG_ODDS_MAX         =  3.00   # very confident OCCUPIED (P ≈ 0.95)
    LOG_ODDS_FREE_THRESH = -0.50   # below → publish  0 (FREE)
    LOG_ODDS_OCC_THRESH  =  1.50   # above → publish 100 (OCCUPIED)
    # between the two   → publish -1 (UNKNOWN)

    # ── Map publishing ────────────────────────────────────────────────────────
    MAP_PUBLISH_RATE = 2.0   # Hz

    # ── 360° scan ─────────────────────────────────────────────────────────────
    SCAN_ANGULAR_VEL = 1.5   # rad/s — ~4 s per 360° in sim
    SCAN_SETTLE_TIME = 1.0   # seconds to wait (motors settle) after stopping

    # ── Frontier selection ────────────────────────────────────────────────────
    MIN_FRONTIER_SIZE   = 8    # cells — ignore tiny clusters
    FRONTIER_NAV_OFFSET = 0.4  # m — goal set this far in front of centroid

    # ── Navigation ────────────────────────────────────────────────────────────
    NAV_TIMEOUT       = 30.0   # s before giving up on a navigation goal
    NO_FRONTIER_LIMIT = 3      # consecutive empty scans → COMPLETE

    # ─────────────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('frontier_explorer')

        # Log-odds grid — float32, 0.0 = maximum uncertainty
        self.log_odds = np.zeros(
            (self.GRID_SIZE, self.GRID_SIZE), dtype=np.float32)

        # Camera data
        self.latest_depth: np.ndarray | None = None
        self.latest_depth_stamp = None   # ROS timestamp of the stored depth frame
        self.camera_K:     np.ndarray | None = None
        self.cv_bridge = CvBridge()

        # State machine
        self.state             = ExploreState.SCANNING
        self.scan_accumulated  = 0.0    # radians rotated so far this scan
        self.scan_last_heading = None   # previous heading sample (rad)
        self.scan_settle_start = None   # time.time() when settle began
        self.nav_start_time    = None
        self.no_frontier_count = 0
        self.base_goal_reached = False

        self.last_map_publish = 0.0

        # TF2
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Image,      '/camera/depth/image_raw',   self._depth_cb,        qos)
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._camera_info_cb,  qos)
        self.create_subscription(
            Bool,       '/base/goal_reached',        self._goal_reached_cb, 10)

        # Publishers
        self.map_pub       = self.create_publisher(OccupancyGrid, '/map',              10)
        self.map_image_pub = self.create_publisher(Image,         '/map_image',        10)
        self.frontier_pub  = self.create_publisher(MarkerArray,   '/frontier_markers', 10)
        self.cmd_vel_pub   = self.create_publisher(Twist,         '/cmd_vel',          10)
        self.base_goal_pub = self.create_publisher(Pose2D,        '/base/target_pose', 10)

        self.get_logger().info('=' * 55)
        self.get_logger().info('Frontier Explorer initialised')
        self.get_logger().info(
            f'  Grid        : {self.GRID_SIZE}×{self.GRID_SIZE} '
            f'@ {self.GRID_RESOLUTION} m/cell '
            f'({self.GRID_SIZE * self.GRID_RESOLUTION:.0f} m²)')
        self.get_logger().info(
            f'  Height band : {self.MIN_OBSTACLE_Z}–{self.MAX_OBSTACLE_Z} m '
            f'(floor excluded)')
        self.get_logger().info(
            f'  Scan speed  : {self.SCAN_ANGULAR_VEL} rad/s '
            f'(≈{2*np.pi/self.SCAN_ANGULAR_VEL:.0f} s per 360°)')
        self.get_logger().info('=' * 55)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _depth_cb(self, msg: Image):
        try:
            depth = self.cv_bridge.imgmsg_to_cv2(msg, '16UC1')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')
            return
        self.latest_depth = depth
        self.latest_depth_stamp = msg.header.stamp
        # Integrate immediately while the callback is live — "latest" TF is
        # within ~1 ms of capture here, eliminating smear and frame gaps.
        if self.camera_K is not None:
            self.update_map()

    def _camera_info_cb(self, msg: CameraInfo):
        self.camera_K = np.array(msg.k).reshape(3, 3)

    def _goal_reached_cb(self, msg: Bool):
        if msg.data:
            self.base_goal_reached = True

    # ── Grid helpers ──────────────────────────────────────────────────────────

    def world_to_grid(self, wx: float, wy: float):
        gx = int((wx - self.GRID_ORIGIN_X) / self.GRID_RESOLUTION)
        gy = int((wy - self.GRID_ORIGIN_Y) / self.GRID_RESOLUTION)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int):
        wx = self.GRID_ORIGIN_X + (gx + 0.5) * self.GRID_RESOLUTION
        wy = self.GRID_ORIGIN_Y + (gy + 0.5) * self.GRID_RESOLUTION
        return wx, wy

    def in_grid(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.GRID_SIZE and 0 <= gy < self.GRID_SIZE

    # ── Robot pose ────────────────────────────────────────────────────────────

    def get_base_pose(self):
        """Return (x, y, theta) of base_link in odom, or None on failure."""
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
        """Return current yaw of base_link in odom (radians), or None."""
        pose = self.get_base_pose()
        return pose[2] if pose is not None else None

    # ── Mapping ───────────────────────────────────────────────────────────────

    def update_map(self):
        """Project the current depth frame into the log-odds occupancy grid.

        Called directly from _depth_cb so that the TF lookup happens within
        ~1 ms of frame capture.  At 1.5 rad/s that is <0.1° of rotational
        error — far below what produces visible smearing.
        """
        if self.latest_depth is None or self.camera_K is None:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', 'camera_depth_optical_frame',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        cam_origin = np.array([t.x, t.y, t.z])

        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
        ])

        fx = self.camera_K[0, 0];  fy = self.camera_K[1, 1]
        cx = self.camera_K[0, 2];  cy = self.camera_K[1, 2]

        depth = self.latest_depth
        h, w  = depth.shape
        step  = self.DEPTH_SUBSAMPLE

        # Subsample pixels
        vs, us    = np.mgrid[0:h:step, 0:w:step]
        us        = us.ravel();  vs = vs.ravel()
        depths_mm = depth[vs, us].astype(np.float64)

        min_mm = self.MIN_DEPTH_M * 1000.0
        max_mm = self.MAX_DEPTH_M * 1000.0
        valid  = (depths_mm > min_mm) & (depths_mm < max_mm)
        us, vs   = us[valid], vs[valid]
        depths_m = depths_mm[valid] / 1000.0

        if len(depths_m) == 0:
            return

        # Back-project to odom frame
        x_cam = (us - cx) * depths_m / fx
        y_cam = (vs - cy) * depths_m / fy
        z_cam = depths_m
        pts_odom = (R @ np.stack([x_cam, y_cam, z_cam], axis=1).T).T + cam_origin

        gxs = ((pts_odom[:, 0] - self.GRID_ORIGIN_X) / self.GRID_RESOLUTION).astype(int)
        gys = ((pts_odom[:, 1] - self.GRID_ORIGIN_Y) / self.GRID_RESOLUTION).astype(int)
        in_bounds = ((gxs >= 0) & (gxs < self.GRID_SIZE) &
                     (gys >= 0) & (gys < self.GRID_SIZE))

        cam_gx, cam_gy = self.world_to_grid(cam_origin[0], cam_origin[1])

        # ── Free-space raytrace for ALL valid returns ─────────────────────────
        # Floor returns (outside the height band) still contribute free-space
        # evidence along the ray — this is what prevents the area in front of
        # the robot from staying UNKNOWN when the camera tilts downward.
        ray_idx  = np.where(in_bounds)[0]
        ray_step = max(1, len(ray_idx) // 500)
        for i in ray_idx[::ray_step]:
            self._raytrace_free(cam_gx, cam_gy, int(gxs[i]), int(gys[i]))

        # ── OCCUPIED update — height-filtered points only ─────────────────────
        obs_mask = (
            (pts_odom[:, 2] > self.MIN_OBSTACLE_Z) &
            (pts_odom[:, 2] < self.MAX_OBSTACLE_Z) &
            in_bounds
        )
        np.add.at(self.log_odds, (gys[obs_mask], gxs[obs_mask]), self.LOG_ODDS_HIT)
        np.clip(self.log_odds, self.LOG_ODDS_MIN, self.LOG_ODDS_MAX, out=self.log_odds)

    def _raytrace_free(self, x0: int, y0: int, x1: int, y1: int):
        """Bresenham's line: apply LOG_ODDS_MISS to every cell from (x0,y0)
        up to (but not including) the endpoint (x1,y1).

        MISS is applied to ALL cells along the ray, including cells already
        above OCC_THRESH.  This is essential for log-odds self-correction:
        small TF timing errors cause walls to be painted ~1 cell off their
        true position; subsequent rays from new angles must be able to apply
        MISS to those stale cells so they decay back to UNKNOWN.  A real wall
        receives far more HIT updates than MISS, so it stays OCCUPIED; a
        single-frame false positive receives mostly MISS and clears quickly.
        """
        dx = abs(x1 - x0);  sx = 1 if x0 < x1 else -1
        dy = abs(y1 - y0);  sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0

        while not (x == x1 and y == y1):
            if self.in_grid(x, y):
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
        """Identify frontier clusters and rank them by information gain.

        A frontier cell is a FREE cell (log-odds ≤ FREE_THRESH) that has at
        least one UNKNOWN neighbour (log-odds between the two thresholds).
        Clusters are connected components of frontier cells (8-connected).

        Returns:
            List of (wx, wy, cluster_size, score) sorted by score descending.
            score = cluster_size / distance_to_robot
        """
        free_mask    = self.log_odds <= self.LOG_ODDS_FREE_THRESH
        unknown_mask = ((self.log_odds > self.LOG_ODDS_FREE_THRESH) &
                        (self.log_odds < self.LOG_ODDS_OCC_THRESH))

        # Frontier = FREE cell adjacent (4-connected) to UNKNOWN
        frontier_mask = np.zeros_like(self.log_odds, dtype=bool)
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            shifted = np.roll(np.roll(unknown_mask, dy, axis=0), dx, axis=1)
            frontier_mask |= (free_mask & shifted)

        fys, fxs = np.where(frontier_mask)
        if len(fxs) == 0:
            return []

        # BFS clustering (8-connected so diagonal frontier strips merge)
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
            mean_gx = np.mean([c[0] for c in cluster])
            mean_gy = np.mean([c[1] for c in cluster])
            wx, wy  = self.grid_to_world(int(mean_gx), int(mean_gy))
            dist    = np.hypot(wx - bx, wy - by)
            score   = len(cluster) / max(dist, 0.5)
            results.append((wx, wy, len(cluster), score))

        results.sort(key=lambda r: r[3], reverse=True)
        return results

    # ── Publishing ────────────────────────────────────────────────────────────

    def publish_map(self):
        """Threshold log-odds → ROS occupancy values and publish both topics."""
        grid = np.full((self.GRID_SIZE, self.GRID_SIZE), -1, dtype=np.int8)
        grid[self.log_odds <= self.LOG_ODDS_FREE_THRESH] = 0
        grid[self.log_odds >= self.LOG_ODDS_OCC_THRESH]  = 100

        # OccupancyGrid
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

        # RGB debug image  (gray=unknown, white=free, black=occupied)
        img = np.full((self.GRID_SIZE, self.GRID_SIZE, 3), 128, dtype=np.uint8)
        img[grid == 0]   = [255, 255, 255]
        img[grid == 100] = [  0,   0,   0]
        img_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding='rgb8')
        img_msg.header.stamp    = msg.header.stamp
        img_msg.header.frame_id = 'odom'
        self.map_image_pub.publish(img_msg)

    def publish_frontiers(self, frontiers):
        """Visualise frontier centroids as green cylinders in RViz."""
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

    # ── State machine helpers ─────────────────────────────────────────────────

    def _start_scan(self):
        """Reset rotation state and enter SCANNING."""
        self.scan_accumulated  = 0.0
        self.scan_last_heading = None
        self.scan_settle_start = None
        self.state = ExploreState.SCANNING
        self.get_logger().info('[SCAN] Starting 360° rotation scan...')

    def _stop_base(self):
        """Publish zero velocity to halt the base."""
        self.cmd_vel_pub.publish(Twist())

    # ── States ────────────────────────────────────────────────────────────────

    def _state_scanning(self):
        """Rotate 360° in place while the map update runs every loop tick."""

        # ── Settle phase: motors stopped, waiting for vibration to die down ──
        if self.scan_settle_start is not None:
            if time.time() - self.scan_settle_start >= self.SCAN_SETTLE_TIME:
                self.get_logger().info('[SCAN] Settled. Selecting frontier...')
                self.state = ExploreState.SELECTING
            return   # don't spin during settle

        # ── Track accumulated rotation via TF heading ─────────────────────────
        heading = self.get_heading()
        if heading is None:
            return

        if self.scan_last_heading is not None:
            delta = heading - self.scan_last_heading
            # Wrap to [-π, π] to handle the ±π discontinuity cleanly
            delta = (delta + np.pi) % (2 * np.pi) - np.pi
            self.scan_accumulated += abs(delta)

        self.scan_last_heading = heading

        if self.scan_accumulated >= 2 * np.pi:
            # Full revolution done — stop and settle
            self._stop_base()
            self.scan_settle_start = time.time()
            self.get_logger().info(
                f'[SCAN] 360° complete '
                f'({np.degrees(self.scan_accumulated):.0f}° accumulated). '
                f'Settling for {self.SCAN_SETTLE_TIME} s...')
        else:
            # Keep rotating
            cmd = Twist()
            cmd.angular.z = self.SCAN_ANGULAR_VEL
            self.cmd_vel_pub.publish(cmd)

    def _state_selecting(self):
        """Find the best frontier and send a navigation goal."""
        frontiers = self.find_frontiers()
        self.publish_frontiers(frontiers)

        if not frontiers:
            self.no_frontier_count += 1
            if self.no_frontier_count >= self.NO_FRONTIER_LIMIT:
                self.get_logger().info(
                    f'[SELECT] No frontiers after {self.NO_FRONTIER_LIMIT} '
                    f'attempts — exploration complete!')
                self.state = ExploreState.COMPLETE
            else:
                self.get_logger().info(
                    f'[SELECT] No frontiers '
                    f'(attempt {self.no_frontier_count}/{self.NO_FRONTIER_LIMIT})'
                    f' — re-scanning.')
                self._start_scan()
            return

        self.no_frontier_count = 0
        wx, wy, size, score = frontiers[0]
        self.get_logger().info(
            f'[SELECT] Best frontier ({wx:.2f}, {wy:.2f}) '
            f'size={size} score={score:.1f}')

        # Set goal slightly in front of the frontier centroid (toward the robot)
        # so we stop in free space rather than inside an unknown region.
        base_pose = self.get_base_pose()
        if base_pose is None:
            self._start_scan()
            return
        bx, by, _ = base_pose
        dx, dy    = wx - bx, wy - by
        dist      = np.hypot(dx, dy)

        if dist > self.FRONTIER_NAV_OFFSET * 2:
            ratio  = (dist - self.FRONTIER_NAV_OFFSET) / dist
            goal_x = bx + dx * ratio
            goal_y = by + dy * ratio
        else:
            goal_x, goal_y = wx, wy

        # Face the frontier; subtract π/2 for the bridge's frame convention
        goal_theta = np.arctan2(dy, dx)
        user_theta = goal_theta - np.pi / 2

        self.get_logger().info(
            f'[SELECT] Navigating to ({goal_x:.2f}, {goal_y:.2f}) '
            f'heading={np.degrees(goal_theta):.0f}°')

        goal = Pose2D()
        goal.x = goal_x;  goal.y = goal_y;  goal.theta = user_theta
        self.base_goal_reached = False
        self.base_goal_pub.publish(goal)
        self.nav_start_time = time.time()
        self.state = ExploreState.NAVIGATING

    def _state_navigating(self):
        """Wait for the base to reach the goal, then re-scan."""
        if self.base_goal_reached:
            self.get_logger().info('[NAV] Goal reached — starting scan.')
            self._start_scan()
            return

        elapsed = time.time() - self.nav_start_time
        if elapsed > self.NAV_TIMEOUT:
            self.get_logger().warn(
                f'[NAV] Timeout after {self.NAV_TIMEOUT:.0f} s — '
                'scanning from current position.')
            self._start_scan()

    def _state_complete(self):
        """Print exploration summary and keep the map alive in RViz."""
        self._stop_base()
        self.publish_map()

        free     = int(np.sum(self.log_odds <= self.LOG_ODDS_FREE_THRESH))
        occupied = int(np.sum(self.log_odds >= self.LOG_ODDS_OCC_THRESH))
        total    = self.GRID_SIZE ** 2
        coverage = (free + occupied) / total * 100

        self.get_logger().info('=' * 55)
        self.get_logger().info('EXPLORATION COMPLETE')
        self.get_logger().info(f'  Coverage : {coverage:.1f}%')
        self.get_logger().info(f'  Free     : {free}')
        self.get_logger().info(f'  Occupied : {occupied}')
        self.get_logger().info(f'  Unknown  : {total - free - occupied}')
        self.get_logger().info('=' * 55)
        self.get_logger().info('Map live — Ctrl+C to stop.')

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1.0)
            self.publish_map()

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        """Block until sensors are ready, then run the exploration loop."""
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

            # Map updates are driven by _depth_cb — nothing to do here.

            now = time.time()
            if now - self.last_map_publish >= 1.0 / self.MAP_PUBLISH_RATE:
                self.publish_map()
                self.last_map_publish = now

            # Advance state machine
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
