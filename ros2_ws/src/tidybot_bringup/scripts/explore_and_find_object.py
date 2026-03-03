#!/usr/bin/env python3
"""
TidyBot2 Explore & Find Object

Combines frontier exploration with color-based object detection.
The robot explores unknown space using frontier-based exploration until
it detects the target colored cube (red, yellow, or blue), then navigates
to it.

State machine:
  1. SCANNING    — 360 rotation scan, mapping depth into occupancy grid.
  2. SELECTING   — pick best frontier (or approach known object).
  3. NAVIGATING  — follow A* waypoints to frontier, checking for target.
  4. APPROACHING — navigate to detected object's world position.
  5. COMPLETE    — object found and reached.

Object detection uses HSV color filtering on the RGB camera feed.
When the target color blob is detected in multiple consecutive frames,
its world position is estimated via depth back-projection and the robot
navigates to it.

Publishes:
  /map               nav_msgs/OccupancyGrid
  /frontier_markers  visualization_msgs/MarkerArray
  /object_marker     visualization_msgs/MarkerArray
  /cmd_vel           geometry_msgs/Twist

Subscribes:
  /camera/depth/image_raw    — depth frames (16UC1, mm)
  /camera/depth/camera_info  — depth camera intrinsics
  /camera/color/image_raw    — RGB frames for object detection

Prerequisites:
    ros2 launch tidybot_bringup sim.launch.py scene:=scene_obstacles.xml

Usage:
    ros2 run tidybot_bringup explore_and_find_object.py --ros-args -p target_color:=red
"""

import heapq
import time
from collections import deque
from enum import Enum

import cv2
import numpy as np
from scipy.ndimage import binary_dilation, label as ndimage_label
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import MapMetaData, OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2DArray

import tf2_ros

try:
    from cv_bridge import CvBridge
except ImportError:
    print("ERROR: cv_bridge is required.")
    raise


class ExploreState(Enum):
    SCANNING    = 0
    SELECTING   = 1
    NAVIGATING  = 2
    APPROACHING = 3
    COMPLETE    = 4


# HSV color ranges for cube detection (OpenCV HSV: H=[0,180], S/V=[0,255])
# Ranges are intentionally wide to handle MuJoCo lighting variation
# (headlight diffuse + ambient + directional light).
# COLOR_RANGES = {
#     'red':    [((0,   80,  50),  (10,  255, 255)),
#                ((165, 80,  50),  (180, 255, 255))],
#     'blue':   [((95,  50,  40),  (135, 255, 255))],
#     'yellow': [((20,  60,  60),  (45,  255, 255))],
#     'green':  [((35,  40,  40),  (90,  255, 255))],
#     'orange': [((8,   100, 80),  (20,  255, 255))],
# }


class ExploreAndFind(Node):

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
    MIN_OBSTACLE_Z = 0.10    # m — clears floor returns (z ≈ 0)
    MAX_OBSTACLE_Z = 0.80    # m — ignores camera mount / ceilings

    # ── Log-odds belief (Thrun, Burgard & Fox, Ch. 9) ────────────────────────
    LOG_ODDS_HIT         =  0.85
    LOG_ODDS_MISS        = -0.40
    LOG_ODDS_MIN         = -10.0
    LOG_ODDS_MAX         =  10.0
    LOG_ODDS_FREE_THRESH = -0.50
    LOG_ODDS_OCC_THRESH  =  1.50

    # ── Angular-diversity obstacle confirmation ───────────────────────────────
    MIN_HIT_ANGLE_SEP = np.radians(20)
    MIN_HIT_COUNT     = 2

    # ── Map publishing ────────────────────────────────────────────────────────
    MAP_PUBLISH_RATE = 1.0   # Hz

    # ── 360 scan ─────────────────────────────────────────────────────────────
    SCAN_ANGULAR_VEL = 0.3   # rad/s
    SCAN_SETTLE_TIME = 0.5   # s

    # ── Frontier selection ────────────────────────────────────────────────────
    MIN_FRONTIER_SIZE   = 8
    MIN_FRONTIER_DIST   = 1.0   # m
    ROBOT_CLEARANCE     = 0.35  # m

    # ── Navigation ────────────────────────────────────────────────────────────
    NAV_TIMEOUT        = 120.0  # s per waypoint
    NO_FRONTIER_LIMIT  = 3
    WAYPOINT_SPACING   = 20
    WAYPOINT_TOLERANCE = 0.3    # m
    NAV_LINEAR_SPEED   = 0.2   # m/s
    OBSTACLE_THRESHOLD = 0.8    # m
    DEPTH_STEER_TIMEOUT = 3.0  # s — force re-plan after steering this long

    # ── Depth integration angular gating ──────────────────────────────────────
    MAX_MAPPING_ANGULAR_VEL = 0.5  # rad/s

    # ── Object detection ──────────────────────────────────────────────────────
    MIN_DETECTION_AREA = 50    # px² — minimum color blob area
    APPROACH_DIST      = 0.30  # m — declare arrived when this close
    DETECT_WINDOW      = 2.0   # s — time window for detection confirmation
    DETECT_COUNT_REQ   = 2     # frames within window to confirm

    # ─────────────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('explore_and_find')

        # Persistent log-odds grid
        self.log_odds  = np.zeros(
            (self.GRID_SIZE, self.GRID_SIZE), dtype=np.float32)
        self.hit_count = np.zeros(
            (self.GRID_SIZE, self.GRID_SIZE), dtype=np.uint8)
        self.hit_angle = np.full(
            (self.GRID_SIZE, self.GRID_SIZE), np.nan, dtype=np.float32)

        # Camera state
        self.latest_depth       = None
        self.latest_depth_stamp = None
        self.latest_rgb         = None
        self.camera_K           = None
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
        self.nav_waypoints     = []
        self.nav_waypoint_idx  = 0
        self.last_cmd_angular  = 0.0
        self.last_approach_replan = 0.0

        # Object detection state
        # self.declare_parameter('target_color', 'red')
        # self.target_color = (
        #     self.get_parameter('target_color')
        #     .get_parameter_value().string_value)
        # if self.target_color not in COLOR_RANGES:
        #     self.get_logger().error(
        #         f'Unknown target_color "{self.target_color}". Using "red".')
        #     self.target_color = 'red'

        # Object detection state (YOLO version)
        self.declare_parameter('target_class_id', 46) # Default to 46 (e.g., banana/apple)
        self.target_class_id = self.get_parameter('target_class_id').get_parameter_value().integer_value
        self.object_world_pos      = None   # (wx, wy) once confirmed
        self.detection_times       = []     # timestamps of recent detections
        self.last_detection_pos    = None   # (wx, wy) of most recent detection
        self.latest_yolo_detection = None   # most recent matching Detection2D
        self.depth_steer_start     = None   # when depth-steering began (stuck detection)

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image,      '/camera/depth/image_raw',
                                 self._depth_cb,       be)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info',
                                 self._camera_info_cb, be)
        # self.create_subscription(Image,      '/camera/color/image_raw',
        #                          self._rgb_cb,         be)
        
        self.bbox_sub = self.create_subscription(
            Detection2DArray, 
            '/objbbox', 
            self._yolo_bbox_cb, 
            10)

        # Publishers
        self.map_pub           = self.create_publisher(
            OccupancyGrid, '/map', 10)
        self.frontier_pub      = self.create_publisher(
            MarkerArray, '/frontier_markers', 10)
        self.object_marker_pub = self.create_publisher(
            MarkerArray, '/object_marker', 10)
        self.cmd_vel_pub       = self.create_publisher(
            Twist, '/cmd_vel', 10)
        latch_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.inflated_map_pub  = self.create_publisher(
            OccupancyGrid, '/inflated_map', latch_qos)
        self.nav_path_pub      = self.create_publisher(
            MarkerArray, '/nav_path', latch_qos)

        self.get_logger().info('=' * 55)
        self.get_logger().info(
            'Explore & Find  (frontier exploration + object detection)')
        self.get_logger().info(f'  Target : YOLO class_id={self.target_class_id}')
        self.get_logger().info(
            f'  Grid   : {self.GRID_SIZE}x{self.GRID_SIZE} '
            f'@ {self.GRID_RESOLUTION} m/cell '
            f'({self.GRID_SIZE * self.GRID_RESOLUTION:.0f} m x '
            f'{self.GRID_SIZE * self.GRID_RESOLUTION:.0f} m)')
        self.get_logger().info(
            f'  Height : {self.MIN_OBSTACLE_Z}-{self.MAX_OBSTACLE_Z} m')
        self.get_logger().info(
            f'  Scan   : {self.SCAN_ANGULAR_VEL} rad/s '
            f'(~{2*np.pi/self.SCAN_ANGULAR_VEL:.0f} s per 360)')
        self.get_logger().info('=' * 55)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _depth_cb(self, msg: Image):
        try:
            depth = self.cv_bridge.imgmsg_to_cv2(msg, '16UC1')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')
            return
        self.latest_depth       = depth
        self.latest_depth_stamp = msg.header.stamp
        if self.camera_K is not None:
            if abs(self.last_cmd_angular) <= self.MAX_MAPPING_ANGULAR_VEL:
                self._integrate_depth()

    # def _rgb_cb(self, msg: Image):
    #     try:
    #         self.latest_rgb = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
    #     except Exception as e:
    #         self.get_logger().warn(f'RGB conversion failed: {e}')
    #         return
    #     if not hasattr(self, '_rgb_logged'):
    #         self._rgb_logged = True
    #         h, w = self.latest_rgb.shape[:2]
    #         self.get_logger().info(
    #             f'[RGB] First frame: {w}x{h} encoding={msg.encoding}')

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
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(f'TF odom->base_link: {e}')
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
        if self.latest_depth is None or self.camera_K is None:
            return

        try:
            stamp  = rclpy.time.Time.from_msg(self.latest_depth_stamp)
            tf_ob  = self.tf_buffer.lookup_transform(
                'odom', 'base_link', stamp, timeout=Duration(seconds=0.1))
        except Exception:
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

        cam_origin = R_ob @ np.array([tc.x, tc.y, tc.z]) + np.array([to.x, to.y, to.z])
        R          = R_ob @ R_bc

        fx = self.camera_K[0, 0];  fy = self.camera_K[1, 1]
        cx = self.camera_K[0, 2];  cy = self.camera_K[1, 2]

        depth = self.latest_depth
        h, w  = depth.shape
        step  = self.DEPTH_SUBSAMPLE

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

        gxs = ((pts_odom[:, 0] - self.GRID_ORIGIN_X) / self.GRID_RESOLUTION).astype(int)
        gys = ((pts_odom[:, 1] - self.GRID_ORIGIN_Y) / self.GRID_RESOLUTION).astype(int)

        dist_xy  = np.hypot(pts_odom[:, 0] - cam_origin[0],
                            pts_odom[:, 1] - cam_origin[1])
        not_self = dist_xy > self.ROBOT_BODY_RADIUS

        in_bounds = ((gxs >= 0) & (gxs < self.GRID_SIZE) &
                     (gys >= 0) & (gys < self.GRID_SIZE))
        in_height = ((pts_odom[:, 2] > self.MIN_OBSTACLE_Z) &
                     (pts_odom[:, 2] < self.MAX_OBSTACLE_Z))

        obs_mask = not_self & in_bounds & in_height

        floor_mask = (not_self & in_bounds & ~in_height &
                      (depths_m < self.FREE_TRACE_MAX_M))

        obs_idx   = np.where(obs_mask)[0]
        floor_idx = np.where(floor_mask)[0]

        cam_gx, cam_gy = self.world_to_grid(cam_origin[0], cam_origin[1])

        # Mark robot footprint as free
        r_cells = int(self.ROBOT_BODY_RADIUS / self.GRID_RESOLUTION) + 1
        fy_min  = max(0, cam_gy - r_cells)
        fy_max  = min(self.GRID_SIZE, cam_gy + r_cells + 1)
        fx_min  = max(0, cam_gx - r_cells)
        fx_max  = min(self.GRID_SIZE, cam_gx + r_cells + 1)
        ffy, ffx = np.meshgrid(
            np.arange(fy_min, fy_max), np.arange(fx_min, fx_max), indexing='ij')
        in_fp   = ((ffy - cam_gy)**2 + (ffx - cam_gx)**2) <= r_cells ** 2
        fp_gy   = ffy[in_fp];  fp_gx = ffx[in_fp]
        clearable = self.hit_count[fp_gy, fp_gx] < self.MIN_HIT_COUNT
        self.log_odds[fp_gy[clearable], fp_gx[clearable]] = np.maximum(
            self.LOG_ODDS_MIN,
            self.log_odds[fp_gy[clearable], fp_gx[clearable]] + self.LOG_ODDS_MISS)

        if len(obs_idx) == 0 and len(floor_idx) == 0:
            return

        if len(obs_idx) > 0:
            ray_step = max(1, len(obs_idx) // 200)
            for i in obs_idx[::ray_step]:
                self._raytrace_free(cam_gx, cam_gy, int(gxs[i]), int(gys[i]))

        if len(floor_idx) > 0:
            ray_step2 = max(1, len(floor_idx) // 100)
            for i in floor_idx[::ray_step2]:
                self._raytrace_free(cam_gx, cam_gy, int(gxs[i]), int(gys[i]))

        if len(obs_idx) > 0:
            np.add.at(self.log_odds, (gys[obs_mask], gxs[obs_mask]), self.LOG_ODDS_HIT)
            np.clip(self.log_odds, self.LOG_ODDS_MIN, self.LOG_ODDS_MAX, out=self.log_odds)

        if len(obs_idx) > 0:
            base_pose = self.get_base_pose()
            if base_pose is not None:
                cam_theta = float(base_pose[2])

                u_yx = np.unique(
                    np.stack([gys[obs_mask], gxs[obs_mask]], axis=1), axis=0)
                u_gy, u_gx = u_yx[:, 0], u_yx[:, 1]

                no_prior = np.isnan(self.hit_angle[u_gy, u_gx])
                if np.any(no_prior):
                    self.hit_angle[u_gy[no_prior], u_gx[no_prior]] = cam_theta
                    self.hit_count[u_gy[no_prior], u_gx[no_prior]] = 1

                has_prior = ~no_prior
                if np.any(has_prior):
                    hp_gy     = u_gy[has_prior]
                    hp_gx     = u_gx[has_prior]
                    prior     = self.hit_angle[hp_gy, hp_gx]
                    diff      = np.abs(cam_theta - prior)
                    diff      = np.minimum(diff, 2 * np.pi - diff)
                    diverse   = diff >= self.MIN_HIT_ANGLE_SEP
                    if np.any(diverse):
                        d_gy = hp_gy[diverse];  d_gx = hp_gx[diverse]
                        self.hit_count[d_gy, d_gx] = np.minimum(
                            self.hit_count[d_gy, d_gx].astype(np.uint16) + 1,
                            255).astype(np.uint8)
                        self.hit_angle[d_gy, d_gx] = cam_theta

    def _raytrace_free(self, x0: int, y0: int, x1: int, y1: int):
        dx = abs(x1 - x0);  sx = 1 if x0 < x1 else -1
        dy = abs(y1 - y0);  sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0

        while not (x == x1 and y == y1):
            if self.in_grid(x, y):
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

        if not (self.in_grid(start_gx, start_gy) and
                self.in_grid(goal_gx, goal_gy)):
            return None

        if not traversable[start_gy, start_gx]:
            if not free_mask[start_gy, start_gx]:
                return None
            traversable[start_gy, start_gx] = True

        if not traversable[goal_gy, goal_gx]:
            # Goal is in inflated zone — snap to nearest traversable cell
            best_d2 = float('inf')
            best_gx, best_gy = None, None
            search_r = int(np.ceil(clearance_m / self.GRID_RESOLUTION)) + 5
            for dy in range(-search_r, search_r + 1):
                for dx in range(-search_r, search_r + 1):
                    nx, ny = goal_gx + dx, goal_gy + dy
                    if (0 <= nx < self.GRID_SIZE and
                            0 <= ny < self.GRID_SIZE and
                            traversable[ny, nx]):
                        d2 = dx * dx + dy * dy
                        if d2 < best_d2:
                            best_d2 = d2
                            best_gx, best_gy = nx, ny
            if best_gx is None:
                return None
            goal_gx, goal_gy = best_gx, best_gy
            goal = (goal_gx, goal_gy)

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

        return None

    def _subsample_path(self, path):
        if len(path) <= 1:
            return path
        waypoints = []
        for i in range(self.WAYPOINT_SPACING, len(path) - 1, self.WAYPOINT_SPACING):
            waypoints.append(path[i])
        waypoints.append(path[-1])
        return waypoints

    def _compute_inflated(self, clearance_m=None):
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
        nav_clear = getattr(self, 'nav_clearance', None)
        inflated = self._compute_inflated(nav_clear)
        # Only look 3 waypoints ahead — distant map noise should not trigger
        # a replan; the robot will reroute when it gets closer.
        lookahead = min(self.nav_waypoint_idx + 3, len(self.nav_waypoints))
        for i in range(self.nav_waypoint_idx, lookahead):
            gx, gy = self.nav_waypoints[i]
            if self.in_grid(gx, gy) and inflated[gy, gx]:
                return True
        return False

    def _replan_from_here(self):
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
        self.publish_nav_path()
        self.get_logger().info(
            f'[NAV] Re-planned: {len(path)} waypoints from '
            f'({bx:.2f},{by:.2f})')
        return True

    # ── Depth obstacle avoidance ─────────────────────────────────────────────

    def _analyze_depth_obstacles(self):
        depth = self.latest_depth
        if depth is None:
            return (True, True, True), (999.0, 999.0, 999.0)

        h, w = depth.shape
        v_start = int(h * 0.3)
        v_end   = int(h * 0.55)  # stop at 55% to avoid floor returns
        margin = int(w * self.DEPTH_H_CROP)
        u_start = margin
        u_end   = w - margin
        third   = (u_end - u_start) // 3

        left_region   = depth[v_start:v_end, u_start:u_start + third]
        center_region = depth[v_start:v_end, u_start + third:u_start + 2 * third]
        right_region  = depth[v_start:v_end, u_start + 2 * third:u_end]

        def sector_min_dist(region):
            min_mm = self.MIN_DEPTH_M * 1000.0
            valid = region[(region > min_mm)].astype(np.float64)
            if len(valid) == 0:
                return 999.0
            return float(np.percentile(valid, 10)) / 1000.0

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

    # ── Object detection ──────────────────────────────────────────────────────

    # def _check_for_object(self):
    #     """Check latest RGB frame for target colored cube.

    #     Runs HSV color filtering, finds the largest matching blob, and
    #     estimates its world position via depth back-projection.  Requires
    #     DETECT_COUNT_REQ detections within DETECT_WINDOW seconds to confirm.

    #     Returns True when the object is confirmed.
    #     """
    #     if self.latest_rgb is None or self.camera_K is None:
    #         return False

    #     hsv = cv2.cvtColor(self.latest_rgb, cv2.COLOR_BGR2HSV)

    #     # Build color mask from all HSV ranges for the target color
    #     mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    #     for lower, upper in COLOR_RANGES.get(self.target_color, []):
    #         mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))

    #     contours, _ = cv2.findContours(
    #         mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #     if not contours:
    #         return False

    #     # Take the largest blob
    #     largest = max(contours, key=cv2.contourArea)
    #     area = cv2.contourArea(largest)
    #     if area < self.MIN_DETECTION_AREA:
    #         return False

    #     # Centroid of the blob
    #     M = cv2.moments(largest)
    #     if M['m00'] == 0:
    #         return False
    #     cu = int(M['m10'] / M['m00'])
    #     cv_pt = int(M['m01'] / M['m00'])

    #     # Estimate world position using depth
    #     pos = self._estimate_object_position(cu, cv_pt)
    #     if pos is None:
    #         return False

    #     # Check consistency — if position jumped far, reset window
    #     if self.last_detection_pos is not None:
    #         d = np.hypot(pos[0] - self.last_detection_pos[0],
    #                      pos[1] - self.last_detection_pos[1])
    #         if d > 1.0:
    #             self.detection_times = []

    #     now = time.time()
    #     self.detection_times.append(now)
    #     self.last_detection_pos = pos

    #     # Prune old detections outside the window
    #     cutoff = now - self.DETECT_WINDOW
    #     self.detection_times = [t for t in self.detection_times if t >= cutoff]

    #     self.get_logger().info(
    #         f'[DETECT] {self.target_color} candidate at '
    #         f'({pos[0]:.2f}, {pos[1]:.2f}) area={area:.0f} '
    #         f'count={len(self.detection_times)}/{self.DETECT_COUNT_REQ} '
    #         f'in {self.DETECT_WINDOW}s window')

    #     if len(self.detection_times) >= self.DETECT_COUNT_REQ:
    #         self.object_world_pos = pos
    #         self.get_logger().info(
    #             f'[DETECT] *** {self.target_color} cube CONFIRMED at '
    #             f'({pos[0]:.2f}, {pos[1]:.2f}) ***')
    #         return True

    #     return False

    def _yolo_bbox_cb(self, msg: Detection2DArray):
        """Callback for YOLO detections from the ObjectClassifier node.

        Uses a temporal confirmation window (DETECT_COUNT_REQ detections
        within DETECT_WINDOW seconds) to avoid false positives from
        single-frame YOLO misclassifications.
        """
        if self.state == ExploreState.COMPLETE:
            return

        for detection in msg.detections:
            if not detection.results:
                continue
            class_id = int(detection.results[0].hypothesis.class_id)

            if class_id != self.target_class_id:
                continue

            u = int(detection.bbox.center.position.x)
            v = int(detection.bbox.center.position.y)
            pos = self._estimate_object_position(u, v)
            if pos is None:
                continue

            # Store latest detection for approach refinement
            self.latest_yolo_detection = detection

            # If already confirmed, just update position (approach
            # refinement handled in _state_approaching)
            if self.object_world_pos is not None:
                return

            # Check consistency — if position jumped far, reset window
            if self.last_detection_pos is not None:
                d = np.hypot(pos[0] - self.last_detection_pos[0],
                             pos[1] - self.last_detection_pos[1])
                if d > 1.0:
                    self.detection_times = []

            now = time.time()
            self.detection_times.append(now)
            self.last_detection_pos = pos

            # Prune old detections outside the window
            cutoff = now - self.DETECT_WINDOW
            self.detection_times = [
                t for t in self.detection_times if t >= cutoff]

            self.get_logger().info(
                f'[DETECT] YOLO class {class_id} candidate at '
                f'({pos[0]:.2f}, {pos[1]:.2f}) '
                f'count={len(self.detection_times)}/{self.DETECT_COUNT_REQ} '
                f'in {self.DETECT_WINDOW}s window')

            if len(self.detection_times) >= self.DETECT_COUNT_REQ:
                self.object_world_pos = pos
                self.get_logger().info(
                    f'[DETECT] *** YOLO class {class_id} CONFIRMED at '
                    f'({pos[0]:.2f}, {pos[1]:.2f}) ***')
                if self.state != ExploreState.APPROACHING:
                    self.state = ExploreState.APPROACHING
                    self._plan_approach()
            return

    def _estimate_object_position(self, u, v):
        """Estimate world (x, y) of pixel (u, v) using depth + TF."""
        if self.latest_depth is None or self.camera_K is None:
            return None

        depth = self.latest_depth
        h, w = depth.shape

        # Average depth over small window for robustness
        half_win = 4
        v_lo = max(0, v - half_win)
        v_hi = min(h, v + half_win + 1)
        u_lo = max(0, u - half_win)
        u_hi = min(w, u + half_win + 1)

        patch = depth[v_lo:v_hi, u_lo:u_hi].astype(np.float64)
        valid = patch[patch > 0]
        if len(valid) == 0:
            return None
        depth_m = float(np.median(valid)) / 1000.0

        if depth_m < self.MIN_DEPTH_M or depth_m > self.MAX_DEPTH_M:
            return None

        # Back-project to camera frame
        fx = self.camera_K[0, 0]
        fy = self.camera_K[1, 1]
        cx_k = self.camera_K[0, 2]
        cy_k = self.camera_K[1, 2]

        x_cam = (u - cx_k) * depth_m / fx
        y_cam = (v - cy_k) * depth_m / fy
        z_cam = depth_m
        pt_cam = np.array([x_cam, y_cam, z_cam])

        # Transform to world frame
        try:
            tf_ob = self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return None

        try:
            tf_bc = self.tf_buffer.lookup_transform(
                'base_link', 'camera_depth_optical_frame',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return None

        def _q2m(q):
            qw, qx, qy, qz = q.w, q.x, q.y, q.z
            return np.array([
                [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)],
            ])

        R_ob = _q2m(tf_ob.transform.rotation)
        R_bc = _q2m(tf_bc.transform.rotation)
        to = tf_ob.transform.translation
        tc = tf_bc.transform.translation

        cam_origin = R_ob @ np.array([tc.x, tc.y, tc.z]) + \
                     np.array([to.x, to.y, to.z])
        R = R_ob @ R_bc

        pt_world = R @ pt_cam + cam_origin
        return (float(pt_world[0]), float(pt_world[1]))

    def _plan_approach(self):
        """Try to plan A* path to object_world_pos.  Returns True on success."""
        if self.object_world_pos is None:
            return False

        pose = self.get_base_pose()
        if pose is None:
            return False
        bx, by, _ = pose
        ox, oy = self.object_world_pos

        # Already close enough?
        if np.hypot(ox - bx, oy - by) < self.APPROACH_DIST:
            self._stop_base()
            self.get_logger().info(
                f'[APPROACH] Already at target object!')
            self.state = ExploreState.COMPLETE
            return True

        robot_gx, robot_gy = self.world_to_grid(bx, by)
        goal_gx, goal_gy = self.world_to_grid(ox, oy)

        # Try cascading inflation levels
        clearances = [None, self.ROBOT_CLEARANCE * 0.5]
        labels     = ['full', 'half']

        for clearance, label in zip(clearances, labels):
            path = self._plan_path(robot_gx, robot_gy, goal_gx, goal_gy,
                                   clearance_m=clearance)
            if path is not None:
                self._stop_base()
                self.nav_waypoints    = path
                self.nav_waypoint_idx = 0
                self.nav_start_time   = time.time()
                self.nav_clearance    = clearance
                self.publish_nav_path()
                self.state = ExploreState.APPROACHING
                self.get_logger().info(
                    f'[APPROACH] Path to target object at '
                    f'({ox:.2f}, {oy:.2f}): {len(path)} waypoints '
                    f'({label} inflation)')
                return True

        self.get_logger().info(
            f'[APPROACH] No path to object at ({ox:.2f}, {oy:.2f}) '
            f'— continuing exploration')
        return False

    def publish_object_marker(self):
        """Publish a sphere marker at the detected object position."""
        if self.object_world_pos is None:
            return
        m = Marker()
        m.header.stamp       = self.get_clock().now().to_msg()
        m.header.frame_id    = 'odom'
        m.ns                 = 'target_object'
        m.id                 = 0
        m.type               = Marker.SPHERE
        m.action             = Marker.ADD
        m.pose.position.x    = self.object_world_pos[0]
        m.pose.position.y    = self.object_world_pos[1]
        m.pose.position.z    = 0.05
        m.pose.orientation.w = 1.0
        m.scale.x = 0.15;  m.scale.y = 0.15;  m.scale.z = 0.15
        m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
        ma = MarkerArray()
        ma.markers.append(m)
        self.object_marker_pub.publish(ma)

    # ── Frontier detection ────────────────────────────────────────────────────

    def find_frontiers(self):
        free_mask    = self.log_odds <= self.LOG_ODDS_FREE_THRESH
        confirmed_occ = ((self.log_odds  >= self.LOG_ODDS_OCC_THRESH) &
                         (self.hit_count >= self.MIN_HIT_COUNT))
        unknown_mask  = ~free_mask & ~confirmed_occ

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

        _labeled, _ = ndimage_label(free_mask, structure=np.ones((3, 3), dtype=bool))
        if (self.in_grid(robot_gx, robot_gy) and
                _labeled[robot_gy, robot_gx] > 0):
            reachable = (_labeled == _labeled[robot_gy, robot_gx])
        else:
            reachable = None

        r_cells = int(self.MAX_DEPTH_M / self.GRID_RESOLUTION)
        ky, kx  = np.ogrid[-r_cells:r_cells + 1, -r_cells:r_cells + 1]
        disc    = (ky ** 2 + kx ** 2) <= r_cells ** 2

        results = []
        n_dist = n_reach = n_clear = 0

        for cluster in clusters:
            cluster_cells = np.array(cluster)
            cxs, cys = cluster_cells[:, 0], cluster_cells[:, 1]

            mgx = int(np.mean(cxs))
            mgy = int(np.mean(cys))
            wx, wy = self.grid_to_world(mgx, mgy)
            dist   = np.hypot(wx - bx, wy - by)

            if dist < self.MIN_FRONTIER_DIST:
                n_dist += 1
                continue

            if reachable is not None:
                if not np.any(reachable[cys, cxs]):
                    n_reach += 1
                    continue

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

            # ── Bias toward last-seen object position ─────────────
            # If the target was glimpsed (even unconfirmed), override
            # the exploration score: rank primarily by proximity to
            # the object, with a small info-gain tiebreaker.
            if self.last_detection_pos is not None:
                dist_to_obj = np.hypot(wx - self.last_detection_pos[0],
                                       wy - self.last_detection_pos[1])
                # Primary: closer to object = higher score
                # 1/(d+0.3) so a frontier 0.5m from object scores ~1.25
                # while one 5m away scores ~0.19
                proximity = 1.0 / (dist_to_obj + 0.3)
                # Normalize info-gain to [0,1] range as tiebreaker
                info_norm = min(nearby_unknown / 500.0, 1.0)
                score = proximity * 10.0 + info_norm

            results.append((wx, wy, len(cluster), score))

        bias_str = ''
        if self.last_detection_pos is not None:
            bias_str = (f', biased toward ({self.last_detection_pos[0]:.1f},'
                        f' {self.last_detection_pos[1]:.1f})')
        self.get_logger().info(
            f'[FRONTIER] {len(clusters)} clusters -> {len(results)} valid '
            f'(filtered: dist={n_dist}, reach={n_reach}, clear={n_clear})'
            f'{bias_str}')

        results.sort(key=lambda r: r[3], reverse=True)
        return results

    # ── Publishing ────────────────────────────────────────────────────────────

    def publish_map(self):
        span  = float(self.LOG_ODDS_MAX - self.LOG_ODDS_MIN)
        ratio = (self.log_odds - self.LOG_ODDS_MIN) / span
        occ   = np.clip(ratio * 100.0, 0.0, 100.0)

        occ[self.hit_count < self.MIN_HIT_COUNT] = np.minimum(
            occ[self.hit_count < self.MIN_HIT_COUNT], 80.0)

        grid = occ.astype(np.int8)
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
        self.publish_inflated_map()
        self.publish_nav_path()

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

    def publish_inflated_map(self):
        """Publish the A*-inflated obstacle layer as a second OccupancyGrid."""
        inflated = self._compute_inflated()
        confirmed_occ = ((self.log_odds >= self.LOG_ODDS_OCC_THRESH) &
                         (self.hit_count >= self.MIN_HIT_COUNT))
        grid = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=np.int8)
        grid[inflated] = 50          # inflated zone — grey
        grid[confirmed_occ] = 100    # real obstacle — black
        grid[(self.hit_count == 0) & (self.log_odds == 0.0)] = -1  # unknown

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
        self.inflated_map_pub.publish(msg)

    def publish_nav_path(self):
        """Publish the current planned waypoints as a LINE_STRIP marker."""
        ma = MarkerArray()
        # Clear old path
        del_m = Marker()
        del_m.header.stamp    = self.get_clock().now().to_msg()
        del_m.header.frame_id = 'odom'
        del_m.ns              = 'nav_path'
        del_m.action          = Marker.DELETEALL
        ma.markers.append(del_m)

        if self.nav_waypoints:
            m = Marker()
            m.header.stamp    = self.get_clock().now().to_msg()
            m.header.frame_id = 'odom'
            m.ns              = 'nav_path'
            m.id              = 0
            m.type            = Marker.LINE_STRIP
            m.action          = Marker.ADD
            m.scale.x         = 0.06
            m.color           = ColorRGBA(r=0.0, g=0.6, b=1.0, a=1.0)
            m.pose.orientation.w = 1.0
            for gx, gy in self.nav_waypoints:
                wx, wy = self.grid_to_world(gx, gy)
                p = Point(); p.x = wx; p.y = wy; p.z = 0.05
                m.points.append(p)
            ma.markers.append(m)

        self.nav_path_pub.publish(ma)

    # ── State machine ─────────────────────────────────────────────────────────

    def _start_scan(self):
        self.scan_accumulated  = 0.0
        self.scan_last_heading = None
        self.scan_settle_start = None
        self.state = ExploreState.SCANNING
        self.get_logger().info('[SCAN] Starting 360 rotation scan...')

    def _stop_base(self):
        self.last_cmd_angular = 0.0
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
                f'[SCAN] 360 done '
                f'({np.degrees(self.scan_accumulated):.0f} accumulated). '
                f'Settling {self.SCAN_SETTLE_TIME} s...')
        else:
            cmd = Twist()
            cmd.angular.z = self.SCAN_ANGULAR_VEL
            self.last_cmd_angular = cmd.angular.z
            self.cmd_vel_pub.publish(cmd)

    def _state_selecting(self):
        # ── If object confirmed, try to approach it first ─────────────
        if self.object_world_pos is not None:
            if self._plan_approach():
                return

        # ── If object was glimpsed (unconfirmed), try approaching that ─
        if self.object_world_pos is None and self.last_detection_pos is not None:
            self.get_logger().info(
                f'[SELECT] Trying unconfirmed sighting at '
                f'({self.last_detection_pos[0]:.2f}, '
                f'{self.last_detection_pos[1]:.2f})')
            self.object_world_pos = self.last_detection_pos
            if self._plan_approach():
                return
            # Couldn't path there — clear and fall through to frontiers
            self.object_world_pos = None

        frontiers = self.find_frontiers()
        self.publish_frontiers(frontiers)

        if not frontiers:
            self.no_frontier_count += 1
            if self.no_frontier_count >= self.NO_FRONTIER_LIMIT:
                self.get_logger().info('[SELECT] Exploration complete — no object found.')
                self.state = ExploreState.COMPLETE
            else:
                self.get_logger().info(
                    f'[SELECT] No frontiers '
                    f'({self.no_frontier_count}/{self.NO_FRONTIER_LIMIT}) '
                    f'— re-scanning.')
                self._start_scan()
            return

        self.no_frontier_count = 0

        base_pose = self.get_base_pose()
        if base_pose is None:
            return
        bx, by, _ = base_pose
        robot_gx, robot_gy = self.world_to_grid(bx, by)

        clearances = [None, self.ROBOT_CLEARANCE * 0.5]
        labels     = ['full', 'half']

        for clearance, label in zip(clearances, labels):
            for wx, wy, size, score in frontiers:
                if clearance is None:
                    obj_info = ''
                    if self.last_detection_pos is not None:
                        d2o = np.hypot(wx - self.last_detection_pos[0],
                                       wy - self.last_detection_pos[1])
                        obj_info = f' dist_obj={d2o:.1f}'
                    self.get_logger().info(
                        f'[SELECT] Trying frontier ({wx:.2f}, {wy:.2f}) '
                        f'cluster={size} score={score:.1f}{obj_info}')

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
                self.nav_clearance    = clearance
                self.publish_nav_path()
                self.state = ExploreState.NAVIGATING
                return

        self.get_logger().warn(
            '[SELECT] No path to any frontier at any clearance — re-scanning.')
        self._start_scan()

    def _state_navigating(self):
        now = time.time()

        if now - self.nav_start_time > self.NAV_TIMEOUT:
            self._stop_base()
            self.get_logger().warn('[NAV] Timeout — re-selecting.')
            self.state = ExploreState.SELECTING
            return

        pose = self.get_base_pose()
        if pose is None:
            return
        bx, by, btheta = pose

        # ── Object detection handled by _yolo_bbox_cb callback ─────────
        if self.object_world_pos is not None:
            if self._plan_approach():
                return
            # Can't reach object yet — continue to frontier

        actual_heading = btheta - np.pi / 2

        if self._is_path_blocked():
            self.get_logger().info('[NAV] Path blocked on map — re-planning...')
            if self._replan_from_here():
                return
            else:
                self._stop_base()
                self.get_logger().warn('[NAV] Re-plan failed — re-selecting.')
                self.state = ExploreState.SELECTING
                return

        if self.nav_waypoint_idx >= len(self.nav_waypoints):
            self._stop_base()
            self.get_logger().info('[NAV] All waypoints reached — starting scan.')
            self._start_scan()
            return

        wp_gx, wp_gy = self.nav_waypoints[self.nav_waypoint_idx]
        wp_wx, wp_wy = self.grid_to_world(wp_gx, wp_gy)
        dx, dy = wp_wx - bx, wp_wy - by
        dist = np.hypot(dx, dy)

        if dist < self.WAYPOINT_TOLERANCE:
            self.nav_waypoint_idx += 1
            self.nav_start_time = time.time()
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

        if not hasattr(self, '_nav_last_log') or now - self._nav_last_log > 2.0:
            self._nav_last_log = now
            self.get_logger().info(
                f'[NAV] wp {self.nav_waypoint_idx}/{len(self.nav_waypoints)} '
                f'dist={dist:.2f} hdg_err={np.degrees(heading_error):.0f} '
                f'pos=({bx:.2f},{by:.2f})')

        cmd = Twist()

        clearance, distances = self._analyze_depth_obstacles()
        left_clear, center_clear, right_clear = clearance
        left_dist, center_dist, right_dist = distances
        min_dist = min(left_dist, center_dist, right_dist)

        if not center_clear:
            if self.depth_steer_start is None:
                self.depth_steer_start = now
            elif now - self.depth_steer_start > self.DEPTH_STEER_TIMEOUT:
                self.get_logger().info(
                    '[NAV] Stuck on depth obstacle — forcing re-plan.')
                self.depth_steer_start = None
                self._stop_base()
                if self._replan_from_here():
                    return
                self.get_logger().warn('[NAV] Re-plan failed — re-selecting.')
                self.state = ExploreState.SELECTING
                return
            self.get_logger().info(
                f'[NAV] Depth obstacle: L={left_dist:.2f} C={center_dist:.2f} '
                f'R={right_dist:.2f} — steering')
            if left_dist > right_dist:
                cmd.angular.z = 1.0
            elif right_dist > left_dist:
                cmd.angular.z = -1.0
            else:
                cmd.angular.z = 1.0
            cmd.linear.x = 0.0
        else:
            self.depth_steer_start = None
            alignment = max(0.0, np.cos(heading_error))
            speed = self.NAV_LINEAR_SPEED * alignment
            if min_dist < 1.5:
                speed *= min(min_dist / 1.5, 1.0)
            cmd.linear.x = float(min(speed, dist * 0.5))
            cmd.angular.z = float(np.clip(2.0 * heading_error, -1.2, 1.2))

        if min_dist < 0.35:
            cmd.linear.x = 0.0

        self.last_cmd_angular = cmd.angular.z
        self.cmd_vel_pub.publish(cmd)

    def _state_approaching(self):
        """Navigate to the detected object's world position."""
        now = time.time()

        if now - self.nav_start_time > self.NAV_TIMEOUT:
            self._stop_base()
            self.get_logger().warn('[APPROACH] Timeout — re-selecting.')
            self.state = ExploreState.SELECTING
            return

        pose = self.get_base_pose()
        if pose is None:
            return
        bx, by, btheta = pose
        actual_heading = btheta - np.pi / 2

        # ── Refine object position while approaching ──────────────
        # Re-detect the object to update position estimate as we get closer
        # (depth accuracy improves at shorter range).
        # if self.latest_rgb is not None and self.camera_K is not None:
        #     hsv = cv2.cvtColor(self.latest_rgb, cv2.COLOR_BGR2HSV)
        #     mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        #     for lower, upper in COLOR_RANGES.get(self.target_color, []):
        #         mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
        #     contours, _ = cv2.findContours(
        #         mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #     if contours:
        #         largest = max(contours, key=cv2.contourArea)
        #         area = cv2.contourArea(largest)
                # if area >= self.MIN_DETECTION_AREA:
                #     M = cv2.moments(largest)
                #     if M['m00'] > 0:
                #         cu = int(M['m10'] / M['m00'])
                #         cv_pt = int(M['m01'] / M['m00'])
                #         new_pos = self._estimate_object_position(cu, cv_pt)
        if hasattr(self, 'latest_yolo_detection') and self.latest_yolo_detection is not None:
            det = self.latest_yolo_detection
            u = int(det.bbox.center.position.x)
            v = int(det.bbox.center.position.y)
            
            new_pos = self._estimate_object_position(u, v)
            if new_pos is not None:
                old_ox, old_oy = self.object_world_pos
                shift = np.hypot(
                    new_pos[0] - old_ox, new_pos[1] - old_oy)
                # Only update if shift is small; depth back-
                # projection is noisy at range so cap at 0.5m.
                # Apply EMA smoothing to prevent single noisy
                # readings from jumping the estimate.
                if shift < 0.5:
                    alpha = 0.4  # blend weight for new reading
                    smoothed = (
                        alpha * new_pos[0] + (1 - alpha) * old_ox,
                        alpha * new_pos[1] + (1 - alpha) * old_oy,
                    )
                    self.object_world_pos = smoothed
                    new_pos = smoothed
                    shift = np.hypot(
                        smoothed[0] - old_ox,
                        smoothed[1] - old_oy)
                    if shift > 0.15:
                        self.get_logger().info(
                            f'[APPROACH] Refined position: '
                            f'({new_pos[0]:.2f}, {new_pos[1]:.2f}) '
                            f'shift={shift:.2f}m')
                        # Re-plan path to updated position
                        goal_gx, goal_gy = self.world_to_grid(
                            new_pos[0], new_pos[1])
                        robot_gx, robot_gy = self.world_to_grid(
                            bx, by)
                        for cl in [None, self.ROBOT_CLEARANCE * 0.5]:
                            path = self._plan_path(
                                robot_gx, robot_gy,
                                goal_gx, goal_gy,
                                clearance_m=cl)
                            if path is not None:
                                self.nav_waypoints = path
                                self.nav_waypoint_idx = 0
                                self.nav_clearance = cl
                                self.publish_nav_path()
                                break

        # Check distance to object
        ox, oy = self.object_world_pos
        dist_to_obj = np.hypot(ox - bx, oy - by)
        if dist_to_obj < self.APPROACH_DIST:
            self._stop_base()
            self.get_logger().info(
                f'[APPROACH] Reached target object! '
                f'dist={dist_to_obj:.2f}m')
            self.state = ExploreState.COMPLETE
            return

        # Re-plan if path blocked — rate-limited to avoid thrashing
        if (self._is_path_blocked() and
                now - self.last_approach_replan > 2.0):
            self.last_approach_replan = now
            self.get_logger().info('[APPROACH] Path blocked — re-planning...')
            if not self._replan_from_here():
                self._stop_base()
                self.get_logger().warn(
                    '[APPROACH] Re-plan failed — re-selecting.')
                self.state = ExploreState.SELECTING
                return

        if self.nav_waypoint_idx >= len(self.nav_waypoints):
            # All waypoints reached but not within APPROACH_DIST
            if dist_to_obj < self.APPROACH_DIST:
                self._stop_base()
                self.state = ExploreState.COMPLETE
                return
            # Try re-planning closer
            goal_gx, goal_gy = self.world_to_grid(ox, oy)
            robot_gx, robot_gy = self.world_to_grid(bx, by)
            for clearance in [None, self.ROBOT_CLEARANCE * 0.5]:
                path = self._plan_path(
                    robot_gx, robot_gy, goal_gx, goal_gy,
                    clearance_m=clearance)
                if path is not None:
                    self.nav_waypoints = path
                    self.nav_waypoint_idx = 0
                    self.nav_start_time = time.time()
                    self.nav_clearance = clearance
                    self.publish_nav_path()
                    return
            # Give up
            self._stop_base()
            self.get_logger().info(
                f'[APPROACH] Close but not within {self.APPROACH_DIST}m '
                f'(dist={dist_to_obj:.2f}m) — declaring complete.')
            self.state = ExploreState.COMPLETE
            return

        wp_gx, wp_gy = self.nav_waypoints[self.nav_waypoint_idx]
        wp_wx, wp_wy = self.grid_to_world(wp_gx, wp_gy)
        dx, dy = wp_wx - bx, wp_wy - by
        dist = np.hypot(dx, dy)

        if dist < self.WAYPOINT_TOLERANCE:
            self.nav_waypoint_idx += 1
            self.nav_start_time = time.time()
            if self.nav_waypoint_idx >= len(self.nav_waypoints):
                if dist_to_obj < self.APPROACH_DIST:
                    self._stop_base()
                    self.get_logger().info(
                        f'[APPROACH] Reached target object! '
                        f'dist={dist_to_obj:.2f}m')
                    self.state = ExploreState.COMPLETE
                    return
                # Might need to re-plan — will handle on next tick
                return
            wp_gx, wp_gy = self.nav_waypoints[self.nav_waypoint_idx]
            wp_wx, wp_wy = self.grid_to_world(wp_gx, wp_gy)
            dx, dy = wp_wx - bx, wp_wy - by
            dist = np.hypot(dx, dy)

        desired_heading = np.arctan2(dy, dx)
        heading_error = self._normalize_angle(desired_heading - actual_heading)

        if not hasattr(self, '_approach_last_log') or now - self._approach_last_log > 2.0:
            self._approach_last_log = now
            self.get_logger().info(
                f'[APPROACH] wp {self.nav_waypoint_idx}/{len(self.nav_waypoints)} '
                f'dist_obj={dist_to_obj:.2f} pos=({bx:.2f},{by:.2f})')

        cmd = Twist()

        clearance, distances = self._analyze_depth_obstacles()
        left_clear, center_clear, right_clear = clearance
        left_dist, center_dist, right_dist = distances
        min_dist = min(left_dist, center_dist, right_dist)

        # During approach, trust the A* path for obstacle avoidance.
        # Depth steering is skipped here because the floor and the target
        # object itself cause persistent false "center blocked" readings.
        # A hard safety stop is kept for truly close obstacles.
        alignment = max(0.0, np.cos(heading_error))
        speed = self.NAV_LINEAR_SPEED * alignment
        if min_dist < 1.5:
            speed *= min(min_dist / 1.5, 1.0)
        cmd.linear.x = float(min(speed, dist * 0.5))
        cmd.angular.z = float(np.clip(2.0 * heading_error, -1.2, 1.2))

        if min_dist < 0.35:
            cmd.linear.x = 0.0

        self.last_cmd_angular = cmd.angular.z
        self.cmd_vel_pub.publish(cmd)

    def _state_complete(self):
        self._stop_base()
        self.publish_map()
        if self.object_world_pos is not None:
            self.publish_object_marker()

        free     = int(np.sum(self.log_odds <= self.LOG_ODDS_FREE_THRESH))
        occupied = int(np.sum(
            (self.log_odds  >= self.LOG_ODDS_OCC_THRESH) &
            (self.hit_count >= self.MIN_HIT_COUNT)))
        total    = self.GRID_SIZE ** 2

        self.get_logger().info('=' * 55)
        if self.object_world_pos is not None:
            ox, oy = self.object_world_pos
            self.get_logger().info(
                f'FOUND TARGET OBJECT at '
                f'({ox:.2f}, {oy:.2f})')
        else:
            self.get_logger().info('EXPLORATION COMPLETE — object not found')
        self.get_logger().info(f'  Coverage : {(free+occupied)/total*100:.1f}%')
        self.get_logger().info(f'  Free     : {free}')
        self.get_logger().info(f'  Confirmed occupied : {occupied}')
        self.get_logger().info('=' * 55)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1.0)
            self.publish_map()
            if self.object_world_pos is not None:
                self.publish_object_marker()

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

        self.get_logger().info(
            f'Sensors ready — exploring for YOLO class_id={self.target_class_id}!')
        self._start_scan()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            now = time.time()
            if now - self.last_map_publish >= 1.0 / self.MAP_PUBLISH_RATE:
                self.publish_map()
                if self.object_world_pos is not None:
                    self.publish_object_marker()
                self.last_map_publish = now

            # Object detection now handled by _yolo_bbox_cb callback.

            if   self.state == ExploreState.SCANNING:
                self._state_scanning()
            elif self.state == ExploreState.SELECTING:
                self._state_selecting()
            elif self.state == ExploreState.NAVIGATING:
                self._state_navigating()
            elif self.state == ExploreState.APPROACHING:
                self._state_approaching()
            elif self.state == ExploreState.COMPLETE:
                self._state_complete()
                break


def main(args=None):
    rclpy.init(args=args)
    node = ExploreAndFind()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
