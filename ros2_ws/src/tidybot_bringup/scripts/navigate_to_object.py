#!/usr/bin/env python3
"""
Navigate to Object Module

Searches for a target object (e.g., 'banana', 'apple') by scanning the environment,
explores with frontier-based navigation and obstacle avoidance using a depth-based
occupancy grid, and positions the robot within grasping range.

Uses YOLO object detection via YOLOObjectDetector for class-based recognition.

Internal sub-states: INIT → SCAN → EXPLORE → APPROACH → ALIGN → POSITIONED

Subscribes (via YOLOObjectDetector):
    /camera/color/image_raw (Image) - RGB for YOLO detection
    /camera/depth/image_raw (Image) - depth for obstacle avoidance + 3D localization
    /camera/color/camera_info (CameraInfo) - intrinsics

Publishes:
    /cmd_vel (Twist) - base velocity
    /camera/pan_tilt_cmd (Float64MultiArray) - camera pointing

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
from scipy.ndimage import binary_dilation

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import OccupancyGrid, MapMetaData

import tf2_ros

from tidybot_perception.yolo_object_detector import YOLOObjectDetector


# ═══════════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════════

# Scanning
SCAN_ROTATION_SPEED = 3.0      # rad/s while scanning  (bridge limit: 1.5)
CAMERA_TILT_NAV = 0.3          # slight downward tilt for navigation

# Exploration
EXPLORE_SPEED = 6.00           # m/s while exploring  (bridge limit: 0.5)
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
# Occupancy Grid  (matches arena: X [-5.0, 5.0], Y [-1.5, 10.5])
# ═══════════════════════════════════════════════════════════════════════
GRID_RESOLUTION = 0.05         # meters per cell
GRID_SIZE = 300                # cells per side (300 * 0.05 = 15m — covers 10x12m arena with margin)
GRID_ORIGIN_X = -7.5           # bottom-left corner in odom (meters)
GRID_ORIGIN_Y = -3.0
MAX_DEPTH_RANGE = 5.0          # meters — ignore depth beyond this
DEPTH_SUBSAMPLE = 4            # process every Nth pixel for speed
MIN_FRONTIER_SIZE = 10         # minimum cells in a frontier cluster
FRONTIER_NAV_OFFSET = 0.3      # navigate slightly in front of frontier (meters)
OBSTACLE_INFLATION_RADIUS = 6  # cells to inflate around obstacles (6 * 0.05m = 0.30m buffer)


class NavState(Enum):
    """Internal navigation sub-states."""
    INIT       = auto()
    SCAN       = auto()
    EXPLORE    = auto()
    APPROACH   = auto()
    ALIGN      = auto()
    POSITIONED = auto()


# ═══════════════════════════════════════════════════════════════════════
# Simple Occupancy Grid
# ═══════════════════════════════════════════════════════════════════════

class SimpleOccupancyGrid:
    """2D occupancy grid built from depth images for frontier exploration."""

    UNKNOWN = -1
    FREE = 0
    OCCUPIED = 100

    def __init__(self):
        self.grid = np.full((GRID_SIZE, GRID_SIZE), self.UNKNOWN, dtype=np.int8)

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

    def update_from_depth(self, depth_image, camera_K, R_cam_to_odom, t_cam_to_odom):
        """Project depth image into odom frame and update grid.

        Args:
            depth_image: 16UC1 depth in mm (480x640)
            camera_K: 3x3 intrinsic matrix
            R_cam_to_odom: 3x3 rotation from camera optical frame to odom
            t_cam_to_odom: (3,) translation from camera optical frame to odom
        """
        h, w = depth_image.shape
        step = DEPTH_SUBSAMPLE

        fx, fy = camera_K[0, 0], camera_K[1, 1]
        cx, cy = camera_K[0, 2], camera_K[1, 2]

        # Camera origin in grid
        cam_gx, cam_gy = self.world_to_grid(t_cam_to_odom[0], t_cam_to_odom[1])

        # Vectorized depth projection (subsampled)
        vs, us = np.mgrid[0:h:step, 0:w:step]
        us = us.ravel()
        vs = vs.ravel()
        depths_mm = depth_image[vs, us].astype(np.float64)

        valid = (depths_mm > 0) & (depths_mm < MAX_DEPTH_RANGE * 1000)
        us = us[valid]
        vs = vs[valid]
        depths_m = depths_mm[valid] / 1000.0

        if len(depths_m) == 0:
            return

        # Project to camera optical frame
        x_cam = (us - cx) * depths_m / fx
        y_cam = (vs - cy) * depths_m / fy
        z_cam = depths_m

        # Transform to odom frame
        points_cam = np.stack([x_cam, y_cam, z_cam], axis=1)
        points_odom = (R_cam_to_odom @ points_cam.T).T + t_cam_to_odom

        # Convert to grid coordinates
        gxs = ((points_odom[:, 0] - GRID_ORIGIN_X) / GRID_RESOLUTION).astype(int)
        gys = ((points_odom[:, 1] - GRID_ORIGIN_Y) / GRID_RESOLUTION).astype(int)

        # Filter: obstacle height range (above floor, below ceiling)
        ground_mask = (points_odom[:, 2] > -0.05) & (points_odom[:, 2] < 0.8)
        in_bounds = (gxs >= 0) & (gxs < GRID_SIZE) & (gys >= 0) & (gys < GRID_SIZE)
        valid_mask = ground_mask & in_bounds

        # Mark occupied cells
        occ_gxs = gxs[valid_mask]
        occ_gys = gys[valid_mask]
        self.grid[occ_gys, occ_gxs] = self.OCCUPIED

        # Raytrace free space (subsample for performance)
        ray_step = max(1, len(occ_gxs) // 500)
        for i in range(0, len(occ_gxs), ray_step):
            self._raytrace_free(cam_gx, cam_gy, occ_gxs[i], occ_gys[i])

        # Mark cells near camera as free
        if self.in_bounds(cam_gx, cam_gy):
            for dy in range(-3, 4):
                for dx in range(-3, 4):
                    nx, ny = cam_gx + dx, cam_gy + dy
                    if self.in_bounds(nx, ny) and self.grid[ny, nx] == self.UNKNOWN:
                        self.grid[ny, nx] = self.FREE

    def _raytrace_free(self, x0, y0, x1, y1):
        """Bresenham's line: mark cells as FREE from (x0,y0) to just before (x1,y1)."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0

        while True:
            if x == x1 and y == y1:
                break
            if self.in_bounds(x, y) and self.grid[y, x] != self.OCCUPIED:
                self.grid[y, x] = self.FREE
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def get_inflated_grid(self):
        """Return a copy of the grid with obstacles inflated by OBSTACLE_INFLATION_RADIUS cells.

        Inflated cells are marked as OCCUPIED so the robot keeps a buffer zone.
        """
        if OBSTACLE_INFLATION_RADIUS <= 0:
            return self.grid.copy()

        inflated = self.grid.copy()
        occupied_mask = self.grid == self.OCCUPIED

        # Create circular structuring element for natural-looking inflation
        r = OBSTACLE_INFLATION_RADIUS
        y, x = np.ogrid[-r:r + 1, -r:r + 1]
        struct = (x * x + y * y) <= r * r

        dilated = binary_dilation(occupied_mask, structure=struct)

        # Only inflate into non-occupied cells (don't overwrite existing FREE→OCCUPIED
        # where the raw grid says FREE, but do mark them OCCUPIED in the inflated copy)
        inflated[dilated] = self.OCCUPIED

        return inflated

    def find_best_frontier(self, robot_x, robot_y, robot_heading):
        """Find the best frontier to explore.

        Args:
            robot_x, robot_y: Robot position in odom frame.
            robot_heading: Robot heading (actual world heading, not MuJoCo theta).

        Returns:
            (goal_x, goal_y) in odom frame, or None if no frontiers.
        """
        # Use inflated grid so frontiers stay outside buffer zones
        inflated = self.get_inflated_grid()

        # Find frontier cells: FREE cells (in inflated grid) adjacent to UNKNOWN
        free_mask = inflated == self.FREE
        unknown_mask = inflated == self.UNKNOWN
        frontier_mask = np.zeros_like(inflated, dtype=bool)

        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            shifted = np.roll(np.roll(unknown_mask, dy, axis=0), dx, axis=1)
            frontier_mask |= (free_mask & shifted)

        frontier_ys, frontier_xs = np.where(frontier_mask)
        if len(frontier_xs) == 0:
            return None

        # Cluster via BFS
        visited = set()
        clusters = []
        frontier_set = set(zip(frontier_xs.tolist(), frontier_ys.tolist()))

        for fx, fy in frontier_set:
            if (fx, fy) in visited:
                continue
            cluster = []
            queue = deque([(fx, fy)])
            visited.add((fx, fy))
            while queue:
                cx, cy = queue.popleft()
                cluster.append((cx, cy))
                for ddx, ddy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                                 (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                    nx, ny = cx + ddx, cy + ddy
                    if (nx, ny) in frontier_set and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        queue.append((nx, ny))

            if len(cluster) >= MIN_FRONTIER_SIZE:
                clusters.append(cluster)

        if not clusters:
            return None

        # Score clusters: prefer large ones nearby, bias toward heading
        best_score = -1
        best_goal = None

        for cluster in clusters:
            xs = [c[0] for c in cluster]
            ys = [c[1] for c in cluster]
            mean_gx = np.mean(xs)
            mean_gy = np.mean(ys)
            wx, wy = self.grid_to_world(int(mean_gx), int(mean_gy))

            dx = wx - robot_x
            dy = wy - robot_y
            dist = np.sqrt(dx**2 + dy**2)
            if dist < 0.3:
                continue  # too close

            # Heading bonus: prefer frontiers in the direction we're facing
            angle_to_frontier = np.arctan2(dy, dx)
            angle_diff = abs(_normalize_angle(angle_to_frontier - robot_heading))
            heading_bonus = 1.0 + 0.5 * (1.0 - angle_diff / np.pi)

            score = (len(cluster) * heading_bonus) / max(dist, 0.5)
            if score > best_score:
                best_score = score
                # Navigate to a point offset back toward the robot
                if dist > FRONTIER_NAV_OFFSET * 2:
                    ratio = (dist - FRONTIER_NAV_OFFSET) / dist
                    best_goal = (robot_x + dx * ratio, robot_y + dy * ratio)
                else:
                    best_goal = (wx, wy)

        return best_goal


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
    Navigate to a target object using YOLO detection, frontier
    exploration, and reactive obstacle avoidance.

    Can be run standalone or used by the state machine.
    """

    def __init__(self, target_object: str = 'banana'):
        super().__init__('navigate_to_object')

        self.target_object = target_object

        # ── Perception (YOLO detection + depth localization) ──
        self.detector = YOLOObjectDetector(self)

        # ── TF2 (for odom→base_link lookups) ──
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Publishers ──
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        # ── Occupancy grid ──
        self.occ_grid = SimpleOccupancyGrid()

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
        self.occ_grid = SimpleOccupancyGrid()
        self.prev_yaw = None
        self.total_rotated = 0.0
        self.scan_count = 0
        self.explore_goal = None
        self.target_base_pt = None
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

    def _get_camera_transform(self):
        """Get rotation matrix and translation for camera→odom transform.

        Returns (R, t) or (None, None) on failure.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'camera_depth_optical_frame',
                rclpy.time.Time(), timeout=Duration(seconds=0.5))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None, None

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

    def _analyze_depth_obstacles(self):
        """Analyze depth image for obstacles in left/center/right sectors.

        Returns:
            (clearance, distances) where each is (left, center, right).
            clearance: tuple of bools (True = clear).
            distances: tuple of floats (meters to nearest obstacle).
        """
        depth = self.detector.latest_depth
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
        """Update the occupancy grid from the latest depth image."""
        depth = self.detector.latest_depth
        if depth is None or self.detector.camera_info is None:
            return

        R, t = self._get_camera_transform()
        if R is None:
            return

        camera_K = np.array(self.detector.camera_info.k).reshape(3, 3)
        self.occ_grid.update_from_depth(depth, camera_K, R, t)

    def _publish_map(self):
        """Publish the occupancy grid to /map for RViz visualization."""
        now = time.time()
        if now - self.last_map_publish_time < self.map_publish_interval:
            return
        self.last_map_publish_time = now

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        msg.info = MapMetaData()
        msg.info.resolution = GRID_RESOLUTION
        msg.info.width = GRID_SIZE
        msg.info.height = GRID_SIZE
        msg.info.origin.position.x = float(GRID_ORIGIN_X)
        msg.info.origin.position.y = float(GRID_ORIGIN_Y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Publish inflated grid so buffer zones are visible in RViz
        msg.data = self.occ_grid.get_inflated_grid().ravel().tolist()
        self.map_pub.publish(msg)

    def _transition_to(self, new_state):
        """Transition to a new state with logging."""
        old = self.nav_state.name
        self.nav_state = new_state
        self.state_start_time = time.time()
        self.get_logger().info(f'[NAV] {old} → {new_state.name}')

    # ═══════════════════════════════════════════════════════════════════
    # State handlers
    # ═══════════════════════════════════════════════════════════════════

    def _handle_init(self):
        """Wait for sensor data and TF to be available."""
        if (self.detector.latest_rgb is None
                or self.detector.latest_depth is None
                or self.detector.camera_info is None):
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

        self._transition_to(NavState.SCAN)

    def _handle_scan(self):
        """Rotate 360° in place scanning for the target color."""
        # Check for target on every tick
        detection = self.detector.detect_and_localize(self.target_object)
        if detection is not None:
            pixel, base_pt = detection
            self.target_base_pt = base_pt
            self.last_detection_time = time.time()
            self._stop_base()
            self.get_logger().info(
                f'[SCAN] Detected {self.target_object} at pixel {pixel}, '
                f'base_link=({base_pt.point.x:.2f}, {base_pt.point.y:.2f}, '
                f'{base_pt.point.z:.2f})')
            self._transition_to(NavState.APPROACH)
            return

        # Rotate in place
        twist = Twist()
        twist.angular.z = SCAN_ROTATION_SPEED
        self.cmd_vel_pub.publish(twist)

        # Track rotation via yaw
        current_yaw = self._get_current_yaw()
        if current_yaw is not None and self.prev_yaw is not None:
            delta = _normalize_angle(current_yaw - self.prev_yaw)
            self.total_rotated += abs(delta)
        self.prev_yaw = current_yaw

        # Check if full 360° rotation is complete
        if self.total_rotated >= 2.0 * np.pi:
            self._stop_base()
            self.scan_count += 1
            self.get_logger().info(
                f'[SCAN] Full rotation #{self.scan_count}, no {self.target_object} found')

            if self.scan_count >= MAX_EXPLORE_SCANS:
                self.get_logger().warn(
                    f'[SCAN] Gave up after {self.scan_count} scans')
                self._stop_base()
                return

            # Reset scan tracking and transition to explore
            self.total_rotated = 0.0
            self.prev_yaw = self._get_current_yaw()
            self.explore_start_time = time.time()
            pose = self._get_base_pose_in_odom()
            self.explore_start_pos = (pose[0], pose[1]) if pose else None
            self.explore_goal = None
            self._transition_to(NavState.EXPLORE)

    def _handle_explore(self):
        """Explore the environment, building an occupancy grid and looking for the target."""
        # Always update the map
        self._update_occupancy_grid()

        # Check for target continuously while exploring
        detection = self.detector.detect(self.target_object)
        if detection is not None:
            full = self.detector.detect_and_localize(self.target_object)
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
            self.total_rotated = 0.0
            self.prev_yaw = self._get_current_yaw()
            self._send_pan_tilt(0.0, CAMERA_TILT_NAV)
            self._transition_to(NavState.SCAN)
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
        detection = self.detector.detect_and_localize(self.target_object)
        if detection is not None:
            pixel, base_pt = detection
            self.target_base_pt = base_pt
            self.last_detection_time = time.time()

        # Check if we've lost the object
        if self.target_base_pt is None or \
                time.time() - self.last_detection_time > OBJECT_LOST_TIMEOUT:
            self._stop_base()
            self.get_logger().warn('[APPROACH] Lost object, returning to SCAN')
            self.total_rotated = 0.0
            self.prev_yaw = self._get_current_yaw()
            self._transition_to(NavState.SCAN)
            return

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
        pixel = self.detector.detect(self.target_object)
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
        full = self.detector.detect_and_localize(self.target_object)
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
