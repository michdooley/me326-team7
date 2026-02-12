#!/usr/bin/env python3
"""
TidyBot2 Frontier Exploration

Autonomously explores a walled environment using depth-based occupancy mapping,
frontier detection, and color-based object detection. Publishes an occupancy grid
and detected object markers to RViz.

Prerequisites:
    ros2 launch tidybot_bringup sim.launch.py scene:=scene_obstacles.xml

Usage:
    ros2 run tidybot_bringup frontier_exploration.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration

import numpy as np
import time
from enum import Enum
from collections import deque

from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool, Float64MultiArray, ColorRGBA

import tf2_ros

try:
    from cv_bridge import CvBridge
    import cv2
except ImportError:
    print("ERROR: cv_bridge and opencv are required.")
    raise


class ExploreState(Enum):
    SCANNING = 0
    SELECTING_FRONTIER = 1
    NAVIGATING = 2
    COMPLETE = 3


# ── Color definitions for cube detection ──────────────────────────────────
COLOR_DEFS = {
    'red': {
        'hsv_ranges': [
            (np.array([0, 100, 50]), np.array([10, 255, 255])),
            (np.array([170, 100, 50]), np.array([180, 255, 255])),
        ],
        'rgba': (0.8, 0.2, 0.2, 1.0),
    },
    'blue': {
        'hsv_ranges': [
            (np.array([100, 100, 50]), np.array([130, 255, 255])),
        ],
        'rgba': (0.2, 0.3, 0.8, 1.0),
    },
    'green': {
        'hsv_ranges': [
            (np.array([35, 100, 50]), np.array([85, 255, 255])),
        ],
        'rgba': (0.2, 0.7, 0.3, 1.0),
    },
    'yellow': {
        'hsv_ranges': [
            (np.array([20, 100, 100]), np.array([35, 255, 255])),
        ],
        'rgba': (0.9, 0.8, 0.1, 1.0),
    },
    'orange': {
        'hsv_ranges': [
            (np.array([10, 100, 100]), np.array([20, 255, 255])),
        ],
        'rgba': (0.9, 0.5, 0.1, 1.0),
    },
}


class FrontierExplorer(Node):
    """Frontier-based exploration with occupancy mapping and object detection."""

    # ── Grid parameters ──
    GRID_RESOLUTION = 0.05  # meters per cell
    GRID_SIZE = 300         # cells per side (300 * 0.05 = 15m)
    GRID_ORIGIN_X = -7.5    # meters (bottom-left corner in odom)
    GRID_ORIGIN_Y = -7.5

    # ── Mapping parameters ──
    MAX_DEPTH_RANGE = 5.0     # meters - ignore depth beyond this
    DEPTH_SUBSAMPLE = 4       # process every Nth pixel
    MAP_PUBLISH_RATE = 2.0    # Hz

    # ── Frontier parameters ──
    MIN_FRONTIER_SIZE = 10    # minimum cells in a frontier cluster
    FRONTIER_NAV_OFFSET = 0.3 # navigate slightly in front of frontier (meters)

    # ── Exploration parameters ──
    SCAN_PAN_ANGLES = [-1.2, 0.0, 1.2]  # camera pan positions for scanning
    SCAN_TILT = 0.3          # slight downward tilt for scanning
    SCAN_DWELL_TIME = 2.5    # seconds at each pan position
    NAV_TIMEOUT = 20.0       # seconds to wait for navigation

    # Cell values
    UNKNOWN = -1
    FREE = 0
    OCCUPIED = 100

    def __init__(self):
        super().__init__('frontier_explorer')

        # ── Occupancy grid ──
        self.grid = np.full((self.GRID_SIZE, self.GRID_SIZE), self.UNKNOWN, dtype=np.int8)

        # ── Object detections: {color: list of (x, y, z) in odom} ──
        self.detected_objects = {}

        # ── State machine ──
        self.state = ExploreState.SCANNING
        self.scan_index = 0
        self.scan_start_time = None
        self.nav_start_time = None
        self.no_frontier_count = 0

        # ── Camera data ──
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_K = None
        self.cv_bridge = CvBridge()

        # ── Navigation ──
        self.base_goal_reached = False

        # ── Timing ──
        self.last_map_publish = 0.0
        self.last_detection_time = 0.0

        # ── TF2 ──
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscribers ──
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self._rgb_cb, qos)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self._depth_cb, qos)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._camera_info_cb, qos)
        self.goal_sub = self.create_subscription(
            Bool, '/base/goal_reached', self._goal_reached_cb, 10)

        # ── Publishers ──
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.objects_pub = self.create_publisher(MarkerArray, '/detected_objects', 10)
        self.frontier_pub = self.create_publisher(MarkerArray, '/frontier_markers', 10)
        self.pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10)
        self.base_target_pub = self.create_publisher(
            Pose2D, '/base/target_pose', 10)

        self.get_logger().info('=' * 50)
        self.get_logger().info('Frontier Explorer initialized')
        self.get_logger().info(f'  Grid: {self.GRID_SIZE}x{self.GRID_SIZE} at {self.GRID_RESOLUTION}m/cell')
        self.get_logger().info(f'  Area: {self.GRID_SIZE * self.GRID_RESOLUTION}m x {self.GRID_SIZE * self.GRID_RESOLUTION}m')
        self.get_logger().info('=' * 50)

    # ── Subscriber callbacks ──────────────────────────────────────────────

    def _rgb_cb(self, msg: Image):
        try:
            self.latest_rgb = self.cv_bridge.imgmsg_to_cv2(msg, 'rgb8')
        except Exception as e:
            self.get_logger().warn(f'RGB conversion failed: {e}')

    def _depth_cb(self, msg: Image):
        try:
            self.latest_depth = self.cv_bridge.imgmsg_to_cv2(msg, '16UC1')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')

    def _camera_info_cb(self, msg: CameraInfo):
        self.camera_K = np.array(msg.k).reshape(3, 3)

    def _goal_reached_cb(self, msg: Bool):
        if msg.data:
            self.base_goal_reached = True

    # ── Helpers ────────────────────────────────────────────────────────────

    def spin_for(self, seconds: float):
        """Spin the node for a given duration."""
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def send_pan_tilt(self, pan: float, tilt: float):
        msg = Float64MultiArray()
        msg.data = [pan, tilt]
        self.pan_tilt_pub.publish(msg)

    def world_to_grid(self, wx: float, wy: float):
        """Convert world (odom) coords to grid indices."""
        gx = int((wx - self.GRID_ORIGIN_X) / self.GRID_RESOLUTION)
        gy = int((wy - self.GRID_ORIGIN_Y) / self.GRID_RESOLUTION)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int):
        """Convert grid indices to world (odom) coords (cell center)."""
        wx = self.GRID_ORIGIN_X + (gx + 0.5) * self.GRID_RESOLUTION
        wy = self.GRID_ORIGIN_Y + (gy + 0.5) * self.GRID_RESOLUTION
        return wx, wy

    def in_grid(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.GRID_SIZE and 0 <= gy < self.GRID_SIZE

    def get_base_pose_in_odom(self):
        """Get (x, y, theta) of base in odom frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time(), timeout=Duration(seconds=1.0))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'TF odom->base_link failed: {e}')
            return None

        t = transform.transform.translation
        q = transform.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = np.arctan2(siny, cosy)
        return (t.x, t.y, theta)

    # ── Depth → Occupancy Grid ────────────────────────────────────────────

    def update_occupancy_grid(self):
        """Project depth image into the occupancy grid."""
        if self.latest_depth is None or self.camera_K is None:
            return

        # Get camera-to-odom transform
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'camera_depth_optical_frame',
                rclpy.time.Time(), timeout=Duration(seconds=0.5))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        # Extract rotation matrix and translation
        t = transform.transform.translation
        q = transform.transform.rotation
        cam_origin = np.array([t.x, t.y, t.z])

        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
        ])

        fx = self.camera_K[0, 0]
        fy = self.camera_K[1, 1]
        cx = self.camera_K[0, 2]
        cy = self.camera_K[1, 2]

        depth = self.latest_depth
        h, w = depth.shape
        step = self.DEPTH_SUBSAMPLE

        # Camera origin in grid coords
        cam_gx, cam_gy = self.world_to_grid(cam_origin[0], cam_origin[1])

        # Vectorized depth projection for subsampled pixels
        vs, us = np.mgrid[0:h:step, 0:w:step]
        us = us.ravel()
        vs = vs.ravel()
        depths_mm = depth[vs, us].astype(np.float64)

        # Filter valid depths
        valid = (depths_mm > 0) & (depths_mm < self.MAX_DEPTH_RANGE * 1000)
        us = us[valid]
        vs = vs[valid]
        depths_m = depths_mm[valid] / 1000.0

        if len(depths_m) == 0:
            return

        # Project to camera frame
        x_cam = (us - cx) * depths_m / fx
        y_cam = (vs - cy) * depths_m / fy
        z_cam = depths_m

        # Transform to odom frame (vectorized)
        points_cam = np.stack([x_cam, y_cam, z_cam], axis=1)  # (N, 3)
        points_odom = (R @ points_cam.T).T + cam_origin  # (N, 3)

        # Convert to grid coordinates
        gxs = ((points_odom[:, 0] - self.GRID_ORIGIN_X) / self.GRID_RESOLUTION).astype(int)
        gys = ((points_odom[:, 1] - self.GRID_ORIGIN_Y) / self.GRID_RESOLUTION).astype(int)

        # Filter to points on the ground plane (z < 0.8m, above floor z > -0.05)
        ground_mask = (points_odom[:, 2] > -0.05) & (points_odom[:, 2] < 0.8)
        in_bounds = (gxs >= 0) & (gxs < self.GRID_SIZE) & (gys >= 0) & (gys < self.GRID_SIZE)
        valid_mask = ground_mask & in_bounds

        # Mark occupied cells
        occ_gxs = gxs[valid_mask]
        occ_gys = gys[valid_mask]
        self.grid[occ_gys, occ_gxs] = self.OCCUPIED

        # Raytrace free space from camera to each occupied cell
        # Use a subset for performance
        ray_step = max(1, len(occ_gxs) // 500)
        for i in range(0, len(occ_gxs), ray_step):
            self._raytrace_free(cam_gx, cam_gy, occ_gxs[i], occ_gys[i])

        # Also mark cells near camera as free
        if self.in_grid(cam_gx, cam_gy):
            radius = 3  # cells
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    nx, ny = cam_gx + dx, cam_gy + dy
                    if self.in_grid(nx, ny) and self.grid[ny, nx] == self.UNKNOWN:
                        self.grid[ny, nx] = self.FREE

    def _raytrace_free(self, x0: int, y0: int, x1: int, y1: int):
        """Bresenham's line: mark cells from (x0,y0) to (x1,y1) as FREE.
        Stop one cell before the endpoint (which is occupied)."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0

        while True:
            # Stop before the occupied endpoint
            if x == x1 and y == y1:
                break
            if self.in_grid(x, y) and self.grid[y, x] != self.OCCUPIED:
                self.grid[y, x] = self.FREE
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    # ── Object Detection ──────────────────────────────────────────────────

    def detect_objects(self):
        """Detect colored cubes in the RGB image and localize in odom."""
        if self.latest_rgb is None or self.latest_depth is None or self.camera_K is None:
            return

        bgr = cv2.cvtColor(self.latest_rgb, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # Get camera transform
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'camera_color_optical_frame',
                rclpy.time.Time(), timeout=Duration(seconds=0.5))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        t = transform.transform.translation
        q = transform.transform.rotation
        cam_origin = np.array([t.x, t.y, t.z])

        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
        ])

        fx = self.camera_K[0, 0]
        fy = self.camera_K[1, 1]
        cx_k = self.camera_K[0, 2]
        cy_k = self.camera_K[1, 2]

        for color_name, color_def in COLOR_DEFS.items():
            # Build mask from all HSV ranges for this color
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in color_def['hsv_ranges']:
                mask |= cv2.inRange(hsv, lower, upper)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area < 50:
                continue

            M = cv2.moments(largest)
            if M['m00'] == 0:
                continue
            px = int(M['m10'] / M['m00'])
            py = int(M['m01'] / M['m00'])

            # Get depth at detected pixel
            depth_mm = int(self.latest_depth[py, px])
            if depth_mm == 0 or depth_mm > self.MAX_DEPTH_RANGE * 1000:
                continue
            depth_m = depth_mm / 1000.0

            # Project to 3D camera frame
            x_cam = (px - cx_k) * depth_m / fx
            y_cam = (py - cy_k) * depth_m / fy
            z_cam = depth_m

            # Transform to odom
            point_cam = np.array([x_cam, y_cam, z_cam])
            point_odom = R @ point_cam + cam_origin

            # Deduplicate
            self._add_or_update_detection(color_name, point_odom)

    def _add_or_update_detection(self, color: str, position: np.ndarray):
        """Add new detection or update existing one if close enough."""
        dedup_dist = 0.15
        if color not in self.detected_objects:
            self.detected_objects[color] = []

        for i, existing in enumerate(self.detected_objects[color]):
            dist = np.linalg.norm(position[:2] - existing[:2])
            if dist < dedup_dist:
                # Running average update
                self.detected_objects[color][i] = 0.7 * existing + 0.3 * position
                return

        self.detected_objects[color].append(position.copy())
        self.get_logger().info(
            f'NEW {color} cube detected at ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})')

    # ── Frontier Detection ────────────────────────────────────────────────

    def find_frontiers(self):
        """Find frontier cells and cluster them.

        Returns:
            List of (centroid_wx, centroid_wy, cluster_size) sorted by score.
        """
        # Find frontier cells: FREE cells adjacent to UNKNOWN
        frontier_mask = np.zeros_like(self.grid, dtype=bool)

        free_mask = self.grid == self.FREE
        unknown_mask = self.grid == self.UNKNOWN

        # Check 4-connectivity for unknown neighbors
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            shifted = np.roll(np.roll(unknown_mask, dy, axis=0), dx, axis=1)
            frontier_mask |= (free_mask & shifted)

        # Get frontier cell coordinates
        frontier_ys, frontier_xs = np.where(frontier_mask)
        if len(frontier_xs) == 0:
            return []

        # Cluster using connected components via BFS
        visited = set()
        clusters = []
        frontier_set = set(zip(frontier_xs.tolist(), frontier_ys.tolist()))

        for fx, fy in frontier_set:
            if (fx, fy) in visited:
                continue
            # BFS to find connected component
            cluster = []
            queue = deque([(fx, fy)])
            visited.add((fx, fy))
            while queue:
                cx, cy = queue.popleft()
                cluster.append((cx, cy))
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                               (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                    nx, ny = cx + dx, cy + dy
                    if (nx, ny) in frontier_set and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        queue.append((nx, ny))

            if len(cluster) >= self.MIN_FRONTIER_SIZE:
                clusters.append(cluster)

        if not clusters:
            return []

        # Compute centroids and score
        base_pose = self.get_base_pose_in_odom()
        if base_pose is None:
            return []
        bx, by, _ = base_pose

        results = []
        for cluster in clusters:
            xs = [c[0] for c in cluster]
            ys = [c[1] for c in cluster]
            mean_gx = np.mean(xs)
            mean_gy = np.mean(ys)
            wx, wy = self.grid_to_world(int(mean_gx), int(mean_gy))

            dist = np.sqrt((wx - bx)**2 + (wy - by)**2)
            # Score: prefer larger frontiers, penalize very far ones
            score = len(cluster) / max(dist, 0.5)
            results.append((wx, wy, len(cluster), score))

        # Sort by score (highest first)
        results.sort(key=lambda r: r[3], reverse=True)
        return results

    # ── Publishing ────────────────────────────────────────────────────────

    def publish_map(self):
        """Publish the occupancy grid."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        msg.info = MapMetaData()
        msg.info.resolution = self.GRID_RESOLUTION
        msg.info.width = self.GRID_SIZE
        msg.info.height = self.GRID_SIZE
        msg.info.origin.position.x = float(self.GRID_ORIGIN_X)
        msg.info.origin.position.y = float(self.GRID_ORIGIN_Y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = self.grid.ravel().tolist()
        self.map_pub.publish(msg)

    def publish_objects(self):
        """Publish detected object markers."""
        marker_array = MarkerArray()
        marker_id = 0

        for color_name, positions in self.detected_objects.items():
            rgba = COLOR_DEFS[color_name]['rgba']
            for pos in positions:
                marker = Marker()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.header.frame_id = 'odom'
                marker.ns = 'detected_objects'
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = float(pos[0])
                marker.pose.position.y = float(pos[1])
                marker.pose.position.z = float(pos[2])
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.08
                marker.scale.y = 0.08
                marker.scale.z = 0.08
                marker.color = ColorRGBA(
                    r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
                marker.lifetime.sec = 0  # persistent
                marker_array.markers.append(marker)
                marker_id += 1

        # Add text labels
        for color_name, positions in self.detected_objects.items():
            for pos in positions:
                marker = Marker()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.header.frame_id = 'odom'
                marker.ns = 'object_labels'
                marker.id = marker_id
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD
                marker.pose.position.x = float(pos[0])
                marker.pose.position.y = float(pos[1])
                marker.pose.position.z = float(pos[2]) + 0.12
                marker.scale.z = 0.08
                marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                marker.text = color_name
                marker_array.markers.append(marker)
                marker_id += 1

        self.objects_pub.publish(marker_array)

    def publish_frontiers(self, frontiers):
        """Publish frontier centroids as markers for debug visualization."""
        marker_array = MarkerArray()

        for i, (wx, wy, size, score) in enumerate(frontiers):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'odom'
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = wx
            marker.pose.position.y = wy
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.2
            # Best frontier is bright green, others fade
            brightness = 1.0 if i == 0 else 0.4
            marker.color = ColorRGBA(r=0.0, g=brightness, b=0.3, a=0.7)
            marker.lifetime.sec = 5
            marker_array.markers.append(marker)

        self.frontier_pub.publish(marker_array)

    # ── State Machine ─────────────────────────────────────────────────────

    def run(self):
        """Main exploration loop."""
        self.get_logger().info('Waiting for camera data and TF...')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.latest_rgb is not None
                    and self.latest_depth is not None
                    and self.camera_K is not None):
                try:
                    self.tf_buffer.lookup_transform(
                        'odom', 'camera_depth_optical_frame',
                        rclpy.time.Time(), timeout=Duration(seconds=0.1))
                    break
                except Exception:
                    pass
        self.get_logger().info('Sensor data ready. Starting exploration!')

        self.state = ExploreState.SCANNING
        self.scan_index = 0
        self.scan_start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            # Always update map and detect objects
            now = time.time()
            self.update_occupancy_grid()

            if now - self.last_detection_time > 0.5:
                self.detect_objects()
                self.last_detection_time = now

            if now - self.last_map_publish > 1.0 / self.MAP_PUBLISH_RATE:
                self.publish_map()
                self.publish_objects()
                self.last_map_publish = now

            # State machine
            if self.state == ExploreState.SCANNING:
                self._state_scanning()
            elif self.state == ExploreState.SELECTING_FRONTIER:
                self._state_selecting_frontier()
            elif self.state == ExploreState.NAVIGATING:
                self._state_navigating()
            elif self.state == ExploreState.COMPLETE:
                self._state_complete()
                break

    def _state_scanning(self):
        """Scan environment by panning camera at current position."""
        now = time.time()

        if self.scan_start_time is None:
            self.scan_start_time = now
            pan = self.SCAN_PAN_ANGLES[self.scan_index]
            self.send_pan_tilt(pan, self.SCAN_TILT)
            self.get_logger().info(
                f'[SCANNING] Pan position {self.scan_index + 1}/{len(self.SCAN_PAN_ANGLES)} '
                f'(pan={pan:.1f})')

        elapsed = now - self.scan_start_time
        if elapsed >= self.SCAN_DWELL_TIME:
            self.scan_index += 1
            if self.scan_index >= len(self.SCAN_PAN_ANGLES):
                # Done scanning, select frontier
                self.scan_index = 0
                self.state = ExploreState.SELECTING_FRONTIER
                self.get_logger().info('[SCANNING] Scan complete, selecting frontier...')
                # Reset camera to forward
                self.send_pan_tilt(0.0, self.SCAN_TILT)
            else:
                # Next pan position
                self.scan_start_time = now
                pan = self.SCAN_PAN_ANGLES[self.scan_index]
                self.send_pan_tilt(pan, self.SCAN_TILT)
                self.get_logger().info(
                    f'[SCANNING] Pan position {self.scan_index + 1}/{len(self.SCAN_PAN_ANGLES)} '
                    f'(pan={pan:.1f})')

    def _state_selecting_frontier(self):
        """Find frontiers and pick the best one to navigate to."""
        frontiers = self.find_frontiers()
        self.publish_frontiers(frontiers)

        if not frontiers:
            self.no_frontier_count += 1
            if self.no_frontier_count >= 3:
                self.get_logger().info('[SELECT] No frontiers found after multiple attempts. Exploration complete!')
                self.state = ExploreState.COMPLETE
                return
            self.get_logger().info(f'[SELECT] No frontiers found (attempt {self.no_frontier_count}/3). Re-scanning...')
            self.state = ExploreState.SCANNING
            self.scan_start_time = None
            return

        self.no_frontier_count = 0

        # Pick best frontier
        wx, wy, size, score = frontiers[0]
        self.get_logger().info(
            f'[SELECT] Best frontier: ({wx:.2f}, {wy:.2f}), '
            f'size={size}, score={score:.1f}')

        # Compute navigation goal: slightly in front of frontier (toward robot)
        base_pose = self.get_base_pose_in_odom()
        if base_pose is None:
            self.state = ExploreState.SCANNING
            self.scan_start_time = None
            return

        bx, by, btheta = base_pose
        dx = wx - bx
        dy = wy - by
        dist = np.sqrt(dx**2 + dy**2)

        if dist > self.FRONTIER_NAV_OFFSET * 2:
            # Navigate to a point offset back toward the robot
            ratio = (dist - self.FRONTIER_NAV_OFFSET) / dist
            goal_x = bx + dx * ratio
            goal_y = by + dy * ratio
        else:
            goal_x = wx
            goal_y = wy

        # Face toward the frontier
        goal_theta = np.arctan2(dy, dx)
        # Convert to user frame (user theta=0 means facing +X, bridge adds pi/2)
        user_theta = goal_theta - np.pi / 2

        self.get_logger().info(
            f'[SELECT] Navigating to ({goal_x:.2f}, {goal_y:.2f}), '
            f'heading={np.degrees(goal_theta):.0f}°')

        msg = Pose2D()
        msg.x = goal_x
        msg.y = goal_y
        msg.theta = user_theta
        self.base_goal_reached = False
        self.base_target_pub.publish(msg)

        self.nav_start_time = time.time()
        self.state = ExploreState.NAVIGATING

    def _state_navigating(self):
        """Wait for navigation to complete."""
        elapsed = time.time() - self.nav_start_time

        if self.base_goal_reached:
            self.get_logger().info('[NAV] Goal reached! Starting scan...')
            self.state = ExploreState.SCANNING
            self.scan_start_time = None
            self.spin_for(0.5)  # let things settle
            return

        if elapsed > self.NAV_TIMEOUT:
            self.get_logger().warn('[NAV] Navigation timed out. Scanning from current position...')
            self.state = ExploreState.SCANNING
            self.scan_start_time = None
            return

    def _state_complete(self):
        """Exploration complete — print summary."""
        # Final publish
        self.publish_map()
        self.publish_objects()

        # Count mapped cells
        free_count = np.sum(self.grid == self.FREE)
        occupied_count = np.sum(self.grid == self.OCCUPIED)
        unknown_count = np.sum(self.grid == self.UNKNOWN)
        total = self.GRID_SIZE * self.GRID_SIZE
        coverage = (free_count + occupied_count) / total * 100

        self.get_logger().info('=' * 50)
        self.get_logger().info('EXPLORATION COMPLETE')
        self.get_logger().info(f'  Coverage: {coverage:.1f}%')
        self.get_logger().info(f'  Free cells: {free_count}')
        self.get_logger().info(f'  Occupied cells: {occupied_count}')
        self.get_logger().info(f'  Unknown cells: {unknown_count}')
        self.get_logger().info('')
        self.get_logger().info('Detected objects:')
        total_objects = 0
        for color, positions in self.detected_objects.items():
            for pos in positions:
                self.get_logger().info(
                    f'  {color}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})')
                total_objects += 1
        self.get_logger().info(f'  Total: {total_objects} objects')
        self.get_logger().info('=' * 50)

        # Keep publishing so RViz stays updated
        self.get_logger().info('Keeping map and markers alive. Press Ctrl+C to stop.')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1.0)
            self.publish_map()
            self.publish_objects()


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
