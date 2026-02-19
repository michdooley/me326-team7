#!/usr/bin/env python3
"""
Occupancy-map scene scan for TidyBot.

This node sweeps the camera pan angle to scan the environment, projects depth into
a 2D occupancy grid in the odom frame, raytraces free space, and publishes /map
for RViz.

Usage:
    ros2 run tidybot_bringup frontier_exploration_codex.py
"""

import time

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray

try:
    from cv_bridge import CvBridge
except ImportError:
    print('ERROR: cv_bridge is required.')
    raise


class FrontierExplorationCodex(Node):
    # Grid configuration
    GRID_RESOLUTION = 0.05
    GRID_SIZE = 300
    GRID_ORIGIN_X = -7.5
    GRID_ORIGIN_Y = -7.5

    # Occupancy values
    UNKNOWN = -1
    FREE = 0
    OCCUPIED = 100

    # Depth mapping parameters
    MAX_DEPTH_RANGE_M = 5.0
    DEPTH_SUBSAMPLE = 4
    MIN_OBSTACLE_Z_M = 0.08
    MAX_OBSTACLE_Z_M = 1.20

    # Scan parameters
    SCAN_PAN_ANGLES = [-1.2, 0.0, 1.2]
    SCAN_TILT = 0.3
    SCAN_DWELL_TIME_S = 2.0

    # Publish rates
    MAP_PUBLISH_RATE_HZ = 2.0

    def __init__(self):
        super().__init__('frontier_exploration_codex')

        self.grid = np.full((self.GRID_SIZE, self.GRID_SIZE), self.UNKNOWN, dtype=np.int8)

        self.latest_depth = None
        self.camera_K = None
        self.cv_bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self._depth_cb, qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._camera_info_cb, qos
        )

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.pan_tilt_pub = self.create_publisher(Float64MultiArray, '/camera/pan_tilt_cmd', 10)

        self.scan_index = 0
        self.scan_start_time = None
        self.sweep_direction = 1
        self.last_map_publish_time = 0.0

        self.get_logger().info('frontier_exploration_codex initialized')

    def _depth_cb(self, msg: Image):
        try:
            self.latest_depth = self.cv_bridge.imgmsg_to_cv2(msg, '16UC1')
        except Exception as exc:
            self.get_logger().warn(f'Depth conversion failed: {exc}')

    def _camera_info_cb(self, msg: CameraInfo):
        self.camera_K = np.array(msg.k).reshape(3, 3)

    def send_pan_tilt(self, pan: float, tilt: float):
        msg = Float64MultiArray()
        msg.data = [pan, tilt]
        self.pan_tilt_pub.publish(msg)

    def world_to_grid(self, wx: float, wy: float):
        gx = int((wx - self.GRID_ORIGIN_X) / self.GRID_RESOLUTION)
        gy = int((wy - self.GRID_ORIGIN_Y) / self.GRID_RESOLUTION)
        return gx, gy

    def in_grid(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.GRID_SIZE and 0 <= gy < self.GRID_SIZE

    def _raytrace_free(self, x0: int, y0: int, x1: int, y1: int):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0

        while True:
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

    def update_occupancy_grid(self):
        if self.latest_depth is None or self.camera_K is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'odom',
                'camera_depth_optical_frame',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

        t = transform.transform.translation
        q = transform.transform.rotation
        cam_origin = np.array([t.x, t.y, t.z])

        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        rot = np.array(
            [
                [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
            ]
        )

        fx = self.camera_K[0, 0]
        fy = self.camera_K[1, 1]
        cx = self.camera_K[0, 2]
        cy = self.camera_K[1, 2]

        depth = self.latest_depth
        h, w = depth.shape

        vs, us = np.mgrid[0:h:self.DEPTH_SUBSAMPLE, 0:w:self.DEPTH_SUBSAMPLE]
        us = us.ravel()
        vs = vs.ravel()
        depth_mm = depth[vs, us].astype(np.float64)

        valid = (depth_mm > 0) & (depth_mm < self.MAX_DEPTH_RANGE_M * 1000.0)
        if not np.any(valid):
            return

        us = us[valid]
        vs = vs[valid]
        depth_m = depth_mm[valid] / 1000.0

        x_cam = (us - cx) * depth_m / fx
        y_cam = (vs - cy) * depth_m / fy
        z_cam = depth_m

        points_cam = np.stack([x_cam, y_cam, z_cam], axis=1)
        points_odom = (rot @ points_cam.T).T + cam_origin

        cam_gx, cam_gy = self.world_to_grid(cam_origin[0], cam_origin[1])
        gxs = ((points_odom[:, 0] - self.GRID_ORIGIN_X) / self.GRID_RESOLUTION).astype(int)
        gys = ((points_odom[:, 1] - self.GRID_ORIGIN_Y) / self.GRID_RESOLUTION).astype(int)

        in_bounds = (
            (gxs >= 0)
            & (gxs < self.GRID_SIZE)
            & (gys >= 0)
            & (gys < self.GRID_SIZE)
        )
        if not np.any(in_bounds):
            return

        # Mark FREE along rays to all valid measured endpoints.
        ray_gxs = gxs[in_bounds]
        ray_gys = gys[in_bounds]
        ray_step = max(1, len(ray_gxs) // 600)
        for i in range(0, len(ray_gxs), ray_step):
            self._raytrace_free(cam_gx, cam_gy, int(ray_gxs[i]), int(ray_gys[i]))

        # Mark OCCUPIED only for points above the floor band.
        obstacle_mask = (
            in_bounds
            & (points_odom[:, 2] > self.MIN_OBSTACLE_Z_M)
            & (points_odom[:, 2] < self.MAX_OBSTACLE_Z_M)
        )
        occ_gxs = gxs[obstacle_mask]
        occ_gys = gys[obstacle_mask]
        if len(occ_gxs) > 0:
            self.grid[occ_gys, occ_gxs] = self.OCCUPIED

        if self.in_grid(cam_gx, cam_gy):
            for dy in range(-3, 4):
                for dx in range(-3, 4):
                    nx = cam_gx + dx
                    ny = cam_gy + dy
                    if self.in_grid(nx, ny) and self.grid[ny, nx] == self.UNKNOWN:
                        self.grid[ny, nx] = self.FREE

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        info = MapMetaData()
        info.resolution = self.GRID_RESOLUTION
        info.width = self.GRID_SIZE
        info.height = self.GRID_SIZE
        info.origin.position.x = float(self.GRID_ORIGIN_X)
        info.origin.position.y = float(self.GRID_ORIGIN_Y)
        info.origin.orientation.w = 1.0
        msg.info = info

        msg.data = self.grid.ravel().tolist()
        self.map_pub.publish(msg)

    def step_scan(self):
        now = time.time()
        if self.scan_start_time is None:
            self.scan_start_time = now
            pan = self.SCAN_PAN_ANGLES[self.scan_index]
            self.send_pan_tilt(pan, self.SCAN_TILT)
            return

        if now - self.scan_start_time < self.SCAN_DWELL_TIME_S:
            return

        self.scan_start_time = now
        self.scan_index += self.sweep_direction

        if self.scan_index >= len(self.SCAN_PAN_ANGLES):
            self.scan_index = len(self.SCAN_PAN_ANGLES) - 2
            self.sweep_direction = -1
        elif self.scan_index < 0:
            self.scan_index = 1
            self.sweep_direction = 1

        pan = self.SCAN_PAN_ANGLES[self.scan_index]
        self.send_pan_tilt(pan, self.SCAN_TILT)

    def run(self):
        self.get_logger().info('Waiting for depth, camera intrinsics, and TF...')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_depth is None or self.camera_K is None:
                continue
            try:
                self.tf_buffer.lookup_transform(
                    'odom', 'camera_depth_optical_frame', rclpy.time.Time(), timeout=Duration(seconds=0.1)
                )
                break
            except Exception:
                continue

        self.get_logger().info('Scene scan started: building occupancy map.')

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            self.step_scan()
            self.update_occupancy_grid()

            now = time.time()
            if now - self.last_map_publish_time >= 1.0 / self.MAP_PUBLISH_RATE_HZ:
                self.publish_map()
                self.last_map_publish_time = now


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorationCodex()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
