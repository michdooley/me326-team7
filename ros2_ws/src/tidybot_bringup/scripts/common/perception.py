"""Perception mixin: camera callbacks, depth-to-world transforms, markers."""

import numpy as np
import rclpy
from rclpy.duration import Duration

from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2DArray


def _q2m(q):
    """Convert a geometry_msgs Quaternion to a 3x3 rotation matrix."""
    qw, qx, qy, qz = q.w, q.x, q.y, q.z
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ])


class PerceptionMixin:
    """Mixin providing camera callbacks, depth estimation, and marker publishing.

    Expects the host class to have:
        self.latest_depth, self.latest_depth_stamp, self.latest_rgb, self.camera_K,
        self.cv_bridge, self.tf_buffer, self.object_marker_pub,
        self.MIN_DEPTH_M, self.MAX_DEPTH_M, self.MIN_CONFIDENCE,
        self.target_class_id, self.latest_yolo_detection, self.object_world_pos
    """

    # ── Sensor callbacks ─────────────────────────────────────────────────────

    def _depth_cb(self, msg: Image):
        try:
            depth = self.cv_bridge.imgmsg_to_cv2(msg, '16UC1')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')
            return
        self.latest_depth = depth
        self.latest_depth_stamp = msg.header.stamp

    def _rgb_cb(self, msg: Image):
        try:
            self.latest_rgb = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def _camera_info_cb(self, msg: CameraInfo):
        self.camera_K = np.array(msg.k).reshape(3, 3)

    # ── Detection callback ───────────────────────────────────────────────────

    def _yolo_bbox_cb(self, msg: Detection2DArray):
        for detection in msg.detections:
            if not detection.results:
                continue
            hyp = detection.results[0].hypothesis
            class_id = int(hyp.class_id)
            if class_id == self.target_class_id and hyp.score >= self.MIN_CONFIDENCE:
                self.latest_yolo_detection = detection
                return

    # ── Position estimation ──────────────────────────────────────────────────

    def _estimate_world_position(self, u, v, min_depth_override=None):
        """Estimate world (odom) XY position from pixel (u, v) + depth.

        Args:
            u, v: pixel coordinates
            min_depth_override: optional minimum depth (m); defaults to self.MIN_DEPTH_M
        """
        if self.latest_depth is None or self.camera_K is None:
            return None

        depth = self.latest_depth
        h, w = depth.shape

        half_win = 4
        v_lo = max(0, v - half_win)
        v_hi = min(h, v + half_win + 1)
        u_lo = max(0, u - half_win)
        u_hi = min(w, u + half_win + 1)

        patch = depth[v_lo:v_hi, u_lo:u_hi].astype(np.float64)
        valid = patch[patch > 0]
        if len(valid) == 0:
            self.get_logger().warn(
                f'[DEPTH] No valid depth at pixel ({u},{v}), '
                f'patch shape={patch.shape}, depth img shape={depth.shape}')
            return None
        depth_m = float(np.median(valid)) / 1000.0

        min_d = min_depth_override if min_depth_override is not None else self.MIN_DEPTH_M
        if depth_m < min_d or depth_m > self.MAX_DEPTH_M:
            self.get_logger().warn(
                f'[DEPTH] depth={depth_m:.3f}m out of range '
                f'[{min_d:.2f}, {self.MAX_DEPTH_M:.2f}] at pixel ({u},{v})')
            return None

        fx = self.camera_K[0, 0]
        fy = self.camera_K[1, 1]
        cx_k = self.camera_K[0, 2]
        cy_k = self.camera_K[1, 2]

        x_cam = (u - cx_k) * depth_m / fx
        y_cam = (v - cy_k) * depth_m / fy
        z_cam = depth_m
        pt_cam = np.array([x_cam, y_cam, z_cam])

        try:
            tf_ob = self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return None

        try:
            tf_bc = self.tf_buffer.lookup_transform(
                'base_link', 'camera_color_optical_frame',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return None

        R_ob = _q2m(tf_ob.transform.rotation)
        R_bc = _q2m(tf_bc.transform.rotation)
        to = tf_ob.transform.translation
        tc = tf_bc.transform.translation

        cam_origin = R_ob @ np.array([tc.x, tc.y, tc.z]) + \
                     np.array([to.x, to.y, to.z])
        R = R_ob @ R_bc

        pt_world = R @ pt_cam + cam_origin
        return (float(pt_world[0]), float(pt_world[1]))

    def _get_position_in_base_link(self, u, v):
        """Get pixel (u, v) position in base_link frame (for arm positioning)."""
        if self.latest_depth is None or self.camera_K is None:
            return None

        depth = self.latest_depth
        h, w = depth.shape

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

        if depth_m < 0.20 or depth_m > self.MAX_DEPTH_M:
            return None

        fx = self.camera_K[0, 0]
        fy = self.camera_K[1, 1]
        cx_k = self.camera_K[0, 2]
        cy_k = self.camera_K[1, 2]

        x_cam = (u - cx_k) * depth_m / fx
        y_cam = (v - cy_k) * depth_m / fy
        z_cam = depth_m
        pt_cam = np.array([x_cam, y_cam, z_cam])

        try:
            tf_bc = self.tf_buffer.lookup_transform(
                'base_link', 'camera_color_optical_frame',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return None

        R_bc = _q2m(tf_bc.transform.rotation)
        tc = tf_bc.transform.translation
        t_bc = np.array([tc.x, tc.y, tc.z])

        pt_base = R_bc @ pt_cam + t_bc
        return pt_base

    # ── Marker publishing ────────────────────────────────────────────────────

    def publish_object_marker(self):
        if self.object_world_pos is None:
            return
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.ns = 'target_object'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = self.object_world_pos[0]
        m.pose.position.y = self.object_world_pos[1]
        m.pose.position.z = 0.05
        m.pose.orientation.w = 1.0
        m.scale.x = 0.15
        m.scale.y = 0.15
        m.scale.z = 0.15
        m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
        ma = MarkerArray()
        ma.markers.append(m)
        self.object_marker_pub.publish(ma)
