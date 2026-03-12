"""Grasp pipeline mixin: detection, point cloud, IK, plan-and-execute."""

import os
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Float64MultiArray
from tidybot_msgs.srv import PlanToTarget
from scipy.spatial.transform import Rotation

from tidybot_perception.grasp_geometry import (
    yaw_to_grasp_quaternion,
    GRIPPER_MAX_OPENING_M,
)

from common.constants import FLOOR_MARGIN, GRIPPER_OPEN_POS, ransac_floor_separate


class GraspMixin:
    """Mixin providing the full grasp pipeline.

    Expects the host class to have:
        self.latest_depth, self.latest_rgb, self.camera_K, self.tf_buffer,
        self.gripper_pub, self.arm_pub, self.plan_client,
        self.latest_yolo_detection, self.current_tilt,
        self.GRASP_ARM_NAME, self.GRASP_MOVE_DURATION,
        self.GRASP_PRE_HEIGHT, self.GRASP_LIFT_HEIGHT,
        self.GRASP_Z_OFFSET, self.GRASP_X_OFFSET, self.GRASP_Y_OFFSET,
        self.GRASP_BBOX_PAD, self.GRASP_SETTLE_TIME,
        self.GRASP_MIN_DEPTH_M, self.GRASP_MAX_DEPTH_M,
        self.GRASP_MIN_POINTS, self.GRASP_TABLE_Z_MIN,
        self.GRASP_OUTLIER_Z_THRESH,
        self.GRASP_SWEET_SPOT_X, self.GRASP_SWEET_SPOT_Y,
        self.GRASP_SWEET_SPOT_RADIUS
    Also requires methods from other mixins:
        self._spin_for(), self._stop_base(), self._set_camera_tilt(),
        self._pp() (if using _grasp_detect_and_center)
    """

    # ── Depth cropping & point cloud ─────────────────────────────────────────

    def _grasp_crop_depth(self, cx, cy, bbox_w, bbox_h):
        if self.latest_depth is None:
            return None, None, None
        h, w = self.latest_depth.shape
        half_w = int(bbox_w * self.GRASP_BBOX_PAD / 2)
        half_h = int(bbox_h * self.GRASP_BBOX_PAD / 2)
        u0 = max(0, cx - half_w)
        v0 = max(0, cy - half_h)
        u1 = min(w, cx + half_w)
        v1 = min(h, cy + half_h)
        return self.latest_depth[v0:v1, u0:u1], u0, v0

    def _grasp_get_tf_matrix(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0))
            t = transform.transform.translation
            q = transform.transform.rotation
            rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
            mat = np.eye(4, dtype=np.float32)
            mat[:3, :3] = rot.as_matrix().astype(np.float32)
            mat[:3, 3] = [t.x, t.y, t.z]
            return mat
        except Exception as e:
            self.get_logger().error(f'[GRASP] TF lookup failed: {e}')
            return None

    def _grasp_depth_to_points(self, depth_roi, u_offset, v_offset):
        if self.camera_K is None:
            return None

        fx = self.camera_K[0, 0]
        fy = self.camera_K[1, 1]
        cx = self.camera_K[0, 2]
        cy = self.camera_K[1, 2]

        depth_m = depth_roi.astype(np.float32) / 1000.0
        roi_h, roi_w = depth_m.shape

        u_grid, v_grid = np.meshgrid(
            np.arange(roi_w) + u_offset,
            np.arange(roi_h) + v_offset)

        mask = ((depth_m > self.GRASP_MIN_DEPTH_M) &
                (depth_m < self.GRASP_MAX_DEPTH_M))
        z = depth_m[mask]
        u = u_grid[mask].astype(np.float32)
        v = v_grid[mask].astype(np.float32)

        if len(z) < self.GRASP_MIN_POINTS:
            return None

        x_cam = (u - cx) * z / fx
        y_cam = (v - cy) * z / fy
        points_cam = np.stack([x_cam, y_cam, z], axis=1)

        tf_mat = self._grasp_get_tf_matrix(
            'base_link', 'camera_color_optical_frame')
        if tf_mat is None:
            return None

        ones = np.ones((len(points_cam), 1), dtype=np.float32)
        pts_h = np.hstack([points_cam, ones])
        pts_base = (tf_mat @ pts_h.T).T[:, :3]

        pts_base = pts_base[pts_base[:, 2] > self.GRASP_TABLE_Z_MIN]

        if len(pts_base) > self.GRASP_MIN_POINTS:
            median_z = np.median(pts_base[:, 2])
            z_mask = (np.abs(pts_base[:, 2] - median_z)
                      < self.GRASP_OUTLIER_Z_THRESH)
            pts_base = pts_base[z_mask]

        if len(pts_base) < self.GRASP_MIN_POINTS:
            return None

        floor_z, obj_pts = ransac_floor_separate(
            pts_base, distance_thresh=FLOOR_MARGIN,
            min_pts=self.GRASP_MIN_POINTS)

        if len(obj_pts) < self.GRASP_MIN_POINTS:
            self.get_logger().warn(
                f'[GRASP] Floor filter removed too many points '
                f'({len(pts_base)} -> {len(obj_pts)}), using all points')
            return pts_base

        self.get_logger().info(
            f'[GRASP] RANSAC floor filter: {len(pts_base)} -> {len(obj_pts)} pts, '
            f'floor_z={floor_z:.3f}m')
        return obj_pts

    # ── Object analysis ──────────────────────────────────────────────────────

    def _grasp_analyze_object(self, points_base):
        centroid = np.mean(points_base, axis=0)
        z_top = float(np.max(points_base[:, 2]))

        pts_xy = points_base[:, :2]
        centered = pts_xy - centroid[:2]

        best_yaw = 0.0
        best_clearance = -1.0
        best_width = float('inf')
        best_center_offset = 0.0
        max_width = 0.0

        for angle_deg in range(0, 180, 5):
            angle = np.radians(angle_deg)
            finger_dir = np.array([np.cos(angle), np.sin(angle)])
            projections = centered @ finger_dir
            proj_min = float(projections.min())
            proj_max = float(projections.max())
            width = proj_max - proj_min

            if width > max_width:
                max_width = width

            center_offset = (proj_min + proj_max) / 2.0
            clearance = (GRIPPER_MAX_OPENING_M - width) / 2.0

            if clearance > best_clearance:
                best_clearance = clearance
                best_yaw = angle
                best_width = width
                best_center_offset = center_offset

        finger_dir = np.array([np.cos(best_yaw), np.sin(best_yaw)])
        grasp_center = centroid.copy()
        grasp_center[0] += best_center_offset * finger_dir[0]
        grasp_center[1] += best_center_offset * finger_dir[1]

        self.get_logger().info(
            f'[GRASP] Max-clearance sweep: '
            f'yaw={np.degrees(best_yaw):.0f}deg, '
            f'grip={best_width*1000:.0f}mm, '
            f'clearance={best_clearance*1000:.1f}mm, '
            f'center_offset={best_center_offset*1000:.1f}mm')

        return {
            'centroid': centroid,
            'grasp_center': grasp_center,
            'grasp_yaw': best_yaw,
            'grip_width': best_width,
            'min_clearance': best_clearance,
            'max_width': max_width,
            'z_top': z_top,
            'num_points': len(points_base),
        }

    def _grasp_compute_poses(self, analysis):
        grasp_center = analysis.get('grasp_center', analysis['centroid'])
        grasp_yaw = analysis['grasp_yaw']
        grip_width = analysis['grip_width']

        if grasp_yaw > np.pi / 2:
            grasp_yaw -= np.pi

        if grip_width > GRIPPER_MAX_OPENING_M:
            self.get_logger().warn(
                f'[GRASP] Width {grip_width*1000:.0f}mm > '
                f'{GRIPPER_MAX_OPENING_M*1000:.0f}mm max '
                f'(proceeding anyway)')

        qw, qx, qy, qz = yaw_to_grasp_quaternion(grasp_yaw)

        grasp_x = float(grasp_center[0]) + self.GRASP_X_OFFSET
        grasp_y = float(grasp_center[1]) + self.GRASP_Y_OFFSET
        grasp_z = analysis['z_top'] + self.GRASP_Z_OFFSET

        grasp_pose = Pose()
        grasp_pose.position.x = grasp_x
        grasp_pose.position.y = grasp_y
        grasp_pose.position.z = float(grasp_z)
        grasp_pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

        pre_grasp_pose = Pose()
        pre_grasp_pose.position.x = grasp_x
        pre_grasp_pose.position.y = grasp_y
        pre_grasp_pose.position.z = float(grasp_z + self.GRASP_PRE_HEIGHT)
        pre_grasp_pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

        info = (f'yaw={np.degrees(grasp_yaw):.0f}deg, '
                f'xyz=({grasp_x:.3f}, {grasp_y:.3f}, {grasp_z:.3f})m, '
                f'offsets=({self.GRASP_X_OFFSET*1000:.0f}, '
                f'{self.GRASP_Y_OFFSET*1000:.0f}, '
                f'{self.GRASP_Z_OFFSET*1000:.0f})mm, '
                f'width={grip_width*1000:.0f}mm')

        return grasp_pose, pre_grasp_pose, info

    # ── IK check & plan-and-execute ──────────────────────────────────────────

    def _try_ik(self, pose, use_orientation=True):
        req = PlanToTarget.Request()
        req.arm_name = self.GRASP_ARM_NAME
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = False
        req.duration = self.GRASP_MOVE_DURATION
        req.max_condition_number = 500.0

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done() or future.exception():
            return False, 'service call timed out'

        result = future.result()
        return result.success, result.message

    def _grasp_plan_and_execute(self, pose, label, use_orientation=True,
                                keep_gripper_open=False):
        req = PlanToTarget.Request()
        req.arm_name = self.GRASP_ARM_NAME
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = True
        req.duration = self.GRASP_MOVE_DURATION
        req.max_condition_number = 500.0

        self.get_logger().info(
            f'  [{label}] Planning + executing...')

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=15.0)
        if not future.done() or future.exception():
            self.get_logger().error(
                f'  [{label}] Service call timed out')
            return False

        result = future.result()
        if result.success:
            if result.executed:
                if keep_gripper_open:
                    grip_msg = Float64MultiArray()
                    grip_msg.data = [GRIPPER_OPEN_POS]
                    deadline = time.time() + self.GRASP_MOVE_DURATION + 0.5
                    while time.time() < deadline:
                        self.gripper_pub.publish(grip_msg)
                        rclpy.spin_once(self, timeout_sec=0.1)
                else:
                    time.sleep(self.GRASP_MOVE_DURATION + 0.5)
            return True

        self.get_logger().warn(
            f'  [{label}] Failed: {result.message}')
        return False

    def _bin_plan_and_execute(self, pose, label, use_orientation=True):
        """Call motion planner for bin positioning (keeps gripper closed)."""
        req = PlanToTarget.Request()
        req.arm_name = self.GRASP_ARM_NAME
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = True
        req.duration = getattr(self, 'BIN_MOVE_DURATION', 2.5)
        req.max_condition_number = getattr(self, 'BIN_MAX_COND_NUM', 500.0)

        self.get_logger().info(f'  [{label}] Planning + executing...')

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        if not future.done() or future.exception():
            self.get_logger().error(f'  [{label}] Service call timed out')
            return False

        result = future.result()
        if result.success:
            if result.executed:
                grip_msg = Float64MultiArray()
                grip_msg.data = [1.0]
                deadline = time.time() + getattr(self, 'BIN_MOVE_DURATION', 2.5) + 0.5
                while time.time() < deadline:
                    self.gripper_pub.publish(grip_msg)
                    rclpy.spin_once(self, timeout_sec=0.1)
            return True

        self.get_logger().warn(f'  [{label}] Failed: {result.message}')
        return False

    # ── Detect & center ──────────────────────────────────────────────────────

    def _grasp_detect_and_center(self):
        """Detect object, center it in frame, and return (cx, cy, bbox_w, bbox_h) or None."""
        self.get_logger().info(
            f'[GRASP] Looking for object at current tilt={self.current_tilt:.2f}...')
        det = None
        self.latest_yolo_detection = None
        self._spin_for(0.5)
        for _ in range(15):
            self._spin_for(0.2)
            if self.latest_yolo_detection is not None:
                det = self.latest_yolo_detection
                break

        if det is None:
            self.get_logger().error(
                '[GRASP] No YOLO detection at current camera pose')
            return None

        # Center the object in the camera frame by adjusting tilt
        self.get_logger().info('[GRASP] Centering object in frame...')
        center_deadline = time.time() + 5.0
        while time.time() < center_deadline:
            self._spin_for(0.2)
            det = self.latest_yolo_detection
            if det is None:
                continue
            self.latest_yolo_detection = None

            v = int(det.bbox.center.position.y)
            if self.latest_depth is None:
                continue
            img_h = self.latest_depth.shape[0]
            img_center_y = img_h / 2.0
            offset_y = v - img_center_y

            if abs(offset_y) < self._pp('center_pixel_thresh_v'):
                self.get_logger().info(
                    f'[GRASP] Object centered vertically '
                    f'(offset={offset_y:.0f}px, tilt={self.current_tilt:.2f})')
                break

            tilt_adj = offset_y / (img_h / 2.0) * self.CENTER_TILT_STEP
            self._set_camera_tilt(self.current_tilt + tilt_adj)

        # Get a fresh detection after centering
        self.latest_yolo_detection = None
        self._spin_for(0.5)
        for _ in range(10):
            self._spin_for(0.2)
            if self.latest_yolo_detection is not None:
                break
        det = self.latest_yolo_detection
        if det is None:
            self.get_logger().error('[GRASP] Lost object after centering')
            return None

        self.latest_yolo_detection = None
        return (int(det.bbox.center.position.x),
                int(det.bbox.center.position.y),
                float(det.bbox.size_x),
                float(det.bbox.size_y))

    def _grasp_compute_from_detection(self, cx, cy, bbox_w, bbox_h):
        """From a detection, compute grasp/pre-grasp poses.

        Returns (grasp_pose, pre_grasp_pose, analysis, info) or None.
        """
        self.get_logger().info(
            f'[GRASP] YOLO detection at pixel ({cx}, {cy}), '
            f'bbox {bbox_w:.0f}x{bbox_h:.0f}')

        depth_roi, u_off, v_off = self._grasp_crop_depth(
            cx, cy, bbox_w, bbox_h)
        if depth_roi is None:
            self.get_logger().error('[GRASP] Failed to crop depth image')
            return None

        pts_base = self._grasp_depth_to_points(depth_roi, u_off, v_off)
        if pts_base is None:
            self.get_logger().error(
                f'[GRASP] Insufficient points in depth ROI '
                f'(need >= {self.GRASP_MIN_POINTS})')
            return None

        self.get_logger().info(
            f'[GRASP] Point cloud: {len(pts_base)} points')

        analysis = self._grasp_analyze_object(pts_base)
        centroid = analysis['centroid']
        self.get_logger().info(
            f'[GRASP] Centroid: ({centroid[0]:.3f}, {centroid[1]:.3f}, '
            f'{centroid[2]:.3f}), z_top={analysis["z_top"]:.3f}m')

        result = self._grasp_compute_poses(analysis)
        if result is None:
            self.get_logger().error('[GRASP] Pose computation failed')
            return None
        grasp_pose, pre_grasp_pose, info = result
        self.get_logger().info(f'[GRASP] Params: {info}')
        return grasp_pose, pre_grasp_pose, analysis, info

    # ── Positioning nudges ───────────────────────────────────────────────────

    def _grasp_reposition_for_sweet_spot(self, obj_x, obj_y):
        dx = obj_x - self.GRASP_SWEET_SPOT_X
        dy = obj_y - self.GRASP_SWEET_SPOT_Y
        dist = np.hypot(dx, dy)

        if dist <= self.GRASP_SWEET_SPOT_RADIUS:
            self.get_logger().info(
                f'[GRASP] Object at ({obj_x:.3f}, {obj_y:.3f}) '
                f'is within sweet spot (dist={dist:.3f}m)')
            return False

        self.get_logger().info(
            f'[GRASP] Object at ({obj_x:.3f}, {obj_y:.3f}) '
            f'is {dist:.3f}m from sweet spot '
            f'({self.GRASP_SWEET_SPOT_X:.3f}, {self.GRASP_SWEET_SPOT_Y:.3f})')

        from geometry_msgs.msg import Twist
        cmd = Twist()
        drive_time = 1.0

        if abs(dy) > 0.03:
            cmd.angular.z = float(np.clip(dy * 0.8, -0.15, 0.15))
        if abs(dx) > 0.03:
            cmd.linear.x = float(np.clip(dx * 0.5, -0.05, 0.05))

        self.get_logger().info(
            f'[GRASP] Nudging: vx={cmd.linear.x:.3f}, '
            f'wz={cmd.angular.z:.3f} for {drive_time:.1f}s')

        deadline = time.time() + drive_time
        while time.time() < deadline and rclpy.ok():
            self.cmd_vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
        self._stop_base()
        self._spin_for(0.5)

        return True

    def _grasp_ik_nudge(self, grasp_x, grasp_y):
        """Nudge base to move object into IK-friendly zone."""
        target_x = self._ik_nudge_target_x
        target_y = -0.24

        dx = target_x - grasp_x
        dy = target_y - grasp_y

        from geometry_msgs.msg import Twist
        cmd = Twist()
        cmd.linear.x = float(np.clip(-dx * 0.5, -0.06, 0.06))
        cmd.angular.z = float(np.clip(-dy * 0.8, -0.15, 0.15))

        dist = np.hypot(dx, dy)
        drive_time = float(np.clip(dist / 0.05, 0.3, 1.5))

        self.get_logger().info(
            f'[GRASP] IK nudge: grasp=({grasp_x:.3f}, {grasp_y:.3f}), '
            f'target=({target_x:.3f}, {target_y:.3f}), '
            f'cmd=(vx={cmd.linear.x:.3f}, wz={cmd.angular.z:.3f}), '
            f'drive={drive_time:.1f}s')

        deadline = time.time() + drive_time
        while time.time() < deadline and rclpy.ok():
            self.cmd_vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
        self._stop_base()
        self._spin_for(0.5)

    # ── Snapshot ─────────────────────────────────────────────────────────────

    def _save_grasp_snapshot(self):
        self._spin_for(0.3)
        if self.latest_rgb is None:
            self.get_logger().warn('[GRASP] No RGB image available for snapshot')
            return
        snap_dir = os.path.expanduser('~/grasp_snapshots')
        os.makedirs(snap_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(snap_dir, f'grasp_{stamp}.jpg')
        import cv2
        cv2.imwrite(path, self.latest_rgb)
        self.get_logger().info(f'[GRASP] Snapshot saved: {path}')
