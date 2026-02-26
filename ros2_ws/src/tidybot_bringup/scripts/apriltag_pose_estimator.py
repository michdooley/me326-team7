#!/usr/bin/env python3
"""
Detect an AprilTag in the camera image and estimate/publish its pose directly.

Uses OpenCV ArUco AprilTag dictionaries (no apriltag_ros required).

Subscribes (defaults):
  /camera/color/image_raw    sensor_msgs/msg/Image
  /camera/color/camera_info  sensor_msgs/msg/CameraInfo

Publishes (defaults):
  /apriltag/pose       geometry_msgs/msg/PoseStamped
  /apriltag/position   geometry_msgs/msg/PointStamped

Key parameter:
  tag_size_m  (float)  Physical edge length of the printed tag in meters.
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class AprilTagPoseEstimator(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_pose_estimator")

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("pose_topic", "/apriltag/pose")
        self.declare_parameter("position_topic", "/apriltag/position")
        self.declare_parameter("tag_family", "36h11")
        self.declare_parameter("target_tag_id", 0)
        self.declare_parameter("tag_size_m", 0.127)  # 5 in
        self.declare_parameter("publish_first_if_missing_id", True)
        self.declare_parameter("debug_draw", False)
        self.declare_parameter("debug_image_topic", "/apriltag/debug_image")

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.target_tag_id = int(self.get_parameter("target_tag_id").value)
        self.tag_size_m = float(self.get_parameter("tag_size_m").value)
        self.publish_first_if_missing_id = bool(
            self.get_parameter("publish_first_if_missing_id").value
        )
        self.debug_draw = bool(self.get_parameter("debug_draw").value)

        if self.tag_size_m <= 0.0:
            raise ValueError("tag_size_m must be > 0")

        self.cv_bridge = CvBridge()
        self.K = None
        self.dist_coeffs = None

        self.pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter("pose_topic").value, 10
        )
        self.pos_pub = self.create_publisher(
            PointStamped, self.get_parameter("position_topic").value, 10
        )
        self.debug_pub = None
        if self.debug_draw:
            self.debug_pub = self.create_publisher(
                Image, self.get_parameter("debug_image_topic").value, 10
            )

        self.dictionary = self._make_dictionary(
            str(self.get_parameter("tag_family").value)
        )
        self.detector = self._make_detector(self.dictionary)

        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, 10)
        self.create_subscription(Image, self.image_topic, self._image_cb, 10)

        self.get_logger().info(
            f"Detecting AprilTag family={self.get_parameter('tag_family').value}, "
            f"target_tag_id={self.target_tag_id}, tag_size_m={self.tag_size_m:.4f}"
        )

    def _make_dictionary(self, family: str):
        fam_map = {
            "16h5": cv2.aruco.DICT_APRILTAG_16h5,
            "25h9": cv2.aruco.DICT_APRILTAG_25h9,
            "36h10": cv2.aruco.DICT_APRILTAG_36h10,
            "36h11": cv2.aruco.DICT_APRILTAG_36h11,
        }
        if family not in fam_map:
            raise ValueError(f"Unsupported tag_family '{family}'. Supported: {sorted(fam_map)}")
        return cv2.aruco.getPredefinedDictionary(fam_map[family])

    def _make_detector(self, dictionary):
        params = cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, "ArucoDetector"):
            return cv2.aruco.ArucoDetector(dictionary, params)
        return (dictionary, params)

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if msg.d:
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        else:
            self.dist_coeffs = np.zeros((5,), dtype=np.float64)

    def _image_cb(self, msg: Image) -> None:
        if self.K is None:
            return

        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}", throttle_duration_sec=2.0)
            return

        corners, ids = self._detect(frame)
        if ids is None or len(ids) == 0:
            if self.debug_draw and self.debug_pub is not None:
                self._publish_debug(frame, msg.header)
            return

        ids_flat = ids.flatten().tolist()
        det_idx = None
        if self.target_tag_id in ids_flat:
            det_idx = ids_flat.index(self.target_tag_id)
        elif self.publish_first_if_missing_id:
            det_idx = 0
        else:
            return

        tag_id = int(ids_flat[det_idx])
        tag_corners = corners[det_idx].reshape(4, 2).astype(np.float64)

        ok, rvec, tvec = self._estimate_pose(tag_corners)
        if not ok:
            self.get_logger().warn("solvePnP failed for AprilTag detection", throttle_duration_sec=2.0)
            return

        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose.position.x = float(tvec[0])
        pose_msg.pose.position.y = float(tvec[1])
        pose_msg.pose.position.z = float(tvec[2])
        pose_msg.pose.orientation = self._rvec_to_quaternion(rvec)

        point_msg = PointStamped()
        point_msg.header = msg.header
        point_msg.point = pose_msg.pose.position

        self.pose_pub.publish(pose_msg)
        self.pos_pub.publish(point_msg)

        self.get_logger().debug(
            f"Tag {tag_id}: x={tvec[0]:.3f}, y={tvec[1]:.3f}, z={tvec[2]:.3f}"
        )

        if self.debug_draw and self.debug_pub is not None:
            dbg = frame.copy()
            cv2.aruco.drawDetectedMarkers(dbg, [tag_corners.astype(np.float32)], np.array([[tag_id]]))
            cv2.drawFrameAxes(dbg, self.K, self.dist_coeffs, rvec, tvec, self.tag_size_m * 0.5)
            self._publish_debug(dbg, msg.header)

    def _detect(self, frame):
        if hasattr(self.detector, "detectMarkers"):
            corners, ids, _ = self.detector.detectMarkers(frame)
            return corners, ids
        dictionary, params = self.detector
        corners, ids, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=params)
        return corners, ids

    def _estimate_pose(self, image_corners):
        half = self.tag_size_m / 2.0
        # Tag coordinate frame centered on tag; Z points out of tag plane.
        object_points = np.array(
            [
                [-half, -half, 0.0],
                [half, -half, 0.0],
                [half, half, 0.0],
                [-half, half, 0.0],
            ],
            dtype=np.float64,
        )

        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            image_corners,
            self.K,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            return False, None, None
        return True, rvec.reshape(3), tvec.reshape(3)

    def _publish_debug(self, frame, header):
        try:
            msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish debug image: {exc}", throttle_duration_sec=2.0)
            return
        msg.header = header
        self.debug_pub.publish(msg)

    @staticmethod
    def _rvec_to_quaternion(rvec) -> Quaternion:
        R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
        # Rotation matrix -> quaternion (x, y, z, w)
        tr = float(np.trace(R))
        if tr > 0.0:
            s = math.sqrt(tr + 1.0) * 2.0
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
        q = Quaternion()
        q.x = float(qx)
        q.y = float(qy)
        q.z = float(qz)
        q.w = float(qw)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AprilTagPoseEstimator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
