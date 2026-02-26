#!/usr/bin/env python3
"""
Subscribe to apriltag_ros detections and publish the selected tag position.

Expected input (default):
  /detections   apriltag_msgs/msg/AprilTagDetectionArray

Publishes (default):
  /apriltag/position   geometry_msgs/msg/PointStamped
  /apriltag/pose       geometry_msgs/msg/PoseStamped

Examples:
  ros2 run tidybot_bringup apriltag_position_publisher.py
  ros2 run tidybot_bringup apriltag_position_publisher.py --ros-args -p target_tag_id:=0
  ros2 run tidybot_bringup apriltag_position_publisher.py --ros-args -p detections_topic:=/camera/tag_detections
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
except ImportError as exc:
    AprilTagDetectionArray = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


class AprilTagPositionPublisher(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_position_publisher")

        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("position_topic", "/apriltag/position")
        self.declare_parameter("pose_topic", "/apriltag/pose")
        self.declare_parameter("target_tag_id", -1)  # -1 = publish first detection

        self.detections_topic = self.get_parameter("detections_topic").value
        self.target_tag_id = int(self.get_parameter("target_tag_id").value)
        position_topic = self.get_parameter("position_topic").value
        pose_topic = self.get_parameter("pose_topic").value

        self.position_pub = self.create_publisher(PointStamped, position_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)

        if AprilTagDetectionArray is None:
            self.get_logger().error(
                "Could not import apriltag_msgs. Install/source apriltag_ros first. "
                f"Import error: {_IMPORT_ERROR}"
            )
            raise RuntimeError("apriltag_msgs is not available")

        self.create_subscription(
            AprilTagDetectionArray,
            self.detections_topic,
            self._detections_cb,
            10,
        )

        filter_text = (
            "first detected tag"
            if self.target_tag_id < 0
            else f"tag id {self.target_tag_id}"
        )
        self.get_logger().info(
            f"Listening on {self.detections_topic}, publishing {filter_text} "
            f"to {position_topic} and {pose_topic}"
        )

    def _detections_cb(self, msg: "AprilTagDetectionArray") -> None:
        if not msg.detections:
            return

        for detection in msg.detections:
            tag_ids = self._extract_tag_ids(detection)
            if self.target_tag_id >= 0 and self.target_tag_id not in tag_ids:
                continue

            pose_stamped = self._extract_pose_stamped(detection)
            if pose_stamped is None:
                self.get_logger().warn(
                    "Received detection but could not extract pose. "
                    "Check your apriltag_msgs message structure.",
                    throttle_duration_sec=2.0,
                )
                return

            point_msg = PointStamped()
            point_msg.header = pose_stamped.header
            point_msg.point = pose_stamped.pose.position

            self.position_pub.publish(point_msg)
            self.pose_pub.publish(pose_stamped)

            self.get_logger().debug(
                f"Published tag {tag_ids} position "
                f"({point_msg.point.x:.3f}, {point_msg.point.y:.3f}, {point_msg.point.z:.3f})"
            )
            return

    @staticmethod
    def _extract_tag_ids(detection) -> list:
        tag_id = getattr(detection, "id", [])
        if isinstance(tag_id, int):
            return [tag_id]
        return list(tag_id)

    def _extract_pose_stamped(self, detection):
        """
        Handle common apriltag_ros message layouts.

        Most ROS2 apriltag_msgs use:
          detection.pose -> PoseWithCovarianceStamped
        so the pose is available at:
          detection.pose.header
          detection.pose.pose.pose
        """
        raw_pose = getattr(detection, "pose", None)
        if raw_pose is None:
            return None

        pose_msg = PoseStamped()

        if hasattr(raw_pose, "header") and hasattr(raw_pose, "pose"):
            pose_msg.header = raw_pose.header
            nested = raw_pose.pose
            if hasattr(nested, "pose"):
                pose_msg.pose = nested.pose
                return pose_msg
            if hasattr(nested, "position") and hasattr(nested, "orientation"):
                pose_msg.pose = nested
                return pose_msg

        if hasattr(raw_pose, "position") and hasattr(raw_pose, "orientation"):
            pose_msg.header = getattr(detection, "header", pose_msg.header)
            pose_msg.pose = raw_pose
            return pose_msg

        return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AprilTagPositionPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
