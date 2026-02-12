#!/usr/bin/env python3
"""
Object Localizer Node

Converts 2D bounding box detections to 3D positions using depth images,
camera intrinsics, and TF transforms.

Subscribes:
    /camera/depth/image_raw (sensor_msgs/Image) - depth camera feed
    /camera/color/camera_info (sensor_msgs/CameraInfo) - camera intrinsics
    /detections (tidybot_msgs/Detection2DArray) - from detector_node

Publishes:
    /object_poses (tidybot_msgs/ObjectPose) - continuous 3D object positions

Services:
    /localize_object (tidybot_msgs/LocalizeObject) - on-demand 2D→3D conversion

TF:
    Listens to camera_color_optical_frame → base_link

Parameters:
    depth_scale (float): depth image scale to meters (default 0.001 for RealSense)
    max_depth (float): ignore depth beyond this distance in meters (default 3.0)
    target_frame (str): output coordinate frame (default "base_link")

Usage:
    ros2 run tidybot_perception object_localizer_node
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from tidybot_msgs.msg import Detection2DArray, ObjectPose
from tidybot_msgs.srv import LocalizeObject


class ObjectLocalizerNode(Node):
    """Converts 2D detections + depth to 3D positions in base_link frame."""

    def __init__(self):
        super().__init__('object_localizer_node')

        # Parameters
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('max_depth', 3.0)
        self.declare_parameter('target_frame', 'base_link')

        self.depth_scale = self.get_parameter('depth_scale').value
        self.max_depth = self.get_parameter('max_depth').value
        self.target_frame = self.get_parameter('target_frame').value

        # Subscribers
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10
        )
        self.detections_sub = self.create_subscription(
            Detection2DArray, '/detections', self.detections_callback, 10
        )

        # Publisher
        self.object_poses_pub = self.create_publisher(ObjectPose, '/object_poses', 10)

        # Service
        self.localize_srv = self.create_service(
            LocalizeObject, '/localize_object', self.localize_callback
        )

        # TF
        # TODO: Initialize tf2_ros.Buffer and tf2_ros.TransformListener
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Internal state
        self.latest_depth = None
        self.camera_info = None

        self.get_logger().info('ObjectLocalizerNode initialized')

    def depth_callback(self, msg: Image):
        """Cache latest depth image."""
        self.latest_depth = msg

    def camera_info_callback(self, msg: CameraInfo):
        """Cache camera intrinsics."""
        self.camera_info = msg

    def detections_callback(self, msg: Detection2DArray):
        """Localize each detection and publish 3D poses."""
        if self.latest_depth is None or self.camera_info is None:
            return
        # TODO: For each detection in msg.detections:
        #   1. Get depth at bounding box center from self.latest_depth
        #   2. Back-project to 3D using camera intrinsics (fx, fy, cx, cy)
        #   3. Transform from camera_color_optical_frame to self.target_frame via TF
        #   4. Publish ObjectPose to /object_poses

    def localize_callback(self, request: LocalizeObject.Request, response: LocalizeObject.Response):
        """Handle on-demand localization service call."""
        if self.latest_depth is None:
            response.success = False
            response.message = 'No depth image received yet'
            return response

        if self.camera_info is None:
            response.success = False
            response.message = 'No camera info received yet'
            return response

        # TODO: Localize request.detection using latest depth + camera info
        # TODO: Populate response.position and response.distance

        response.success = False
        response.message = 'Not yet implemented'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
