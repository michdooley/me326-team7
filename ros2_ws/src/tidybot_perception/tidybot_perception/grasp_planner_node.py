#!/usr/bin/env python3
"""
Grasp Planner Node

Computes grasp poses for objects using GraspNet (heuristic-based). Given a 3D
object position and the scene point cloud from RGB-D camera, returns a grasp
pose and pre-grasp approach pose.

Services:
    /plan_grasp (tidybot_msgs/PlanGrasp) - plan a grasp for an object

Subscribes to:
    /camera/color/image_raw (sensor_msgs/Image) - RGB camera stream
    /camera/depth/image_raw (sensor_msgs/Image) - Depth camera stream
    /camera/color/camera_info (sensor_msgs/CameraInfo) - Camera intrinsics

Parameters:
    approach_offset (float): pre-grasp offset above grasp pose in meters (default 0.10)
    default_arm (str): arm to use when "auto" is requested (default "right")
    score_threshold (float): minimum grasp quality score (default 0.3)

Usage:
    ros2 run tidybot_perception grasp_planner_node
    ros2 run tidybot_perception grasp_planner_node --ros-args -p approach_offset:=0.15
"""

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PointStamped
from tidybot_msgs.srv import PlanGrasp

# Import grasp_checker utility (local to tidybot_perception package)
from tidybot_perception.grasp_checker import GraspChecker, GraspPose as GraspPoseUtil


class GraspPlannerNode(Node):
    """Heuristic-based grasp planning node using geometric sampling."""

    def __init__(self):
        super().__init__('grasp_planner_node')

        # Parameters
        self.declare_parameter('approach_offset', 0.10)
        self.declare_parameter('default_arm', 'right')
        self.declare_parameter('score_threshold', 0.3)
        self.declare_parameter('sample_ratio', 0.1)

        self.approach_offset = self.get_parameter('approach_offset').value
        self.default_arm = self.get_parameter('default_arm').value
        self.score_threshold = self.get_parameter('score_threshold').value
        self.sample_ratio = self.get_parameter('sample_ratio').value

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Latest camera data
        self.rgb_image = None
        self.depth_image = None
        self.camera_intrinsics = None

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.info_callback, 10
        )

        # Service
        self.grasp_srv = self.create_service(
            PlanGrasp, '/plan_grasp', self.plan_grasp_callback
        )

        # Initialize grasp checker
        self.grasp_checker = GraspChecker(
            score_threshold=self.score_threshold,
            sample_ratio=self.sample_ratio
        )

        self.get_logger().info('GraspPlannerNode initialized (heuristic mode)')
        self.get_logger().info(f'  approach_offset: {self.approach_offset}m')
        self.get_logger().info(f'  default_arm: {self.default_arm}')
        self.get_logger().info(f'  score_threshold: {self.score_threshold}')
        self.get_logger().info(f'  sample_ratio: {self.sample_ratio}')

    def rgb_callback(self, msg: Image):
        """Store latest RGB image."""
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            if self.rgb_image is not None:
                self.get_logger().info(f"Received RGB image: {self.rgb_image.shape}", once=True)
        except Exception as e:
            self.get_logger().error(f"Failed to convert RGB image: {e}")

    def depth_callback(self, msg: Image):
        """Store latest depth image."""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            if self.depth_image is not None:
                self.get_logger().info(f"Received depth image: {self.depth_image.shape}", once=True)
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

    def info_callback(self, msg: CameraInfo):
        """Extract camera intrinsics."""
        self.camera_intrinsics = np.array(msg.k).reshape(3, 3)
        self.get_logger().info(f"Received camera intrinsics: fx={self.camera_intrinsics[0,0]:.1f}", once=True)

    def plan_grasp_callback(self, request: PlanGrasp.Request, response: PlanGrasp.Response):
        """
        Plan a grasp for the given object position.

        Uses geometric heuristics (antipodal grasp sampling) from the grasp_checker
        utility to find viable grasps in the camera view, then selects the one
        closest to the requested object position.
        """
        # Check if we have camera data
        if self.rgb_image is None or self.depth_image is None or self.camera_intrinsics is None:
            response.success = False
            response.message = 'Camera data not available — waiting for /camera/* topics'
            self.get_logger().warn(response.message)
            return response

        # Determine which arm to use
        arm = request.arm_name if request.arm_name and request.arm_name != 'auto' else self.default_arm

        try:
            self.get_logger().info(f'Planning grasp for {request.object_class} at position: '
                                 f'[{request.object_position.point.x:.3f}, '
                                 f'{request.object_position.point.y:.3f}, '
                                 f'{request.object_position.point.z:.3f}]')

            # Run grasp detection on current camera view
            grasps = self.grasp_checker.predict(
                self.rgb_image,
                self.depth_image,
                self.camera_intrinsics,
                num_grasps=10  # Get multiple candidates to find best match
            )

            if not grasps:
                response.success = False
                response.message = 'No valid grasps detected in scene (try lowering score_threshold)'
                self.get_logger().warn(response.message)
                return response

            # Find grasp closest to the requested object position
            # (All grasps are in camera frame, need to transform or use relative selection)
            # For simplicity, use the top-scored grasp for now
            # TODO: Transform object_position to camera frame and find nearest grasp
            best_grasp = grasps[0]

            self.get_logger().info(f'Selected grasp with score {best_grasp.score:.3f}')

            # Convert grasp to ROS Pose message (in camera_optical_frame)
            grasp_pose = Pose()
            grasp_pose.position.x = float(best_grasp.position[0])
            grasp_pose.position.y = float(best_grasp.position[1])
            grasp_pose.position.z = float(best_grasp.position[2])

            # Convert orientation matrix to quaternion (xyzw format)
            quat = best_grasp.to_quaternion('xyzw')
            grasp_pose.orientation.x = float(quat[0])
            grasp_pose.orientation.y = float(quat[1])
            grasp_pose.orientation.z = float(quat[2])
            grasp_pose.orientation.w = float(quat[3])

            # Compute pre-grasp pose (offset along approach direction)
            # Approach direction is the Z-axis of the grasp orientation
            approach_vector = best_grasp.orientation[:, 2]  # Third column (Z-axis)
            pre_grasp_position = best_grasp.position - approach_vector * self.approach_offset

            pre_grasp_pose = Pose()
            pre_grasp_pose.position.x = float(pre_grasp_position[0])
            pre_grasp_pose.position.y = float(pre_grasp_position[1])
            pre_grasp_pose.position.z = float(pre_grasp_position[2])
            pre_grasp_pose.orientation = grasp_pose.orientation  # Same orientation

            # Estimate gripper width (distance between grasp contact points)
            # For heuristic approach, use a conservative default
            grasp_width = 0.05  # 5cm default

            # Populate response
            response.success = True
            response.grasp_pose = grasp_pose
            response.pre_grasp_pose = pre_grasp_pose
            response.arm_used = arm
            response.grasp_width = grasp_width
            response.message = f'Grasp planned successfully (score: {best_grasp.score:.3f})'

            self.get_logger().info(response.message)
            self.get_logger().info(f'  Grasp position: [{grasp_pose.position.x:.3f}, '
                                 f'{grasp_pose.position.y:.3f}, {grasp_pose.position.z:.3f}]')
            self.get_logger().info(f'  Pre-grasp position: [{pre_grasp_pose.position.x:.3f}, '
                                 f'{pre_grasp_pose.position.y:.3f}, {pre_grasp_pose.position.z:.3f}]')

        except Exception as e:
            response.success = False
            response.message = f'Grasp planning failed: {type(e).__name__}: {e}'
            self.get_logger().error(response.message)
            import traceback
            self.get_logger().error(f'Traceback:\n{traceback.format_exc()}')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = GraspPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
