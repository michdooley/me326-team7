#!/usr/bin/env python3
"""
Grasp Planner Node

Computes grasp poses for objects using GraspNet. Given a 3D object position
and the scene point cloud, returns a grasp pose and pre-grasp approach pose.

Services:
    /plan_grasp (tidybot_msgs/PlanGrasp) - plan a grasp for an object

Depends on:
    /plan_to_target (tidybot_msgs/PlanToTarget) - validates reachability

Parameters:
    model_path (str): path to GraspNet weights
    approach_offset (float): pre-grasp offset above grasp pose in meters (default 0.10)
    default_arm (str): arm to use when "auto" is requested (default "right")

Usage:
    ros2 run tidybot_perception grasp_planner_node
    ros2 run tidybot_perception grasp_planner_node --ros-args -p model_path:=/path/to/graspnet.pth
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PointStamped
from tidybot_msgs.srv import PlanGrasp, PlanToTarget


class GraspPlannerNode(Node):
    """GraspNet-based grasp planning node."""

    def __init__(self):
        super().__init__('grasp_planner_node')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('approach_offset', 0.10)
        self.declare_parameter('default_arm', 'right')

        self.model_path = self.get_parameter('model_path').value
        self.approach_offset = self.get_parameter('approach_offset').value
        self.default_arm = self.get_parameter('default_arm').value

        # Service
        self.grasp_srv = self.create_service(
            PlanGrasp, '/plan_grasp', self.plan_grasp_callback
        )

        # Client for reachability validation
        self.plan_to_target_client = self.create_client(PlanToTarget, '/plan_to_target')

        # Load GraspNet model
        self.model = None
        self._load_model()

        self.get_logger().info('GraspPlannerNode initialized')
        if not self.model_path:
            self.get_logger().warn('No model_path set — grasp planning will not run until configured')

    def _load_model(self):
        """Load the GraspNet model."""
        if not self.model_path:
            return
        # TODO: Load GraspNet model
        # self.model = load_graspnet(self.model_path)
        self.get_logger().info(f'GraspNet model loaded from {self.model_path}')

    def plan_grasp_callback(self, request: PlanGrasp.Request, response: PlanGrasp.Response):
        """Plan a grasp for the given object position."""
        if self.model is None:
            response.success = False
            response.message = 'GraspNet model not loaded — set model_path parameter'
            return response

        arm = request.arm_name if request.arm_name and request.arm_name != 'auto' else self.default_arm

        # TODO: Use GraspNet to predict grasp pose from object position + scene point cloud
        # TODO: Compute pre_grasp_pose (offset above grasp_pose by self.approach_offset)
        # TODO: Optionally validate reachability via /plan_to_target service
        # TODO: Populate response fields:
        #   response.grasp_pose = ...
        #   response.pre_grasp_pose = ...
        #   response.arm_used = arm
        #   response.grasp_width = ...

        response.success = False
        response.message = 'Not yet implemented'
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
