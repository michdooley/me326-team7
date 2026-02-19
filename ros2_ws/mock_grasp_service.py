#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tidybot_msgs.srv import PlanGrasp
from geometry_msgs.msg import Pose


def handle_plan_grasp(request, response, node: Node):
    response.success = True
    response.message = 'mocked'

    grasp = Pose()
    grasp.position.x = 0.4
    grasp.position.y = 0.0
    grasp.position.z = 0.2
    grasp.orientation.w = 1.0

    pre = Pose()
    pre.position.x = 0.4
    pre.position.y = 0.0
    pre.position.z = 0.3
    pre.orientation.w = 1.0

    response.grasp_pose = grasp
    response.pre_grasp_pose = pre
    response.arm_used = 'right'
    response.grasp_width = 0.08

    node.get_logger().info('Returning mocked grasp')
    return response


def main():
    rclpy.init()
    node = Node('mock_grasp_server')
    node.create_service(PlanGrasp, '/plan_grasp', lambda req, res: handle_plan_grasp(req, res, node))
    node.get_logger().info('Mock /plan_grasp service ready')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
