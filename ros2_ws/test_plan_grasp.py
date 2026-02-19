#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tidybot_msgs.srv import PlanGrasp


def run_test():
    rclpy.init()
    node = Node('test_grasp_client')
    client = node.create_client(PlanGrasp, '/plan_grasp')
    if not client.wait_for_service(timeout_sec=5.0):
        print('Service /plan_grasp not available')
        return 1

    req = PlanGrasp.Request()
    req.object_position.header.frame_id = 'base_link'
    req.object_position.point.x = 0.4
    req.object_position.point.y = 0.0
    req.object_position.point.z = 0.2
    req.object_class = 'block'
    req.arm_name = 'right'

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    if not future.done():
        print('Service call timed out')
        return 2

    res = future.result()
    print('Service response: success=', res.success, ' message=', res.message)
    print('Grasp pose:', res.grasp_pose)
    print('Pre-grasp pose:', res.pre_grasp_pose)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(run_test())
