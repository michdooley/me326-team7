#!/usr/bin/env python3
"""
Grasp Planner Client Helper

Simple client library for calling the GPD grasp planner service (/plan_grasp).
Provides synchronous and asynchronous wrappers around the ROS2 service call.

Usage:
    from grasp_planner_client import GraspPlannerClient
    
    client = GraspPlannerClient(node)
    
    # Synchronous call (blocks until response)
    grasp_pose, pre_grasp_pose = client.plan_grasp_sync(
        object_position=(0.4, 0.0, 0.2),
        object_class="block",
        arm_name="right",
        timeout=3.0
    )
    
    # Or asynchronous
    future = client.plan_grasp_async(
        object_position=(0.4, 0.0, 0.2),
        object_class="block",
        arm_name="right"
    )
    # ... do other work ...
    grasp_pose, pre_grasp_pose = client.get_grasp_result(future)
"""

from typing import Tuple, Optional
import geometry_msgs.msg
from tidybot_msgs.srv import PlanGrasp

import rclpy
from rclpy.node import Node


class GraspPlannerClient:
    """Client for GPD-based grasp planning."""

    def __init__(self, node: Node, service_name: str = '/plan_grasp', timeout: float = 5.0):
        """
        Initialize the grasp planner client.

        Args:
            node: ROS2 node
            service_name: name of the grasp planning service
            timeout: default timeout for synchronous calls (seconds)
        """
        self.node = node
        self.client = node.create_client(PlanGrasp, service_name)
        self.service_name = service_name
        self.default_timeout = timeout

        # Wait for service to be available
        if not self.client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"Service {service_name} not available after {timeout}s")

        self.node.get_logger().info(f"Connected to {service_name}")

    def plan_grasp_sync(
        self,
        object_position: Tuple[float, float, float],
        object_class: str = "block",
        arm_name: str = "auto",
        frame_id: str = "base_link",
        timeout: Optional[float] = None,
    ) -> Tuple[geometry_msgs.msg.Pose, geometry_msgs.msg.Pose]:
        """
        Plan a grasp synchronously (blocking).

        Args:
            object_position: (x, y, z) position of object in frame_id
            object_class: object class name for grasp strategy selection
            arm_name: "right", "left", or "auto"
            frame_id: coordinate frame of object_position
            timeout: timeout in seconds (uses default if None)

        Returns:
            (grasp_pose, pre_grasp_pose) tuple of geometry_msgs.Pose objects

        Raises:
            RuntimeError if service fails or times out
        """
        if timeout is None:
            timeout = self.default_timeout

        request = PlanGrasp.Request()
        request.object_position.header.frame_id = frame_id
        request.object_position.point.x = object_position[0]
        request.object_position.point.y = object_position[1]
        request.object_position.point.z = object_position[2]
        request.object_class = object_class
        request.arm_name = arm_name

        self.node.get_logger().debug(
            f"Planning grasp: position=({object_position[0]:.3f}, "
            f"{object_position[1]:.3f}, {object_position[2]:.3f}), "
            f"class={object_class}, arm={arm_name}"
        )

        future = self.client.call_async(request)

        try:
            # Wait for result with timeout
            import time
            start_time = time.time()
            while not future.done():
                if (time.time() - start_time) > timeout:
                    raise TimeoutError(
                        f"Grasp planning timed out after {timeout}s"
                    )
                rclpy.spin_once(self.node, timeout_sec=0.1)

            response = future.result()

            if not response.success:
                raise RuntimeError(f"Grasp planning failed: {response.message}")

            self.node.get_logger().info(
                f"Grasp planned successfully. Arm: {response.arm_used}, "
                f"Width: {response.grasp_width:.3f}m"
            )

            return response.grasp_pose, response.pre_grasp_pose

        except TimeoutError as e:
            self.node.get_logger().error(str(e))
            raise
        except Exception as e:
            self.node.get_logger().error(f"Grasp planning exception: {e}")
            raise RuntimeError(f"Grasp planning failed: {e}") from e

    def plan_grasp_async(
        self,
        object_position: Tuple[float, float, float],
        object_class: str = "block",
        arm_name: str = "auto",
        frame_id: str = "base_link",
    ) -> rclpy.task.Future:
        """
        Plan a grasp asynchronously (non-blocking).

        Args:
            object_position: (x, y, z) position of object
            object_class: object class name
            arm_name: "right", "left", or "auto"
            frame_id: coordinate frame

        Returns:
            rclpy.task.Future that will contain the response

        Usage:
            future = client.plan_grasp_async(...)
            # ... do other work ...
            response = future.result()  # blocks until ready
        """
        request = PlanGrasp.Request()
        request.object_position.header.frame_id = frame_id
        request.object_position.point.x = object_position[0]
        request.object_position.point.y = object_position[1]
        request.object_position.point.z = object_position[2]
        request.object_class = object_class
        request.arm_name = arm_name

        self.node.get_logger().debug(
            f"Planning grasp (async): position=({object_position[0]:.3f}, "
            f"{object_position[1]:.3f}, {object_position[2]:.3f})"
        )

        return self.client.call_async(request)

    def get_grasp_result(
        self, future: rclpy.task.Future
    ) -> Tuple[geometry_msgs.msg.Pose, geometry_msgs.msg.Pose]:
        """
        Get result from an asynchronous grasp planning call.

        Args:
            future: rclpy.task.Future from plan_grasp_async()

        Returns:
            (grasp_pose, pre_grasp_pose) tuple

        Raises:
            RuntimeError if the service call failed
        """
        try:
            response = future.result()

            if not response.success:
                raise RuntimeError(f"Grasp planning failed: {response.message}")

            self.node.get_logger().info(
                f"Grasp planned successfully. Arm: {response.arm_used}"
            )

            return response.grasp_pose, response.pre_grasp_pose

        except Exception as e:
            self.node.get_logger().error(f"Failed to get grasp result: {e}")
            raise RuntimeError(f"Grasp planning failed: {e}") from e


# Example usage in task1_retrieve.py:
"""
from grasp_planner_client import GraspPlannerClient

class Task1RetrieveNode(Node):
    def __init__(self):
        super().__init__('task1_retrieve')
        try:
            self.grasp_client = GraspPlannerClient(self)
        except RuntimeError as e:
            self.get_logger().error(f"Failed to connect to grasp planner: {e}")
    
    def plan_grasp_state(self):
        '''PLAN_GRASP state'''
        try:
            grasp_pose, pre_grasp_pose = self.grasp_client.plan_grasp_sync(
                object_position=(0.4, 0.0, 0.2),
                object_class=self.target_object,
                arm_name="right",
                timeout=3.0
            )
            
            # Use the grasp poses
            self.move_arm_to_pose(pre_grasp_pose)
            # ... rest of task ...
            
            self.state = Task1State.PRE_GRASP
            
        except RuntimeError as e:
            self.get_logger().error(f"Grasp planning failed: {e}")
            self.state = Task1State.SEARCH  # retry
"""
