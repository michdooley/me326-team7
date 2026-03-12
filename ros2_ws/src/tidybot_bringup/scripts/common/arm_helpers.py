"""Arm/gripper/camera-tilt mixin for TidyBot2 task scripts."""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from tidybot_msgs.msg import ArmCommand


class ArmHelpersMixin:
    """Mixin providing arm, gripper, and camera-tilt control.

    Expects the host class to have:
        self.gripper_pub, self.arm_pub, self.pan_tilt_pub,
        self.plan_client, self.current_tilt,
        self.GRASP_GRIPPER_CLOSE_REPEATS, self.GRASP_GRIPPER_CLOSE_WAIT
    """

    def _set_gripper(self, position):
        msg = Float64MultiArray()
        msg.data = [position]
        for _ in range(10):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(0.5)

    def _close_gripper_firm(self):
        msg = Float64MultiArray()
        msg.data = [1.0]
        for _ in range(self.GRASP_GRIPPER_CLOSE_REPEATS):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(self.GRASP_GRIPPER_CLOSE_WAIT)
        for _ in range(10):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def _go_home(self, duration=3.0):
        self.get_logger().info(
            f'[ARM] Returning arm home over {duration}s')
        msg = ArmCommand()
        msg.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.duration = duration
        self.arm_pub.publish(msg)
        time.sleep(duration + 0.5)

    def _retract_holding(self, trophy_pose=None):
        """Retract arm up and forward from trophy pose, out of camera's way."""
        self.get_logger().info('[ARM] Retracting arm to overhead pose via IK')
        pose = Pose()
        if trophy_pose is not None:
            pose.position.x = trophy_pose.position.x
            pose.position.y = trophy_pose.position.y - 0.12
            pose.position.z = trophy_pose.position.z + 0.10
        else:
            pose.position.x = 0.0
            pose.position.y = -0.20
            pose.position.z = 0.45
        if not self._bin_plan_and_execute(pose, 'RETRACT', use_orientation=False):
            self.get_logger().warn('[ARM] IK retract failed, gripper remains closed')
        time.sleep(0.5)

    def _set_camera_tilt(self, tilt):
        self.current_tilt = float(np.clip(tilt, -1.5, 0.9))
        pt_msg = Float64MultiArray()
        pt_msg.data = [0.0, self.current_tilt]
        for _ in range(5):
            self.pan_tilt_pub.publish(pt_msg)
            rclpy.spin_once(self, timeout_sec=0.02)
