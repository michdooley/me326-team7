"""
Shared helper functions for TidyBot2 task scripts.

Centralizes common patterns used across test scripts and state machines.

Usage:
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from robot_helpers import call_service_sync, create_pose, open_gripper, close_gripper, stop_base
"""

from geometry_msgs.msg import Pose, Twist, Quaternion, Point
from std_msgs.msg import Float64MultiArray


def call_service_sync(node, client, request, timeout_sec=15.0):
    """
    Call a ROS2 service synchronously (blocking).

    Args:
        node: ROS2 Node instance (for logging and spinning)
        client: ROS2 service client
        request: Service request message
        timeout_sec: Timeout in seconds

    Returns:
        Service response, or None on timeout
    """
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f'Service {client.srv_name} not available')
        return None

    future = client.call_async(request)

    import rclpy
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)

    if future.result() is None:
        node.get_logger().error(f'Service call to {client.srv_name} timed out')
        return None

    return future.result()


def create_pose(x, y, z, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
    """
    Create a geometry_msgs/Pose from position and quaternion.

    Args:
        x, y, z: Position
        qw, qx, qy, qz: Quaternion (wxyz convention, default = identity)

    Returns:
        geometry_msgs/Pose
    """
    pose = Pose()
    pose.position = Point(x=float(x), y=float(y), z=float(z))
    pose.orientation = Quaternion(w=float(qw), x=float(qx), y=float(qy), z=float(qz))
    return pose


def open_gripper(publisher, effort=0.5):
    """
    Publish an open gripper command.

    Args:
        publisher: ROS2 publisher for gripper topic (Float64MultiArray)
        effort: Gripper effort 0.0–1.0 (unused in sim, controls force on real)
    """
    msg = Float64MultiArray()
    msg.data = [0.0]  # 0.0 = open
    publisher.publish(msg)


def close_gripper(publisher, effort=0.5):
    """
    Publish a close gripper command.

    Args:
        publisher: ROS2 publisher for gripper topic (Float64MultiArray)
        effort: Gripper effort 0.0–1.0
    """
    msg = Float64MultiArray()
    msg.data = [1.0]  # 1.0 = closed
    publisher.publish(msg)


def stop_base(cmd_vel_pub):
    """Publish zero velocity to stop the base."""
    cmd_vel_pub.publish(Twist())


# Standard end-effector orientations (wxyz quaternions)
# Fingers pointing straight down
ORIENT_FINGERS_DOWN = (0.5, 0.5, 0.5, -0.5)
# Fingers pointing down, rotated 90 degrees
ORIENT_FINGERS_DOWN_ROT90 = (0.707107, 0.0, 0.707107, 0.0)

# Common arm positions: [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]
ARM_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ARM_SLEEP = [0.0, -1.76, 1.56, 0.0, 0.65, 0.0]
ARM_READY = [0.0, 0.0, 0.6, 0.0, 0.9, 0.0]
