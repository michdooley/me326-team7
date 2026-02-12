#!/usr/bin/env python3
"""
Navigate to Object Module

Searches for a target object by scanning the environment, then navigates
toward it while avoiding obstacles via depth, and positions the robot
within grasping range.

Internal sub-states: SCAN → APPROACH → ALIGN → POSITIONED

Subscribes:
    /detections (Detection2DArray) - from detector_node
    /object_poses (ObjectPose) - from object_localizer_node
    /odom (Odometry) - robot position
    /camera/depth/image_raw (Image) - for obstacle avoidance

Publishes:
    /cmd_vel (Twist) - base velocity
    /camera/pan_tilt_cmd (Float64MultiArray) - camera pointing

Usage:
    # As a standalone test:
    ros2 run tidybot_bringup navigate_to_object.py

    # Or import in the state machine:
    from navigate_to_object import NavigateToObject
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import time
from enum import Enum, auto

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tidybot_msgs.msg import Detection2DArray, ObjectPose


class NavState(Enum):
    """Internal navigation sub-states."""
    SCAN       = auto()  # Rotate in place, looking for target object
    APPROACH   = auto()  # Drive toward detected object
    ALIGN      = auto()  # Fine-tune position for grasping
    POSITIONED = auto()  # Done — object is within grasping range


# Configuration
SCAN_ROTATION_SPEED = 0.3     # rad/s while scanning
APPROACH_SPEED = 0.15         # m/s while approaching
GRASP_DISTANCE = 0.40         # meters — stop this far from object
ALIGN_TOLERANCE = 0.05        # meters — position tolerance for alignment


class NavigateToObject(Node):
    """
    Navigate to a target object. Can be run standalone or used by the state machine.

    Args:
        target_object: The class name to search for (e.g. "apple")
    """

    def __init__(self, target_object: str = ''):
        super().__init__('navigate_to_object')

        self.target_object = target_object

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pan_tilt_pub = self.create_publisher(Float64MultiArray, '/camera/pan_tilt_cmd', 10)

        # Subscribers
        self.detections_sub = self.create_subscription(
            Detection2DArray, '/detections', self.detections_callback, 10
        )
        self.object_poses_sub = self.create_subscription(
            ObjectPose, '/object_poses', self.object_poses_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, qos
        )

        # Internal state
        self.nav_state = NavState.SCAN
        self.state_start_time = time.time()
        self.latest_detections = None
        self.latest_object_pose = None
        self.latest_odom = None
        self.latest_depth = None

        # Control loop at 50Hz
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info(f'NavigateToObject: searching for "{target_object}"')

    # ---- Subscriber callbacks ----

    def detections_callback(self, msg: Detection2DArray):
        self.latest_detections = msg

    def object_poses_callback(self, msg: ObjectPose):
        self.latest_object_pose = msg

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def depth_callback(self, msg: Image):
        self.latest_depth = msg

    # ---- Public interface ----

    def is_positioned(self) -> bool:
        """Returns True when the robot is positioned within grasping range."""
        return self.nav_state == NavState.POSITIONED

    def get_object_pose(self) -> ObjectPose:
        """Returns the latest 3D pose of the target object, or None."""
        return self.latest_object_pose

    def set_target(self, target_object: str):
        """Set a new target object and reset to SCAN state."""
        self.target_object = target_object
        self.nav_state = NavState.SCAN
        self.state_start_time = time.time()
        self.get_logger().info(f'New target: "{target_object}"')

    # ---- Control loop ----

    def control_loop(self):
        """Navigation state machine at 50Hz."""
        now = time.time()
        elapsed = now - self.state_start_time

        if not self.target_object:
            return

        # ==================== SCAN ====================
        if self.nav_state == NavState.SCAN:
            # TODO: Rotate in place to scan for the target object
            # Check self.latest_detections for target_object match
            # If found, transition to APPROACH
            #
            # twist = Twist()
            # twist.angular.z = SCAN_ROTATION_SPEED
            # self.cmd_vel_pub.publish(twist)
            pass

        # ==================== APPROACH ====================
        elif self.nav_state == NavState.APPROACH:
            # TODO: Drive toward object using self.latest_object_pose
            # Use depth image for obstacle avoidance
            # Stop when within GRASP_DISTANCE
            # Transition to ALIGN when close enough
            pass

        # ==================== ALIGN ====================
        elif self.nav_state == NavState.ALIGN:
            # TODO: Fine-tune position so object is centered in view
            # and at correct distance for grasping
            # Transition to POSITIONED when aligned
            pass

        # ==================== POSITIONED ====================
        elif self.nav_state == NavState.POSITIONED:
            # Stop the base — we're in position
            self.cmd_vel_pub.publish(Twist())


def main(args=None):
    """Standalone test: search for an object passed as ROS parameter."""
    rclpy.init(args=args)

    # Can pass target via: --ros-args -p target_object:=apple
    node = NavigateToObject(target_object='')
    node.declare_parameter('target_object', '')
    target = node.get_parameter('target_object').value
    if target:
        node.set_target(target)
    else:
        node.get_logger().warn('No target_object parameter set. Use: --ros-args -p target_object:=apple')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
