#!/usr/bin/env python3
"""
TidyBot2 Object Retrieval Task (Objective 1)

Finds a red cube in a cluttered environment, navigates to it while avoiding
obstacles, picks it up, and returns to the starting position.

Prerequisites:
    ros2 launch tidybot_bringup sim.launch.py scene:=scene_obstacles.xml

Usage:
    ros2 run tidybot_bringup objective1_task.py
"""

import sys
import os
import time
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration

from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float64MultiArray
from tidybot_msgs.srv import PlanToTarget

import tf2_ros

try:
    from cv_bridge import CvBridge
except ImportError:
    print("ERROR: cv_bridge is required.")
    raise

# Add scripts directory to path so subtasks package is importable
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from subtasks import SubtaskStatus, ScanForObject, NavigateTo, PickUpObject


class TaskState(Enum):
    INIT = auto()
    SCAN = auto()
    NAVIGATE_TO_OBJECT = auto()
    PICK_UP = auto()
    NAVIGATE_HOME = auto()
    DONE = auto()
    FAILED = auto()


class ObjectRetrievalNode(Node):
    """Master node for the object retrieval task."""

    def __init__(self):
        super().__init__('object_retrieval')

        # ── Camera subscribers ──
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cv_bridge = CvBridge()

        self.latest_rgb = None
        self.latest_depth = None
        self.camera_K = None

        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self._rgb_cb, qos)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self._depth_cb, qos)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._camera_info_cb, qos)

        # ── Publishers ──
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10)
        self.right_gripper_pub = self.create_publisher(
            Float64MultiArray, '/right_gripper/cmd', 10)
        self.base_target_pub = self.create_publisher(
            Pose2D, '/base/target_pose', 10)

        # ── Navigation feedback ──
        self.base_goal_reached = False
        self.goal_sub = self.create_subscription(
            Bool, '/base/goal_reached', self._goal_reached_cb, 10)

        # ── Motion planning service ──
        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')

        # ── TF2 ──
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── State machine ──
        self.state = TaskState.INIT
        self.state_start_time = time.time()
        self.home_pose = None           # (x, y, theta) in odom frame
        self.object_odom_pos = None     # np.array([x, y, z]) in odom frame

        # ── Subtask instances ──
        self.scan_task = ScanForObject(self)
        self.nav_task = NavigateTo(self)
        self.pickup_task = PickUpObject(self)

        # ── Control loop at 20Hz ──
        self.timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info('=' * 50)
        self.get_logger().info('Object Retrieval Task — Initialized')
        self.get_logger().info('=' * 50)

    # ── Subscriber callbacks ──────────────────────────────────────────

    def _rgb_cb(self, msg: Image):
        try:
            self.latest_rgb = self.cv_bridge.imgmsg_to_cv2(msg, 'rgb8')
        except Exception as e:
            self.get_logger().warn(f'RGB conversion failed: {e}')

    def _depth_cb(self, msg: Image):
        try:
            self.latest_depth = self.cv_bridge.imgmsg_to_cv2(msg, '16UC1')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')

    def _camera_info_cb(self, msg: CameraInfo):
        self.camera_K = np.array(msg.k).reshape(3, 3)

    def _goal_reached_cb(self, msg: Bool):
        if msg.data:
            self.base_goal_reached = True

    # ── State machine ─────────────────────────────────────────────────

    def _transition_to(self, new_state):
        self.get_logger().info(f'State: {self.state.name} → {new_state.name}')
        self.state = new_state
        self.state_start_time = time.time()

    def _control_loop(self):
        if self.state == TaskState.INIT:
            self._handle_init()
        elif self.state == TaskState.SCAN:
            self._handle_scan()
        elif self.state == TaskState.NAVIGATE_TO_OBJECT:
            self._handle_navigate_to_object()
        elif self.state == TaskState.PICK_UP:
            self._handle_pick_up()
        elif self.state == TaskState.NAVIGATE_HOME:
            self._handle_navigate_home()
        elif self.state == TaskState.DONE:
            self._handle_done()
        elif self.state == TaskState.FAILED:
            self._handle_failed()

    # ── State handlers ────────────────────────────────────────────────

    def _handle_init(self):
        """Wait for camera data and TF, record home position."""
        if (self.latest_rgb is None
                or self.latest_depth is None
                or self.camera_K is None):
            return

        try:
            self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return

        # Record home position
        self.home_pose = self._get_base_pose_in_odom()
        if self.home_pose is None:
            return

        hx, hy, htheta = self.home_pose
        self.get_logger().info(
            f'Home recorded: ({hx:.2f}, {hy:.2f}, '
            f'{np.degrees(htheta):.0f}° mujoco)')
        self._transition_to(TaskState.SCAN)

    def _handle_scan(self):
        """Run the ScanForObject subtask."""
        if not self.scan_task.started:
            self.scan_task.start(
                rotation_speed=0.4,
                max_scan_attempts=3,
                drive_distance=0.5,
                camera_tilt=0.5,
            )

        status = self.scan_task.update()

        if status == SubtaskStatus.SUCCEEDED:
            self.object_odom_pos = self.scan_task.get_result()
            self.get_logger().info(
                f'Object found at odom: ({self.object_odom_pos[0]:.2f}, '
                f'{self.object_odom_pos[1]:.2f}, {self.object_odom_pos[2]:.2f})')
            self.scan_task.started = False
            self._transition_to(TaskState.NAVIGATE_TO_OBJECT)
        elif status == SubtaskStatus.FAILED:
            self.get_logger().error('Scan failed — object not found!')
            self.scan_task.started = False
            self._transition_to(TaskState.FAILED)

    def _handle_navigate_to_object(self):
        """Navigate toward the detected object, stopping short for pickup."""
        if not self.nav_task.started:
            current = self._get_base_pose_in_odom()
            if current is None:
                return

            ox, oy, oz = self.object_odom_pos
            cx, cy, ctheta = current

            dx = ox - cx
            dy = oy - cy
            dist = np.sqrt(dx**2 + dy**2)

            # Stop 0.4m short of the object (within arm reach)
            approach_dist = max(0.0, dist - 0.4)
            if dist > 0.01:
                approach_x = cx + (dx / dist) * approach_dist
                approach_y = cy + (dy / dist) * approach_dist
            else:
                approach_x, approach_y = cx, cy

            # Face toward the object (real-world heading)
            approach_theta = np.arctan2(dy, dx)

            self.nav_task.start(
                target_x=approach_x,
                target_y=approach_y,
                target_theta=approach_theta,
                position_tolerance=0.15,
                angle_tolerance=0.2,
                obstacle_threshold=0.5,
                timeout=30.0,
            )

        status = self.nav_task.update()

        if status == SubtaskStatus.SUCCEEDED:
            self.get_logger().info('Arrived near object — starting pickup')
            self.nav_task.started = False
            self._transition_to(TaskState.PICK_UP)
        elif status == SubtaskStatus.FAILED:
            self.get_logger().error('Navigation to object failed!')
            self.nav_task.started = False
            self._transition_to(TaskState.FAILED)

    def _handle_pick_up(self):
        """Run the blocking pickup sequence."""
        # Cancel timer during blocking pickup
        self.timer.cancel()

        self.pickup_task.run_blocking(self.object_odom_pos)

        if self.pickup_task.success:
            self.get_logger().info('Pickup successful! Navigating home.')
            self._transition_to(TaskState.NAVIGATE_HOME)
        else:
            self.get_logger().error('Pickup failed!')
            self._transition_to(TaskState.FAILED)

        # Restart the control loop timer
        self.timer = self.create_timer(0.05, self._control_loop)

    def _handle_navigate_home(self):
        """Navigate back to the starting position."""
        if not self.nav_task.started:
            hx, hy, htheta = self.home_pose
            # Convert MuJoCo theta to real-world heading
            home_heading = htheta - np.pi / 2

            self.nav_task.start(
                target_x=hx,
                target_y=hy,
                target_theta=home_heading,
                position_tolerance=0.15,
                angle_tolerance=0.3,
                obstacle_threshold=0.5,
                timeout=45.0,
            )

        status = self.nav_task.update()

        if status == SubtaskStatus.SUCCEEDED:
            self.get_logger().info('Arrived home!')
            self.nav_task.started = False
            self._transition_to(TaskState.DONE)
        elif status == SubtaskStatus.FAILED:
            self.get_logger().warn('Navigation home failed, but object was picked up.')
            self.nav_task.started = False
            self._transition_to(TaskState.DONE)

    def _handle_done(self):
        """Log completion once."""
        if time.time() - self.state_start_time < 0.1:
            self.get_logger().info('')
            self.get_logger().info('=' * 50)
            self.get_logger().info('MISSION COMPLETE: Object retrieved!')
            self.get_logger().info('=' * 50)

    def _handle_failed(self):
        """Log failure once."""
        if time.time() - self.state_start_time < 0.1:
            self.get_logger().error('')
            self.get_logger().error('=' * 50)
            self.get_logger().error('MISSION FAILED')
            self.get_logger().error('=' * 50)

    # ── Helpers ───────────────────────────────────────────────────────

    def _get_base_pose_in_odom(self):
        """Get (x, y, theta) in odom frame. Theta in MuJoCo convention."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time(), timeout=Duration(seconds=1.0))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = np.arctan2(siny, cosy)
        return (t.x, t.y, theta)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectRetrievalNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Emergency stop
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.get_logger().info('Interrupted — stopping robot.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
