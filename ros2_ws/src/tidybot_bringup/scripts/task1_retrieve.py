#!/usr/bin/env python3
"""
Task 1: Object Retrieval State Machine

Orchestrates the full object retrieval task:
    LISTEN → SEARCH → APPROACH → PLAN_GRASP → PRE_GRASP → GRASP →
    VERIFY_GRASP → RETURN → DONE

Each state calls a service or publishes a command, then waits for the result.
Heavy logic lives in service nodes (detector, localizer, grasp planner, motion planner).

Topics used:
    /cmd_vel (Twist) - base velocity
    /camera/pan_tilt_cmd (Float64MultiArray) - camera pointing
    /right_arm/cmd, /left_arm/cmd (ArmCommand) - arm positions
    /right_gripper/cmd, /left_gripper/cmd (Float64MultiArray) - gripper control
    /detections (Detection2DArray) - object detections (subscribed)
    /object_poses (ObjectPose) - 3D object positions (subscribed)
    /joint_states (JointState) - robot state (subscribed)
    /odom (Odometry) - base position (subscribed)

Services called:
    /detect (Detect) - on-demand object detection
    /localize_object (LocalizeObject) - 2D→3D conversion
    /plan_grasp (PlanGrasp) - grasp pose planning
    /plan_to_target (PlanToTarget) - arm motion planning + execution

Usage:
    # Terminal 1: Launch sim + perception
    ros2 launch tidybot_bringup task.launch.py

    # Terminal 2: Run task
    ros2 run tidybot_bringup task1_retrieve.py
"""

import rclpy
from rclpy.node import Node

import time
from enum import Enum, auto

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tidybot_msgs.msg import ArmCommand, Detection2DArray, ObjectPose
from tidybot_msgs.srv import Detect, LocalizeObject, PlanGrasp, PlanToTarget


class Task1State(Enum):
    """States for the object retrieval task."""
    LISTEN       = auto()  # Wait for voice command → get target object
    SEARCH       = auto()  # Scan environment for target object
    APPROACH     = auto()  # Navigate to grasping range
    PLAN_GRASP   = auto()  # Call /plan_grasp service
    PRE_GRASP    = auto()  # Move arm to approach pose via /plan_to_target
    GRASP        = auto()  # Execute grasp + close gripper
    VERIFY_GRASP = auto()  # Check if object is grasped (gripper feedback)
    RETURN       = auto()  # Navigate back to start position
    DONE         = auto()
    ERROR        = auto()


class Task1Retrieve(Node):
    """Object retrieval state machine."""

    def __init__(self):
        super().__init__('task1_retrieve')

        # ---- Publishers ----
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.right_arm_pub = self.create_publisher(ArmCommand, '/right_arm/cmd', 10)
        self.left_arm_pub = self.create_publisher(ArmCommand, '/left_arm/cmd', 10)
        self.right_gripper_pub = self.create_publisher(Float64MultiArray, '/right_gripper/cmd', 10)
        self.left_gripper_pub = self.create_publisher(Float64MultiArray, '/left_gripper/cmd', 10)
        self.pan_tilt_pub = self.create_publisher(Float64MultiArray, '/camera/pan_tilt_cmd', 10)

        # ---- Subscribers ----
        self.detections_sub = self.create_subscription(
            Detection2DArray, '/detections', self.detections_callback, 10
        )
        self.object_poses_sub = self.create_subscription(
            ObjectPose, '/object_poses', self.object_poses_callback, 10
        )
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # ---- Service Clients ----
        self.detect_client = self.create_client(Detect, '/detect')
        self.localize_client = self.create_client(LocalizeObject, '/localize_object')
        self.plan_grasp_client = self.create_client(PlanGrasp, '/plan_grasp')
        self.plan_to_target_client = self.create_client(PlanToTarget, '/plan_to_target')

        # ---- State machine ----
        self.state = Task1State.LISTEN
        self.state_start_time = None

        # ---- Task data ----
        self.target_object = None       # e.g. "apple"
        self.latest_detections = None
        self.latest_object_pose = None
        self.latest_joint_states = None
        self.latest_odom = None
        self.start_pose = None          # where to return to
        self.grasp_plan = None          # result from /plan_grasp
        self.active_arm = 'right'       # which arm to use

        # Control loop at 50Hz
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info('=' * 50)
        self.get_logger().info('Task 1: Object Retrieval')
        self.get_logger().info('=' * 50)
        self.get_logger().info(
            'States: LISTEN → SEARCH → APPROACH → PLAN_GRASP → '
            'PRE_GRASP → GRASP → VERIFY → RETURN → DONE'
        )

    # ---- Subscriber callbacks ----

    def detections_callback(self, msg: Detection2DArray):
        self.latest_detections = msg

    def object_poses_callback(self, msg: ObjectPose):
        self.latest_object_pose = msg

    def joint_states_callback(self, msg: JointState):
        self.latest_joint_states = msg

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    # ---- State machine ----

    def transition_to(self, new_state: Task1State):
        """Transition to a new state."""
        self.get_logger().info(f'State: {self.state.name} → {new_state.name}')
        self.state = new_state
        self.state_start_time = time.time()

    def control_loop(self):
        """Main control loop — runs the state machine at 50Hz."""
        now = time.time()

        if self.state_start_time is None:
            self.state_start_time = now
            self.get_logger().info(f'Starting state: {self.state.name}')

        elapsed = now - self.state_start_time

        # ==================== LISTEN ====================
        if self.state == Task1State.LISTEN:
            # TODO: Call voice_command module to get target object
            # For now, can be hardcoded for testing:
            # self.target_object = "apple"
            # self.start_pose = self.latest_odom
            # self.transition_to(Task1State.SEARCH)
            pass

        # ==================== SEARCH ====================
        elif self.state == Task1State.SEARCH:
            # TODO: Use NavigateToObject class or scan behavior
            # Rotate in place, check /detections for target_object
            # If found, transition to APPROACH
            pass

        # ==================== APPROACH ====================
        elif self.state == Task1State.APPROACH:
            # TODO: Drive toward object using /object_poses
            # Stop when within grasping range
            # Transition to PLAN_GRASP when positioned
            pass

        # ==================== PLAN_GRASP ====================
        elif self.state == Task1State.PLAN_GRASP:
            # TODO: Call /plan_grasp service with object position
            # Store result in self.grasp_plan
            # Transition to PRE_GRASP on success
            pass

        # ==================== PRE_GRASP ====================
        elif self.state == Task1State.PRE_GRASP:
            # TODO: Call /plan_to_target with grasp_plan.pre_grasp_pose
            # Open gripper
            # Transition to GRASP when arm is in position
            pass

        # ==================== GRASP ====================
        elif self.state == Task1State.GRASP:
            # TODO: Call /plan_to_target with grasp_plan.grasp_pose
            # Close gripper
            # Transition to VERIFY_GRASP
            pass

        # ==================== VERIFY_GRASP ====================
        elif self.state == Task1State.VERIFY_GRASP:
            # TODO: Check gripper joint state — if fingers stopped short of
            # fully closed, object is likely grasped
            # Transition to RETURN on success, ERROR on failure
            pass

        # ==================== RETURN ====================
        elif self.state == Task1State.RETURN:
            # TODO: Navigate back to self.start_pose
            # Transition to DONE when arrived
            pass

        # ==================== DONE ====================
        elif self.state == Task1State.DONE:
            if elapsed < 0.1:
                self.get_logger().info('')
                self.get_logger().info('=' * 50)
                self.get_logger().info('Task 1 complete!')
                self.get_logger().info('=' * 50)

        # ==================== ERROR ====================
        elif self.state == Task1State.ERROR:
            if elapsed < 0.1:
                # Stop everything
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().error('Task 1 failed — stopping robot')


def main(args=None):
    rclpy.init(args=args)
    node = Task1Retrieve()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop base on exit
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
