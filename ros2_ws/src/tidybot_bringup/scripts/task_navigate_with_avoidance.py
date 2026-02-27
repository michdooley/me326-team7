#!/usr/bin/env python3
"""
Navigation task with obstacle-aware behaviors.

Usage:
    ros2 run tidybot_bringup task_navigate_with_avoidance.py
    ros2 run tidybot_bringup task_navigate_with_avoidance.py --ros-args -p navigation_mode:=rviz_goal
    ros2 run tidybot_bringup task_navigate_with_avoidance.py --ros-args -p navigation_mode:=reactive_forward

Modes:
    - seek_target (default): search for and approach target class from /objbbox.
    - rviz_goal: navigate to a clicked 2D Nav Goal in RViz.
    - reactive_forward: drive forward and turn when blocked.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from vision_msgs.msg import Detection2DArray
from enum import Enum
import numpy as np
from scipy.spatial.transform import Rotation

try:
    from cv_bridge import CvBridge
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


class State(Enum):
    INIT = 0
    NAVIGATE_TO_TARGET = 1
    ARRIVED = 2
    DONE = 3


class NavigationTaskNode(Node):
    """Click goals in RViz to navigate TidyBot around."""

    # Arrival tolerances
    POSITION_TOLERANCE = 0.1  #meters
    ANGLE_TOLERANCE = 0.15  #rads

    # Hold time at goal before accepting new goal pose
    ARRIVAL_HOLD_TIME = 1.0
    DEPTH_MIN_MM = 150
    DEPTH_MAX_MM = 1800
    MIN_COMPONENT_PIXELS = 400
    OBSTACLE_IGNORE_BOTTOM_FRAC = 0.10
    OBSTACLE_MAX_WIDTH_FRAC = 0.90
    DETECTION_LOG_INTERVAL = 1.0
    STOP_DISTANCE_M = 0.55
    CRITICAL_STOP_DISTANCE_M = 0.35
    # Front-only blocking region: narrower center lane and lower image area.
    FORWARD_REGION_X_MIN_FRAC = 0.35
    FORWARD_REGION_X_MAX_FRAC = 0.65
    FORWARD_REGION_Y_MIN_FRAC = 0.40
    MIN_BLOCKING_AREA_FRAC = 0.01
    STOP_LOG_INTERVAL = 1.0
    DEFAULT_NAVIGATION_MODE = 'seek_target'
    REACTIVE_FORWARD_STEP_M = 0.35
    REACTIVE_TURN_STEP_RAD = 0.30
    SEEK_TARGET_STOP_DISTANCE_M = 0.2
    SEEK_HEADING_GAIN = 0.45
    SEEK_FORWARD_STEP_M = 0.25
    APPROACH_MAX_BEARING_RAD = 0.30
    APPROACH_CENTER_TOLERANCE = 0.12
    APPROACH_CENTER_HYSTERESIS = 0.06
    APPROACH_TURN_SIGN = -1.0
    APPROACH_AVOID_STEP_M = 0.30
    APPROACH_AVOID_MIN_TURN_RAD = 0.45
    APPROACH_OBSTACLE_TARGET_MARGIN_M = 0.20
    APPROACH_AVOID_TURN_STEP_RAD = 0.22
    APPROACH_AVOID_MAX_DURATION_S = 4.0
    APPROACH_CLEAR_FRAMES = 4
    APPROACH_AVOID_TURNS_BEFORE_NUDGE = 1
    APPROACH_AVOID_NUDGE_STEP_M = 0.28
    APPROACH_BLOCK_MAX_DISTANCE_M = 0.55
    APPROACH_BLOCK_LOG_INTERVAL = 1.0
    APPROACH_BLOCK_CORRIDOR_X_MIN_FRAC = 0.25
    APPROACH_BLOCK_CORRIDOR_X_MAX_FRAC = 0.75
    APPROACH_BLOCK_CORRIDOR_Y_MIN_FRAC = 0.30
    APPROACH_BLOCK_CORRIDOR_PERCENTILE = 20.0
    APPROACH_BLOCK_MIN_VALID_PIXELS = 120
    RED_MIN_PIXELS = 120
    TARGET_CLASS_ID = 46
    YOLO_DETECTION_MAX_AGE_S = 1.5
    SEEK_SEARCH_SCAN_TOTAL_RAD = 2.0 * np.pi
    SEEK_SEARCH_SCAN_STEP_RAD = 0.10
    SEEK_SEARCH_SCAN_EXPLORED_RAD = 1.8
    SEEK_SEARCH_MOVE_DURATION_S = 2.5
    SEEK_SEARCH_MOVE_DISTANCE_M = 0.9
    SEEK_SEARCH_OPEN_HEADING_MAX_OFFSET_RAD = 1.2
    SEEK_SEARCH_VISIT_PENALTY = 0.25
    APPROACH_DETECT_FRAMES = 3
    APPROACH_LOST_FRAMES = 12
    APPROACH_TARGET_MEMORY_S = 1.5
    APPROACH_REACHED_VISIBLE_FRAMES = 2
    SIMPLE_YOLO_APPROACH_ENABLE = True
    SIMPLE_CENTER_TURN_STEP_RAD = 0.08
    SIMPLE_FORWARD_STEP_M = 0.10
    SIMPLE_MIN_FORWARD_STEP_M = 0.03
    APPROACH_TILT_DOWN_SIGN = 1.0
    SIMPLE_TILT_DOWN_STEP_RAD = 0.06
    SIMPLE_TILT_DOWN_LIMIT_RAD = 1.00
    SIMPLE_TILT_BASE_SCALE_ROTATE = 0.45
    SIMPLE_TILT_DEADBAND_FRAC = 0.01
    SIMPLE_TILT_UP_SCALE = 0.60
    APPROACH_STOP_DISTANCE_BUFFER_M = 0.08
    SIMPLE_REACH_BBOX_AREA_FRAC = 0.060
    AUTO_TILT_ENABLE = True
    AUTO_TILT_TARGET_Y_FRAC = 0.76
    AUTO_TILT_DEADBAND_FRAC = 0.03
    AUTO_TILT_GAIN = 0.20
    AUTO_TILT_UP_SCALE = 0.35
    AUTO_TILT_STEP_RAD = 0.08
    AUTO_TILT_MIN_RAD = -1.20
    AUTO_TILT_MAX_RAD = 0.30
    AUTO_TILT_NEAR_DIST_M = 1.20
    AUTO_TILT_NEAR_EXTRA_DOWN = 0.16
    EXPLORE_MEMORY_CELL_SIZE_M = 0.5
    EXPLORE_CANDIDATE_COUNT = 12
    EXPLORE_UNEXPLORED_VISITS_THRESHOLD = 1

    def __init__(self):
        super().__init__('navigation_task_node')

        # Navigation mode:
        # - rviz_goal: click a 2D goal in RViz and navigate to it.
        # - reactive_forward: keep driving forward; if blocked, rotate until clear.
        # - seek_target: navigate to detected target using YOLO bbox + depth.
        self.navigation_mode = self.declare_parameter(
            'navigation_mode', self.DEFAULT_NAVIGATION_MODE
        ).value
        if self.navigation_mode == 'seek_red_cylinder':
            # Backward-compatible alias.
            self.navigation_mode = 'seek_target'
        if self.navigation_mode not in ('rviz_goal', 'reactive_forward', 'seek_target'):
            self.get_logger().warn(
                f'Unknown navigation_mode="{self.navigation_mode}", using "{self.DEFAULT_NAVIGATION_MODE}".'
            )
            self.navigation_mode = self.DEFAULT_NAVIGATION_MODE

        self.reactive_forward_step_m = float(
            self.declare_parameter('reactive_forward_step_m', self.REACTIVE_FORWARD_STEP_M).value
        )
        self.reactive_turn_step_rad = float(
            self.declare_parameter('reactive_turn_step_rad', self.REACTIVE_TURN_STEP_RAD).value
        )
        self.seek_target_stop_distance_m = float(
            self.declare_parameter('seek_target_stop_distance_m', self.SEEK_TARGET_STOP_DISTANCE_M).value
        )
        self.seek_heading_gain = float(
            self.declare_parameter('seek_heading_gain', self.SEEK_HEADING_GAIN).value
        )
        self.seek_forward_step_m = float(
            self.declare_parameter('seek_forward_step_m', self.SEEK_FORWARD_STEP_M).value
        )
        self.approach_max_bearing_rad = float(
            self.declare_parameter(
                'approach_max_bearing_rad',
                self.APPROACH_MAX_BEARING_RAD
            ).value
        )
        self.approach_center_tolerance = float(
            self.declare_parameter(
                'approach_center_tolerance',
                self.APPROACH_CENTER_TOLERANCE
            ).value
        )
        self.approach_center_hysteresis = float(
            self.declare_parameter(
                'approach_center_hysteresis',
                self.APPROACH_CENTER_HYSTERESIS
            ).value
        )
        self.approach_turn_sign = float(
            self.declare_parameter(
                'approach_turn_sign',
                self.APPROACH_TURN_SIGN
            ).value
        )
        self.approach_avoid_step_m = float(
            self.declare_parameter(
                'approach_avoid_step_m',
                self.APPROACH_AVOID_STEP_M
            ).value
        )
        self.approach_avoid_min_turn_rad = float(
            self.declare_parameter(
                'approach_avoid_min_turn_rad',
                self.APPROACH_AVOID_MIN_TURN_RAD
            ).value
        )
        self.approach_obstacle_target_margin_m = float(
            self.declare_parameter(
                'approach_obstacle_target_margin_m',
                self.APPROACH_OBSTACLE_TARGET_MARGIN_M
            ).value
        )
        self.approach_avoid_turn_step_rad = float(
            self.declare_parameter(
                'approach_avoid_turn_step_rad',
                self.APPROACH_AVOID_TURN_STEP_RAD
            ).value
        )
        self.approach_avoid_max_duration_s = float(
            self.declare_parameter(
                'approach_avoid_max_duration_s',
                self.APPROACH_AVOID_MAX_DURATION_S
            ).value
        )
        self.approach_clear_frames = int(
            self.declare_parameter(
                'approach_clear_frames',
                self.APPROACH_CLEAR_FRAMES
            ).value
        )
        self.approach_avoid_turns_before_nudge = int(
            self.declare_parameter(
                'approach_avoid_turns_before_nudge',
                self.APPROACH_AVOID_TURNS_BEFORE_NUDGE
            ).value
        )
        self.approach_avoid_nudge_step_m = float(
            self.declare_parameter(
                'approach_avoid_nudge_step_m',
                self.APPROACH_AVOID_NUDGE_STEP_M
            ).value
        )
        self.approach_block_max_distance_m = float(
            self.declare_parameter(
                'approach_block_max_distance_m',
                self.APPROACH_BLOCK_MAX_DISTANCE_M
            ).value
        )
        self.approach_block_corridor_x_min_frac = float(
            self.declare_parameter(
                'approach_block_corridor_x_min_frac',
                self.APPROACH_BLOCK_CORRIDOR_X_MIN_FRAC
            ).value
        )
        self.approach_block_corridor_x_max_frac = float(
            self.declare_parameter(
                'approach_block_corridor_x_max_frac',
                self.APPROACH_BLOCK_CORRIDOR_X_MAX_FRAC
            ).value
        )
        self.approach_block_corridor_y_min_frac = float(
            self.declare_parameter(
                'approach_block_corridor_y_min_frac',
                self.APPROACH_BLOCK_CORRIDOR_Y_MIN_FRAC
            ).value
        )
        self.approach_block_corridor_percentile = float(
            self.declare_parameter(
                'approach_block_corridor_percentile',
                self.APPROACH_BLOCK_CORRIDOR_PERCENTILE
            ).value
        )
        self.approach_block_min_valid_pixels = int(
            self.declare_parameter(
                'approach_block_min_valid_pixels',
                self.APPROACH_BLOCK_MIN_VALID_PIXELS
            ).value
        )
        self.seek_search_scan_total_rad = float(
            self.declare_parameter(
                'seek_search_scan_total_rad',
                self.SEEK_SEARCH_SCAN_TOTAL_RAD
            ).value
        )
        self.seek_search_scan_step_rad = float(
            self.declare_parameter(
                'seek_search_scan_step_rad',
                self.SEEK_SEARCH_SCAN_STEP_RAD
            ).value
        )
        self.seek_search_move_duration_s = float(
            self.declare_parameter(
                'seek_search_move_duration_s',
                self.SEEK_SEARCH_MOVE_DURATION_S
            ).value
        )
        self.seek_search_move_distance_m = float(
            self.declare_parameter(
                'seek_search_move_distance_m',
                self.SEEK_SEARCH_MOVE_DISTANCE_M
            ).value
        )
        self.seek_search_open_heading_max_offset_rad = float(
            self.declare_parameter(
                'seek_search_open_heading_max_offset_rad',
                self.SEEK_SEARCH_OPEN_HEADING_MAX_OFFSET_RAD
            ).value
        )
        self.seek_search_visit_penalty = float(
            self.declare_parameter(
                'seek_search_visit_penalty',
                self.SEEK_SEARCH_VISIT_PENALTY
            ).value
        )
        self.approach_detect_frames = int(
            self.declare_parameter(
                'approach_detect_frames',
                self.APPROACH_DETECT_FRAMES
            ).value
        )
        self.approach_lost_frames = int(
            self.declare_parameter(
                'approach_lost_frames',
                self.APPROACH_LOST_FRAMES
            ).value
        )
        self.approach_target_memory_s = float(
            self.declare_parameter(
                'approach_target_memory_s',
                self.APPROACH_TARGET_MEMORY_S
            ).value
        )
        self.approach_reached_visible_frames = int(
            self.declare_parameter(
                'approach_reached_visible_frames',
                self.APPROACH_REACHED_VISIBLE_FRAMES
            ).value
        )
        self.simple_yolo_approach_enable = bool(
            self.declare_parameter(
                'simple_yolo_approach_enable',
                self.SIMPLE_YOLO_APPROACH_ENABLE
            ).value
        )
        self.simple_center_turn_step_rad = float(
            self.declare_parameter(
                'simple_center_turn_step_rad',
                self.SIMPLE_CENTER_TURN_STEP_RAD
            ).value
        )
        self.simple_forward_step_m = float(
            self.declare_parameter(
                'simple_forward_step_m',
                self.SIMPLE_FORWARD_STEP_M
            ).value
        )
        self.simple_min_forward_step_m = float(
            self.declare_parameter(
                'simple_min_forward_step_m',
                self.SIMPLE_MIN_FORWARD_STEP_M
            ).value
        )
        self.approach_tilt_down_sign = float(
            self.declare_parameter(
                'approach_tilt_down_sign',
                self.APPROACH_TILT_DOWN_SIGN
            ).value
        )
        self.approach_stop_distance_buffer_m = float(
            self.declare_parameter(
                'approach_stop_distance_buffer_m',
                self.APPROACH_STOP_DISTANCE_BUFFER_M
            ).value
        )
        self.simple_reach_bbox_area_frac = float(
            self.declare_parameter(
                'simple_reach_bbox_area_frac',
                self.SIMPLE_REACH_BBOX_AREA_FRAC
            ).value
        )
        self.simple_tilt_down_step_rad = float(
            self.declare_parameter(
                'simple_tilt_down_step_rad',
                self.SIMPLE_TILT_DOWN_STEP_RAD
            ).value
        )
        self.simple_tilt_down_limit_rad = float(
            self.declare_parameter(
                'simple_tilt_down_limit_rad',
                self.SIMPLE_TILT_DOWN_LIMIT_RAD
            ).value
        )
        self.simple_tilt_base_scale_rotate = float(
            self.declare_parameter(
                'simple_tilt_base_scale_rotate',
                self.SIMPLE_TILT_BASE_SCALE_ROTATE
            ).value
        )
        self.simple_tilt_deadband_frac = float(
            self.declare_parameter(
                'simple_tilt_deadband_frac',
                self.SIMPLE_TILT_DEADBAND_FRAC
            ).value
        )
        self.simple_tilt_up_scale = float(
            self.declare_parameter(
                'simple_tilt_up_scale',
                self.SIMPLE_TILT_UP_SCALE
            ).value
        )
        self.use_yolo_target_detection = bool(
            self.declare_parameter('use_yolo_target_detection', True).value
        )
        self.target_class_id = int(
            self.declare_parameter('target_class_id', self.TARGET_CLASS_ID).value
        )
        self.yolo_detection_max_age_s = float(
            self.declare_parameter('yolo_detection_max_age_s', self.YOLO_DETECTION_MAX_AGE_S).value
        )
        self.auto_tilt_enable = bool(
            self.declare_parameter('auto_tilt_enable', self.AUTO_TILT_ENABLE).value
        )
        self.auto_tilt_target_y_frac = float(
            self.declare_parameter('auto_tilt_target_y_frac', self.AUTO_TILT_TARGET_Y_FRAC).value
        )
        self.auto_tilt_deadband_frac = float(
            self.declare_parameter('auto_tilt_deadband_frac', self.AUTO_TILT_DEADBAND_FRAC).value
        )
        self.auto_tilt_gain = float(
            self.declare_parameter('auto_tilt_gain', self.AUTO_TILT_GAIN).value
        )
        self.auto_tilt_up_scale = float(
            self.declare_parameter('auto_tilt_up_scale', self.AUTO_TILT_UP_SCALE).value
        )
        self.auto_tilt_step_rad = float(
            self.declare_parameter('auto_tilt_step_rad', self.AUTO_TILT_STEP_RAD).value
        )
        self.auto_tilt_min_rad = float(
            self.declare_parameter('auto_tilt_min_rad', self.AUTO_TILT_MIN_RAD).value
        )
        self.auto_tilt_max_rad = float(
            self.declare_parameter('auto_tilt_max_rad', self.AUTO_TILT_MAX_RAD).value
        )
        self.auto_tilt_near_dist_m = float(
            self.declare_parameter('auto_tilt_near_dist_m', self.AUTO_TILT_NEAR_DIST_M).value
        )
        self.auto_tilt_near_extra_down = float(
            self.declare_parameter('auto_tilt_near_extra_down', self.AUTO_TILT_NEAR_EXTRA_DOWN).value
        )
        self.explore_memory_cell_size_m = float(
            self.declare_parameter(
                'explore_memory_cell_size_m',
                self.EXPLORE_MEMORY_CELL_SIZE_M
            ).value
        )
        self.explore_candidate_count = int(
            self.declare_parameter(
                'explore_candidate_count',
                self.EXPLORE_CANDIDATE_COUNT
            ).value
        )
        self.explore_unexplored_visits_threshold = int(
            self.declare_parameter(
                'explore_unexplored_visits_threshold',
                self.EXPLORE_UNEXPLORED_VISITS_THRESHOLD
            ).value
        )

        # Publishing movement info
        self.base_pose_pub = self.create_publisher(Pose2D, '/base/target_pose', 10)
        self.pan_tilt_pub = self.create_publisher(Float64MultiArray, '/camera/pan_tilt_cmd', 10)

        # Subscriber
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.yolo_sub = self.create_subscription(
            Detection2DArray, '/objbbox', self.yolo_detections_callback, 10
        )

        # Camera-based obstacle detection state
        self.obstacle_bbox_depth = None  # (x, y, w, h) in depth image coordinates
        self.obstacle_distance_m = None
        self.latest_depth_shape = None
        self.last_detection_log_time = self.get_clock().now()
        self.last_stop_log_time = self.get_clock().now()
        self.last_approach_block_log_time = self.get_clock().now()
        self.was_blocked = False
        self.red_target_visible = False
        self.red_bbox_color = None  # (x, y, w, h) in color image
        self.red_target_distance_m = None
        self.yolo_bbox_color = None  # latest target-class bbox in color image coords
        self.yolo_bbox_time = None
        self.last_red_seen_time = None
        self.last_red_bbox_color = None
        self.last_red_distance_m = None
        self.last_target_heading = None
        self.last_target_heading_time = None
        self.last_target_distance_m = None
        self.latest_depth_mm = None
        self.latest_depth_valid = None
        self.latest_color_shape = None
        self.red_target_reached_logged = False
        self.seek_mode = 'search_scan'  # search_scan, search_move, approach
        self.scan_accumulated_yaw = 0.0
        self.scan_last_theta = 0.0
        self.search_move_start_time = self.get_clock().now()
        self.search_move_start_x = 0.0
        self.search_move_start_y = 0.0
        self.search_move_target_x = 0.0
        self.search_move_target_y = 0.0
        self.search_move_target_theta = 0.0
        self.red_detect_streak = 0
        self.red_lost_streak = 0
        self.approach_avoid_active = False
        self.approach_avoid_side = 1.0  # +1 left, -1 right
        self.approach_avoid_start_time = self.get_clock().now()
        self.approach_clear_streak = 0
        self.approach_avoid_cmd_count = 0
        self.approach_centered_locked = False
        self.approach_reached_visible_streak = 0
        self.visit_counts = {}
        self.last_visited_cell = None
        self.scanned_cells = set()
        self.camera_pan_cmd = 0.0
        self.camera_tilt_cmd = 0.0

        if CV_AVAILABLE:
            camera_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.cv_bridge = CvBridge()
            self.color_sub = self.create_subscription(
                Image, '/camera/color/image_raw', self.color_callback, camera_qos
            )
            self.depth_sub = self.create_subscription(
                Image, '/camera/depth/image_raw', self.depth_callback, camera_qos
            )
            self.overlay_pub = self.create_publisher(Image, '/camera/obstacle_overlay', 10)
        else:
            self.get_logger().warn(
                'cv_bridge/opencv not available, camera obstacle overlay is disabled.'
            )

        # State tracking
        self.state = State.INIT
        self.state_start_time = self.get_clock().now()

        # Current pose (from odometry)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_received = False

        # Target pose from RViz
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0
        self.has_goal = False

        # Main control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Ready! Click "2D Nav Goal" in RViz (or press G) to navigate.')
        self.get_logger().info(f'Navigation mode: {self.navigation_mode}')
        if self.navigation_mode == 'reactive_forward':
            self.get_logger().info(
                'Reactive mode enabled: driving forward, rotating when obstacle blocks path.'
            )
        elif self.navigation_mode == 'seek_target':
            self.get_logger().info(
                'Seek-target mode enabled: SEARCH_SCAN -> SEARCH_MOVE -> APPROACH.'
            )
            if not CV_AVAILABLE:
                self.get_logger().warn(
                    'seek_target requires cv_bridge/opencv image processing for detection.'
                )
            if self.use_yolo_target_detection:
                self.get_logger().info(
                    f'Using YOLO detections from /objbbox (target_class_id={self.target_class_id}).'
                )
        if CV_AVAILABLE:
            self.get_logger().info(
                'Obstacle overlay publisher active on /camera/obstacle_overlay'
            )

    def odom_callback(self, msg: Odometry):
        """Track where the robot currently is."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract heading from quaternion
        quat = msg.pose.pose.orientation
        rotation = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        euler = rotation.as_euler('xyz', degrees=False)
        mujoco_theta = euler[2]

        # Convert from MuJoCo frame (theta=π/2 is +X) to user frame (theta=0 is +X)
        self.current_theta = mujoco_theta - np.pi / 2

        self.odom_received = True
        self._record_visit(self.current_x, self.current_y)

    def goal_callback(self, msg: PoseStamped):
        """Handle RViz clicks - extract target position and heading."""
        if self.navigation_mode != 'rviz_goal':
            self.get_logger().info(
                f'Ignoring RViz goal_pose because navigation_mode is {self.navigation_mode}.'
            )
            return

        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y

        # Extract heading from the arrow direction
        quat = msg.pose.orientation
        rotation = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        euler = rotation.as_euler('xyz', degrees=False)
        self.target_theta = euler[2]

        self.has_goal = True

        # If we're sitting idle, start navigating immediately
        if self.state in [State.ARRIVED, State.DONE]:
            self.state = State.NAVIGATE_TO_TARGET
            self.state_start_time = self.get_clock().now()

        self.get_logger().info(
            f'New goal: ({self.target_x:.2f}, {self.target_y:.2f}) '
            f'facing {np.rad2deg(self.target_theta):.0f}°'
        )

    def yolo_detections_callback(self, msg: Detection2DArray):
        """Track latest YOLO bbox for configured target class in color-image coordinates."""
        if not self.use_yolo_target_detection:
            return

        best_bbox = None
        best_score = -1.0
        for detection in msg.detections:
            if not detection.results:
                continue
            class_raw = detection.results[0].hypothesis.class_id
            try:
                class_id = int(class_raw)
            except (TypeError, ValueError):
                continue
            if class_id != self.target_class_id:
                continue

            score = float(detection.results[0].hypothesis.score)
            cx = float(detection.bbox.center.position.x)
            cy = float(detection.bbox.center.position.y)
            w = float(detection.bbox.size_x)
            h = float(detection.bbox.size_y)
            x = int(round(cx - 0.5 * w))
            y = int(round(cy - 0.5 * h))
            bbox = (x, y, int(max(1, round(w))), int(max(1, round(h))))
            if score > best_score:
                best_score = score
                best_bbox = bbox

        if best_bbox is not None:
            self.yolo_bbox_color = best_bbox
            self.yolo_bbox_time = self.get_clock().now()

    def _get_yolo_target_bbox(self):
        """Return recent target bbox from YOLO, clipped to latest color image shape."""
        if not self.use_yolo_target_detection:
            return None
        if self.yolo_bbox_color is None or self.yolo_bbox_time is None:
            return None
        if self.latest_color_shape is None:
            return None

        age_s = (self.get_clock().now() - self.yolo_bbox_time).nanoseconds / 1e9
        if age_s > self.yolo_detection_max_age_s:
            return None

        x, y, w, h = self.yolo_bbox_color
        color_h, color_w = self.latest_color_shape
        x = max(0, min(x, color_w - 1))
        y = max(0, min(y, color_h - 1))
        w = max(1, min(w, color_w - x))
        h = max(1, min(h, color_h - y))
        return x, y, w, h

    def get_distance_to_target(self) -> float:
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        return np.sqrt(dx**2 + dy**2)

    def get_angle_error(self) -> float:
        error = self.target_theta - self.current_theta
        # Wrap to [-π, π]
        while error > np.pi:
            error -= 2 * np.pi
        while error < -np.pi:
            error += 2 * np.pi
        return error

    def is_at_target(self) -> bool:
        distance = self.get_distance_to_target()
        angle_error = abs(self.get_angle_error())
        return distance < self.POSITION_TOLERANCE and angle_error < self.ANGLE_TOLERANCE

    def depth_callback(self, msg: Image):
        """Detect the nearest obstacle region from depth image."""
        try:
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')
            return

        if depth_image is None:
            return

        if depth_image.ndim > 2:
            depth_image = depth_image[:, :, 0]

        self.latest_depth_shape = depth_image.shape[:2]
        depth_mm = self._to_mm_depth(depth_image)
        self.latest_depth_mm = depth_mm
        self.latest_depth_valid = np.isfinite(depth_mm) & (depth_mm > self.DEPTH_MIN_MM) & (depth_mm < self.DEPTH_MAX_MM)
        self.obstacle_bbox_depth, self.obstacle_distance_m = self._detect_obstacle(depth_mm)

        if self.obstacle_bbox_depth is not None and self.obstacle_distance_m is not None:
            now = self.get_clock().now()
            dt = (now - self.last_detection_log_time).nanoseconds / 1e9
            if dt >= self.DETECTION_LOG_INTERVAL:
                x, y, w, h = self.obstacle_bbox_depth
                self.get_logger().info(
                    f'Obstacle at {self.obstacle_distance_m:.2f}m '
                    f'(depth bbox x={x}, y={y}, w={w}, h={h})'
                )
                self.last_detection_log_time = now

    def color_callback(self, msg: Image):
        """Draw obstacle bounding box onto color frame and publish overlay."""
        if self.latest_depth_shape is None:
            return

        try:
            color_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Color conversion failed: {e}')
            return

        if color_image is None:
            return

        self.latest_color_shape = color_image.shape[:2]
        overlay = color_image.copy()
        if self.use_yolo_target_detection:
            target_bbox = self._get_yolo_target_bbox()
        else:
            target_bbox = self._detect_red_target_bbox(color_image)
        self.red_target_visible = target_bbox is not None
        self.red_bbox_color = target_bbox
        self.red_target_distance_m = self._estimate_red_target_distance()

        if self.red_target_visible:
            self.last_red_seen_time = self.get_clock().now()
            self.last_red_bbox_color = self.red_bbox_color
            self.last_red_distance_m = self.red_target_distance_m
            if self.auto_tilt_enable and not (
                self.use_yolo_target_detection and self.simple_yolo_approach_enable
            ):
                self._update_camera_tilt_for_target(self.red_bbox_color, self.red_target_distance_m)
            rx, ry, rw, rh = self.red_bbox_color
            cv2.rectangle(overlay, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 2)
            if self.red_target_distance_m is not None:
                target_label = (
                    f'Class {self.target_class_id}' if self.use_yolo_target_detection else 'Red target'
                )
                cv2.putText(
                    overlay,
                    f'{target_label}: {self.red_target_distance_m:.2f} m',
                    (rx, max(20, ry - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )
        elif (
            self.auto_tilt_enable
            and not (self.use_yolo_target_detection and self.simple_yolo_approach_enable)
            and self.last_red_seen_time is not None
            and self.last_red_bbox_color is not None
        ):
            # Keep mild tilt recentering during brief detector dropouts.
            dt = (self.get_clock().now() - self.last_red_seen_time).nanoseconds / 1e9
            if dt <= self.approach_target_memory_s:
                self._update_camera_tilt_for_target(self.last_red_bbox_color, self.last_red_distance_m)

        if self.obstacle_bbox_depth is not None and self.obstacle_distance_m is not None:
            bbox_color = self._map_bbox_to_color(
                self.obstacle_bbox_depth,
                self.latest_depth_shape,
                overlay.shape[:2],
            )
            x, y, w, h = bbox_color
            is_blocking = self.is_obstacle_blocking()
            box_color = (0, 0, 255) if is_blocking else (0, 255, 255)
            cv2.rectangle(overlay, (x, y), (x + w, y + h), box_color, 2)
            status = 'STOP' if is_blocking else 'Obstacle'
            label = f'{status}: {self.obstacle_distance_m:.2f} m'
            cv2.putText(
                overlay,
                label,
                (x, max(20, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                box_color,
                2,
            )

        overlay_msg = self.cv_bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        overlay_msg.header = msg.header
        self.overlay_pub.publish(overlay_msg)

    def _update_camera_tilt_for_target(self, bbox, distance_m):
        """Tilt camera to keep low/near target in view while approaching."""
        if bbox is None or self.latest_color_shape is None:
            return
        _, y, _, h = bbox
        image_h = float(max(self.latest_color_shape[0], 1))
        center_y_frac = (y + 0.5 * h) / image_h

        desired_y_frac = self.auto_tilt_target_y_frac
        if distance_m is not None and distance_m < self.auto_tilt_near_dist_m:
            desired_y_frac = min(0.90, desired_y_frac + self.auto_tilt_near_extra_down)

        error = desired_y_frac - center_y_frac
        if abs(error) < self.auto_tilt_deadband_frac:
            return

        # Asymmetric proportional correction:
        # - tilt down quickly when target is too high in image,
        # - recover up more slowly to avoid oscillation/hunting.
        #
        # NOTE: In this setup, increasing camera_tilt_cmd tilts camera downward.
        if error > 0.0:
            down_delta = float(
                np.clip(self.auto_tilt_gain * error, 0.0, self.auto_tilt_step_rad)
            )
            self.camera_tilt_cmd += down_delta
        else:
            up_scale = max(0.0, min(self.auto_tilt_up_scale, 1.0))
            up_delta = float(
                np.clip(
                    self.auto_tilt_gain * (-error) * up_scale,
                    0.0,
                    self.auto_tilt_step_rad * up_scale,
                )
            )
            self.camera_tilt_cmd -= up_delta

        self.camera_tilt_cmd = float(
            np.clip(self.camera_tilt_cmd, self.auto_tilt_min_rad, self.auto_tilt_max_rad)
        )
        msg = Float64MultiArray()
        msg.data = [self.camera_pan_cmd, self.camera_tilt_cmd]
        self.pan_tilt_pub.publish(msg)

    def _to_mm_depth(self, depth_image: np.ndarray) -> np.ndarray:
        """Normalize depth image to millimeters."""
        if np.issubdtype(depth_image.dtype, np.floating):
            depth_mm = depth_image.astype(np.float32)
            if np.nanmax(depth_mm) < 20.0:
                depth_mm *= 1000.0
        else:
            depth_mm = depth_image.astype(np.float32)
        return depth_mm

    def _detect_obstacle(self, depth_mm: np.ndarray):
        """Return (bbox, distance_m) for nearest valid depth cluster."""
        valid = np.isfinite(depth_mm) & (depth_mm > self.DEPTH_MIN_MM) & (depth_mm < self.DEPTH_MAX_MM)
        depth_h, depth_w = depth_mm.shape[:2]
        # Ignore a bottom band to avoid floor dominating as a giant obstacle blob.
        ignore_rows = int(depth_h * self.OBSTACLE_IGNORE_BOTTOM_FRAC)
        if ignore_rows > 0:
            valid[depth_h - ignore_rows:, :] = False
        if not np.any(valid):
            return None, None

        mask = (valid.astype(np.uint8) * 255)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None

        best_bbox = None
        best_depth_mm = None

        for contour in contours:
            if cv2.contourArea(contour) < self.MIN_COMPONENT_PIXELS:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            # Reject floor-like giant horizontal blobs that span most of the image.
            if w > int(depth_w * self.OBSTACLE_MAX_WIDTH_FRAC):
                continue
            roi_depth = depth_mm[y:y + h, x:x + w]
            roi_valid = valid[y:y + h, x:x + w]
            if not np.any(roi_valid):
                continue

            median_depth = float(np.median(roi_depth[roi_valid]))
            if best_depth_mm is None or median_depth < best_depth_mm:
                best_depth_mm = median_depth
                best_bbox = (x, y, w, h)

        if best_bbox is None:
            return None, None

        return best_bbox, best_depth_mm / 1000.0

    def _map_bbox_to_color(self, bbox_depth, depth_shape, color_shape):
        """Scale depth-image bbox to color-image coordinates."""
        dx, dy, dw, dh = bbox_depth
        depth_h, depth_w = depth_shape
        color_h, color_w = color_shape

        scale_x = color_w / float(depth_w)
        scale_y = color_h / float(depth_h)

        x = int(dx * scale_x)
        y = int(dy * scale_y)
        w = int(dw * scale_x)
        h = int(dh * scale_y)

        x = max(0, min(x, color_w - 1))
        y = max(0, min(y, color_h - 1))
        w = max(1, min(w, color_w - x))
        h = max(1, min(h, color_h - y))
        return x, y, w, h

    def _map_bbox_color_to_depth(self, bbox_color, color_shape, depth_shape):
        """Scale color-image bbox to depth-image coordinates."""
        cx, cy, cw, ch = bbox_color
        color_h, color_w = color_shape
        depth_h, depth_w = depth_shape

        scale_x = depth_w / float(color_w)
        scale_y = depth_h / float(color_h)

        x = int(cx * scale_x)
        y = int(cy * scale_y)
        w = int(cw * scale_x)
        h = int(ch * scale_y)

        x = max(0, min(x, depth_w - 1))
        y = max(0, min(y, depth_h - 1))
        w = max(1, min(w, depth_w - x))
        h = max(1, min(h, depth_h - y))
        return x, y, w, h

    def _detect_red_target_bbox(self, bgr_image):
        """Detect largest red blob in color image; return bbox or None."""
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        lower1 = np.array([0, 70, 40], dtype=np.uint8)
        upper1 = np.array([10, 255, 255], dtype=np.uint8)
        lower2 = np.array([165, 70, 40], dtype=np.uint8)
        upper2 = np.array([180, 255, 255], dtype=np.uint8)

        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.RED_MIN_PIXELS:
            return None
        return cv2.boundingRect(largest)

    def _estimate_red_target_distance(self):
        """Estimate red target distance from depth ROI under red bbox."""
        if self.red_bbox_color is None:
            return None
        if self.latest_depth_mm is None or self.latest_depth_valid is None:
            return None
        if self.latest_color_shape is None or self.latest_depth_shape is None:
            return None

        dx, dy, dw, dh = self._map_bbox_color_to_depth(
            self.red_bbox_color,
            self.latest_color_shape,
            self.latest_depth_shape,
        )
        # Use only the center region of the target bbox to reduce depth bleed from nearby clutter.
        cx1 = dx + int(0.25 * dw)
        cx2 = dx + int(0.75 * dw)
        cy1 = dy + int(0.25 * dh)
        cy2 = dy + int(0.75 * dh)
        cx1 = max(dx, min(cx1, dx + dw - 1))
        cy1 = max(dy, min(cy1, dy + dh - 1))
        cx2 = max(cx1 + 1, min(cx2, dx + dw))
        cy2 = max(cy1 + 1, min(cy2, dy + dh))

        roi_depth = self.latest_depth_mm[cy1:cy2, cx1:cx2]
        roi_valid = self.latest_depth_valid[cy1:cy2, cx1:cx2]
        if not np.any(roi_valid):
            return None
        # Slightly near-biased percentile helps prevent stopping too early from background pixels.
        return float(np.percentile(roi_depth[roi_valid], 40.0) / 1000.0)

    def is_obstacle_blocking(self) -> bool:
        """True when a near obstacle overlaps the robot's forward camera region."""
        if self.obstacle_bbox_depth is None or self.obstacle_distance_m is None:
            return False
        if self.latest_depth_shape is None:
            return False
        if self.obstacle_distance_m > self.STOP_DISTANCE_M:
            return False

        depth_h, depth_w = self.latest_depth_shape
        x, y, w, h = self.obstacle_bbox_depth
        x2 = x + w
        y2 = y + h

        lane_x1 = int(depth_w * self.FORWARD_REGION_X_MIN_FRAC)
        lane_x2 = int(depth_w * self.FORWARD_REGION_X_MAX_FRAC)
        lane_y1 = int(depth_h * self.FORWARD_REGION_Y_MIN_FRAC)

        overlaps_forward_region = (x2 > lane_x1) and (x < lane_x2) and (y2 > lane_y1)
        area_frac = (w * h) / float(depth_w * depth_h)
        is_large_enough = area_frac >= self.MIN_BLOCKING_AREA_FRAC
        is_critical = self.obstacle_distance_m <= self.CRITICAL_STOP_DISTANCE_M

        return overlaps_forward_region and (is_large_enough or is_critical)

    def publish_hold_position(self):
        """Command base to hold current robot pose."""
        target = Pose2D()
        target.x = self.current_x
        target.y = self.current_y
        target.theta = self.current_theta
        self.base_pose_pub.publish(target)

    def publish_pose_target(self, x: float, y: float, theta: float):
        target = Pose2D()
        target.x = x
        target.y = y
        target.theta = theta
        self.base_pose_pub.publish(target)

    def control_loop(self):
        current_time = self.get_clock().now()

        if self.state == State.INIT:
            self.handle_init(current_time)
        elif self.state == State.NAVIGATE_TO_TARGET:
            self.handle_navigate(current_time)
        elif self.state == State.ARRIVED:
            self.handle_arrived(current_time)
        elif self.state == State.DONE:
            self.handle_done()

    def handle_init(self, current_time):
        """Wait for odometry and first goal click."""
        if not self.odom_received:
            return
        if self.navigation_mode == 'rviz_goal' and not self.has_goal:
            return

        if self.navigation_mode == 'seek_target':
            self.seek_mode = 'search_scan'
            self.scan_accumulated_yaw = 0.0
            self.scan_last_theta = self.current_theta

        self.get_logger().info(f'Starting from ({self.current_x:.2f}, {self.current_y:.2f})')
        self.transition_to_state(State.NAVIGATE_TO_TARGET, current_time)

    def handle_navigate(self, current_time):
        """Publish target and check if we've arrived."""
        if self.navigation_mode == 'reactive_forward':
            self.handle_reactive_forward(current_time)
            return
        if self.navigation_mode == 'seek_target':
            self.handle_seek_target(current_time)
            return

        if self.is_at_target():
            self.get_logger().info('Arrived!')
            self.transition_to_state(State.ARRIVED, current_time)
            return

        if self.is_obstacle_blocking():
            self.publish_hold_position()
            now = self.get_clock().now()
            if not self.was_blocked:
                self.get_logger().warn(
                    f'SAFETY STOP: blocked by obstacle at {self.obstacle_distance_m:.2f}m'
                )
                self.last_stop_log_time = now
            else:
                dt = (now - self.last_stop_log_time).nanoseconds / 1e9
                if dt >= self.STOP_LOG_INTERVAL:
                    self.get_logger().warn(
                        f'Still stopped: obstacle at {self.obstacle_distance_m:.2f}m'
                    )
                    self.last_stop_log_time = now
            self.was_blocked = True
            return

        if self.was_blocked:
            self.get_logger().info('Path cleared. Resuming navigation to goal.')
            self.was_blocked = False

        # Send target to base controller
        self.publish_pose_target(self.target_x, self.target_y, self.target_theta)

        # Log progress every 2 seconds
        time_in_state = (current_time - self.state_start_time).nanoseconds / 1e9
        if int(time_in_state) % 2 == 0 and int(time_in_state * 10) % 20 == 0:
            distance = self.get_distance_to_target()
            self.get_logger().info(f'{distance:.2f}m away...')

    def handle_reactive_forward(self, current_time):
        """Drive forward; if blocked, rotate in place until obstacle clears."""
        _ = current_time
        if self.is_obstacle_blocking():
            now = self.get_clock().now()
            if not self.was_blocked:
                self.get_logger().warn(
                    f'SAFETY STOP: obstacle at {self.obstacle_distance_m:.2f}m, rotating to clear path'
                )
                self.last_stop_log_time = now
            else:
                dt = (now - self.last_stop_log_time).nanoseconds / 1e9
                if dt >= self.STOP_LOG_INTERVAL:
                    self.get_logger().warn(
                        f'Still blocked: obstacle at {self.obstacle_distance_m:.2f}m, rotating'
                    )
                    self.last_stop_log_time = now

            rotate_theta = self.current_theta + self.reactive_turn_step_rad
            self.publish_pose_target(self.current_x, self.current_y, rotate_theta)
            self.was_blocked = True
            return

        if self.was_blocked:
            self.get_logger().info('Path cleared. Continuing forward.')
            self.was_blocked = False

        forward_x = self.current_x + self.reactive_forward_step_m * np.cos(self.current_theta)
        forward_y = self.current_y + self.reactive_forward_step_m * np.sin(self.current_theta)
        self.publish_pose_target(forward_x, forward_y, self.current_theta)

    def handle_seek_target(self, current_time):
        """Primary objective: navigate toward YOLO target with SEARCH/APPROACH loop."""
        if self.red_target_reached_logged:
            self.publish_hold_position()
            return
        detect_frames_required = max(1, self.approach_detect_frames)
        lost_frames_required = max(1, self.approach_lost_frames)
        if self.use_yolo_target_detection:
            # YOLO detections can flicker; enter approach quickly and keep target lock longer.
            detect_frames_required = 1
            lost_frames_required = max(lost_frames_required, 20)

        if self.red_target_visible:
            self.red_detect_streak += 1
            self.red_lost_streak = 0
        else:
            self.red_detect_streak = 0
            self.red_lost_streak += 1

        if self.seek_mode != 'approach' and self.red_detect_streak >= detect_frames_required:
            self.seek_mode = 'approach'
            self.approach_avoid_active = False
            self.approach_centered_locked = False
            self.approach_reached_visible_streak = 0
            self.get_logger().info('Seek mode: APPROACH (target detected).')

        if self.seek_mode != 'approach' and self._recenter_from_last_target(current_time):
            return

        if self.seek_mode == 'approach' and self.red_lost_streak >= lost_frames_required:
            memory_active = False
            if self.last_red_seen_time is not None:
                dt = (current_time - self.last_red_seen_time).nanoseconds / 1e9
                memory_active = dt <= self.approach_target_memory_s
            if not memory_active:
                self.seek_mode = 'search_scan'
                self.approach_avoid_active = False
                self.approach_centered_locked = False
                self.approach_reached_visible_streak = 0
                self.scan_accumulated_yaw = 0.0
                self.scan_last_theta = self.current_theta
                self.get_logger().info('Seek mode: SEARCH_SCAN (lost target).')

        if self.seek_mode == 'approach':
            self._handle_approach_mode()
        elif self.seek_mode == 'search_scan':
            self._handle_search_scan_mode()
        else:
            self._handle_search_move_mode(current_time)

    def _wrapped_angle_diff(self, current: float, previous: float) -> float:
        """Smallest signed angle difference current - previous."""
        return np.arctan2(np.sin(current - previous), np.cos(current - previous))

    def _heading_from_target_bbox(self, bbox):
        """Compute desired heading and normalized x error from target bbox."""
        if bbox is None or self.latest_color_shape is None:
            return self.current_theta, 0.0

        rx, _, rw, _ = bbox
        img_center_x = rx + 0.5 * rw
        color_width = float(max(self.latest_color_shape[1], 1))
        x_error = ((img_center_x / color_width) - 0.5) * 2.0
        bearing = np.clip(
            self.approach_turn_sign * self.seek_heading_gain * x_error,
            -self.approach_max_bearing_rad,
            self.approach_max_bearing_rad,
        )
        return self.current_theta + bearing, x_error

    def _recenter_from_last_target(self, current_time) -> bool:
        """Rotate toward last seen target for a short window to improve reacquisition."""
        if self.last_target_heading is None or self.last_target_heading_time is None:
            return False
        dt = (current_time - self.last_target_heading_time).nanoseconds / 1e9
        if dt > self.approach_target_memory_s:
            return False

        self.publish_pose_target(self.current_x, self.current_y, self.last_target_heading)
        return True

    def _nudge_tilt(self, down: bool, scale: float = 1.0):
        """Nudge camera tilt in simple mode (down or up)."""
        if not self.auto_tilt_enable:
            return
        step = max(0.0, float(scale)) * self.simple_tilt_down_step_rad
        step = min(step, self.simple_tilt_down_step_rad * 2.0)
        down_sign = 1.0 if self.approach_tilt_down_sign >= 0.0 else -1.0
        cmd_sign = down_sign if down else -down_sign
        self.camera_tilt_cmd += cmd_sign * step

        # Extend allowed tilt envelope in the configured "down" direction for simple mode.
        down_limit = abs(self.simple_tilt_down_limit_rad)
        if down_sign > 0.0:
            tilt_min = self.auto_tilt_min_rad
            tilt_max = max(self.auto_tilt_max_rad, down_limit)
        else:
            tilt_min = min(self.auto_tilt_min_rad, -down_limit)
            tilt_max = self.auto_tilt_max_rad
        self.camera_tilt_cmd = float(
            np.clip(self.camera_tilt_cmd, tilt_min, tilt_max)
        )
        msg = Float64MultiArray()
        msg.data = [self.camera_pan_cmd, self.camera_tilt_cmd]
        self.pan_tilt_pub.publish(msg)

    def _handle_simple_yolo_approach(self):
        """
        Simplified YOLO approach:
        1) rotate in place until bbox is centered,
        2) then drive straight forward,
        3) tilt camera down only until target is vertically centered.
        """
        now = self.get_clock().now()
        target_visible_now = self.red_target_visible and self.red_bbox_color is not None

        if target_visible_now:
            bbox = self.red_bbox_color
            distance_m = self.red_target_distance_m
            target_theta, x_error = self._heading_from_target_bbox(bbox)
            self.last_target_heading = target_theta
            self.last_target_heading_time = now
            if distance_m is not None:
                self.last_target_distance_m = distance_m
        else:
            if self.last_target_heading is None or self.last_target_heading_time is None:
                self.publish_hold_position()
                return
            dt = (now - self.last_target_heading_time).nanoseconds / 1e9
            if dt > self.approach_target_memory_s:
                self.publish_hold_position()
                return
            target_theta = self.last_target_heading
            x_error = 1.0  # force recenter behavior while blind
            distance_m = self.last_target_distance_m

        # Vertical framing control (simple rule):
        # - if bbox center is lower than desired row, tilt down
        # - if bbox center is higher than desired row, tilt up
        need_tilt_down = False
        need_tilt_up = False
        tilt_error = 0.0
        if target_visible_now and self.latest_color_shape is not None:
            _, by, _, bh = self.red_bbox_color
            image_h = float(max(self.latest_color_shape[0], 1))
            center_y_frac = (by + 0.5 * bh) / image_h
            tilt_error = center_y_frac - self.auto_tilt_target_y_frac
            need_tilt_down = tilt_error > self.simple_tilt_deadband_frac
            need_tilt_up = tilt_error < -self.simple_tilt_deadband_frac

        # Stop when centered and close by depth (or obstacle-range fallback).
        stop_distance = self.seek_target_stop_distance_m + self.approach_stop_distance_buffer_m
        distance_for_stop = distance_m if distance_m is not None else self.last_target_distance_m
        close_by_depth = (
            distance_for_stop is not None
            and distance_for_stop <= stop_distance
        )
        close_by_bbox = False
        if target_visible_now and self.latest_color_shape is not None and distance_for_stop is None:
            _, _, bw, bh = self.red_bbox_color
            image_h, image_w = self.latest_color_shape
            bbox_area_frac = (bw * bh) / float(max(1, image_h * image_w))
            close_by_bbox = bbox_area_frac >= self.simple_reach_bbox_area_frac

        # Allow a bit more x-error only when close by depth; keep strict for bbox-only fallback.
        if close_by_depth:
            stop_center_tolerance = self.approach_center_tolerance * 1.4
        else:
            stop_center_tolerance = self.approach_center_tolerance
        reached_visible = (
            target_visible_now
            and abs(x_error) <= stop_center_tolerance
            and (close_by_depth or close_by_bbox)
        )
        if reached_visible:
            self.approach_reached_visible_streak += 1
        else:
            self.approach_reached_visible_streak = 0

        if self.approach_reached_visible_streak >= max(1, self.approach_reached_visible_frames):
            self.publish_hold_position()
            if not self.red_target_reached_logged:
                self.get_logger().info(
                    f'Target reached within {distance_m:.2f}m. Holding position.'
                )
                self.red_target_reached_logged = True
            return
        self.red_target_reached_logged = False

        # Rotate slowly until centered.
        if target_visible_now:
            centered = abs(x_error) <= self.approach_center_tolerance
        else:
            centered = False
        if not centered:
            yaw_delta = self._wrapped_angle_diff(target_theta, self.current_theta)
            yaw_step = float(np.clip(yaw_delta, -self.simple_center_turn_step_rad, self.simple_center_turn_step_rad))
            self.publish_pose_target(self.current_x, self.current_y, self.current_theta + yaw_step)
            if target_visible_now and (need_tilt_down or need_tilt_up):
                tilt_scale = float(np.clip(
                    self.simple_tilt_base_scale_rotate + abs(tilt_error) * 1.2,
                    0.25,
                    0.95,
                ))
                if need_tilt_down:
                    self._nudge_tilt(down=True, scale=tilt_scale)
                elif need_tilt_up:
                    self._nudge_tilt(
                        down=False,
                        scale=tilt_scale * max(0.0, min(1.0, self.simple_tilt_up_scale)),
                    )
            return

        # Centered: move straight forward.
        # If we can see target but have no depth estimate, don't drive blind.
        if target_visible_now and distance_m is None:
            self.publish_hold_position()
            if need_tilt_down or need_tilt_up:
                tilt_scale = float(np.clip(0.40 + abs(tilt_error) * 1.0, 0.30, 0.90))
                if need_tilt_down:
                    self._nudge_tilt(down=True, scale=tilt_scale)
                else:
                    self._nudge_tilt(
                        down=False,
                        scale=tilt_scale * max(0.0, min(1.0, self.simple_tilt_up_scale)),
                    )
            return
        # If target is lost and we have no remembered range, don't drive blind.
        if (not target_visible_now) and distance_m is None:
            self.publish_hold_position()
            return

        if distance_m is not None:
            advance = float(np.clip(
                distance_m - self.seek_target_stop_distance_m,
                self.simple_min_forward_step_m,
                self.simple_forward_step_m,
            ))
        else:
            advance = self.simple_forward_step_m * 0.6

        target_x = self.current_x + advance * np.cos(self.current_theta)
        target_y = self.current_y + advance * np.sin(self.current_theta)
        self.publish_pose_target(target_x, target_y, self.current_theta)

        if target_visible_now and (need_tilt_down or need_tilt_up):
            tilt_scale = 0.45 + abs(tilt_error) * 1.25
            if distance_m is not None and distance_m < self.auto_tilt_near_dist_m:
                tilt_scale += 0.10
            tilt_scale = float(np.clip(tilt_scale, 0.35, 0.95))
            if need_tilt_down:
                self._nudge_tilt(down=True, scale=tilt_scale)
            else:
                self._nudge_tilt(
                    down=False,
                    scale=tilt_scale * max(0.0, min(1.0, self.simple_tilt_up_scale)),
                )

    def _current_cell_is_unexplored(self) -> bool:
        current_cell = self._cell_from_xy(self.current_x, self.current_y)
        visits = self.visit_counts.get(current_cell, 0)
        return (
            visits <= self.explore_unexplored_visits_threshold
            and current_cell not in self.scanned_cells
        )

    def _required_scan_radians(self) -> float:
        if self._current_cell_is_unexplored():
            return self.seek_search_scan_total_rad
        return self.SEEK_SEARCH_SCAN_EXPLORED_RAD

    def _start_search_move(self, current_time):
        heading_offset = self._select_open_heading_offset_from_depth()
        target_theta = self.current_theta + heading_offset
        target_x = self.current_x + self.seek_search_move_distance_m * np.cos(target_theta)
        target_y = self.current_y + self.seek_search_move_distance_m * np.sin(target_theta)

        self.seek_mode = 'search_move'
        self.search_move_start_time = current_time
        self.search_move_start_x = self.current_x
        self.search_move_start_y = self.current_y
        self.search_move_target_x = target_x
        self.search_move_target_y = target_y
        self.search_move_target_theta = target_theta
        self.get_logger().info(
            f'Search move: heading offset {np.rad2deg(heading_offset):.0f} deg '
            f'to less-visited/open space.'
        )

    def _handle_search_scan_mode(self):
        yaw_delta = abs(self._wrapped_angle_diff(self.current_theta, self.scan_last_theta))
        self.scan_accumulated_yaw += yaw_delta
        self.scan_last_theta = self.current_theta

        required_scan = self._required_scan_radians()
        if self.scan_accumulated_yaw >= required_scan:
            if self._current_cell_is_unexplored():
                self.scanned_cells.add(self._cell_from_xy(self.current_x, self.current_y))
            self.scan_accumulated_yaw = 0.0
            self.seek_mode = 'search_move'
            self._start_search_move(self.get_clock().now())
            return

        rotate_theta = self.current_theta + self.seek_search_scan_step_rad
        self.publish_pose_target(self.current_x, self.current_y, rotate_theta)

    def _handle_search_move_mode(self, current_time):
        if self.is_obstacle_blocking():
            self.seek_mode = 'search_scan'
            self.scan_accumulated_yaw = 0.0
            self.scan_last_theta = self.current_theta
            self.get_logger().info('Search move interrupted by obstacle, returning to scan.')
            return

        elapsed = (current_time - self.search_move_start_time).nanoseconds / 1e9
        dx = self.current_x - self.search_move_start_x
        dy = self.current_y - self.search_move_start_y
        moved = np.hypot(dx, dy)
        if elapsed >= self.seek_search_move_duration_s or moved >= self.seek_search_move_distance_m:
            self.seek_mode = 'search_scan'
            self.scan_accumulated_yaw = 0.0
            self.scan_last_theta = self.current_theta
            self.get_logger().info('Search move complete, rescanning.')
            return

        self.publish_pose_target(
            self.search_move_target_x,
            self.search_move_target_y,
            self.search_move_target_theta,
        )

    def _handle_approach_mode(self):
        if self.use_yolo_target_detection and self.simple_yolo_approach_enable:
            self._handle_simple_yolo_approach()
            return

        now = self.get_clock().now()
        target_visible_now = self.red_target_visible and self.red_bbox_color is not None
        bbox = self.red_bbox_color
        distance_m = self.red_target_distance_m

        heading_error = 0.0
        if target_visible_now:
            target_theta, x_error = self._heading_from_target_bbox(bbox)
            self.last_target_heading = target_theta
            self.last_target_heading_time = now

            # If target is visible but depth ROI dropped out, use recent target range estimate.
            if distance_m is None and self.last_red_seen_time is not None and self.last_red_distance_m is not None:
                dt = (now - self.last_red_seen_time).nanoseconds / 1e9
                if dt <= self.approach_target_memory_s:
                    distance_m = self.last_red_distance_m
            if distance_m is not None:
                self.last_target_distance_m = distance_m
        else:
            # During brief YOLO dropout, keep following last known target heading/range.
            if self.last_target_heading is None or self.last_target_heading_time is None:
                self.publish_hold_position()
                return
            dt = (now - self.last_target_heading_time).nanoseconds / 1e9
            if dt > self.approach_target_memory_s:
                self.publish_hold_position()
                return

            target_theta = self.last_target_heading
            heading_error = self._wrapped_angle_diff(target_theta, self.current_theta)
            x_error = 0.0
            if distance_m is None:
                distance_m = self.last_target_distance_m

        blocked = self._is_approach_path_blocked(distance_m, target_visible_now)
        obstacle_is_target = False
        if (
            blocked
            and target_visible_now
            and distance_m is not None
            and self.obstacle_distance_m is not None
            and abs(self.obstacle_distance_m - distance_m) <= self.approach_obstacle_target_margin_m
        ):
            obstacle_is_target = True

        if blocked and not obstacle_is_target and not self.approach_avoid_active:
            self.approach_avoid_active = True
            self.approach_avoid_start_time = self.get_clock().now()
            self.approach_clear_streak = 0
            self.approach_avoid_cmd_count = 0
            self.approach_avoid_side = self._choose_avoid_side(bbox)
            self.get_logger().info(
                f'APPROACH blocked: entering avoidance mode (side={"left" if self.approach_avoid_side > 0 else "right"}).'
            )

        if self.approach_avoid_active:
            if not blocked or obstacle_is_target:
                self.approach_clear_streak += 1
            else:
                self.approach_clear_streak = 0

            elapsed_avoid = (now - self.approach_avoid_start_time).nanoseconds / 1e9
            if self.approach_clear_streak >= max(1, self.approach_clear_frames) or elapsed_avoid >= self.approach_avoid_max_duration_s:
                self.approach_avoid_active = False
                self.approach_clear_streak = 0
                self.approach_avoid_cmd_count = 0
                self.get_logger().info('APPROACH: exiting avoidance mode, resuming target tracking.')
            else:
                cadence = max(1, self.approach_avoid_turns_before_nudge) + 1
                do_nudge = (self.approach_avoid_cmd_count % cadence) == (cadence - 1)
                if self.approach_avoid_cmd_count >= 6:
                    # If we've been avoiding for a while, bias toward forward probes.
                    do_nudge = True
                if do_nudge:
                    nudge_theta = self.current_theta + self.approach_avoid_side * 0.12
                    avoid_theta = nudge_theta
                    avoid_x = self.current_x + self.approach_avoid_nudge_step_m * np.cos(nudge_theta)
                    avoid_y = self.current_y + self.approach_avoid_nudge_step_m * np.sin(nudge_theta)
                else:
                    avoid_theta = self.current_theta + self.approach_avoid_side * self.approach_avoid_turn_step_rad
                    avoid_x = self.current_x + self.approach_avoid_step_m * np.cos(avoid_theta)
                    avoid_y = self.current_y + self.approach_avoid_step_m * np.sin(avoid_theta)
                self.approach_avoid_cmd_count += 1
                self.publish_pose_target(avoid_x, avoid_y, avoid_theta)
                return

        # Only declare "reached" using fresh visible centered detections (no memory-only latch).
        centered_now = target_visible_now and abs(x_error) <= self.approach_center_tolerance
        reached_now = (
            target_visible_now
            and centered_now
            and distance_m is not None
            and distance_m <= self.seek_target_stop_distance_m
        )
        if reached_now:
            self.approach_reached_visible_streak += 1
        else:
            self.approach_reached_visible_streak = 0

        if self.approach_reached_visible_streak >= max(1, self.approach_reached_visible_frames):
            self.publish_hold_position()
            if not self.red_target_reached_logged:
                self.get_logger().info(
                    f'Target reached within {distance_m:.2f}m. Holding position.'
                )
                self.red_target_reached_logged = True
            return
        self.red_target_reached_logged = False

        # Rotate in place until target is centered (or remembered heading is aligned).
        if target_visible_now:
            if self.approach_centered_locked:
                should_rotate = abs(x_error) > (
                    self.approach_center_tolerance + self.approach_center_hysteresis
                )
            else:
                should_rotate = abs(x_error) > self.approach_center_tolerance
            self.approach_centered_locked = not should_rotate
        else:
            should_rotate = abs(heading_error) > 0.08
            self.approach_centered_locked = False
        if should_rotate:
            self.publish_pose_target(self.current_x, self.current_y, target_theta)
            return

        # Once centered, advance forward toward current/remembered target range.
        forward_cap = self.seek_forward_step_m if target_visible_now else (self.seek_forward_step_m * 0.6)
        if distance_m is not None:
            advance = np.clip(
                distance_m - self.seek_target_stop_distance_m,
                0.04,
                forward_cap,
            )
        else:
            advance = forward_cap * 0.6

        # When target is centered in view, avoid micro-rotations during approach.
        if target_visible_now:
            target_theta = self.current_theta

        target_x = self.current_x + advance * np.cos(target_theta)
        target_y = self.current_y + advance * np.sin(target_theta)
        self.publish_pose_target(target_x, target_y, target_theta)

    def _is_approach_path_blocked(self, target_distance_m, target_visible) -> bool:
        """
        Simple approach-mode blocker check.
        Only avoid when a near obstacle is directly in front of the robot.
        """
        if not self.is_obstacle_blocking():
            return False

        if self.obstacle_distance_m is None or self.obstacle_distance_m > self.approach_block_max_distance_m:
            return False

        # If we know target range, do not avoid when obstacle depth likely belongs to the target.
        if target_visible and target_distance_m is not None:
            if self.obstacle_distance_m >= target_distance_m - self.approach_obstacle_target_margin_m:
                return False

        now = self.get_clock().now()
        dt = (now - self.last_approach_block_log_time).nanoseconds / 1e9
        if dt >= self.APPROACH_BLOCK_LOG_INTERVAL:
            self.get_logger().info(
                f'APPROACH path blocked (front): obstacle={self.obstacle_distance_m:.2f}m '
                f'target={target_distance_m if target_distance_m is not None else float("nan"):.2f}m'
            )
            self.last_approach_block_log_time = now
        return True

    def _choose_avoid_side(self, bbox) -> float:
        """Choose left/right avoidance side using depth openness with target-side tie-break."""
        left_open, right_open = self._left_right_openness_from_depth()
        if abs(left_open - right_open) > 0.1:
            return 1.0 if left_open > right_open else -1.0

        if bbox is not None and self.latest_color_shape is not None:
            rx, _, rw, _ = bbox
            img_center_x = rx + 0.5 * rw
            color_width = float(max(self.latest_color_shape[1], 1))
            x_error = ((img_center_x / color_width) - 0.5) * 2.0
            side = np.sign(self.approach_turn_sign * x_error)
            if side != 0.0:
                return float(side)
        return 1.0

    def _left_right_openness_from_depth(self):
        """Return median open depth (m) for left and right forward halves."""
        if self.latest_depth_mm is None or self.latest_depth_valid is None:
            return 0.0, 0.0
        depth_h, depth_w = self.latest_depth_shape
        y1 = int(depth_h * 0.45)
        y2 = int(depth_h * 0.92)
        mid = depth_w // 2

        left_depth = self.latest_depth_mm[y1:y2, :mid]
        left_valid = self.latest_depth_valid[y1:y2, :mid]
        right_depth = self.latest_depth_mm[y1:y2, mid:]
        right_valid = self.latest_depth_valid[y1:y2, mid:]

        left_open = float(np.median(left_depth[left_valid]) / 1000.0) if np.any(left_valid) else 0.0
        right_open = float(np.median(right_depth[right_valid]) / 1000.0) if np.any(right_valid) else 0.0
        return left_open, right_open

    def _select_open_heading_offset_from_depth(self) -> float:
        """
        Pick heading offset from current heading using depth openness + memory penalty.
        Returns angle offset in radians.
        """
        max_offset = self.seek_search_open_heading_max_offset_rad
        if self.latest_depth_mm is None or self.latest_depth_valid is None:
            return 0.0

        depth_h, depth_w = self.latest_depth_shape
        y1 = int(depth_h * 0.40)
        y2 = int(depth_h * 0.92)
        num_sectors = 9

        best_score = None
        best_offset = 0.0
        for i in range(num_sectors):
            frac_center = (i + 0.5) / num_sectors
            frac_left = i / num_sectors
            frac_right = (i + 1) / num_sectors
            x1 = int(frac_left * depth_w)
            x2 = int(frac_right * depth_w)
            if x2 <= x1:
                continue

            roi_depth = self.latest_depth_mm[y1:y2, x1:x2]
            roi_valid = self.latest_depth_valid[y1:y2, x1:x2]
            if np.any(roi_valid):
                open_depth_m = float(np.median(roi_depth[roi_valid]) / 1000.0)
            else:
                open_depth_m = 0.0

            offset = (frac_center - 0.5) * 2.0 * max_offset
            probe_theta = self.current_theta + offset
            probe_dist = max(0.5, min(open_depth_m, self.seek_search_move_distance_m))
            probe_x = self.current_x + probe_dist * np.cos(probe_theta)
            probe_y = self.current_y + probe_dist * np.sin(probe_theta)
            visits = self._visit_count_at(probe_x, probe_y)
            score = open_depth_m - self.seek_search_visit_penalty * visits

            if best_score is None or score > best_score:
                best_score = score
                best_offset = offset

        return best_offset

    def _cell_from_xy(self, x: float, y: float):
        """Map world position to exploration-memory grid cell."""
        cell_size = max(self.explore_memory_cell_size_m, 1e-3)
        return (
            int(np.floor(x / cell_size)),
            int(np.floor(y / cell_size)),
        )

    def _record_visit(self, x: float, y: float):
        """Increment visit count only when entering a new memory cell."""
        cell = self._cell_from_xy(x, y)
        if cell != self.last_visited_cell:
            self.visit_counts[cell] = self.visit_counts.get(cell, 0) + 1
            self.last_visited_cell = cell

    def _visit_count_at(self, x: float, y: float) -> int:
        return self.visit_counts.get(self._cell_from_xy(x, y), 0)

    def handle_arrived(self, current_time):
        """Hold position briefly, then mark ready for next goal."""
        # Keep holding the target position
        target = Pose2D()
        target.x = self.target_x
        target.y = self.target_y
        target.theta = self.target_theta
        self.base_pose_pub.publish(target)

        # After holding for a bit, mark as done
        time_in_state = (current_time - self.state_start_time).nanoseconds / 1e9
        if time_in_state >= self.ARRIVAL_HOLD_TIME:
            self.get_logger().info('Ready for next goal!')
            self.transition_to_state(State.DONE, current_time)

    def handle_done(self):
        """Idle state - waiting for next RViz click."""
        self.publish_hold_position()

    def transition_to_state(self, new_state: State, current_time):
        self.state = new_state
        self.state_start_time = current_time


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = NavigationTaskNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
