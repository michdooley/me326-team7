#!/usr/bin/env python3
"""
Banana pickup: find a banana, drive to it, pick it up, return home.

Self-contained ROS2 node. No obstacle avoidance — assumes clear path.

State machine:
    WAIT     -> hold position until YOLO detects banana (no spinning)
    APPROACH -> drive toward banana, centering it and closing distance
    GRASP    -> stop, tilt camera, detect, depth->3D, IK pick
    RETURN   -> drive back to start pose
    DONE     -> release object, stow arm, hold position

Requires:
    ros2 launch tidybot_bringup sim.launch.py  (or real.launch.py use_planner:=true)
    ros2 run object_classification classifier

Usage:
    ros2 run tidybot_bringup banana_pickup.py
    ros2 run tidybot_bringup banana_pickup.py --ros-args -p sim:=false
"""

import time
from enum import Enum

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation

import tf2_ros

from geometry_msgs.msg import Pose, Pose2D, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray
from tidybot_msgs.msg import ArmCommand
from tidybot_msgs.srv import PlanToTarget
from vision_msgs.msg import Detection2DArray

try:
    from cv_bridge import CvBridge
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
BANANA_CLASS_ID = '46'
GRIPPER_MAX_OPENING_M = 0.074
SLEEP_POSE = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]


class TaskState(Enum):
    WAIT = 0
    APPROACH = 1
    GRASP = 2
    RETURN = 3
    DONE = 4


def yaw_to_grasp_quaternion(yaw_rad):
    """Fingers-down quaternion with yaw around base_link Z."""
    R_down = Rotation.from_quat([0.5, 0.5, -0.5, 0.5])
    R_yaw = Rotation.from_euler('z', yaw_rad)
    R_grasp = R_yaw * R_down
    qx, qy, qz, qw = R_grasp.as_quat()
    return float(qw), float(qx), float(qy), float(qz)


def wrap_angle(a):
    """Wrap angle to [-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class BananaPickupNode(Node):

    # --- Approach tuning ---
    STOP_DEPTH_M = 0.20          # stop when banana is this close (depth meters)
    TURN_STEP_RAD = 0.08         # how much to yaw per cycle to center banana
    FORWARD_STEP_M = 0.10        # how far to step forward per cycle
    CENTER_TOLERANCE = 0.12      # fraction of image width: banana is "centered"

    # --- Grasp tuning ---
    GRASP_CAMERA_TILT_RAD = 0.65
    PRE_GRASP_HEIGHT = 0.10
    LIFT_HEIGHT = 0.15
    Z_OFFSET = -0.02
    GRIPPER_CLOSE_REPEATS = 20
    GRIPPER_CLOSE_WAIT = 2.0
    MIN_DEPTH_M = 0.1
    MAX_DEPTH_M = 2.0
    MIN_POINTS = 20
    TABLE_Z_MIN = 0.005
    OUTLIER_Z_THRESH = 0.05
    BBOX_PAD = 1.0

    # --- Return tuning ---
    RETURN_POS_TOL = 0.12
    RETURN_ANG_TOL = 0.20

    def __init__(self):
        super().__init__('banana_pickup_node')

        self.is_sim = bool(self.declare_parameter('sim', True).value)
        self.grasp_arm = str(self.declare_parameter('grasp_arm', 'right').value)
        self.grasp_move_duration = float(self.declare_parameter(
            'grasp_duration', 2.0 if self.is_sim else 3.0).value)

        self.state = TaskState.WAIT

        # --- Odometry ---
        self.odom_received = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_theta = 0.0
        self.home_set = False

        # --- YOLO ---
        self.banana_bbox = None        # (cx, cy, w, h) in pixels
        self.banana_conf = 0.0
        self.banana_time = None
        self._last_raw_det = None

        # --- Depth ---
        self.latest_depth_mm = None
        self.cv_bridge = CvBridge() if CV_AVAILABLE else None

        # --- Camera info ---
        self.camera_info = None
        self.depth_camera_info = None

        # --- Camera pan/tilt state ---
        self.camera_pan = 0.0
        self.camera_tilt = 0.0

        # --- Approach state ---
        self.approach_depth_m = None  # latest depth to banana center

        # --- TF2 ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- QoS ---
        best_effort_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # --- Subscribers ---
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Detection2DArray, '/objbbox', self._yolo_cb, 10)
        self.create_subscription(Image, '/camera/depth/image_raw',
                                 self._depth_cb, best_effort_qos)
        self.create_subscription(Image, '/camera/color/image_raw',
                                 self._color_cb, best_effort_qos)
        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 self._cam_info_cb, 10)
        if not self.is_sim:
            self.create_subscription(CameraInfo, '/camera/depth/camera_info',
                                     self._depth_info_cb, 10)

        # --- Publishers ---
        self.base_pub = self.create_publisher(Pose2D, '/base/target_pose', 10)
        self.pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, f'/{self.grasp_arm}_gripper/cmd', 10)
        self.arm_pub = self.create_publisher(
            ArmCommand, f'/{self.grasp_arm}_arm/cmd', 10)

        # --- IK service ---
        self._srv_group = MutuallyExclusiveCallbackGroup()
        self.plan_client = self.create_client(
            PlanToTarget, '/plan_to_target', callback_group=self._srv_group)

        # --- Control loop (10 Hz) ---
        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f'BananaPickup ready (sim={self.is_sim}, arm={self.grasp_arm})')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        euler = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
        # MuJoCo frame -> user frame
        self.current_theta = euler[2] - np.pi / 2
        self.odom_received = True
        if not self.home_set:
            self.home_x = self.current_x
            self.home_y = self.current_y
            self.home_theta = self.current_theta
            self.home_set = True
            self.get_logger().info(
                f'Home: ({self.home_x:.2f}, {self.home_y:.2f}), '
                f'{np.rad2deg(self.home_theta):.0f} deg')

    def _depth_cb(self, msg):
        if self.cv_bridge is None:
            return
        try:
            raw = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            return
        if raw is None:
            return
        if raw.ndim > 2:
            raw = raw[:, :, 0]
        # Normalize to float32 mm
        if raw.dtype == np.float32:
            self.latest_depth_mm = raw
        elif raw.dtype == np.float64:
            self.latest_depth_mm = raw.astype(np.float32)
        elif raw.dtype == np.uint16:
            self.latest_depth_mm = raw.astype(np.float32)
        else:
            self.latest_depth_mm = raw.astype(np.float32)

    def _color_cb(self, msg):
        # We only need shape info for centering logic
        if self.cv_bridge is None:
            return
        try:
            img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        if img is not None:
            self._color_shape = img.shape[:2]  # (h, w)

    def _cam_info_cb(self, msg):
        self.camera_info = msg

    def _depth_info_cb(self, msg):
        self.depth_camera_info = msg

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _banana_age_s(self):
        if self.banana_time is None:
            return float('inf')
        return (self.get_clock().now() - self.banana_time).nanoseconds / 1e9

    def _banana_visible(self, max_age=1.5):
        return self.banana_bbox is not None and self._banana_age_s() < max_age

    def _hold(self):
        msg = Pose2D()
        msg.x = self.current_x
        msg.y = self.current_y
        msg.theta = self.current_theta + np.pi / 2  # back to MuJoCo frame
        self.base_pub.publish(msg)

    def _send_base(self, x, y, theta):
        msg = Pose2D()
        msg.x = x
        msg.y = y
        msg.theta = theta + np.pi / 2  # user->MuJoCo frame
        self.base_pub.publish(msg)

    def _set_pan_tilt(self, pan, tilt):
        self.camera_pan = pan
        self.camera_tilt = tilt
        msg = Float64MultiArray()
        msg.data = [pan, tilt]
        self.pan_tilt_pub.publish(msg)

    def _get_banana_depth_m(self):
        """Estimate depth to banana center from depth image."""
        if self.latest_depth_mm is None or self.banana_bbox is None:
            return None
        cx, cy, bw, bh = self.banana_bbox
        depth = self.latest_depth_mm
        h, w = depth.shape[:2]
        # Sample a small patch around center
        r = 5
        u0 = max(0, int(cx) - r)
        u1 = min(w, int(cx) + r)
        v0 = max(0, int(cy) - r)
        v1 = min(h, int(cy) + r)
        patch = depth[v0:v1, u0:u1].flatten()
        valid = patch[(patch > 150) & (patch < 5000)]
        if len(valid) < 3:
            return None
        return float(np.median(valid)) / 1000.0

    def _spin_for(self, seconds):
        time.sleep(seconds)

    def _wait_future(self, future, timeout=10.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if future.done():
                return True
            time.sleep(0.05)
        return False

    # ------------------------------------------------------------------
    # State machine tick (10 Hz)
    # ------------------------------------------------------------------
    def _tick(self):
        if not self.odom_received:
            return

        if self.state == TaskState.WAIT:
            self._do_wait()
        elif self.state == TaskState.APPROACH:
            self._do_approach()
        elif self.state == TaskState.GRASP:
            pass  # grasp runs blocking in its own call
        elif self.state == TaskState.RETURN:
            self._do_return()
        elif self.state == TaskState.DONE:
            self._hold()

    # ------------------------------------------------------------------
    # WAIT: hold position until YOLO sees the banana (no spinning)
    # ------------------------------------------------------------------
    def _do_wait(self):
        if self._banana_visible():
            self.get_logger().info(
                f'Banana detected (conf={self.banana_conf:.2f})! Approaching.')
            self.state = TaskState.APPROACH
            return

        # Hold still — banana should be right in front
        self._hold()

    # ------------------------------------------------------------------
    # APPROACH: center banana in frame, drive forward until close
    # ------------------------------------------------------------------
    def _do_approach(self):
        if not self._banana_visible(max_age=3.0):
            self.get_logger().info('Lost banana — holding position, waiting.')
            self._hold()
            self.state = TaskState.WAIT
            return

        cx, cy, bw, bh = self.banana_bbox
        img_w = getattr(self, '_color_shape', (480, 640))[1]

        # Fraction of image: 0=left, 1=right
        frac = cx / img_w
        offset = frac - 0.5  # negative=left, positive=right

        # Check depth
        depth_m = self._get_banana_depth_m()
        if depth_m is not None:
            self.approach_depth_m = depth_m

        # Auto-tilt camera to keep banana in view
        img_h = getattr(self, '_color_shape', (480, 640))[0]
        y_frac = cy / img_h
        if y_frac > 0.80:
            self._set_pan_tilt(self.camera_pan,
                               min(self.camera_tilt + 0.06, 1.0))
        elif y_frac < 0.30:
            self._set_pan_tilt(self.camera_pan,
                               max(self.camera_tilt - 0.06, -0.5))

        # Are we close enough?
        if depth_m is not None and depth_m < self.STOP_DEPTH_M:
            self.get_logger().info(
                f'Close enough (depth={depth_m:.2f}m). Starting grasp.')
            self._hold()
            self.state = TaskState.GRASP
            self._run_grasp()
            return

        # Steer: turn to center, then step forward
        if abs(offset) > self.CENTER_TOLERANCE:
            # Turn toward banana — negative offset means turn left (positive theta)
            turn = -offset * 0.5  # proportional control
            turn = np.clip(turn, -self.TURN_STEP_RAD, self.TURN_STEP_RAD)
            self._send_base(self.current_x, self.current_y,
                            self.current_theta + turn)
        else:
            # Drive forward in current heading
            step = self.FORWARD_STEP_M
            if depth_m is not None and depth_m < 0.5:
                step = self.FORWARD_STEP_M * 0.5  # slow down when close
            dx = step * np.cos(self.current_theta)
            dy = step * np.sin(self.current_theta)
            self._send_base(self.current_x + dx, self.current_y + dy,
                            self.current_theta)

    # ------------------------------------------------------------------
    # GRASP: blocking pipeline
    # ------------------------------------------------------------------
    def _run_grasp(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info('GRASP PIPELINE START')
        self.get_logger().info('=' * 50)

        success = False
        try:
            success = self._execute_grasp()
        except Exception as e:
            self.get_logger().error(f'Grasp exception: {e}')

        # Reset camera level
        self._set_pan_tilt(0.0, 0.0)
        self._spin_for(0.5)

        if success:
            self.get_logger().info('Grasp succeeded! Returning home.')
        else:
            self.get_logger().warn('Grasp failed. Returning home empty-handed.')

        self.grasp_success = success
        self.state = TaskState.RETURN

    def _execute_grasp(self):
        # Wait for planner
        self.get_logger().info('Waiting for /plan_to_target...')
        if not self.plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/plan_to_target not available!')
            return False

        # Tilt camera for close-range view
        self._set_pan_tilt(0.0, self.GRASP_CAMERA_TILT_RAD)
        self._spin_for(1.5)

        # Detect banana
        det = self._detect_banana()
        if det is None:
            self.get_logger().error('Cannot find banana for grasp.')
            return False

        cx = int(det.bbox.center.position.x)
        cy = int(det.bbox.center.position.y)
        bw = float(det.bbox.size_x)
        bh = float(det.bbox.size_y)
        self.get_logger().info(f'  Banana at pixel ({cx}, {cy}), {bw:.0f}x{bh:.0f}')

        # Transform bbox for real hardware depth alignment
        if not self.is_sim and self.depth_camera_info is not None and self.camera_info is not None:
            cx, cy, bw, bh = self._transform_bbox_to_depth(cx, cy, bw, bh)

        # Get 3D points
        pts = self._get_object_points(cx, cy, bw, bh)
        if pts is None:
            self.get_logger().error('Insufficient depth points.')
            return False
        self.get_logger().info(f'  {len(pts)} 3D points')

        # Analyze
        analysis = self._analyze_object(pts)
        centroid = analysis['centroid']
        self.get_logger().info(
            f'  Centroid: ({centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f})')

        # Compute poses
        grasp_pose, pre_grasp_pose, info = self._compute_grasp_pose(analysis)
        self.get_logger().info(f'  Grasp: {info}')

        # IK check
        self.get_logger().info('  Checking IK...')
        use_orient = True
        ok, msg = self._try_ik(grasp_pose, use_orientation=True)
        if not ok:
            self.get_logger().warn(f'  IK w/ orient failed: {msg}')
            ok, msg = self._try_ik(grasp_pose, use_orientation=False)
            if ok:
                use_orient = False
            else:
                self.get_logger().error(f'  IK failed: {msg}')
                return False

        # Open gripper
        self.get_logger().info('  Opening gripper...')
        self._set_gripper(0.0)

        # Pre-grasp
        self.get_logger().info('  Moving to pre-grasp...')
        if not self._plan_and_execute(pre_grasp_pose, 'pre-grasp', use_orient):
            self.get_logger().error('  Pre-grasp failed.')
            self._stow_arm()
            return False
        self._spin_for(1.0)

        # Descend
        self.get_logger().info('  Descending...')
        if not self._plan_and_execute(grasp_pose, 'grasp', use_orient):
            self.get_logger().error('  Grasp descent failed.')
            self._stow_arm()
            return False
        self._spin_for(1.0)

        # Close gripper
        self.get_logger().info('  Closing gripper...')
        self._close_gripper_firm()

        # Lift
        self.get_logger().info('  Lifting...')
        lift_pose = Pose()
        lift_pose.position.x = grasp_pose.position.x
        lift_pose.position.y = grasp_pose.position.y
        lift_pose.position.z = grasp_pose.position.z + self.LIFT_HEIGHT
        lift_pose.orientation = grasp_pose.orientation
        if not self._plan_and_execute(lift_pose, 'lift', use_orient):
            self.get_logger().warn('  Lift failed, continuing anyway.')

        self.get_logger().info('GRASP PIPELINE COMPLETE')
        return True

    def _detect_banana(self):
        """Poll for fresh banana detection."""
        for _ in range(20):
            self._spin_for(0.2)
            if not self._banana_visible(max_age=1.0):
                continue
            # Reconstruct detection from latest _yolo_cb data
            # We need the raw Detection2D — re-subscribe isn't needed since
            # _yolo_cb stores bbox. But for the grasp pipeline we need the
            # raw msg. We'll store it.
            if hasattr(self, '_last_raw_det') and self._last_raw_det is not None:
                return self._last_raw_det
        return None

    def _yolo_cb(self, msg):
        """Track best banana detection and store raw Detection2D for grasp."""
        best, best_conf = None, 0.0
        for det in msg.detections:
            for hyp in det.results:
                if (hyp.hypothesis.class_id == BANANA_CLASS_ID
                        and hyp.hypothesis.score > best_conf):
                    best = det
                    best_conf = hyp.hypothesis.score
        if best is not None:
            self.banana_bbox = (
                float(best.bbox.center.position.x),
                float(best.bbox.center.position.y),
                float(best.bbox.size_x),
                float(best.bbox.size_y),
            )
            self.banana_conf = best_conf
            self.banana_time = self.get_clock().now()
            self._last_raw_det = best

    def _transform_bbox_to_depth(self, u, v, bw, bh):
        K_rgb = np.array(self.camera_info.k).reshape(3, 3)
        K_depth = np.array(self.depth_camera_info.k).reshape(3, 3)
        pt = np.array([u, v, 1.0])
        pt_norm = np.linalg.inv(K_rgb) @ pt
        pt_depth = K_depth @ pt_norm
        u_d, v_d = int(round(pt_depth[0])), int(round(pt_depth[1]))
        sx = K_depth[0, 0] / K_rgb[0, 0]
        sy = K_depth[1, 1] / K_rgb[1, 1]
        return u_d, v_d, bw * sx, bh * sy

    def _get_object_points(self, cx, cy, bbox_w, bbox_h):
        if self.latest_depth_mm is None:
            return None

        if not self.is_sim and self.depth_camera_info is not None:
            K = self.depth_camera_info.k
            tf_frame = 'camera_depth_optical_frame'
        else:
            if self.camera_info is None:
                return None
            K = self.camera_info.k
            tf_frame = 'camera_color_optical_frame'

        fx, fy, cx_k, cy_k = K[0], K[4], K[2], K[5]

        h, w = self.latest_depth_mm.shape[:2]
        half_w = int(bbox_w * self.BBOX_PAD / 2)
        half_h = int(bbox_h * self.BBOX_PAD / 2)
        u0 = max(0, cx - half_w)
        v0 = max(0, cy - half_h)
        u1 = min(w, cx + half_w)
        v1 = min(h, cy + half_h)
        roi = self.latest_depth_mm[v0:v1, u0:u1]

        depth_m = roi.astype(np.float32) / 1000.0

        roi_h, roi_w = depth_m.shape[:2]
        u_grid, v_grid = np.meshgrid(
            np.arange(roi_w) + u0, np.arange(roi_h) + v0)
        mask = (depth_m > self.MIN_DEPTH_M) & (depth_m < self.MAX_DEPTH_M)
        z = depth_m[mask]
        u_pts = u_grid[mask].astype(np.float32)
        v_pts = v_grid[mask].astype(np.float32)

        if len(z) < self.MIN_POINTS:
            return None

        x_cam = (u_pts - cx_k) * z / fx
        y_cam = (v_pts - cy_k) * z / fy
        points_cam = np.stack([x_cam, y_cam, z], axis=1)

        tf_mat = self._get_tf_matrix('base_link', tf_frame)
        if tf_mat is None:
            return None

        ones = np.ones((len(points_cam), 1), dtype=np.float32)
        pts_h = np.hstack([points_cam, ones])
        pts_base = (tf_mat @ pts_h.T).T[:, :3]

        # Filter above table
        pts_base = pts_base[pts_base[:, 2] > self.TABLE_Z_MIN]

        # Remove Z outliers
        if len(pts_base) > self.MIN_POINTS:
            med_z = np.median(pts_base[:, 2])
            pts_base = pts_base[np.abs(pts_base[:, 2] - med_z) < self.OUTLIER_Z_THRESH]

        if len(pts_base) < self.MIN_POINTS:
            return None
        return pts_base

    def _get_tf_matrix(self, target, source):
        try:
            import rclpy.time
            import rclpy.duration
            t = self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
            tr = t.transform.translation
            q = t.transform.rotation
            rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
            mat = np.eye(4, dtype=np.float32)
            mat[:3, :3] = rot.as_matrix().astype(np.float32)
            mat[:3, 3] = [tr.x, tr.y, tr.z]
            return mat
        except Exception as e:
            self.get_logger().error(f'TF lookup failed: {e}')
            return None

    def _analyze_object(self, pts):
        centroid = np.mean(pts, axis=0)
        z_top = float(np.max(pts[:, 2]))

        centered = pts[:, :2] - centroid[:2]
        best_yaw, min_w = 0.0, float('inf')
        for deg in range(0, 180, 5):
            angle = np.radians(deg)
            d = np.array([np.cos(angle), np.sin(angle)])
            width = float(np.ptp(centered @ d))
            if width < min_w:
                min_w = width
                best_yaw = angle

        return {
            'centroid': centroid,
            'grasp_yaw': best_yaw,
            'grip_width': min_w,
            'z_top': z_top,
        }

    def _compute_grasp_pose(self, analysis):
        centroid = analysis['centroid']
        yaw = analysis['grasp_yaw']
        grip_w = analysis['grip_width']

        qw, qx, qy, qz = yaw_to_grasp_quaternion(yaw)
        gz = analysis['z_top'] + self.Z_OFFSET

        grasp_pose = Pose()
        grasp_pose.position.x = float(centroid[0])
        grasp_pose.position.y = float(centroid[1])
        grasp_pose.position.z = float(gz)
        grasp_pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

        pre = Pose()
        pre.position.x = float(centroid[0])
        pre.position.y = float(centroid[1])
        pre.position.z = float(gz + self.PRE_GRASP_HEIGHT)
        pre.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

        info = (f'yaw={np.degrees(yaw):.0f}deg, z={gz:.3f}m, '
                f'width={grip_w*1000:.0f}mm')
        return grasp_pose, pre, info

    def _try_ik(self, pose, use_orientation=True):
        req = PlanToTarget.Request()
        req.arm_name = self.grasp_arm
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = False
        req.duration = self.grasp_move_duration
        req.max_condition_number = 200.0

        future = self.plan_client.call_async(req)
        if not self._wait_future(future):
            return False, 'timeout'
        if future.exception():
            return False, str(future.exception())
        r = future.result()
        return r.success, r.message

    def _plan_and_execute(self, pose, label, use_orientation=True):
        req = PlanToTarget.Request()
        req.arm_name = self.grasp_arm
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = True
        req.duration = self.grasp_move_duration
        req.max_condition_number = 100.0

        self.get_logger().info(f'    [{label}] plan+execute...')
        future = self.plan_client.call_async(req)
        if not self._wait_future(future, 15.0):
            self.get_logger().error(f'    [{label}] timeout')
            return False
        if future.exception():
            self.get_logger().error(f'    [{label}] exception: {future.exception()}')
            return False
        r = future.result()
        if r.success and r.executed:
            time.sleep(self.grasp_move_duration + 0.5)
            return True
        if r.success:
            return True
        self.get_logger().error(f'    [{label}] failed: {r.message}')
        return False

    def _set_gripper(self, pos):
        msg = Float64MultiArray()
        msg.data = [pos]
        for _ in range(10):
            self.gripper_pub.publish(msg)
            time.sleep(0.05)
        time.sleep(0.5)

    def _close_gripper_firm(self):
        msg = Float64MultiArray()
        msg.data = [1.0]
        for _ in range(self.GRIPPER_CLOSE_REPEATS):
            self.gripper_pub.publish(msg)
            time.sleep(0.05)
        time.sleep(self.GRIPPER_CLOSE_WAIT)
        for _ in range(10):
            self.gripper_pub.publish(msg)
            time.sleep(0.05)

    def _stow_arm(self, duration=3.0):
        self.get_logger().info('Stowing arm...')
        msg = ArmCommand()
        msg.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.duration = duration
        self.arm_pub.publish(msg)
        time.sleep(duration + 0.5)

    # ------------------------------------------------------------------
    # RETURN: drive back to home pose
    # ------------------------------------------------------------------
    def _do_return(self):
        dx = self.home_x - self.current_x
        dy = self.home_y - self.current_y
        dist = np.hypot(dx, dy)
        angle_err = wrap_angle(self.home_theta - self.current_theta)

        if dist < self.RETURN_POS_TOL and abs(angle_err) < self.RETURN_ANG_TOL:
            self.get_logger().info('Arrived home.')
            if getattr(self, 'grasp_success', False):
                self.get_logger().info('Releasing object and stowing arm.')
                self._set_gripper(0.0)
                time.sleep(0.5)
                self._stow_arm()
            self.state = TaskState.DONE
            self._hold()
            self.get_logger().info('Task complete.')
            return

        self._send_base(self.home_x, self.home_y, self.home_theta)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = BananaPickupNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
