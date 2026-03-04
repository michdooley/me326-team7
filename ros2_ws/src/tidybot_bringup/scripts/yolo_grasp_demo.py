#!/usr/bin/env python3
"""
YOLO Grasp Demo — Object Detection via Classifier + Geometric Grasp Planning

Detects objects using the external YOLO classifier node (/objbbox),
estimates 3D position via depth back-projection + TF2, and executes
a top-down grasp using the geometric grasp planner (/plan_grasp).

Requires:
    - ros2 launch tidybot_bringup sim.launch.py scene:=scene_fruit_grasp.xml
    - ros2 run object_classification classifier

Usage:
    # Default: grasp a banana with right arm
    ros2 run tidybot_bringup yolo_grasp_demo.py

    # Target a specific object/arm:
    ros2 run tidybot_bringup yolo_grasp_demo.py --ros-args \
        -p target:=apple -p arm:=right -p camera_tilt:=0.5
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import tf2_ros

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from tidybot_msgs.msg import ArmCommand
from tidybot_msgs.srv import PlanToTarget, PlanGrasp
from cv_bridge import CvBridge

from tidybot_perception.coord_converter import CoordConverter

# YOLO COCO class name → string ID (must match classifier.py output)
YOLO_CLASS_IDS = {
    'person': '0',
    'banana': '46',
    'apple': '47',
    'orange': '49',
}

# Fingers-down orientation (wxyz) for top-down grasp
ORIENT_FINGERS_DOWN = (0.5, 0.5, 0.5, -0.5)

# Grasp geometry
PRE_GRASP_Z_OFFSET = 0.10
GRASP_Z_OFFSET = 0.00
LIFT_HEIGHT = 0.15

# Verification
MAX_GRASP_RETRIES = 3
VERIFY_ARM_POSE = [0.0, 0.4, -0.3, 0.0, 0.5, 0.0]
VERIFY_ARM_DURATION = 2.5
VERIFY_CAMERA_PAN = 0.0
VERIFY_CAMERA_TILT = 0.3
VERIFY_CAMERA_SETTLE_TIME = 1.5


class YoloGraspDemo(Node):
    """Detect via YOLO classifier, plan grasp, execute pick-and-place."""

    def __init__(self):
        super().__init__('yolo_grasp_demo')

        # Parameters
        self.declare_parameter('target', 'banana')
        self.declare_parameter('arm', 'right')
        self.declare_parameter('camera_tilt', 0.5)
        self.declare_parameter('duration', 2.0)
        self.declare_parameter('max_retries', MAX_GRASP_RETRIES)

        self.target_object = self.get_parameter('target').value
        self.arm_name = self.get_parameter('arm').value
        self.camera_tilt = self.get_parameter('camera_tilt').value
        self.move_duration = self.get_parameter('duration').value
        self.max_retries = self.get_parameter('max_retries').value

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.coord_conv = CoordConverter(self.tf_buffer)

        # CV bridge
        self._bridge = CvBridge()

        # Camera state
        self.latest_depth = None
        self.camera_info = None
        self.latest_detections = None

        # Subscribers
        qos_be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Detection2DArray, '/objbbox', self._detections_cb, 10)
        self.create_subscription(
            Image, '/camera/depth/image_raw', self._depth_cb, qos_be)
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._info_cb, 10)

        # Publishers
        self.pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, f'/{self.arm_name}_gripper/cmd', 10)
        self.arm_pub = self.create_publisher(
            ArmCommand, f'/{self.arm_name}_arm/cmd', 10)
        self.marker_pub = self.create_publisher(Marker, '/grasp_debug_markers', 10)
        self.marker_id = 0

        # Service clients
        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')
        self.grasp_planner_client = self.create_client(PlanGrasp, '/plan_grasp')
        self.grasp_planner_available = False

    # ── Callbacks ─────────────────────────────────────────────────────

    def _detections_cb(self, msg):
        self.latest_detections = msg

    def _depth_cb(self, msg):
        self.latest_depth = self._bridge.imgmsg_to_cv2(msg, '16UC1')

    def _info_cb(self, msg):
        self.camera_info = msg

    # ── Utilities ─────────────────────────────────────────────────────

    def spin_for(self, seconds):
        """Spin to receive messages for the given duration."""
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def set_camera(self, pan=0.0, tilt=0.65):
        """Publish camera pan/tilt and wait for it to settle."""
        msg = Float64MultiArray()
        msg.data = [pan, tilt]
        for _ in range(5):
            self.pan_tilt_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.spin_for(1.5)

    def set_gripper(self, position):
        """Set gripper (0.0 = open, 1.0 = closed)."""
        msg = Float64MultiArray()
        msg.data = [position]
        for _ in range(10):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(0.5)

    def go_home(self, duration=3.0):
        """Smoothly return arm to home position."""
        self.get_logger().info(f'Returning arm to home over {duration}s...')
        msg = ArmCommand()
        msg.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.duration = duration
        self.arm_pub.publish(msg)
        time.sleep(duration + 0.5)

    def set_arm_joints(self, joint_positions, duration):
        """Publish an ArmCommand to the active arm and wait."""
        msg = ArmCommand()
        msg.joint_positions = list(joint_positions)
        msg.duration = float(duration)
        self.arm_pub.publish(msg)
        time.sleep(duration + 0.5)

    def publish_marker(self, x, y, z, r, g, b, label='', scale=0.03):
        """Publish a sphere marker in base_link for RViz debugging."""
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'grasp_debug'
        m.id = self.marker_id
        self.marker_id += 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.pose.orientation.w = 1.0
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        m.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=0.8)
        m.lifetime.sec = 60
        if label:
            m.text = label
        self.marker_pub.publish(m)

    # ── YOLO detection ────────────────────────────────────────────────

    def detect(self, target_class):
        """Find the best YOLO detection matching target_class from /objbbox.

        Returns:
            (u, v) pixel centroid of highest-confidence match, or None.
        """
        if self.latest_detections is None:
            return None
        target_id = YOLO_CLASS_IDS.get(target_class.lower())
        if target_id is None:
            self.get_logger().warn(
                f'Unknown target class "{target_class}". '
                f'Available: {list(YOLO_CLASS_IDS.keys())}')
            return None

        best, best_conf = None, 0.0
        for det in self.latest_detections.detections:
            for hyp in det.results:
                if (hyp.hypothesis.class_id == target_id
                        and hyp.hypothesis.score > best_conf):
                    best = (int(det.bbox.center.position.x),
                            int(det.bbox.center.position.y))
                    best_conf = hyp.hypothesis.score
        return best

    def detect_and_localize(self, target_class):
        """Detect an object and return its 3D position in base_link.

        Returns:
            ((u, v), PointStamped_in_base_link), or None.
        """
        centroid = self.detect(target_class)
        if centroid is None:
            return None

        base_pt = self.coord_conv.pixel_to_base_link(
            centroid[0], centroid[1],
            self.latest_depth, self.camera_info, self.get_clock())
        if base_pt is None:
            return None

        return (centroid, base_pt)

    # ── Motion planning ───────────────────────────────────────────────

    def plan_and_execute(self, pose, label, use_orientation=True):
        """Call /plan_to_target synchronously. Returns True on success."""
        req = PlanToTarget.Request()
        req.arm_name = self.arm_name
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = True
        req.duration = self.move_duration
        req.max_condition_number = 100.0

        self.get_logger().info(
            f'  [{label}] pos=({pose.position.x:.3f}, {pose.position.y:.3f}, '
            f'{pose.position.z:.3f})')

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

        if not future.done() or future.exception():
            self.get_logger().error(f'  [{label}] service call failed')
            return False

        result = future.result()
        if result.success:
            self.get_logger().info(
                f'  [{label}] OK  pos_err={result.position_error:.4f}m')
            if result.executed:
                time.sleep(self.move_duration + 0.5)
            return True
        else:
            self.get_logger().warn(f'  [{label}] FAILED: {result.message}')
            return False

    # ── Pose helpers ──────────────────────────────────────────────────

    @staticmethod
    def make_pose(x, y, z, qw, qx, qy, qz):
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = float(qw)
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        return pose

    def grasp_pose_at(self, x, y, z):
        """Fingers-down pose at the given base_link position."""
        qw, qx, qy, qz = ORIENT_FINGERS_DOWN
        return self.make_pose(x, y, z, qw, qx, qy, qz)

    # ── Post-grasp verification ───────────────────────────────────────

    def verify_grasp(self) -> bool:
        """Present gripper to camera and check YOLO sees the target."""
        self.get_logger().info('  [verify] Moving arm to present pose...')
        self.set_arm_joints(VERIFY_ARM_POSE, VERIFY_ARM_DURATION)

        self.get_logger().info('  [verify] Aiming camera at gripper...')
        self.set_camera(pan=VERIFY_CAMERA_PAN, tilt=VERIFY_CAMERA_TILT)
        self.spin_for(VERIFY_CAMERA_SETTLE_TIME)

        self.get_logger().info(
            f'  [verify] Checking for "{self.target_object}" via YOLO...')
        centroid = self.detect(self.target_object)

        if centroid is not None:
            u, v = centroid
            self.get_logger().info(
                f'  [verify] Object detected at pixel ({u}, {v}) — grasp confirmed')
            return True
        else:
            self.get_logger().warn(
                f'  [verify] Target "{self.target_object}" not found — grasp failed')
            return False

    # ── Main pipeline ─────────────────────────────────────────────────

    def _detect_object(self):
        """Point camera at table and localize the target via YOLO."""
        self.get_logger().info('Setting camera tilt')
        self.set_camera(pan=0.0, tilt=self.camera_tilt)
        self.spin_for(2.0)

        self.get_logger().info(f'Detecting "{self.target_object}" via YOLO classifier')

        # Poll for detection (YOLO may need a few frames)
        for _ in range(20):
            result = self.detect_and_localize(self.target_object)
            if result is not None:
                break
            self.spin_for(0.2)

        if result is None:
            self.get_logger().error(
                f'Could not find "{self.target_object}" in camera view!')
            return None

        (u, v), base_pt = result
        ox, oy, oz = base_pt.point.x, base_pt.point.y, base_pt.point.z
        self.get_logger().info(f'  Detected at pixel ({u}, {v})')
        self.get_logger().info(
            f'  Object in base_link: ({ox:.3f}, {oy:.3f}, {oz:.3f})')
        return (ox, oy, oz, base_pt)

    def _call_plan_grasp(self, base_pt):
        """Call /plan_grasp service. Returns PlanGrasp.Response or None."""
        if not self.grasp_planner_available:
            return None

        req = PlanGrasp.Request()
        req.object_position = base_pt
        req.object_class = self.target_object
        req.arm_name = self.arm_name

        self.get_logger().info('  Calling /plan_grasp for geometric grasp...')
        future = self.grasp_planner_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if not future.done() or future.exception():
            self.get_logger().warn('  /plan_grasp call failed')
            return None

        result = future.result()
        if result.success:
            self.get_logger().info(f'  Grasp planner: {result.message}')
        else:
            self.get_logger().warn(f'  Grasp planner failed: {result.message}')
        return result

    def _execute_grasp(self, ox, oy, oz, base_pt=None):
        """Run pre-grasp → grasp → close sequence. Returns True on success."""
        grasp_result = None
        if base_pt is not None:
            grasp_result = self._call_plan_grasp(base_pt)

        if grasp_result is not None and grasp_result.success:
            pre_grasp_pose = grasp_result.pre_grasp_pose
            grasp_pose = grasp_result.grasp_pose
            self.get_logger().info('  Using geometric grasp planner')
        else:
            grasp_z = oz + GRASP_Z_OFFSET
            pre_grasp_z = grasp_z + PRE_GRASP_Z_OFFSET
            pre_grasp_pose = self.grasp_pose_at(ox, oy, pre_grasp_z)
            grasp_pose = self.grasp_pose_at(ox, oy, grasp_z)
            self.get_logger().info('  Using default fingers-down orientation')

        # Debug markers
        gp = grasp_pose.position
        pp = pre_grasp_pose.position
        self.publish_marker(ox, oy, oz, 0.0, 1.0, 0.0,
                            label='detected_surface', scale=0.02)
        self.publish_marker(gp.x, gp.y, gp.z, 1.0, 0.0, 0.0,
                            label='grasp_target', scale=0.02)
        self.publish_marker(pp.x, pp.y, pp.z, 0.0, 0.0, 1.0,
                            label='pre_grasp', scale=0.02)

        self.get_logger().info('Opening gripper')
        self.set_gripper(0.0)

        self.get_logger().info('Moving to pre-grasp')
        if not self.plan_and_execute(pre_grasp_pose, 'pre-grasp'):
            self.get_logger().error('Pre-grasp motion failed')
            return False

        self.get_logger().info('Descending to grasp')
        if not self.plan_and_execute(grasp_pose, 'grasp'):
            self.get_logger().error('Grasp motion failed')
            return False

        self.get_logger().info('Closing gripper')
        self.set_gripper(1.0)
        time.sleep(1.0)
        return True

    def run(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info('YOLO Grasp Demo (geometric planner)')
        self.get_logger().info(f'  Target       : {self.target_object}')
        self.get_logger().info(f'  Arm          : {self.arm_name}')
        self.get_logger().info(f'  Camera tilt  : {self.camera_tilt}')
        self.get_logger().info(f'  Max retries  : {self.max_retries}')
        self.get_logger().info('=' * 50)

        # Wait for motion planner
        self.get_logger().info('Waiting for /plan_to_target service...')
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('  ...waiting')
        self.get_logger().info('Service available.')

        # Check for grasp planner
        self.get_logger().info('Checking for /plan_grasp service (3s timeout)...')
        self.grasp_planner_available = \
            self.grasp_planner_client.wait_for_service(timeout_sec=3.0)
        if self.grasp_planner_available:
            self.get_logger().info(
                'Geometric grasp planner available — using optimized grasps')
        else:
            self.get_logger().warn(
                'Grasp planner not available — using default orientation')

        # Wait for YOLO detections to start flowing
        self.get_logger().info('Waiting for YOLO classifier (/objbbox)...')
        for i in range(50):
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_detections is not None:
                break
        if self.latest_detections is None:
            self.get_logger().warn(
                'No YOLO detections received — is classifier running? '
                '(ros2 run object_classification classifier)')

        # ── Grasp + verify loop ───────────────────────────────────────
        grasp_confirmed = False
        for attempt in range(1, self.max_retries + 1):
            self.get_logger().info('')
            self.get_logger().info(f'=== Grasp attempt {attempt}/{self.max_retries} ===')

            detection = self._detect_object()
            if detection is None:
                self.get_logger().error('Object detection failed — aborting')
                return
            ox, oy, oz, base_pt = detection

            if not self._execute_grasp(ox, oy, oz, base_pt):
                self.get_logger().error('Grasp execution failed — aborting')
                return

            self.get_logger().info('Verifying grasp...')
            if self.verify_grasp():
                grasp_confirmed = True
                break

            self.get_logger().warn(
                f'Grasp verification failed (attempt {attempt}/{self.max_retries})')
            self.set_gripper(0.0)
            time.sleep(0.5)
            self.go_home()

        if not grasp_confirmed:
            self.get_logger().error(
                f'Grasp failed after {self.max_retries} attempts — giving up')
            self.go_home()
            return

        # ── Grasp confirmed — lift and place ──────────────────────────
        self.get_logger().info('')
        self.get_logger().info('Grasp confirmed — proceeding to lift')

        grasp_z = oz + GRASP_Z_OFFSET
        pre_grasp_z = grasp_z + PRE_GRASP_Z_OFFSET

        self.get_logger().info('Lifting object')
        lift = self.grasp_pose_at(ox, oy, grasp_z + LIFT_HEIGHT)
        if not self.plan_and_execute(lift, 'lift'):
            self.get_logger().error('Lift motion failed')
            return

        self.get_logger().info('Placing object back down')
        place = self.grasp_pose_at(ox, oy, grasp_z)
        if not self.plan_and_execute(place, 'place-down'):
            self.get_logger().error('Place motion failed')
            return

        self.get_logger().info('Releasing object')
        self.set_gripper(0.0)
        time.sleep(0.5)

        self.get_logger().info('Retracting arm')
        retract = self.grasp_pose_at(ox, oy, pre_grasp_z)
        self.plan_and_execute(retract, 'retract')

        self.go_home()

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('YOLO Grasp Demo complete!')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = YoloGraspDemo()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
