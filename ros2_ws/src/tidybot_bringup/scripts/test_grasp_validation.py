#!/usr/bin/env python3
"""
Grasp Validation Test

Standalone script to test and tune grasp validation logic.
Moves the arm to a "verify" pose, points the camera forward,
and checks whether YOLO detects the target object in the expected
region of the image (upper portion = held by gripper).

Designed to run with the scene_grasp_validation.xml MuJoCo scene,
which has a banana welded to the right gripper for testing.

Usage:
    # Terminal 1: Launch sim with validation scene
    ros2 launch tidybot_bringup sim.launch.py scene:=scene_grasp_validation.xml

    # Terminal 2: Start YOLO classifier
    ros2 run object_classification classifier

    # Terminal 3: Run this test
    ros2 run tidybot_bringup test_grasp_validation.py

    # With custom parameters:
    ros2 run tidybot_bringup test_grasp_validation.py --ros-args \
        -p target:=apple -p arm:=right -p v_threshold:=320
"""

import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float64MultiArray
from tidybot_msgs.msg import ArmCommand
from cv_bridge import CvBridge

# YOLO COCO class name -> string ID
YOLO_CLASS_IDS = {
    'person': '0',
    'banana': '46',
    'apple': '47',
    'orange': '49',
}

# Arm pose that presents the held object in front of the camera
# [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]
# Lower and more centered so the object appears in the camera frame
VERIFY_ARM_POSE = [0.3, -0.2, 0.7, 0.0, 0.2, 0.0]
VERIFY_ARM_DURATION = 2.5

# Camera angles: negative tilt looks upward
VERIFY_CAMERA_PAN = 0.0
VERIFY_CAMERA_TILT = -0.6
VERIFY_SETTLE_TIME = 2.0

# Image dimensions
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480


class TestGraspValidation(Node):
    """Test grasp validation by checking YOLO detection position."""

    def __init__(self):
        super().__init__('test_grasp_validation')

        # Parameters
        self.declare_parameter('target', 'banana')
        self.declare_parameter('arm', 'right')
        self.declare_parameter('v_threshold', 320.0)
        self.target_object = self.get_parameter('target').value
        self.arm_name = self.get_parameter('arm').value
        self.v_threshold = self.get_parameter('v_threshold').value

        # State
        self.latest_detections = None
        self.latest_rgb = None
        self._bridge = CvBridge()

        # Subscribers
        self.create_subscription(
            Detection2DArray, '/objbbox', self._detections_cb, 10)
        self.create_subscription(
            Image, '/camera/color/image_raw', self._rgb_cb, 10)

        # Publishers
        self.arm_pub = self.create_publisher(
            ArmCommand, f'/{self.arm_name}_arm/cmd', 10)
        self.pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, f'/{self.arm_name}_gripper/cmd', 10)

    # -- Callbacks ---------------------------------------------------------

    def _detections_cb(self, msg):
        self.latest_detections = msg

    def _rgb_cb(self, msg):
        self.latest_rgb = msg

    # -- Utilities ---------------------------------------------------------

    def spin_for(self, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def set_camera(self, pan, tilt):
        """Publish camera pan/tilt and wait for it to settle."""
        msg = Float64MultiArray()
        msg.data = [pan, tilt]
        for _ in range(5):
            self.pan_tilt_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.spin_for(1.5)

    def set_arm_joints(self, positions, duration):
        """Publish ArmCommand and wait for motion to complete."""
        msg = ArmCommand()
        msg.joint_positions = list(positions)
        msg.duration = float(duration)
        self.arm_pub.publish(msg)
        time.sleep(duration + 0.5)

    def set_gripper(self, position):
        """Set gripper (0.0 = open, 1.0 = closed)."""
        msg = Float64MultiArray()
        msg.data = [position]
        for _ in range(10):
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(0.5)

    def detect(self, target_class):
        """Find the best YOLO detection matching target_class."""
        if self.latest_detections is None:
            return None
        target_id = YOLO_CLASS_IDS.get(target_class.lower())
        if target_id is None:
            return None

        best, best_conf = None, 0.0
        for det in self.latest_detections.detections:
            for hyp in det.results:
                if (hyp.hypothesis.class_id == target_id
                        and hyp.hypothesis.score > best_conf):
                    best = det
                    best_conf = hyp.hypothesis.score
        return best

    # -- Validation logic --------------------------------------------------

    def verify_grasp(self):
        """Move to verify pose, check if YOLO detects object in upper frame.

        Returns True if object detected in expected region, False otherwise.
        """
        self.get_logger().info('Moving arm to verify pose...')
        self.set_arm_joints(VERIFY_ARM_POSE, VERIFY_ARM_DURATION)

        self.get_logger().info(
            f'Pointing camera (pan={VERIFY_CAMERA_PAN}, '
            f'tilt={VERIFY_CAMERA_TILT})...')
        self.set_camera(VERIFY_CAMERA_PAN, VERIFY_CAMERA_TILT)

        self.get_logger().info(
            f'Waiting {VERIFY_SETTLE_TIME}s for camera to settle...')
        self.spin_for(VERIFY_SETTLE_TIME)

        # Try multiple detection attempts
        det = None
        for attempt in range(10):
            det = self.detect(self.target_object)
            if det is not None:
                break
            self.spin_for(0.3)

        if det is None:
            self.get_logger().warn(
                f'No "{self.target_object}" detected in camera view')
            self.get_logger().warn('RESULT: FAIL — object not visible')
            return False

        u = det.bbox.center.position.x
        v = det.bbox.center.position.y
        w = det.bbox.size_x
        h = det.bbox.size_y
        conf = det.results[0].hypothesis.score if det.results else 0.0

        self.get_logger().info(
            f'Detection: center=({u:.0f}, {v:.0f}), '
            f'size={w:.0f}x{h:.0f}, conf={conf:.2f}')

        if v < self.v_threshold:
            self.get_logger().info(
                f'RESULT: PASS — v={v:.0f} < threshold={self.v_threshold:.0f} '
                f'(object in upper frame = held by gripper)')
            return True
        else:
            self.get_logger().warn(
                f'RESULT: FAIL — v={v:.0f} >= threshold={self.v_threshold:.0f} '
                f'(object in lower frame = likely dropped)')
            return False

    # -- Main --------------------------------------------------------------

    def run(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info('Grasp Validation Test')
        self.get_logger().info(f'  Target      : {self.target_object}')
        self.get_logger().info(f'  Arm         : {self.arm_name}')
        self.get_logger().info(f'  V threshold : {self.v_threshold}')
        self.get_logger().info('=' * 50)

        # Wait for camera images
        self.get_logger().info('Waiting for camera images...')
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_rgb is not None:
                break
        if self.latest_rgb is None:
            self.get_logger().error('No camera images received — aborting')
            return

        # Wait for YOLO detections
        self.get_logger().info('Waiting for YOLO classifier (/objbbox)...')
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_detections is not None:
                break
        if self.latest_detections is None:
            self.get_logger().warn(
                'No YOLO detections yet — classifier may not be running')

        # Close gripper (in case scene doesn't start closed)
        self.get_logger().info('Closing gripper...')
        self.set_gripper(1.0)

        # Run validation
        self.get_logger().info('')
        result = self.verify_grasp()

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        if result:
            self.get_logger().info('Grasp validation: PASSED')
        else:
            self.get_logger().info('Grasp validation: FAILED')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = TestGraspValidation()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
