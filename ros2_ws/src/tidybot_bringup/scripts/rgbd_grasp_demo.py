#!/usr/bin/env python3
"""
RGB-D Grasp Demo

Detects colored objects using HSV segmentation, estimates 3D position
via depth back-projection + TF2, and executes a top-down pick-and-place grasp.

Usage:
    # Terminal 1: Start simulation with grasp scene
    ros2 launch tidybot_bringup sim.launch.py scene:=scene_rgbd_grasp.xml

    # Terminal 2: Run this demo (defaults to red cube, right arm)
    ros2 run tidybot_bringup rgbd_grasp_demo.py

    # Or target a specific color/arm:
    ros2 run tidybot_bringup rgbd_grasp_demo.py --ros-args -p color:=green -p arm:=left
"""

import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray, ColorRGBA
from visualization_msgs.msg import Marker
from tidybot_msgs.msg import ArmCommand
from tidybot_msgs.srv import PlanToTarget

from tidybot_perception.rgbd_object_detector import RGBDObjectDetector


# Fingers-down orientation (wxyz) for top-down grasp in base_link
ORIENT_FINGERS_DOWN = (0.5, 0.5, 0.5, -0.5)

# Grasp geometry
PRE_GRASP_Z_OFFSET = 0.10   # 10 cm above grasp for approach
GRASP_Z_OFFSET = 0.00       # offset from detected surface (camera sees object top)
LIFT_HEIGHT = 0.15           # lift 15 cm after grasping


class RGBDGraspDemo(Node):
    """Self-contained node for detect → grasp → place pipeline."""

    def __init__(self):
        super().__init__('rgbd_grasp_demo')

        # Parameters
        self.declare_parameter('color', 'red')
        self.declare_parameter('arm', 'right')
        self.declare_parameter('camera_tilt', 0.5)
        self.declare_parameter('duration', 2.0)

        self.target_color = self.get_parameter('color').value
        self.arm_name = self.get_parameter('arm').value
        self.camera_tilt = self.get_parameter('camera_tilt').value
        self.move_duration = self.get_parameter('duration').value

        # Perception (subscribes to camera topics + sets up TF2)
        self.detector = RGBDObjectDetector(self)

        # Publishers
        self.pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, f'/{self.arm_name}_gripper/cmd', 10)

        # Arm command publisher (for returning to home via smooth interpolation)
        self.arm_pub = self.create_publisher(
            ArmCommand, f'/{self.arm_name}_arm/cmd', 10)

        # Debug marker publisher (visible in RViz)
        self.marker_pub = self.create_publisher(Marker, '/grasp_debug_markers', 10)
        self.marker_id = 0

        # Service client
        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')

    # ── Utilities ───────────────────────────────────────────────────

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
        """Smoothly return arm to home position over the given duration."""
        self.get_logger().info(f'Returning arm to home over {duration}s...')
        msg = ArmCommand()
        msg.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.duration = duration
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
        m.lifetime.sec = 60  # persist for 60s
        if label:
            m.text = label
        self.marker_pub.publish(m)
        self.get_logger().info(
            f'  Published marker "{label}" at ({x:.3f}, {y:.3f}, {z:.3f})')

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

    # ── Pose helpers ────────────────────────────────────────────────

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

    # ── Main pipeline ──────────────────────────────────────────────

    def run(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info('RGB-D Grasp Demo')
        self.get_logger().info(f'  Target color : {self.target_color}')
        self.get_logger().info(f'  Arm          : {self.arm_name}')
        self.get_logger().info(f'  Camera tilt  : {self.camera_tilt}')
        self.get_logger().info('=' * 50)

        # Wait for motion planner
        self.get_logger().info('Waiting for /plan_to_target service...')
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('  ...waiting')
        self.get_logger().info('Service available.')

        # ── Step 1: Point camera down ───────────────────────────────
        self.get_logger().info('Step 1: Setting camera tilt')
        self.set_camera(pan=0.0, tilt=self.camera_tilt)
        self.spin_for(2.0)  # let images arrive

        # ── Step 2: Detect and localize object ──────────────────────
        self.get_logger().info(f'Step 2: Detecting {self.target_color} object')
        result = self.detector.detect_and_localize(self.target_color)
        if result is None:
            self.get_logger().error(
                f'Could not find {self.target_color} object in camera view!')
            return
        (u, v), base_pt = result
        ox = base_pt.point.x
        oy = base_pt.point.y
        oz = base_pt.point.z
        self.get_logger().info(f'  Detected at pixel ({u}, {v})')
        self.get_logger().info(
            f'  Object in base_link: ({ox:.3f}, {oy:.3f}, {oz:.3f})')

        # ── Publish debug markers in RViz ───────────────────────────
        grasp_z = oz + GRASP_Z_OFFSET
        pre_grasp_z = grasp_z + PRE_GRASP_Z_OFFSET
        self.publish_marker(ox, oy, oz, 0.0, 1.0, 0.0,
                            label='detected_surface', scale=0.02)
        self.publish_marker(ox, oy, grasp_z, 1.0, 0.0, 0.0,
                            label='grasp_target', scale=0.02)
        self.publish_marker(ox, oy, pre_grasp_z, 0.0, 0.0, 1.0,
                            label='pre_grasp', scale=0.02)

        # ── Step 3: Open gripper ────────────────────────────────────
        self.get_logger().info('Step 3: Opening gripper')
        self.set_gripper(0.0)

        # ── Step 4: Move to pre-grasp ───────────────────────────────
        self.get_logger().info('Step 4: Moving to pre-grasp')
        pre_grasp = self.grasp_pose_at(ox, oy, pre_grasp_z)
        if not self.plan_and_execute(pre_grasp, 'pre-grasp'):
            self.get_logger().error('Pre-grasp motion failed')
            return

        # ── Step 5: Move to grasp ───────────────────────────────────
        self.get_logger().info('Step 5: Descending to grasp')
        grasp = self.grasp_pose_at(ox, oy, grasp_z)
        if not self.plan_and_execute(grasp, 'grasp'):
            self.get_logger().error('Grasp motion failed')
            return

        # ── Step 6: Close gripper ───────────────────────────────────
        self.get_logger().info('Step 6: Closing gripper')
        self.set_gripper(1.0)
        time.sleep(1.0)

        # ── Step 7: Lift ────────────────────────────────────────────
        self.get_logger().info('Step 7: Lifting object')
        lift = self.grasp_pose_at(ox, oy, grasp_z + LIFT_HEIGHT)
        if not self.plan_and_execute(lift, 'lift'):
            self.get_logger().error('Lift motion failed')
            return

        # ── Step 8: Place back down ─────────────────────────────────
        self.get_logger().info('Step 8: Placing object back down')
        place = self.grasp_pose_at(ox, oy, grasp_z)
        if not self.plan_and_execute(place, 'place-down'):
            self.get_logger().error('Place motion failed')
            return

        # ── Step 9: Release ─────────────────────────────────────────
        self.get_logger().info('Step 9: Releasing object')
        self.set_gripper(0.0)
        time.sleep(0.5)

        # ── Step 10: Retract ────────────────────────────────────────
        self.get_logger().info('Step 10: Retracting arm')
        retract = self.grasp_pose_at(ox, oy, pre_grasp_z)
        self.plan_and_execute(retract, 'retract')

        # ── Step 11: Return to home ─────────────────────────────────
        self.get_logger().info('Step 11: Returning to home')
        self.go_home()

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Grasp demo complete!')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = RGBDGraspDemo()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
