#!/usr/bin/env python3
"""
Grasp Planner Node — GR-ConvNet Based

Uses GR-ConvNet (Generative Residual Convolutional Neural Network) to predict
optimal antipodal grasps from depth images for a top-down parallel jaw gripper.

Pipeline:
    1. Receive /plan_grasp request with object 3D position
    2. Project object position to image pixel via TF2
    3. Crop depth image around object, resize to 224x224
    4. Run GR-ConvNet inference (CPU) → quality, angle, width maps
    5. Find best grasp near object centroid
    6. Convert pixel-space grasp to 3D base_link pose
    7. Validate reachability via /plan_to_target
    8. Return grasp_pose + pre_grasp_pose

Services provided:
    /plan_grasp (tidybot_msgs/PlanGrasp)

Services used:
    /plan_to_target (tidybot_msgs/PlanToTarget) — IK reachability check

Parameters:
    model_path (str): Path to GR-ConvNet weights file
    approach_offset (float): Pre-grasp height above grasp (default 0.10m)
    default_arm (str): Arm when "auto" requested (default "right")
    grasp_z_offset (float): Z offset from detected surface (default 0.0)
    crop_size (int): Crop region size in pixels (default 300)

Usage:
    ros2 run tidybot_perception grasp_planner_node \\
        --ros-args -p model_path:=/path/to/grconvnet_cornell.pt
"""

import os
import numpy as np
import cv2

import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import tf2_ros
import tf2_geometry_msgs  # noqa: F401

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped
from cv_bridge import CvBridge

from tidybot_msgs.srv import PlanGrasp, PlanToTarget

from tidybot_perception.grconvnet import GenerativeResnet, post_process_output, detect_grasps
from tidybot_perception.grasp_geometry import (
    yaw_to_grasp_quaternion,
    image_angle_to_base_link_yaw,
    grasp_width_pixels_to_meters,
    GRIPPER_MAX_OPENING_M,
)
from tidybot_perception.rgbd_object_detector import RGBDObjectDetector

# Default orientation (no yaw) for fallback
ORIENT_FINGERS_DOWN = (0.5, 0.5, 0.5, -0.5)

# GR-ConvNet input size
GRCONVNET_INPUT_SIZE = 224


class GraspPlannerNode(Node):
    """GR-ConvNet-based grasp planning node."""

    def __init__(self):
        super().__init__('grasp_planner_node')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('approach_offset', 0.10)
        self.declare_parameter('default_arm', 'right')
        self.declare_parameter('grasp_z_offset', 0.0)
        self.declare_parameter('crop_size', 300)

        self.model_path = self.get_parameter('model_path').value
        self.approach_offset = self.get_parameter('approach_offset').value
        self.default_arm = self.get_parameter('default_arm').value
        self.grasp_z_offset = self.get_parameter('grasp_z_offset').value
        self.crop_size = self.get_parameter('crop_size').value

        # Perception (subscribes to camera topics + TF2)
        self.detector = RGBDObjectDetector(self)

        # Load GR-ConvNet model
        self.model = None
        self.input_channels = 1
        self.device = torch.device('cpu')
        self._load_model()

        # Service
        self.grasp_srv = self.create_service(
            PlanGrasp, '/plan_grasp', self.plan_grasp_callback)

        # Client for IK reachability validation
        self.plan_to_target_client = self.create_client(
            PlanToTarget, '/plan_to_target')

        self.get_logger().info('GraspPlannerNode initialized (GR-ConvNet)')
        self.get_logger().info(f'  model_path: {self.model_path}')
        self.get_logger().info(f'  approach_offset: {self.approach_offset}m')
        self.get_logger().info(f'  default_arm: {self.default_arm}')
        self.get_logger().info(f'  crop_size: {self.crop_size}px')

    # ── Model loading ────────────────────────────────────────────────

    def _load_model(self):
        """Load GR-ConvNet pretrained weights."""
        if not self.model_path:
            self.get_logger().warn(
                'No model_path set — grasp planning will use fallback orientation')
            return

        if not os.path.isfile(self.model_path):
            self.get_logger().error(f'Model file not found: {self.model_path}')
            return

        try:
            self.get_logger().info(f'Loading GR-ConvNet from {self.model_path}...')
            state_dict = torch.load(
                self.model_path, map_location=self.device, weights_only=True)

            # Detect input channels from weights
            input_channels = state_dict['conv1.weight'].shape[1]
            self.input_channels = input_channels

            self.model = GenerativeResnet(input_channels=input_channels)
            self.model.load_state_dict(state_dict)
            self.model.to(self.device)
            self.model.eval()
            self.get_logger().info(
                f'GR-ConvNet loaded ({input_channels}ch, '
                f'{sum(p.numel() for p in self.model.parameters())} params)')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            self.model = None
            self.input_channels = 1

    # ── Depth preprocessing ──────────────────────────────────────────

    def _preprocess_rgbd(self, depth_img, rgb_img, center_u, center_v):
        """
        Crop and resize RGBD image around the object for GR-ConvNet.

        Args:
            depth_img: (H, W) uint16 depth image in mm
            rgb_img: (H, W, 3) uint8 RGB image (or None for depth-only)
            center_u, center_v: Pixel center of object

        Returns:
            (input_tensor, crop_info) where:
                input_tensor: (1, C, 224, 224) float32 tensor (C=4 for RGBD, 1 for depth)
                crop_info: dict with crop bounds for mapping back
        """
        h, w = depth_img.shape
        half = self.crop_size // 2

        # Compute crop bounds, clamped to image edges
        u_lo = max(0, center_u - half)
        u_hi = min(w, center_u + half)
        v_lo = max(0, center_v - half)
        v_hi = min(h, center_v + half)

        # Crop depth
        depth_crop = depth_img[v_lo:v_hi, u_lo:u_hi].copy()

        # Convert to float32 meters and inpaint zeros
        depth_f = depth_crop.astype(np.float32) / 1000.0
        invalid = (depth_f < 0.1) | (depth_f > 5.0)
        if invalid.any():
            valid_vals = depth_f[~invalid]
            if len(valid_vals) > 0:
                depth_f[invalid] = np.median(valid_vals)
            else:
                depth_f[invalid] = 0.5  # fallback

        # Normalize depth: zero-mean
        mean_depth = depth_f.mean()
        depth_f = depth_f - mean_depth

        # Resize depth to 224x224
        depth_resized = cv2.resize(
            depth_f, (GRCONVNET_INPUT_SIZE, GRCONVNET_INPUT_SIZE),
            interpolation=cv2.INTER_LINEAR)

        # Build input tensor
        if self.input_channels == 4 and rgb_img is not None:
            # RGBD: crop and resize RGB too
            rgb_crop = rgb_img[v_lo:v_hi, u_lo:u_hi].copy()
            rgb_resized = cv2.resize(
                rgb_crop, (GRCONVNET_INPUT_SIZE, GRCONVNET_INPUT_SIZE),
                interpolation=cv2.INTER_LINEAR)
            # Normalize RGB to [0, 1]
            rgb_f = rgb_resized.astype(np.float32) / 255.0
            # Stack: (224, 224, 4) → RGBD
            rgbd = np.dstack([rgb_f, depth_resized])
            # (H, W, 4) → (4, H, W) → (1, 4, H, W)
            tensor = torch.from_numpy(
                rgbd.transpose(2, 0, 1)).unsqueeze(0)
        else:
            # Depth-only: (1, 1, 224, 224)
            tensor = torch.from_numpy(
                depth_resized).unsqueeze(0).unsqueeze(0)

        crop_info = {
            'u_lo': u_lo, 'u_hi': u_hi,
            'v_lo': v_lo, 'v_hi': v_hi,
            'crop_w': u_hi - u_lo,
            'crop_h': v_hi - v_lo,
            'center_u': center_u,
            'center_v': center_v,
        }

        return tensor, crop_info

    def _map_grasp_to_original(self, grasp, crop_info):
        """
        Map GR-ConvNet grasp coordinates from 224x224 back to original image.

        Args:
            grasp: Grasp object with center (row, col) in 224x224 space
            crop_info: Dict from _preprocess_depth

        Returns:
            (u, v, angle, width_pixels) in original image coordinates
        """
        row, col = grasp.center

        # Scale from 224x224 back to crop size
        scale_x = crop_info['crop_w'] / GRCONVNET_INPUT_SIZE
        scale_y = crop_info['crop_h'] / GRCONVNET_INPUT_SIZE

        # Map to original image coordinates
        u = int(col * scale_x + crop_info['u_lo'])
        v = int(row * scale_y + crop_info['v_lo'])

        # Scale width from 224x224 pixels to original image pixels
        width_pixels = grasp.length * scale_x

        return u, v, grasp.angle, width_pixels

    # ── IK validation ────────────────────────────────────────────────

    def _check_ik_reachability(self, pose, arm_name, timeout=10.0):
        """
        Call /plan_to_target with execute=False to check if a pose is reachable.

        Returns:
            PlanToTarget.Response or None on failure.
        """
        if not self.plan_to_target_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/plan_to_target service not available')
            return None

        req = PlanToTarget.Request()
        req.arm_name = arm_name
        req.target_pose = pose
        req.use_orientation = True
        req.execute = False
        req.duration = 2.0
        req.max_condition_number = 100.0

        future = self.plan_to_target_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        if not future.done() or future.exception():
            return None
        return future.result()

    # ── Main service callback ────────────────────────────────────────

    def plan_grasp_callback(self, request, response):
        """Plan a grasp for the given object position."""
        arm = request.arm_name
        if not arm or arm == 'auto':
            arm = self.default_arm

        obj_pos = request.object_position
        ox = obj_pos.point.x
        oy = obj_pos.point.y
        oz = obj_pos.point.z

        self.get_logger().info(
            f'Planning grasp for "{request.object_class}" at '
            f'({ox:.3f}, {oy:.3f}, {oz:.3f}), arm={arm}')

        # Check if model is loaded
        if self.model is None:
            self.get_logger().warn('No model loaded — using fallback orientation')
            return self._fallback_grasp(ox, oy, oz, arm, response)

        # Check we have camera data
        if self.detector.latest_depth is None or self.detector.camera_info is None:
            self.get_logger().warn('No depth/camera_info available — using fallback')
            return self._fallback_grasp(ox, oy, oz, arm, response)

        # Step 1: Project object 3D position to image pixel
        pixel = self.detector.base_link_to_pixel(ox, oy, oz)
        if pixel is None:
            self.get_logger().warn('Could not project object to image — using fallback')
            return self._fallback_grasp(ox, oy, oz, arm, response)

        center_u, center_v = pixel
        self.get_logger().info(f'  Object projects to pixel ({center_u}, {center_v})')

        # Bounds check
        h, w = self.detector.latest_depth.shape
        if not (0 <= center_u < w and 0 <= center_v < h):
            self.get_logger().warn('Object pixel out of image bounds — using fallback')
            return self._fallback_grasp(ox, oy, oz, arm, response)

        # Step 2: Preprocess RGBD image
        rgb_img = self.detector.latest_rgb if hasattr(self.detector, 'latest_rgb') else None
        input_tensor, crop_info = self._preprocess_rgbd(
            self.detector.latest_depth, rgb_img, center_u, center_v)

        # Step 3: Run GR-ConvNet inference
        try:
            with torch.no_grad():
                input_tensor = input_tensor.to(self.device)
                pred = self.model.predict(input_tensor)

            q_img, ang_img, width_img = post_process_output(
                pred['pos'], pred['cos'], pred['sin'], pred['width'])
        except Exception as e:
            self.get_logger().error(f'GR-ConvNet inference failed: {e}')
            return self._fallback_grasp(ox, oy, oz, arm, response)

        # Step 4: Detect grasps (up to 5 candidates)
        grasps = detect_grasps(q_img, ang_img, width_img, no_grasps=5)

        if not grasps:
            self.get_logger().warn('No grasps detected — using fallback')
            return self._fallback_grasp(ox, oy, oz, arm, response)

        self.get_logger().info(f'  GR-ConvNet found {len(grasps)} grasp candidates')

        # Step 5: Try each grasp candidate (best quality first)
        for i, grasp in enumerate(grasps):
            u, v, angle_image, width_px = self._map_grasp_to_original(
                grasp, crop_info)

            self.get_logger().info(
                f'  Candidate {i}: pixel=({u},{v}) '
                f'angle={np.degrees(angle_image):.1f}deg '
                f'width={width_px:.1f}px '
                f'quality={q_img[grasp.center]:.3f}')

            # Back-project grasp center to 3D
            grasp_pt = self.detector.pixel_to_base_link(u, v)
            if grasp_pt is None:
                self.get_logger().info(f'    Skipping: back-projection failed')
                continue

            gx = grasp_pt.point.x
            gy = grasp_pt.point.y
            gz = grasp_pt.point.z + self.grasp_z_offset

            # Convert image angle to base_link yaw
            yaw = image_angle_to_base_link_yaw(
                angle_image, (u, v), self.detector.pixel_to_base_link)

            if yaw is None:
                self.get_logger().info(f'    Skipping: angle conversion failed')
                continue

            # Build grasp orientation
            qw, qx, qy, qz = yaw_to_grasp_quaternion(yaw)

            # Estimate grasp width in meters
            fx = self.detector.camera_info.k[0]
            depth_at_grasp = self.detector.latest_depth[
                min(v, h - 1), min(u, w - 1)]
            depth_m = float(depth_at_grasp) / 1000.0 if depth_at_grasp > 0 else 0.5
            width_m = grasp_width_pixels_to_meters(width_px, depth_m, fx)

            # Check if object fits in gripper
            if width_m > GRIPPER_MAX_OPENING_M:
                self.get_logger().info(
                    f'    Skipping: width {width_m*1000:.0f}mm > '
                    f'max {GRIPPER_MAX_OPENING_M*1000:.0f}mm')
                continue

            # Build grasp pose
            grasp_pose = Pose()
            grasp_pose.position = Point(x=float(gx), y=float(gy), z=float(gz))
            grasp_pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

            # Validate IK reachability
            ik_result = self._check_ik_reachability(grasp_pose, arm)
            if ik_result is None or not ik_result.success:
                msg = ik_result.message if ik_result else 'service error'
                self.get_logger().info(f'    Skipping: IK failed — {msg}')
                continue

            self.get_logger().info(
                f'    Selected! yaw={np.degrees(yaw):.1f}deg '
                f'width={width_m*1000:.0f}mm '
                f'IK_err={ik_result.position_error:.4f}m')

            # Build pre-grasp pose (above grasp, same orientation)
            pre_grasp_pose = Pose()
            pre_grasp_pose.position = Point(
                x=float(gx), y=float(gy),
                z=float(gz + self.approach_offset))
            pre_grasp_pose.orientation = grasp_pose.orientation

            response.success = True
            response.grasp_pose = grasp_pose
            response.pre_grasp_pose = pre_grasp_pose
            response.arm_used = arm
            response.grasp_width = float(width_m)
            response.message = (
                f'GR-ConvNet grasp: yaw={np.degrees(yaw):.1f}deg, '
                f'width={width_m*1000:.0f}mm, '
                f'quality={q_img[grasp.center]:.3f}')
            return response

        # All candidates failed IK
        self.get_logger().warn('All GR-ConvNet candidates failed — using fallback')
        return self._fallback_grasp(ox, oy, oz, arm, response)

    # ── Fallback ─────────────────────────────────────────────────────

    def _fallback_grasp(self, ox, oy, oz, arm, response):
        """Use the original hardcoded fingers-down orientation."""
        grasp_z = oz + self.grasp_z_offset
        qw, qx, qy, qz = ORIENT_FINGERS_DOWN

        grasp_pose = Pose()
        grasp_pose.position = Point(x=float(ox), y=float(oy), z=float(grasp_z))
        grasp_pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

        pre_grasp_pose = Pose()
        pre_grasp_pose.position = Point(
            x=float(ox), y=float(oy),
            z=float(grasp_z + self.approach_offset))
        pre_grasp_pose.orientation = grasp_pose.orientation

        # Validate IK
        ik_result = self._check_ik_reachability(grasp_pose, arm)
        if ik_result is not None and not ik_result.success:
            response.success = False
            response.message = f'Fallback orientation not reachable: {ik_result.message}'
            return response

        response.success = True
        response.grasp_pose = grasp_pose
        response.pre_grasp_pose = pre_grasp_pose
        response.arm_used = arm
        response.grasp_width = 0.0
        response.message = 'Fallback: default fingers-down orientation'
        self.get_logger().info(f'  {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GraspPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
