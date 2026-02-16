"""
PickUpObject subtask — Approach, grasp, and lift a detected object.

Refactored from pick_up_block.py. Uses blocking calls since the robot
is stationary during pickup and arm planning is inherently sequential.
"""

import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, Pose2D
from std_msgs.msg import Float64MultiArray
from tidybot_msgs.srv import PlanToTarget

import tf2_ros

try:
    import cv2
except ImportError:
    raise ImportError("opencv-python is required: pip install opencv-python")


class PickUpObject:
    """Approach a detected object, grasp it, and lift. Blocking execution."""

    # Fingers-down grasp orientation (quaternion wxyz in base_link frame)
    ORIENT_FINGERS_DOWN = (0.5, 0.5, 0.5, -0.5)

    # Right arm shoulder position in base_link frame
    RIGHT_SHOULDER = np.array([-0.15, -0.12, 0.45])

    # Reach limits
    MAX_REACH = 0.38
    IDEAL_REACH = 0.30

    # Default HSV ranges for red detection (widened for MuJoCo lighting)
    DEFAULT_HSV_RANGES = [
        (np.array([0, 40, 30]), np.array([20, 255, 255])),
        (np.array([160, 40, 30]), np.array([180, 255, 255])),
    ]

    def __init__(self, node):
        self.node = node
        self.success = False
        self._hsv_ranges = self.DEFAULT_HSV_RANGES

    def run_blocking(self, object_odom_pos, hsv_ranges=None):
        """Execute the full pickup sequence. Blocks until complete.

        Args:
            object_odom_pos: np.array([x, y, z]) approximate position in odom.
            hsv_ranges: Optional HSV ranges for detection. Defaults to red.
        """
        if hsv_ranges is not None:
            self._hsv_ranges = hsv_ranges

        n = self.node
        self.success = False
        qw, qx, qy, qz = self.ORIENT_FINGERS_DOWN

        # Step 0: Wait for planner
        n.get_logger().info('[PICKUP] Waiting for planner service...')
        while not n.plan_client.wait_for_service(timeout_sec=1.0):
            n.get_logger().info('[PICKUP]   Still waiting...')
        n.get_logger().info('[PICKUP] Planner ready.')

        # Step 1: Tilt camera down
        n.get_logger().info('[PICKUP] Tilting camera down...')
        self._send_pan_tilt(0.0, 0.6)
        self._spin_for(2.0)

        # Step 2: Localize object using TF (odom→base_link), with approach loop
        n.get_logger().info('[PICKUP] Localizing object via TF...')
        block_pos = None

        for attempt in range(5):
            n.get_logger().info(f'[PICKUP]   Attempt {attempt + 1}/5')

            # Transform known odom position to base_link using TF
            target = self._odom_to_base_link(object_odom_pos)
            if target is None:
                n.get_logger().warn('[PICKUP]   TF transform failed, retrying...')
                self._spin_for(0.5)
                continue

            # Try camera-based refinement (if it works, use it; otherwise keep TF result)
            camera_target = self._detect_and_localize()
            if camera_target is not None:
                # Sanity: camera result should be roughly consistent with TF
                diff = np.linalg.norm(camera_target[:2] - target[:2])
                if diff < 0.3:
                    n.get_logger().info(
                        f'[PICKUP]   Camera refinement accepted (diff={diff:.3f}m)')
                    target = camera_target
                else:
                    n.get_logger().info(
                        f'[PICKUP]   Camera result too far from TF (diff={diff:.3f}m), using TF')

            reach = self._compute_reach(target)
            n.get_logger().info(
                f'[PICKUP]   Object at ({target[0]:.3f}, {target[1]:.3f}, '
                f'{target[2]:.3f}), reach={reach:.3f}m')

            if reach <= self.MAX_REACH:
                block_pos = target
                n.get_logger().info('[PICKUP]   Within reach!')
                break

            # Drive closer
            moved = self._move_base_closer(target)
            if not moved:
                block_pos = target
                break
            self._spin_for(1.0)

        if block_pos is None:
            n.get_logger().error('[PICKUP] Could not detect/reach object!')
            self.success = False
            return

        bx, by, bz = block_pos[0], block_pos[1], block_pos[2]
        n.get_logger().info(f'[PICKUP] Final target: ({bx:.3f}, {by:.3f}, {bz:.3f})')

        # Step 3: Open gripper
        n.get_logger().info('[PICKUP] Opening gripper...')
        self._send_gripper(0.0)
        self._spin_for(1.0)

        # Step 4: Arm approach (10cm above)
        n.get_logger().info('[PICKUP] Arm approach...')
        approach_pose = self._create_pose(bx, by, bz + 0.10, qw, qx, qy, qz)
        if not self._call_planner('right', approach_pose):
            n.get_logger().error('[PICKUP] Approach failed!')
            self.success = False
            return
        self._spin_for(2.5)

        # Step 5: Descend to grasp height
        n.get_logger().info('[PICKUP] Descending to grasp...')
        grasp_pose = self._create_pose(bx, by, bz + 0.02, qw, qx, qy, qz)
        if not self._call_planner('right', grasp_pose):
            n.get_logger().error('[PICKUP] Descend failed!')
            self.success = False
            return
        self._spin_for(2.5)

        # Step 6: Close gripper
        n.get_logger().info('[PICKUP] Closing gripper...')
        self._send_gripper(1.0)
        self._spin_for(2.0)

        # Step 7: Lift
        n.get_logger().info('[PICKUP] Lifting...')
        lift_pose = self._create_pose(bx, by, bz + 0.15, qw, qx, qy, qz)
        if not self._call_planner('right', lift_pose):
            n.get_logger().error('[PICKUP] Lift failed!')
            self.success = False
            return
        self._spin_for(2.5)

        n.get_logger().info('[PICKUP] Pickup complete!')
        self.success = True

    # ── Detection (from pick_up_block.py) ─────────────────────────────

    def _detect_and_localize(self):
        """Pan camera, detect object, return 3D position in base_link."""
        n = self.node
        detection = None
        for attempt in range(10):
            rclpy.spin_once(n, timeout_sec=0.1)
            detection = self._detect_object_pixel()
            if detection is not None:
                break
            if attempt > 0:
                self._spin_for(0.3)

        if detection is None:
            n.get_logger().warn('[PICKUP]   Pixel detection failed (no red object in view)')
            return None

        cx, cy, contour = detection
        result = self._pixel_to_base_link(cx, cy, contour)
        if result is None:
            n.get_logger().warn(f'[PICKUP]   Depth projection failed at pixel ({cx}, {cy})')
        return result

    def _detect_object_pixel(self):
        """Detect target object in latest RGB image. Returns (cx, cy, contour) or None."""
        n = self.node
        rgb = n.latest_rgb
        if rgb is None:
            n.get_logger().warn('[PICKUP]     No RGB image available')
            return None

        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in self._hsv_ranges:
            mask |= cv2.inRange(hsv, lower, upper)

        red_pixels = int(np.count_nonzero(mask))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            # Save debug image on first failure
            if not hasattr(self, '_debug_saved'):
                self._debug_saved = True
                cv2.imwrite('/tmp/pickup_debug_rgb.png', bgr)
                cv2.imwrite('/tmp/pickup_debug_mask.png', mask)
                ch, cw = hsv.shape[0] // 2, hsv.shape[1] // 2
                center_hsv = hsv[ch, cw]
                n.get_logger().warn(
                    f'[PICKUP]     DEBUG: saved /tmp/pickup_debug_*.png, '
                    f'center HSV={center_hsv}')
            n.get_logger().info(
                f'[PICKUP]     HSV mask: {red_pixels} red pixels, 0 contours '
                f'(img {rgb.shape[1]}x{rgb.shape[0]})')
            return None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        n.get_logger().info(
            f'[PICKUP]     HSV mask: {red_pixels} red px, '
            f'{len(contours)} contours, largest area={area:.0f}')

        if area < 50:
            return None

        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        n.get_logger().info(f'[PICKUP]     Detected at pixel ({cx}, {cy})')
        return (cx, cy, largest)

    def _pixel_to_base_link(self, cx, cy, contour=None):
        """Convert pixel + depth to 3D point in base_link frame."""
        n = self.node
        if n.latest_depth is None or n.camera_K is None:
            n.get_logger().warn(f'[PICKUP]   No depth/K: depth={n.latest_depth is not None}, K={n.camera_K is not None}')
            return None

        # Use median depth within the contour region for robustness
        if contour is not None:
            contour_mask = np.zeros(n.latest_depth.shape[:2], dtype=np.uint8)
            cv2.drawContours(contour_mask, [contour], -1, 255, -1)
            roi_depths = n.latest_depth[contour_mask > 0].astype(np.float64)
            valid_depths = roi_depths[(roi_depths > 0) & (roi_depths < 5000)]
            if len(valid_depths) > 0:
                depth_mm = float(np.median(valid_depths))
            else:
                depth_mm = float(n.latest_depth[cy, cx])
        else:
            depth_mm = float(n.latest_depth[cy, cx])

        if depth_mm == 0:
            n.get_logger().warn(f'[PICKUP]   Depth=0 at pixel ({cx}, {cy})')
            return None
        depth_m = depth_mm / 1000.0
        n.get_logger().info(f'[PICKUP]   Depth at ({cx},{cy}): {depth_m:.3f}m (median over contour)')

        fx = n.camera_K[0, 0]
        fy = n.camera_K[1, 1]
        px = n.camera_K[0, 2]
        py = n.camera_K[1, 2]

        x_cam = (cx - px) * depth_m / fx
        y_cam = (cy - py) * depth_m / fy
        z_cam = depth_m

        try:
            transform = n.tf_buffer.lookup_transform(
                'base_link', 'camera_color_optical_frame',
                rclpy.time.Time(), timeout=Duration(seconds=1.0))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            n.get_logger().error(f'[PICKUP] TF lookup failed: {e}')
            return None

        t = transform.transform.translation
        q = transform.transform.rotation
        trans = np.array([t.x, t.y, t.z])

        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
        ])

        point_cam = np.array([x_cam, y_cam, z_cam])
        point_base = R @ point_cam + trans

        # Sanity check: object on a table/ground should have z roughly in [-0.3, 0.5]
        if point_base[2] < -0.5 or point_base[2] > 1.0:
            n.get_logger().warn(
                f'[PICKUP]   Rejected: z={point_base[2]:.3f} out of range '
                f'(base_link pos: {point_base[0]:.3f}, {point_base[1]:.3f}, {point_base[2]:.3f})')
            return None

        return point_base

    def _odom_to_base_link(self, odom_pos):
        """Transform a point from odom frame to base_link frame using TF."""
        n = self.node
        try:
            transform = n.tf_buffer.lookup_transform(
                'base_link', 'odom',
                rclpy.time.Time(), timeout=Duration(seconds=1.0))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            n.get_logger().error(f'[PICKUP] TF odom→base_link failed: {e}')
            return None

        t = transform.transform.translation
        q = transform.transform.rotation
        trans = np.array([t.x, t.y, t.z])

        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
        ])

        point_base = R @ np.array(odom_pos) + trans
        n.get_logger().info(
            f'[PICKUP]   TF: odom({odom_pos[0]:.3f}, {odom_pos[1]:.3f}, {odom_pos[2]:.3f}) '
            f'→ base_link({point_base[0]:.3f}, {point_base[1]:.3f}, {point_base[2]:.3f})')
        return point_base

    # ── Base movement (from pick_up_block.py) ─────────────────────────

    def _compute_reach(self, block_base_link):
        dx = block_base_link[0] - self.RIGHT_SHOULDER[0]
        dy = block_base_link[1] - self.RIGHT_SHOULDER[1]
        return np.sqrt(dx**2 + dy**2)

    def _move_base_closer(self, block_base_link):
        """Drive base closer using /base/target_pose for precise positioning."""
        n = self.node
        reach = self._compute_reach(block_base_link)

        if reach <= self.MAX_REACH:
            return False

        move_forward = min(reach - self.IDEAL_REACH, 0.5)  # Cap at 0.5m per step
        n.get_logger().info(f'[PICKUP]   Driving {move_forward:.3f}m closer (reach={reach:.3f}m)')

        base_pose = self._get_base_pose_in_odom()
        if base_pose is None:
            return False

        ox, oy, otheta = base_pose
        user_heading = otheta - np.pi / 2

        target_x = ox + move_forward * np.cos(user_heading)
        target_y = oy + move_forward * np.sin(user_heading)
        target_theta = user_heading

        msg = Pose2D()
        msg.x = target_x
        msg.y = target_y
        msg.theta = target_theta
        n.base_goal_reached = False
        n.base_target_pub.publish(msg)

        timeout = time.time() + 15.0
        while not n.base_goal_reached and time.time() < timeout:
            rclpy.spin_once(n, timeout_sec=0.1)

        self._spin_for(1.0)
        return True

    def _get_base_pose_in_odom(self):
        try:
            transform = self.node.tf_buffer.lookup_transform(
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

    # ── Arm planning (from pick_up_block.py) ──────────────────────────

    def _call_planner(self, arm_name, pose, use_orientation=True, duration=2.0):
        n = self.node
        request = PlanToTarget.Request()
        request.arm_name = arm_name
        request.target_pose = pose
        request.use_orientation = use_orientation
        request.execute = True
        request.duration = duration
        request.max_condition_number = 100.0

        pos = pose.position
        n.get_logger().info(
            f'[PICKUP]   Planning {arm_name} to ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})')

        future = n.plan_client.call_async(request)
        rclpy.spin_until_future_complete(n, future, timeout_sec=15.0)

        if not future.done():
            n.get_logger().error('[PICKUP]   Service call timed out!')
            return False

        if future.exception() is not None:
            n.get_logger().error(f'[PICKUP]   Service exception: {future.exception()}')
            return False

        result = future.result()
        if result.success:
            n.get_logger().info(f'[PICKUP]   OK: {result.message}')
            return True
        else:
            n.get_logger().warn(f'[PICKUP]   FAILED: {result.message}')
            return False

    def _create_pose(self, x, y, z, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = float(qw)
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        return pose

    # ── Helpers ───────────────────────────────────────────────────────

    def _spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _send_gripper(self, position):
        msg = Float64MultiArray()
        msg.data = [float(position)]
        self.node.right_gripper_pub.publish(msg)

    def _send_pan_tilt(self, pan, tilt):
        msg = Float64MultiArray()
        msg.data = [float(pan), float(tilt)]
        self.node.pan_tilt_pub.publish(msg)
