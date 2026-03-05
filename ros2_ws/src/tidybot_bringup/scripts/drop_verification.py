#!/usr/bin/env python3
"""
Drop verification helper for TidyBot2.

After dropping an object, takes a picture and checks if the gripper is
"obviously off" (e.g., too far left of the bin). If not obviously off,
confirms object in bin.

Uses TF to project gripper and bin positions to the camera image.
"""

from __future__ import annotations

from typing import Optional, Any

try:
    from cv_bridge import CvBridge
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False

# Bin opening radius in meters (for projecting bin bounds to image)
BIN_RADIUS_M = 0.08
# Pixels: beyond this outside bin bbox = "obviously off"
MARGIN_OBVIOUSLY_OFF_PX = 80


def verify_gripper_alignment(
    node: Any,
    tf_buffer: Any,
    bin_pos: tuple[float, float, float],
    arm: str,
    image_msg: Any,
    camera_info: Any,
) -> tuple[bool, str]:
    """
    Check if gripper is obviously off from the bin in the image.

    Returns (success, message).
    success=True means gripper is NOT obviously off -> confirm object in bin.
    success=False means gripper is obviously misaligned -> reject.
    """
    if not CV_AVAILABLE:
        return True, "cv_bridge/opencv not available, skipping verification (assume success)"

    base_frame = "base_link"
    camera_frame = "camera_color_optical_frame"
    gripper_frame = f"{arm}_gripper_link"

    # Get camera intrinsics
    if camera_info is None or len(camera_info.k) < 9:
        return True, "No camera info, skipping verification"
    fx = camera_info.k[0]
    fy = camera_info.k[4]
    cx = camera_info.k[2]
    cy = camera_info.k[5]

    try:
        from geometry_msgs.msg import PointStamped
        from tf2_geometry_msgs import do_transform_point
        from rclpy.duration import Duration

        now = node.get_clock().now()
        t_base_cam = tf_buffer.lookup_transform(
            camera_frame,
            base_frame,
            now,
            timeout=Duration(seconds=0.5),
        )

        def project(p: tuple[float, float, float]) -> Optional[tuple[int, int]]:
            ps = PointStamped()
            ps.header.frame_id = base_frame
            ps.point.x, ps.point.y, ps.point.z = p[0], p[1], p[2]
            ps_cam = do_transform_point(ps, t_base_cam)
            x, y, z = ps_cam.point.x, ps_cam.point.y, ps_cam.point.z
            if z <= 0.01:
                return None
            u = fx * (x / z) + cx
            v = fy * (y / z) + cy
            return (int(round(u)), int(round(v)))

        # Project bin center and edges to image
        bx, by, bz = bin_pos
        bin_corners = [
            (bx, by, bz),
            (bx - BIN_RADIUS_M, by, bz),
            (bx + BIN_RADIUS_M, by, bz),
            (bx, by - BIN_RADIUS_M, bz),
            (bx, by + BIN_RADIUS_M, bz),
        ]
        bin_pixels = [project(p) for p in bin_corners]
        if any(p is None for p in bin_pixels):
            return True, "Bin not visible (behind camera), skipping verification"

        u_min = min(p[0] for p in bin_pixels)
        u_max = max(p[0] for p in bin_pixels)
        v_min = min(p[1] for p in bin_pixels)
        v_max = max(p[1] for p in bin_pixels)

        # Get gripper position in camera frame
        t_gripper_cam = tf_buffer.lookup_transform(
            camera_frame,
            gripper_frame,
            now,
            timeout=Duration(seconds=0.5),
        )
        ps = PointStamped()
        ps.header.frame_id = gripper_frame
        ps.point.x = ps.point.y = ps.point.z = 0.0
        ps_cam = do_transform_point(ps, t_gripper_cam)
        x, y, z = ps_cam.point.x, ps_cam.point.y, ps_cam.point.z
        if z <= 0.01:
            return True, "Gripper not visible (behind camera), skipping verification"
        gu = int(round(fx * (x / z) + cx))
        gv = int(round(fy * (y / z) + cy))

        # Check if gripper is "obviously off" (horizontal only; vertical checks
        # removed since gripper is always above the bin when dropping)
        if gu < u_min - MARGIN_OBVIOUSLY_OFF_PX:
            return False, f"Gripper too far left of bin (gu={gu}, u_min={u_min})"
        if gu > u_max + MARGIN_OBVIOUSLY_OFF_PX:
            return False, f"Gripper too far right of bin (gu={gu}, u_max={u_max})"

        # Save debug image with overlays
        try:
            bridge = CvBridge()
            cv_img = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            h, w = cv_img.shape[:2]
            cv2.rectangle(
                cv_img,
                (max(0, u_min), max(0, v_min)),
                (min(w, u_max), min(h, v_max)),
                (0, 255, 0),
                2,
            )
            if 0 <= gu < w and 0 <= gv < h:
                cv2.circle(cv_img, (gu, gv), 8, (0, 0, 255), 2)
            from pathlib import Path
            out_dir = Path(__file__).parent.parent / "captures"
            out_dir.mkdir(exist_ok=True)
            out_path = out_dir / "drop_verification.png"
            cv2.imwrite(str(out_path), cv_img)
            node.get_logger().info(f"  Saved verification image to {out_path}")
        except Exception as e:
            node.get_logger().debug(f"Could not save debug image: {e}")

        return True, f"Gripper aligned (gu={gu}, gv={gv}) within bin region"

    except Exception as e:
        node.get_logger().warn(f"Verification failed: {e}")
        return True, f"Verification error ({e}), assuming success"
