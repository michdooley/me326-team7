"""Tests for RGBDObjectDetector floor-clamping logic."""

import numpy as np
import pytest
from unittest.mock import MagicMock, patch

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo
from builtin_interfaces.msg import Time


# We need to mock several ROS subsystems before importing the detector,
# since its module-level imports pull in tf2/cv_bridge.  Instead we
# patch at the instance level after import.

from tidybot_perception.rgbd_object_detector import RGBDObjectDetector


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_camera_info(fx=456.0, fy=456.0, cx=320.0, cy=240.0):
    """Build a minimal CameraInfo with the given intrinsics."""
    info = CameraInfo()
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    return info


def _make_depth_image(value_mm=400, shape=(480, 640)):
    """Create a uniform uint16 depth image (value in millimetres)."""
    return np.full(shape, value_mm, dtype=np.uint16)


def _fake_point_camera_to_base(z_value):
    """Return a mock transform function that produces a point at given z."""
    def _transform(pt):
        out = PointStamped()
        out.header.frame_id = 'base_link'
        out.point.x = 0.0
        out.point.y = -0.4
        out.point.z = float(z_value)
        return out
    return _transform


def _build_detector(floor_z_min=None):
    """Construct an RGBDObjectDetector with all ROS pieces mocked out."""
    mock_node = MagicMock()
    mock_node.get_clock.return_value.now.return_value.to_msg.return_value = Time()

    with patch('tidybot_perception.rgbd_object_detector.tf2_ros'):
        with patch('tidybot_perception.rgbd_object_detector.CoordConverter'):
            if floor_z_min is not None:
                det = RGBDObjectDetector(mock_node, floor_z_min=floor_z_min)
            else:
                det = RGBDObjectDetector(mock_node)

    # Give it valid depth + camera info so pixel_to_base_link doesn't
    # bail out early.
    det.latest_depth = _make_depth_image(400)       # 0.4 m
    det.camera_info = _make_camera_info()
    return det


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestFloorClamp:
    """Verify that points below the floor get clamped upward."""

    def test_point_below_floor_is_clamped(self):
        det = _build_detector()
        det.coord_conv.point_camera_to_base = _fake_point_camera_to_base(-0.05)

        result = det.pixel_to_base_link(320, 240)

        assert result is not None
        assert result.point.z == pytest.approx(det._floor_z)

    def test_point_well_above_floor_passes_through(self):
        det = _build_detector()
        det.coord_conv.point_camera_to_base = _fake_point_camera_to_base(0.10)

        result = det.pixel_to_base_link(320, 240)

        assert result is not None
        assert result.point.z == pytest.approx(0.10)

    def test_point_exactly_at_floor_passes_through(self):
        det = _build_detector()
        z = det._floor_z
        det.coord_conv.point_camera_to_base = _fake_point_camera_to_base(z)

        result = det.pixel_to_base_link(320, 240)

        assert result is not None
        assert result.point.z == pytest.approx(z)

    def test_custom_floor_z_min(self):
        det = _build_detector(floor_z_min=0.02)
        det.coord_conv.point_camera_to_base = _fake_point_camera_to_base(0.01)

        result = det.pixel_to_base_link(320, 240)

        assert result is not None
        assert result.point.z == pytest.approx(0.02)

    def test_warning_logged_on_clamp(self):
        det = _build_detector()
        det.coord_conv.point_camera_to_base = _fake_point_camera_to_base(-0.03)

        det.pixel_to_base_link(320, 240)

        det._node.get_logger().warn.assert_called_once()
        msg = det._node.get_logger().warn.call_args[0][0]
        assert 'Clamping' in msg
        assert 'below floor' in msg

    def test_no_warning_when_above_floor(self):
        det = _build_detector()
        det.coord_conv.point_camera_to_base = _fake_point_camera_to_base(0.10)

        det.pixel_to_base_link(320, 240)

        det._node.get_logger().warn.assert_not_called()
