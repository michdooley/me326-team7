# Grasp Calibration Test Tool

Interactive tool for rapidly testing and troubleshooting depth-to-world coordinate transforms for grasping on real hardware.

## Problem

`explore_and_find_object.py` uses **color camera intrinsics** (fx=610) to backproject **depth pixels** and transforms through `camera_color_optical_frame`. On the real D435, the depth sensor has different intrinsics (fx=380) and a different optical frame. This causes systematic positional errors in grasp poses.

## Three Backprojection Modes

| Mode | Intrinsics | TF Frame | Depth Image | Notes |
|------|-----------|----------|-------------|-------|
| `depth_native` | Depth (fx=380) | `camera_depth_optical_frame` | Raw | Theoretically correct; remaps YOLO pixels RGB→depth |
| `color_aligned` | Color (fx=610) | `camera_color_optical_frame` | Warped to color | Valid but may have interpolation artifacts |
| `color_raw` | Color (fx=610) | `camera_color_optical_frame` | Raw | Current buggy behavior (baseline) |

## Usage

```bash
# Terminal 1 — launch robot
ros2 launch tidybot_bringup real.launch.py

# Terminal 2 — run calibration tool
ros2 run tidybot_bringup grasp_calibration_test.py
```

Place a banana (or other YOLO-detectable object) in front of the robot.

## Interactive Commands

| Key | Action |
|-----|--------|
| `Enter` | Retract arm → re-detect → move to pre-grasp (above object) |
| `1` | Cycle backprojection mode (`depth_native` / `color_aligned` / `color_raw`) |
| `2` | Toggle pixel mapping (`rgb_to_depth` / `direct`) |
| `3+` / `3-` | Adjust X offset ±5mm (default: -50mm) |
| `4+` / `4-` | Adjust Y offset ±5mm (default: -50mm) |
| `5+` / `5-` | Adjust Z offset ±5mm (default: 0mm) |
| `6+` / `6-` | Adjust pre-grasp height ±10mm (default: 50mm) |
| `r` | Reset all offsets to defaults |
| `p` | Print full diagnostics (intrinsics, TF matrices, pixel remap examples) |
| `g` | Execute full grasp (descend + close gripper + lift) |
| `q` | Return arm to sleep and quit |

## Workflow

1. Press `Enter` — arm retracts out of camera view, YOLO detects the object, computes the grasp pipeline, and moves the gripper to the pre-grasp pose (~5cm above the object)
2. Visually inspect — is the gripper centered above the object?
3. Toggle modes with `1` and `2`, adjust offsets with `3`-`6`
4. Press `Enter` again to re-test with new settings
5. Once alignment looks good, press `g` to test a full grasp

## Example Output

```
=== Iteration 3 | Mode: depth_native | Pixels: rgb_to_depth ===
Offsets: X=-50mm  Y=-50mm  Z=0mm  Pre-grasp height=50mm
YOLO: pixel (312, 245), bbox 85x62
Pixel mapping: RGB→Depth: (195, 153)
Intrinsics: fx=380.5 fy=380.5 cx=324.5 cy=241.7
TF frame: camera_depth_optical_frame
Raw points: 847
After RANSAC: 312 (floor_z=0.003m)
Centroid (base_link): (0.041, -0.258, 0.023)
z_top: 0.048m, grip width: 32mm, yaw: 45deg
Grasp pose: (-0.009, -0.308, 0.048)
Pre-grasp:  (-0.009, -0.308, 0.098)
IK: SUCCESS
Moving to pre-grasp...
>>> At pre-grasp. Inspect alignment visually.
```

## After Calibration

Once the best mode and offsets are identified, update `_grasp_depth_to_points()` in `explore_and_find_object.py` to:

1. Subscribe to `/camera/depth/camera_info`
2. Use depth intrinsics + `camera_depth_optical_frame` on real hardware
3. Remap YOLO pixel coords from RGB→depth before cropping
4. Apply any offset corrections found during calibration
