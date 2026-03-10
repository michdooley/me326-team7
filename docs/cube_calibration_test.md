# Cube Calibration Test — Precision Alignment

Fine-tuning tool for depth→world coordinate transforms using a 1-inch red cube. Designed for sub-centimeter alignment verification.

## How It Differs from `grasp_calibration_test.py`

| Aspect | grasp_calibration_test | cube_calibration_test |
|--------|----------------------|----------------------|
| Detection | YOLO (banana) | **HSV red** color detection |
| RANSAC | Yes | **Disabled** (cube too small) |
| Gripper at pre-grasp | Open | **Closed** (seam = visual reference) |
| Pre-grasp height | 5cm | **2cm** (precision inspection) |
| Grasp yaw | Clearance sweep | **Fixed yaw=0** (cube is symmetric) |
| Default offsets | 0, 0, 0 | 0, 0, 0 |

## Why Closed Gripper?

When the gripper is fully closed, the finger tips form a thin seam line. At 2cm above the cube, you can precisely judge:
- **X/Y alignment**: Is the seam centered over the cube?
- **Orientation**: Is the seam parallel to a cube edge?
- **Z accuracy**: Does 2cm look correct?

## Usage

```bash
# Terminal 1 — launch robot
ros2 launch tidybot_bringup real.launch.py

# Terminal 2 — run cube calibration
ros2 run tidybot_bringup cube_calibration_test.py
```

Place a **red 1-inch cube** in front of the robot on the floor/table.

## Interactive Commands

| Key | Action |
|-----|--------|
| `Enter` | Retract arm → detect red cube → close gripper → move to 2cm above |
| `1` | Cycle backprojection mode (`depth_native` / `color_aligned` / `color_raw`) |
| `2` | Toggle pixel mapping (`rgb_to_depth` / `direct`) |
| `3+` / `3-` | Adjust X offset ±5mm |
| `4+` / `4-` | Adjust Y offset ±5mm |
| `5+` / `5-` | Adjust Z offset ±5mm |
| `6+` / `6-` | Adjust pre-grasp height ±5mm |
| `r` | Reset all offsets to defaults (0, 0, 0) |
| `p` | Print diagnostics (intrinsics, TF, HSV detection status) |
| `g` | Full grasp (open gripper, descend, close, lift) |
| `q` | Return arm to sleep and quit |

## Workflow

1. Press `Enter` — arm retracts, HSV detects red blob, gripper closes, arm moves to 2cm above cube
2. Look at the closed gripper seam relative to the cube center
3. If off in X: press `3+` or `3-` to nudge ±5mm
4. If off in Y: press `4+` or `4-`
5. If height feels wrong: press `5+`/`5-` (Z offset) or `6+`/`6-` (pre-grasp height)
6. Toggle mode with `1` to compare `depth_native` vs `color_raw`
7. Press `Enter` again to re-test
8. When happy: press `g` to test a full grasp on the cube

## Expected Output

The cube is ~25.4mm tall. Each iteration prints the measured cube height from the point cloud — this is a good sanity check:

```
z_top: 0.0280m, z_bottom: 0.0030m, cube height: 25.0mm (expected ~25.4mm)
```

If the measured height is way off, the depth pipeline has issues.

## HSV Detection Notes

The red HSV ranges used:
- Range 1: H=0-10, S=100-255, V=80-255
- Range 2: H=165-180, S=100-255, V=80-255

If detection is unreliable under your lighting, you can tune `RED_HSV_RANGES` at the top of `cube_calibration_test.py`. Press `p` to test detection without moving the arm.
