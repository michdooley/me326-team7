# Top-Down Grasp Demo

Simple depth-based top-down grasping using YOLO object detection and depth point cloud analysis. Works on both simulation and real hardware.

## How It Works

1. **YOLO Detection** — Finds the target object (banana, apple, orange) using the external YOLO classifier
2. **Depth Point Cloud** — Crops depth image to YOLO bounding box, back-projects to 3D in `base_link` frame
3. **Min-Width Sweep** — Sweeps 0-180° to find the narrowest cross-section of the object (optimal grip angle)
4. **Grasp Execution** — Opens gripper → moves to pre-grasp above object → descends → closes gripper → lifts

For elongated objects (e.g. banana), the gripper automatically orients to grip across the narrowest dimension.

## Prerequisites

### Simulation
- MuJoCo simulation running
- YOLO classifier node running

### Real Hardware
- Real robot launched with motion planner enabled
- RealSense D435 camera connected
- YOLO classifier node running
- Interbotix SDK installed on robot

## Running the Demo

### Simulation

```bash
# Terminal 1: Start simulation
cd ros2_ws
source setup_env.bash
ros2 launch tidybot_bringup sim.launch.py

# Terminal 2: Start YOLO classifier
cd ros2_ws
source setup_env.bash
ros2 run object_classification classifier

# Terminal 3: Run grasp demo (sim is default)
cd ros2_ws
source setup_env.bash
ros2 run tidybot_bringup topdown_grasp_demo.py
```

### Real Hardware

```bash
# Terminal 1: Start real hardware with motion planner
cd ros2_ws
source setup_env.bash
ros2 launch tidybot_bringup real.launch.py use_planner:=true

# Terminal 2: Start YOLO classifier
cd ros2_ws
source setup_env.bash
ros2 run object_classification classifier

# Terminal 3: Run grasp demo with sim:=false
cd ros2_ws
source setup_env.bash
ros2 run tidybot_bringup topdown_grasp_demo.py --ros-args -p sim:=false
```

## ROS Parameters

| Parameter | Type | Default (sim) | Default (real) | Description |
|-----------|------|---------------|----------------|-------------|
| `sim` | bool | `true` | — | Set to `false` for real hardware |
| `target` | string | `banana` | `banana` | YOLO object class to grasp (`banana`, `apple`, `orange`) |
| `arm` | string | `right` | `right` | Which arm to use (`right` or `left`) |
| `duration` | float | `2.0` | `3.0` | Motion execution duration in seconds |
| `pre_grasp_height` | float | `0.10` | `0.10` | Height above grasp point for approach (meters) |
| `lift_height` | float | `0.15` | `0.15` | How high to lift after grasping (meters) |
| `z_offset` | float | `-0.02` | `-0.02` | Z offset from object top surface (negative = grip lower) |
| `bbox_pad` | float | `1.3` | `1.3` | Multiplier to pad YOLO bounding box for depth ROI |

### Example with custom parameters

```bash
# Grasp an apple with the left arm, real hardware
ros2 run tidybot_bringup topdown_grasp_demo.py --ros-args \
    -p sim:=false \
    -p target:=apple \
    -p arm:=left \
    -p z_offset:=-0.03

# Sim with longer motion duration
ros2 run tidybot_bringup topdown_grasp_demo.py --ros-args \
    -p duration:=3.0 \
    -p target:=orange
```

## Sim vs Real Differences

When `sim:=false`, the script automatically:
- **Aligns depth to RGB** — The RealSense D435 has separate depth and RGB sensors with different intrinsics. The depth image is warped to match the RGB frame before processing.
- **Uses conservative timing** — Default motion duration is 3.0s instead of 2.0s
- **Adds safety prompts** — Requires pressing Enter before executing arm motions
- **Returns to sleep pose** — Uses smooth cosine-interpolated trajectory to a safe tucked position instead of all-zeros home
- **Tracks joint states** — Monitors current arm joint positions for safe motion interpolation

## Using as a Library

The `TopdownGraspDemo` class can be imported and used from other scripts (e.g. `task1_retrieve.py`):

```python
from topdown_grasp_demo import TopdownGraspDemo

# Create node (reads ROS params)
node = TopdownGraspDemo()

# Execute grasp — returns True if holding object
success = node.run()

if success:
    # Do something with the held object (navigate, etc.)
    ...

    # Release when done
    node.release()
```

The `run()` method returns `True` if the robot is successfully holding the object, `False` on failure. The caller controls when to release via `node.release()` or `node.set_gripper(0.0)`.

## Troubleshooting

### "Service not available" error
Make sure the motion planner is running:
- **Sim**: Should auto-start with `sim.launch.py`
- **Real**: Launch with `use_planner:=true`: `ros2 launch tidybot_bringup real.launch.py use_planner:=true`

### "No YOLO detections" warning
Start the YOLO classifier: `ros2 run object_classification classifier`

### "No depth/camera_info received" error
- **Sim**: Check that `sim.launch.py` is running and the MuJoCo viewer is open
- **Real**: Check that the RealSense camera is connected: `ros2 topic echo /camera/color/image_raw --once`

### Object too wide warning
This is a non-fatal warning. The point cloud extent may overestimate the actual object width. The grasp will proceed anyway.

### IK failure
If the object is out of the arm's reach, the script will try position-only IK (without orientation constraints). If both fail, try:
- Moving the robot closer to the object
- Trying the other arm (`-p arm:=left`)
- Adjusting `z_offset` if the grasp height is problematic
