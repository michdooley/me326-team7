# Grasp + Verify Demo

Compound demo that combines top-down grasping with post-grasp YOLO verification. Picks up a target object, moves the arm to a verify pose, and checks whether YOLO detects the object in the expected region of the camera frame. Retries on failure.

Works on both simulation and real hardware via the `sim` parameter.

## How It Works

1. **Top-Down Grasp** — Detects target object via YOLO, analyzes depth point cloud for optimal grip angle, executes top-down grasp (inherited from `topdown_grasp_demo.py`)
2. **Verify Pose** — Moves arm to a presentation pose and aims camera at the gripper region
3. **YOLO Check** — If YOLO detects the target object in the upper portion of the frame (v < threshold), the grasp is verified
4. **Retry Loop** — On failure (grasp miss or verification fail), releases, returns home, and retries up to `max_retries` times

## Prerequisites

### Simulation
- MuJoCo simulation running (with fruit scene)
- YOLO classifier node running

### Real Hardware
- Robot launched with motion planner enabled
- RealSense D435 camera connected
- YOLO classifier node running
- Good lighting for YOLO detection
- Object placed on table within arm reach

## Running the Demo

### Simulation

```bash
# Terminal 1: Start simulation with fruit scene
cd ros2_ws
source setup_env.bash
ros2 launch tidybot_bringup sim.launch.py scene:=scene_fruit_grasp.xml

# Terminal 2: Start YOLO classifier
cd ros2_ws
source setup_env.bash
ros2 run object_classification classifier

# Terminal 3: Run compound demo
cd ros2_ws
source setup_env.bash
ros2 run tidybot_bringup grasp_and_verify_demo.py
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

# Terminal 3: Run compound demo with sim:=false
cd ros2_ws
source setup_env.bash
ros2 run tidybot_bringup grasp_and_verify_demo.py --ros-args -p sim:=false
```

### Real Hardware — Step-by-Step Testing

For first-time testing on real hardware:

```bash
# Start with a single attempt to verify the pipeline works
ros2 run tidybot_bringup grasp_and_verify_demo.py --ros-args \
    -p sim:=false \
    -p target:=banana \
    -p arm:=right \
    -p max_retries:=1

# Once confident, enable retries
ros2 run tidybot_bringup grasp_and_verify_demo.py --ros-args \
    -p sim:=false \
    -p target:=banana \
    -p max_retries:=3
```

### Safety Notes (Real Hardware)

- The script inherits safety prompts from `topdown_grasp_demo.py` — it will prompt for Enter before executing arm motions
- Keep hand near E-stop during first runs
- Start with `max_retries:=1` to test a single grasp+verify cycle
- If the verify pose looks wrong, adjust `v_threshold` or the verify pose constants in the script
- The arm returns to a safe sleep pose between retries and at the end

## ROS Parameters

All parameters from `topdown_grasp_demo.py` are inherited, plus:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sim` | bool | `true` | Set to `false` for real hardware |
| `target` | string | `banana` | YOLO object class to grasp (`banana`, `apple`, `orange`) |
| `arm` | string | `right` | Which arm to use (`right` or `left`) |
| `max_retries` | int | `3` | Maximum grasp+verify attempts |
| `v_threshold` | float | `320.0` | YOLO bbox v-coordinate threshold (pixels). Object above this = held. |
| `duration` | float | `2.0` (sim) / `3.0` (real) | Motion execution duration in seconds |
| `pre_grasp_height` | float | `0.10` | Height above grasp point for approach (meters) |
| `lift_height` | float | `0.15` | How high to lift after grasping (meters) |
| `z_offset` | float | `-0.02` | Z offset from object top surface (negative = grip lower) |

### Example with custom parameters

```bash
# Grasp an apple with the left arm, 2 retries
ros2 run tidybot_bringup grasp_and_verify_demo.py --ros-args \
    -p sim:=false \
    -p target:=apple \
    -p arm:=left \
    -p max_retries:=2 \
    -p v_threshold:=300.0
```

## How Verification Works

After a successful grasp and lift:

1. **Sim**: Arm moves to `[0.3, -0.2, 0.7, 0.0, 0.2, 0.0]`, camera tilts up (`-0.6`)
2. **Real**: Arm moves to `[0.0, 0.4, -0.3, 0.0, 0.5, 0.0]`, camera tilts down (`0.3`)

The camera then observes the gripper region. If YOLO detects the target object with its bounding box center in the **upper portion** of the 640x480 frame (v < `v_threshold`), the object is confirmed as held.

- **PASS**: Object detected with v < threshold (upper frame = in gripper)
- **FAIL**: Object not detected, or v >= threshold (lower frame = on table/dropped)

## Using as a Library

```python
from grasp_and_verify_demo import GraspAndVerifyDemo

node = GraspAndVerifyDemo()

# Full pipeline with retries — returns True if holding verified object
success = node.run_with_verification()

if success:
    # Object is verified as held — do something with it
    # ...
    node.release()  # When done
```

## Troubleshooting

### "Service not available" error
Make sure the motion planner is running:
- **Sim**: Should auto-start with `sim.launch.py`
- **Real**: Launch with `use_planner:=true`

### "No YOLO detections" warning
Start the YOLO classifier: `ros2 run object_classification classifier`

### Verification always fails (object detected but v >= threshold)
The default `v_threshold` of 320 may need tuning for your setup. Try:
- Lowering it: `-p v_threshold:=250.0` (stricter — object must be higher in frame)
- Raising it: `-p v_threshold:=400.0` (more permissive)

### Verification fails — object not visible
The verify arm pose may need adjustment for your specific arm/camera configuration. Check the `VERIFY_ARM_POSE_SIM` / `VERIFY_ARM_POSE_REAL` constants in the script.

### All retries fail
- Check that the object is within arm reach
- Ensure good lighting for YOLO detection
- Try a different arm: `-p arm:=left`
- Increase motion duration: `-p duration:=4.0`
