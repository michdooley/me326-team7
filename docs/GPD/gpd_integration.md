# GPD Integration Guide for TidyBot2 Task 1

This guide explains how to integrate the GPD (Grasp Pose Detection) grasp planner into TidyBot2's Task 1 object retrieval pipeline.

Note: the repository includes `grasp_libraries/install_gpd.sh` and tracks the GPD source as a git submodule at `grasp_libraries/gpd`.
After cloning this repo run:

```bash
git submodule update --init --recursive
```

Then run the installer from the repo root to build and install GPD consistently across machines:

```bash
cd /home/elisabeth/me326-team7
chmod +x grasp_libraries/install_gpd.sh
./grasp_libraries/install_gpd.sh
```

## Overview

The GPD grasp planner is a high-quality grasp detection algorithm that:
- Analyzes a 3D point cloud of the scene
- Generates grasp pose candidates (6-DOF position + orientation)
- Classifies them using a trained neural network
- Returns the best viable grasps ranked by quality score

**Performance:**
- Input: Point cloud (~10k-20k points)
- Processing time: ~1-3 seconds on CPU, 200-500ms on GPU
- Output: List of ranked grasp poses (typically 10-100 viable grasps)

## Architecture Integration

```
Task 1 State Machine (task1_retrieve.py)
         │
         ├─ LISTEN ──→ Get target object name
         ├─ SEARCH ──→ Navigate to find object, build point cloud
         │
         ├─ APPROACH ─→ Get close to object
         │     │
         │     └──→ [Navigator publishes /camera/depth/cloud]
         │
         ├─ PLAN_GRASP ─→ Call /plan_grasp service
         │     │
         │     └──→ [GraspPlannerNode processes cloud, returns grasp]
         │
         ├─ PRE_GRASP ──→ Move to pre-grasp pose
         ├─ GRASP ─────→ Close gripper
         ├─ VERIFY_GRASP
         ├─ RETURN ─────→ Return to start
         └─ DONE
```

## Installation & Setup

### 1. Install GPD Library (one-time setup)

Follow the instructions in [gpd_setup.md](./gpd_setup.md) or use the repo installer (recommended). Summary (repo-based):

```bash
# Ensure system deps are installed
sudo apt install -y libpcl-dev libeigen3-dev libopencv-dev libvtk-dev build-essential cmake

# From the repo root (after initializing submodules):
cd /home/elisabeth/me326-team7
chmod +x grasp_libraries/install_gpd.sh
./grasp_libraries/install_gpd.sh
```

### 2. Copy GPD Config & Models to TidyBot Workspace

```bash
# Copy pre-trained neural network models
mkdir -p /home/elisabeth/me326-team7/ros2_ws/src/tidybot_perception/models
cp -r ~/grasp_libraries/gpd/models/* /home/elisabeth/me326-team7/ros2_ws/src/tidybot_perception/models/

# The tidybot_gpd package config file is already in place
# (ros2_ws/src/tidybot_gpd/config/gpd_params.cfg)
```

### 3. Build tidybot_gpd ROS2 Package

```bash
cd /home/elisabeth/me326-team7/ros2_ws
source /opt/ros/humble/setup.bash

# Build the GPD ROS2 wrapper
colcon build --packages-select tidybot_gpd

source install/setup.bash
```

If the build fails with "GPD not found", verify that GPD was installed:
```bash
find /usr/local -name "*gpd*" 2>/dev/null
# Should see: /usr/local/lib/libgpd.so and /usr/local/lib/cmake/gpd/...
```

---

## Running the Grasp Planner

### Option A: Standalone Testing (No Task Integration)

Test the grasp planner node in isolation:

**Terminal 1 — Start simulator:**
```bash
cd /home/elisabeth/me326-team7/ros2_ws
source setup_env.bash
ros2 launch tidybot_bringup sim.launch.py show_mujoco_viewer:=true use_rviz:=false
```

**Terminal 2 — Launch the grasp planner:**
```bash
cd /home/elisabeth/me326-team7/ros2_ws
source install/setup.bash
ros2 launch tidybot_gpd grasp_planner.launch.py
```

**Terminal 3 — Test the service:**
```bash
# Wait for the grasp planner to be ready, then call the service
sleep 2

# Test with a realistic point cloud
# Assuming your robot is looking at an object at (0.4m, 0.0m, 0.2m)
ros2 service call /plan_grasp tidybot_msgs/srv/PlanGrasp \
  "{object_position: {header: {frame_id: 'base_link'}, point: {x: 0.4, y: 0.0, z: 0.2}}, object_class: 'block', arm_name: 'right'}"

# Expected output: success=true, grasp_pose, pre_grasp_pose, arm_used, grasp_width
```

**Terminal 4 — Monitor the grasp planner logs:**
```bash
# Watch what the grasp planner is doing
ros2 topic echo /rosout --filter "GraspPlannerNode"
```

### Option B: Full Task 1 Integration

Run the complete Task 1 object retrieval with grasp planning:

**Terminal 1 — Simulator:**
```bash
ros2 launch tidybot_bringup sim.launch.py
```

**Terminal 2 — All perception nodes (detection, localization, grasp planning):**
```bash
# This launch file will be created in task1_bringup.launch.py
# For now, launch individually:

# In separate terminals or use tmux

# Launch 2a — Grasp planner
ros2 launch tidybot_gpd grasp_planner.launch.py

# Launch 2b — Object detector (if available)
# ros2 run tidybot_perception detector_node --ros-args -p model_path:=/path/to/yolov8.pt

# Launch 2c — Object localizer (if available)
# ros2 run tidybot_perception object_localizer_node
```

**Terminal 3 — Task 1 state machine:**
```bash
ros2 run tidybot_bringup task1_retrieve.py
# Follow on-screen prompts to trigger state transitions or voice commands
```

---

## Understanding the Grasp Planner Output

When `/plan_grasp` service is called, it returns:

```
success (bool) — Whether grasp planning succeeded
grasp_pose (geometry_msgs/Pose) — The optimal grasp pose
  - position: grasp center point in base_link frame
  - orientation: quarternion (wxyz) for gripper orientation
pre_grasp_pose (geometry_msgs/Pose) — Approach pose (above the grasp by approach_offset)
arm_used (string) — Which arm was selected ("right" or "left")
grasp_width (float64) — Suggested gripper width (typically 0.08m)
message (string) — Status message or error description
```

### Example Response

```yaml
success: true
grasp_pose:
  position:
    x: 0.425
    y: 0.012
    z: 0.185
  orientation:
    w: 0.707
    x: 0.0
    y: 0.707
    z: 0.0

pre_grasp_pose:
  position:
    x: 0.425
    y: 0.012
    z: 0.285  # z + 0.10m (approach_offset)
  orientation:
    w: 0.707
    x: 0.0
    y: 0.707
    z: 0.0

arm_used: "right"
grasp_width: 0.08
message: "Grasp planned successfully"
```

---

## Troubleshooting

### Issue: "GPD not initialized" or "config_file parameter not set"

**Solution:** Make sure you're launching with the config file parameter:

```bash
ros2 launch tidybot_gpd grasp_planner.launch.py \
  config_file:=/path/to/gpd_params.cfg
```

### Issue: Service call hangs or times out

**Cause:** Grasp planner is waiting for a point cloud.

**Solution:**
1. Make sure your point cloud source is publishing: `ros2 topic list | grep cloud`
2. Check the point cloud has data: `ros2 topic echo /camera/depth/cloud | head -20`
3. Verify the topic name matches what grasp planner expects: look in launch file

### Issue: "No grasps found in point cloud"

**Cause:** The scene doesn't contain graspable geometry in the expected workspace.

**Solutions:**
1. Verify the object is within the `workspace` bounds in `gpd_params.cfg`
2. Check point cloud quality: is it dense and noise-free?
3. Increase `num_samples` in config (more candidates = better chance of finding grasps)
4. Ensure point cloud origin is at the robot base (not at camera)

### Issue: Slow grasp detection (>3-4 seconds)

**Solutions:**
1. Reduce `num_samples` in `gpd_params.cfg` (e.g., 5000 instead of 15000)
2. Use 3-channel model instead of 15-channel (faster, lower quality)
3. Install OpenVINO for GPU acceleration
4. Downsample input point cloud before sending to GPD

### Issue: Grasps are unrealistic or collision-prone

**Solutions:**
1. Double-check gripper dimensions in `gpd_params.cfg`:
   - `radius`, `finger_width`, `hand_outer_diameter`, `max_aperture`
2. Verify camera position is accurate (`cam_position`)
3. Ensure point cloud is in the correct frame and has correct scale

---

## References & Credits

- **GPD Library:** https://github.com/atenpas/gpd
- **Paper:** [Grasp Pose Detection in Point Clouds](http://arxiv.org/abs/1706.09911)
- **Original ROS wrapper:** https://github.com/atenpas/gpd_ros

