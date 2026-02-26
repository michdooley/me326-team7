# Testing: Grasp Pipeline Refactor

## Prerequisites

```bash
# Install vision_msgs (needed by classifier + task1_retrieve)
sudo apt install -y ros-humble-vision-msgs

# Rebuild workspace
cd ~/me326-team7/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## Step 1: Smoke Test grasp_demo.py with Classifier Node

Tests that the YOLO refactor works (grasp_demo.py now consumes /objbbox instead of running its own YOLO).

```bash
# Terminal 1: sim
ros2 launch tidybot_bringup sim.launch.py scene:=scene_fruit_grasp.xml

# Terminal 2: classifier (grasp_demo.py now depends on this)
ros2 run object_classification classifier

# Terminal 3: GraspNet planner
ros2 run tidybot_perception grasp_planner_node \
    --ros-args -p model_path:=$HOME/graspnet-baseline/logs/checkpoint-rs.tar

# Terminal 4: grasp demo
ros2 run tidybot_bringup grasp_demo.py --ros-args -p object:=banana
```

### What to check
- [ ] grasp_demo.py starts WITHOUT loading its own YOLO model
- [ ] Logs show detection coming from /objbbox (not inline YOLO)
- [ ] Grasp executes end-to-end: pre-grasp -> grasp -> close -> lift
- [ ] Deleted scripts are gone: `ls ros2_ws/src/tidybot_bringup/scripts/rgbd_grasp_demo*` returns nothing

### Troubleshooting
- **"No module named 'vision_msgs'"**: Run `sudo apt install -y ros-humble-vision-msgs`, rebuild
- **"No detections"**: Make sure classifier (Terminal 2) is running and publishing: `ros2 topic echo /objbbox --once`
- **"/plan_grasp not available"**: Make sure grasp_planner_node (Terminal 3) is running: `ros2 service list | grep plan_grasp`

---

## Step 2: Test task1_retrieve.py with GraspNet + Lift + Verify

Tests the new LIFT state and /plan_grasp integration in the full state machine.

```bash
# Terminals 1-3: same as Step 1

# Terminal 4: task1 (instead of grasp_demo)
ros2 run tidybot_bringup task1_retrieve.py \
    --ros-args -p target_object:=banana -p sim:=true
```

### What to check in logs
- [ ] Startup shows `Mode: sim`
- [ ] Startup shows `Target object: banana` (from parameter)
- [ ] PLAN_GRASP: `Calling GraspNet /plan_grasp ...` then `GraspNet succeeded`
- [ ] GRASP: `Closing gripper...`
- [ ] LIFT: `State: GRASP -> LIFT` then `[lift] OK` -- arm visibly moves up ~10cm
- [ ] VERIFY_GRASP: `[verify] Moving arm to present pose...` -- camera check happens AFTER lift
- [ ] Either `grasp confirmed!` or `not confirmed -- retrying`

### Troubleshooting
- **Stuck in SEARCH**: NavigateToObject can't find the object. Check classifier is detecting: `ros2 topic echo /objbbox --once`
- **PLAN_GRASP timeout (8s)**: Camera not seeing object after positioning. Check camera tilt is working: `ros2 topic echo /camera/pan_tilt_cmd --once`
- **[lift] Failed (non-fatal)**: IK can't reach the lift pose. Not blocking -- verify will still run. Check if grasp pose z is reasonable.
- **[pre-grasp] FAILED / [grasp] FAILED**: IK or collision check failing. Check object position in logs (`Object at base_link: ...`). May need to adjust robot positioning.

---

## Step 3: Test Top-Down Fallback (no GraspNet)

Kill the grasp_planner_node and verify task1 still works with top-down grasps.

```bash
# Kill Terminal 3 (grasp_planner_node), then run task1 again:
ros2 run tidybot_bringup task1_retrieve.py \
    --ros-args -p target_object:=banana -p sim:=true
```

### What to check
- [ ] Logs show: `/plan_grasp not available -- using top-down fallback`
- [ ] Logs show: `Top-down grasp: pre_z=... grasp_z=...`
- [ ] Grasp still executes (pre-grasp -> grasp -> lift -> verify)

---

## Step 4: Test target_object Parameter

```bash
# With apple (requires apple in the scene)
ros2 run tidybot_bringup task1_retrieve.py \
    --ros-args -p target_object:=apple -p sim:=true
```

### What to check
- [ ] Startup logs show `Target object: apple`
- [ ] Robot searches for and approaches an apple (not banana)

---

## Quick Debug Commands

```bash
# Check if classifier is publishing detections
ros2 topic echo /objbbox --once

# Check if GraspNet service is available
ros2 service list | grep plan_grasp

# Check camera topics
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw

# Check gripper topics (sim)
ros2 topic echo /right_gripper/cmd --once

# Monitor state transitions
ros2 topic echo /rosout --field msg | grep "State:"
```
