# TidyBot2 — System Overview

## What It Is

TidyBot2 is a mobile bimanual robot (2x WX250s arms, pan-tilt RealSense D435 camera, omnidirectional base) built for Stanford ME326 Collaborative Robotics. It autonomously finds, grasps, and delivers household objects using vision-based perception and voice commands.

## Architecture

```
Voice Command ──► State Machine ──► Base Control (cmd_vel)
                      │              Arm Control (IK / motion planner)
YOLO Detection ──────►│              Gripper Control
Face Recognition ────►│
AprilTag Detection ──►│
Depth + TF ──────────►│
```

All three tasks share a common code base via **mixin classes** in `scripts/common/`:

| Module | Role |
|--------|------|
| `constants.py` | YOLO class map, AprilTag IDs, joint poses, RANSAC |
| `perception.py` | Camera callbacks, depth→world projection, YOLO filtering |
| `grasp_pipeline.py` | Top-down grasp planning, IK, visual servoing, execution |
| `arm_helpers.py` | Gripper open/close, home/retract poses, camera tilt |
| `base_control.py` | Base motion, pose queries, waypoint recording |

## The Three Tasks

| Task | Script | Summary |
|------|--------|---------|
| **1** | `task1_explore_and_find.py` | Voice-commanded object retrieval with return-to-start |
| **2** | `task2_pick_and_place.py` | Pick object (YOLO) → find bin (AprilTag) → drop |
| **3** | `task3_pick_and_bring.py` | Pick object → find person (face recognition) → present → optional bin placement |

See the individual task docs for details.

## Quick Start

```bash
# Build
cd ros2_ws && colcon build --packages-select tidybot_perception tidybot_bringup && source install/setup.bash

# Launch sim
ros2 launch tidybot_bringup sim.launch.py scene:=scene_bins.xml

# Launch YOLO
ros2 run object_classification classifier

# Run a task (e.g. Task 2)
ros2 launch tidybot_bringup pick_and_place.launch.py target_object:=banana
```

## Key ROS Topics

| Topic | Purpose |
|-------|---------|
| `/objbbox` | YOLO detections (Detection2DArray) |
| `/target_object` | Voice: which object to find |
| `/user_command` | Voice: "get", "drop", "bring it to the bin" |
| `/face_detections` | Face recognition bounding boxes |
| `/cmd_vel` | Base velocity |
| `/left_arm/command`, `/right_arm/command` | Arm joint commands |
