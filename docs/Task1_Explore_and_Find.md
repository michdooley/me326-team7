# Task 1 — Explore & Find Object

**Script:** `scripts/task1_explore_and_find.py`

## What It Does

Voice-commanded object retrieval. The robot scans its surroundings, finds a requested object, grasps it, returns to its starting position, and waits for a drop command.

## State Machine

```
WAITING_FOR_COMMAND → SCANNING → CENTERING → APPROACHING → GRASPING → RETURNING_HOME → WAITING_FOR_DROP
```

| State | Behavior |
|-------|----------|
| `WAITING_FOR_COMMAND` | Listens for voice command or uses hardcoded `target_object` param |
| `SCANNING` | 360° rotation scan, checks YOLO detections each frame |
| `CENTERING` | Rotates base + tilts camera to center the object in frame |
| `APPROACHING` | Drives toward the object with continuous visual servoing |
| `GRASPING` | Runs top-down grasp pipeline (from `GraspMixin`) |
| `RETURNING_HOME` | Follows recorded waypoints back to starting pose |
| `WAITING_FOR_DROP` | Waits for "drop" voice command, then opens gripper |

## How to Run

**Simulation (no voice):**
```bash
# Terminal 1: sim
ros2 launch tidybot_bringup sim.launch.py scene:=scene_banana_test.xml show_mujoco_viewer:=false

# Terminal 2: nav + grasp
ros2 launch tidybot_bringup nav.launch.py target_object:=banana user_command:=get
```

**With voice:**
```bash
# Terminal 2:
ros2 launch tidybot_bringup nav.launch.py skip_voice:=false

# Terminal 3:
ros2 run tidybot_bringup voice_command.py
```

## Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_object` | `""` | Object to find (YOLO class name) |
| `user_command` | `""` | Pre-set command (`get` to skip voice) |
| `skip_voice` | `true` | Skip waiting for voice input |
