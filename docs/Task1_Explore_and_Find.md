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

**Simulation:**
```bash
# Terminal 1: sim
ros2 launch tidybot_bringup sim.launch.py scene:=scene_banana_test.xml

# Terminal 2: YOLO
ros2 run object_classification classifier

# Terminal 3: task
ros2 run tidybot_bringup task1_explore_and_find.py --ros-args -p skip_voice:=true -p target_object:=banana
```

**Real hardware:**
```bash
# Terminal 1: robot
ros2 launch tidybot_bringup real.launch.py use_planner:=true

# Terminal 2: YOLO
ros2 run object_classification classifier

# Terminal 3: task
ros2 run tidybot_bringup task1_explore_and_find.py --ros-args -p skip_voice:=false

# Terminal 4: voice
ros2 run tidybot_bringup voice_command.py
```

**Parameter overrides:**
```bash
ros2 run tidybot_bringup task1_explore_and_find.py --ros-args \
    -p skip_voice:=true -p target_object:=banana -p user_command:=get
```

## Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_object` | `""` | Object to find (YOLO class name) |
| `user_command` | `""` | Pre-set command (`get` to skip voice) |
| `skip_voice` | `true` | Skip waiting for voice input |
