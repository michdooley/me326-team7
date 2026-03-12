# Task 2 — Pick and Place

**Script:** `scripts/task2_pick_and_place.py`

## What It Does

Two-phase pick-and-place: find and grasp an object using YOLO, then find a bin using AprilTag detection and drop the object in it.

## State Machine

```
Phase 1 (Object):  WAITING → SCANNING_OBJECT → CENTERING_OBJECT → APPROACHING_OBJECT → GRASPING
Phase 2 (Bin):     SCANNING_BIN → CENTERING_BIN → APPROACHING_BIN → POSITIONING → DROPPING → TASK_COMPLETE
```

| State | Behavior |
|-------|----------|
| `SCANNING_OBJECT/BIN` | 360° scan using YOLO (object) or AprilTag (bin) |
| `CENTERING_*` | Rotate + tilt to center detection in frame |
| `APPROACHING_*` | Drive toward target with visual servoing |
| `GRASPING` | Top-down grasp pipeline |
| `POSITIONING` | IK-based arm positioning over the bin |
| `DROPPING` | Open gripper, retract arm |

## Phase Routing

The script uses `PHASE_PARAMS` to switch detection sources and approach distances between phases:

- **Object phase:** YOLO detections, 0.38m approach distance
- **Bin phase:** AprilTag detections, 0.50m approach distance, 0.20m min depth override

## How to Run

**Simulation:**
```bash
# Terminal 1: sim
ros2 launch tidybot_bringup sim.launch.py scene:=scene_bins.xml

# Terminal 2: perception (YOLO + AprilTag for Task 2)
ros2 launch tidybot_bringup perception.launch.py use_face_recognition:=false

# Terminal 3: task
ros2 run tidybot_bringup task2_pick_and_place.py --ros-args -p skip_voice:=true
```

**Real hardware:**
```bash
# Terminal 1: robot
ros2 launch tidybot_bringup real.launch.py use_planner:=true

# Terminal 2: perception (YOLO + AprilTag for Task 2)
ros2 launch tidybot_bringup perception.launch.py use_face_recognition:=false

# Terminal 3: task
ros2 run tidybot_bringup task2_pick_and_place.py --ros-args -p skip_voice:=false

# Terminal 4: voice
ros2 run tidybot_bringup voice_command.py
```

**Parameter overrides:**
```bash
ros2 run tidybot_bringup task2_pick_and_place.py --ros-args \
    -p skip_voice:=true -p target_object:=banana
```

## Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_object` | `""` | YOLO class name to pick |
| `skip_voice` | `true` | Skip waiting for voice input |
