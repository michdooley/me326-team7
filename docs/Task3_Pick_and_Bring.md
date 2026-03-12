# Task 3 — Pick and Bring

**Script:** `scripts/task3_pick_and_bring.py`

## What It Does

Three-phase pipeline: grasp an object, find a specific person using face recognition, present the object, then either drop it or place it in a bin based on a voice command.

## State Machine

```
Phase 1 (Object):  WAITING → SCANNING_OBJECT → CENTERING → APPROACHING → GRASPING → (validate)
Phase 2 (Person):  SCANNING_PERSON → CENTERING_PERSON → APPROACHING_PERSON → WAITING_FOR_DROP
Phase 3 (Bin):     SCANNING_BIN → CENTERING_BIN → APPROACHING_BIN → POSITIONING → DROPPING → DONE
```

| State | Behavior |
|-------|----------|
| `GRASPING` | Grasp + optional validation (re-check YOLO with object in gripper) |
| `SCANNING_PERSON` | Scan for target person via face recognition |
| `APPROACHING_PERSON` | Drive toward person (stops at 0.6m) |
| `WAITING_FOR_DROP` | Present object in trophy pose; await voice command |
| `POSITIONING/DROPPING` | If "bring to bin" — AprilTag bin placement |

## Voice Decision at WAITING_FOR_DROP

After presenting the object to the person:
- **"drop the \<object\>"** → opens gripper, done
- **"bring it to the \<object\> bin"** → transitions to bin-finding phase (Phase 3)

## How to Run

**Simulation:**
```bash
# Terminal 1: sim
ros2 launch tidybot_bringup sim.launch.py scene:=scene_bins.xml

# Terminal 2: perception (all nodes for Task 3)
ros2 launch tidybot_bringup perception.launch.py

# Terminal 3: task
ros2 run tidybot_bringup task3_pick_and_bring.py --ros-args -p skip_voice:=true
```

**Real hardware:**
```bash
# Terminal 1: robot
ros2 launch tidybot_bringup real.launch.py use_planner:=true

# Terminal 2: perception (all nodes for Task 3)
ros2 launch tidybot_bringup perception.launch.py

# Terminal 3: task
ros2 run tidybot_bringup task3_pick_and_bring.py --ros-args -p skip_voice:=false

# Terminal 4: voice
ros2 run tidybot_bringup voice_command.py
```

**Parameter overrides:**
```bash
ros2 run tidybot_bringup task3_pick_and_bring.py --ros-args \
    -p skip_voice:=true -p target_object:=banana -p target_person:=michael \
    -p skip_grasp_validation:=true
```

## Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_object` | `""` | YOLO class name to pick |
| `target_person` | `""` | Name for face recognition |
| `skip_voice` | `true` | Skip waiting for voice input |
| `skip_grasp_validation` | `false` | Skip post-grasp YOLO check |
