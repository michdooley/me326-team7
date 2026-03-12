# Pick-and-Place Testing Directions

## Sim Testing

```bash
# Terminal 1: Robot + MuJoCo sim
ros2 launch tidybot_bringup sim.launch.py scene:=scene_bins.xml use_motion_planner:=true

# Terminal 2: YOLO classifier
ros2 run object_classification classifier

# Terminal 3: Pick-and-place (hardcoded banana)
ros2 launch tidybot_bringup pick_and_place.launch.py target_object:=banana

# Terminal 3 alt: With voice commands
ros2 launch tidybot_bringup pick_and_place.launch.py skip_voice:=false

# Terminal 4 (if using voice): Interactive voice
ros2 run tidybot_bringup voice_command.py

# Manual voice command alternative (no mic needed):
ros2 topic pub --once /user_command std_msgs/msg/String "data: 'get'"
ros2 topic pub --once /target_object std_msgs/msg/String "data: 'banana'"
```

## Real Hardware Testing

```bash
# Terminal 1: Real robot drivers
ros2 launch tidybot_bringup real.launch.py use_motion_planner:=true

# Terminal 2: YOLO classifier
ros2 run object_classification classifier

# Terminal 3: Pick-and-place
ros2 launch tidybot_bringup pick_and_place.launch.py target_object:=banana

# Terminal 4 (optional): Voice commands
ros2 run tidybot_bringup voice_command.py
```

## What the launch file does
- Starts `apriltag_detector_node` (from tidybot_perception) for Phase 2 bin finding
- Starts `pick_and_place.py` with parameters
- Does NOT start classifier or voice_command (run those separately)

## Key parameters
- `target_object` (default: banana) — which object to find and grasp
- `user_command` (default: get) — action verb
- `skip_voice` (default: true) — if false, waits for voice commands
- `arm_name` (default: right) — which arm to use

## State machine flow
Phase 1 (YOLO): SCANNING_OBJECT -> CENTERING_OBJECT -> APPROACHING_OBJECT -> OBJECT_REACHED -> GRASPING
Phase 2 (AprilTag): SCANNING_BIN -> CENTERING_BIN -> APPROACHING_BIN -> POSITIONING -> DROPPING
Then: TASK_COMPLETE -> WAITING_FOR_COMMAND (loops)

## TAG_MAP (fruit -> AprilTag ID)
banana=0, apple=1, orange=2

## MuJoCo gripper tuning (2026-03-10)
- Reduced gripper actuator kp from 1500 to 500 (was squeezing through objects)
- Added condim=6, friction=3.0, solref/solimp, margin=0.0005 to finger geoms for better grip
- File: simulation/assets/mujoco/tidybot_wx250s_bimanual.xml
