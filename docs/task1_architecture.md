# Task 1: Object Retrieval — Architecture & Development Guide

## Overview

Task 1 implements voice-commanded object retrieval: the robot listens for a spoken command, finds the target object in the scene, navigates to it, picks it up, and returns to its starting position.

**Example command:** *"Locate the apple in the scene and retrieve it."*

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                        task1_retrieve.py                             │
│                     (Master State Machine)                           │
│  LISTEN → SEARCH → APPROACH → PLAN_GRASP → PRE_GRASP → GRASP →    │
│  VERIFY_GRASP → RETURN → DONE                                       │
└──────┬──────────┬──────────────────────┬───────────────────────────┘
       │          │                      │
       ▼          ▼                      ▼
  ┌─────────┐ ┌──────────┐         ┌─────────────┐
  │  voice   │ │navigate  │         │   grasp     │
  │ command  │ │to_object │         │  planner    │
  └─────────┘ └──────────┘         │   _node     │
       │          │  ▲               └─────────────┘
       │          │  │                     │
       ▼          ▼  │                     ▼
  ┌─────────┐ ┌──────┐               ┌─────────────┐
  │Google   │ │/cmd  │               │/plan_to     │
  │Cloud    │ │_vel  │               │_target      │
  │STT +   │ │      │               │(existing IK)│
  │Gemini  │ └──────┘               └─────────────┘
  └─────────┘
                    ┌────────────────────────┐
                    │    classifier.py        │
                    │ (object_classification) │
                    │    YOLOv8 detection     │
                    └────────┬───────────────┘
                             │
                    ┌────────┴───────────┐
                    │  /objbbox          │
                    │  /camera/color/    │
                    │   image_yolo       │
                    └────────────────────┘
```

Both `navigate_to_object.py` and `task1_retrieve.py` subscribe to `/objbbox` for YOLO detections. 3D localization (depth → base_link) is done inline using `CoordConverter.pixel_to_base_link()`.

---

## Packages & Files

### `tidybot_msgs` — Interface Definitions

All custom messages and services. **Build this first** — everything else depends on it.

| File | Type | Purpose |
|------|------|---------|
| `msg/Detection2D.msg` | Message | Single bounding box (class, confidence, x, y, w, h) |
| `msg/Detection2DArray.msg` | Message | Array of detections per camera frame |
| `msg/ObjectPose.msg` | Message | 3D position of a detected object in `base_link` frame |
| `srv/PlanGrasp.srv` | Service | Object position → grasp pose + pre-grasp pose |

### `object_classification` — YOLO Detection Node

| File | Node Name | Role |
|------|-----------|------|
| `classifier.py` | `object_classifier` | YOLOv8 detection → `/objbbox` + annotated image |

### `tidybot_perception` — Perception Utilities

| File | Role |
|------|------|
| `coord_converter.py` | TF2 frame transforms + `pixel_to_base_link()` depth back-projection |
| `yolo_object_detector.py` | Standalone YOLO detector (legacy — used by `test_yolo_camera.py`) |
| `rgbd_object_detector.py` | HSV color segmentation detector (legacy) |
| `grasp_planner_node.py` | GraspNet grasp pose prediction (future) |

### `tidybot_bringup` — Task Scripts

| File | Role |
|------|------|
| `scripts/task1_retrieve.py` | Master state machine orchestrating the full task |
| `scripts/voice_command.py` | Audio → Google Cloud STT → Gemini → target object |
| `scripts/navigate_to_object.py` | Search + approach behavior (importable class) |
| `utilities/robot_helpers.py` | Shared helper functions (service calls, pose creation, gripper control) |
| `launch/task.launch.py` | Launches perception nodes |

---

## Message Flow Between Modules

### Topics (continuous data streams)

```
/camera/color/image_raw ──────► classifier.py ──► /objbbox (vision_msgs/Detection2DArray)
                                      │
                                      └──────────► /camera/color/image_yolo (annotated)
                                                        │
/camera/depth/image_raw ──┐                             │
/camera/color/camera_info ┼──► navigate_to_object ◄─────┘ (subscribes to /objbbox)
                          │           │
                          │           ├──► /cmd_vel
                          │           └──► /camera/pan_tilt_cmd
                          │
                          └──► task1_retrieve ◄──── /objbbox
                                      │
                                      ├──► /cmd_vel (RETURN state)
                                      ├──► /right_arm/cmd
                                      └──► /right_gripper/cmd
```

| Topic | Type | Publisher | Subscriber(s) |
|-------|------|-----------|---------------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | MuJoCo bridge / RealSense | `classifier` |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | MuJoCo bridge / RealSense | `navigate_to_object`, `task1_retrieve` |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | MuJoCo bridge / RealSense | `navigate_to_object`, `task1_retrieve` |
| `/objbbox` | `vision_msgs/Detection2DArray` | `classifier` | `navigate_to_object`, `task1_retrieve` |
| `/camera/color/image_yolo` | `sensor_msgs/Image` | `classifier` | RViz (YOLO Detections display) |
| `/target_object` | `std_msgs/String` | `voice_command` | `task1_retrieve` |
| `/user_command` | `std_msgs/String` | `voice_command` | `task1_retrieve` |
| `/cmd_vel` | `geometry_msgs/Twist` | `navigate_to_object`, `task1_retrieve` | base controller |
| `/camera/pan_tilt_cmd` | `Float64MultiArray` | `navigate_to_object` | pan-tilt controller |
| `/right_arm/cmd` | `ArmCommand` | `task1_retrieve` | arm controller |
| `/right_gripper/cmd` | `Float64MultiArray` | `task1_retrieve` | gripper controller |
| `/joint_states` | `sensor_msgs/JointState` | MuJoCo bridge / hardware | `task1_retrieve` |
| `/odom` | `nav_msgs/Odometry` | base controller | `navigate_to_object`, `task1_retrieve` |
| `/map` | `nav_msgs/OccupancyGrid` | `navigate_to_object` | RViz |

### Services (request/response)

| Service | Type | Server | Called by |
|---------|------|--------|-----------|
| `/plan_grasp` | `PlanGrasp` | `grasp_planner_node` | `task1_retrieve` |
| `/plan_to_target` | `PlanToTarget` | `motion_planner` (existing) | `task1_retrieve`, `grasp_planner_node` |
| `/microphone/record` | `AudioRecord` | `microphone_node` (existing) | `voice_command` |

---

## Development Guides by Module

Each module can be developed and tested independently. The general workflow:

1. Launch sim: `ros2 launch tidybot_bringup sim.launch.py`
2. Run the classifier: `ros2 run object_classification classifier`
3. Run your node in a third terminal
4. Verify outputs with `ros2 topic echo` or check service responses

### Module A: `classifier.py` — YOLO Object Detection (Implemented)

**Package:** `object_classification`
**Node name:** `object_classifier`

Runs YOLOv8 on each camera frame, publishes bounding box detections and an annotated image for RViz.

**Inputs:**
| Input | Topic/Type | Source |
|-------|-----------|--------|
| RGB image | `/camera/color/image_raw` (`sensor_msgs/Image`) | MuJoCo bridge / RealSense |

**Outputs:**
| Output | Topic | Type |
|--------|-------|------|
| Bounding box detections | `/objbbox` | `vision_msgs/Detection2DArray` |
| Annotated image | `/camera/color/image_yolo` | `sensor_msgs/Image` |

**Detection format:** Each detection has `bbox.center.position.x/y` (pixel centroid), `bbox.size_x/y` (pixel dimensions), and `results[0].hypothesis.class_id` (COCO class ID as string: `"0"`=person, `"46"`=banana, `"47"`=apple, `"49"`=orange), `results[0].hypothesis.score` (confidence).

**Currently filtered classes:** `[0, 46, 47]` (person, banana, apple). Edit `classifier.py` line 27 to add more.

**How to test:**
```bash
# Terminal 1 — sim
ros2 launch tidybot_bringup sim.launch.py

# Terminal 2 — run classifier
ros2 run object_classification classifier

# Terminal 3 — verify detections
ros2 topic echo /objbbox

# Check annotated image in RViz: "YOLO Detections" display
ros2 topic hz /camera/color/image_yolo
```

---

### Module B: Object Localization (Inline via CoordConverter)

There is no separate localizer node. 3D localization is performed inline by `navigate_to_object.py` and `task1_retrieve.py` using:

```python
from tidybot_perception.coord_converter import CoordConverter

# In __init__:
self.coord_conv = CoordConverter(self.tf_buffer)

# To localize a pixel detection:
base_pt = self.coord_conv.pixel_to_base_link(
    u, v, self.latest_depth, self.camera_info, self.get_clock())
```

`CoordConverter.pixel_to_base_link()` does:
1. Sample an 11x11 depth patch around (u, v), take median of valid depths (0.1–5m)
2. Back-project to camera optical frame using intrinsics
3. Transform to `base_link` via TF2
4. Clamp z to floor level (0.005m)

---

### Module C: `grasp_planner_node.py` — GraspNet Grasp Planning

**Owner's goal:** Given a 3D object position, predict a grasp pose using GraspNet and return it via service.

**Inputs you receive:**
| Input | Service | Source |
|-------|---------|--------|
| Object position + class | `/plan_grasp` request (`PlanGrasp`) | `task1_retrieve` state machine |

**Outputs you produce:**
| Output | Field | Type |
|--------|-------|------|
| Grasp pose | `response.grasp_pose` | `geometry_msgs/Pose` |
| Pre-grasp pose | `response.pre_grasp_pose` | `geometry_msgs/Pose` |
| Arm used | `response.arm_used` | `string` |
| Gripper width | `response.grasp_width` | `float64` |

**Key implementation steps:**
1. Load GraspNet model in `_load_model()`
2. In `plan_grasp_callback()`:
   - Get scene point cloud (may need to subscribe to depth + camera_info internally, or receive from caller)
   - Run GraspNet inference → ranked list of grasp candidates
   - Select best reachable grasp (optionally validate via `/plan_to_target` service)
   - Compute `pre_grasp_pose` by offsetting `grasp_pose` by `self.approach_offset` along approach direction
3. Return the grasp pose in `base_link` frame

**How to test (standalone — no other modules needed):**
```bash
# Terminal 1 — sim (need motion planner for optional reachability check)
ros2 launch tidybot_bringup sim.launch.py

# Terminal 2 — run your node
ros2 run tidybot_perception grasp_planner_node \
    --ros-args -p model_path:=/path/to/graspnet_weights.pth

# Terminal 3 — call the service with a fake object position
ros2 service call /plan_grasp tidybot_msgs/srv/PlanGrasp "{
  object_position: {header: {frame_id: 'base_link'}, point: {x: 0.4, y: 0.0, z: 0.15}},
  object_class: 'block',
  arm_name: 'right'
}"
```

---

### Module D: `voice_command.py` — Voice → Target Object

**Owner's goal:** Record audio, transcribe with Google Cloud STT, extract target object + action via Gemini.

**Inputs you receive:**
| Input | Service | Source |
|-------|---------|--------|
| Audio recording | `/microphone/record` (`AudioRecord`) | `microphone_node` (existing) |

**Outputs you produce:**
| Output | Topic | Type | QoS |
|--------|-------|------|-----|
| Target object name | `/target_object` | `std_msgs/String` | Transient local (latched) |
| Action command | `/user_command` | `std_msgs/String` | Transient local (latched) |

**Also exposes:** `get_voice_command(node) → (target_object, action)` function for direct import.

**Key implementation steps:**
1. Call `/microphone/record` service with `start=True`, wait, then `start=False` to get audio data
2. Send `audio_data` to Google Cloud Speech-to-Text → get transcript
3. Send transcript to Gemini with a prompt like:
   > *"Extract the target object and action from this robot command. Return JSON: {target_object: string, action: string}. Command: '{transcript}'"*
4. Parse response, publish to `/target_object` and `/user_command`

**How to test:**

*Without a microphone (development machine):*
```python
# Temporarily hardcode audio or transcript in get_voice_command():
def get_voice_command(node):
    # TEMP: skip recording, test LLM extraction
    transcript = "find the red apple and bring it back"
    # ... send to Gemini ...
    return target_object, action
```

*With microphone on the robot:*
```bash
# Terminal 1 — launch real hardware (includes microphone_node)
ros2 launch tidybot_bringup real.launch.py

# Terminal 2 — run voice command
ros2 run tidybot_bringup voice_command.py

# Terminal 3 — verify output
ros2 topic echo /target_object
ros2 topic echo /user_command
```

**Environment setup:**
```bash
export GOOGLE_APPLICATION_CREDENTIALS="/path/to/service-account-key.json"
# or
export GOOGLE_CLOUD_PROJECT="your-project-id"
```

---

### Module E: `navigate_to_object.py` — Search + Approach (Implemented)

**Owner's goal:** Scan the environment to find a target object, then drive toward it and stop within grasping range.

Subscribes to `/objbbox` from the classifier and does its own depth-based 3D localization inline via `CoordConverter.pixel_to_base_link()`. Also builds an occupancy grid from depth for frontier-based exploration.

**Inputs:**
| Input | Topic/Type | Source | Available in sim? |
|-------|-----------|--------|-------------------|
| YOLO detections | `/objbbox` (`vision_msgs/Detection2DArray`) | `classifier` | Yes (run classifier) |
| Depth image | `/camera/depth/image_raw` (`sensor_msgs/Image`) | MuJoCo bridge | Yes |
| Camera intrinsics | `/camera/color/camera_info` (`sensor_msgs/CameraInfo`) | MuJoCo bridge | Yes |
| TF (odom→base_link) | TF2 | MuJoCo bridge | Yes |

**Outputs:**
| Output | Topic | Type |
|--------|-------|------|
| Base velocity | `/cmd_vel` | `geometry_msgs/Twist` |
| Camera pointing | `/camera/pan_tilt_cmd` | `Float64MultiArray` |
| Occupancy grid | `/map` | `nav_msgs/OccupancyGrid` |
| (programmatic) | `is_positioned()` returns `True` | Boolean |
| (programmatic) | `get_object_position()` returns `PointStamped` | `PointStamped` in base_link |

**State machine:** INIT → SCAN → EXPLORE → APPROACH → ALIGN → POSITIONED

**YOLO class mapping** (defined as `YOLO_CLASS_IDS` constant):
```python
{'person': '0', 'banana': '46', 'apple': '47', 'orange': '49'}
```

**How to test:**
```bash
# Terminal 1 — sim
ros2 launch tidybot_bringup sim.launch.py

# Terminal 2 — classifier (required — provides /objbbox)
ros2 run object_classification classifier

# Terminal 3 — run navigation
ros2 run tidybot_bringup navigate_to_object.py --ros-args -p target_object:=banana

# Terminal 4 — monitor
ros2 topic echo /cmd_vel
```

**Testing progressively:**
1. First, test SCAN alone — verify the robot rotates and you can see `/objbbox` detections
2. If no fruit visible, confirm the classifier is detecting the target class
3. Then test APPROACH — watch the robot drive toward the detected object
4. Then test ALIGN — verify it stops at the right distance

---

### Module F: `task1_retrieve.py` — Master State Machine (Implemented)

**Owner's goal:** Orchestrate the full task by calling all other modules in sequence.

Subscribes to `/objbbox` for YOLO detections (same pattern as `navigate_to_object.py`). Uses `NavigateToObject` for search/approach and `/plan_to_target` for arm motion.

**How to run:**
```bash
# Terminal 1 — sim
ros2 launch tidybot_bringup sim.launch.py

# Terminal 2 — classifier (required)
ros2 run object_classification classifier

# Terminal 3 — run task
ros2 run tidybot_bringup task1_retrieve.py
```

**Testing state-by-state:**

*Skip to a specific state by editing `__init__`:*
```python
# Test grasp states only
self.state = Task1State.PLAN_GRASP
self.target_object = "banana"
```

---

### Shared: `robot_helpers.py` — Utility Library

Not a ROS2 node — a Python module imported by other scripts.

**Available functions:**

| Function | Purpose |
|----------|---------|
| `call_service_sync(node, client, request, timeout)` | Blocking service call with timeout |
| `create_pose(x, y, z, qw, qx, qy, qz)` | Build a `geometry_msgs/Pose` |
| `open_gripper(publisher)` | Publish gripper open command |
| `close_gripper(publisher)` | Publish gripper close command |
| `stop_base(cmd_vel_pub)` | Publish zero velocity |

**Available constants:**

| Constant | Value | Use |
|----------|-------|-----|
| `ORIENT_FINGERS_DOWN` | `(0.5, 0.5, 0.5, -0.5)` | Quaternion wxyz for downward grasp |
| `ORIENT_FINGERS_DOWN_ROT90` | `(0.707, 0.0, 0.707, 0.0)` | Rotated downward grasp |
| `ARM_HOME` | `[0, 0, 0, 0, 0, 0]` | Home position |
| `ARM_SLEEP` | `[0, -1.76, 1.56, 0, 0.65, 0]` | Compact sleep position |
| `ARM_READY` | `[0, 0, 0.6, 0, 0.9, 0]` | Ready-to-grasp position |

---

### Shared: `CoordConverter` — Coordinate Conversion Utility

**File:** `tidybot_perception/tidybot_perception/coord_converter.py`

Wraps TF2 lookups for frame transforms and depth back-projection.

**Key methods:**

| Method | Purpose |
|--------|---------|
| `pixel_to_base_link(u, v, depth_image, camera_info, clock)` | Depth pixel → 3D PointStamped in base_link |
| `point_camera_to_base(point_stamped)` | Transform PointStamped from camera → base_link |
| `camera_to_base(pose_stamped)` | Transform PoseStamped from camera → base_link |
| `depth_pixel_to_camera_point(u, v, depth_m, fx, fy, cx, cy)` | Back-project pixel to 3D camera frame (static) |

---

## Quick Reference: Running Everything

```bash
# Step 1: Build
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tidybot_msgs tidybot_perception object_classification tidybot_bringup
source install/setup.bash

# Step 2: Launch sim
ros2 launch tidybot_bringup sim.launch.py

# Step 3: Run classifier (new terminal)
ros2 run object_classification classifier

# Step 4: Run task (new terminal)
ros2 run tidybot_bringup task1_retrieve.py
```

## Quick Reference: Useful Debug Commands

```bash
# See all active topics
ros2 topic list

# Monitor YOLO detections
ros2 topic echo /objbbox

# Check classifier publish rate
ros2 topic hz /objbbox

# Monitor occupancy grid
ros2 topic hz /map

# List available services
ros2 service list

# Monitor state machine (look at logger output)
ros2 run tidybot_bringup task1_retrieve.py  # state transitions print to console
```
