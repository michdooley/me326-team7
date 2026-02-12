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
└──────┬──────────┬──────────┬──────────┬──────────┬───────────────────┘
       │          │          │          │          │
       ▼          ▼          ▼          ▼          ▼
  ┌─────────┐ ┌──────────┐ ┌────────┐ ┌────────┐ ┌─────────────┐
  │  voice   │ │navigate  │ │detector│ │object  │ │   grasp     │
  │ command  │ │to_object │ │ _node  │ │local-  │ │  planner    │
  │          │ │          │ │ (YOLO) │ │izer   │ │   _node     │
  └─────────┘ └──────────┘ └────────┘ └────────┘ └─────────────┘
       │          │  ▲          │          │  ▲          │
       │          │  │          │          │  │          │
       ▼          ▼  │          ▼          ▼  │          ▼
  ┌─────────┐ ┌──────┐    ┌────────┐ ┌────────┐    ┌─────────────┐
  │Google   │ │/cmd  │    │/camera/│ │/camera/│    │/plan_to     │
  │Cloud    │ │_vel  │    │color/  │ │depth/  │    │_target      │
  │STT +   │ │      │    │image   │ │image   │    │(existing IK)│
  │Gemini  │ └──────┘    │_raw    │ │_raw    │    └─────────────┘
  └─────────┘             └────────┘ └────────┘
```

---

## Packages & Files

### `tidybot_msgs` — Interface Definitions

All custom messages and services. **Build this first** — everything else depends on it.

| File | Type | Purpose |
|------|------|---------|
| `msg/Detection2D.msg` | Message | Single bounding box (class, confidence, x, y, w, h) |
| `msg/Detection2DArray.msg` | Message | Array of detections per camera frame |
| `msg/ObjectPose.msg` | Message | 3D position of a detected object in `base_link` frame |
| `srv/Detect.srv` | Service | On-demand detection with optional class filter |
| `srv/LocalizeObject.srv` | Service | Convert 2D bounding box → 3D position |
| `srv/PlanGrasp.srv` | Service | Object position → grasp pose + pre-grasp pose |

### `tidybot_perception` — Perception Nodes (new package)

Persistent, always-on nodes for detection, localization, and grasp planning.

| File | Node Name | Role |
|------|-----------|------|
| `detector_node.py` | `detector_node` | YOLO object detection |
| `object_localizer_node.py` | `object_localizer_node` | Depth + bbox → 3D position |
| `grasp_planner_node.py` | `grasp_planner_node` | GraspNet grasp pose prediction |

### `tidybot_bringup` — Task Scripts

| File | Role |
|------|------|
| `scripts/task1_retrieve.py` | Master state machine orchestrating the full task |
| `scripts/voice_command.py` | Audio → Google Cloud STT → Gemini → target object |
| `scripts/navigate_to_object.py` | Search + approach behavior (importable class) |
| `utilities/robot_helpers.py` | Shared helper functions (service calls, pose creation, gripper control) |
| `launch/task.launch.py` | Launches all perception nodes |

---

## Message Flow Between Modules

### Topics (continuous data streams)

```
/camera/color/image_raw ──────────► detector_node ──► /detections
                                                          │
/camera/depth/image_raw ──┐                               ▼
/camera/color/camera_info ┼──────► object_localizer ──► /object_poses
                          │                                │
                          │                                ▼
                          └──────► navigate_to_object ──► /cmd_vel
                                         │
                                         └──────────────► /camera/pan_tilt_cmd
```

| Topic | Type | Publisher | Subscriber(s) |
|-------|------|-----------|---------------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | MuJoCo bridge / RealSense | `detector_node` |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | MuJoCo bridge / RealSense | `object_localizer_node`, `navigate_to_object` |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | MuJoCo bridge / RealSense | `object_localizer_node` |
| `/detections` | `Detection2DArray` | `detector_node` | `object_localizer_node`, `navigate_to_object`, `task1_retrieve` |
| `/object_poses` | `ObjectPose` | `object_localizer_node` | `navigate_to_object`, `task1_retrieve` |
| `/target_object` | `std_msgs/String` | `voice_command` | `task1_retrieve` |
| `/user_command` | `std_msgs/String` | `voice_command` | `task1_retrieve` |
| `/cmd_vel` | `geometry_msgs/Twist` | `navigate_to_object`, `task1_retrieve` | base controller |
| `/camera/pan_tilt_cmd` | `Float64MultiArray` | `navigate_to_object` | pan-tilt controller |
| `/right_arm/cmd` | `ArmCommand` | `task1_retrieve` | arm controller |
| `/right_gripper/cmd` | `Float64MultiArray` | `task1_retrieve` | gripper controller |
| `/joint_states` | `sensor_msgs/JointState` | MuJoCo bridge / hardware | `task1_retrieve` |
| `/odom` | `nav_msgs/Odometry` | base controller | `navigate_to_object`, `task1_retrieve` |

### Services (request/response)

| Service | Type | Server | Called by |
|---------|------|--------|-----------|
| `/detect` | `Detect` | `detector_node` | `task1_retrieve` |
| `/localize_object` | `LocalizeObject` | `object_localizer_node` | `task1_retrieve` |
| `/plan_grasp` | `PlanGrasp` | `grasp_planner_node` | `task1_retrieve` |
| `/plan_to_target` | `PlanToTarget` | `motion_planner` (existing) | `task1_retrieve`, `grasp_planner_node` |
| `/microphone/record` | `AudioRecord` | `microphone_node` (existing) | `voice_command` |

---

## Development Guides by Module

Each module can be developed and tested independently. The general workflow:

1. Launch sim: `ros2 launch tidybot_bringup sim.launch.py`
2. Run your node in a second terminal
3. Fake any missing upstream inputs with `ros2 topic pub` or `ros2 service call`
4. Verify outputs with `ros2 topic echo` or check service responses

### Module A: `detector_node.py` — YOLO Object Detection

**Owner's goal:** Run YOLO on camera frames, publish bounding boxes, serve detection requests.

**Inputs you receive:**
| Input | Topic/Type | Source | Available in sim? |
|-------|-----------|--------|-------------------|
| RGB image | `/camera/color/image_raw` (`sensor_msgs/Image`) | MuJoCo bridge | Yes — already publishing |

**Outputs you produce:**
| Output | Topic or Service | Type |
|--------|-----------------|------|
| Continuous detections | `/detections` (topic) | `Detection2DArray` |
| On-demand detection | `/detect` (service) | `Detect` |

**Key implementation steps:**
1. Load YOLO model in `_load_model()` (e.g., `from ultralytics import YOLO`)
2. In `image_callback()`, convert ROS Image → OpenCV via `cv_bridge`, run inference, publish `Detection2DArray`
3. In `detect_callback()`, run inference on `self.latest_image`, optionally filter by `request.target_class`
4. Respect `self.confidence_threshold` and `self.publish_rate` parameters

**How to test:**
```bash
# Terminal 1 — sim provides camera images
ros2 launch tidybot_bringup sim.launch.py

# Terminal 2 — run your node
ros2 run tidybot_perception detector_node \
    --ros-args -p model_path:=/path/to/yolov8n.pt -p confidence_threshold:=0.3

# Terminal 3 — verify continuous detections
ros2 topic echo /detections

# Test the service
ros2 service call /detect tidybot_msgs/srv/Detect "{target_class: 'cup'}"

# Check publishing rate
ros2 topic hz /detections
```

**Sim tip:** Place objects in the MuJoCo scene by editing `simulation/assets/mujoco/scene_wx250s_bimanual.xml` or using `scene_pickup.xml` which already has a block.

---

### Module B: `object_localizer_node.py` — Depth → 3D Position

**Owner's goal:** Take a 2D bounding box + depth image → output 3D position in `base_link` frame.

**Inputs you receive:**
| Input | Topic/Type | Source | Available in sim? |
|-------|-----------|--------|-------------------|
| Depth image | `/camera/depth/image_raw` (`sensor_msgs/Image`) | MuJoCo bridge | Yes |
| Camera intrinsics | `/camera/color/camera_info` (`sensor_msgs/CameraInfo`) | MuJoCo bridge | Yes |
| 2D detections | `/detections` (`Detection2DArray`) | `detector_node` | **No — fake it** |

**Outputs you produce:**
| Output | Topic or Service | Type |
|--------|-----------------|------|
| Continuous 3D poses | `/object_poses` (topic) | `ObjectPose` |
| On-demand localization | `/localize_object` (service) | `LocalizeObject` |

**Key implementation steps:**
1. Initialize `tf2_ros.Buffer` + `TransformListener` to look up `camera_color_optical_frame → base_link`
2. In `detections_callback()`, for each detection:
   - Compute bounding box center pixel `(cx, cy)`
   - Read depth at `(cx, cy)` from `self.latest_depth`
   - Back-project to 3D: `X = (cx - cam_cx) * depth / fx`, `Y = (cy - cam_cy) * depth / fy`, `Z = depth`
   - Transform from camera frame to `base_link` using TF
   - Publish `ObjectPose`
3. Reference `utilities/align_depth_to_rgb.py` for depth alignment if needed

**How to test (without detector_node):**
```bash
# Terminal 1 — sim provides depth + camera_info
ros2 launch tidybot_bringup sim.launch.py

# Terminal 2 — run your node
ros2 run tidybot_perception object_localizer_node

# Terminal 3 — publish a FAKE detection (pretend YOLO found an object at pixel 320,240)
ros2 topic pub /detections tidybot_msgs/msg/Detection2DArray "{
  header: {frame_id: 'camera_color_optical_frame'},
  detections: [{class_name: 'block', confidence: 0.95, x: 280, y: 200, width: 80, height: 80}]
}" -r 5

# Terminal 4 — verify 3D output
ros2 topic echo /object_poses

# Test the service
ros2 service call /localize_object tidybot_msgs/srv/LocalizeObject "{
  detection: {class_name: 'block', confidence: 0.95, x: 280, y: 200, width: 80, height: 80}
}"
```

**Validation:** Point the camera at a known object in the MuJoCo scene (e.g., the block in `scene_pickup.xml`). Check if the output 3D position roughly matches the object's position in the scene XML.

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

**Validation:** Take the returned `grasp_pose` and test it with the existing motion planner:
```bash
# Use the pose from /plan_grasp response to call /plan_to_target
ros2 service call /plan_to_target tidybot_msgs/srv/PlanToTarget "{
  arm_name: 'right',
  target_pose: {position: {x: 0.4, y: 0.0, z: 0.15}, orientation: {w: 0.5, x: 0.5, y: 0.5, z: -0.5}},
  execute: true,
  duration: 2.0
}"
# Watch the arm move in MuJoCo viewer
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

*Test just the Google Cloud pipeline (no ROS2):*
```python
# Quick standalone test
from google.cloud import speech
client = speech.SpeechClient()
# ... test with a .wav file ...
```

**Environment setup:**
```bash
export GOOGLE_APPLICATION_CREDENTIALS="/path/to/service-account-key.json"
# or
export GOOGLE_CLOUD_PROJECT="your-project-id"
```

---

### Module E: `navigate_to_object.py` — Search + Approach

**Owner's goal:** Scan the environment to find a target object, then drive toward it and stop within grasping range.

**Inputs you receive:**
| Input | Topic/Type | Source | Available in sim? |
|-------|-----------|--------|-------------------|
| Detections | `/detections` (`Detection2DArray`) | `detector_node` | **No — fake it** |
| 3D object poses | `/object_poses` (`ObjectPose`) | `object_localizer_node` | **No — fake it** |
| Odometry | `/odom` (`nav_msgs/Odometry`) | base controller | Yes |
| Depth image | `/camera/depth/image_raw` (`sensor_msgs/Image`) | MuJoCo bridge | Yes |

**Outputs you produce:**
| Output | Topic | Type |
|--------|-------|------|
| Base velocity | `/cmd_vel` | `geometry_msgs/Twist` |
| Camera pointing | `/camera/pan_tilt_cmd` | `Float64MultiArray` |
| (programmatic) | `is_positioned()` returns `True` | Boolean |
| (programmatic) | `get_object_pose()` returns pose | `ObjectPose` |

**Key implementation steps:**
1. **SCAN state:** Rotate in place (`/cmd_vel` angular.z), sweep camera with `/camera/pan_tilt_cmd`. Check `/detections` each tick — if any detection matches `self.target_object`, transition to APPROACH.
2. **APPROACH state:** Use `/object_poses` to get 3D position of target. Compute heading error and distance. Publish `/cmd_vel` to drive toward object. Use depth image for obstacle avoidance (check for close obstacles in the path). Stop when within `GRASP_DISTANCE`.
3. **ALIGN state:** Fine-tune so the object is centered in view and at the right distance. Small adjustments only.
4. **POSITIONED state:** Publish zero velocity. `is_positioned()` returns `True`.

**How to test (without detector or localizer):**
```bash
# Terminal 1 — sim
ros2 launch tidybot_bringup sim.launch.py

# Terminal 2 — run your node
ros2 run tidybot_bringup navigate_to_object.py --ros-args -p target_object:=block

# Terminal 3 — fake a detection stream (object is visible)
ros2 topic pub /detections tidybot_msgs/msg/Detection2DArray "{
  header: {frame_id: 'camera_color_optical_frame'},
  detections: [{class_name: 'block', confidence: 0.9, x: 300, y: 250, width: 60, height: 60}]
}" -r 10

# Terminal 4 — fake a 3D pose (object is 1.5m ahead, slightly left)
ros2 topic pub /object_poses tidybot_msgs/msg/ObjectPose "{
  header: {frame_id: 'base_link'},
  class_name: 'block',
  position: {header: {frame_id: 'base_link'}, point: {x: 1.5, y: 0.2, z: 0.15}},
  confidence: 0.9
}" -r 10

# Watch the robot move in MuJoCo viewer
# Terminal 5 — monitor base commands
ros2 topic echo /cmd_vel
```

**Testing progressively:**
1. First, test SCAN alone — verify the robot rotates and you can trigger APPROACH by publishing a matching detection
2. Then test APPROACH — publish a fake object_pose and verify the robot drives toward it
3. Then test ALIGN — verify it stops at the right distance

---

### Module F: `task1_retrieve.py` — Master State Machine

**Owner's goal:** Orchestrate the full task by calling all other modules in sequence.

This is the **integration module** — develop it last, or develop it incrementally by testing one state at a time.

**Inputs you receive:**
All topics and services from every other module (see the full topic/service tables above).

**Outputs you produce:**
Commands to base (`/cmd_vel`), arms (`/right_arm/cmd`), grippers (`/right_gripper/cmd`), camera (`/camera/pan_tilt_cmd`).

**Key implementation steps:**
1. Each state should be thin — call a service, wait for result, transition
2. Use `robot_helpers.call_service_sync()` for service calls
3. Use `robot_helpers.open_gripper()` / `close_gripper()` / `stop_base()` for common commands
4. Import `NavigateToObject` from `navigate_to_object.py` for SEARCH/APPROACH states
5. Import `get_voice_command` from `voice_command.py` for LISTEN state

**How to test state-by-state:**

*Skip to a specific state by editing `__init__`:*
```python
# Test grasp states only
self.state = Task1State.PLAN_GRASP
self.target_object = "block"
# Fake a known object pose:
self.latest_object_pose = ObjectPose()
self.latest_object_pose.position.point.x = 0.4
self.latest_object_pose.position.point.y = 0.0
self.latest_object_pose.position.point.z = 0.15
```

*Run with all perception nodes:*
```bash
# Terminal 1
ros2 launch tidybot_bringup sim.launch.py
# Terminal 2
ros2 launch tidybot_bringup task.launch.py
# Terminal 3
ros2 run tidybot_bringup task1_retrieve.py
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

**How to import:**
```python
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'utilities'))
from robot_helpers import call_service_sync, create_pose, open_gripper
```

**Quick test:**
```bash
cd ros2_ws
source install/setup.bash
python3 -c "
import sys; sys.path.insert(0, 'src/tidybot_bringup/utilities')
from robot_helpers import create_pose, ARM_READY, ORIENT_FINGERS_DOWN
pose = create_pose(0.3, 0.0, 0.2, *ORIENT_FINGERS_DOWN)
print(f'Created pose: pos=({pose.position.x}, {pose.position.y}, {pose.position.z})')
print(f'ARM_READY: {ARM_READY}')
"
```

---

## Quick Reference: Running Everything

```bash
# Step 1: Build
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tidybot_msgs tidybot_perception tidybot_bringup
source install/setup.bash

# Step 2: Launch sim
ros2 launch tidybot_bringup sim.launch.py

# Step 3: Launch perception (new terminal)
ros2 launch tidybot_bringup task.launch.py

# Step 4: Run task (new terminal)
ros2 run tidybot_bringup task1_retrieve.py
```

## Quick Reference: Useful Debug Commands

```bash
# See all active topics
ros2 topic list

# Monitor detections
ros2 topic echo /detections

# Monitor object poses
ros2 topic echo /object_poses

# Check topic publishing rate
ros2 topic hz /detections

# List available services
ros2 service list

# Test a service
ros2 service call /detect tidybot_msgs/srv/Detect "{target_class: ''}"

# Monitor state machine (look at logger output)
ros2 run tidybot_bringup task1_retrieve.py  # state transitions print to console
```
