# TidyBot2 Project Memory

## Project: me326-team7 (Stanford Collaborative Robotics 2026)
ROS2 Jazzy (Ubuntu 24.04, Python 3.12) + MuJoCo simulation, bimanual WX250s arms.

## Critical Coordinate Frames
- **base_link**: +X = robot LEFT, **-Y = robot FORWARD**, +Z = up
  - Right arm shoulder: (-0.15, -0.12, 0.45) in base_link
  - Left arm shoulder: (+0.15, -0.12, 0.45) in base_link
- **camera_color_optical_frame**: +Z = forward (depth), +X = right, +Y = down
- **World frame** (MuJoCo XML): with base_th=π/2, robot forward = +X world
  - World → base_link: world_x = -pos_base.y, world_y = pos_base.x
- **PlanToTarget service**: poses in base_link, quaternion in wxyz format

## Python / ROS Environment
- ROS2 Jazzy, Python 3.12, system Python at /usr/bin/python3
- uv manages .venv at /home/michael-dooley/me326-team7/.venv (Python 3.12)
- setup_env.bash adds .venv/lib/python3.12/site-packages to PYTHONPATH
- Always `source ros2_ws/setup_env.bash` before running ROS2 commands
- Build with: `cd ros2_ws && colcon build`

## GraspNet Installation (grasp-v2 branch)
- Repo: ~/graspnet-baseline (cloned from graspnet/graspnet-baseline)
- Checkpoint: ~/graspnet-baseline/logs/checkpoint-rs.tar (PyTorch ZIP format, load with torch.load)
- Python deps installed in .venv: torch+cu124, torchvision, open3d, graspnetAPI, autolab_core
- C++ extensions compiled into .venv:
  - pointnet2: ~/graspnet-baseline/pointnet2/ (compiled with CUDA_HOME=/usr)
  - knn: ~/graspnet-baseline/knn/
- graspnetAPI __init__.py patched to skip broken GraspNetEval import
- CUDA toolkit: nvidia-cuda-toolkit (CUDA 12.0), GPU driver supports CUDA 13.0

## Key Files (grasp-v2 branch)
- `simulation/assets/mujoco/scene_grasp.xml` — 3 colored cubes on floor
- `ros2_ws/src/tidybot_perception/tidybot_perception/coord_converter.py` — TF2 cam→base
- `ros2_ws/src/tidybot_perception/tidybot_perception/grasp_planner_node.py` — GraspNet node
- `ros2_ws/src/tidybot_bringup/scripts/camera_scanner.py` — HSV cube detection
- `ros2_ws/src/tidybot_bringup/scripts/grasp_demo.py` — full pipeline demo

## Key Topics / Services
- `/camera/pan_tilt_cmd` — Float64MultiArray [pan, tilt], **positive tilt = look down, positive pan = look LEFT** (both axes inverted from naive expectation — verified empirically)
- `/plan_to_target` — PlanToTarget.srv (arm_name, target_pose in base_link, wxyz quat)
- `/plan_grasp` — PlanGrasp.srv (object_position PointStamped, returns grasp+pregrasp in base_link)
- `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/color/camera_info`

## Grasp Orientation (base_link frame, wxyz quaternion)
- Fingers down: (0.5, 0.5, 0.5, -0.5) — site x→ base_link -Z
- Fingers down rot90: (0.707, 0.0, 0.707, 0.0)

## Common Issues
- graspnetAPI import fails → check the try/except patch in .venv graspnetAPI/__init__.py
- CUDA extension compile needs: CUDA_HOME=/usr + venv python setup.py install
- ROS2 Jazzy (not Humble) is installed on this machine
- Camera topic: /camera/color/image_raw (not /camera/rgb/image_raw as in some docs)

## See Also
- `docs/graspnet_installation.md` — [create if needed for detailed install steps]
- `ros2_ws/src/tidybot_bringup/scripts/test_planner_sim.py` — IK planner usage reference
