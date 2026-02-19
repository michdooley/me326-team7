# GPD Grasp Planner - Quick Start Checklist

Use this checklist to set up the GPD (Grasp Pose Detection) grasp planner for TidyBot2.

**Time estimate:** 30-45 minutes (plus GPD compilation time: 10-15 minutes)

---

## Phase 1: Install GPD Library (one-time setup)

**Estimated time: 15 minutes + compile time**

- [ ] Install system dependencies
  ```bash
  sudo apt update
  sudo apt install -y libpcl-dev libeigen3-dev libopencv-dev cmake build-essential git
  ```

- [ ] Clone and build GPD
  ```bash
  mkdir -p ~/grasp_libraries
  cd ~/grasp_libraries
  git clone https://github.com/atenpas/gpd.git
  cd gpd && mkdir build && cd build
  cmake ..
  make -j$(nproc)
  ```

- [ ] **IMPORTANT:** Install GPD as a library
  ```bash
  sudo make install
  ```

- [ ] Verify installation
  ```bash
  find /usr/local -name "libgpd.so"  # Should find /usr/local/lib/libgpd.so
  ```

---

## Phase 2: Setup TidyBot2 ROS2 Package

**Estimated time: 5 minutes**

- [ ] Copy GPD pre-trained models to TidyBot workspace
  ```bash
  mkdir -p /home/elisabeth/me326-team7/ros2_ws/src/tidybot_perception/models
  cp -r ~/grasp_libraries/gpd/models/* /home/elisabeth/me326-team7/ros2_ws/src/tidybot_perception/models/
  ```

- [ ] Verify config files exist
  ```bash
  ls -la /home/elisabeth/me326-team7/ros2_ws/src/tidybot_gpd/config/gpd_params.cfg
  # Should exist and have path references to the models directory above
  ```

---

## Phase 3: Build ROS2 Packages

**Estimated time: 5-10 minutes**

- [ ] Build tidybot_gpd package
  ```bash
  cd /home/elisabeth/me326-team7/ros2_ws
  source /opt/ros/humble/setup.bash
  colcon build --packages-select tidybot_gpd
  ```

- [ ] Check for build errors
  ```bash
  # If compilation fails with "gpd not found", see troubleshooting section
  ```

- [ ] Source the build
  ```bash
  source install/setup.bash
  ```

---

## Phase 4: Test Grasp Planner Standalone

**Estimated time: 5-10 minutes**

- [ ] Terminal 1 — Start MuJoCo simulator
  ```bash
  cd /home/elisabeth/me326-team7/ros2_ws
  source setup_env.bash
  ros2 launch tidybot_bringup sim.launch.py show_mujoco_viewer:=true use_rviz:=false
  ```

- [ ] Terminal 2 — Launch grasp planner
  ```bash
  source install/setup.bash
  ros2 launch tidybot_gpd grasp_planner.launch.py
  ```

- [ ] Terminal 3 — Call grasp planning service
  ```bash
  sleep 2  # Wait for planner to initialize
  
  ros2 service call /plan_grasp tidybot_msgs/srv/PlanGrasp \
    "{object_position: {header: {frame_id: 'base_link'}, point: {x: 0.4, y: 0.0, z: 0.2}}, object_class: 'block', arm_name: 'right'}"
  ```

- [ ] Verify response
  - [ ] `success: true`
  - [ ] `grasp_pose` has valid position and orientation
  - [ ] `pre_grasp_pose` is ~0.1m above grasp_pose (adjust via approach_offset param)
  - [ ] `arm_used: right`
  - [ ] `grasp_width: 0.08`

---

## Troubleshooting

### Build Error: "Could not find gpd"

**Solution:** GPD library wasn't installed properly.

```bash
# Verify GPD was installed:
ls -la /usr/local/lib/libgpd.so
ls -la /usr/local/lib/cmake/gpd/

# If missing, reinstall:
cd ~/grasp_libraries/gpd/build
sudo make install
```

### Build Error: "pcl/common/time.h: No such file"

**Solution:** PCL development headers not installed.

```bash
sudo apt install -y libpcl-dev
# Then rebuild
colcon build --packages-select tidybot_gpd
```

### Service Call Hangs or Returns "No point cloud received"

**Cause:** Point cloud is not being published.

**Solution:**
```bash
# Check if publisher exists
ros2 topic list | grep cloud

# If missing, ensure your navigator publishes:
ros2 topic pub /camera/depth/cloud sensor_msgs/PointCloud2 "{...}"

# Or monitor what's being published:
ros2 topic echo /camera/depth/cloud
```

### "No grasps found in point cloud"

**Cause:** Scene doesn't contain graspable objects in expected workspace.

**Solutions:**
1. Check workspace bounds in `gpd_params.cfg`
2. Verify object is in the workspace region
3. Increase `num_samples` in config
4. Ensure point cloud has good quality (>1000 points per object)

### Grasp detection is very slow (>3 seconds)

**Solutions:**
1. Reduce `num_samples`: `num_samples = 5000` (instead of 15000)
2. Use 3-channel model (faster, lower quality)
3. Reduce workspace size
4. Enable GPU if available: `use_gpu = 0`

---

## Useful Commands

```bash
# Monitor grasp planner logs
ros2 topic echo /rosout | grep "GraspPlannerNode"

# Check service is available
ros2 service list | grep plan_grasp

# Get service definition
ros2 service type /plan_grasp

# Record point clouds for offline testing
ros2 bag record /camera/depth/cloud

# Test grasp planning offline
~/grasp_libraries/gpd/build/detect_grasps \
  /home/elisabeth/me326-team7/ros2_ws/src/tidybot_gpd/config/gpd_params.cfg \
  test_cloud.pcd
```

---

## Notes

- **Full Setup Guide:** See [gpd_setup.md](./gpd_setup.md)
- **Integration Guide:** See [gpd_integration.md](./gpd_integration.md)
- **Task 1 Architecture:** See [task1_architecture.md](./task1_architecture.md)

---

Done! You now have a working GPD grasp planner. Next steps:
1. Integrate into your Task 1 state machine
2. Test with your actual objects
3. Optimize gpd_params.cfg for your use case

