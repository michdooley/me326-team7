# GPD (Grasp Pose Detection) Setup Guide

This document provides step-by-step instructions to install and integrate the GPD (Grasp Pose Detection) library with TidyBot2's ROS2 grasp planner.

## Overview

**GPD** is a state-of-the-art library for detecting 6-DOF grasp poses for 2-finger parallel-jaw grippers in 3D point clouds. Key features:

- Works with novel objects (no CAD models required)
- Works in dense clutter
- Outputs full 6-DOF grasp poses (not just top-down grasps)
- Supports multiple CNN frameworks (OpenVINO, Caffe, Eigen)
- Runtime: ~1-2 seconds per 10k-point cloud

**References:**
- Main library: https://github.com/atenpas/gpd
- ROS wrapper: https://github.com/atenpas/gpd_ros
- Paper: [Grasp Pose Detection in Point Clouds](http://arxiv.org/abs/1706.09911)

---

## Step 1: Install System Dependencies

```bash
# Update apt
sudo apt update

# Install core dependencies for GPD
sudo apt install -y \
    libpcl-dev \
    libeigen3-dev \
    libopencv-dev \
    cmake \
    build-essential \
    git
```

### Optional: OpenVINO (recommended for speed)

OpenVINO provides faster inference than pure Caffe or Eigen. Install the open-source version:

```bash
# Download OpenVINO (check releases for latest version)
cd ~/Downloads
wget https://github.com/openvinotoolkit/openvino/releases/download/2023.2.0/l_openvino_toolkit_ubuntu22_2023.2.0.13089.cfd42bd2cb0_x86_64.tgz

# Extract and setup
tar -xzf l_openvino_toolkit_ubuntu22_2023.2.0*.tgz
cd l_openvino_toolkit_ubuntu22_2023.2.0*
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install

# Add to environment
echo "source /opt/intel/openvino_2023/bin/setupvars.sh" >> ~/.bashrc
source ~/.bashrc
```

If OpenVINO setup is complex, skip it — GPD will fallback to Caffe or Eigen (slower but works).

---

## Step 2: Build and Install GPD Library

Clone and build the GPD library (this is a requirement for the ROS2 wrapper):

```bash
# Create a directory for GPD (outside your ROS workspace)
mkdir -p ~/grasp_libraries
cd ~/grasp_libraries

# Clone GPD repository
git clone https://github.com/atenpas/gpd.git
cd gpd

# Create build directory
mkdir -p build && cd build

# Build with CMake (without OpenVINO first, can add later)
cmake ..
make -j$(nproc)

# Install as a shared library (important for ROS2 wrapper to find it)
sudo make install
```

### Build with OpenVINO (optional, faster)

If you installed OpenVINO, rebuild GPD with it:

```bash
cd ~/grasp_libraries/gpd/build
rm CMakeCache.txt

# Set OpenVINO path
export InferenceEngine_DIR=/opt/intel/openvino_2023/runtime/cmake

# Reconfigure and build
cmake .. -DUSE_OPENVINO=ON
make -j$(nproc)
sudo make install
```

### Verify Installation

Test GPD with a sample point cloud:

```bash
cd ~/grasp_libraries/gpd

# Run on a sample PCD file
./build/detect_grasps cfg/eigen_params.cfg tutorials/krylon.pcd
```

You should see a PCL viewer with grasp poses visualized.

---

## Step 3: Configure GPD Parameters

GPD requires a configuration file. Copy the default and customize it for TidyBot2:

```bash
# Copy default config to your TidyBot workspace
cd /home/elisabeth/me326-team7/ros2_ws/src/tidybot_perception/config
cp ~/grasp_libraries/gpd/cfg/eigen_params.cfg gpd_params.cfg
```

Edit `gpd_params.cfg` to match your robot's workspace and gripper:

```bash
nano gpd_params.cfg
```

Key parameters to adjust:

| Parameter | Value | Reason |
|-----------|-------|--------|
| `workspace` | `-0.5 0.5 -0.5 0.5 0.0 1.5` | 3D cube where grasp search happens (adjust to your workspace) |
| `num_samples` | `10000` | Number of grasp candidates to generate (higher = slower but more thorough) |
| `num_threads` | `$(nproc)` | Number of CPU cores to use for parallel processing |
| `camera_position` | `0 0 0.8` | Height of your camera above base_link |
| `radius` | `0.015` | Gripper finger radius (TidyBot gripper is ~1.5cm) |
| `finger_width` | `0.02` | Distance between fingers when closed |
| `hand_outer_diameter` | `0.15` | Maximum gripper opening width |
| `min_aperture` | `0.0` | Minimum gripper opening for grasps |

For TidyBot2 with RealSense camera mounted at ~0.8m height:

```
# Workspace: cube from -0.5m to +0.5m in X/Y, 0.0m to 1.5m in Z
workspace = -0.5 0.5 -0.5 0.5 0.0 1.5

# Use all available cores
num_threads = 8

# Increase samples for better grasp quality
num_samples = 15000

# TidyBot gripper specs
radius = 0.015
finger_width = 0.02
hand_outer_diameter = 0.12
min_aperture = 0.0
max_aperture = 0.10

# Camera at ~0.8m height
camera_position = 0 0 0.8
```

### Pre-trained Models

GPD comes with pre-trained CNN models. Copy them to your workspace:

```bash
# Copy model files
mkdir -p /home/elisabeth/me326-team7/ros2_ws/src/tidybot_perception/models
cp -r ~/grasp_libraries/gpd/models/* /home/elisabeth/me326-team7/ros2_ws/src/tidybot_perception/models/

# List available models
ls -la /home/elisabeth/me326-team7/ros2_ws/src/tidybot_perception/models/
```

GPD provides models for:
- **15-channel** (default, higher quality): `single_view_15_channels.caffemodel`
- **3-channel** (faster): `single_view_3_channels.caffemodel`
- **Two-view** options with different camera angles

In the config file, point to the weight file:

```
weight_file = /home/elisabeth/me326-team7/ros2_ws/src/tidybot_perception/models/caffe/15channels/single_view_15_channels.caffemodel
model_file = /home/elisabeth/me326-team7/ros2_ws/src/tidybot_perception/models/caffe/15channels/lenet.prototxt
```

---

## Step 4: Update LD_LIBRARY_PATH (if needed)

If the ROS2 node can't find GPD libraries at runtime, add them to your environment:

```bash
# Add to ~/.bashrc
echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc

# Or temporarily for current session
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

---

## Step 5: Test GPD with Point Cloud Files

Before integrating with ROS2, test GPD on sample point clouds:

```bash
# Create a test point cloud from MuJoCo or RealSense depth image
# (see ROS2 grasp planner node for conversion code)

# Run GPD on the PCD file
~/grasp_libraries/gpd/build/detect_grasps /home/elisabeth/me326-team7/ros2_ws/src/tidybot_perception/config/gpd_params.cfg sample_cloud.pcd

# View results in PCL viewer
# - Press 'g' to show grasps
# - Press 'h' to see other commands
```

---

## Step 6: Build tidybot_gpd ROS2 Package

Once GPD is installed, build the ROS2 wrapper:

```bash
cd /home/elisabeth/me326-team7/ros2_ws
source /opt/ros/humble/setup.bash

# Build the GPD ROS2 package
colcon build --packages-select tidybot_gpd

source install/setup.bash
```

If the build fails with "GPD not found", verify:
1. GPD was installed with `sudo make install`
2. CMake can find it: `find /usr/local -name "GpdConfig.cmake" 2>/dev/null`
3. If not in `/usr/local`, update CMakeLists.txt in tidybot_gpd to point to your GPD installation

---

## Troubleshooting

### Error: "libgpd.so not found" or "GpdConfig.cmake not found"

**Solution:** GPD wasn't installed properly.

```bash
cd ~/grasp_libraries/gpd/build
sudo make install
# Check installation
find /usr/local -name "*gpd*"
# Should show /usr/local/lib/libgpd.so and /usr/local/lib/cmake/gpd/
```

### Error: "CMake can't find PCL" or "Eigen"

**Solution:** Reinstall dependencies:

```bash
sudo apt install -y libpcl-dev libeigen3-dev
```

### Error: "InferenceEngine not found" (if using OpenVINO)

**Solution:** Set the path before building:

```bash
export InferenceEngine_DIR=/opt/intel/openvino_2023/runtime/cmake
cd ~/grasp_libraries/gpd/build
cmake .. -DUSE_OPENVINO=ON
make -j$(nproc)
sudo make install
```

### Slow grasp detection (>2-3 seconds)

**Solutions:**
1. Reduce `num_samples` in config (e.g., 5000 instead of 15000)
2. Use 3-channel model instead of 15-channel (faster, lower quality)
3. Install OpenVINO for faster inference
4. Reduce point cloud size by downsampling before sending to GPD

### Grasps not found or low quality

**Solutions:**
1. Check `workspace` bounds in config — ensure they cover your target object
2. Increase `num_samples` (more candidates = better grasps)
3. Ensure point cloud has good resolution (at least 1mm accuracy)
4. Verify `camera_position` is correct (affects surface normal estimation)
5. Adjust gripper dimensions (`radius`, `finger_width`, `hand_outer_diameter`)

---

## Next Steps

Once GPD is installed and `tidybot_gpd` package builds successfully:

1. **Test the ROS2 service:**
   ```bash
   ros2 service call /plan_grasp tidybot_msgs/srv/PlanGrasp \
     "{object_position: {header: {frame_id: 'base_link'}, point: {x: 0.4, y: 0.0, z: 0.2}}, object_class: 'block', arm_name: 'right'}"
   ```

2. **Check grasp quality:**
   - Visualize output poses in RViz
   - Validate with `/plan_to_target` motion planner
   - Test on hardware with real objects

3. **Optimize for your gripper:**
   - Adjust config parameters based on success rate
   - May need to retrain CNN models on your gripper type (advanced)

---

## References

- [GPD GitHub](https://github.com/atenpas/gpd)
- [Original Paper](http://arxiv.org/abs/1706.09911)
- [OpenVINO Installation](https://docs.openvino.ai/latest/index.html)
- [PCL Documentation](https://pointclouds.org/)

