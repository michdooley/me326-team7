# Connecting to TidyBot

### Step 1: Join Lab WiFi

Ask the teaching team for WiFi credentials and connect both your machine and the robot to the same network.

### Step 2: Copy SSH Key to Robot

```bash
ssh-copy-id locobot@100.106.67.118
```

When prompted, enter the robot password: `locobot`

This enables passwordless SSH login for future sessions.

### Step 3: SSH into Robot

```bash
ssh -X locobot@100.106.67.118
```

The `-X` flag enables X11 forwarding to display GUI applications (like RViz) from the robot on your screen.

### Step 4: Start Robot Hardware Drivers

On the robot (via SSH):

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd ~/collaborative-robotics-2026/ros2_ws
source setup_env.bash
ros2 launch tidybot_bringup robot.launch.py
```

**Tip:** Use tmux to keep this running even if you disconnect:

```bash
tmux new -s tidybot
# Run the launch command above
# Press Ctrl+b d to detach
```

### Step 5: Verify Connection from Your Machine

Open a new terminal on your machine:

```bash
source ~/.bashrc  # Load ROS_DOMAIN_ID and RMW settings
cd ~/collaborative-robotics-2026/ros2_ws
source setup_env.bash
ros2 topic list
```

You should see topics like:
- `/cmd_vel`
- `/joint_states`
- `/left_arm/command`
- `/right_arm/command`
- `/camera/rgb/image_raw`
- etc.

### Step 6: Test Communication

Try echoing a topic to verify you're receiving data:

```bash
ros2 topic echo /joint_states
```

Or publish a simple command:

```bash
# Move base forward slowly
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" -r 10
```

### Step 7: Run Test Scripts

```bash
cd ~/collaborative-robotics-2026/ros2_ws
source setup_env.bash

# Comprehensive hardware test
ros2 run tidybot_bringup test_real_hardware.py

# Or individual component tests
ros2 run tidybot_bringup test_base_sim.py
ros2 run tidybot_bringup test_arms_sim.py
```

### Step 8: Launch RViz Remotely (Optional)

View the robot state in RViz from your machine:

```bash
ros2 launch tidybot_client client_rviz.launch.py
```

## Troubleshooting

### Can't see any topics

**Check environment variables:**
```bash
echo $ROS_DOMAIN_ID  # Should be 42
echo $RMW_IMPLEMENTATION  # Should be rmw_cyclonedds_cpp
```

**Verify both machines are on the same network:**
```bash
ping <ROBOT_IP>
```

**Check if robot launch file is running:**
```bash
ssh locobot@<ROBOT_IP>
ps aux | grep ros2
```

### Topics visible but no data

**Check network firewall settings:**
```bash
sudo ufw status
```

Cyclone DDS uses UDP multicast. If your firewall is blocking it, you may need to allow it.

### SSH connection drops frequently

Use tmux on the robot to persist sessions:

```bash
ssh locobot@<ROBOT_IP>
tmux new -s robot_session
ros2 launch tidybot_bringup robot.launch.py
# Ctrl+b d to detach

# If connection drops, reconnect and reattach:
ssh locobot@<ROBOT_IP>
tmux attach -t robot_session
```

### X11 forwarding not working

**Ensure X11 forwarding is enabled on the robot:**

On the robot, edit `/etc/ssh/sshd_config`:
```bash
X11Forwarding yes
X11DisplayOffset 10
```

Then restart SSH service:
```bash
sudo systemctl restart ssh
```

**Install X11 apps on your local machine:**
```bash
sudo apt install x11-apps
xeyes  # Test X11 forwarding
```

## Key ROS2 Topics Reference

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Base velocity commands |
| `/left_arm/command` | ArmCommand | Left arm joint positions + duration |
| `/right_arm/command` | ArmCommand | Right arm joint positions + duration |
| `/left_gripper/command` | GripperCommand | Left gripper (0=open, 1=closed) |
| `/right_gripper/command` | GripperCommand | Right gripper |
| `/camera/pan_tilt` | PanTilt | Camera pan/tilt angles |
| `/joint_states` | sensor_msgs/JointState | All joint positions/velocities |
| `/camera/rgb/image_raw` | sensor_msgs/Image | RGB camera feed |
| `/camera/depth/image_raw` | sensor_msgs/Image | Depth camera feed |

## Additional Resources

- Main documentation: [README.md](../README.md)
- Detailed remote setup: [remote_setup.md](remote_setup.md)
- ROS2 Humble docs: https://docs.ros.org/en/humble/
- Cyclone DDS: https://github.com/eclipse-cyclonedds/cyclonedds
