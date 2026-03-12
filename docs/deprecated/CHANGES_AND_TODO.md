# Drop Item: Summary for Debugging

## What’s Done

**New script: `ros2_ws/src/tidybot_bringup/scripts/drop_item_sim.py`**

- **Goal:** Assume arm start pose is unknown and the gripper already has an item; extend the arm and drop the item into the bin.
- **Behavior:**
  1. Sends **right** arm to a fixed “over bin” pose: `[0.0, 0.5, 0.6, 0.0, -0.5, 0.0]` rad  
     `[waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]`
  2. After 2.5 s, opens the right gripper (publishes `0.0`) to drop the item.

- **Topics:**
  - `/right_arm/cmd` (ArmCommand: joint_positions + duration)
  - `/right_gripper/cmd` (Float64MultiArray: one float, 0 = open, 1 = closed)

- **Status:** Script exists but is **not** in the install list. It was removed from `CMakeLists.txt` after errors were reported when running the sim launch (exact error not captured). The launch file does **not** start this script; it only runs the MuJoCo bridge, arm controllers, motion planner, and RViz.

---

## What to Do (Debug / Finish)

1. **Re-add to install**  
   In `ros2_ws/src/tidybot_bringup/CMakeLists.txt`, add `scripts/drop_item_sim.py` to the `install(PROGRAMS ...)` block so you can run:
   ```bash
   ros2 run tidybot_bringup drop_item_sim.py
   ```

2. **Run and debug**  
   - Terminal 1: `ros2 launch tidybot_bringup sim.launch.py`  
   - Terminal 2: `ros2 run tidybot_bringup drop_item_sim.py` (after rebuild)  
   If the launch fails, capture the full error and fix the cause.

3. **Tuning (optional)**  
   - Change bin height/position by editing `DROP_POSITION` in the script.  
   - Use left arm by changing `arm="right"` to `arm="left"` in `main()`.
