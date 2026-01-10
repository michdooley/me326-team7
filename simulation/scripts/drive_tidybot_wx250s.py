#!/usr/bin/env python3
"""
TidyBot2 with Bimanual WX250S Arms - MuJoCo Simulation Demo

This script demonstrates how to:
1. Load the TidyBot2 mobile robot with dual WX250S 6-DOF arms in MuJoCo
2. Control the mobile base (x, y, rotation)
3. Control both arm joints (left/right: waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate)
4. Control both grippers (open/close)

The demo drives the robot forward 1 meter while performing a synchronized arm wave motion.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
from pathlib import Path


def main():
    # ==========================================================================
    # STEP 1: Load the MuJoCo model
    # ==========================================================================
    # The scene XML file includes the robot, floor, and lighting
    # Get path relative to this script's location
    script_dir = Path(__file__).parent
    model_path = script_dir / "../assets/mujoco/scene_wx250s_bimanual.xml"
    model_path = model_path.resolve()  # Convert to absolute path
    model = mujoco.MjModel.from_xml_path(str(model_path))  # Static model definition
    data = mujoco.MjData(model)  # Dynamic simulation state (positions, velocities, etc.)

    # ==========================================================================
    # STEP 2: Get joint IDs for reading robot state
    # ==========================================================================
    # Joints let us READ the current position/velocity of each degree of freedom.
    # Use data.qpos[joint_id] to get the current joint position.
    
    # --- Mobile Base Joints ---
    # The base has 3 degrees of freedom: x position, y position, and rotation (theta)
    base_joint_x = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint_x")      # Forward/backward (meters)
    base_joint_y = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint_y")      # Left/right strafe (meters)
    base_joint_theta = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint_th") # Rotation (radians)

    # ==========================================================================
    # STEP 3: Get actuator IDs for controlling the robot
    # ==========================================================================
    # Actuators let us COMMAND the robot to move. Set data.ctrl[actuator_id] to
    # send a control signal. For position-controlled joints, this is the target position.
    
    # --- Mobile Base Actuators ---
    # These control the base position directly (position control mode)
    base_ctrl_x = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "joint_x")      # Target x position (meters)
    base_ctrl_y = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "joint_y")      # Target y position (meters)
    base_ctrl_theta = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "joint_th") # Target rotation (radians)

    # --- Right WX250S Arm Actuators (6 joints) ---
    # The arm has 6 degrees of freedom, from base to end-effector:
    right_arm_ctrl_waist = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_waist")            # Rotates entire arm left/right
    right_arm_ctrl_shoulder = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_shoulder")      # Raises/lowers upper arm
    right_arm_ctrl_elbow = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_elbow")            # Bends the elbow
    right_arm_ctrl_forearm_roll = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_forearm_roll")  # Rotates forearm
    right_arm_ctrl_wrist_angle = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_wrist_angle")    # Bends wrist up/down
    right_arm_ctrl_wrist_rotate = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_wrist_rotate")  # Rotates wrist

    # --- Right Gripper Actuator ---
    right_gripper_ctrl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_fingers_actuator")  # 0 = open, 255 = closed

    # --- Left WX250S Arm Actuators (6 joints) ---
    left_arm_ctrl_waist = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_waist")
    left_arm_ctrl_shoulder = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_shoulder")
    left_arm_ctrl_elbow = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_elbow")
    left_arm_ctrl_forearm_roll = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_forearm_roll")
    left_arm_ctrl_wrist_angle = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_wrist_angle")
    left_arm_ctrl_wrist_rotate = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_wrist_rotate")

    # --- Left Gripper Actuator ---
    left_gripper_ctrl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_fingers_actuator")

    # ==========================================================================
    # STEP 4: Initialize robot to home position
    # ==========================================================================
    mujoco.mj_resetData(model, data)

    # Set initial joint positions (data.qpos stores all joint positions)
    # Order matches the order joints are defined in the XML
    home_qpos = [
        # Mobile base (3 DOF)
        0, 0, 0,              # x=0m, y=0m, theta=0rad (at origin, facing forward)
        # Right WX250S arm (6 DOF)
        0, 0, 0, 0, 0, 0,     # All arm joints at zero (straight up position)
        # Right gripper fingers (multiple joints for finger segments)
        0, 0, 0, 0, 0, 0, 0, 0,
        # Left WX250S arm (6 DOF)
        0, 0, 0, 0, 0, 0,
        # Left gripper fingers
        0, 0, 0, 0, 0, 0, 0, 0
    ]
    data.qpos[:len(home_qpos)] = home_qpos

    # Set initial control targets to match current position (no movement yet)
    # --- Base controls ---
    data.ctrl[base_ctrl_x] = 0.0
    data.ctrl[base_ctrl_y] = 0.0
    data.ctrl[base_ctrl_theta] = 0.0

    # --- Right arm controls (all at zero = home position) ---
    data.ctrl[right_arm_ctrl_waist] = 0.0
    data.ctrl[right_arm_ctrl_shoulder] = 0.0
    data.ctrl[right_arm_ctrl_elbow] = 0.0
    data.ctrl[right_arm_ctrl_forearm_roll] = 0.0
    data.ctrl[right_arm_ctrl_wrist_angle] = 0.0
    data.ctrl[right_arm_ctrl_wrist_rotate] = 0.0
    data.ctrl[right_gripper_ctrl] = 0.0  # 0 = fully open

    # --- Left arm controls (all at zero = home position) ---
    data.ctrl[left_arm_ctrl_waist] = 0.0
    data.ctrl[left_arm_ctrl_shoulder] = 0.0
    data.ctrl[left_arm_ctrl_elbow] = 0.0
    data.ctrl[left_arm_ctrl_forearm_roll] = 0.0
    data.ctrl[left_arm_ctrl_wrist_angle] = 0.0
    data.ctrl[left_arm_ctrl_wrist_rotate] = 0.0
    data.ctrl[left_gripper_ctrl] = 0.0  # 0 = fully open

    # Compute forward kinematics to update all derived quantities
    mujoco.mj_forward(model, data)

    # ==========================================================================
    # STEP 5: Set up the demo motion
    # ==========================================================================
    print("Starting TidyBot2 with bimanual WX250S arms simulation...")
    print("Initial position: x={:.3f}m, y={:.3f}m, theta={:.3f}rad".format(
        data.qpos[base_joint_x], data.qpos[base_joint_y], data.qpos[base_joint_theta]))

    # Target: drive the base 1 meter forward in the x direction
    target_x_position = 1.0

    # ==========================================================================
    # STEP 6: Run the simulation with visualization
    # ==========================================================================
    # launch_passive creates a viewer that doesn't block - we control the sim loop
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Configure the camera for a good view of the robot
        viewer.cam.distance = 3.0      # Distance from target point (meters)
        viewer.cam.elevation = -20     # Angle above/below horizon (degrees)
        viewer.cam.azimuth = 90        # Rotation around vertical axis (degrees)

        # Disable transparency for clearer visualization
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False

        # Track simulation state
        start_time = time.time()
        base_target_reached = False
        arm_demo_complete = False

        print(f"Driving forward to x={target_x_position}m while moving arm...")

        # ----------------------------------------------------------------------
        # Main simulation loop - runs until you close the viewer window
        # ----------------------------------------------------------------------
        while viewer.is_running():
            step_start = time.time()
            elapsed_time = time.time() - start_time

            # ==================================================================
            # CONTROL THE MOBILE BASE
            # ==================================================================
            # Set the target position for the base. The actuators use position
            # control, so setting ctrl to 1.0 means "move to x=1.0 meters".
            data.ctrl[base_ctrl_x] = target_x_position  # Drive forward to 1 meter
            data.ctrl[base_ctrl_y] = 0.0                # No lateral movement
            data.ctrl[base_ctrl_theta] = 0.0            # No rotation

            # ==================================================================
            # CONTROL BOTH ARMS - Demo: Raise arms, wave, return home
            # ==================================================================
            # Arm joint values are in radians. Positive/negative depends on
            # the joint's axis definition in the URDF/XML.

            if elapsed_time < 5.0:
                # Phase 1 (0-5 sec): Raise both arms into a bent position
                # Right arm
                data.ctrl[right_arm_ctrl_shoulder] = -0.5      # Raise shoulder (negative = up)
                data.ctrl[right_arm_ctrl_elbow] = 1.0          # Bend elbow
                data.ctrl[right_arm_ctrl_wrist_angle] = -0.5   # Angle wrist
                # Left arm (mirrored)
                data.ctrl[left_arm_ctrl_shoulder] = -0.5
                data.ctrl[left_arm_ctrl_elbow] = 1.0
                data.ctrl[left_arm_ctrl_wrist_angle] = -0.5

            elif elapsed_time < 10.0:
                # Phase 2 (5-10 sec): Wave by oscillating the waist joints
                # np.sin creates smooth back-and-forth motion
                wave_angle = np.sin(elapsed_time * 2) * 0.5  # Oscillate ±0.5 rad
                # Right arm
                data.ctrl[right_arm_ctrl_waist] = wave_angle
                data.ctrl[right_arm_ctrl_shoulder] = -0.5
                data.ctrl[right_arm_ctrl_elbow] = 1.0
                data.ctrl[right_arm_ctrl_wrist_angle] = -0.5
                # Left arm (wave in opposite phase for variety)
                data.ctrl[left_arm_ctrl_waist] = -wave_angle
                data.ctrl[left_arm_ctrl_shoulder] = -0.5
                data.ctrl[left_arm_ctrl_elbow] = 1.0
                data.ctrl[left_arm_ctrl_wrist_angle] = -0.5

            elif not arm_demo_complete:
                # Phase 3 (after 10 sec): Return all arm joints to home position
                # Right arm
                data.ctrl[right_arm_ctrl_waist] = 0.0
                data.ctrl[right_arm_ctrl_shoulder] = 0.0
                data.ctrl[right_arm_ctrl_elbow] = 0.0
                data.ctrl[right_arm_ctrl_wrist_angle] = 0.0
                # Left arm
                data.ctrl[left_arm_ctrl_waist] = 0.0
                data.ctrl[left_arm_ctrl_shoulder] = 0.0
                data.ctrl[left_arm_ctrl_elbow] = 0.0
                data.ctrl[left_arm_ctrl_wrist_angle] = 0.0
                arm_demo_complete = True
                print("Bimanual arm demo complete!")

            # ==================================================================
            # STEP THE SIMULATION
            # ==================================================================
            # This advances physics by one timestep (defined in model.opt.timestep)
            mujoco.mj_step(model, data)

            # Sync the viewer to show the updated state
            viewer.sync()

            # ==================================================================
            # CHECK IF BASE REACHED TARGET
            # ==================================================================
            current_x = data.qpos[base_joint_x]  # Read current x position
            if not base_target_reached and abs(current_x - target_x_position) < 0.01:
                print(f"Base target reached! Final position: x={current_x:.3f}m")
                print(f"Time taken: {time.time() - start_time:.2f}s")
                print("Close the viewer window to exit.")
                base_target_reached = True

            # ==================================================================
            # MAINTAIN REAL-TIME SIMULATION SPEED
            # ==================================================================
            # Sleep to match real time (prevents simulation from running too fast)
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    print("Simulation complete!")

if __name__ == "__main__":
    main()
