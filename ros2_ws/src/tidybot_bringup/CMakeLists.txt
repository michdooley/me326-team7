cmake_minimum_required(VERSION 3.8)
project(tidybot_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install rviz config files
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# ── Main task scripts ──
install(PROGRAMS
  scripts/task1_explore_and_find.py
  scripts/task2_pick_and_place.py
  scripts/task3_pick_and_bring.py
  scripts/voice_command.py
  scripts/speech_to_text.py
  scripts/safe_stop.py
  scripts/diagnose_dynamixel.py
  scripts/example_state_machine.py
  scripts/trajectory_tracking.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Test scripts (hardware / sim validation) ──
install(PROGRAMS
  scripts/tests/test_base_sim.py
  scripts/tests/test_camera_sim.py
  scripts/tests/test_arms_sim.py
  scripts/tests/test_grippers_sim.py
  scripts/tests/test_planner_sim.py
  scripts/tests/test_real_hardware.py
  scripts/tests/test_base_real.py
  scripts/tests/test_arms_real.py
  scripts/tests/test_grippers_real.py
  scripts/tests/test_pan_tilt_real.py
  scripts/tests/test_torque_hold.py
  scripts/tests/test_full_robot.py
  scripts/tests/test_tf.py
  scripts/tests/test_planner_real.py
  scripts/tests/test_simple_arm.py
  scripts/tests/test_motion_planner.py
  scripts/tests/test_microphone.py
  scripts/tests/test_yolo_camera.py
  scripts/tests/test_grasp_calibration.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Checkpoint scripts (older / intermediate work) ──
install(PROGRAMS
  scripts/checkpoints/objective1_task.py
  scripts/checkpoints/task1_retrieve.py
  scripts/checkpoints/explore_and_find_object.py
  scripts/checkpoints/pick_and_place.py
  scripts/checkpoints/pick_and_bring.py
  scripts/checkpoints/pick_up_block.py
  scripts/checkpoints/pick_up_block_real.py
  scripts/checkpoints/find_bin_and_drop.py
  scripts/checkpoints/navigate_to_object.py
  scripts/checkpoints/frontier_exploration.py
  scripts/checkpoints/task_navigate_to_position.py
  scripts/checkpoints/topdown_grasp_demo.py
  scripts/checkpoints/grasp_and_verify_demo.py
  scripts/checkpoints/rgbd_grasp_demo.py
  scripts/checkpoints/rgbd_grasp_demo_real.py
  scripts/checkpoints/grasp_calibration_test.py
  scripts/checkpoints/cube_calibration_test.py
  scripts/checkpoints/ik_workspace_scan.py
  scripts/checkpoints/map_arm_workspace.py
  scripts/checkpoints/grasp_reachability_map.py
  scripts/checkpoints/grasp_visualizer.py
  scripts/checkpoints/ik_scan_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install subtask modules (importable by objective1_task.py)
install(DIRECTORY scripts/subtasks
  DESTINATION lib/${PROJECT_NAME}
)

# Install shared modules (importable by task scripts)
install(DIRECTORY scripts/common
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
