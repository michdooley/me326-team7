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

# Install Python scripts as executables
install(PROGRAMS
  scripts/test_base_sim.py
  scripts/test_camera_sim.py
  scripts/test_arms_sim.py
  scripts/test_grippers_sim.py
  scripts/test_planner_sim.py
  scripts/test_real_hardware.py
  scripts/test_base_real.py
  scripts/test_arms_real.py
  scripts/test_grippers_real.py
  scripts/test_pan_tilt_real.py
  scripts/test_torque_hold.py
  scripts/test_full_robot.py
  scripts/test_tf.py
  scripts/test_planner_real.py
  scripts/diagnose_dynamixel.py
  scripts/example_state_machine.py
  scripts/trajectory_tracking.py
  scripts/task_navigate_to_position.py
  scripts/test_simple_arm.py
  scripts/test_motion_planner.py
  scripts/pick_up_block.py
  scripts/pick_up_block_real.py
  scripts/frontier_exploration.py
  scripts/explore_and_find_object.py
  scripts/topdown_grasp_demo.py
  scripts/voice_command.py
  scripts/speech_to_text.py
  scripts/objective1_task.py
  # scripts/test_grasp_validation.py
  scripts/grasp_and_verify_demo.py
  scripts/safe_stop.py
  scripts/find_bin_and_drop.py
  scripts/map_arm_workspace.py
  scripts/test_grasp_calibration.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install subtask modules (importable by objective1_task.py)
install(DIRECTORY scripts/subtasks
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
