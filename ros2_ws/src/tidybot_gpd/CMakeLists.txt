cmake_minimum_required(VERSION 3.8)
project(tidybot_gpd)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tidybot_msgs REQUIRED)
find_package(pcl REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Find GPD library (installed via sudo make install in ~/grasp_libraries/gpd/build)
# If CMake can't find it, set this manually:
# set(GPD_DIR /path/to/gpd/install)
find_package(gpd REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Create grasp planner node executable
add_executable(grasp_planner_node
  src/grasp_planner_node.cpp
)

# Link libraries
target_link_libraries(grasp_planner_node
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gpd
)

# Link ROS2 dependencies
ament_target_dependencies(grasp_planner_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tidybot_msgs
  pcl_ros
  tf2
  tf2_geometry_msgs
)

# Install executable
install(TARGETS grasp_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
