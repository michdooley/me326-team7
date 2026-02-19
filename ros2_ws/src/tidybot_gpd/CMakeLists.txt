cmake_minimum_required(VERSION 3.8)
project(tidybot_gpd)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tidybot_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find PCL manually (system library, not ROS2 package)
find_library(PCL_COMMON_LIB pcl_common PATHS /usr/lib/aarch64-linux-gnu)
find_library(PCL_FILTERS_LIB pcl_filters PATHS /usr/lib/aarch64-linux-gnu)
find_library(PCL_KDTREE_LIB pcl_kdtree PATHS /usr/lib/aarch64-linux-gnu)
find_library(PCL_SEGMENTATION_LIB pcl_segmentation PATHS /usr/lib/aarch64-linux-gnu)
find_library(PCL_IO_LIB pcl_io PATHS /usr/lib/aarch64-linux-gnu)
find_library(PCL_FEATURES_LIB pcl_features PATHS /usr/lib/aarch64-linux-gnu)

set(PCL_INCLUDE_DIRS /usr/include/pcl-1.12)
set(PCL_LIBRARIES 
  ${PCL_COMMON_LIB}
  ${PCL_FILTERS_LIB}
  ${PCL_KDTREE_LIB}
  ${PCL_SEGMENTATION_LIB}
  ${PCL_IO_LIB}
  ${PCL_FEATURES_LIB}
)

# Find GPD library (installed manually to /usr/local)
find_path(GPD_INCLUDE_DIR gpd/grasp_detector.h PATHS /usr/local/include)
find_library(GPD_LIBRARY NAMES gpd PATHS /usr/local/lib)

if(NOT GPD_LIBRARY OR NOT GPD_INCLUDE_DIR)
  message(FATAL_ERROR "GPD library not found. Please check /usr/local/lib/libgpd.so and /usr/local/include/gpd/")
endif()

message(STATUS "Found GPD: ${GPD_LIBRARY}")
message(STATUS "Found GPD include: ${GPD_INCLUDE_DIR}")

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GPD_INCLUDE_DIR}/..
)

# VTK headers (PCL visualizer requires vtkSmartPointer.h)
include_directories(/usr/include/vtk-9.1)

# ROS2 pcl_conversions headers
find_package(pcl_conversions REQUIRED)
include_directories(/opt/ros/humble/include)

# Some ROS header packages (pcl_conversions) install nested include folders.
# Add the specific pcl_conversions include dir so <pcl_conversions/pcl_conversions.h>
# resolves correctly.
include_directories(/opt/ros/humble/include/pcl_conversions)

# Some message packages like pcl_msgs also install nested include folders
include_directories(/opt/ros/humble/include/pcl_msgs)

# Create grasp planner node executable
add_executable(grasp_planner_node
  src/grasp_planner_node.cpp
)

# Link libraries
target_link_libraries(grasp_planner_node
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GPD_LIBRARY}
)

# Ensure target picks up ROS include subdir for pcl_conversions
target_include_directories(grasp_planner_node PRIVATE /opt/ros/humble/include/pcl_conversions)
target_include_directories(grasp_planner_node PRIVATE /opt/ros/humble/include/pcl_msgs)

# Link ROS2 dependencies
ament_target_dependencies(grasp_planner_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tidybot_msgs
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
