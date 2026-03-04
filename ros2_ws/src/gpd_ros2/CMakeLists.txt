cmake_minimum_required(VERSION 3.8)
project(gpd_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(pcl_conversions REQUIRED)

# Find non-ROS dependencies
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraspConfig.msg"
  "msg/GraspConfigList.msg"
  "msg/CloudSources.msg"
  "srv/DetectGrasps.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# Find GPD library (check local build first, then system install)
set(GPD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." CACHE PATH "Root of the project repo")
find_library(GPD_LIB gpd HINTS
  "${GPD_ROOT}/third_party/gpd/build"
  /usr/local/lib
  ${CMAKE_INSTALL_PREFIX}/lib
)
find_path(GPD_INCLUDE_DIR gpd/grasp_detector.h HINTS
  "${GPD_ROOT}/third_party/gpd/include"
  /usr/local/include
  ${CMAKE_INSTALL_PREFIX}/include
)

if(GPD_LIB AND GPD_INCLUDE_DIR)
  message(STATUS "GPD library found: ${GPD_LIB}")
  message(STATUS "GPD include dir: ${GPD_INCLUDE_DIR}")

  # GPD detection service node
  add_executable(gpd_detect_grasps_server src/gpd_ros2_server.cpp)
  target_include_directories(gpd_detect_grasps_server PRIVATE
    include
    ${GPD_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(gpd_detect_grasps_server
    ${GPD_LIB}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
  )
  ament_target_dependencies(gpd_detect_grasps_server
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    pcl_conversions
  )

  # Link with generated messages
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(gpd_detect_grasps_server "${cpp_typesupport_target}")

  # Install executable
  install(TARGETS gpd_detect_grasps_server
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING "GPD library not found. Only building messages/services. "
    "Install GPD from https://github.com/atenpas/gpd and run 'sudo make install'")
endif()

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
