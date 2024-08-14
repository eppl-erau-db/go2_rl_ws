cmake_minimum_required(VERSION 3.8)
project(go2_sdk_integration)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Specify include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/common
  ${CMAKE_CURRENT_SOURCE_DIR}/include/nlohmann
)

# Specify the node executables
add_executable(go2_ai_switcher src/go2_ai_switcher.cpp src/common/ros2_sport_client.cpp)
add_executable(go2_sport_switcher src/go2_sport_switcher.cpp src/common/ros2_sport_client.cpp)
add_executable(go2_motion_switcher_service src/go2_motion_switcher_service.cpp src/common/ros2_sport_client.cpp)
add_executable(go2_motion_shut_off src/go2_motion_shut_off.cpp)
add_executable(go2_audio_shut_off src/go2_audio_shut_off.cpp)

# Find ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# Link libraries
ament_target_dependencies(go2_ai_switcher
  rclcpp
  std_msgs
  rosbag2_cpp
  unitree_api
  unitree_go
)

ament_target_dependencies(go2_sport_switcher
  rclcpp
  std_msgs
  rosbag2_cpp
  unitree_api
  unitree_go
)
ament_target_dependencies(go2_motion_switcher_service
  rclcpp
  std_msgs
  std_srvs
  rosbag2_cpp
  unitree_api
  unitree_go
)

ament_target_dependencies(go2_motion_shut_off
  rclcpp
)

ament_target_dependencies(go2_audio_shut_off
  rclcpp
)

# Install targets
install(TARGETS
  go2_ai_switcher
  go2_sport_switcher
  go2_motion_switcher_service
  go2_audio_shut_off
  go2_motion_shut_off
  DESTINATION lib/${PROJECT_NAME})

# Testing support
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
