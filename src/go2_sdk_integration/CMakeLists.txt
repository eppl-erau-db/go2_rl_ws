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
include_directories(include include/common)

# Specify the node executable
add_executable(go2_motion_switcher src/go2_motion_switcher.cpp)

# Find ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# Link libraries
ament_target_dependencies(go2_motion_switcher
  rclcpp
  std_msgs
  rosbag2_cpp
)

# Install targets
install(TARGETS
  go2_motion_switcher
  DESTINATION lib/${PROJECT_NAME})

# If the motion_switcher executable is in the bin directory of your package
# Ensure it is installed correctly
install(FILES bin/motion_switcher
  DESTINATION lib/${PROJECT_NAME})

# Testing support
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
