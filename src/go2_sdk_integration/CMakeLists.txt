cmake_minimum_required(VERSION 3.5)
project(go2_sdk_integration)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Set compiler options for warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Include directories
include_directories(include)
link_directories(src)

# Add executable for the motion switcher node
add_executable(go2_motion_switcher src/go2_motion_switcher.cpp)
target_link_libraries(go2_motion_switcher ${rclcpp_LIBRARIES})

# Install the motion_switcher executable and the node executable
install(
  TARGETS go2_motion_switcher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY bin/
  DESTINATION lib/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

# Testing (if needed)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize package
ament_package()
