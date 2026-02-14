cmake_minimum_required(VERSION 3.14)
project(rl_deploy)

# Standards & warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_go REQUIRED)         # your Unitree msgs (LowCmd/LowState)
find_package(rosidl_runtime_cpp REQUIRED) # fixes bounded_vector.hpp includes
find_package(blind_locomotion REQUIRED)   # for Button message

# --- Core library (pure C++ logic) ---
add_library(rl_deploy_core
  src/core/posture_monitor.cpp
  src/core/mode_state_machine.cpp
  src/core/lowcmd_builder.cpp
  src/third_party_common/motor_crc.cpp 
)

target_include_directories(rl_deploy_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party_include/common> # vendor headers (motor_crc.h)
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rl_deploy_core
  rclcpp
  std_msgs
  unitree_go
  rosidl_runtime_cpp
  blind_locomotion
)

# node executable
add_executable(go2_controller_node
  src/nodes/go2_controller_node.cpp
)

target_include_directories(go2_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/common>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(go2_controller_node rl_deploy_core)

ament_target_dependencies(go2_controller_node
  rclcpp
  geometry_msgs
  std_msgs
  unitree_go
  rosidl_runtime_cpp
  blind_locomotion
)

# install
install(TARGETS rl_deploy_core go2_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

# (Optional) export compile_commands for IntelliSense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Lint/tests (unchanged)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
