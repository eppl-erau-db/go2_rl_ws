cmake_minimum_required(VERSION 3.8)
project(blind_locomotion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Button.msg"
)

# Install Python modules - use install directly instead of ament_python_install_package
# to avoid conflicts with rosidl_generate_interfaces
install(DIRECTORY
  blind_locomotion/
  DESTINATION lib/python3/dist-packages/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# Install Python executables
install(PROGRAMS
  blind_locomotion/rl_actions.py
  blind_locomotion/rl_reach_actions.py
  blind_locomotion/controller_commands.py
  blind_locomotion/test_action_publisher.py
  blind_locomotion/keyboard_buttons.py
  blind_locomotion/lowcmd_decoder.py
  blind_locomotion/lowstate_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install model files
install(DIRECTORY share/models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
