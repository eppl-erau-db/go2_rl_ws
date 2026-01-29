#!/bin/bash

# Source unitree_ros2 setup
source $WORKSPACE_DIR/src/unitree_ros2/setup_local.sh

# Build ROS2 packages using colcon
cd $WORKSPACE_DIR
colcon build --packages-select unitree_api &&
colcon build --packages-select unitree_go &&
colcon build --packages-select blind_locomotion &&
colcon build --packages-select rl_deploy &&
colcon build --packages-select go2_launch

