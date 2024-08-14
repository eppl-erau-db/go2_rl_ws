#!/bin/bash

# Set the workspace directory
WORKSPACE_DIR=~/workspaces/go2_rl_ws

# Build unitree_sdk2
cd $WORKSPACE_DIR/unitree_sdk2 || exit
mkdir -p build
cd build || exit
cmake ..
make
cd ..

# Source unitree_ros2 setup
source $WORKSPACE_DIR/src/unitree_ros2/setup.sh

# Build ROS2 packages using colcon
cd $WORKSPACE_DIR || exit
colcon build --packages-select unitree_api &&
colcon build --packages-select unitree_go &&
colcon build --packages-select unitree_ros2_python --symlink-install &&
colcon build --packages-select rl_navigation --symlink-install &&
colcon build --packages-select go2_lidar_decoder --symlink-install &&
colcon build --packages-select go2_sdk_integration &&
colcon build --packages-select rl_deploy &&
colcon build --packages-select rl_deploy_nav &&
colcon build --packages-select go2_launch

echo "Setup completed successfully."

