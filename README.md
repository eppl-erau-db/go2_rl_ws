# RL Control Deployment - Unitree Go2

## Introduction

This repository contains a ROS2 Humble workspace allowing for the real world implementation of a trained RL policy on the Go2. To train the RL policy, [Isaac Lab](https://github.com/isaac-sim/IsaacLab) was used, and all ONNX models can be found in the share directory of the unitree_ros2_python package. Note that the observations tensor has been edited to not include the height scan or the base linear velocities as these are not easily attainable in low state, which the robot has to be in to deliver low level commands. Development is currently underway in LiDAR decoding for height map information and sensor fusion for linear velocity information. Here is the current configuration:

![RL Control FlowChart](https://github.com/gabearod2/go2_rl_ws/blob/main/images/RL_CONTROL.jpeg)

## Setup

Before setup, ensure you have installed [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) and are familiar to connecting your system to the Go2, through ethernet, referring to [Unitree's documentation](https://support.unitree.com/home/en/developer/Quick_start.).

To start, clone this repository into your ROS2 workspaces directory:
```bash
cd ~/workspaces
git clone --recurse-submodules https://github.com/gabearod2/go2_rl_ws
```

Resolving dependencies:
```bash
pip install onnxruntime-gpu # or onnxruntime-cpu
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-rosidl-generator-dds-idl
```

Ensuring you have not sourced ROS2, compile cyclonedds:
```bash
cd ~/workspaces/go2_rl_ws/src/unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b humble
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd ..
colcon build --packages-select cyclonedds
```

Source ros and build unitree ROS2:
```bash
source /opt/ros/humble/setup.bash
colcon build
```

Connecting the ethernet cord to the quadruped, get the name of the connection:
```bash
ifconfig
```

After using ifconfig to get name of connection, edit setup.sh file, using enp114s0 as an example:
```bash
sudo gedit ~/workspaces/go2_rl_ws/src/unitree_ros2/setup.sh
```
```bash
#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/humble/setup.bash
source ~/workspaces/go2_rl_ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp114s0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
```

Then, compile unitree_go, unitree_api, rl_deploy, go2_launch, and unitree_ros2_python packages:
```bash
cd ~/workspaces/go2_rl_ws
source ~/workspaces/go2_rl_ws/src/unitree_ros2/setup.sh
colcon build --packages-select unitree_go &&
colcon build --packages-select unitree_api &&
colcon build --packages-select rl_deploy &&
colcon build --packages-select unitree_ros2_python --symlink-install &&
colcon build --packages-select go2_launch
```

Finally, restart your pc, as recommended by Unitree.

## Deployment

To deploy, ensure the quadraped is LYING DOWN with SPORT MODE OFF (do so in the app), as support for switching modes is not yet integrated. 

Open a terminal, source unitree_ros and lse_go2_ws, and launch:
```bash
source ~/workspaces/go2_rl_ws/src/unitree_ros2/setup.sh &&
source ~/workspaces/go2_rl_ws/install/setup.sh &&
cd ~/workspaces/go2_rl_ws &&
ros2 launch go2_launch run_obs_nodes.launch.py
```

Open a new terminal, and run the low command message publisher:
```bash
source ~/workspaces/go2_rl_ws/src/unitree_ros2/setup.sh &&
source ~/workspaces/go2_rl_ws/install/setup.sh &&
cd ~/workspaces/go2_rl_ws &&
ros2 run rl_deploy go2_rl_control
```

This repository was developed entirely within Embry-Riddle Aeronautical University's Engineering Physics Propulsion Laboratory! Check us out [here](https://daytonabeach.erau.edu/about/labs/engineering-physics-propulsion-lab). 

This is an experimental code, we are not responsible for any damages! Use at your own risk.

## Training

To find the training environment I used through Isaac Lab, follow my forked [Isaac Lab repo](https://github.com/gabearod2/IsaacLab/tree/rl_deployment). Specifically, the current training environments can be found in the "rl_deployment" branch. To edit which onnx model you use, edit go2_rl_actions to use a different ONNX model, ensuring it takes the same input as the current models.  
