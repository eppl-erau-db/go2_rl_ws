# RL Control Deployment - Unitree Go2

This repo contains the go2_rl_ws which contains real-world implementation of the trained RL control. To find the training environment through Isaac Lab, follow this repo: https://github.com/gabearod2/IsaacLab.git. Specifically, the current training environments can be found in the "main2" branch.

To setup, first install ROS2 humble, then clone this branch:
```bash
cd ~
git clone https://github.com/gabearod2/workspaces
```

First resolving dependencies:
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-rosidl-generator-dds-idl
```

Ensuring you have not sourced ROS2, compile cyclonedds:
```bash
cd ~/workspaces/lse_go2_ws/src/unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b humble
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd ..
colcon build --packages-select cyclonedds
```

Then, compile unitree_go, unitree_api, rl_deploy, and unitree_ros2_python packages:
```bash
cd ~/workspaces/lse_go2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select unitree_go
colcon build --packages-select unitree_api
colcon build --packages-select rl_deploy
colcon build --packages-select unitree_ros2_python --symlink-install #symlink install for ease of editing
```

Connecting the ethernet cord to the quadruped, get the name of the connection:
```bash
ifconfig
```

After using ifconfig to get name of connection, edit setup.sh file, using enp114s0 as an example:
```bash
sudo gedit ~/workspaces/lse_go2_ws/src/unitree_ros2/setup.sh
```
```bash
#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/humble/setup.bash
source ~/workspaces/lse_go2_ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp114s0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
```

Finally, restart your pc.

To deploy, open a terminal and source unitree_ros and lse_go2_ws:
```bash
source ~/workspaces/lse_go2_ws/src/unitree_ros2/setup.sh &&
source ~/workspaces/lse_go2_ws/install/setup.sh &&
cd ~/workspaces/lse_go2_ws
```

Then, to test:
```bash
ros2 run unitree_ros2_python go2_base_vel
ros2 run unitree_ros2_python go2_joint_pos_vel
ros2 run unitree_ros2_python go2_projected_gravity
ros2 run unitree_ros2_python go2_velocity_commands
ros2 run unitree_ros2_python go2_rl_actions
```
