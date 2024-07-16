# RL Control Deployment - Unitree Go2

This repository contains the go2_rl_ws, allowing for the real world imeplementation of trained RL policy on the Go2, entirely through ROS2 Humble. This workspace takes the information needed for the observations from the Go2's onboard computer, and throws them into the trained RL policy. To train the RL policy, Isaac Lab was used; perfectly tuning the rewards and environment is an ongoing process. All ONNX models can be found in the share directory of the unitree_ros2_python package. To find the training environment I used through Isaac Lab, follow this repo: https://github.com/gabearod2/IsaacLab.git. Specifically, the current training environments can be found in the "main2" branch. Note that the observations have been edited to not include the height scan or the base linear velocities as these are not easily attainable in low state, which the robot has to be in to deliver low level commands. 

Before setup, ensure you have installed [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) and are familiar to connecting your system to the Go2, through ethernet, referring to [Unitree's documentation](https://support.unitree.com/home/en/developer/Quick_start.)

To setup, first install ROS2 humble, then clone this repo into your workspaces directory:
```bash
cd ~/workspaces
git clone https://github.com/gabearod2/go2_rl_ws
```

First resolving dependencies:
```bash
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

Then, compile unitree_go, unitree_api, rl_deploy, and unitree_ros2_python packages:
```bash
cd ~/workspaces/go2_rl_ws
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
source ~/workspaces/go2_rl_ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp114s0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
```

Finally, restart your pc.

To deploy, open a terminal and source unitree_ros and lse_go2_ws:
```bash
source ~/workspaces/go2_rl_ws/src/unitree_ros2/setup.sh &&
source ~/workspaces/go2_rl_ws/install/setup.sh &&
cd ~/workspaces/go2_rl_ws
```

Then, to test:
```bash
ros2 run unitree_ros2_python go2_base_vel
ros2 run unitree_ros2_python go2_joint_pos_vel
ros2 run unitree_ros2_python go2_projected_gravity
ros2 run unitree_ros2_python go2_velocity_commands
ros2 run unitree_ros2_python go2_rl_actions
```

This repository was developed entirely within Embry-Riddle's Engineering Physics Propulstion Laboratory! Check us out [here](https://daytonabeach.erau.edu/about/labs/engineering-physics-propulsion-lab). 
