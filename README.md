# RL Control and Navigation Deployment - Unitree Go2

## Introduction and Scope

This repository contains a ROS2 Humble workspace allowing for the real world implementation of a trained RL policy on the Go2. Currently, there is support for blind locomotion policies. To train the RL policies, [Isaac Lab](https://github.com/isaac-sim/IsaacLab) was used.

### Policy Information

Locomotion models can be found in the share directory of the blind_locomotion package. They are in the ONNX format for maximum compatibility.

### FlowChart

Here is the current configuration of the locomotion policies:

![RL Control FlowChart](https://github.com/gabearod2/go2_rl_ws/blob/main/images/RL%20CONTROL.jpeg)

## Setup

Before setup, ensure you have installed [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) and are familiar to connecting your system to the Go2, through Ethernet, referring to [Unitree's documentation](https://support.unitree.com/home/en/developer/Quick_start.).

To start, clone this repository into your ROS2 workspaces directory:
```bash
cd ~/workspaces
git clone --recurse-submodules https://github.com/eppl-erau-db/go2_rl_ws
```

Resolve the required dependencies (you can make a conda or venv env if you'd like):
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

Source ROS2 and build unitree ROS2:
```bash
source /opt/ros/humble/setup.bash
colcon build
```

Connecting the ethernet cord to the quadruped, get the name of the connection:
```bash
ifconfig
```

After using ifconfig to get name of connection, edit the unitree_ros2's setup.sh file. Using enp114s0 as an example:
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

Also using the name of the connection, you will have to set it as the paramter in the launch files, for the basic deployment example you will only have to update go2_walk_nodes_onnx.launch.py. Again using enp114s0 as an example:
```bash
gedit ~/workspaces/go2_rl_ws/src/go2_launch/launch/go2_walk_nodes_onnx.launch.py
```
```bash
def generate_launch_description():
    # Shared parameters
    shared_params = {'network_interface': "enp114s0"}  # TODO: CHANGE TO YOUR INTERFACE NAME
```

Then, make the setup.bash an executable and run it:
```bash
cd ~/workspaces/go2_rl_ws &&
chmod +x setup.bash &&
source ./setup.bash
```

Finally, restart your PC, as recommended by Unitree.

## Body Control Deployment

To deploy, ensure the quadraped is LYING DOWN with SPORT MODE OFF (do so in the app), as support for switching modes is not yet integrated. Tethering the top of the quadruped is also advised. Work is underway for better and safer testing. Here is the wireless remote control mapping:

### Wireless Remote Mapping

UP --> Stand Command

DOWN --> Sit Command

START --> Start Walking Command (Deploying RL Actions)

SELECT --> Stop Walking Command, Goes to Stand Position

LEFT JOYSTICK --> Linear Velocity Commands

RIGHT JOYSTICK --> Angular Velocity Commands

A --> Soft Abort, Similar to Damping from Unitree

B --> Kill Command, Only for EMERGENCIES

### Terminal Commands (Flat Policy)

Open a terminal, source unitree_ros and lse_go2_ws, and launch:
```bash
source ~/workspaces/go2_rl_ws/src/unitree_ros2/setup.sh &&
source ~/workspaces/go2_rl_ws/install/setup.sh &&
cd ~/workspaces/go2_rl_ws &&
ros2 launch go2_launch go2_walk_nodes_onnx.launch.py
```

Open a new terminal, and run the low command message publisher:
```bash
source ~/workspaces/go2_rl_ws/src/unitree_ros2/setup.sh &&
source ~/workspaces/go2_rl_ws/install/setup.sh &&
cd ~/workspaces/go2_rl_ws &&
ros2 run rl_deploy go2_rl_control
```
## Video of Preliminary Results

The following is a sneak peek into the longer [video](https://youtu.be/o3_ABcsxeG8) of the pronking gait that has currently been achieved:

![Video preview](https://github.com/gabearod2/go2_rl_ws/blob/main/images/pronking.gif)

## Comments and Disclaimer

This repository was developed entirely within Embry-Riddle Aeronautical University's Engineering Physics Propulsion Laboratory! Check us out [here](https://daytonabeach.erau.edu/about/labs/engineering-physics-propulsion-lab). Thank you also to the RoboVerse community!

This is an experimental code, we are not responsible for any damages! Use at your own risk.

## Training

To find the training environment I used through Isaac Lab, follow my forked [Isaac Lab repo](https://github.com/gabearod2/IsaacLab/). To edit which RL policy you use, edit go2_rl_actions.py to use a different ONNX model, ensuring it takes the same input as the current models.  

Future work is to train the quadruped in similar fashion to the following, [Extreme Parkour](https://github.com/chengxuxin/extreme-parkour.git) for the best results.
