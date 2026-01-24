# Unitree Go2 RL Deployment

## Introduction and Scope

A ROS2 Humble workspace allowing for the real world implementation of a RL-trained blind locomotion policies on the Go2. Deploying policies on a real robot is risky, deploy at your own risk. Locomotion models can be found in the share directory of the `blind_locomotion/share/models` package. They are in the ONNX format for maximum compatibility and were trained using Isaac Lab.

## Setup

Before setup, ensure you have installed [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) and are familiar to connecting your system to the Go2, through Ethernet, referring to [Unitree's documentation](https://support.unitree.com/home/en/developer/Quick_start.).

Clone this workspace repo + submodules
```bash
git clone --recurse-submodules https://github.com/eppl-erau-db/go2_rl_ws
```

Build the sdk for the shutoff motion client.
```bash
cd sdk
mkdir build
cd build
cmake ..
make 
```

Set env variabnle and create venv environment
```bash
cd go2_rl_ws
export WORKSPACE_DIR="$PWD"
python3 -m venv .venv
```

Activate venv environment
```bash
source .venv/bin/activate
```

Resolve dependencies 
```bash
pip install onnxruntime
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-rosidl-generator-dds-idl
```

Ensuring you have **NOT** sourced ROS2, compile cyclonedds
```bash
cd $WORKSPACE_DIR/src/unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b humble
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd ..
colcon build --packages-select cyclonedds
```

Source ROS2 and build unitree ROS2 
```bash
cd ..
source /opt/ros/humble/setup.bash
colcon build
```

Connecting the ethernet cord to the quadruped, use `ifconfig` get the name of the connection. Edit `go2_rl_ws/src/unitree_ros2/setup.sh` file to include the correct name. Using enp114s0 as an example:
```bash
#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/humble/setup.bash
source $WORKSPACE_DIR/src/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp114s0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
```

Also using the name of the connection, you will have to set it as the parameter in the launch files. For basic deployment example you will only have to update `go2_rl_ws/src/go2_launch/launch/go2_walk.launch.py`. Again, using enp114s0 as an example:
```bash
def generate_launch_description():
    # Shared parameters
    shared_params = {'network_interface': "enp114s0"}  # TODO: CHANGE TO YOUR INTERFACE NAME
```

Then, make the `setup.bash` an executable and run it:
```bash
cd $WORKSPACE_DIR &&
chmod +x setup.bash &&
source ./setup.bash
```

Finally, restart your PC, as recommended by Unitree.

---

## Locomotion Deployment

Before deploying, ensure the quadraped is **LYING DOWN** with **SPORT MODE OFF**. You can find how to disable sport mode through servive status [here](https://support.unitree.com/home/en/developer/App_binding). Tethering the top of the quadruped is also advised. Work is underway for better and safer testing. 

### Wireless Remote Mapping

| INPUT                  | ACTION                                   | NOTES |
|:-----------------------|:-----------------------------------------|:------|
| **D-Pad ↑ (up)**       | **Stand**                                | Transitions from Sit/Idle → Stand pose |
| **D-Pad ↓**            | **Sit**                                  | Safe, slow descent to Sit pose |
| **START**              | **Start Walking**                        | Deploy RL policy; enables velocity command tracking |
| **SELECT**             | **Stop Walking → Stand**                 | Cancels RL policy and recenters |
| **Left Joystick**      | **Linear Velocity (x, y)**               | x: forward (+) / back (–), y: left (+) / right (–) |
| **Right Joystick**     | **Angular Velocity (yaw)**               | CCW (left), CW (right) |
| **A**                  | **Soft Abort (Damping)**                 | Smoothly damps to lying position |
| **B (HOLD)**           | **EMERGENCY STOP**                       | Kills all motion immediately. **Use only in emergencies!** |

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
--- 

## Comments and Disclaimer

This repository was developed entirely within Embry-Riddle Aeronautical University's Engineering Physics Propulsion Laboratory! Check us out [here](https://eppl.us). Thank you also to the RoboVerse community!

This is an experimental code, we are not responsible for any damages! Use at your own risk.

