# Unitree Go2 RL Deployment

## Introduction and Scope

A ROS2 Humble workspace for deploying RL-trained blind locomotion policies on the Unitree Go2 quadruped robot. Policies are trained using Isaac Lab (NVIDIA simulation) and exported to ONNX format for real-world deployment.

**Warning:** Deploying policies on a real robot is risky. Deploy at your own risk and always use safety tethers during testing.

Locomotion models can be found in `blind_locomotion/share/models/` in ONNX format.

---

## Prerequisites

Before setup, ensure you have:
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) installed
- Ethernet connection capability to the Go2
- Familiarity with [Unitree's documentation](https://support.unitree.com/home/en/developer/Quick_start)

---

## Setup

### 1. Clone Repository

```bash
git clone --recurse-submodules https://github.com/eppl-erau-db/go2_rl_ws
git switch {BRANCH_NAME}
git pull --recurse-submodules
```

### 2. Build Unitree SDK

Build the SDK for the sport mode shutoff utility:

```bash
cd sdk/unitree_sdk2/
mkdir build && cd build
cmake ..
make
```

Verify the build succeeded:
```bash
cd ../../../
export WORKSPACE_DIR="$PWD"
python3 -m venv .venv
```

### 3. Python Environment Setup

```bash
cd $WORKSPACE_DIR
python3 -m venv .venv
source .venv/bin/activate
```

### 4. Install Dependencies

```bash
pip install onnxruntime
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-rosidl-generator-dds-idl
```

### 5. Build CycloneDDS

**Important:** Ensure you have **NOT** sourced ROS2 before this step.

```bash
cd $WORKSPACE_DIR/src/unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b humble
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd $WORKSPACE_DIR
colcon build --packages-select cyclonedds
```

### 6. Build ROS2 Packages

Source ROS2 and build the workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build
```

### 7. Network Configuration

Connect the Ethernet cable to the Go2 and find your interface name:

```bash
ifconfig
# Look for the interface connected to 192.168.123.x network (e.g., enp1s0, eth0)
```

Edit `src/unitree_ros2/setup.sh` with your interface name:

```bash
#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/humble/setup.bash
source $WORKSPACE_DIR/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp1s0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
```

Make the setup script executable:

```bash
chmod +x $WORKSPACE_DIR/src/unitree_ros2/setup.sh
```

**Recommended:** Restart your PC after completing setup (per Unitree's recommendation).

---

## Quick Start: Stand/Sit Test

Before deploying RL policies, verify basic functionality with this stand/sit test.

### Pre-Flight Checklist

- [ ] Robot is **laying down** on a flat surface
- [ ] Ethernet cable connected to robot
- [ ] Safety tether attached (recommended)
- [ ] Battery charged above 50%

### Step 1: Disable Sport Mode

In **Terminal 1**, run the sport mode shutoff utility:

```bash
cd ~/workspaces/go2_rl_ws
./sdk/unitree_sdk2/build/bin/go2_shutoff_motion enp1s0
```

**Expected output:**
```
[UDP] Local IP: 192.168.123.xxx
[INFO] Connecting to robot...
Press ENTER to deactivate sport mode...
[DONE] Motion deactivated.
```

Press Enter when prompted and wait for the `[DONE]` message.

### Step 2: Launch Test

In **Terminal 2**, launch the stand/sit test:

```bash
source ~/workspaces/go2_rl_ws/src/unitree_ros2/setup.sh
ros2 launch go2_launch go2_real_test.launch.py
```

Optional: Add `show_decoder:=true` to see commanded joint positions:
```bash
ros2 launch go2_launch go2_real_test.launch.py show_decoder:=true
```

### Step 3: Keyboard Control

A new terminal window will automatically open with keyboard controls:

| Key | Action | Description |
|-----|--------|-------------|
| `u` | **STAND** | Rise to standing position |
| `d` | **LAY DOWN** | Lower to laying position |
| `a` | **DAMPING** | Soft stop - hold position with low gains |
| `b` | **KILL** | Emergency stop - motors disabled! |
| `q` | **QUIT** | Exit keyboard controller |

### Expected Behavior

1. **On launch:** Robot stays in IDLE mode, no movement
2. **Press 'u':** Robot rises smoothly to standing position
3. **Press 'd':** Robot lowers to laying position (rear hips splay outward)
4. **Press 'a':** Robot holds current position, can be manually repositioned
5. **Press 'b':** **EMERGENCY STOP** - All motors disabled, robot collapses!

### Safety Notes

- The **'b' (KILL)** button will **disable all motors** - the robot will collapse immediately!
- Use KILL only in emergencies
- For normal shutdown: Press 'a' (DAMPING) first, then Ctrl+C in the launch terminal

---

## Utility Tools

### lowstate_monitor.py

View actual joint angles from the robot in real-time. Useful for tuning joint positions.

```bash
source ~/workspaces/go2_rl_ws/src/unitree_ros2/setup.sh
ros2 run blind_locomotion lowstate_monitor.py
```

**Example output:**
```
[LowState #250] Joint positions (radians):
  FR: hip= -0.046  thigh=  1.262  calf= -2.784
  FL: hip=  0.048  thigh=  1.257  calf= -2.794
  RR: hip= -0.341  thigh=  1.278  calf= -2.810
  RL: hip=  0.317  thigh=  1.265  calf= -2.788

  // Copy-paste for constants.hpp SitPos:
   -0.046,  1.262, -2.784,   // FR
    0.048,  1.257, -2.794,   // FL
   -0.341,  1.278, -2.810,   // RR
    0.317,  1.265, -2.788,   // RL
```

### lowcmd_decoder.py

View commanded positions being sent to the robot:

```bash
ros2 run blind_locomotion lowcmd_decoder.py
```

### keyboard_buttons.py

Standalone keyboard control (useful for debugging):

```bash
ros2 run blind_locomotion keyboard_buttons.py
```

---

## Tuning Joint Positions

Different Go2 units may have slightly different optimal positions. Use this process to tune the standing and laying positions for your robot.

### Process

1. **Run lowstate_monitor** while the robot is in the desired position:
   ```bash
   ros2 run blind_locomotion lowstate_monitor.py
   ```

2. **Position the robot manually** (with sport mode disabled and robot in damping mode)

3. **Copy the printed values** from lowstate_monitor output

4. **Update constants.hpp:**
   ```bash
   # Edit: src/rl_deploy/include/rl_deploy/constants.hpp
   ```

5. **Rebuild:**
   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --packages-select rl_deploy
   ```

### Current Tuned Values

These values were measured from a real Go2 robot:

**Standing Position** (`StandPos`):
| Joint | Value (rad) |
|-------|-------------|
| hip | 0.0 |
| thigh | 0.8 |
| calf | -1.5 |

**Laying Position** (`SitPos`):
| Leg | Hip | Thigh | Calf |
|-----|-----|-------|------|
| FR | -0.046 | 1.262 | -2.784 |
| FL | 0.048 | 1.257 | -2.794 |
| RR | -0.341 | 1.278 | -2.810 |
| RL | 0.317 | 1.265 | -2.788 |

**Note:** Rear hips are splayed outward (RR negative, RL positive) to allow the body to lay flat.

### Control Gains

| Mode | Kp | Kd | Description |
|------|----|----|-------------|
| Standing | 50.0 | 5.0 | High stiffness for stable stand |
| Sitting | 30.0 | 10.0 | Medium stiffness, high damping for smooth descent |
| Damping | 5.0 | 6.0 | Low stiffness, allows manual manipulation |
| Walking (RL) | 20.0 | 0.5 | RL policy control |

---

## Locomotion Deployment

### Wireless Remote Mapping

| Input | Action | Notes |
|:------|:-------|:------|
| **D-Pad Up** | **Stand** | Transitions from Sit/Idle to Stand pose |
| **D-Pad Down** | **Sit** | Safe descent to laying position |
| **START** | **Start Walking** | Deploy RL policy; enables velocity tracking |
| **SELECT** | **Stop Walking** | Cancels RL policy and returns to stand |
| **Left Joystick** | **Linear Velocity (x, y)** | x: forward/back, y: left/right |
| **Right Joystick** | **Angular Velocity (yaw)** | CCW (left), CW (right) |
| **A** | **Soft Abort (Damping)** | Smoothly damps to safe position |
| **B (HOLD)** | **EMERGENCY STOP** | Kills all motion. **Use only in emergencies!** |

### Terminal Commands

**Terminal 1** - Disable sport mode:
```bash
cd ~/workspaces/go2_rl_ws
./sdk/unitree_sdk2/build/bin/go2_shutoff_motion enp1s0
```

**Terminal 2** - Launch locomotion:
```bash
source ~/workspaces/go2_rl_ws/src/unitree_ros2/setup.sh
ros2 launch go2_launch go2_walk.launch.py
```

---

## Troubleshooting

### LowState Timeout

**Symptom:** `[WARN] LowState timeout! Entering damping mode.`

**Causes & Solutions:**
- **Ethernet disconnected:** Check cable connection
- **Wrong interface name:** Verify interface in `setup.sh` matches your system (`ifconfig`)
- **Robot powered off:** Check robot power and battery level
- **CycloneDDS not configured:** Ensure `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is set

### Robot Shutdown During Stand

**Symptom:** Robot powers off or enters protection mode while standing

**Causes & Solutions:**
- **Low battery:** Charge battery above 50%
- **Motor overcurrent:** Reduce `kp_stand` in `constants.hpp` (try 40.0 instead of 50.0)
- **Position overshoot:** Increase `kd_stand` for more damping

### Robot Not Responding to Commands

**Symptom:** Keyboard inputs have no effect

**Causes & Solutions:**
- **Sport mode still active:** Run `go2_shutoff_motion` again
- **Wrong topic names:** Verify `/lowstate` and `/lowcmd` topics are active:
  ```bash
  ros2 topic list | grep -E "lowstate|lowcmd"
  ```
- **Keyboard terminal not focused:** Click on the keyboard control terminal window

### Stand Position Incorrect

**Symptom:** Robot doesn't reach expected standing height

**Solution:** Use `lowstate_monitor.py` to measure actual positions and update `constants.hpp`:
1. Manually position robot to desired stand height
2. Run lowstate_monitor and record values
3. Update `StandPos` in `constants.hpp`
4. Rebuild with `colcon build --packages-select rl_deploy`

---

## Package Structure

```
go2_rl_ws/
├── src/
│   ├── blind_locomotion/     # Python RL inference and utilities
│   │   ├── rl_actions.py            # ONNX policy inference
│   │   ├── lowstate_monitor.py      # Joint angle viewer
│   │   ├── lowcmd_decoder.py        # Command decoder
│   │   ├── keyboard_buttons.py      # Keyboard control
│   │   └── share/models/            # ONNX policy files
│   ├── rl_deploy/            # C++ low-level control
│   │   ├── go2_controller_node      # Main controller
│   │   └── include/constants.hpp    # Joint positions & gains
│   ├── go2_launch/           # Launch files
│   │   ├── go2_real_test.launch.py  # Stand/sit test
│   │   └── go2_walk.launch.py       # Full locomotion
│   └── unitree_ros2/         # Unitree ROS2 interface
└── sdk/
    └── unitree_sdk2/         # Unitree C++ SDK
        └── build/bin/go2_shutoff_motion
```

---

## Comments and Disclaimer

This repository was developed at Embry-Riddle Aeronautical University's Engineering Physics Propulsion Laboratory. Check us out at [eppl.us](https://eppl.us). Thank you to the RoboVerse community!

**This is experimental software. We are not responsible for any damages. Use at your own risk.**
