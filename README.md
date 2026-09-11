# Unitree Go2 RL Locomotion

ROS 2 Humble workspace for running an Isaac Lab–trained blind locomotion policy (ONNX)
on a real Unitree Go2. One launch file takes the robot from "just powered on" to
"standing, odometry live, waiting for START".

**Deploying a policy on a real robot is risky. Use a tether, keep a hand on the remote,
and deploy at your own risk.**

---

## 1. Setup (once)

```bash
git clone --recurse-submodules https://github.com/eppl-erau-db/go2_rl_ws
cd go2_rl_ws

# Unitree SDK helpers (sport-mode shutoff / restore, sport client)
cd sdk/unitree_sdk2 && mkdir -p build && cd build && cmake .. && make && cd ../../..

# Dependencies
pip install onnxruntime
sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl

# CycloneDDS (do NOT have ROS sourced for this step)
cd src/unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b humble
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd ../../../..
colcon build --packages-select cyclonedds

# Workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Plug the Ethernet cable into the Go2, find the interface on the `192.168.123.x` network
(`ifconfig`), and put its name in [setup.sh](setup.sh) at the workspace root
(`NetworkInterface name="enp1s0"`). Reboot the PC once after this.

Leg odometry lives in a separate workspace, `~/ros2_ws` (`leg_odometry_ros`). It must
be built and installed there; the walk launch sources it on its own.

---

## 2. Walk the robot

### The two rules

1. **Every terminal:** `cd ~/workspaces/go2_rl_ws && source setup.sh`. Nothing else.
2. **Never `source ~/ros2_ws/install/setup.bash` in the terminal that runs the launch**
   (or any SDK helper). It breaks the Unitree SDK. The launch sources it only inside the
   odometry child process, and warns you if it sees it in your shell.

### One command

Power on the robot and let it stand on its own. Then, in a fresh terminal:

```bash
cd ~/workspaces/go2_rl_ws
source setup.sh
ros2 launch go2_launch go2_walk.launch.py
```

What it does, in order:

| Step | Who | Done when |
|---|---|---|
| Lie the robot down | `go2_sport_client` | `body_height` ≤ 0.15 m **or** joints within 0.3 rad of `SitPos` **or** `mode == 5` |
| Turn off Unitree motion control | `go2_shutoff_motion` | `[DONE] Motion deactivated.` |
| Start the low-level stack | `controller_commands`, `rl_actions`, `go2_controller_node` | `Mode transition: IDLE -> STANDING` |
| Stand the robot | `go2_controller_node` (`stand_on_start`) | `/stand_ready` true |
| Wait for a confirmed stand | `go2_stand_ready_gate` | `Stand confirmed` |
| Start leg odometry, headless | `ros2 launch leg_odometry_ros leg_odom.launch.py headless:=true` (in a shell that sources `~/ros2_ws`) | `Calibration Done` |
| Hand over to you | — | `press START to enter WALKING` |

Odometry is deliberately started only after the stand is confirmed: starting it while the
robot is lying down accumulates drift.

### Remote

| Input | Action |
|---|---|
| **START** | Enter WALKING (only from a good stand) |
| **Left stick** | Linear velocity x / y |
| **Right stick** | Yaw rate |
| **UP** | Stand (also exits walking back to a rigid stand) |
| **DOWN** | Sit, then motors idle |
| **A** | Damping — soft stop, robot can be moved by hand |
| **B** | **KILL — motors off, robot collapses.** Emergencies only. |

To finish: **DOWN**, then Ctrl+C in the launch terminal.

### Launch recipes

Default (flat 48-dim policy `go2_flat_v0`, auto lie-down / shutoff / stand / odometry):
```bash
ros2 launch go2_launch go2_walk.launch.py
```

Height-aware 49-dim policy (needs the base-height estimator):
```bash
ros2 launch go2_launch go2_walk.launch.py policy_name:=locomotion_policy enable_base_height_estimator:=true
```

Robot already lying down with motion control already off (e.g. a previous run stopped there):
```bash
ros2 launch go2_launch go2_walk.launch.py skip_motion_prep:=true
```

Stand on **UP** instead of automatically:
```bash
ros2 launch go2_launch go2_walk.launch.py auto_stand:=false
```

Run odometry yourself (or not at all):
```bash
ros2 launch go2_launch go2_walk.launch.py enable_leg_odometry:=false
```

Odometry with RViz, or from another workspace:
```bash
ros2 launch go2_launch go2_walk.launch.py leg_odom_headless:=false leg_odom_workspace:=~/other_ws
```

Different Ethernet interface:
```bash
ros2 launch go2_launch go2_walk.launch.py network_interface:=eth0
```

Full argument list: `ros2 launch go2_launch go2_walk.launch.py -s`.

Policies are ONNX files in [src/blind_locomotion/share/models/](src/blind_locomotion/share/models/);
`policy_name` is the filename without `.onnx`. The node reads the input size from the model:
48 → flat, 49 → expects `/base_height`.

### Give control back to Unitree (no reboot)

After a session the robot's own motion control is off. Stop the launch (Ctrl+C), leave the
robot lying on flat ground, then:

```bash
cd ~/workspaces/go2_rl_ws
source setup.sh
./sdk/unitree_sdk2/build/bin/go2_restore_motion enp1s0
```

It tries the mode aliases `ai`, `normal`, `advanced`, `mcf` until the firmware accepts one
(`7004` = wrong alias, it moves on). **The robot will stand up by itself** once the service is
back. Pass the alias explicitly (`… enp1s0 ai`) once you know which one works.

### Manual, step by step

If you want each step under your own control:

```bash
# Terminal 1 — robot lying down; turn motion control off
cd ~/workspaces/go2_rl_ws && source setup.sh
./sdk/unitree_sdk2/build/bin/go2_shutoff_motion enp1s0

# Terminal 2 — launch, then press UP to stand
cd ~/workspaces/go2_rl_ws && source setup.sh
ros2 launch go2_launch go2_walk.launch.py skip_motion_prep:=true auto_stand:=false enable_leg_odometry:=false

# Terminal 3 — only once the robot is standing
cd ~/workspaces/go2_rl_ws && source setup.sh
source ~/ros2_ws/install/setup.bash
ros2 launch leg_odometry_ros leg_odom.launch.py headless:=true
```

Then **START**.

---

## 3. Checks and troubleshooting

Live checks:
```bash
ros2 topic echo /controller_mode      # IDLE / STANDING / WALKING / ...
ros2 topic echo /stand_ready          # true when standing with good posture
ros2 topic hz /odom /actions /cmd_vel /lowcmd
```

| Symptom | Cause / fix |
|---|---|
| `Timed out waiting for lie-down confirmation` | Message prints the last `mode`, `body_height` and joint error — see which check is off. If `SitPos` was retuned, mirror it in `SIT_POS` in `go2_motion_prep.py`. |
| `go2_shutoff_motion did not confirm …` / `SelectMode failed` | `~/ros2_ws` is sourced in this shell. Open a fresh terminal, `source setup.sh` only. |
| Lie-down/shutoff step fails but the robot is already down | Relaunch with `skip_motion_prep:=true`. |
| `Stand-ready gate exited with return code 1` | No confirmed stand within `stand_ready_timeout_sec` (30 s). Check `/stand_ready`; if the robot is up but the flag stays false, retune `StandPos` in `constants.hpp`. |
| `LowState timeout! Entering damping mode` | Cable, wrong interface in `setup.sh`, robot off, or `RMW_IMPLEMENTATION` not cyclonedds. |
| Robot powers off / protection mode while standing | Battery < 50 %, or lower `kp_stand` in `constants.hpp` (try 40). |
| Remote does nothing | Motion control still on (`go2_shutoff_motion`), or `/lowstate` `/lowcmd` not listed in `ros2 topic list`. |
| Policy rails / robot shakes in WALKING | Check `/odom` is live and fresh (`odom_timeout_sec`); do not clip actions — large values are expected with `kp=25, kd=0.5`. |

---

## 4. Tools and tuning

```bash
ros2 launch go2_launch go2_real_test.launch.py   # stand/sit test, no policy; keys u/d/a/b/q
ros2 run blind_locomotion lowstate_monitor.py    # live joint angles, prints SitPos-style block
ros2 run blind_locomotion lowcmd_decoder.py      # what is being commanded
```

Joint targets and gains live in
[src/rl_deploy/include/rl_deploy/constants.hpp](src/rl_deploy/include/rl_deploy/constants.hpp)
(`StandPos`, `SitPos`, `position_tolerance = 0.3`, `kp/kd` per mode — walking uses
`kp=25, kd=0.5`). Retune by reading `lowstate_monitor.py` in the target pose, editing the
array, then `colcon build --symlink-install --packages-select rl_deploy`. Keep `SIT_POS` in
[go2_motion_prep.py](src/blind_locomotion/blind_locomotion/go2_motion_prep.py) in sync.

---

## 5. Layout

```
src/
├── blind_locomotion/        Python: policy inference, remote → cmd_vel/buttons,
│   │                        lie-down + shutoff helper, stand-ready gate, tools
│   └── share/models/        ONNX policies
├── rl_deploy/               C++: go2_controller_node (mode FSM, PD gains, safety)
├── go2_launch/launch/       go2_walk.launch.py  ← this README
│                            go2_real_test.launch.py, nav / reach / push variants
├── base_height_estimator/   /base_height for 49-dim policies
└── unitree_ros2/            Unitree ROS 2 messages + CycloneDDS
sdk/unitree_sdk2/build/bin/  go2_shutoff_motion, go2_restore_motion, go2_sport_client
setup.sh                     the one script to source
```

Navigation, pedipulation and ai_sport workflows are documented in
[launch_commands.md](launch_commands.md).

---

Developed at Embry-Riddle Aeronautical University's Engineering Physics Propulsion
Laboratory ([eppl.us](https://eppl.us)). Thanks to the RoboVerse community.
**Experimental software — we are not responsible for any damage. Use at your own risk.**
