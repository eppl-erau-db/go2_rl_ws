"""
Real robot test launch file for stand/sit transitions WITHOUT the RL policy.

This allows testing the go2_controller_node state machine on the REAL ROBOT:
- test_action_publisher: Publishes standing position actions at 25Hz
- keyboard_buttons: Keyboard input for button control (opens gnome-terminal)
- go2_controller_node: The component under test
- lowcmd_decoder: (optional) Human-readable /lowcmd display

NOTE: This does NOT include fake_lowstate_publisher - the real robot
publishes /lowstate directly via DDS.

=============================================================================
                           PRE-FLIGHT CHECKLIST
=============================================================================
1. Robot is LAYING DOWN or tethered/suspended
2. Ethernet connected (check interface name with `ifconfig`)
3. Sport mode disabled (see below)

=============================================================================
                         DEPLOYMENT INSTRUCTIONS
=============================================================================

TERMINAL 1 - Disable Sport Mode (run ONCE before launching):
    cd ~/workspaces/go2_rl_ws
    ./sdk/unitree_sdk2/build/bin/go2_shutoff_motion enp1s0
    # Press Enter when prompted
    # Wait for "[DONE] Motion deactivated."

TERMINAL 2 - Launch this file:
    source ~/workspaces/go2_rl_ws/src/unitree_ros2/setup.sh
    ros2 launch go2_launch go2_real_test.launch.py

TERMINAL 3 - Keyboard control (auto-opens via gnome-terminal):
    u = STAND (stand up)
    d = SIT (sit down)
    s = START (not used in this test)
    x = SELECT (not used in this test)
    a = DAMPING (soft abort - hold position with low gains)
    b = KILL (emergency stop - motors disabled, robot will collapse!)
    q = quit

=============================================================================
                              EXPECTED BEHAVIOR
=============================================================================
1. On launch: Robot stays in IDLE mode, waiting for button input
2. Press 'u': IDLE -> STANDING (robot rises to standing position)
3. Press 'd': STANDING -> SITTING (robot lowers to sitting position)
4. Press 'a': -> DAMPING (hold current position, can be manually moved)
5. Press 'b': -> KILLED (EMERGENCY STOP - all motors disabled!)

=============================================================================
                              SAFETY NOTES
=============================================================================
- The 'b' (KILL) button will DISABLE ALL MOTORS - robot will collapse!
- Use KILL only in emergencies
- For normal shutdown, use 'a' (DAMPING) first, then Ctrl+C

Joint positions used (from main branch, previously tested on real robot):
- Stand: hip=0.0, thigh=1.1rad, calf=-1.8rad
- Sit: hip=-0.1, thigh=1.1rad, calf=-2.0/-2.6rad

Gains used (from main branch):
- Standing: Kp=50, Kd=5
- Sitting: Kp=30, Kd=10
- Damping: Kp=5, Kd=1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='true',
        description='Enable verbose logging in go2_controller_node'
    )
    
    show_decoder_arg = DeclareLaunchArgument(
        'show_decoder',
        default_value='false',
        description='Show lowcmd_decoder output (human-readable commands)'
    )
    
    action_frequency_arg = DeclareLaunchArgument(
        'action_frequency',
        default_value='25.0',
        description='Frequency of action publishing in Hz'
    )

    return LaunchDescription([
        # Launch arguments
        verbose_arg,
        show_decoder_arg,
        action_frequency_arg,
        
        # NOTE: No fake_lowstate_publisher here!
        # The real robot publishes /lowstate directly via DDS.
        
        # Test action publisher - publishes standing position (no RL inference)
        Node(
            package='blind_locomotion',
            executable='test_action_publisher.py',
            name='test_action_publisher',
            parameters=[{
                'publish_frequency': LaunchConfiguration('action_frequency'),
            }],
            output='screen'
        ),
        
        # Keyboard button input - runs in its own terminal for keyboard capture
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c', 
                 'source ~/workspaces/go2_rl_ws/src/unitree_ros2/setup.sh && '
                 'ros2 run blind_locomotion keyboard_buttons.py; exec bash'],
            name='keyboard_buttons',
            output='screen'
        ),
        
        # The component under test - go2_controller_node
        Node(
            package='rl_deploy',
            executable='go2_controller_node',
            name='go2_controller_node',
            parameters=[{
                'verbose': LaunchConfiguration('verbose'),
            }],
            output='screen'
        ),
        
        # Optional: LowCmd decoder for human-readable output
        Node(
            package='blind_locomotion',
            executable='lowcmd_decoder.py',
            name='lowcmd_decoder',
            condition=IfCondition(LaunchConfiguration('show_decoder')),
            output='screen'
        ),
    ])
