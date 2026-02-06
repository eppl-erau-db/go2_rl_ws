"""
Launch file for RL Reaching/Standing Policy deployment.

This launch file starts:
1. go2_controller_node - Low-level motor control (200 Hz)
2. rl_reach_actions - ONNX policy inference for reaching/standing (25 Hz)
3. controller_commands - Wireless controller to buttons (for mode switching)

Usage:
    ros2 launch go2_launch go2_reach.launch.py policy_name:=SimplePolicy
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    policy_name_arg = DeclareLaunchArgument(
        'policy_name',
        default_value='SimplePolicy',
        description='Name of the ONNX policy file (without .onnx extension)'
    )
    
    policy_frequency_arg = DeclareLaunchArgument(
        'policy_frequency',
        default_value='25',
        description='RL policy inference frequency in Hz'
    )
    
    scale_factor_arg = DeclareLaunchArgument(
        'scale_factor',
        default_value='0.25',
        description='Action scaling factor from Isaac Lab'
    )
    
    enable_joint_limit_monitor_arg = DeclareLaunchArgument(
        'enable_joint_limit_monitor',
        default_value='true',
        description='Emergency sit on joint limit violation (set false to disable)'
    )

    return LaunchDescription([
        # Launch arguments
        policy_name_arg,
        policy_frequency_arg,
        scale_factor_arg,
        enable_joint_limit_monitor_arg,
        
        # Wireless controller handler - converts joystick to buttons for mode switching
        Node(
            package='blind_locomotion',
            executable='controller_commands.py',
            name='controller_commands',
            output='screen'
        ),
        
        # RL reaching policy inference node - runs ONNX model and publishes actions
        Node(
            package='blind_locomotion',
            executable='rl_reach_actions.py',
            name='rl_reach_actions',
            parameters=[{
                'policy_name': LaunchConfiguration('policy_name'),
                'policy_frequency': LaunchConfiguration('policy_frequency'),
                'scale_factor': LaunchConfiguration('scale_factor'),
            }],
            output='screen'
        ),
        
        # Motor command controller - takes actions and sends to robot
        Node(
            package='rl_deploy',
            executable='go2_controller_node',
            name='go2_controller_node',
            parameters=[{
                'enable_joint_limit_monitor': LaunchConfiguration('enable_joint_limit_monitor'),
            }],
            output='screen'
        ),
    ])
