from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    policy_name_arg = DeclareLaunchArgument(
        'policy_name',
        default_value='old_policy',
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
    
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='1.0',
        description='Maximum linear velocity (m/s)'
    )
    
    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='1.0',
        description='Maximum angular velocity (rad/s)'
    )

    return LaunchDescription([
        # Launch arguments
        policy_name_arg,
        policy_frequency_arg,
        scale_factor_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        
        # Wireless controller handler - converts joystick to cmd_vel and buttons
        Node(
            package='blind_locomotion',
            executable='controller_commands.py',
            name='controller_commands',
            parameters=[{
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            }],
            output='screen'
        ),
        
        # RL policy inference node - runs ONNX model and publishes actions
        Node(
            package='blind_locomotion',
            executable='rl_actions.py',
            name='rl_actions',
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
            output='screen'
        ),
    ])
