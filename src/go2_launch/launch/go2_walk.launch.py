from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Policy + observation args.
    policy_name_arg = DeclareLaunchArgument(
        'policy_name',
        default_value='go2_flat_v0',
        description='Name of the ONNX policy file (without .onnx extension)',
    )
    policy_frequency_arg = DeclareLaunchArgument(
        'policy_frequency',
        default_value='50',
        description='RL policy inference frequency in Hz',
    )
    scale_factor_arg = DeclareLaunchArgument(
        'scale_factor',
        default_value='0.25',
        description='Action scaling factor from Isaac Lab',
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic used for base linear velocity',
    )
    odom_timeout_sec_arg = DeclareLaunchArgument(
        'odom_timeout_sec',
        default_value='0.5',
        description='Pause policy when odom age exceeds this timeout',
    )
    cmd_vel_deadband_arg = DeclareLaunchArgument(
        'cmd_vel_deadband',
        default_value='0.1',
        description='Deadband for cmd_vel observation component',
    )

    enable_joint_limit_monitor_arg = DeclareLaunchArgument(
        'enable_joint_limit_monitor',
        default_value='true',
        description='Emergency sit on joint limit violation (set false to disable)',
    )

    enable_base_height_estimator_arg = DeclareLaunchArgument(
        'enable_base_height_estimator',
        default_value='false',
        description='Launch base height estimator node (not needed for flat policy)',
    )

    return LaunchDescription(
        [
            policy_name_arg,
            policy_frequency_arg,
            scale_factor_arg,
            odom_topic_arg,
            odom_timeout_sec_arg,
            cmd_vel_deadband_arg,
            enable_joint_limit_monitor_arg,
            enable_base_height_estimator_arg,
            LogInfo(
                msg=(
                    '[go2_walk] Manual odom workflow: stand robot first, then run in a separate terminal:\n'
                    'source ~/ros2_ws/install/setup.bash && '
                    'ros2 launch leg_odometry_ros leg_odom.launch.py'
                )
            ),
            LogInfo(msg='[go2_walk] After /odom is live, press START to enter WALKING.'),
            LogInfo(msg='[go2_walk] Runtime checks: ros2 topic hz /cmd_vel ; ros2 topic hz /actions'),
            Node(
                package='blind_locomotion',
                executable='controller_commands.py',
                name='controller_commands',
                output='screen',
            ),
            Node(
                package='blind_locomotion',
                executable='rl_actions.py',
                name='rl_actions',
                parameters=[
                    {
                        'policy_name': LaunchConfiguration('policy_name'),
                        'policy_frequency': LaunchConfiguration('policy_frequency'),
                        'scale_factor': LaunchConfiguration('scale_factor'),
                        'odom_topic': LaunchConfiguration('odom_topic'),
                        'odom_timeout_sec': LaunchConfiguration('odom_timeout_sec'),
                        'cmd_vel_deadband': LaunchConfiguration('cmd_vel_deadband'),
                    }
                ],
                output='screen',
            ),
            Node(
                condition=IfCondition(LaunchConfiguration('enable_base_height_estimator')),
                package='base_height_estimator',
                executable='base_height_node',
                name='base_height_estimator',
                parameters=[
                    {
                        'force_threshold': 20.0,
                        'publish_rate_hz': 50.0,
                        'filter_alpha': 0.2,
                    }
                ],
                output='screen',
            ),
            Node(
                package='rl_deploy',
                executable='go2_controller_node',
                name='go2_controller_node',
                parameters=[
                    {
                        'enable_joint_limit_monitor': LaunchConfiguration('enable_joint_limit_monitor'),
                    }
                ],
                output='screen',
            ),
        ]
    )
