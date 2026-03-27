"""
Launch file for RL Reaching/Standing Policy deployment.

This launch file starts:
1. go2_controller_node - Low-level motor control (200 Hz)
2. rl_reach_actions - ONNX policy inference for reaching/standing (50 Hz)
3. controller_commands - Wireless controller to buttons (for mode switching)
4. base_height_estimator - Base height from lowstate FK/contact

Usage:
    ros2 launch go2_launch go2_reach.launch.py policy_name:=SimplePolicy
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    policy_name_arg = DeclareLaunchArgument(
        'policy_name',
        default_value='right_reach_policy_53_nono',
        description='Name of the ONNX policy file (without .onnx extension)'
    )
    
    policy_frequency_arg = DeclareLaunchArgument(
        'policy_frequency',
        default_value='50',
        description='RL policy inference frequency in Hz'
    )
    
    scale_factor_arg = DeclareLaunchArgument(
        'scale_factor',
        default_value='0.25',
        description='Action scaling factor from Isaac Lab'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic used for base linear velocity'
    )

    odom_timeout_sec_arg = DeclareLaunchArgument(
        'odom_timeout_sec',
        default_value='0.5',
        description='Pause policy when odom age exceeds this timeout'
    )

    base_height_topic_arg = DeclareLaunchArgument(
        'base_height_topic',
        default_value='/base_height',
        description='Base-height topic consumed by reach policy'
    )

    lowstate_timeout_sec_arg = DeclareLaunchArgument(
        'lowstate_timeout_sec',
        default_value='0.5',
        description='Pause reach policy when /lowstate age exceeds this timeout'
    )

    pose_command_topic_arg = DeclareLaunchArgument(
        'pose_command_topic',
        default_value='pose_command',
        description='Pose command topic consumed by reach policy'
    )

    pose_timeout_sec_arg = DeclareLaunchArgument(
        'pose_timeout_sec',
        default_value='0.5',
        description='Pose command staleness timeout in seconds'
    )

    pose_x_min_arg = DeclareLaunchArgument(
        'pose_x_min',
        default_value='0.20',
        description='Minimum FR_foot target position x command (forward)'
    )

    pose_x_max_arg = DeclareLaunchArgument(
        'pose_x_max',
        default_value='0.50',
        description='Maximum FR_foot target position x command (forward)'
    )

    pose_y_min_arg = DeclareLaunchArgument(
        'pose_y_min',
        default_value='-0.30',
        description='Minimum FR_foot target position y command (left/right)'
    )

    pose_y_max_arg = DeclareLaunchArgument(
        'pose_y_max',
        default_value='-0.15',
        description='Maximum FR_foot target position y command (left/right)'
    )

    pose_z_min_arg = DeclareLaunchArgument(
        'pose_z_min',
        default_value='0.0',
        description='Minimum FR_foot target position z command (up/down)'
    )

    pose_z_max_arg = DeclareLaunchArgument(
        'pose_z_max',
        default_value='0.30',
        description='Maximum FR_foot target position z command (up/down)'
    )
    
    enable_joint_limit_monitor_arg = DeclareLaunchArgument(
        'enable_joint_limit_monitor',
        default_value='false',
        description='Emergency sit on joint limit violation (set false to disable)'
    )

    return LaunchDescription([
        # Launch arguments
        policy_name_arg,
        policy_frequency_arg,
        scale_factor_arg,
        odom_topic_arg,
        odom_timeout_sec_arg,
        base_height_topic_arg,
        lowstate_timeout_sec_arg,
        enable_joint_limit_monitor_arg,
        pose_command_topic_arg,
        pose_timeout_sec_arg,
        pose_x_min_arg,
        pose_x_max_arg,
        pose_y_min_arg,
        pose_y_max_arg,
        pose_z_min_arg,
        pose_z_max_arg,
        LogInfo(
            msg=(
                '[go2_reach] Manual odom workflow: stand robot first, then run in a separate terminal:\n'
                'source ~/ros2_ws/install/setup.bash && '
                'ros2 launch leg_odometry_ros leg_odom.launch.py'
            )
        ),
        LogInfo(msg='[go2_reach] After /odom is live, press START to enter WALKING.'),
        LogInfo(msg='[go2_reach] Runtime checks: ros2 topic hz /odom ; ros2 topic hz /actions'),
        
        # Wireless controller handler - cmd_vel/buttons + pose command for reach policy
        Node(
            package='blind_locomotion',
            executable='controller_commands.py',
            name='controller_commands',
            parameters=[{
                'publish_pose_command': True,
                'pose_command_topic': LaunchConfiguration('pose_command_topic'),
                'pose_x_min': LaunchConfiguration('pose_x_min'),
                'pose_x_max': LaunchConfiguration('pose_x_max'),
                'pose_y_min': LaunchConfiguration('pose_y_min'),
                'pose_y_max': LaunchConfiguration('pose_y_max'),
                'pose_z_min': LaunchConfiguration('pose_z_min'),
                'pose_z_max': LaunchConfiguration('pose_z_max'),
            }],
            output='screen'
        ),

        # Base-height estimator for reach observation[6]
        Node(
            package='base_height_estimator',
            executable='base_height_node',
            name='base_height_estimator',
            parameters=[{
                'force_threshold': 20.0,
                'publish_rate_hz': 50.0,
                'filter_alpha': 0.2,
            }],
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
                'odom_topic': LaunchConfiguration('odom_topic'),
                'odom_timeout_sec': LaunchConfiguration('odom_timeout_sec'),
                'base_height_topic': LaunchConfiguration('base_height_topic'),
                'lowstate_timeout_sec': LaunchConfiguration('lowstate_timeout_sec'),
                'pose_command_topic': LaunchConfiguration('pose_command_topic'),
                'pose_timeout_sec': LaunchConfiguration('pose_timeout_sec'),
                'pose_x_min': LaunchConfiguration('pose_x_min'),
                'pose_x_max': LaunchConfiguration('pose_x_max'),
                'pose_y_min': LaunchConfiguration('pose_y_min'),
                'pose_y_max': LaunchConfiguration('pose_y_max'),
                'pose_z_min': LaunchConfiguration('pose_z_min'),
                'pose_z_max': LaunchConfiguration('pose_z_max'),
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
