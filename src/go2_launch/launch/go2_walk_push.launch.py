from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    walking_policy_name_arg = DeclareLaunchArgument(
        'walking_policy_name',
        default_value='locomotion_policy',
        description='Locomotion ONNX policy file (without .onnx extension)',
    )
    walking_policy_frequency_arg = DeclareLaunchArgument(
        'walking_policy_frequency',
        default_value='50',
        description='Locomotion policy inference frequency in Hz',
    )
    walking_scale_factor_arg = DeclareLaunchArgument(
        'walking_scale_factor',
        default_value='0.25',
        description='Locomotion policy action scaling factor',
    )
    pedipulation_policy_name_arg = DeclareLaunchArgument(
        'pedipulation_policy_name',
        default_value='reachpolicy18v2',
        description='Pedipulation ONNX policy file (without .onnx extension)',
    )
    pedipulation_policy_frequency_arg = DeclareLaunchArgument(
        'pedipulation_policy_frequency',
        default_value='50',
        description='Pedipulation policy inference frequency in Hz',
    )
    pedipulation_scale_factor_arg = DeclareLaunchArgument(
        'pedipulation_scale_factor',
        default_value='0.25',
        description='Pedipulation policy action scaling factor',
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic used for both policies',
    )
    odom_timeout_sec_arg = DeclareLaunchArgument(
        'odom_timeout_sec',
        default_value='0.5',
        description='Pause either policy when odom age exceeds this timeout',
    )
    cmd_vel_deadband_arg = DeclareLaunchArgument(
        'cmd_vel_deadband',
        default_value='0.1',
        description='Deadband for cmd_vel observation component',
    )
    base_height_topic_arg = DeclareLaunchArgument(
        'base_height_topic',
        default_value='/base_height',
        description='Base height topic used by the pedipulation policy',
    )
    base_height_timeout_sec_arg = DeclareLaunchArgument(
        'base_height_timeout_sec',
        default_value='0.5',
        description='Pause locomotion policy when base-height age exceeds this timeout',
    )
    lowstate_timeout_sec_arg = DeclareLaunchArgument(
        'lowstate_timeout_sec',
        default_value='0.5',
        description='Pause pedipulation policy when /lowstate age exceeds this timeout',
    )
    pose_command_topic_arg = DeclareLaunchArgument(
        'pose_command_topic',
        default_value='pose_command',
        description='Pose command topic consumed by the pedipulation policy',
    )
    pose_timeout_sec_arg = DeclareLaunchArgument(
        'pose_timeout_sec',
        default_value='0.5',
        description='Pose command staleness timeout in seconds',
    )
    pose_x_min_arg = DeclareLaunchArgument(
        'pose_x_min',
        default_value='0.20',
        description='Minimum FR_foot target position x command (forward)',
    )
    pose_x_max_arg = DeclareLaunchArgument(
        'pose_x_max',
        default_value='0.50',
        description='Maximum FR_foot target position x command (forward)',
    )
    pose_y_min_arg = DeclareLaunchArgument(
        'pose_y_min',
        default_value='-0.30',
        description='Minimum FR_foot target position y command (left/right)',
    )
    pose_y_max_arg = DeclareLaunchArgument(
        'pose_y_max',
        default_value='-0.15',
        description='Maximum FR_foot target position y command (left/right)',
    )
    pose_z_min_arg = DeclareLaunchArgument(
        'pose_z_min',
        default_value='0.0',
        description='Minimum FR_foot target position z command (up/down)',
    )
    pose_z_max_arg = DeclareLaunchArgument(
        'pose_z_max',
        default_value='0.30',
        description='Maximum FR_foot target position z command (up/down)',
    )
    push_mid_z_arg = DeclareLaunchArgument(
        'push_mid_z',
        default_value='0.15',
        description='Mid-trajectory z height for the blind push',
    )
    push_duration_sec_arg = DeclareLaunchArgument(
        'push_duration_sec',
        default_value='4.0',
        description='Blind push trajectory duration in seconds',
    )
    enable_joint_limit_monitor_arg = DeclareLaunchArgument(
        'enable_joint_limit_monitor',
        default_value='false',
        description='Emergency sit on joint limit violation (set false to disable)',
    )

    return LaunchDescription([
        walking_policy_name_arg,
        walking_policy_frequency_arg,
        walking_scale_factor_arg,
        pedipulation_policy_name_arg,
        pedipulation_policy_frequency_arg,
        pedipulation_scale_factor_arg,
        odom_topic_arg,
        odom_timeout_sec_arg,
        cmd_vel_deadband_arg,
        base_height_topic_arg,
        base_height_timeout_sec_arg,
        lowstate_timeout_sec_arg,
        pose_command_topic_arg,
        pose_timeout_sec_arg,
        pose_x_min_arg,
        pose_x_max_arg,
        pose_y_min_arg,
        pose_y_max_arg,
        pose_z_min_arg,
        pose_z_max_arg,
        push_mid_z_arg,
        push_duration_sec_arg,
        enable_joint_limit_monitor_arg,
        LogInfo(
            msg=(
                '[go2_walk_push] Manual odom workflow: stand robot first, then run in a separate terminal:\n'
                'source ~/ros2_ws/install/setup.bash && '
                'ros2 launch leg_odometry_ros leg_odom.launch.py'
            )
        ),
        LogInfo(msg='[go2_walk_push] After /odom is live, press START to enter WALKING.'),
        LogInfo(msg='[go2_walk_push] Press SELECT to enter PEDIPULATION.'),
        LogInfo(msg='[go2_walk_push] In PEDIPULATION: F1 runs the blind push, START cancels/returns to WALKING.'),
        LogInfo(msg='[go2_walk_push] Runtime checks: ros2 topic hz /locomotion_actions ; ros2 topic hz /pedipulation_actions ; ros2 topic hz /lowcmd'),
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
            parameters=[{
                'policy_name': LaunchConfiguration('walking_policy_name'),
                'policy_frequency': LaunchConfiguration('walking_policy_frequency'),
                'scale_factor': LaunchConfiguration('walking_scale_factor'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'odom_timeout_sec': LaunchConfiguration('odom_timeout_sec'),
                'cmd_vel_deadband': LaunchConfiguration('cmd_vel_deadband'),
                'base_height_topic': LaunchConfiguration('base_height_topic'),
                'base_height_timeout_sec': LaunchConfiguration('base_height_timeout_sec'),
            }],
            remappings=[('actions', 'locomotion_actions')],
            output='screen',
        ),
        Node(
            package='base_height_estimator',
            executable='base_height_node',
            name='base_height_estimator',
            parameters=[{
                'force_threshold': 20.0,
                'publish_rate_hz': 50.0,
                'filter_alpha': 0.2,
            }],
            output='screen',
        ),
        Node(
            package='blind_locomotion',
            executable='rl_reach_actions.py',
            name='rl_reach_actions',
            parameters=[{
                'policy_name': LaunchConfiguration('pedipulation_policy_name'),
                'policy_frequency': LaunchConfiguration('pedipulation_policy_frequency'),
                'scale_factor': LaunchConfiguration('pedipulation_scale_factor'),
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
            remappings=[('actions', 'pedipulation_actions')],
            output='screen',
        ),
        Node(
            package='blind_locomotion',
            executable='pedipulation_executor.py',
            name='pedipulation_executor',
            parameters=[{
                'pose_command_topic': LaunchConfiguration('pose_command_topic'),
                'publish_rate_hz': LaunchConfiguration('pedipulation_policy_frequency'),
                'trajectory_duration_sec': LaunchConfiguration('push_duration_sec'),
                'pose_x_min': LaunchConfiguration('pose_x_min'),
                'pose_x_max': LaunchConfiguration('pose_x_max'),
                'pose_y_min': LaunchConfiguration('pose_y_min'),
                'pose_y_max': LaunchConfiguration('pose_y_max'),
                'pose_z_min': LaunchConfiguration('pose_z_min'),
                'pose_z_max': LaunchConfiguration('pose_z_max'),
                'push_mid_z': LaunchConfiguration('push_mid_z'),
            }],
            output='screen',
        ),
        Node(
            package='rl_deploy',
            executable='go2_controller_node',
            name='go2_controller_node',
            parameters=[{
                'enable_joint_limit_monitor': LaunchConfiguration('enable_joint_limit_monitor'),
                'walking_actions_topic': 'locomotion_actions',
                'pedipulation_actions_topic': 'pedipulation_actions',
                'pedipulation_execute_service': 'pedipulation/execute_push',
                'pedipulation_cancel_service': 'pedipulation/cancel_push',
            }],
            output='screen',
        ),
    ])
