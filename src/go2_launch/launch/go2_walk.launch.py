import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Policy + observation args.
    policy_name_arg = DeclareLaunchArgument(
        'policy_name',
        default_value='locomotion_policy_v2',
        description='Name of the ONNX policy file (without .onnx extension)',
    )
    policy_frequency_arg = DeclareLaunchArgument(
        'policy_frequency',
        default_value='50',
        description='RL policy inference frequency in Hz',
    )
    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='200',
        description='Action publish frequency in Hz (must be multiple of policy_frequency)',
    )
    scale_factor_arg = DeclareLaunchArgument(
        'scale_factor',
        default_value='0.25',
        description='Action scaling factor from Isaac Lab',
    )
    default_hip_q_arg = DeclareLaunchArgument(
        'default_hip_q',
        default_value='0.0',
        description='Default hip joint position offset',
    )
    default_thigh_q_arg = DeclareLaunchArgument(
        'default_thigh_q',
        default_value='0.8',
        description='Default thigh joint position offset',
    )
    default_calf_q_arg = DeclareLaunchArgument(
        'default_calf_q',
        default_value='-1.5',
        description='Default calf joint position offset',
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
    base_height_timeout_sec_arg = DeclareLaunchArgument(
        'base_height_timeout_sec',
        default_value='0.5',
        description='Pause policy when base_height age exceeds this timeout',
    )
    cmd_vel_deadband_arg = DeclareLaunchArgument(
        'cmd_vel_deadband',
        default_value='0.1',
        description='Deadband for cmd_vel observation component',
    )

    # Controller command range args.
    lin_vel_x_min_arg = DeclareLaunchArgument('lin_vel_x_min', default_value='-1.0')
    lin_vel_x_max_arg = DeclareLaunchArgument('lin_vel_x_max', default_value='1.0')
    lin_vel_y_min_arg = DeclareLaunchArgument('lin_vel_y_min', default_value='-1.0')
    lin_vel_y_max_arg = DeclareLaunchArgument('lin_vel_y_max', default_value='1.0')
    ang_vel_z_min_arg = DeclareLaunchArgument('ang_vel_z_min', default_value='-1.0')
    ang_vel_z_max_arg = DeclareLaunchArgument('ang_vel_z_max', default_value='1.0')
    heading_min_arg = DeclareLaunchArgument('heading_min', default_value=str(-math.pi))
    heading_max_arg = DeclareLaunchArgument('heading_max', default_value=str(math.pi))
    axis_lin_x_arg = DeclareLaunchArgument('axis_lin_x', default_value='left_y')
    axis_lin_y_arg = DeclareLaunchArgument('axis_lin_y', default_value='left_x')
    axis_ang_z_arg = DeclareLaunchArgument('axis_ang_z', default_value='right_x')
    invert_lin_x_arg = DeclareLaunchArgument('invert_lin_x', default_value='false')
    invert_lin_y_arg = DeclareLaunchArgument('invert_lin_y', default_value='true')
    invert_ang_z_arg = DeclareLaunchArgument('invert_ang_z', default_value='true')
    controller_timeout_sec_arg = DeclareLaunchArgument('controller_timeout_sec', default_value='0.5')
    controller_debug_enabled_arg = DeclareLaunchArgument('controller_debug_enabled', default_value='true')
    controller_debug_rate_hz_arg = DeclareLaunchArgument('controller_debug_rate_hz', default_value='5.0')
    controller_debug_only_nonzero_cmd_arg = DeclareLaunchArgument(
        'controller_debug_only_nonzero_cmd', default_value='false'
    )

    policy_debug_enabled_arg = DeclareLaunchArgument('policy_debug_enabled', default_value='true')
    policy_debug_rate_hz_arg = DeclareLaunchArgument('policy_debug_rate_hz', default_value='5.0')
    policy_debug_publish_obs_topic_arg = DeclareLaunchArgument(
        'policy_debug_publish_obs_topic', default_value='false'
    )
    policy_debug_publish_action_topic_arg = DeclareLaunchArgument(
        'policy_debug_publish_action_topic', default_value='false'
    )
    go2_debug_enabled_arg = DeclareLaunchArgument('go2_debug_enabled', default_value='true')
    go2_debug_rate_hz_arg = DeclareLaunchArgument('go2_debug_rate_hz', default_value='5.0')

    enable_joint_limit_monitor_arg = DeclareLaunchArgument(
        'enable_joint_limit_monitor',
        default_value='true',
        description='Emergency sit on joint limit violation (set false to disable)',
    )

    bhe_force_threshold_arg = DeclareLaunchArgument(
        'bhe_force_threshold',
        default_value='20.0',
        description='Min foot force (N) to count as stance for height estimation',
    )
    bhe_publish_rate_hz_arg = DeclareLaunchArgument(
        'bhe_publish_rate_hz',
        default_value='50.0',
        description='Max publish rate for base height estimator',
    )
    bhe_filter_alpha_arg = DeclareLaunchArgument(
        'bhe_filter_alpha',
        default_value='0.2',
        description='EMA smoothing factor for base height (1.0 = no filter)',
    )

    return LaunchDescription(
        [
            policy_name_arg,
            policy_frequency_arg,
            publish_frequency_arg,
            scale_factor_arg,
            default_hip_q_arg,
            default_thigh_q_arg,
            default_calf_q_arg,
            odom_topic_arg,
            odom_timeout_sec_arg,
            base_height_timeout_sec_arg,
            cmd_vel_deadband_arg,
            lin_vel_x_min_arg,
            lin_vel_x_max_arg,
            lin_vel_y_min_arg,
            lin_vel_y_max_arg,
            ang_vel_z_min_arg,
            ang_vel_z_max_arg,
            heading_min_arg,
            heading_max_arg,
            axis_lin_x_arg,
            axis_lin_y_arg,
            axis_ang_z_arg,
            invert_lin_x_arg,
            invert_lin_y_arg,
            invert_ang_z_arg,
            controller_timeout_sec_arg,
            controller_debug_enabled_arg,
            controller_debug_rate_hz_arg,
            controller_debug_only_nonzero_cmd_arg,
            policy_debug_enabled_arg,
            policy_debug_rate_hz_arg,
            policy_debug_publish_obs_topic_arg,
            policy_debug_publish_action_topic_arg,
            go2_debug_enabled_arg,
            go2_debug_rate_hz_arg,
            enable_joint_limit_monitor_arg,
            bhe_force_threshold_arg,
            bhe_publish_rate_hz_arg,
            bhe_filter_alpha_arg,
            LogInfo(
                msg=(
                    '[go2_walk] Manual odom workflow: stand robot first, then run in a separate terminal:\n'
                    'source ~/ros2_ws/install/setup.bash && '
                    'ros2 launch leg_odometry_ros leg_odom.launch.py'
                )
            ),
            LogInfo(msg='[go2_walk] After /odom is live, press START to enter WALKING.'),
            LogInfo(msg='[go2_walk] Runtime checks: ros2 topic hz /cmd_vel ; ros2 topic hz /actions'),
            LogInfo(
                msg=(
                    '[go2_walk] Optional debug topics (if enabled): '
                    'ros2 topic echo /debug/rl_obs ; ros2 topic echo /debug/rl_raw_action'
                )
            ),
            Node(
                package='blind_locomotion',
                executable='controller_commands.py',
                name='controller_commands',
                parameters=[
                    {
                        'lin_vel_x_min': LaunchConfiguration('lin_vel_x_min'),
                        'lin_vel_x_max': LaunchConfiguration('lin_vel_x_max'),
                        'lin_vel_y_min': LaunchConfiguration('lin_vel_y_min'),
                        'lin_vel_y_max': LaunchConfiguration('lin_vel_y_max'),
                        'ang_vel_z_min': LaunchConfiguration('ang_vel_z_min'),
                        'ang_vel_z_max': LaunchConfiguration('ang_vel_z_max'),
                        'heading_min': LaunchConfiguration('heading_min'),
                        'heading_max': LaunchConfiguration('heading_max'),
                        'axis_lin_x': LaunchConfiguration('axis_lin_x'),
                        'axis_lin_y': LaunchConfiguration('axis_lin_y'),
                        'axis_ang_z': LaunchConfiguration('axis_ang_z'),
                        'invert_lin_x': LaunchConfiguration('invert_lin_x'),
                        'invert_lin_y': LaunchConfiguration('invert_lin_y'),
                        'invert_ang_z': LaunchConfiguration('invert_ang_z'),
                        'timeout_sec': LaunchConfiguration('controller_timeout_sec'),
                        'debug_enabled': LaunchConfiguration('controller_debug_enabled'),
                        'debug_rate_hz': LaunchConfiguration('controller_debug_rate_hz'),
                        'debug_only_nonzero_cmd': LaunchConfiguration(
                            'controller_debug_only_nonzero_cmd'
                        ),
                    }
                ],
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
                        'publish_frequency': LaunchConfiguration('publish_frequency'),
                        'scale_factor': LaunchConfiguration('scale_factor'),
                        'default_hip_q': LaunchConfiguration('default_hip_q'),
                        'default_thigh_q': LaunchConfiguration('default_thigh_q'),
                        'default_calf_q': LaunchConfiguration('default_calf_q'),
                        'odom_topic': LaunchConfiguration('odom_topic'),
                        'odom_timeout_sec': LaunchConfiguration('odom_timeout_sec'),
                        'base_height_timeout_sec': LaunchConfiguration('base_height_timeout_sec'),
                        'cmd_vel_deadband': LaunchConfiguration('cmd_vel_deadband'),
                        'debug_enabled': LaunchConfiguration('policy_debug_enabled'),
                        'debug_rate_hz': LaunchConfiguration('policy_debug_rate_hz'),
                        'debug_publish_obs_topic': LaunchConfiguration(
                            'policy_debug_publish_obs_topic'
                        ),
                        'debug_publish_action_topic': LaunchConfiguration(
                            'policy_debug_publish_action_topic'
                        ),
                    }
                ],
                output='screen',
            ),
            Node(
                package='base_height_estimator',
                executable='base_height_node',
                name='base_height_estimator',
                parameters=[
                    {
                        'force_threshold': LaunchConfiguration('bhe_force_threshold'),
                        'publish_rate_hz': LaunchConfiguration('bhe_publish_rate_hz'),
                        'filter_alpha': LaunchConfiguration('bhe_filter_alpha'),
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
                        'debug_enabled': LaunchConfiguration('go2_debug_enabled'),
                        'debug_rate_hz': LaunchConfiguration('go2_debug_rate_hz'),
                    }
                ],
                output='screen',
            ),
        ]
    )
