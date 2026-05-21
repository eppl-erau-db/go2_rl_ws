import sys
from pathlib import Path
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _float_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _float_array_param(*names):
    return ParameterValue(
        [[LaunchConfiguration(name)] for name in names],
        value_type=List[float],
    )


def _load_motion_prep_helper():
    try:
        from blind_locomotion.go2_motion_prep import prepare_robot_for_low_level_control
        return prepare_robot_for_low_level_control
    except ModuleNotFoundError:
        sys.modules.pop('blind_locomotion', None)

    launch_file_path = Path(__file__).resolve()
    candidate_paths = []
    for parent in launch_file_path.parents:
        candidate_paths.append(
            parent / 'install' / 'blind_locomotion' / 'lib' / 'python3' / 'dist-packages'
        )
        candidate_paths.append(parent / 'src' / 'blind_locomotion')

    seen = set()
    for candidate in candidate_paths:
        if candidate in seen or not candidate.is_dir():
            continue
        seen.add(candidate)
        sys.path.insert(0, str(candidate))
        try:
            from blind_locomotion.go2_motion_prep import prepare_robot_for_low_level_control
            return prepare_robot_for_low_level_control
        except ModuleNotFoundError:
            sys.modules.pop('blind_locomotion', None)

    raise ModuleNotFoundError(
        'Could not import blind_locomotion.go2_motion_prep from the built '
        'workspace or source tree.'
    )


def _runtime_actions():
    return [
        LogInfo(
            msg=(
                '[go2_nav_walk_push] Manual odom workflow: after startup prep finishes, stand the robot when ready, then run in a separate terminal:\n'
                'source ~/ros2_ws/install/setup.bash && '
                'ros2 launch leg_odometry_ros leg_odom.launch.py'
            )
        ),
        LogInfo(msg='[go2_nav_walk_push] After /odom is live and calibrated, press START to enter WALKING and arm navigation.'),
        LogInfo(msg='[go2_nav_walk_push] Launch-time nav_goal_* values begin their timeout window when START is pressed.'),
        LogInfo(msg='[go2_nav_walk_push] A continuously published /nav_goal overrides the launch-time goal while it stays fresh.'),
        LogInfo(msg='[go2_nav_walk_push] Navigation publishes high-level cmd_vel; move the wireless sticks to override it while held.'),
        LogInfo(msg='[go2_nav_walk_push] Press SELECT to enter PEDIPULATION.'),
        LogInfo(msg='[go2_nav_walk_push] In PEDIPULATION: F1 runs the blind push, START cancels/returns to WALKING.'),
        LogInfo(msg='[go2_nav_walk_push] Runtime checks: ros2 topic hz /nav_cmd_vel ; ros2 topic hz /cmd_vel ; ros2 topic hz /locomotion_actions ; ros2 topic hz /pedipulation_actions ; ros2 topic hz /lowcmd'),
        Node(
            package='blind_locomotion',
            executable='controller_commands.py',
            name='controller_commands',
            remappings=[('cmd_vel', 'wireless_cmd_vel')],
            output='screen',
        ),
        Node(
            package='blind_locomotion',
            executable='rl_nav_commands.py',
            name='rl_nav_commands',
            parameters=[{
                'policy_name': LaunchConfiguration('nav_policy_name'),
                'policy_frequency': LaunchConfiguration('nav_policy_frequency'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'odom_timeout_sec': _float_param('odom_timeout_sec'),
                'lowstate_timeout_sec': _float_param('nav_lowstate_timeout_sec'),
                'goal_topic': LaunchConfiguration('nav_goal_topic'),
                'goal_timeout_sec': _float_param('nav_goal_timeout_sec'),
                'default_goal': _float_array_param(
                    'nav_goal_0',
                    'nav_goal_1',
                    'nav_goal_2',
                    'nav_goal_3',
                ),
            }],
            remappings=[('cmd_vel', 'nav_cmd_vel')],
            output='screen',
        ),
        Node(
            package='blind_locomotion',
            executable='cmd_vel_mux.py',
            name='cmd_vel_mux',
            parameters=[{
                'publish_frequency': _float_param('cmd_vel_mux_frequency'),
                'nav_timeout_sec': _float_param('cmd_vel_mux_timeout_sec'),
                'wireless_timeout_sec': _float_param('cmd_vel_mux_timeout_sec'),
                'manual_override_deadband': _float_param('manual_override_deadband'),
            }],
            output='screen',
        ),
        Node(
            package='blind_locomotion',
            executable='rl_actions.py',
            name='rl_actions',
            parameters=[{
                'policy_name': LaunchConfiguration('walking_policy_name'),
                'policy_frequency': LaunchConfiguration('walking_policy_frequency'),
                'scale_factor': _float_param('walking_scale_factor'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'odom_timeout_sec': _float_param('odom_timeout_sec'),
                'cmd_vel_deadband': _float_param('cmd_vel_deadband'),
                'base_height_topic': LaunchConfiguration('base_height_topic'),
                'base_height_timeout_sec': _float_param('base_height_timeout_sec'),
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
                'scale_factor': _float_param('pedipulation_scale_factor'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'odom_timeout_sec': _float_param('odom_timeout_sec'),
                'base_height_topic': LaunchConfiguration('base_height_topic'),
                'lowstate_timeout_sec': _float_param('lowstate_timeout_sec'),
                'pose_command_topic': LaunchConfiguration('pose_command_topic'),
                'pose_timeout_sec': _float_param('pose_timeout_sec'),
                'pose_x_min': _float_param('pose_x_min'),
                'pose_x_max': _float_param('pose_x_max'),
                'pose_y_min': _float_param('pose_y_min'),
                'pose_y_max': _float_param('pose_y_max'),
                'pose_z_min': _float_param('pose_z_min'),
                'pose_z_max': _float_param('pose_z_max'),
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
                'trajectory_duration_sec': _float_param('push_duration_sec'),
                'pose_x_min': _float_param('pose_x_min'),
                'pose_x_max': _float_param('pose_x_max'),
                'pose_y_min': _float_param('pose_y_min'),
                'pose_y_max': _float_param('pose_y_max'),
                'pose_z_min': _float_param('pose_z_min'),
                'pose_z_max': _float_param('pose_z_max'),
                'push_mid_z': _float_param('push_mid_z'),
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
    ]


def _prepare_robot_and_launch(context):
    prepare_robot_for_low_level_control = _load_motion_prep_helper()

    prepare_robot_for_low_level_control(
        network_interface=LaunchConfiguration('network_interface').perform(context),
        sport_state_topic=LaunchConfiguration('sport_state_topic').perform(context),
        allow_missing_sport_state=True,
    )

    return [
        LogInfo(
            msg='[go2_nav_walk_push] Startup prep complete: high-level motion is released or already inactive; runtime nodes are starting.'
        ),
        *_runtime_actions(),
    ]


def generate_launch_description():
    network_interface_arg = DeclareLaunchArgument(
        'network_interface',
        default_value='enp1s0',
        description='Robot Ethernet interface used by the Unitree SDK helper binaries',
    )
    sport_state_topic_arg = DeclareLaunchArgument(
        'sport_state_topic',
        default_value='/sportmodestate',
        description='Sport-mode state topic used to verify the robot reached lie-down mode before shutting off sport_mode',
    )
    nav_policy_name_arg = DeclareLaunchArgument(
        'nav_policy_name',
        default_value='nav_policy',
        description='Navigation ONNX policy file (without .onnx extension)',
    )
    nav_policy_frequency_arg = DeclareLaunchArgument(
        'nav_policy_frequency',
        default_value='12',
        description='Navigation policy inference frequency in Hz',
    )
    nav_goal_topic_arg = DeclareLaunchArgument(
        'nav_goal_topic',
        default_value='nav_goal',
        description='Topic providing the 4-element navigation goal vector',
    )
    nav_goal_timeout_sec_arg = DeclareLaunchArgument(
        'nav_goal_timeout_sec',
        default_value='0.5',
        description='Lifetime of a fresh /nav_goal message, and of the launch-time nav goal after START is pressed',
    )
    nav_goal_0_arg = DeclareLaunchArgument(
        'nav_goal_0',
        default_value='0.0',
        description='Default navigation goal element 0 (sim example uses 2.0)',
    )
    nav_goal_1_arg = DeclareLaunchArgument(
        'nav_goal_1',
        default_value='0.0',
        description='Default navigation goal element 1',
    )
    nav_goal_2_arg = DeclareLaunchArgument(
        'nav_goal_2',
        default_value='0.0',
        description='Default navigation goal element 2',
    )
    nav_goal_3_arg = DeclareLaunchArgument(
        'nav_goal_3',
        default_value='0.0',
        description='Default navigation goal element 3 (sim example uses 0.75)',
    )
    nav_lowstate_timeout_sec_arg = DeclareLaunchArgument(
        'nav_lowstate_timeout_sec',
        default_value='0.5',
        description='Pause nav policy when /lowstate age exceeds this timeout',
    )
    cmd_vel_mux_frequency_arg = DeclareLaunchArgument(
        'cmd_vel_mux_frequency',
        default_value='50.0',
        description='cmd_vel arbitration frequency in Hz',
    )
    cmd_vel_mux_timeout_sec_arg = DeclareLaunchArgument(
        'cmd_vel_mux_timeout_sec',
        default_value='0.5',
        description='Timeout for nav/wireless cmd_vel inputs to the mux',
    )
    manual_override_deadband_arg = DeclareLaunchArgument(
        'manual_override_deadband',
        default_value='0.1',
        description='Wireless stick magnitude required to override navigation cmd_vel',
    )
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
        description='Odometry topic used by all three policies',
    )
    odom_timeout_sec_arg = DeclareLaunchArgument(
        'odom_timeout_sec',
        default_value='0.5',
        description='Pause nav, locomotion, or pedipulation when odom age exceeds this timeout',
    )
    cmd_vel_deadband_arg = DeclareLaunchArgument(
        'cmd_vel_deadband',
        default_value='0.1',
        description='Deadband for the locomotion policy cmd_vel observation component',
    )
    base_height_topic_arg = DeclareLaunchArgument(
        'base_height_topic',
        default_value='/base_height',
        description='Base height topic used by locomotion and pedipulation',
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
        network_interface_arg,
        sport_state_topic_arg,
        nav_policy_name_arg,
        nav_policy_frequency_arg,
        nav_goal_topic_arg,
        nav_goal_timeout_sec_arg,
        nav_goal_0_arg,
        nav_goal_1_arg,
        nav_goal_2_arg,
        nav_goal_3_arg,
        nav_lowstate_timeout_sec_arg,
        cmd_vel_mux_frequency_arg,
        cmd_vel_mux_timeout_sec_arg,
        manual_override_deadband_arg,
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
        OpaqueFunction(function=_prepare_robot_and_launch),
    ])
