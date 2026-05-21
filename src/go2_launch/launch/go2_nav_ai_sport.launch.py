# Copyright 2026 go2_rl_ws contributors
# SPDX-License-Identifier: MIT
import sys
from pathlib import Path
from typing import List

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events.process import ShutdownProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _bool_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _float_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _int_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def _float_array_param(*names):
    return ParameterValue(
        [[LaunchConfiguration(name)] for name in names],
        value_type=List[float],
    )


def _is_true(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


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


def _make_ai_sport_cmd_vel_node(*, activate_ai_sport):
    return Node(
        package='blind_locomotion',
        executable='ai_sport_cmd_vel.py',
        name='ai_sport_cmd_vel',
        parameters=[{
            'activate_ai_sport': activate_ai_sport,
            'disable_sport_mode_before_ai_sport': _bool_param(
                'disable_sport_mode_before_ai_sport'
            ),
            'require_ai_sport_confirmation': _bool_param(
                'require_ai_sport_confirmation'
            ),
            'require_start_button': _bool_param('require_start_button'),
            'exit_on_select': _bool_param('enable_pedipulation_handoff'),
            'publish_frequency': _float_param('sport_cmd_frequency'),
            'cmd_timeout_sec': _float_param('sport_cmd_timeout_sec'),
            'cmd_deadband': _float_param('sport_cmd_deadband'),
            'max_linear_x': _float_param('max_linear_x'),
            'max_linear_y': _float_param('max_linear_y'),
            'max_angular_z': _float_param('max_angular_z'),
        }],
        output='screen',
    )


def _ai_sport_phase_actions(*, activate_ai_sport):
    ai_sport_cmd_vel_node = _make_ai_sport_cmd_vel_node(
        activate_ai_sport=activate_ai_sport
    )
    return [
        ai_sport_cmd_vel_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=ai_sport_cmd_vel_node,
                on_exit=_on_ai_sport_exit,
            ),
            condition=IfCondition(LaunchConfiguration('enable_pedipulation_handoff')),
        ),
    ]


def _shutdown_process_action(target_node):
    return EmitEvent(
        event=ShutdownProcess(
            process_matcher=lambda action, target=target_node: action is target
        )
    )


def _on_low_level_controller_exit(event, context, low_level_helper_nodes):
    if context.is_shutdown:
        return []

    if not _is_true(LaunchConfiguration('enable_ai_sport_return').perform(context)):
        return []

    if event.returncode != 0:
        return [
            LogInfo(
                msg=(
                    '[go2_nav_ai_sport] Low-level controller exited with '
                    f'return code {event.returncode}; not returning to ai_sport.'
                )
            )
        ]

    return [
        LogInfo(
            msg=(
                '[go2_nav_ai_sport] Low-level controller requested ai_sport '
                'return; stopping low-level helper nodes and waiting for /lowcmd '
                'to clear.'
            )
        ),
        *[_shutdown_process_action(node) for node in low_level_helper_nodes],
        TimerAction(
            period=LaunchConfiguration('ai_sport_return_delay_sec'),
            actions=[
                LogInfo(
                    msg=(
                        '[go2_nav_ai_sport] Restarting ai_sport bridge. Press '
                        'START to arm motion after the robot is ready.'
                    )
                ),
                *_ai_sport_phase_actions(
                    activate_ai_sport=_bool_param('activate_ai_sport_on_return')
                ),
            ],
        ),
    ]


def _make_standing_handoff_node():
    return Node(
        package='blind_locomotion',
        executable='go2_standing_handoff.py',
        name='go2_standing_handoff',
        parameters=[{
            'network_interface': LaunchConfiguration('network_interface'),
            'sport_state_topic': LaunchConfiguration('sport_state_topic'),
            'state_timeout_sec': _float_param('standing_handoff_state_timeout_sec'),
            'stable_timeout_sec': _float_param('standing_handoff_stable_timeout_sec'),
            'stable_required_samples': _int_param('standing_handoff_required_samples'),
            'max_state_age_sec': _float_param('standing_handoff_max_state_age_sec'),
            'min_body_height': _float_param('standing_handoff_min_body_height'),
            'max_linear_velocity': _float_param(
                'standing_handoff_max_linear_velocity'
            ),
            'max_yaw_speed': _float_param('standing_handoff_max_yaw_speed'),
            'stop_publish_frequency': _float_param(
                'standing_handoff_stop_frequency'
            ),
            'enable_publish_duration_sec': _float_param(
                'standing_handoff_enable_publish_duration_sec'
            ),
            'shutoff_timeout_sec': _float_param('standing_handoff_shutoff_timeout_sec'),
        }],
        output='screen',
    )


def _low_level_runtime_actions(*, wait_for_handoff_enable=False):
    rl_actions_node = Node(
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
    )
    base_height_node = Node(
        package='base_height_estimator',
        executable='base_height_node',
        name='base_height_estimator',
        parameters=[{
            'force_threshold': 20.0,
            'publish_rate_hz': 50.0,
            'filter_alpha': 0.2,
        }],
        output='screen',
    )
    rl_reach_actions_node = Node(
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
    )
    pedipulation_executor_node = Node(
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
    )
    go2_controller_node = Node(
        package='rl_deploy',
        executable='go2_controller_node',
        name='go2_controller_node',
        parameters=[{
            'enable_joint_limit_monitor': _bool_param('enable_joint_limit_monitor'),
            'stand_on_start': _bool_param('stand_on_low_level_start'),
            'ignore_pedipulate_until_select_released': True,
            'require_fresh_pedipulation_actions_for_entry': True,
            'disable_start_walking': True,
            'enable_ai_sport_return': _bool_param('enable_ai_sport_return'),
            'ai_sport_return_sit_timeout_sec': _float_param(
                'ai_sport_return_sit_timeout_sec'
            ),
            'wait_for_handoff_enable': wait_for_handoff_enable,
            'walking_actions_topic': 'locomotion_actions',
            'pedipulation_actions_topic': 'pedipulation_actions',
            'pedipulation_execute_service': 'pedipulation/execute_push',
            'pedipulation_cancel_service': 'pedipulation/cancel_push',
        }],
        output='screen',
    )
    low_level_helper_nodes = [
        rl_actions_node,
        base_height_node,
        rl_reach_actions_node,
        pedipulation_executor_node,
    ]

    return [
        LogInfo(
            msg=(
                '[go2_nav_ai_sport] Low-level stack is starting in STANDING. '
                'Restart leg odometry once the robot is standing and steady.'
            )
        ),
        LogInfo(
            msg=(
                '[go2_nav_ai_sport] After /odom is fresh, press SELECT again '
                'to enter PEDIPULATION.'
            )
        ),
        LogInfo(
            msg=(
                '[go2_nav_ai_sport] In PEDIPULATION: F1 runs the blind push, '
                'UP cancels/returns to STANDING, DOWN sits/returns to idle. '
                'From idle/sitting, SELECT returns to ai_sport.'
            )
        ),
        LogInfo(
            msg=(
                '[go2_nav_ai_sport] Low-level checks: ros2 topic hz /odom ; '
                'ros2 topic hz /locomotion_actions ; '
                'ros2 topic hz /pedipulation_actions ; ros2 topic hz /lowcmd'
            )
        ),
        rl_actions_node,
        base_height_node,
        rl_reach_actions_node,
        pedipulation_executor_node,
        go2_controller_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=go2_controller_node,
                on_exit=(
                    lambda event, context, nodes=low_level_helper_nodes:
                    _on_low_level_controller_exit(event, context, nodes)
                ),
            ),
            condition=IfCondition(LaunchConfiguration('enable_ai_sport_return')),
        ),
    ]


def _prepare_robot_and_launch_low_level(context):
    handoff_strategy = (
        LaunchConfiguration('low_level_handoff_strategy')
        .perform(context)
        .strip()
        .lower()
    )

    if handoff_strategy == 'lie_down':
        prepare_robot_for_low_level_control = _load_motion_prep_helper()

        prepare_robot_for_low_level_control(
            network_interface=LaunchConfiguration('network_interface').perform(context),
            sport_state_topic=LaunchConfiguration('sport_state_topic').perform(context),
            allow_missing_sport_state=True,
        )

        return [
            LogInfo(
                msg=(
                    '[go2_nav_ai_sport] Sport/ai_sport motion is released; '
                    'low-level pedipulation runtime nodes are starting.'
                )
            ),
            *_low_level_runtime_actions(wait_for_handoff_enable=False),
        ]

    if handoff_strategy == 'standing_experimental':
        return [
            LogInfo(
                msg=(
                    '[go2_nav_ai_sport] Experimental standing handoff selected: '
                    'starting low-level runtime gated on /low_level_handoff/enable.'
                )
            ),
            *_low_level_runtime_actions(wait_for_handoff_enable=True),
            _make_standing_handoff_node(),
        ]

    raise RuntimeError(
        "Unsupported low_level_handoff_strategy "
        f"{handoff_strategy!r}; expected 'lie_down' or 'standing_experimental'."
    )


def _on_ai_sport_exit(event, context):
    if context.is_shutdown:
        return []

    if not _is_true(LaunchConfiguration('enable_pedipulation_handoff').perform(context)):
        return []

    if event.returncode != 0:
        return [
            LogInfo(
                msg=(
                    '[go2_nav_ai_sport] ai_sport bridge exited with return code '
                    f'{event.returncode}; not starting low-level handoff.'
                )
            )
        ]

    return [
        LogInfo(
            msg=(
                '[go2_nav_ai_sport] SELECT handoff received: preparing '
                'high-level to low-level takeover.'
            )
        ),
        OpaqueFunction(function=_prepare_robot_and_launch_low_level),
    ]


def generate_launch_description():
    enable_nav_policy_arg = DeclareLaunchArgument(
        'enable_nav_policy',
        default_value='true',
        description='Start the RL nav policy. Set false for wireless-only ai_sport testing.',
    )
    enable_pedipulation_handoff_arg = DeclareLaunchArgument(
        'enable_pedipulation_handoff',
        default_value='true',
        description='Let SELECT stop ai_sport and transition into low-level pedipulation.',
    )
    enable_ai_sport_return_arg = DeclareLaunchArgument(
        'enable_ai_sport_return',
        default_value='true',
        description='Let SELECT from low-level idle/sitting stop low-level control and return to ai_sport.',
    )
    activate_ai_sport_on_return_arg = DeclareLaunchArgument(
        'activate_ai_sport_on_return',
        default_value='true',
        description='Publish robot_state ServiceSwitch requests when returning from low-level control to ai_sport.',
    )
    ai_sport_return_delay_sec_arg = DeclareLaunchArgument(
        'ai_sport_return_delay_sec',
        default_value='1.0',
        description='Delay after low-level controller exit before restarting the ai_sport bridge.',
    )
    ai_sport_return_sit_timeout_sec_arg = DeclareLaunchArgument(
        'ai_sport_return_sit_timeout_sec',
        default_value='6.0',
        description='Maximum time for the low-level controller to hold sitting before exiting for ai_sport return.',
    )
    network_interface_arg = DeclareLaunchArgument(
        'network_interface',
        default_value='enp1s0',
        description='Robot Ethernet interface used by the Unitree SDK helper binaries',
    )
    low_level_handoff_strategy_arg = DeclareLaunchArgument(
        'low_level_handoff_strategy',
        default_value='lie_down',
        description=(
            'Low-level handoff strategy: lie_down (default) or '
            'standing_experimental (no StandDown; waits for stable upright state '
            'then releases high-level motion).'
        ),
    )
    sport_state_topic_arg = DeclareLaunchArgument(
        'sport_state_topic',
        default_value='/sportmodestate',
        description='Sport-mode state topic used to verify lie-down before shutting off sport_mode',
    )
    activate_ai_sport_arg = DeclareLaunchArgument(
        'activate_ai_sport',
        default_value='false',
        description='Publish robot_state ServiceSwitch requests to enable ai_sport. Leave false when ai_sport was enabled from the Unitree app.',
    )
    disable_sport_mode_before_ai_sport_arg = DeclareLaunchArgument(
        'disable_sport_mode_before_ai_sport',
        default_value='false',
        description='Switch sport_mode off before requesting ai_sport.',
    )
    require_ai_sport_confirmation_arg = DeclareLaunchArgument(
        'require_ai_sport_confirmation',
        default_value='false',
        description='Do not forward cmd_vel until robot_state confirms ai_sport status=1.',
    )
    require_start_button_arg = DeclareLaunchArgument(
        'require_start_button',
        default_value='true',
        description='Require wireless START before the nav policy and ai_sport bridge forward motion commands.',
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
        description='Lifetime of a fresh /nav_goal message, and of the launch-time nav goal after START is pressed or the node starts with START gating disabled',
    )
    nav_goal_0_arg = DeclareLaunchArgument(
        'nav_goal_0',
        default_value='0.0',
        description='Default navigation goal element 0',
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
        description='Default navigation goal element 3',
    )
    nav_lowstate_timeout_sec_arg = DeclareLaunchArgument(
        'nav_lowstate_timeout_sec',
        default_value='0.5',
        description='Pause nav policy when /lowstate age exceeds this timeout',
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic used by nav, locomotion, and pedipulation policies',
    )
    odom_timeout_sec_arg = DeclareLaunchArgument(
        'odom_timeout_sec',
        default_value='0.5',
        description='Pause policies when odom age exceeds this timeout',
    )
    cmd_vel_mux_frequency_arg = DeclareLaunchArgument(
        'cmd_vel_mux_frequency',
        default_value='12.0',
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
    sport_cmd_frequency_arg = DeclareLaunchArgument(
        'sport_cmd_frequency',
        default_value='50.0',
        description='ai_sport Move request publish frequency in Hz',
    )
    sport_cmd_timeout_sec_arg = DeclareLaunchArgument(
        'sport_cmd_timeout_sec',
        default_value='0.5',
        description='Stop ai_sport movement when muxed cmd_vel age exceeds this timeout',
    )
    sport_cmd_deadband_arg = DeclareLaunchArgument(
        'sport_cmd_deadband',
        default_value='0.02',
        description='Deadband applied by ai_sport bridge before sending Move requests',
    )
    max_linear_x_arg = DeclareLaunchArgument(
        'max_linear_x',
        default_value='0.5',
        description='Absolute forward/backward velocity limit sent to ai_sport/nav',
    )
    max_linear_y_arg = DeclareLaunchArgument(
        'max_linear_y',
        default_value='0.3',
        description='Absolute lateral velocity limit sent to ai_sport/nav',
    )
    max_angular_z_arg = DeclareLaunchArgument(
        'max_angular_z',
        default_value='0.8',
        description='Absolute yaw velocity limit sent to ai_sport/nav',
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
    stand_on_low_level_start_arg = DeclareLaunchArgument(
        'stand_on_low_level_start',
        default_value='true',
        description='Command STANDING as soon as the low-level controller starts after handoff.',
    )
    standing_handoff_state_timeout_sec_arg = DeclareLaunchArgument(
        'standing_handoff_state_timeout_sec',
        default_value='5.0',
        description='Timeout waiting for /sportmodestate in standing handoff.',
    )
    standing_handoff_stable_timeout_sec_arg = DeclareLaunchArgument(
        'standing_handoff_stable_timeout_sec',
        default_value='8.0',
        description='Timeout waiting for stable upright state in standing handoff.',
    )
    standing_handoff_required_samples_arg = DeclareLaunchArgument(
        'standing_handoff_required_samples',
        default_value='5',
        description='Consecutive stable sport-mode samples required before ReleaseMode.',
    )
    standing_handoff_max_state_age_sec_arg = DeclareLaunchArgument(
        'standing_handoff_max_state_age_sec',
        default_value='0.25',
        description='Maximum sport-mode state age accepted by standing handoff.',
    )
    standing_handoff_min_body_height_arg = DeclareLaunchArgument(
        'standing_handoff_min_body_height',
        default_value='0.18',
        description='Minimum body height accepted by standing handoff.',
    )
    standing_handoff_max_linear_velocity_arg = DeclareLaunchArgument(
        'standing_handoff_max_linear_velocity',
        default_value='0.05',
        description='Maximum linear velocity norm accepted by standing handoff.',
    )
    standing_handoff_max_yaw_speed_arg = DeclareLaunchArgument(
        'standing_handoff_max_yaw_speed',
        default_value='0.10',
        description='Maximum yaw speed accepted by standing handoff.',
    )
    standing_handoff_stop_frequency_arg = DeclareLaunchArgument(
        'standing_handoff_stop_frequency',
        default_value='20.0',
        description='StopMove publish frequency during standing handoff.',
    )
    standing_handoff_enable_publish_duration_sec_arg = DeclareLaunchArgument(
        'standing_handoff_enable_publish_duration_sec',
        default_value='2.0',
        description='How long to publish the low-level handoff enable signal.',
    )
    standing_handoff_shutoff_timeout_sec_arg = DeclareLaunchArgument(
        'standing_handoff_shutoff_timeout_sec',
        default_value='20.0',
        description='Timeout for go2_shutoff_motion during standing handoff.',
    )

    ai_sport_phase_actions = _ai_sport_phase_actions(
        activate_ai_sport=_bool_param('activate_ai_sport')
    )

    return LaunchDescription([
        enable_nav_policy_arg,
        enable_pedipulation_handoff_arg,
        enable_ai_sport_return_arg,
        activate_ai_sport_on_return_arg,
        ai_sport_return_delay_sec_arg,
        ai_sport_return_sit_timeout_sec_arg,
        network_interface_arg,
        low_level_handoff_strategy_arg,
        sport_state_topic_arg,
        activate_ai_sport_arg,
        disable_sport_mode_before_ai_sport_arg,
        require_ai_sport_confirmation_arg,
        require_start_button_arg,
        nav_policy_name_arg,
        nav_policy_frequency_arg,
        nav_goal_topic_arg,
        nav_goal_timeout_sec_arg,
        nav_goal_0_arg,
        nav_goal_1_arg,
        nav_goal_2_arg,
        nav_goal_3_arg,
        nav_lowstate_timeout_sec_arg,
        odom_topic_arg,
        odom_timeout_sec_arg,
        cmd_vel_mux_frequency_arg,
        cmd_vel_mux_timeout_sec_arg,
        manual_override_deadband_arg,
        sport_cmd_frequency_arg,
        sport_cmd_timeout_sec_arg,
        sport_cmd_deadband_arg,
        max_linear_x_arg,
        max_linear_y_arg,
        max_angular_z_arg,
        walking_policy_name_arg,
        walking_policy_frequency_arg,
        walking_scale_factor_arg,
        pedipulation_policy_name_arg,
        pedipulation_policy_frequency_arg,
        pedipulation_scale_factor_arg,
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
        stand_on_low_level_start_arg,
        standing_handoff_state_timeout_sec_arg,
        standing_handoff_stable_timeout_sec_arg,
        standing_handoff_required_samples_arg,
        standing_handoff_max_state_age_sec_arg,
        standing_handoff_min_body_height_arg,
        standing_handoff_max_linear_velocity_arg,
        standing_handoff_max_yaw_speed_arg,
        standing_handoff_stop_frequency_arg,
        standing_handoff_enable_publish_duration_sec_arg,
        standing_handoff_shutoff_timeout_sec_arg,
        LogInfo(
            msg=(
                '[go2_nav_ai_sport] ai_sport phase assumes ai_sport was enabled '
                'from the Unitree app and publishes /api/sport/request Move commands.'
            )
        ),
        LogInfo(
            msg=(
                '[go2_nav_ai_sport] Motion is held until START is pressed by default. '
                'Wireless sticks override nav while held; A stops/disarms, DOWN '
                'stands down/disarms, B damps/disarms. Press START again to stop/disarm.'
            )
        ),
        LogInfo(
            msg=(
                '[go2_nav_ai_sport] At the target pose, press SELECT to stop ai_sport, '
                'prepare the configured low-level handoff, and start low-level standing.'
            )
        ),
        LogInfo(
            msg=(
                '[go2_nav_ai_sport] After low-level standing, manually restart leg '
                'odometry; once /odom is fresh, press SELECT again for PEDIPULATION, '
                'then F1 for the pre-recorded push. To return to ai_sport after '
                'the push, press DOWN to sit/idle, then SELECT.'
            )
        ),
        LogInfo(
            msg=(
                '[go2_nav_ai_sport] Nav goal test: ros2 topic pub /nav_goal '
                'std_msgs/msg/Float32MultiArray "{data: [1.0, 0.0, 0.0, 0.0]}" -r 10'
            )
        ),
        LogInfo(
            msg=(
                '[go2_nav_ai_sport] Runtime checks before handoff: '
                'ros2 topic hz /wireless_cmd_vel ; ros2 topic hz /nav_cmd_vel ; '
                'ros2 topic hz /cmd_vel ; ros2 topic echo /api/sport/request'
            )
        ),
        Node(
            package='blind_locomotion',
            executable='controller_commands.py',
            name='controller_commands',
            remappings=[('cmd_vel', 'wireless_cmd_vel')],
            output='screen',
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('enable_nav_policy')),
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
                'require_start_button': _bool_param('require_start_button'),
                'default_goal': _float_array_param(
                    'nav_goal_0',
                    'nav_goal_1',
                    'nav_goal_2',
                    'nav_goal_3',
                ),
                'cmd_vel_x_limit': _float_param('max_linear_x'),
                'cmd_vel_y_limit': _float_param('max_linear_y'),
                'cmd_vel_yaw_limit': _float_param('max_angular_z'),
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
        *ai_sport_phase_actions,
    ])
