# Copyright 2026 go2_rl_ws contributors
# SPDX-License-Identifier: MIT
"""
One-shot walking bring-up for the Go2.

Sequence, starting from a powered-on robot (which stands on its own):

  1. Lie the robot down via the sport client and shut off sport mode
     (blind_locomotion.go2_motion_prep, runs before any node starts).
  2. Start controller_commands, rl_actions and go2_controller_node with
     stand_on_start so the low-level controller stands the robot up.
  3. Run go2_stand_ready_gate, which exits once go2_controller_node reports a
     confirmed stand.
  4. Start leg odometry headless from the odometry workspace. It is launched
     in a child shell that sources that workspace, so the terminal running
     this launch must NOT have it sourced (it breaks the Unitree SDK helpers
     used in step 1).
  5. Press START on the wireless remote to enter WALKING.

Run from the workspace root with:  source setup.sh && ros2 launch go2_launch go2_walk.launch.py
"""

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _bool_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _float_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _int_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


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


def _make_stand_ready_gate_node():
    return Node(
        package='blind_locomotion',
        executable='go2_stand_ready_gate.py',
        name='go2_stand_ready_gate',
        parameters=[{
            'stand_ready_topic': LaunchConfiguration('stand_ready_topic'),
            'required_consecutive_samples': _int_param('stand_ready_samples'),
            'settle_sec': _float_param('stand_ready_settle_sec'),
            'timeout_sec': _float_param('stand_ready_timeout_sec'),
        }],
        output='screen',
    )


def _make_leg_odometry_process(workspace_root, headless):
    """
    Launch leg odometry inside a shell that sources the odometry workspace.

    The parent launch process deliberately does not have that workspace in its
    environment, so it is sourced only for this child.
    """
    setup_script = os.path.join(workspace_root, 'install', 'setup.bash')
    headless_value = 'true' if headless else 'false'
    return ExecuteProcess(
        cmd=[
            'bash', '-c',
            'source "$0" && exec ros2 launch leg_odometry_ros leg_odom.launch.py headless:=$1',
            setup_script,
            headless_value,
        ],
        name='leg_odometry',
        output='screen',
        emulate_tty=True,
    )


def _runtime_actions():
    return [
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
                'policy_name': LaunchConfiguration('policy_name'),
                'policy_frequency': LaunchConfiguration('policy_frequency'),
                'scale_factor': LaunchConfiguration('scale_factor'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'odom_timeout_sec': LaunchConfiguration('odom_timeout_sec'),
                'cmd_vel_deadband': LaunchConfiguration('cmd_vel_deadband'),
                'base_height_topic': LaunchConfiguration('base_height_topic'),
                'base_height_timeout_sec': LaunchConfiguration('base_height_timeout_sec'),
            }],
            output='screen',
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('enable_base_height_estimator')),
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
            package='rl_deploy',
            executable='go2_controller_node',
            name='go2_controller_node',
            parameters=[{
                'enable_joint_limit_monitor': _bool_param('enable_joint_limit_monitor'),
                'stand_on_start': _bool_param('auto_stand'),
            }],
            output='screen',
        ),
    ]


def _on_stand_ready_exit(event, context):
    if context.is_shutdown:
        return []

    if event.returncode != 0:
        return [
            LogInfo(
                msg=(
                    '[go2_walk] Stand-ready gate exited with return code '
                    f'{event.returncode}; NOT starting leg odometry. '
                    'Press UP to stand, then start leg odometry manually.'
                )
            )
        ]

    if not _is_true(LaunchConfiguration('enable_leg_odometry').perform(context)):
        return [
            LogInfo(
                msg=(
                    '[go2_walk] Robot is standing. Leg odometry is disabled '
                    '(enable_leg_odometry:=false); start /odom yourself, then press START.'
                )
            )
        ]

    workspace_root = os.path.expanduser(
        LaunchConfiguration('leg_odom_workspace').perform(context)
    )
    headless = _is_true(LaunchConfiguration('leg_odom_headless').perform(context))

    return [
        LogInfo(
            msg=(
                '[go2_walk] Robot is standing. Starting leg odometry from '
                f'{workspace_root} (headless={headless}).'
            )
        ),
        _make_leg_odometry_process(workspace_root, headless),
        LogInfo(
            msg=(
                '[go2_walk] Once /odom is live (ros2 topic hz /odom), '
                'press START to enter WALKING. Sticks command velocity; '
                'SELECT/UP returns to STANDING, A damps, B kills.'
            )
        ),
    ]


def _warn_if_odom_workspace_sourced(workspace_root):
    """Warn when the odometry workspace is sourced; the Unitree SDK helpers misbehave then."""
    install_dir = os.path.join(workspace_root, 'install')
    prefixes = os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
    if any(p and os.path.realpath(p).startswith(os.path.realpath(install_dir)) for p in prefixes):
        return [
            LogInfo(
                msg=(
                    f'[go2_walk] WARNING: {install_dir} is sourced in this terminal. '
                    'go2_shutoff_motion / go2_sport_client are known to fail in that '
                    'environment. Open a fresh terminal and only `source setup.sh` '
                    'from the go2_rl_ws root.'
                )
            )
        ]
    return []


def _prepare_robot_and_launch(context):
    workspace_root = os.path.expanduser(
        LaunchConfiguration('leg_odom_workspace').perform(context)
    )
    actions = _warn_if_odom_workspace_sourced(workspace_root)

    if _is_true(LaunchConfiguration('skip_motion_prep').perform(context)):
        actions.append(
            LogInfo(
                msg=(
                    '[go2_walk] skip_motion_prep:=true: assuming the robot is '
                    'lying down with sport mode already off.'
                )
            )
        )
    else:
        prepare_robot_for_low_level_control = _load_motion_prep_helper()
        prepare_robot_for_low_level_control(
            network_interface=LaunchConfiguration('network_interface').perform(context),
            sport_state_topic=LaunchConfiguration('sport_state_topic').perform(context),
            allow_missing_sport_state=True,
        )
        actions.append(
            LogInfo(
                msg=(
                    '[go2_walk] Robot is lying down and sport mode is off; '
                    'starting low-level runtime nodes.'
                )
            )
        )

    stand_ready_gate = _make_stand_ready_gate_node()

    if _is_true(LaunchConfiguration('auto_stand').perform(context)):
        actions.append(
            LogInfo(msg='[go2_walk] auto_stand:=true: controller will stand the robot now.')
        )
    else:
        actions.append(
            LogInfo(msg='[go2_walk] auto_stand:=false: press UP on the remote to stand.')
        )

    actions.extend(_runtime_actions())
    actions.append(stand_ready_gate)
    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=stand_ready_gate,
                on_exit=_on_stand_ready_exit,
            )
        )
    )
    return actions


def generate_launch_description():
    # Bring-up sequencing args.
    network_interface_arg = DeclareLaunchArgument(
        'network_interface',
        default_value='enp1s0',
        description='Robot Ethernet interface used by the Unitree SDK helper binaries',
    )
    sport_state_topic_arg = DeclareLaunchArgument(
        'sport_state_topic',
        default_value='/sportmodestate',
        description='Sport-mode state topic used to confirm lie-down before shutting off '
                    'sport mode',
    )
    skip_motion_prep_arg = DeclareLaunchArgument(
        'skip_motion_prep',
        default_value='false',
        description='Skip the automatic lie-down + sport-mode shutoff (robot must already '
                    'be down with sport mode off)',
    )
    auto_stand_arg = DeclareLaunchArgument(
        'auto_stand',
        default_value='true',
        description='Have go2_controller_node stand the robot as soon as it starts '
                    '(otherwise press UP)',
    )
    enable_leg_odometry_arg = DeclareLaunchArgument(
        'enable_leg_odometry',
        default_value='true',
        description='Launch leg_odometry_ros once the robot is confirmed standing',
    )
    leg_odom_workspace_arg = DeclareLaunchArgument(
        'leg_odom_workspace',
        default_value='~/ros2_ws',
        description='Workspace containing leg_odometry_ros; sourced only inside the '
                    'odometry child process',
    )
    leg_odom_headless_arg = DeclareLaunchArgument(
        'leg_odom_headless',
        default_value='true',
        description='Pass headless:=true to leg_odom.launch.py (no RViz)',
    )
    stand_ready_topic_arg = DeclareLaunchArgument(
        'stand_ready_topic',
        default_value='stand_ready',
        description='Bool topic from go2_controller_node that is true while standing with '
                    'a good posture',
    )
    stand_ready_samples_arg = DeclareLaunchArgument(
        'stand_ready_samples',
        default_value='10',
        description='Consecutive true stand_ready samples (20 Hz) required before '
                    'releasing the gate',
    )
    stand_ready_settle_sec_arg = DeclareLaunchArgument(
        'stand_ready_settle_sec',
        default_value='1.0',
        description='Extra settle time after a confirmed stand before starting leg odometry',
    )
    stand_ready_timeout_sec_arg = DeclareLaunchArgument(
        'stand_ready_timeout_sec',
        default_value='30.0',
        description='Give up waiting for a confirmed stand after this many seconds (0 = '
                    'wait forever)',
    )

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
    base_height_topic_arg = DeclareLaunchArgument(
        'base_height_topic',
        default_value='/base_height',
        description='Base height topic used for 49-dim policies',
    )
    base_height_timeout_sec_arg = DeclareLaunchArgument(
        'base_height_timeout_sec',
        default_value='0.5',
        description='Pause policy when base-height age exceeds this timeout (49-dim only)',
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

    return LaunchDescription([
        network_interface_arg,
        sport_state_topic_arg,
        skip_motion_prep_arg,
        auto_stand_arg,
        enable_leg_odometry_arg,
        leg_odom_workspace_arg,
        leg_odom_headless_arg,
        stand_ready_topic_arg,
        stand_ready_samples_arg,
        stand_ready_settle_sec_arg,
        stand_ready_timeout_sec_arg,
        policy_name_arg,
        policy_frequency_arg,
        scale_factor_arg,
        odom_topic_arg,
        odom_timeout_sec_arg,
        cmd_vel_deadband_arg,
        base_height_topic_arg,
        base_height_timeout_sec_arg,
        enable_joint_limit_monitor_arg,
        enable_base_height_estimator_arg,
        LogInfo(
            msg=(
                '[go2_walk] Bring-up: lie down -> sport mode off -> stand -> '
                'leg odometry -> press START to walk.'
            )
        ),
        OpaqueFunction(function=_prepare_robot_and_launch),
    ])
