# Copyright 2026 go2_rl_ws contributors
# SPDX-License-Identifier: MIT
"""Regression tests for go2_nav_ai_sport.launch.py."""

import importlib.util
import os
from pathlib import Path
from unittest.mock import MagicMock, patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


os.environ.setdefault('ROS_LOG_DIR', '/tmp/ros_logs')
Path(os.environ['ROS_LOG_DIR']).mkdir(parents=True, exist_ok=True)


def _load_launch_description():
    return _load_launch_module().generate_launch_description()


def _load_launch_module():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'go2_nav_ai_sport.launch.py'
    )
    spec = importlib.util.spec_from_file_location(
        'go2_nav_ai_sport_launch',
        launch_path,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context_with_arguments(overrides=None):
    launch_description = _load_launch_description()
    context = LaunchContext()

    for name, value in (overrides or {}).items():
        context.launch_configurations[name] = value

    for entity in launch_description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)

    return launch_description, context


def _node_executable(entity):
    return entity._Node__node_executable


def _node_condition(entity):
    return getattr(entity, 'condition', None) or getattr(entity, '_Action__condition', None)


def _evaluate_node_parameters(executable, overrides=None):
    module = _load_launch_module()
    launch_description = module.generate_launch_description()
    context = LaunchContext()

    for name, value in (overrides or {}).items():
        context.launch_configurations[name] = value

    for entity in launch_description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)

    inspectable_entities = [
        *launch_description.entities,
        *module._low_level_runtime_actions(),
    ]

    for entity in inspectable_entities:
        if isinstance(entity, Node) and _node_executable(entity) == executable:
            return evaluate_parameters(context, entity._Node__parameters)[0]

    raise AssertionError(f'Node executable not found: {executable}')


def test_launch_includes_ai_sport_workflow_and_deferred_low_level_nodes():
    module = _load_launch_module()
    launch_description = _load_launch_description()
    top_level_executables = {
        _node_executable(entity)
        for entity in launch_description.entities
        if isinstance(entity, Node)
    }

    assert {
        'controller_commands.py',
        'rl_nav_commands.py',
        'cmd_vel_mux.py',
        'ai_sport_cmd_vel.py',
    }.issubset(top_level_executables)

    assert top_level_executables.isdisjoint({
        'rl_actions.py',
        'rl_reach_actions.py',
        'pedipulation_executor.py',
        'base_height_node',
        'go2_controller_node',
    })

    low_level_executables = {
        _node_executable(entity)
        for entity in module._low_level_runtime_actions()
        if isinstance(entity, Node)
    }

    assert {
        'rl_actions.py',
        'rl_reach_actions.py',
        'pedipulation_executor.py',
        'base_height_node',
        'go2_controller_node',
    }.issubset(low_level_executables)


def test_nav_policy_node_is_conditioned_by_enable_nav_policy():
    launch_description, context = _context_with_arguments({'enable_nav_policy': 'false'})
    nav_nodes = [
        entity
        for entity in launch_description.entities
        if isinstance(entity, Node) and _node_executable(entity) == 'rl_nav_commands.py'
    ]

    assert len(nav_nodes) == 1
    condition = _node_condition(nav_nodes[0])
    assert isinstance(condition, IfCondition)
    assert condition.evaluate(context) is False


def test_default_goal_and_velocity_limits_evaluate_to_float_types():
    nav_params = _evaluate_node_parameters(
        'rl_nav_commands.py',
        overrides={
            'require_start_button': 'true',
            'nav_goal_0': '2',
            'nav_goal_1': '0',
            'nav_goal_2': '0',
            'nav_goal_3': '1',
            'max_linear_x': '0.4',
            'max_linear_y': '0.25',
            'max_angular_z': '0.7',
        },
    )

    assert nav_params['default_goal'] == [2.0, 0.0, 0.0, 1.0]
    assert all(isinstance(value, float) for value in nav_params['default_goal'])
    assert nav_params['require_start_button'] is True
    assert nav_params['cmd_vel_x_limit'] == 0.4
    assert nav_params['cmd_vel_y_limit'] == 0.25
    assert nav_params['cmd_vel_yaw_limit'] == 0.7


def test_default_ai_sport_launch_assumes_app_enabled_ai_sport_and_requires_start():
    nav_params = _evaluate_node_parameters('rl_nav_commands.py')
    ai_sport_params = _evaluate_node_parameters('ai_sport_cmd_vel.py')

    assert nav_params['require_start_button'] is True
    assert ai_sport_params['activate_ai_sport'] is False
    assert ai_sport_params['disable_sport_mode_before_ai_sport'] is False
    assert ai_sport_params['require_ai_sport_confirmation'] is False
    assert ai_sport_params['require_start_button'] is True
    assert ai_sport_params['exit_on_select'] is True


def test_ai_sport_driver_parameters_evaluate_to_bool_and_float_types():
    params = _evaluate_node_parameters(
        'ai_sport_cmd_vel.py',
        overrides={
            'activate_ai_sport': 'false',
            'disable_sport_mode_before_ai_sport': 'false',
            'require_ai_sport_confirmation': 'false',
            'require_start_button': 'false',
            'enable_pedipulation_handoff': 'false',
            'sport_cmd_frequency': '25',
            'sport_cmd_timeout_sec': '1',
            'sport_cmd_deadband': '0.05',
            'max_linear_x': '0.4',
            'max_linear_y': '0.2',
            'max_angular_z': '0.6',
        },
    )

    assert params['activate_ai_sport'] is False
    assert params['disable_sport_mode_before_ai_sport'] is False
    assert params['require_ai_sport_confirmation'] is False
    assert params['require_start_button'] is False
    assert params['exit_on_select'] is False
    assert params['publish_frequency'] == 25.0
    assert params['cmd_timeout_sec'] == 1.0
    assert params['cmd_deadband'] == 0.05
    assert params['max_linear_x'] == 0.4
    assert params['max_linear_y'] == 0.2
    assert params['max_angular_z'] == 0.6


def test_low_level_controller_starts_standing_and_guards_transition_select():
    params = _evaluate_node_parameters('go2_controller_node')

    assert params['stand_on_start'] is True
    assert params['wait_for_handoff_enable'] is False
    assert params['ignore_pedipulate_until_select_released'] is True
    assert params['require_fresh_pedipulation_actions_for_entry'] is True
    assert params['disable_start_walking'] is True
    assert params['enable_ai_sport_return'] is True
    assert params['ai_sport_return_sit_timeout_sec'] == 6.0
    assert params['walking_actions_topic'] == 'locomotion_actions'
    assert params['pedipulation_actions_topic'] == 'pedipulation_actions'


def test_default_handoff_strategy_uses_lie_down_prep_and_ungated_controller():
    module = _load_launch_module()
    _, context = _context_with_arguments()
    fake_prepare = MagicMock()

    with patch.object(module, '_load_motion_prep_helper', return_value=fake_prepare):
        actions = module._prepare_robot_and_launch_low_level(context)

    fake_prepare.assert_called_once_with(
        network_interface='enp1s0',
        sport_state_topic='/sportmodestate',
        allow_missing_sport_state=True,
    )

    nodes = [entity for entity in actions if isinstance(entity, Node)]
    executables = {_node_executable(entity) for entity in nodes}
    assert 'go2_standing_handoff.py' not in executables

    controller = next(
        entity for entity in nodes if _node_executable(entity) == 'go2_controller_node'
    )
    params = evaluate_parameters(context, controller._Node__parameters)[0]
    assert params['wait_for_handoff_enable'] is False


def test_standing_handoff_strategy_starts_helper_and_gates_low_level():
    module = _load_launch_module()
    _, context = _context_with_arguments({
        'low_level_handoff_strategy': 'standing_experimental',
    })

    actions = module._prepare_robot_and_launch_low_level(context)
    nodes = [entity for entity in actions if isinstance(entity, Node)]
    executables = {_node_executable(entity) for entity in nodes}

    assert 'go2_standing_handoff.py' in executables

    controller = next(
        entity for entity in nodes if _node_executable(entity) == 'go2_controller_node'
    )
    controller_params = evaluate_parameters(context, controller._Node__parameters)[0]
    assert controller_params['wait_for_handoff_enable'] is True

    helper = next(
        entity for entity in nodes if _node_executable(entity) == 'go2_standing_handoff.py'
    )
    helper_params = evaluate_parameters(context, helper._Node__parameters)[0]
    assert helper_params['network_interface'] == 'enp1s0'
    assert helper_params['sport_state_topic'] == '/sportmodestate'
    assert helper_params['stable_required_samples'] == 5
