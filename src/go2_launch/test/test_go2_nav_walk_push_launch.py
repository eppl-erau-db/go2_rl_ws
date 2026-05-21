"""Regression tests for typed parameters in go2_nav_walk_push.launch.py."""

import importlib.util
import os
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


os.environ.setdefault('ROS_LOG_DIR', '/tmp/ros_logs')
Path(os.environ['ROS_LOG_DIR']).mkdir(parents=True, exist_ok=True)


def _load_launch_module():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'go2_nav_walk_push.launch.py'
    )
    spec = importlib.util.spec_from_file_location(
        'go2_nav_walk_push_launch',
        launch_path,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_launch_description():
    return _load_launch_module().generate_launch_description()


def _all_inspectable_entities():
    module = _load_launch_module()
    launch_description = module.generate_launch_description()
    return [
        *launch_description.entities,
        *module._runtime_actions(),
    ]


def _evaluate_node_parameters(executable, overrides=None):
    entities = _all_inspectable_entities()
    context = LaunchContext()

    for name, value in (overrides or {}).items():
        context.launch_configurations[name] = value

    for entity in entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)

    for entity in entities:
        if isinstance(entity, Node) and entity._Node__node_executable == executable:
            return evaluate_parameters(context, entity._Node__parameters)[0]

    raise AssertionError(f'Node executable not found: {executable}')


def test_default_goal_evaluates_to_float_array():
    params = _evaluate_node_parameters('rl_nav_commands.py')

    assert params['default_goal'] == [0.0, 0.0, 0.0, 0.0]
    assert isinstance(params['default_goal'], list)
    assert all(isinstance(value, float) for value in params['default_goal'])
    assert isinstance(params['goal_timeout_sec'], float)


def test_integer_like_overrides_are_coerced_to_float_types():
    nav_params = _evaluate_node_parameters(
        'rl_nav_commands.py',
        overrides={
            'nav_goal_0': '2',
            'nav_goal_1': '0',
            'nav_goal_2': '0',
            'nav_goal_3': '1',
        },
    )
    mux_params = _evaluate_node_parameters(
        'cmd_vel_mux.py',
        overrides={'cmd_vel_mux_frequency': '50'},
    )

    assert nav_params['default_goal'] == [2.0, 0.0, 0.0, 1.0]
    assert all(isinstance(value, float) for value in nav_params['default_goal'])
    assert mux_params['publish_frequency'] == 50.0
    assert isinstance(mux_params['publish_frequency'], float)
