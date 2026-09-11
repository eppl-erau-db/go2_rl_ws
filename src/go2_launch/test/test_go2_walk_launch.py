"""Structure and parameter tests for go2_walk.launch.py."""

import importlib.util
import os
from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


os.environ.setdefault('ROS_LOG_DIR', '/tmp/ros_logs')
Path(os.environ['ROS_LOG_DIR']).mkdir(parents=True, exist_ok=True)


def _load_launch_module():
    launch_path = Path(__file__).resolve().parents[1] / 'launch' / 'go2_walk.launch.py'
    spec = importlib.util.spec_from_file_location('go2_walk_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context_with_defaults(overrides=None):
    module = _load_launch_module()
    launch_description = module.generate_launch_description()
    context = LaunchContext()
    for name, value in (overrides or {}).items():
        context.launch_configurations[name] = value
    for entity in launch_description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return module, context


def _process_cmd(process, context):
    return [perform_substitutions(context, part) for part in process.process_description.cmd]


def _evaluate_node_parameters(executable, overrides=None):
    module, context = _context_with_defaults(overrides)
    for entity in [*module._runtime_actions(), module._make_stand_ready_gate_node()]:
        if isinstance(entity, Node) and entity._Node__node_executable == executable:
            return evaluate_parameters(context, entity._Node__parameters)[0]
    raise AssertionError(f'Node executable not found: {executable}')


def test_runtime_nodes_present():
    module = _load_launch_module()
    executables = {
        entity._Node__node_executable
        for entity in module._runtime_actions()
        if isinstance(entity, Node)
    }
    assert executables == {
        'controller_commands.py',
        'rl_actions.py',
        'base_height_node',
        'go2_controller_node',
    }


def test_controller_stands_on_start_by_default():
    params = _evaluate_node_parameters('go2_controller_node')
    assert params['stand_on_start'] is True
    assert params['enable_joint_limit_monitor'] is True


def test_auto_stand_can_be_disabled():
    params = _evaluate_node_parameters('go2_controller_node', {'auto_stand': 'false'})
    assert params['stand_on_start'] is False


def test_default_policy_is_flat_48_dim():
    params = _evaluate_node_parameters('rl_actions.py')
    assert params['policy_name'] == 'go2_flat_v0'


def test_stand_ready_gate_parameters_are_typed():
    params = _evaluate_node_parameters('go2_stand_ready_gate.py')
    assert params['stand_ready_topic'] == 'stand_ready'
    assert params['required_consecutive_samples'] == 10
    assert isinstance(params['required_consecutive_samples'], int)
    assert isinstance(params['settle_sec'], float)
    assert isinstance(params['timeout_sec'], float)


def test_leg_odometry_process_sources_workspace_and_runs_headless():
    module, context = _context_with_defaults()
    process = module._make_leg_odometry_process('/tmp/some_ws', headless=True)
    assert isinstance(process, ExecuteProcess)
    cmd = _process_cmd(process, context)
    assert cmd[0] == 'bash' and cmd[1] == '-c'
    assert 'leg_odometry_ros leg_odom.launch.py' in cmd[2]
    assert cmd[3] == '/tmp/some_ws/install/setup.bash'
    assert cmd[4] == 'true'


def test_gate_failure_does_not_start_leg_odometry():
    module, context = _context_with_defaults()
    actions = module._on_stand_ready_exit(SimpleNamespace(returncode=1), context)
    assert actions and all(isinstance(a, LogInfo) for a in actions)


def test_gate_success_starts_leg_odometry_from_expanded_workspace():
    module, context = _context_with_defaults({'leg_odom_workspace': '~/odom_ws'})
    actions = module._on_stand_ready_exit(SimpleNamespace(returncode=0), context)
    processes = [a for a in actions if isinstance(a, ExecuteProcess)]
    assert len(processes) == 1
    cmd = _process_cmd(processes[0], context)
    assert cmd[3] == os.path.expanduser('~/odom_ws/install/setup.bash')
    assert cmd[4] == 'true'


def test_leg_odometry_can_be_disabled():
    module, context = _context_with_defaults({'enable_leg_odometry': 'false'})
    actions = module._on_stand_ready_exit(SimpleNamespace(returncode=0), context)
    assert not any(isinstance(a, ExecuteProcess) for a in actions)
