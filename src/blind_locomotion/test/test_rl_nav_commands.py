"""Focused tests for navigation policy observation and output handling."""
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np

from blind_locomotion.rl_nav_commands import RLNavCommandsNode


class _FakeElapsed:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1e9)


class _FakeTime:
    def __init__(self, seconds):
        self.seconds = float(seconds)

    def __sub__(self, other):
        return _FakeElapsed(self.seconds - other.seconds)


class _FakeClock:
    def __init__(self, seconds=0.0):
        self.current = _FakeTime(seconds)

    def now(self):
        return self.current


class _FakeSession:
    def __init__(self, output):
        self.output = np.asarray(output, dtype=np.float32)
        self.last_input = None

    def get_inputs(self):
        return [SimpleNamespace(name='obs')]

    def run(self, _, inputs):
        self.last_input = inputs['obs']
        return [self.output.reshape(1, -1)]


def _make_node():
    node = object.__new__(RLNavCommandsNode)
    node._logger = MagicMock()
    node.get_logger = MagicMock(return_value=node._logger)
    node._clock = _FakeClock(1.0)
    node.get_clock = MagicMock(return_value=node._clock)
    node.publisher = MagicMock()
    node.ort_session = _FakeSession([1.5, -1.25, 0.4])
    node.nav_output_scale = 1.0
    node.default_goal = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    node.goal = node.default_goal.copy()
    node.goal_topic = 'nav_goal'
    node.goal_timeout_sec = 0.5
    node.base_lin_vel = np.array([0.2, -0.1, 0.05], dtype=np.float32)
    node.proj_gravity = np.array([0.0, 0.0, -1.0], dtype=np.float32)
    node.cmd_vel_limits = np.array([1.0, 1.0, 1.0], dtype=np.float32)
    node.lowstate_received = True
    node.odom_received = True
    node.goal_received = False
    node.require_start_button = True
    node.nav_enabled = True
    node.last_start_button = False
    node.lowstate_timeout_sec = 0.5
    node.odom_timeout_sec = 0.5
    node.last_lowstate_time = _FakeTime(0.8)
    node.last_odom_time = _FakeTime(0.8)
    node.last_goal_time = _FakeTime(0.0)
    node.last_enable_time = _FakeTime(0.8)
    return node


def test_goal_callback_updates_goal_and_timestamp():
    node = _make_node()
    node._clock.current = _FakeTime(2.5)

    RLNavCommandsNode.goal_callback(
        node,
        SimpleNamespace(data=[2.0, 0.0, 0.0, 0.75]),
    )

    np.testing.assert_allclose(node.goal, np.array([2.0, 0.0, 0.0, 0.75], dtype=np.float32))
    assert node.goal_received is True
    assert node.last_goal_time.seconds == 2.5


def test_start_button_enables_navigation_and_resets_launch_goal_window():
    node = _make_node()
    node.nav_enabled = False
    node._clock.current = _FakeTime(2.5)

    RLNavCommandsNode.buttons_callback(node, SimpleNamespace(start=True))

    assert node.nav_enabled is True
    assert node.last_enable_time.seconds == 2.5


def test_goal_vector_uses_default_when_goal_is_stale():
    node = _make_node()
    node.default_goal = np.array([2.0, 0.0, 0.0, 0.75], dtype=np.float32)
    node.goal = np.array([3.0, 1.0, -1.0, 0.5], dtype=np.float32)
    node.goal_received = True
    node.last_goal_time = _FakeTime(0.0)
    node.last_enable_time = _FakeTime(0.8)

    goal = RLNavCommandsNode._goal_vector(node, _FakeTime(1.0))

    np.testing.assert_allclose(goal, node.default_goal)
    assert any('Nav goal stale' in str(call.args[0]) for call in node._logger.warn.call_args_list)


def test_publish_cmd_vel_is_zero_before_start_button():
    node = _make_node()
    node.nav_enabled = False

    RLNavCommandsNode.publish_cmd_vel(node)

    published = node.publisher.publish.call_args[0][0]
    assert published.linear.x == 0.0
    assert published.linear.y == 0.0
    assert published.angular.z == 0.0
    assert node.ort_session.last_input is None


def test_publish_cmd_vel_accepts_fresh_goal_without_start_when_configured():
    node = _make_node()
    node.require_start_button = False
    node.nav_enabled = False
    node.goal = np.array([2.0, 0.0, 0.0, 0.75], dtype=np.float32)
    node.goal_received = True
    node.last_goal_time = _FakeTime(0.8)

    RLNavCommandsNode.publish_cmd_vel(node)

    expected_obs = np.array(
        [[0.2, -0.1, 0.05, 0.0, 0.0, -1.0, 2.0, 0.0, 0.0, 0.75]],
        dtype=np.float32,
    )
    np.testing.assert_allclose(node.ort_session.last_input, expected_obs)
    assert node.nav_enabled is True


def test_publish_cmd_vel_builds_obs_and_clamps_output():
    node = _make_node()
    node.default_goal = np.array([2.0, 0.0, 0.0, 0.75], dtype=np.float32)

    RLNavCommandsNode.publish_cmd_vel(node)

    expected_obs = np.array(
        [[0.2, -0.1, 0.05, 0.0, 0.0, -1.0, 2.0, 0.0, 0.0, 0.75]],
        dtype=np.float32,
    )
    np.testing.assert_allclose(node.ort_session.last_input, expected_obs)

    published = node.publisher.publish.call_args[0][0]
    assert np.isclose(published.linear.x, 1.0)
    assert np.isclose(published.linear.y, -1.0)
    assert np.isclose(published.angular.z, 0.4)


def test_publish_cmd_vel_publishes_zero_when_odom_is_stale():
    node = _make_node()
    node.last_odom_time = _FakeTime(0.0)

    RLNavCommandsNode.publish_cmd_vel(node)

    published = node.publisher.publish.call_args[0][0]
    assert published.linear.x == 0.0
    assert published.linear.y == 0.0
    assert published.angular.z == 0.0


def test_launch_goal_expires_after_timeout_from_start():
    node = _make_node()
    node.default_goal = np.array([2.0, 0.0, 0.0, 0.75], dtype=np.float32)
    node.goal_received = False
    node.last_enable_time = _FakeTime(0.0)

    goal = RLNavCommandsNode._goal_vector(node, _FakeTime(1.0))

    assert goal is None
