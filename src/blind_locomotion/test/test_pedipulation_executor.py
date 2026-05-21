from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from blind_locomotion.pedipulation_executor import (
    PedipulationExecutor,
)


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


def _make_node():
    node = object.__new__(PedipulationExecutor)
    node._logger = MagicMock()
    node.get_logger = MagicMock(return_value=node._logger)
    node._clock = _FakeClock(0.0)
    node.get_clock = MagicMock(return_value=node._clock)
    node.trajectory_duration_sec = 4.0
    node.home_position = np.array([0.20, -0.15, 0.00], dtype=np.float32)
    node.push_position = np.array([0.50, -0.15, 0.075], dtype=np.float32)
    node.position_spline = PedipulationExecutor._build_position_spline(node)
    node.substate = PedipulationExecutor.HOME_HOLD
    node.execution_start_time = None
    node.pose_publisher = MagicMock()
    return node


def test_evaluate_trajectory_hits_configured_waypoints():
    node = _make_node()

    np.testing.assert_allclose(
        PedipulationExecutor._evaluate_trajectory(node, 0.0),
        node.home_position,
    )
    np.testing.assert_allclose(
        PedipulationExecutor._evaluate_trajectory(node, 0.33 * node.trajectory_duration_sec),
        node.push_position,
    )
    np.testing.assert_allclose(
        PedipulationExecutor._evaluate_trajectory(node, 4.0),
        node.home_position,
        atol=1e-6,
    )


def test_execute_service_starts_trajectory():
    node = _make_node()
    node._try_update_push_position_from_obstacle = MagicMock(return_value=True)
    response = SimpleNamespace(success=False, message='')

    PedipulationExecutor._handle_execute_push(node, None, response)

    assert response.success is True
    assert response.message == 'Started blind push trajectory'
    assert node.substate == PedipulationExecutor.EXECUTING
    assert node.execution_start_time.seconds == 0.0


def test_execute_service_rejects_when_already_running():
    node = _make_node()
    node.substate = PedipulationExecutor.EXECUTING
    node.execution_start_time = _FakeTime(0.0)
    response = SimpleNamespace(success=False, message='')

    PedipulationExecutor._handle_execute_push(node, None, response)

    assert response.success is False
    assert response.message == 'Push trajectory already executing'


def test_cancel_service_returns_to_home_hold():
    node = _make_node()
    node.substate = PedipulationExecutor.EXECUTING
    node.execution_start_time = _FakeTime(1.0)
    response = SimpleNamespace(success=False, message='')

    PedipulationExecutor._handle_cancel_push(node, None, response)

    assert response.success is True
    assert response.message == 'Cancelled active push trajectory'
    assert node.substate == PedipulationExecutor.HOME_HOLD
    assert node.execution_start_time is None


def test_current_target_position_finishes_and_returns_home():
    node = _make_node()
    node.substate = PedipulationExecutor.EXECUTING
    node.execution_start_time = _FakeTime(0.0)
    node._clock.current = _FakeTime(4.5)

    target = PedipulationExecutor._current_target_position(node)

    np.testing.assert_allclose(target, node.home_position)
    assert node.substate == PedipulationExecutor.HOME_HOLD
    assert node.execution_start_time is None


def test_timer_publishes_identity_orientation_home_pose():
    node = _make_node()

    PedipulationExecutor._timer_callback(node)

    pose_msg = node.pose_publisher.publish.call_args.args[0]
    assert pose_msg.position.x == pytest.approx(0.20)
    assert pose_msg.position.y == pytest.approx(-0.15)
    assert pose_msg.position.z == pytest.approx(0.0)
    assert pose_msg.orientation.w == pytest.approx(1.0)
    assert pose_msg.orientation.x == pytest.approx(0.0)
    assert pose_msg.orientation.y == pytest.approx(0.0)
    assert pose_msg.orientation.z == pytest.approx(0.0)
