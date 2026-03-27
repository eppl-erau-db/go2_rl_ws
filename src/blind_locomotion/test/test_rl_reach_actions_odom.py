"""Focused tests for odom handling and observation gating in RLReachActionsNode."""
from types import SimpleNamespace
from unittest.mock import MagicMock
import sys

import numpy as np

# test/conftest.py may provide a minimal unitree_go.msg stub without LowState.
try:
    import unitree_go.msg as _unitree_msg
except (ImportError, ModuleNotFoundError):
    from types import ModuleType

    _unitree_pkg = ModuleType("unitree_go")
    _unitree_msg = ModuleType("unitree_go.msg")
    _unitree_pkg.msg = _unitree_msg
    sys.modules.setdefault("unitree_go", _unitree_pkg)
    sys.modules.setdefault("unitree_go.msg", _unitree_msg)

if not hasattr(_unitree_msg, "LowState"):
    _unitree_msg.LowState = type("LowState", (), {})

from blind_locomotion.rl_reach_actions import RLReachActionsNode


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


def _make_odom_msg(vx, vy, vz, frame_id="odom", child_frame_id="base"):
    return SimpleNamespace(
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=float(vx), y=float(vy), z=float(vz))
            )
        ),
        header=SimpleNamespace(frame_id=frame_id),
        child_frame_id=child_frame_id,
    )


def _make_node():
    node = object.__new__(RLReachActionsNode)

    node._logger = MagicMock()
    node.get_logger = MagicMock(return_value=node._logger)
    node._clock = _FakeClock(0.0)
    node.get_clock = MagicMock(return_value=node._clock)

    node.base_lin_vel = np.zeros(3, dtype=np.float32)
    node.odom_topic = "/odom"
    node.odom_timeout_sec = 0.5
    node.odom_received = False
    node.last_odom_time = _FakeTime(0.0)
    node.last_odom_frame_id = None
    node.last_odom_child_frame_id = None

    node.lowstate_received = False
    node.lowstate_timeout_sec = 0.5
    node.last_lowstate_time = _FakeTime(0.0)

    return node


def _warn_messages(node):
    return [str(call.args[0]) for call in node._logger.warn.call_args_list]


def _info_messages(node):
    return [str(call.args[0]) for call in node._logger.info.call_args_list]


def test_odom_callback_updates_base_lin_vel_and_sets_received():
    node = _make_node()
    node._clock.current = _FakeTime(1.25)

    RLReachActionsNode.odom_callback(node, _make_odom_msg(0.2, -0.1, 0.05))

    np.testing.assert_allclose(node.base_lin_vel, np.array([0.2, -0.1, 0.05], dtype=np.float32))
    assert node.odom_received is True
    assert node.last_odom_time.seconds == 1.25
    assert node.last_odom_frame_id == "odom"
    assert node.last_odom_child_frame_id == "base"

    joined_logs = "\n".join(_info_messages(node))
    assert "Received first odom on /odom" in joined_logs
    assert 'frame_id="odom"' in joined_logs


def test_obs_ready_false_without_lowstate():
    node = _make_node()

    ready = RLReachActionsNode._obs_ready(node, _FakeTime(1.0))

    assert ready is False
    assert "Waiting for /lowstate before policy inference" in _warn_messages(node)[0]


def test_obs_ready_false_without_odom_when_lowstate_fresh():
    node = _make_node()
    node.lowstate_received = True
    node.last_lowstate_time = _FakeTime(0.8)

    ready = RLReachActionsNode._obs_ready(node, _FakeTime(1.0))

    assert ready is False
    assert any("Waiting for /odom before policy inference" in msg for msg in _warn_messages(node))


def test_obs_ready_false_when_odom_is_stale():
    node = _make_node()
    node.lowstate_received = True
    node.last_lowstate_time = _FakeTime(0.8)
    node.odom_received = True
    node.last_odom_time = _FakeTime(0.0)

    ready = RLReachActionsNode._obs_ready(node, _FakeTime(1.0))

    assert ready is False
    assert any("Odom stale" in msg for msg in _warn_messages(node))


def test_obs_ready_true_with_fresh_lowstate_and_odom():
    node = _make_node()
    node.lowstate_received = True
    node.last_lowstate_time = _FakeTime(0.9)
    node.odom_received = True
    node.last_odom_time = _FakeTime(0.9)

    ready = RLReachActionsNode._obs_ready(node, _FakeTime(1.0))

    assert ready is True
    node._logger.warn.assert_not_called()
