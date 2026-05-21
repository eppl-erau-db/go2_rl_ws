"""Focused tests for nav-vs-wireless cmd_vel arbitration."""
from types import SimpleNamespace
from unittest.mock import MagicMock

from blind_locomotion.cmd_vel_mux import CmdVelMuxNode


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


def _make_twist(x=0.0, y=0.0, yaw=0.0):
    return SimpleNamespace(
        linear=SimpleNamespace(x=float(x), y=float(y), z=0.0),
        angular=SimpleNamespace(x=0.0, y=0.0, z=float(yaw)),
    )


def _make_node():
    node = object.__new__(CmdVelMuxNode)
    node._logger = MagicMock()
    node.get_logger = MagicMock(return_value=node._logger)
    node._clock = _FakeClock(1.0)
    node.get_clock = MagicMock(return_value=node._clock)
    node.publisher = MagicMock()
    node.latest_nav_cmd = _make_twist(0.4, 0.1, -0.2)
    node.latest_wireless_cmd = _make_twist()
    node.nav_received = True
    node.wireless_received = True
    node.last_nav_time = _FakeTime(0.9)
    node.last_wireless_time = _FakeTime(0.9)
    node.nav_timeout_sec = 0.5
    node.wireless_timeout_sec = 0.5
    node.manual_override_deadband = 0.1
    node.last_source = 'idle'
    return node


def test_selected_command_prefers_wireless_when_override_is_active():
    node = _make_node()
    node.latest_wireless_cmd = _make_twist(0.5, 0.0, 0.0)

    twist, source = CmdVelMuxNode._selected_command(node, _FakeTime(1.0))

    assert source == 'wireless'
    assert twist.linear.x == 0.5


def test_selected_command_falls_back_to_navigation_when_wireless_is_idle():
    node = _make_node()

    twist, source = CmdVelMuxNode._selected_command(node, _FakeTime(1.0))

    assert source == 'nav'
    assert twist.linear.x == 0.4
    assert twist.angular.z == -0.2


def test_selected_command_returns_idle_when_inputs_are_stale():
    node = _make_node()
    node.last_nav_time = _FakeTime(0.0)
    node.last_wireless_time = _FakeTime(0.0)

    twist, source = CmdVelMuxNode._selected_command(node, _FakeTime(1.0))

    assert source == 'idle'
    assert twist.linear.x == 0.0
    assert twist.linear.y == 0.0
    assert twist.angular.z == 0.0
