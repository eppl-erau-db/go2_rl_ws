# Copyright 2026 go2_rl_ws contributors
# SPDX-License-Identifier: MIT
"""Focused tests for the ai_sport cmd_vel bridge."""

import json
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from blind_locomotion.ai_sport_cmd_vel import (
    AI_SPORT_SERVICE_NAME,
    AiSportCmdVelNode,
    ROBOT_SPORT_API_ID_DAMP,
    ROBOT_SPORT_API_ID_MOVE,
    ROBOT_SPORT_API_ID_STANDDOWN,
    ROBOT_SPORT_API_ID_STOPMOVE,
    ROBOT_STATE_API_ID_SERVICE_SWITCH,
    SPORT_MODE_SERVICE_NAME,
    make_move_request,
    make_robot_state_service_switch_request,
    make_sport_request,
)
from blind_locomotion.msg import Button


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


def _make_button(**kwargs):
    msg = Button()
    for field in ('up', 'down', 'start', 'select', 'a', 'b', 'emergency_sit', 'f1'):
        if not hasattr(msg, field):
            continue
        setattr(msg, field, bool(kwargs.get(field, False)))
    return msg


def _make_node():
    node = object.__new__(AiSportCmdVelNode)
    node._logger = MagicMock()
    node.get_logger = MagicMock(return_value=node._logger)
    node._clock = _FakeClock(10.0)
    node.get_clock = MagicMock(return_value=node._clock)

    node.sport_requests = []
    node.robot_state_requests = []
    node.sport_request_publisher = SimpleNamespace(
        publish=lambda msg: node.sport_requests.append(msg)
    )
    node.robot_state_request_publisher = SimpleNamespace(
        publish=lambda msg: node.robot_state_requests.append(msg)
    )

    node.activate_ai_sport = False
    node.disable_sport_mode_before_ai_sport = True
    node.require_ai_sport_confirmation = True
    node.ai_sport_confirmed = True
    node.require_start_button = True
    node.exit_on_select = False
    node.cmd_timeout_sec = 0.5
    node.cmd_deadband = 0.02
    node.max_linear_x = 0.5
    node.max_linear_y = 0.3
    node.max_angular_z = 0.8
    node.ai_sport_request_period_sec = 2.0
    node.stop_request_period_sec = 0.0

    now = node.get_clock().now()
    node.latest_cmd_vel = _make_twist()
    node.cmd_vel_received = False
    node.last_cmd_vel_time = now
    node.last_stop_request_time = _FakeTime(0.0)
    node.last_ai_sport_request_time = _FakeTime(0.0)
    node.armed = False
    node.motion_active = False
    node.low_level_handoff_requested = False

    node.last_start_button = False
    node.last_select_button = False
    node.last_a_button = False
    node.last_b_button = False
    node.last_down_button = False
    node.last_emergency_sit_button = False
    return node


def test_move_request_uses_sport_move_api_and_xyz_json():
    req = make_move_request(0.2, -0.1, 0.3)

    assert req.header.identity.api_id == ROBOT_SPORT_API_ID_MOVE
    assert json.loads(req.parameter) == pytest.approx(
        {'x': 0.2, 'y': -0.1, 'z': 0.3}
    )


def test_basic_sport_requests_use_expected_api_ids():
    assert make_sport_request(ROBOT_SPORT_API_ID_STOPMOVE).header.identity.api_id == 1003
    assert make_sport_request(ROBOT_SPORT_API_ID_STANDDOWN).header.identity.api_id == 1005
    assert make_sport_request(ROBOT_SPORT_API_ID_DAMP).header.identity.api_id == 1001


def test_ai_sport_switch_request_uses_robot_state_service_switch_shape():
    req = make_robot_state_service_switch_request(AI_SPORT_SERVICE_NAME, 1)

    assert req.header.identity.api_id == ROBOT_STATE_API_ID_SERVICE_SWITCH
    assert json.loads(req.parameter) == {'name': 'ai_sport', 'switch': 1}


def test_ai_sport_activation_switches_sport_mode_off_then_ai_sport_on():
    node = _make_node()
    node.activate_ai_sport = True
    node.ai_sport_confirmed = False

    AiSportCmdVelNode.publish_ai_sport_request(node, force=True)

    assert [json.loads(req.parameter) for req in node.robot_state_requests] == [
        {'name': SPORT_MODE_SERVICE_NAME, 'switch': 0},
        {'name': AI_SPORT_SERVICE_NAME, 'switch': 1},
    ]


def test_forced_ai_sport_activation_publishes_even_when_already_confirmed():
    node = _make_node()
    node.activate_ai_sport = True
    node.ai_sport_confirmed = True

    AiSportCmdVelNode.publish_ai_sport_request(node, force=True)

    assert [json.loads(req.parameter) for req in node.robot_state_requests] == [
        {'name': SPORT_MODE_SERVICE_NAME, 'switch': 0},
        {'name': AI_SPORT_SERVICE_NAME, 'switch': 1},
    ]


def test_ai_sport_confirmation_response_allows_motion_forwarding():
    node = _make_node()
    node.activate_ai_sport = True
    node.ai_sport_confirmed = False
    node.armed = True
    node.cmd_vel_received = True
    node.latest_cmd_vel = _make_twist(0.2, 0.0, 0.0)
    node.last_cmd_vel_time = node.get_clock().now()

    AiSportCmdVelNode.tick(node)
    assert node.sport_requests == []

    response = SimpleNamespace(
        header=SimpleNamespace(
            identity=SimpleNamespace(api_id=ROBOT_STATE_API_ID_SERVICE_SWITCH)
        ),
        data=json.dumps({'name': AI_SPORT_SERVICE_NAME, 'status': 1}),
    )
    AiSportCmdVelNode.robot_state_response_callback(node, response)
    AiSportCmdVelNode.tick(node)

    assert node.ai_sport_confirmed is True
    assert node.sport_requests[-1].header.identity.api_id == ROBOT_SPORT_API_ID_MOVE


def test_start_arms_bridge_and_tick_publishes_clamped_move():
    node = _make_node()

    AiSportCmdVelNode.buttons_callback(node, _make_button(start=True))
    AiSportCmdVelNode.cmd_vel_callback(node, _make_twist(1.0, -1.0, 2.0))
    AiSportCmdVelNode.tick(node)

    assert node.armed is True
    req = node.sport_requests[-1]
    assert req.header.identity.api_id == ROBOT_SPORT_API_ID_MOVE
    assert json.loads(req.parameter) == {
        'x': 0.5,
        'y': -0.3,
        'z': 0.8,
    }


def test_start_stops_and_disarms_when_already_armed():
    node = _make_node()
    node.armed = True
    node.motion_active = True

    AiSportCmdVelNode.buttons_callback(node, _make_button(start=True))

    assert node.armed is False
    assert node.motion_active is False
    assert node.sport_requests[-1].header.identity.api_id == ROBOT_SPORT_API_ID_STOPMOVE


def test_select_is_ignored_by_default():
    node = _make_node()
    node.armed = True
    node.motion_active = True

    AiSportCmdVelNode.buttons_callback(node, _make_button(select=True))

    assert node.low_level_handoff_requested is False
    assert node.sport_requests == []


def test_select_requests_low_level_handoff_when_enabled():
    node = _make_node()
    node.exit_on_select = True
    node.armed = True
    node.motion_active = True

    with patch('blind_locomotion.ai_sport_cmd_vel.threading.Thread') as thread_cls:
        with pytest.raises(SystemExit) as exc:
            AiSportCmdVelNode.buttons_callback(node, _make_button(select=True))

    assert node.low_level_handoff_requested is True
    assert node.armed is False
    assert node.motion_active is False
    assert node.sport_requests[-1].header.identity.api_id == ROBOT_SPORT_API_ID_STOPMOVE
    assert exc.value.code == 0
    thread_cls.assert_called_once()
    thread_cls.return_value.start.assert_called_once_with()


def test_a_button_stops_and_disarms():
    node = _make_node()
    node.armed = True
    node.motion_active = True

    AiSportCmdVelNode.buttons_callback(node, _make_button(a=True))

    assert node.armed is False
    assert node.motion_active is False
    assert node.sport_requests[-1].header.identity.api_id == ROBOT_SPORT_API_ID_STOPMOVE


def test_down_and_emergency_sit_stand_down_and_disarm():
    for field in ('down', 'emergency_sit'):
        node = _make_node()
        node.armed = True
        node.motion_active = True

        AiSportCmdVelNode.buttons_callback(node, _make_button(**{field: True}))

        assert node.armed is False
        assert node.motion_active is False
        assert node.sport_requests[-1].header.identity.api_id == ROBOT_SPORT_API_ID_STANDDOWN


def test_b_button_damps_and_disarms():
    node = _make_node()
    node.armed = True
    node.motion_active = True

    AiSportCmdVelNode.buttons_callback(node, _make_button(b=True))

    assert node.armed is False
    assert node.motion_active is False
    assert node.sport_requests[-1].header.identity.api_id == ROBOT_SPORT_API_ID_DAMP


def test_stale_cmd_vel_results_in_stop_move():
    node = _make_node()
    node.armed = True
    node.motion_active = True
    node.cmd_vel_received = True
    node.latest_cmd_vel = _make_twist(0.2, 0.0, 0.0)
    node.last_cmd_vel_time = _FakeTime(0.0)

    AiSportCmdVelNode.tick(node)

    assert node.sport_requests[-1].header.identity.api_id == ROBOT_SPORT_API_ID_STOPMOVE
    assert node.motion_active is False


def test_deadbanded_cmd_vel_results_in_stop_move():
    node = _make_node()
    node.armed = True
    node.motion_active = True
    node.cmd_vel_received = True
    node.latest_cmd_vel = _make_twist(0.01, 0.0, 0.0)
    node.last_cmd_vel_time = node.get_clock().now()

    AiSportCmdVelNode.tick(node)

    assert node.sport_requests[-1].header.identity.api_id == ROBOT_SPORT_API_ID_STOPMOVE
    assert node.motion_active is False
