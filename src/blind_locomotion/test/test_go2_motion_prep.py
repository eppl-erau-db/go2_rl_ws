# Copyright 2026 go2_rl_ws contributors
# SPDX-License-Identifier: MIT
"""Tests for lie-down detection used before shutting off sport mode."""

from types import SimpleNamespace

from blind_locomotion.go2_motion_prep import (
    DEFAULT_LIE_DOWN_BODY_HEIGHT_MAX,
    LIE_DOWN_MODE,
    SIT_POS,
    lie_down_reason,
    max_sit_joint_error,
)


STAND_POS = (
    -0.1, 0.8, -1.5,
    0.1, 0.8, -1.5,
    -0.1, 1.0, -1.5,
    0.1, 1.0, -1.5,
)


def _state(*, mode=0, body_height=0.32):
    return SimpleNamespace(mode=mode, body_height=body_height)


def test_standing_robot_is_not_lying_down():
    assert lie_down_reason(_state(), list(STAND_POS)) is None


def test_lie_down_mode_is_accepted():
    assert lie_down_reason(_state(mode=LIE_DOWN_MODE), list(STAND_POS)) == 'mode'


def test_low_body_height_is_accepted_when_mode_never_reports_lie_down():
    # Observed on hardware: mode stays 0 while lying, body_height ~0.075.
    assert lie_down_reason(_state(mode=0, body_height=0.075), None) == 'body_height'


def test_zero_body_height_is_treated_as_invalid():
    assert lie_down_reason(_state(mode=0, body_height=0.0), None) is None


def test_body_height_threshold_is_inclusive():
    state = _state(body_height=DEFAULT_LIE_DOWN_BODY_HEIGHT_MAX)
    assert lie_down_reason(state, None) == 'body_height'
    state = _state(body_height=DEFAULT_LIE_DOWN_BODY_HEIGHT_MAX + 0.01)
    assert lie_down_reason(state, None) is None


def test_joints_near_sit_pos_are_accepted_without_sport_state():
    joints = [q + 0.05 for q in SIT_POS]
    assert lie_down_reason(None, joints) == 'joints'


def test_joints_outside_tolerance_are_rejected():
    joints = list(SIT_POS)
    joints[1] += 0.31
    assert lie_down_reason(None, joints) is None
    assert lie_down_reason(None, joints, sit_joint_tolerance_rad=0.35) == 'joints'


def test_max_sit_joint_error_handles_missing_joints():
    assert max_sit_joint_error(None) == float('inf')
    assert max_sit_joint_error([0.0] * 5) == float('inf')
    assert max_sit_joint_error(list(SIT_POS)) == 0.0
