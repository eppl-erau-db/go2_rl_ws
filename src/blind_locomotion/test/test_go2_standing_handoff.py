# Copyright 2026 go2_rl_ws contributors
# SPDX-License-Identifier: MIT
"""Focused tests for the standing ai_sport to low-level handoff guard."""

from types import SimpleNamespace

from blind_locomotion.go2_standing_handoff import (
    LIE_DOWN_MODE,
    StandingStabilityTracker,
    is_stable_upright_state,
)


def _state(*, mode=1, body_height=0.32, velocity=None, yaw_speed=0.0):
    return SimpleNamespace(
        mode=mode,
        body_height=body_height,
        velocity=[0.0, 0.0, 0.0] if velocity is None else velocity,
        yaw_speed=yaw_speed,
    )


def test_stable_upright_state_accepts_fresh_still_standing_state():
    assert is_stable_upright_state(_state(), state_age_sec=0.05)


def test_stable_upright_state_rejects_low_body_height():
    assert not is_stable_upright_state(
        _state(body_height=0.10),
        state_age_sec=0.05,
    )


def test_stable_upright_state_rejects_lie_down_mode():
    assert not is_stable_upright_state(
        _state(mode=LIE_DOWN_MODE),
        state_age_sec=0.05,
    )


def test_stable_upright_state_rejects_linear_motion():
    assert not is_stable_upright_state(
        _state(velocity=[0.06, 0.0, 0.0]),
        state_age_sec=0.05,
    )


def test_stable_upright_state_rejects_yaw_motion():
    assert not is_stable_upright_state(
        _state(yaw_speed=0.11),
        state_age_sec=0.05,
    )


def test_stable_upright_state_rejects_stale_state():
    assert not is_stable_upright_state(_state(), state_age_sec=0.5)


def test_stability_tracker_requires_consecutive_stable_samples():
    tracker = StandingStabilityTracker(required_samples=3)
    state = _state()

    assert not tracker.update(state, state_age_sec=0.0)
    assert not tracker.update(state, state_age_sec=0.0)
    assert tracker.update(state, state_age_sec=0.0)


def test_stability_tracker_resets_on_unstable_sample():
    tracker = StandingStabilityTracker(required_samples=2)

    assert not tracker.update(_state(), state_age_sec=0.0)
    assert not tracker.update(_state(velocity=[0.2, 0.0, 0.0]), state_age_sec=0.0)
    assert not tracker.update(_state(), state_age_sec=0.0)
    assert tracker.update(_state(), state_age_sec=0.0)
