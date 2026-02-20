"""Tests for wireless controller velocity outputs used by the locomotion policy."""
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from blind_locomotion.controller_commands import (
    clamp,
    stick_to_range,
    STICK_FIELDS,
    LIN_X_RANGE,
    LIN_Y_RANGE,
    ANG_Z_RANGE,
    AXIS_LIN_X,
    AXIS_LIN_Y,
    AXIS_ANG_Z,
    INVERT_LIN_X,
    INVERT_LIN_Y,
    INVERT_ANG_Z,
)


# ---------------------------------------------------------------------------
# Pure function tests
# ---------------------------------------------------------------------------

class TestClamp:
    def test_within_range(self):
        assert clamp(0.5, 0.0, 1.0) == 0.5

    def test_below_low(self):
        assert clamp(-2.0, -1.0, 1.0) == -1.0

    def test_above_high(self):
        assert clamp(3.0, -1.0, 1.0) == 1.0

    def test_at_boundaries(self):
        assert clamp(-1.0, -1.0, 1.0) == -1.0
        assert clamp(1.0, -1.0, 1.0) == 1.0

    def test_zero(self):
        assert clamp(0.0, -1.0, 1.0) == 0.0


class TestStickToRange:
    """stick_to_range maps a [-1, 1] stick value into an arbitrary [lo, hi] range."""

    def test_center_stick_gives_midpoint(self):
        assert stick_to_range(0.0, -1.0, 1.0) == pytest.approx(0.0)
        assert stick_to_range(0.0, 0.0, 2.0) == pytest.approx(1.0)

    def test_full_positive_gives_hi(self):
        assert stick_to_range(1.0, -1.0, 1.0) == pytest.approx(1.0)
        assert stick_to_range(1.0, 0.0, 2.0) == pytest.approx(2.0)

    def test_full_negative_gives_lo(self):
        assert stick_to_range(-1.0, -1.0, 1.0) == pytest.approx(-1.0)
        assert stick_to_range(-1.0, 0.0, 2.0) == pytest.approx(0.0)

    def test_half_deflection(self):
        assert stick_to_range(0.5, -1.0, 1.0) == pytest.approx(0.5)
        assert stick_to_range(-0.5, 0.0, 2.0) == pytest.approx(0.5)

    def test_asymmetric_range(self):
        # range [-0.5, 1.5] → center 0.5, half 1.0
        assert stick_to_range(0.0, -0.5, 1.5) == pytest.approx(0.5)
        assert stick_to_range(1.0, -0.5, 1.5) == pytest.approx(1.5)
        assert stick_to_range(-1.0, -0.5, 1.5) == pytest.approx(-0.5)

    def test_clamps_beyond_stick_range(self):
        # stick value beyond [-1, 1] should still be clamped to [lo, hi]
        result = stick_to_range(2.0, -1.0, 1.0)
        assert result == pytest.approx(1.0)
        result = stick_to_range(-2.0, -1.0, 1.0)
        assert result == pytest.approx(-1.0)


# ---------------------------------------------------------------------------
# Helpers to build mock WirelessController messages
# ---------------------------------------------------------------------------

def make_wireless_msg(lx=0.0, ly=0.0, rx=0.0, ry=0.0, keys=0):
    """Return a lightweight mock WirelessController message."""
    return SimpleNamespace(lx=lx, ly=ly, rx=rx, ry=ry, keys=keys)


# ---------------------------------------------------------------------------
# Node-level velocity output tests
# ---------------------------------------------------------------------------

def _make_node(overrides=None):
    """Instantiate WirelessControl with rclpy mocked out.

    Returns (node, published_twists, published_buttons) where the lists
    accumulate every message published on cmd_vel / buttons.

    *overrides* is a dict of node attribute names to values, applied after
    setting defaults from the module-level constants.
    """
    from blind_locomotion.controller_commands import WirelessControl

    published_twists = []
    published_buttons = []

    with patch('blind_locomotion.controller_commands.Node.__init__'):
        node = object.__new__(WirelessControl)

        # Minimal logger stub.
        node._logger = MagicMock()
        node.get_logger = MagicMock(return_value=node._logger)

        # Set defaults from module constants.
        node.lin_x_range = LIN_X_RANGE
        node.lin_y_range = LIN_Y_RANGE
        node.ang_z_range = ANG_Z_RANGE
        node.axis_lin_x = AXIS_LIN_X
        node.axis_lin_y = AXIS_LIN_Y
        node.axis_ang_z = AXIS_ANG_Z
        node.invert_lin_x = INVERT_LIN_X
        node.invert_lin_y = INVERT_LIN_Y
        node.invert_ang_z = INVERT_ANG_Z

        # Apply overrides.
        if overrides:
            for attr, value in overrides.items():
                setattr(node, attr, value)

        # Fake clock.
        fake_time = MagicMock()
        fake_time.__sub__ = MagicMock(
            return_value=MagicMock(nanoseconds=0)
        )
        node.get_clock = MagicMock(
            return_value=MagicMock(now=MagicMock(return_value=fake_time))
        )
        node.last_msg_time = fake_time
        node.last_axis_warn_time = fake_time
        node.in_timeout = False

        # Capture publishes.
        vel_pub = MagicMock()
        vel_pub.publish = lambda msg: published_twists.append(msg)
        node.vel_publisher = vel_pub

        btn_pub = MagicMock()
        btn_pub.publish = lambda msg: published_buttons.append(msg)
        node.buttons_publisher = btn_pub

    return node, published_twists, published_buttons


class TestDefaultAxisMapping:
    """Default config: left_y → lin_x, left_x → lin_y (inverted), right_x → ang_z (inverted)."""

    def test_neutral_sticks_give_zero_velocity(self):
        node, twists, _ = _make_node()
        node.wireless_controller_callback(make_wireless_msg())
        assert len(twists) == 1
        assert twists[0].linear.x == pytest.approx(0.0)
        assert twists[0].linear.y == pytest.approx(0.0)
        assert twists[0].angular.z == pytest.approx(0.0)

    def test_full_forward_left_stick(self):
        """ly = 1.0 → lin_x should be max (1.0) with no inversion."""
        node, twists, _ = _make_node()
        node.wireless_controller_callback(make_wireless_msg(ly=1.0))
        assert twists[0].linear.x == pytest.approx(1.0)
        assert twists[0].linear.y == pytest.approx(0.0)
        assert twists[0].angular.z == pytest.approx(0.0)

    def test_full_backward_left_stick(self):
        """ly = -1.0 → lin_x should be min (-1.0) with no inversion."""
        node, twists, _ = _make_node()
        node.wireless_controller_callback(make_wireless_msg(ly=-1.0))
        assert twists[0].linear.x == pytest.approx(-1.0)

    def test_full_left_strafe(self):
        """lx = 1.0 with invert_lin_y=True → lin_y = stick_to_range(-1.0, -1, 1) = -1.0."""
        node, twists, _ = _make_node()
        node.wireless_controller_callback(make_wireless_msg(lx=1.0))
        assert twists[0].linear.y == pytest.approx(-1.0)

    def test_full_right_strafe(self):
        """lx = -1.0 with invert_lin_y=True → lin_y = stick_to_range(1.0, -1, 1) = 1.0."""
        node, twists, _ = _make_node()
        node.wireless_controller_callback(make_wireless_msg(lx=-1.0))
        assert twists[0].linear.y == pytest.approx(1.0)

    def test_full_yaw_left(self):
        """rx = -1.0 with invert_ang_z=True → ang_z = stick_to_range(1.0, -1, 1) = 1.0."""
        node, twists, _ = _make_node()
        node.wireless_controller_callback(make_wireless_msg(rx=-1.0))
        assert twists[0].angular.z == pytest.approx(1.0)

    def test_full_yaw_right(self):
        """rx = 1.0 with invert_ang_z=True → ang_z = stick_to_range(-1.0, -1, 1) = -1.0."""
        node, twists, _ = _make_node()
        node.wireless_controller_callback(make_wireless_msg(rx=1.0))
        assert twists[0].angular.z == pytest.approx(-1.0)

    def test_diagonal_input(self):
        """All sticks at 0.5: combined forward + strafe + yaw."""
        node, twists, _ = _make_node()
        node.wireless_controller_callback(
            make_wireless_msg(lx=0.5, ly=0.5, rx=0.5)
        )
        assert twists[0].linear.x == pytest.approx(0.5)   # ly=0.5, no invert
        assert twists[0].linear.y == pytest.approx(-0.5)   # lx=0.5, inverted
        assert twists[0].angular.z == pytest.approx(-0.5)  # rx=0.5, inverted


class TestCustomVelocityRanges:
    """Verify stick_to_range scaling with non-default velocity ranges."""

    def test_forward_only_range(self):
        """lin_x in [0, 2] — neutral stick → 1.0, full forward → 2.0."""
        node, twists, _ = _make_node({'lin_x_range': (0.0, 2.0)})
        node.wireless_controller_callback(make_wireless_msg(ly=0.0))
        assert twists[0].linear.x == pytest.approx(1.0)

        twists.clear()
        node.wireless_controller_callback(make_wireless_msg(ly=1.0))
        assert twists[0].linear.x == pytest.approx(2.0)

        twists.clear()
        node.wireless_controller_callback(make_wireless_msg(ly=-1.0))
        assert twists[0].linear.x == pytest.approx(0.0)

    def test_narrow_yaw_range(self):
        """ang_z in [-0.5, 0.5]."""
        node, twists, _ = _make_node({'ang_z_range': (-0.5, 0.5)})
        # rx = -1.0, inverted → stick = 1.0 → 0.5
        node.wireless_controller_callback(make_wireless_msg(rx=-1.0))
        assert twists[0].angular.z == pytest.approx(0.5)

        twists.clear()
        # rx = 1.0, inverted → stick = -1.0 → -0.5
        node.wireless_controller_callback(make_wireless_msg(rx=1.0))
        assert twists[0].angular.z == pytest.approx(-0.5)


class TestInversionOverrides:
    """Verify inversion flags can be toggled."""

    def test_no_inversions(self):
        node, twists, _ = _make_node({
            'invert_lin_x': False,
            'invert_lin_y': False,
            'invert_ang_z': False,
        })
        node.wireless_controller_callback(
            make_wireless_msg(lx=1.0, ly=1.0, rx=1.0)
        )
        assert twists[0].linear.x == pytest.approx(1.0)   # ly=1.0
        assert twists[0].linear.y == pytest.approx(1.0)   # lx=1.0, no invert
        assert twists[0].angular.z == pytest.approx(1.0)   # rx=1.0, no invert

    def test_all_inversions(self):
        node, twists, _ = _make_node({
            'invert_lin_x': True,
            'invert_lin_y': True,
            'invert_ang_z': True,
        })
        node.wireless_controller_callback(
            make_wireless_msg(lx=1.0, ly=1.0, rx=1.0)
        )
        assert twists[0].linear.x == pytest.approx(-1.0)  # ly=1.0, inverted
        assert twists[0].linear.y == pytest.approx(-1.0)   # lx=1.0, inverted
        assert twists[0].angular.z == pytest.approx(-1.0)  # rx=1.0, inverted


class TestOutOfRangeStickValues:
    """Stick values beyond [-1, 1] should be clamped before mapping."""

    def test_oversaturated_forward(self):
        node, twists, _ = _make_node()
        node.wireless_controller_callback(make_wireless_msg(ly=1.5))
        assert twists[0].linear.x == pytest.approx(1.0)

    def test_oversaturated_negative(self):
        node, twists, _ = _make_node()
        node.wireless_controller_callback(make_wireless_msg(ly=-1.5))
        assert twists[0].linear.x == pytest.approx(-1.0)


class TestButtonMapping:
    """Verify wireless controller key codes map to the correct button fields."""

    @pytest.mark.parametrize('keys,field', [
        (4096, 'up'),
        (16384, 'down'),
        (4, 'start'),
        (8, 'select'),
        (256, 'a'),
        (512, 'b'),
    ])
    def test_single_button(self, keys, field):
        node, _, buttons = _make_node()
        node.wireless_controller_callback(make_wireless_msg(keys=keys))
        assert len(buttons) == 1
        assert getattr(buttons[0], field) is True
        # All other buttons should be False.
        for other in ('up', 'down', 'start', 'select', 'a', 'b', 'emergency_sit'):
            if other != field:
                assert getattr(buttons[0], other) is False

    def test_no_buttons_pressed(self):
        node, _, buttons = _make_node()
        node.wireless_controller_callback(make_wireless_msg(keys=0))
        btn = buttons[0]
        assert all(
            getattr(btn, f) is False
            for f in ('up', 'down', 'start', 'select', 'a', 'b', 'emergency_sit')
        )


class TestMultipleCallbacks:
    """Ensure each callback publishes independently."""

    def test_sequential_commands(self):
        node, twists, _ = _make_node()

        node.wireless_controller_callback(make_wireless_msg(ly=1.0))
        node.wireless_controller_callback(make_wireless_msg(ly=0.0))
        node.wireless_controller_callback(make_wireless_msg(ly=-1.0))

        assert len(twists) == 3
        assert twists[0].linear.x == pytest.approx(1.0)
        assert twists[1].linear.x == pytest.approx(0.0)
        assert twists[2].linear.x == pytest.approx(-1.0)
