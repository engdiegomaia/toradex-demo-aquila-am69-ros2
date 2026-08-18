"""Unit tests for the public Twist to private gait-input contract."""

from demo_simulation.twist_to_inputs import (
    _has_motion_command,
    _to_safe_stick,
    _twist_to_inputs,
)
from geometry_msgs.msg import Twist
import pytest


def _twist(x=0.0, y=0.0, yaw=0.0):
    message = Twist()
    message.linear.x = x
    message.linear.y = y
    message.angular.z = yaw
    return message


def test_zero_twist_maps_to_centered_sticks():
    result = _twist_to_inputs(_twist())

    assert result.command == 0
    assert result.ly == 0.0
    assert result.lx == 0.0
    assert result.rx == 0.0
    assert result.ry == 0.0


def test_low_commands_keep_unit_gain_and_controller_signs():
    result = _twist_to_inputs(_twist(x=0.015, y=0.015, yaw=0.015))

    assert result.ly == pytest.approx(0.015)
    assert result.lx == pytest.approx(-0.015)
    assert result.rx == pytest.approx(-0.015)


def test_all_axes_saturate_symmetrically():
    positive = _twist_to_inputs(_twist(x=9.0, y=9.0, yaw=9.0))
    negative = _twist_to_inputs(_twist(x=-9.0, y=-9.0, yaw=-9.0))

    assert (positive.ly, positive.lx, positive.rx) == (0.03, -0.03, -0.03)
    assert (negative.ly, negative.lx, negative.rx) == (-0.03, 0.03, 0.03)


def test_safe_stick_boundary_is_inclusive():
    assert _to_safe_stick(0.03) == pytest.approx(0.03)
    assert _to_safe_stick(-0.03) == pytest.approx(-0.03)


def test_zero_twist_does_not_start_trotting():
    assert not _has_motion_command(_twist())


def test_any_motion_axis_starts_trotting():
    assert _has_motion_command(_twist(yaw=0.001))
