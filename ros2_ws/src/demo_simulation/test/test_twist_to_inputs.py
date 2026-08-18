"""Unit tests for the public Twist to private gait-input contract."""

from demo_simulation.twist_to_inputs import (
    _CommandGate,
    _has_motion_command,
    _StartLatch,
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


def test_gate_publishes_a_fresh_command():
    gate = _CommandGate(timeout_s=0.3)
    gate.record(_twist(x=0.01), now=10.0)

    sample = gate.sample(now=10.2)

    assert not gate.is_stale(now=10.2)
    assert sample.ly == pytest.approx(0.01)
    assert sample.command == 0


def test_gate_zeroes_a_stale_command():
    gate = _CommandGate(timeout_s=0.3)
    gate.record(_twist(x=0.01, y=0.01, yaw=0.01), now=10.0)

    sample = gate.sample(now=10.4)

    assert gate.is_stale(now=10.4)
    assert (sample.ly, sample.lx, sample.rx) == (0.0, 0.0, 0.0)


def test_gate_is_stale_before_any_command_arrives():
    gate = _CommandGate(timeout_s=0.3)

    assert gate.is_stale(now=0.0)
    assert gate.sample(now=0.0).ly == 0.0


def test_gate_keeps_the_command_exactly_at_the_timeout():
    # The boundary is inclusive: a command that is exactly one timeout old is
    # still the operator's command, not a gap in the stream.
    gate = _CommandGate(timeout_s=0.5)
    gate.record(_twist(x=0.02), now=0.0)

    assert not gate.is_stale(now=0.5)
    assert gate.is_stale(now=0.6)


def test_gate_refreshes_on_every_command():
    gate = _CommandGate(timeout_s=0.3)
    gate.record(_twist(x=0.02), now=1.0)
    gate.record(_twist(x=0.01), now=1.2)

    sample = gate.sample(now=1.4)

    assert not gate.is_stale(now=1.4)
    assert sample.ly == pytest.approx(0.01)


def test_gate_keeps_the_controller_sign_convention():
    gate = _CommandGate(timeout_s=0.3)
    gate.record(_twist(y=0.02, yaw=0.02), now=1.0)

    sample = gate.sample(now=1.0)

    assert sample.lx == pytest.approx(-0.02)
    assert sample.rx == pytest.approx(-0.02)


def test_start_latch_repeats_the_trot_command():
    # One message is not enough: the controller reads a struct, not a stream,
    # so a single command=4 can be overwritten before any update loop sees it.
    latch = _StartLatch(ticks=3)
    latch.arm()

    assert [latch.next_command() for _ in range(3)] == [4, 4, 4]


def test_start_latch_goes_quiet_after_the_window():
    latch = _StartLatch(ticks=2)
    latch.arm()
    latch.next_command()
    latch.next_command()

    assert latch.next_command() == 0


def test_start_latch_is_quiet_until_armed():
    assert _StartLatch(ticks=2).next_command() == 0


def test_start_latch_rearms():
    latch = _StartLatch(ticks=1)
    latch.arm()
    latch.next_command()
    latch.arm()

    assert latch.next_command() == 4
