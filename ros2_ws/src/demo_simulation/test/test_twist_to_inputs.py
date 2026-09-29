"""Unit tests for the public Twist to private gait-input contract."""

from demo_simulation.twist_to_inputs import (
    _CMD_NONE,
    _CMD_STAND_STEP,
    _CMD_START_TROT,
    _CommandGate,
    _has_motion_command,
    _RestandSequence,
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

    assert (positive.ly, positive.lx, positive.rx) == (0.5, -0.5, -0.5)
    assert (negative.ly, negative.lx, negative.rx) == (-0.5, 0.5, 0.5)


def test_safe_stick_boundary_is_inclusive():
    assert _to_safe_stick(0.5) == pytest.approx(0.5)
    assert _to_safe_stick(-0.5) == pytest.approx(-0.5)


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


# --- stop, teleport, resume -------------------------------------------------
#
# What these tests guard is not a style preference: each one corresponds to a
# MEASURED way of dropping the robot. See HOLD_SERVICE in twist_to_inputs.py.


def _drain(sequence, now, ticks):
    """Commands emitted over `ticks` consecutive ticks at the same instant."""
    return [sequence.next_command(now) for _ in range(ticks)]


def test_a_sequencia_e_inerte_antes_do_hold():
    sequence = _RestandSequence()

    assert not sequence.active
    assert not sequence.holding
    assert sequence.next_command(0.0) == _CMD_NONE


def test_o_hold_sai_de_trotting_com_o_comando_de_stand():
    sequence = _RestandSequence()
    sequence.hold()

    assert sequence.active
    assert sequence.holding
    assert sequence.next_command(0.0) == _CMD_STAND_STEP


def test_o_comando_de_stand_sai_exatamente_uma_vez():
    """
    A HELD `2` takes FIXEDSTAND to FIXEDDOWN and the robot lies down.

    StateFixedStand::checkChange, case 2. Repeating the stand-up command is
    safe (StateTrotting ignores it), repeating the lie-down one is not -- and
    that is the difference between resettling and dismounting the robot.
    """
    sequence = _RestandSequence()
    sequence.hold()

    emitted = _drain(sequence, 1.0, 40)

    assert emitted[0] == _CMD_STAND_STEP
    assert _CMD_STAND_STEP not in emitted[1:]


def test_o_hold_publica_zeros_em_vez_de_ficar_calado():
    """
    Silence does not work: the byte written to the controller persists.

    Stopping publishing would leave the transition's `2` in effect until the
    next tick that publishes anything.
    """
    sequence = _RestandSequence()
    sequence.hold()
    sequence.next_command(0.0)

    assert _drain(sequence, 2.5, 5) == [_CMD_NONE] * 5


def test_o_hold_nao_tem_prazo_proprio():
    """
    The teleport window closes via `resume`, never via time.

    A deadline here would be a race against the relay's service call: the
    robot would start walking again in the middle of the teleport.
    """
    sequence = _RestandSequence(settle_s=5.0)
    sequence.hold()
    sequence.next_command(0.0)

    assert _drain(sequence, 10_000.0, 20) == [_CMD_NONE] * 20
    assert sequence.holding


def test_o_resume_so_volta_a_trotting_depois_de_assentar():
    sequence = _RestandSequence(settle_s=5.0, trot_ticks=3)
    sequence.hold()
    sequence.next_command(0.0)
    sequence.resume(0.0)

    assert not sequence.holding
    assert sequence.next_command(4.999) == _CMD_NONE
    assert sequence.next_command(5.0) == _CMD_START_TROT


def test_o_resume_repete_o_comando_de_trote_na_janela():
    """
    Publishing `4` just once is the failure mode measured on 18/08/2026.

    See _START_TROT_HOLD_S: the subscription writes to a single struct and
    the update loop reads whatever is there, so an isolated `4` can be
    overwritten before any update sees it -- and the robot is left standing
    in fixed stand, silently.
    """
    sequence = _RestandSequence(settle_s=1.0, trot_ticks=4)
    sequence.hold()
    sequence.next_command(0.0)
    sequence.resume(0.0)

    assert _drain(sequence, 1.0, 4) == [_CMD_START_TROT] * 4


def test_a_sequencia_fica_inerte_no_fim():
    sequence = _RestandSequence(settle_s=1.0, trot_ticks=2)
    sequence.hold()
    sequence.next_command(0.0)
    sequence.resume(0.0)
    _drain(sequence, 1.0, 2)

    assert sequence.next_command(1.0) == _CMD_NONE
    assert not sequence.active


def test_um_resume_sem_hold_nao_faz_nada():
    """
    Otherwise a stray resume would inject `4` into a robot that was never stopped.

    This is the path where the relay fails BEFORE the hold: nothing to resume.
    """
    sequence = _RestandSequence(settle_s=1.0, trot_ticks=2)
    sequence.resume(0.0)

    assert not sequence.active
    assert _drain(sequence, 5.0, 5) == [_CMD_NONE] * 5


def test_a_sequencia_serve_um_segundo_reset():
    sequence = _RestandSequence(settle_s=1.0, trot_ticks=1)
    sequence.hold()
    sequence.next_command(0.0)
    sequence.resume(0.0)
    _drain(sequence, 1.0, 2)
    assert not sequence.active

    sequence.hold()

    assert sequence.next_command(10.0) == _CMD_STAND_STEP


def test_o_assentamento_nao_avanca_com_o_tempo_simulado_parado():
    """
    A paused world must not make the sequence progress.

    `now` is simulated time (see _now in the node). If the operator pauses
    during the reset, the wait must actually wait.
    """
    sequence = _RestandSequence(settle_s=5.0, trot_ticks=2)
    sequence.hold()
    sequence.next_command(3.0)
    sequence.resume(3.0)

    assert _drain(sequence, 3.0, 50) == [_CMD_NONE] * 50
    assert sequence.active
