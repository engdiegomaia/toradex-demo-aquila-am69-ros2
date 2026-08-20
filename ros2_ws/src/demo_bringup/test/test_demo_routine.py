"""
Unit tests for the exhibition choreography.

Only the ROS-free parts are covered here: the timeline, the SI-to-Twist
conversion, and whether the pattern closes. Those are where a silent error
produces a robot moving at a speed nobody asked for, or one that slowly walks
out of the display area -- both measured failures, both cheap to guard.
"""

import math

from demo_bringup.demo_routine import (
    default_choreography,
    Schedule,
    Segment,
    to_twist,
    VX_MAX,
    VY_MAX,
    WZ_MAX,
    YAW_TRACKING_ARC,
    YAW_TRACKING_SPOT,
)


def _segments():
    return (Segment('a', 0.1, 0.0, 0.0, 4.0),
            Segment('b', 0.0, 0.0, 0.1, 2.0))


def test_settle_is_interleaved_after_every_segment():
    schedule = Schedule(_segments(), settle_s=1.0)

    assert schedule.total_s == 8.0
    assert schedule.segment_count == 2


def test_segment_is_active_during_its_own_window():
    schedule = Schedule(_segments(), settle_s=1.0)

    assert schedule.at(0.0)[0].name == 'a'
    assert schedule.at(3.9)[0].name == 'a'


def test_settle_returns_no_segment_so_the_caller_publishes_nothing():
    schedule = Schedule(_segments(), settle_s=1.0)

    segment, finished = schedule.at(4.5)

    assert segment is None
    assert finished is False


def test_second_segment_follows_the_first_settle():
    schedule = Schedule(_segments(), settle_s=1.0)

    assert schedule.at(5.0)[0].name == 'b'
    assert schedule.at(6.9)[0].name == 'b'


def test_pattern_repeats_when_looping():
    schedule = Schedule(_segments(), settle_s=1.0, loop=True)

    assert schedule.at(8.0)[0].name == 'a'
    assert schedule.at(13.0)[0].name == 'b'


def test_pattern_finishes_when_not_looping():
    schedule = Schedule(_segments(), settle_s=1.0, loop=False)

    segment, finished = schedule.at(8.0)

    assert segment is None
    assert finished is True


def test_zero_duration_segments_are_dropped_instead_of_stalling():
    schedule = Schedule((Segment('nada', 0.1, 0.0, 0.0, 0.0),), settle_s=0.0)

    assert schedule.segment_count == 0
    assert schedule.at(0.0) == (None, True)


def test_forward_command_converts_with_the_measured_gain():
    # v_x = 0.4 * linear.x, so the validated 0.10 m/s point is linear.x = 0.25.
    assert to_twist(0.10, 0.0, 0.0).linear.x == 0.25


def test_lateral_and_yaw_use_their_own_gains():
    message = to_twist(0.0, 0.15, 0.25)

    assert message.linear.y == 0.5
    assert message.angular.z == 0.5


def test_command_beyond_the_envelope_is_clamped_not_passed_through():
    message = to_twist(10.0, -10.0, 10.0)

    assert message.linear.x == 0.5
    assert message.linear.y == -0.5
    assert message.angular.z == 0.5


def test_default_choreography_stays_inside_the_reachable_envelope():
    for segment in default_choreography():
        assert abs(segment.vx) <= VX_MAX
        assert abs(segment.vy) <= VY_MAX
        assert abs(segment.wz) <= WZ_MAX


def test_default_choreography_keeps_yaw_below_the_qp_saturation_point():
    # Phase 2 measured the balance QP saturating near 0.13 rad/s; asking for
    # more only drags the feet.
    for segment in default_choreography():
        assert abs(segment.wz) <= 0.13


def _integrate(segments):
    """Roda a cinemática de uniciclo com os ganhos de guinada medidos."""
    x = y = theta = 0.0
    for segment in segments:
        steps = 500
        dt = segment.duration_s / steps
        in_place = abs(segment.vx) < 1e-9 and abs(segment.vy) < 1e-9
        gain = YAW_TRACKING_SPOT if in_place else YAW_TRACKING_ARC
        for _ in range(steps):
            theta += segment.wz * gain * dt
            x += (segment.vx * math.cos(theta) - segment.vy * math.sin(theta)) * dt
            y += (segment.vx * math.sin(theta) + segment.vy * math.cos(theta)) * dt
    return x, y, math.degrees(theta)


def test_default_choreography_returns_to_where_it_started():
    # O padrão fecha por geometria, não por cancelamento de erro. Se este teste
    # cair, o robô sai da área de exposição: a versão anterior, que pareava arcos
    # invertendo o sinal de wz, andava 1,32 m em x e 0,89 m em y por passada.
    x, y, heading = _integrate(default_choreography())
    assert math.hypot(x, y) < 0.01
    assert abs(heading % 360.0) < 1.0 or abs(heading % 360.0 - 360.0) < 1.0


def test_calibrated_turns_sweep_the_angle_that_was_asked_for():
    # A guinada é sub-rastreada em 1-3%; sem dividir pelos ganhos medidos um
    # giro comandado por tempo fecha curto e a figura precessa rápido.
    spot = [s for s in default_choreography() if s.name.startswith('giro 90')]
    arcs = [s for s in default_choreography() if s.name.startswith('arco 90')]
    assert len(spot) == 4 and len(arcs) == 4
    for segment in spot:
        swept = math.degrees(segment.wz * YAW_TRACKING_SPOT * segment.duration_s)
        assert abs(swept - 90.0) < 0.5
    for segment in arcs:
        swept = math.degrees(segment.wz * YAW_TRACKING_ARC * segment.duration_s)
        assert abs(swept - 90.0) < 0.5
