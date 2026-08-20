"""
Unit tests for the SI-to-stick conversion.

The whole point of this node is a unit conversion that nothing else in the stack
can see going wrong: publish stick where SI was meant and the robot moves at 40 %
of the request with no error anywhere. These tests are the only place that
mismatch is caught.
"""

from demo_bringup.cmd_vel_si_to_stick import (
    convert,
    STICK_CLAMP,
    to_stick,
    VX_PER_STICK,
    VY_PER_STICK,
    WZ_PER_STICK,
)
from geometry_msgs.msg import Twist
import pytest


def _si(vx=0.0, vy=0.0, wz=0.0):
    t = Twist()
    t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
    return t


def test_round_trip_through_the_controller_gain_recovers_the_si_request():
    # Este é o teste que dá sentido ao nó: o manche publicado, multiplicado pelo
    # ganho que o controlador aplica, tem de devolver a velocidade pedida.
    for si_vx in (0.05, 0.10, 0.15):
        stick = to_stick(si_vx, VX_PER_STICK)
        assert stick * VX_PER_STICK == pytest.approx(si_vx)


def test_the_validated_speed_maps_to_the_stick_value_the_results_recorded():
    # docs/results/ml35-f4-parcial.md: "linear.x = 0.25 (-> v_cmd = 0,1 m/s)".
    assert to_stick(0.10, VX_PER_STICK) == pytest.approx(0.25)


def test_the_stick_clamp_corresponds_to_the_documented_top_speed():
    # O clamp de 0.5 no manche é 0,20 m/s de verdade. Se este número mudar, o
    # envelope documentado em guias e resultados deixou de valer.
    assert STICK_CLAMP * VX_PER_STICK == pytest.approx(0.20)
    assert STICK_CLAMP * WZ_PER_STICK == pytest.approx(0.25)


def test_conversion_scales_all_three_axes_by_their_own_gain():
    out = convert(_si(vx=0.08, vy=0.06, wz=0.10))
    assert out.linear.x == pytest.approx(0.08 / VX_PER_STICK)
    assert out.linear.y == pytest.approx(0.06 / VY_PER_STICK)
    assert out.angular.z == pytest.approx(0.10 / WZ_PER_STICK)


def test_conversion_does_not_flip_any_sign():
    # `twist_to_inputs` já nega lx e rx. Negar aqui também faria o robô virar
    # para o lado errado, e o Nav2 corrigiria aumentando o erro.
    out = convert(_si(vx=-0.08, vy=-0.06, wz=-0.10))
    assert out.linear.x < 0 and out.linear.y < 0 and out.angular.z < 0


def test_request_above_the_envelope_is_clamped_not_wrapped():
    out = convert(_si(vx=10.0, wz=-10.0))
    assert out.linear.x == pytest.approx(STICK_CLAMP)
    assert out.angular.z == pytest.approx(-STICK_CLAMP)


def test_zero_stays_exactly_zero():
    # Silêncio é o comando de parada, mas um zero explícito tem de sair zero:
    # um viés aqui faria o robô derivar durante toda pausa do Nav2.
    out = convert(_si())
    assert (out.linear.x, out.linear.y, out.angular.z) == (0.0, 0.0, 0.0)
