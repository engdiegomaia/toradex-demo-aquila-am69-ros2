"""
Orbita das cameras de cena: a matematica, sem no e sem Gazebo.

O que da errado aqui nao aparece como erro em lugar nenhum -- aparece como uma
camera que aponta para o lugar errado, ou que nao se mexe. Por isso o alvo dos
testes e a classe `Orbit`, que e a unica parte com logica de verdade e a unica
que da para exercitar sem simulador.
"""

import math

from demo_simulation.scene_view_controller import (
    MAX_FOLLOW_OFFSET_M,
    MIN_PITCH_RAD,
    Orbit,
)
from geometry_msgs.msg import Twist


def _twist(**fields):
    """Twist com um eixo mexido, os outros cinco em zero."""
    message = Twist()
    for name, value in fields.items():
        axis, component = name.split('_')
        setattr(getattr(message, axis), component, value)
    return message


def _iso():
    """Build the launch default iso view."""
    return Orbit(-3.0, 3.0, 2.4, 0.5150, -0.7854)


def _top():
    """Build the launch default top view: the degenerate case, pitch = pi/2."""
    return Orbit(0.0, 0.0, 6.0, math.pi / 2, math.pi / 2)


# --- o enquadramento medido tem de sobreviver a ida e volta -----------------

def test_orbit_round_trips_the_launch_pose():
    """
    Position() tem de devolver a pose com que a camera nasceu.

    A orbita e semeada por (x, y, z, pitch, yaw) e guarda (alvo, distancia,
    pitch, yaw). Se a inversao estiver errada, a camera SALTA no primeiro comando
    -- ela nasce no lugar certo, spawnada pelo `create`, e vai para outro no
    primeiro set_pose.
    """
    for orbit in (_iso(), _top()):
        x, y, z = orbit.position()
        expected = orbit.home[:3]
        assert math.isclose(x, expected[0], abs_tol=1e-9)
        assert math.isclose(y, expected[1], abs_tol=1e-9)
        assert math.isclose(z, expected[2], abs_tol=1e-9)


def test_top_view_target_is_the_point_below_the_camera():
    """Com pitch = pi/2 o alvo e o ponto sob a camera: cos(pitch) = 0."""
    top = _top()
    assert math.isclose(top.target[0], 0.0, abs_tol=1e-9)
    assert math.isclose(top.target[1], 0.0, abs_tol=1e-9)


# --- seguir o robo ---------------------------------------------------------

def test_following_moves_the_camera_by_the_robot_displacement():
    """
    Seguir move o ALVO, e so ele.

    Distancia, azimute e inclinacao sao o enquadramento que alguem mediu contra
    a pegada real do cenario (ver o cabecalho de scene_cameras.launch.py).
    Seguir o robo nao e motivo para mexer neles, e o teste que garante isso e
    este: a camera desliza, nao reenquadra.
    """
    orbit = _iso()
    before = (orbit.distance, orbit.pitch, orbit.yaw)
    start = orbit.position()
    # O alvo de casa NAO e exatamente a origem: o yaw do launch e -0.7854, um
    # pi/4 arredondado, e sobra ~1 mm. Comparar contra (0, 0) faria este teste
    # falhar por um erro que nao e o que ele guarda.
    target_before = orbit.target

    orbit.follow((2.0, -1.0))
    moved = orbit.position()

    assert (orbit.distance, orbit.pitch, orbit.yaw) == before
    # A camera anda o MESMO vetor que o alvo andou.
    assert math.isclose(moved[0] - start[0], 2.0 - target_before[0], abs_tol=1e-9)
    assert math.isclose(moved[1] - start[1], -1.0 - target_before[1], abs_tol=1e-9)
    assert math.isclose(moved[2], start[2], abs_tol=1e-9)


def test_following_the_top_view_puts_the_camera_over_the_robot():
    """
    Na vista de topo, seguir significa ficar EM CIMA do robo.

    E o caso degenerado do modelo orbital e o mais visivel na tela: se o alvo e
    a posicao divergirem aqui, o robo aparece na borda do quadro em vez do
    centro. A altura nao muda -- o zoom da vista de topo E a altura.
    """
    top = _top()
    height = top.position()[2]

    top.follow((-4.855, 4.855))
    x, y, z = top.position()

    assert math.isclose(x, -4.855, abs_tol=1e-9)
    assert math.isclose(y, 4.855, abs_tol=1e-9)
    assert math.isclose(z, height, abs_tol=1e-9)


def test_pan_while_following_survives_the_next_follow_tick():
    """
    O pan tem de morar no OFFSET enquanto segue.

    Escrito no alvo, ele seria sobreposto pelo `follow()` do tique seguinte,
    100 ms depois: o operador aperta "mover para a direita", a imagem pisca de
    volta e o botao parece nao funcionar. Foi para isso que `apply` ganhou o
    argumento `following`.
    """
    orbit = _iso()
    orbit.follow((0.0, 0.0))

    orbit.apply(_twist(linear_y=0.8), following=True)
    panned = orbit.position()

    # O tique seguinte, com o robo parado no mesmo lugar.
    orbit.follow((0.0, 0.0))
    assert orbit.position() == panned, 'o pan foi engolido pelo tique de follow'
    assert orbit.follow_offset != (0.0, 0.0)


def test_pan_when_not_following_moves_the_world_target():
    """Parado, o pan continua movendo um ponto do mundo, como antes."""
    orbit = _iso()
    target = orbit.target

    orbit.apply(_twist(linear_y=0.8), following=False)

    assert orbit.target != target
    assert orbit.follow_offset == (0.0, 0.0), (
        'o pan em modo livre nao deve sujar o offset de seguimento'
    )


def test_follow_offset_is_bounded():
    """
    Dedo preso no botao de pan nao pode perder o robo de vista.

    Sem teto, segurar "mover" enquanto segue afasta o alvo indefinidamente e o
    robo sai do quadro -- exatamente o que seguir existe para evitar. Nao ha
    botao que traga de volta, so o recentrar.
    """
    orbit = _iso()
    for _ in range(200):
        orbit.apply(_twist(linear_y=0.8), following=True)

    assert math.hypot(*orbit.follow_offset) <= MAX_FOLLOW_OFFSET_M + 1e-9


def test_reset_clears_the_follow_offset():
    """
    Recentrar significa o mesmo nos dois modos.

    Parado, volta ao enquadramento medido. Seguindo, volta a ter o robo no
    centro -- que so acontece se o offset zerar aqui.
    """
    orbit = _iso()
    orbit.apply(_twist(linear_y=3.0), following=True)
    home = _iso().position()

    orbit.reset()

    assert orbit.follow_offset == (0.0, 0.0)
    assert orbit.position() == home


# --- os limites que existiam antes continuam valendo -----------------------

def test_pitch_never_reaches_the_ground():
    """
    Abaixo de MIN_PITCH_RAD a orbita perde o alvo.

    sin(pitch) -> 0 e a distancia explode; o clamp e o que impede a divisao por
    quase-zero em `reset`.
    """
    orbit = _iso()
    for _ in range(50):
        orbit.apply(_twist(angular_y=-0.18))

    assert orbit.pitch >= MIN_PITCH_RAD
