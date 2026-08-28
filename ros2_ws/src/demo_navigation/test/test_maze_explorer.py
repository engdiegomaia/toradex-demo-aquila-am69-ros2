"""
Máquina de estados e contrato de status do executivo de exploração.

O que está sob teste não é o algoritmo de fronteira -- esse mora em
`frontier.py` e tem os próprios testes. É a parte que o cockpit e o Nav2 veem: o
vocabulário de estados, o JSON publicado, e as três transições cujo erro custa
uma corrida de aceitação inteira -- duas buscas simultâneas, uma meta de
fronteira que volta para sempre, e o marcador velho tratado como fresco.
"""

import json
import math

from demo_navigation.frontier import Frontier
from demo_navigation.maze_explorer import MazeExplorer, STATES
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from std_srvs.srv import Trigger


@pytest.fixture
def node():
    """Um MazeExplorer com o status capturado em vez de publicado."""
    rclpy.init()
    explorer = MazeExplorer()
    published: list[dict] = []
    explorer._status_pub.publish = lambda message: published.append(
        json.loads(message.data))
    explorer.published = published
    yield explorer
    explorer.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def trigger(node) -> Trigger.Response:
    """Uma resposta de serviço vazia, como o rclpy entrega ao callback."""
    del node
    return Trigger.Response()


def test_starts_idle_so_the_launch_never_moves_the_robot(node) -> None:
    """O nó sobe junto com o Nav2 e não pode navegar sem alguém pedir."""
    assert node._state == 'idle'


def test_status_carries_every_field_the_cockpit_reads(node) -> None:
    """O HUD lê estes campos por nome; faltar um apaga parte da tela."""
    node._publish_status()
    payload = node.published[-1]
    for field in ('state', 'elapsed_s', 'frontier_count', 'goal',
                  'blacklisted', 'marker_visible', 'message'):
        assert field in payload, field
    assert payload['state'] in STATES


def test_status_reports_the_goal_in_flight_not_just_a_count(node) -> None:
    """Sem as coordenadas da meta, o operador não sabe para onde ele foi."""
    node._current = Frontier(x=1.5, y=-2.25, cells=12, information_gain_m=0.6)
    node._publish_status()
    goal = node.published[-1]['goal']
    assert goal['x'] == 1.5 and goal['y'] == -2.25
    assert goal['cells'] == 12


def test_every_reachable_state_is_in_the_published_vocabulary(node) -> None:
    """`_publish_status` afirma isso; o teste garante que a asserção é possível."""
    for state in ('idle', 'waiting_map', 'selecting', 'navigating',
                  'homing_exit', 'completed', 'failed', 'cancelled'):
        node._state = state
        node._publish_status()
        assert node.published[-1]['state'] == state


def test_start_takes_the_robot_out_of_idle(node) -> None:
    """Só o serviço arma a busca -- nunca o mapa chegando sozinho."""
    response = node._start(None, trigger(node))
    assert response.success is True
    assert node._state == 'waiting_map'


def test_start_refuses_a_second_run_while_one_is_in_flight(node) -> None:
    """
    Duas buscas no mesmo Nav2 se preemptam e o log não acusa.

    O servidor navigate_to_pose aceita uma meta só: a segunda aborta a primeira,
    cujo callback chega depois e reescreve o estado da nova. É a mesma
    realimentação que o nav_trial documenta em "metas concorrentes".
    """
    node._start(None, trigger(node))
    for state in ('waiting_map', 'selecting', 'navigating', 'homing_exit'):
        node._state = state
        response = node._start(None, trigger(node))
        assert response.success is False, state
        assert node._state == state


def test_start_is_allowed_again_after_a_terminal_state(node) -> None:
    """Uma corrida que terminou não pode travar o cockpit para sempre."""
    for state in ('completed', 'failed', 'cancelled', 'idle'):
        node._state = state
        assert node._start(None, trigger(node)).success is True


def test_start_clears_the_blacklist_of_the_previous_run(node) -> None:
    """A blacklist vale durante uma execução, não entre partidas frias."""
    node._blacklist.append((1.0, 2.0))
    node._homing_failures = 2
    node._start(None, trigger(node))
    assert node._blacklist == []
    assert node._homing_failures == 0


def test_cancel_is_idempotent(node) -> None:
    """O cockpit pode chamar duas vezes; o reset do Nav2 chama junto."""
    node._start(None, trigger(node))
    first = node._cancel(None, trigger(node))
    second = node._cancel(None, trigger(node))
    assert first.success is True and second.success is True
    assert node._state == 'cancelled'


def test_cancel_retires_the_epoch_so_late_callbacks_are_ignored(node) -> None:
    """
    A época é o que impede o callback obsoleto de ressuscitar a busca.

    Sem ela, o resultado da meta cancelada chega depois do `cancel`, encontra o
    nó em `cancelled` e o devolve a `selecting` -- o robô volta a andar depois
    de o operador ter mandado parar.
    """
    node._start(None, trigger(node))
    stale = node._epoch
    node._cancel(None, trigger(node))
    assert node._epoch != stale


def test_blacklisting_returns_to_selecting_and_remembers_the_failure(node) -> None:
    """Uma fronteira que falhou não pode ser a próxima escolha imediata."""
    node._state = 'navigating'
    node._current = Frontier(x=3.0, y=4.0, cells=10, information_gain_m=0.5)
    node._blacklist_current('fronteira terminou com status 6')
    assert node._state == 'selecting'
    assert (3.0, 4.0) in node._blacklist
    assert node._current is None


def test_blacklist_survives_within_the_run_and_filters_by_radius(node) -> None:
    """
    O filtro é por raio, não por igualdade de coordenada.

    A fronteira reaparece deslocada de alguns centímetros a cada atualização do
    mapa; comparar coordenadas exatas faria a blacklist nunca casar e o robô
    voltaria à mesma parede até o prazo total estourar.
    """
    radius = float(node.get_parameter('blacklist_radius_m').value)
    node._blacklist.append((3.0, 4.0))
    near = Frontier(x=3.0 + radius / 2.0, y=4.0, cells=10,
                    information_gain_m=0.5)
    far = Frontier(x=3.0 + radius * 2.0, y=4.0, cells=10,
                   information_gain_m=0.5)
    assert _is_blacklisted(node, near) is True
    assert _is_blacklisted(node, far) is False


def _is_blacklisted(node, frontier) -> bool:
    """Mesmo predicado que `_begin_selection` aplica aos candidatos."""
    radius = float(node.get_parameter('blacklist_radius_m').value)
    return any(math.hypot(frontier.x - x, frontier.y - y) <= radius
               for x, y in node._blacklist)


def test_homing_returns_to_exploration_after_three_failures(node) -> None:
    """Insistir num marcador inalcançável consome o prazo total da corrida."""
    node._state = 'homing_exit'
    for _ in range(2):
        node._homing_failed('aproximação terminou com status 6')
        assert node._state == 'homing_exit'
    node._homing_failed('aproximação terminou com status 6')
    assert node._state == 'selecting'
    assert node._homing_failures == 0


def test_marker_goes_stale_and_stops_counting_as_visible(node) -> None:
    """
    Uma pose antiga é indistinguível de uma atual se ninguém olhar o relógio.

    O detector publica só quando confirma; parar de publicar é como ele diz que
    perdeu o painel. Sem o prazo, o explorador ficaria em `homing_exit`
    perseguindo a última pose vista para sempre.
    """
    stale_s = float(node.get_parameter('marker_stale_s').value)
    node._exit_pose_map = (5.0, 5.0)
    node._exit_seen_s = node._now_s()
    node._publish_status()
    assert node.published[-1]['marker_visible'] is True

    node._exit_seen_s = node._now_s() - stale_s - 1.0
    node._publish_status()
    assert node.published[-1]['marker_visible'] is False


def test_total_timeout_fails_the_run_instead_of_running_forever(node) -> None:
    """O prazo total é o que torna a aceitação HIL uma medida, e não uma espera."""
    node._start(None, trigger(node))
    node._started_s = node._now_s() - float(
        node.get_parameter('total_timeout_s').value) - 1.0
    node._tick()
    assert node._state == 'failed'
    assert node.published[-1]['state'] == 'failed'


def test_tick_is_inert_once_the_run_is_over(node) -> None:
    """Um estado terminal não pode voltar a mandar meta sozinho."""
    for state in ('idle', 'completed', 'failed', 'cancelled'):
        node._state = state
        node._tick()
        assert node._state == state


def test_selection_waits_for_the_map_instead_of_planning_blind(node) -> None:
    """Sem mapa ou sem TF, escolher fronteira é escolher no vazio."""
    node._state = 'selecting'
    node._map = None
    node._begin_selection()
    assert node._state == 'waiting_map'


def test_grid_reads_resolution_and_origin_from_the_live_map(node) -> None:
    """Origem trocada por zero põe toda fronteira no lugar errado."""
    grid_message = OccupancyGrid()
    grid_message.info.width = 4
    grid_message.info.height = 3
    grid_message.info.resolution = 0.05
    grid_message.info.origin.position.x = -1.25
    grid_message.info.origin.position.y = 2.5
    grid_message.info.origin.orientation.w = 1.0
    grid_message.data = [0] * 12
    node._map = grid_message

    grid = node._grid()
    assert (grid.width, grid.height) == (4, 3)
    assert grid.resolution == pytest.approx(0.05)
    assert (grid.origin_x, grid.origin_y) == (-1.25, 2.5)
    assert grid.origin_yaw == pytest.approx(0.0)


def test_exit_pose_without_tf_is_dropped_rather_than_used_raw(node) -> None:
    """
    A pose do detector vem no frame da câmera e é inútil em `map`.

    Usá-la sem transformar mandaria o robô para uma meta a poucos metros da
    PRÓPRIA câmera, no referencial errado -- que é um alvo plausível e errado,
    a pior classe de falha aqui.
    """
    pose = PoseStamped()
    pose.header.frame_id = 'front_camera'
    pose.pose.position.x = 3.0
    node._on_exit_pose(pose)
    assert node._exit_pose_map is None
