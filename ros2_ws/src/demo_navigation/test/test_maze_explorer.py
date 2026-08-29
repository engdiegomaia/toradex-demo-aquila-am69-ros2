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
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
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


def _is_suppressed(node, frontier) -> bool:
    """As duas listas juntas, que e o que o filtro de candidatos aplica."""
    radius = float(node.get_parameter('blacklist_radius_m').value)
    return any(math.hypot(frontier.x - x, frontier.y - y) <= radius
               for x, y in list(node._blacklist) + list(node._refused)
               + list(node._timed_out))


class _Wrapped:
    """O que `get_result_async()` entrega: status mais o resultado da acao."""

    def __init__(self, status: int) -> None:
        self.status = status
        self.result = None


class _Future:
    """Future ja resolvido, do jeito que o rclpy chama o callback."""

    def __init__(self, value) -> None:
        self._value = value

    def result(self):
        return self._value


def test_a_frontier_the_planner_refuses_is_retired(node) -> None:
    """
    O modo de falha que consumiu 77% do orcamento na fumaca de 28/08.

    `_blacklist_current` so dispara quando o Nav2 RECUSA a meta ou quando a meta
    despachada expira. Uma fronteira cujo `ComputePathToPose` REPROVA nunca
    passava por ali: `_best` ficava `None`, a mensagem virava "planner rejeitou
    todas as fronteiras", e o mesmo candidato morto era oferecido de novo no
    ciclo seguinte -- 459 vezes seguidas, com o mapa congelado.
    """
    node._start(None, trigger(node))
    node._state = 'selecting'
    frontier = Frontier(x=-3.06, y=0.28, cells=70, information_gain_m=0.9)
    node._candidates = [frontier]
    node._candidate_index = 1
    node._best = None

    node._on_path_result(_Future(_Wrapped(GoalStatus.STATUS_ABORTED)),
                         node._epoch, frontier)

    assert _is_suppressed(node, frontier) is True, (
        'fronteira reprovada pelo planejador continua sendo oferecida')


def test_retiring_a_refused_frontier_does_not_retire_the_epoch(node) -> None:
    """
    A blacklist normal troca de epoca; esta NAO pode.

    `_blacklist_current` incrementa `_epoch` de proposito, para invalidar o
    callback da meta que estava em voo. Aqui nao ha meta em voo: ha uma rodada
    de validacao em andamento, e trocar a epoca no meio dela faz
    `_on_path_result` dos candidatos seguintes retornar cedo. A validacao
    pararia na metade e o ciclo morreria em silencio.
    """
    node._start(None, trigger(node))
    node._state = 'selecting'
    frontier = Frontier(x=1.0, y=1.0, cells=10, information_gain_m=0.5)
    node._candidates = [frontier]
    node._candidate_index = 1
    epoch = node._epoch

    node._on_path_result(_Future(_Wrapped(GoalStatus.STATUS_ABORTED)),
                         node._epoch, frontier)

    assert node._epoch == epoch


def test_a_frontier_the_planner_accepts_is_not_retired(node) -> None:
    """A guarda nao pode aposentar o caminho feliz junto."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    frontier = Frontier(x=2.0, y=2.0, cells=10, information_gain_m=0.5)
    node._candidates = [frontier]
    node._candidate_index = 1

    path = SimpleNamespace(path=SimpleNamespace(poses=[]))
    wrapped = _Wrapped(GoalStatus.STATUS_SUCCEEDED)
    wrapped.result = path
    node._on_path_result(_Future(wrapped), node._epoch, frontier)

    assert node._blacklist == [] and node._refused == []


def test_a_planner_refusal_is_provisional_and_lifts_when_the_map_grows(
        node) -> None:
    """
    "Inalcancavel agora" nao e "inalcancavel sempre", e a diferenca e o mapa.

    `ExplorationGrid` roda com `allow_unknown: false`, entao uma fronteira
    distante e reprovada porque o CAMINHO ate ela atravessa desconhecido -- nao
    porque a fronteira seja ruim. Medido na rodada 1 (29/08): as duas unicas
    reprovacoes foram (0.07, 3.20) e (-2.93, 0.15), a 2,3 m e 2,7 m do robo, e
    aposenta-las de vez matou a metade distante do labirinto. Sobraram 3
    clusters e 157 celulas de fronteira real com zero candidatos permitidos.

    Reduzir o raio nao resolve: o ponto anotado E o centroide do cluster, entao
    qualquer raio maior que zero mata o proprio cluster que o gerou. O que tem
    de mudar e a permanencia.
    """
    node._start(None, trigger(node))
    node._state = 'selecting'
    far = Frontier(x=-2.93, y=0.15, cells=80, information_gain_m=1.2)
    node._candidates = [far]
    node._candidate_index = 1

    node._on_path_result(_Future(_Wrapped(GoalStatus.STATUS_ABORTED)),
                         node._epoch, far)
    assert _is_suppressed(node, far) is True

    node._current = Frontier(x=0.0, y=0.5, cells=10, information_gain_m=0.5)
    node._on_nav_result(_Future(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED)), node._epoch, True)

    assert _is_suppressed(node, far) is False, (
        'chegar a uma meta muda o mapa; a reprovacao anterior tem de expirar')


def test_reaching_a_goal_does_not_lift_a_hard_blacklist(node) -> None:
    """
    A blacklist dura permanece: ela registra falha de EXECUCAO, nao de mapa.

    O Nav2 devolver falha explicita para aquela meta diz algo sobre aquela
    fronteira que mapa novo nao desmente. Confundir as listas traz de volta o
    livelock da fumaca de 28/08 por outro caminho.

    A expiracao de meta JA NAO e exemplo disto: a rodada 3 mostrou que ela
    marca uma tentativa travada, nao uma fronteira invalida. Ver
    `test_a_timed_out_frontier_is_not_hard_blacklisted`.
    """
    node._start(None, trigger(node))
    node._state = 'navigating'
    node._current = Frontier(x=3.0, y=4.0, cells=10, information_gain_m=0.5)
    node._blacklist_current('fronteira terminou com status 6')
    epoch = node._epoch

    node._current = Frontier(x=0.0, y=0.5, cells=10, information_gain_m=0.5)
    node._on_nav_result(_Future(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED)), epoch, True)

    assert (3.0, 4.0) in node._blacklist


def test_the_goal_timeout_leaves_room_for_more_than_one_goal(node) -> None:
    """
    O prazo por meta e o prazo total nao sao independentes.

    Curto demais e ele expira metas que estavam progredindo, e cada expiracao
    manda a fronteira para a blacklist DURA -- foi o que matou a rodada 2, com
    3 expiracoes engolindo os 4 clusters restantes. Longo demais e uma meta
    ruim consome a corrida inteira. O piso util e caber pelo menos tres vezes
    no orcamento total.
    """
    goal = float(node.get_parameter('goal_timeout_s').value)
    total = float(node.get_parameter('total_timeout_s').value)
    assert goal * 3 <= total, (
        f'{goal} s por meta nao cabe tres vezes em {total} s de orcamento')


def test_the_goal_timeout_stays_at_the_value_that_was_measured_best(
        node) -> None:
    """
    Trava um experimento REPROVADO para que ninguem o repita.

    A rodada 2 expirou tres metas distantes em exatamente 90,0 s, o que le como
    "o teto e curto demais". A rodada 3 subiu para 180 s e piorou tudo: 4,26 m
    contra 21,93 m, 3746 celulas contra 8915, razao de trabalho 8,6% contra
    41,7%, e nenhuma deteccao do marcador -- porque travou 180 s numa meta a
    0,4 m do robo. O teto corta travamento, nao travessia lenta.

    Evidencia: docs/results/ml35-f5-exploration-r3.md.
    """
    assert float(node.get_parameter('goal_timeout_s').value) == 90.0, (
        '180 s foi medido e REPROVADO na rodada 3 -- ler '
        'docs/results/ml35-f5-exploration-r3.md antes de tentar de novo. O que '
        'falta corrigir e a permanencia da blacklist, nao o teto.')


def _timed_out_frontier(node):
    """Uma fronteira levada ate a expiracao de meta, como o `_tick` faz."""
    frontier = Frontier(x=-3.295, y=0.428, cells=35, information_gain_m=1.75)
    node._start(None, trigger(node))
    node._state = 'navigating'
    node._current = frontier
    node._timeout_current('meta de fronteira expirou')
    return frontier


def test_a_timed_out_frontier_is_not_hard_blacklisted(node) -> None:
    """
    Expirar uma meta marca a TENTATIVA, nao a fronteira. Rodada 3 provou isso.

    Uma meta a 0,4 m do robo consumiu 180 s inteiros: o teto corta travamento,
    e travamento fala da pose, do costmap e do plano daquele instante -- nada
    disso e permanente. Na rodada 2, tres expiracoes viraram tres pontos
    permanentes que engoliram os quatro clusters restantes aos 570 s, e a
    corrida morreu com fronteira real disponivel.

    Evidencia: docs/results/ml35-f5-exploration-r{2,3}.md.
    """
    frontier = _timed_out_frontier(node)

    assert node._blacklist == [], (
        'expiracao de meta nao pode entrar na blacklist dura')
    assert (frontier.x, frontier.y) in node._timed_out
    assert node._state == 'selecting'


def test_a_timed_out_frontier_is_suppressed_at_once(node) -> None:
    """Provisoria nao quer dizer frouxa: a meta seguinte tem de ser outra."""
    frontier = _timed_out_frontier(node)
    assert _is_suppressed(node, frontier) is True


def test_map_republication_does_not_release_a_timed_out_frontier(node) -> None:
    """
    O que libera e progresso, nao tempo nem mensagem.

    `slam_toolbox` republica `/map` a cada 1 s mexa o mapa ou nao. Se a
    republicacao limpasse a supressao, a fronteira travada voltaria a cada
    segundo e o livelock de 28/08 estaria de volta por outro caminho.
    """
    frontier = _timed_out_frontier(node)
    for _ in range(5):
        node._on_map(OccupancyGrid())
    assert _is_suppressed(node, frontier) is True


def test_reaching_another_frontier_releases_a_timed_out_frontier(node) -> None:
    """Chegar noutro lugar muda pose, costmap e plano -- os tres motivos."""
    frontier = _timed_out_frontier(node)

    node._current = Frontier(x=0.0, y=0.5, cells=10, information_gain_m=0.5)
    node._on_nav_result(_Future(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED)), node._epoch, True)

    assert _is_suppressed(node, frontier) is False


def test_an_explicit_nav2_failure_is_still_hard_in_this_round(node) -> None:
    """
    Uma politica por rodada. O resultado de falha do Nav2 continua duro.

    Trocar as duas permanencias na mesma rodada faria o resultado ilegivel:
    nao daria para dizer qual das duas produziu a diferenca.
    """
    node._start(None, trigger(node))
    node._state = 'navigating'
    node._current = Frontier(x=3.0, y=4.0, cells=10, information_gain_m=0.5)
    node._on_nav_result(_Future(SimpleNamespace(
        status=GoalStatus.STATUS_ABORTED)), node._epoch, True)

    assert (3.0, 4.0) in node._blacklist
    assert node._timed_out == []


def test_a_new_run_clears_every_suppression_list(node) -> None:
    """Supressao vale dentro de uma execucao, nunca entre partidas frias."""
    node._blacklist.append((1.0, 2.0))
    node._refused.append((3.0, 4.0))
    node._timed_out.append((5.0, 6.0))
    node._state = 'failed'

    node._start(None, trigger(node))

    assert node._blacklist == []
    assert node._refused == []
    assert node._timed_out == []


def test_a_barren_selection_fails_the_run_instead_of_idling(node) -> None:
    """
    Aposentar fronteiras sem condicao terminal troca um livelock por outro.

    Com a correcao acima, a blacklist pode acabar engolindo todas as fronteiras.
    O codigo antigo escrevia "nenhuma fronteira segura alcancavel" e continuava
    em `selecting` para sempre -- silencioso, e indistinguivel de estar
    trabalhando. O robo parado nao produz mapa novo, entao a situacao nunca se
    resolve sozinha: e falha, e tem de ser declarada.
    """
    node._start(None, trigger(node))
    node._state = 'selecting'
    limit = int(node.get_parameter('barren_selections_limit').value)

    for _ in range(limit):
        assert node._state == 'selecting'
        node._note_barren_selection()

    assert node._state == 'failed'
    assert 'fronteira' in node._message


def test_dispatching_a_goal_clears_the_barren_streak(node) -> None:
    """A contagem e de ciclos CONSECUTIVOS; uma meta despachada zera."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    node._note_barren_selection()
    node._note_barren_selection()
    assert node._barren_cycles == 2
    node._barren_cycles = 0  # o que `_send_navigation` faz ao despachar
    node._note_barren_selection()
    assert node._barren_cycles == 1
    assert node._state == 'selecting'


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


# --- custo da selecao de fronteira -----------------------------------------
#
# Medido neste host x86 sobre um mapa de SLAM do tamanho do maze11 (234 x 284
# celulas) a 95% explorado: `extract_frontiers` custava 158,6 ms e era chamado a
# cada tick de 1 Hz enquanto o estado fosse `selecting` sem meta pendente -- que
# e exatamente o caso "planner rejeitou todas as fronteiras". No AM69 isso e um
# core preso sem produzir nada. O gate da Etapa 4 pede p95 abaixo de 100 ms.

def _map_message(width: int = 4, height: int = 3) -> OccupancyGrid:
    message = OccupancyGrid()
    message.info.width = width
    message.info.height = height
    message.info.resolution = 0.05
    message.info.origin.orientation.w = 1.0
    message.data = [0] * (width * height)
    return message


@pytest.fixture
def selecting(node, monkeypatch):
    """Um nó pronto para selecionar, com a extração contada em vez de corrida."""
    calls: list[int] = []

    def counted(grid, **kwargs):
        calls.append(1)
        return []

    monkeypatch.setattr(
        'demo_navigation.maze_explorer.extract_frontiers', counted)
    node._robot_pose = lambda: (0.0, 0.0)
    node._on_map(_map_message())
    node._state = 'selecting'
    node.extract_calls = calls
    return node


def test_selection_is_not_recomputed_while_map_and_blacklist_stand(selecting):
    """Sem mapa novo a extração daria o mesmo resultado -- e custa um core."""
    selecting._begin_selection()
    for _ in range(5):
        selecting._begin_selection()

    assert len(selecting.extract_calls) == 1
    assert selecting._selection_cycle == 1


def test_a_new_map_invalidates_the_selection_cache(selecting):
    """Mapa novo é informação nova: aí sim vale reextrair."""
    selecting._begin_selection()
    selecting._on_map(_map_message())
    selecting._begin_selection()

    assert len(selecting.extract_calls) == 2


def test_a_new_blacklist_entry_invalidates_the_selection_cache(selecting):
    """A fronteira reprovada muda o resultado mesmo com o mapa parado."""
    selecting._begin_selection()
    selecting._blacklist.append((1.0, 1.0))
    selecting._begin_selection()

    assert len(selecting.extract_calls) == 2


def test_restarting_the_run_invalidates_the_selection_cache(selecting):
    """
    Iniciar ou cancelar a busca tem de forçar extração.

    A época entra na chave por isso: sem ela, um `start` logo após um `cancel`,
    com o mesmo mapa e a blacklist já limpa, herdaria o cache da corrida
    anterior e o explorador ficaria parado esperando um mapa novo.
    """
    selecting._begin_selection()
    selecting._epoch += 1
    selecting._begin_selection()

    assert len(selecting.extract_calls) == 2


def test_a_map_update_while_navigating_does_not_replace_the_goal(selecting):
    """Reagir a /map em voo faria o robô abandonar a fronteira a cada mapa."""
    goal = Frontier(x=2.0, y=3.0, cells=12, information_gain_m=1.0)
    selecting._current = goal
    selecting._state = 'navigating'
    selecting._started_s = selecting._now_s()
    selecting._goal_started_s = selecting._now_s()

    selecting._on_map(_map_message())
    selecting._tick()

    assert selecting._current == goal
    assert selecting._state == 'navigating'
    assert selecting.extract_calls == []


def test_status_carries_the_cost_of_the_search(selecting):
    """Sem estes campos não há como provar o gate de CPU da Etapa 4."""
    selecting._begin_selection()
    selecting._publish_status()
    payload = selecting.published[-1]

    for field in ('frontier_extract_ms', 'frontier_cells', 'frontier_clusters',
                  'candidates_checked', 'path_requests', 'selection_cycle'):
        assert field in payload, field
    assert payload['selection_cycle'] == 1


def test_extraction_is_timed_on_a_monotonic_clock(selecting, monkeypatch):
    """
    O tempo de extração NÃO pode sair de `/clock`.

    Sob `use_sim_time` o relógio de simulação pausa, salta e corre fora do tempo
    real -- os três já observados neste projeto. O número que se quer aqui é CPU
    gasta de verdade, e ele só existe no relógio monotônico.
    """
    ticks = iter([100.0, 100.25])
    monkeypatch.setattr(
        'demo_navigation.maze_explorer.time.monotonic', lambda: next(ticks))

    selecting._begin_selection()

    assert selecting._frontier_extract_ms == pytest.approx(250.0)
