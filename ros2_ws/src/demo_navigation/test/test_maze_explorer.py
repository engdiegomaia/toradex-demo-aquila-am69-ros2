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
from demo_navigation import maze_explorer as maze_explorer_module
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
        # Nav2 action servers populate `.result` even for an aborted goal
        # (error_code/error_msg plus an empty path) -- `None` here would be a
        # test-double gap, not a real possibility, so default to the shape a
        # real ComputePathToPose result actually has.
        self.result = SimpleNamespace(
            error_code=0, error_msg='', path=SimpleNamespace(poses=[]))


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
    node._candidate_index = 0
    node._candidate_alt_index = 1
    node._best = None

    node._on_path_result(_Future(_Wrapped(GoalStatus.STATUS_ABORTED)),
                         node._epoch, frontier, (frontier.x, frontier.y))

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
    node._candidate_alt_index = 1
    epoch = node._epoch

    node._on_path_result(_Future(_Wrapped(GoalStatus.STATUS_ABORTED)),
                         node._epoch, frontier, (frontier.x, frontier.y))

    assert node._epoch == epoch


def test_a_frontier_the_planner_accepts_is_not_retired(node) -> None:
    """A guarda nao pode aposentar o caminho feliz junto."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    frontier = Frontier(x=2.0, y=2.0, cells=10, information_gain_m=0.5)
    node._candidates = [frontier]
    node._candidate_index = 1

    path = SimpleNamespace(path=SimpleNamespace(poses=[]),
                           error_code=0, error_msg='')
    wrapped = _Wrapped(GoalStatus.STATUS_SUCCEEDED)
    wrapped.result = path
    node._candidate_index = 0
    node._on_path_result(_Future(wrapped), node._epoch, frontier,
                         (frontier.x, frontier.y))

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
    node._candidate_index = 0
    node._candidate_alt_index = 1

    node._on_path_result(_Future(_Wrapped(GoalStatus.STATUS_ABORTED)),
                         node._epoch, far, (far.x, far.y))
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


def test_the_goal_timeout_is_never_raised_again(node) -> None:
    """
    Trava um experimento REPROVADO para que ninguem o repita.

    A rodada 2 expirou tres metas distantes em exatamente 90,0 s, o que le como
    "o teto e curto demais". A rodada 3 subiu para 180 s e piorou tudo: 4,26 m
    contra 21,93 m, 3746 celulas contra 8915, razao de trabalho 8,6% contra
    41,7%, e nenhuma deteccao do marcador -- porque travou 180 s numa meta a
    0,4 m do robo. O teto corta travamento, nao travessia lenta.

    Este teste nasceu como `== 90.0` para barrar aquela subida. O limite REAL
    que ele defende e o teto: baixar anda no mesmo sentido do que a rodada 3
    mediu. O piso fica em
    `test_goal_timeout_is_sized_from_the_measured_goal_durations`, que usa a
    distribuicao de duracoes de R5; os dois juntos prendem o valor.

    Evidencia: docs/results/ml35-f5-exploration-r3.md.
    """
    assert float(node.get_parameter('goal_timeout_s').value) <= 90.0, (
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


def _run_selection(node, monkeypatch, frontiers, robot=(0.0, 0.0, 0.0)):
    """
    Roda `_begin_selection` sobre um conjunto fixo, sem TF, grid nem Nav2.

    O que esta sob teste e a filtragem, nao a extracao nem o despacho, entao os
    tres colaboradores externos saem do caminho.
    """
    node._start(None, trigger(node))
    node._state = 'selecting'
    node._map_seq += 1
    monkeypatch.setattr(node, '_grid', lambda: object())
    monkeypatch.setattr(node, '_robot_pose', lambda: robot)
    monkeypatch.setattr(node, '_validate_next', lambda: None)
    monkeypatch.setattr(maze_explorer_module, 'extract_frontiers',
                        lambda grid, **_kwargs: list(frontiers))
    node._begin_selection()


def test_a_frontier_inside_the_goal_tolerance_is_never_dispatched(
        node, monkeypatch) -> None:
    """
    O modo de falha da rodada 4, atacado na causa.

    `xy_goal_tolerance` do Nav2 e 0,25 m. Uma fronteira a 0,20 m do robo faz o
    Nav2 devolver sucesso sem que nada se mova; a selecao volta ao mesmo ponto
    e o ciclo se repete. Na rodada 4 isso rodou 565 vezes em 580 s com o robo
    dentro de uma caixa de 11 mm x 25 mm.

    Evidencia: docs/results/ml35-f5-exploration-r4.md.
    """
    near = Frontier(x=0.20, y=0.0, cells=8, information_gain_m=0.4)
    far = Frontier(x=2.0, y=0.0, cells=90, information_gain_m=4.5)
    _run_selection(node, monkeypatch, [near, far])

    assert near not in node._candidates
    assert node._near_skipped == 1


def test_the_next_frontier_out_is_selected_instead(node, monkeypatch) -> None:
    """Descartar a de perto tem de deixar a exploracao seguir, nao parar."""
    near = Frontier(x=0.20, y=0.0, cells=8, information_gain_m=0.4)
    far = Frontier(x=2.0, y=0.0, cells=90, information_gain_m=4.5)
    _run_selection(node, monkeypatch, [near, far])

    assert node._candidates == [far]
    assert node._frontier_count == 1


def test_a_frontier_exactly_at_the_limit_stays_eligible(
        node, monkeypatch) -> None:
    """O limite e inclusivo; senao o corte vira uma faixa morta ambigua."""
    limit = float(node.get_parameter('min_frontier_distance_m').value)
    edge = Frontier(x=limit, y=0.0, cells=20, information_gain_m=1.0)
    _run_selection(node, monkeypatch, [edge])

    assert node._candidates == [edge]
    assert node._near_skipped == 0


def test_skipping_a_near_frontier_never_suppresses_it(
        node, monkeypatch) -> None:
    """
    O corte e relativo a pose ATUAL, nunca uma anotacao permanente.

    Bastou uma lista permanente demais para matar a rodada 1. Andar alguns
    centimetros tem de devolver a fronteira a disputa sozinho.
    """
    near = Frontier(x=0.20, y=0.0, cells=8, information_gain_m=0.4)
    _run_selection(node, monkeypatch, [near])

    assert node._blacklist == []
    assert node._refused == []
    assert node._timed_out == []

    _run_selection(node, monkeypatch, [near], robot=(-1.0, 0.0, 0.0))
    assert node._candidates == [near]


def test_a_selection_with_only_near_frontiers_counts_as_no_progress(
        node, monkeypatch) -> None:
    """
    Ficar sem candidatos por proximidade e ausencia de progresso.

    Se o ciclo nao contasse, o robo cercado so por fronteiras dentro da
    tolerancia ficaria em `selecting` calado ate o prazo total.
    """
    near = Frontier(x=0.20, y=0.0, cells=8, information_gain_m=0.4)
    _run_selection(node, monkeypatch, [near])

    assert node._candidates == []
    assert node._barren_cycles == 1


def test_status_reports_how_many_near_frontiers_were_skipped(
        node, monkeypatch) -> None:
    """Sem a metrica no status, o descarte e invisivel na analise da corrida."""
    near = Frontier(x=0.20, y=0.0, cells=8, information_gain_m=0.4)
    far = Frontier(x=2.0, y=0.0, cells=90, information_gain_m=4.5)
    _run_selection(node, monkeypatch, [near, far])
    node._publish_status()

    assert node.published[-1]['near_frontiers_skipped'] == 1


def _pose_at(x: float, y: float) -> PoseStamped:
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    return pose


def test_setback_point_walks_back_from_the_path_end() -> None:
    """A straight 1 m path recessed by 0.4 m lands 0.6 m from the start."""
    path = [_pose_at(0.0, 0.0), _pose_at(1.0, 0.0)]
    point = maze_explorer_module._setback_point(path, setback_m=0.4)
    assert point == pytest.approx((0.6, 0.0))


def test_setback_point_discarded_when_it_would_fall_inside_goal_tolerance(
) -> None:
    """
    R11 (29/08): the robot never moved, for minutes, on an unchanging map.

    A short path (candidate close to the robot) recessed by the default
    0.40 m setback landed ~0.23 m from the robot's own current pose --
    inside Nav2's 0.25 m `xy_goal_tolerance`. `SimpleGoalChecker` called the
    goal reached without the robot moving at all, so the map never grew and
    the identical frontier kept getting re-selected forever. The fix is a
    `min_travel_m` floor: a setback point this close to the path's start is
    discarded (`None`) instead of returned, so the caller falls back to the
    original, already-validated endpoint.
    """
    path = [_pose_at(0.0, 0.0), _pose_at(0.6, 0.0)]
    point = maze_explorer_module._setback_point(
        path, setback_m=0.4, min_travel_m=0.35)
    assert point is None


def test_setback_point_kept_when_it_clears_the_travel_floor() -> None:
    """The same geometry with a floor it actually clears is kept, not dropped."""
    path = [_pose_at(0.0, 0.0), _pose_at(0.6, 0.0)]
    point = maze_explorer_module._setback_point(
        path, setback_m=0.4, min_travel_m=0.15)
    assert point == pytest.approx((0.2, 0.0))


def test_setback_point_on_a_path_shorter_than_the_setback_is_discarded(
) -> None:
    """
    A too-short path is discarded, not collapsed to the robot's own pose.

    It used to fall back to the path's own start point -- the robot's
    current pose, an even more degenerate target than the R11 stall.
    `min_travel_m` defaults to 0.0, but the start point is by definition
    zero distance from itself, so it is always discarded.
    """
    path = [_pose_at(0.0, 0.0), _pose_at(0.1, 0.0)]
    assert maze_explorer_module._setback_point(path, setback_m=0.4) is None


def test_setback_point_handles_empty_and_single_pose_paths() -> None:
    assert maze_explorer_module._setback_point([], setback_m=0.4) is None
    single = [_pose_at(2.0, 3.0)]
    assert maze_explorer_module._setback_point(
        single, setback_m=0.4) == pytest.approx((2.0, 3.0))
    assert maze_explorer_module._setback_point(
        single, setback_m=0.4, min_travel_m=5.0) is None


def test_r4a_leaves_the_r4_timeout_policy_alone(node) -> None:
    """Uma variavel por rodada: a permanencia do timeout nao se mexe aqui."""
    frontier = _timed_out_frontier(node)
    assert node._blacklist == []
    assert (frontier.x, frontier.y) in node._timed_out


def _run_provisionally_suppressed_selection(
        node, monkeypatch, frontiers, *, hard=False) -> None:
    """Run one selection where every real frontier starts suppressed."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    node._map_seq += 1
    monkeypatch.setattr(node, '_grid', lambda: object())
    monkeypatch.setattr(node, '_robot_pose', lambda: (0.0, 0.0, 0.0))
    monkeypatch.setattr(node, '_validate_next', lambda: None)

    def fake_extract(grid, stats=None, **_kwargs):
        # Real `extract_frontiers` always reports `raw_clusters` -- a
        # provisional/hard dead end here means a cluster WAS observed and
        # then suppressed, not that none existed (see
        # `test_a_zero_raw_cluster_selection_starts_an_observation_recovery`
        # for that other case).
        if stats is not None:
            stats['raw_clusters'] = len(frontiers)
        return list(frontiers)
    monkeypatch.setattr(
        maze_explorer_module, 'extract_frontiers', fake_extract)
    targets = node._blacklist if hard else node._refused
    targets.extend((item.x, item.y) for item in frontiers)
    node._begin_selection()


def test_all_provisional_suppressions_are_released_once(
        node, monkeypatch) -> None:
    """A provisional-only dead end gets one chance to explore again."""
    frontiers = [
        Frontier(x=1.0, y=0.0, cells=20, information_gain_m=1.0),
        Frontier(x=2.0, y=0.0, cells=30, information_gain_m=1.5),
    ]
    _run_provisionally_suppressed_selection(node, monkeypatch, frontiers)

    assert node._refused == []
    assert node._timed_out == []
    assert node._blacklist == []
    assert node._candidates == frontiers
    assert node._provisional_recoveries == 1
    assert node._provisional_recovery_used is True
    assert node._barren_cycles == 0


def test_provisional_recovery_cannot_repeat_without_progress(
        node, monkeypatch) -> None:
    """Repeated refusal after recovery counts barren instead of livelocking."""
    frontier = Frontier(x=1.0, y=0.0, cells=20, information_gain_m=1.0)
    _run_provisionally_suppressed_selection(node, monkeypatch, [frontier])

    node._refused.append((frontier.x, frontier.y))
    node._map_seq += 1
    node._begin_selection()

    assert node._refused == [(frontier.x, frontier.y)]
    assert node._candidates == []
    assert node._provisional_recoveries == 1
    assert node._barren_cycles == 1


def test_hard_blacklist_is_never_released_by_deadlock_recovery(
        node, monkeypatch) -> None:
    """Recovery must not resurrect a frontier with an execution failure."""
    frontier = Frontier(x=1.0, y=0.0, cells=20, information_gain_m=1.0)
    _run_provisionally_suppressed_selection(
        node, monkeypatch, [frontier], hard=True)

    assert node._blacklist == [(frontier.x, frontier.y)]
    assert node._candidates == []
    assert node._provisional_recoveries == 0
    assert node._barren_cycles == 1


def test_successful_motion_rearms_provisional_recovery(node) -> None:
    """Only a reached exploration goal permits another recovery attempt."""
    node._provisional_recovery_used = True
    node._state = 'navigating'
    node._current = Frontier(x=1.0, y=0.0, cells=20, information_gain_m=1.0)

    node._on_nav_result(_Future(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED)), node._epoch, True)

    assert node._provisional_recovery_used is False


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


def test_a_zero_raw_cluster_selection_starts_an_observation_recovery(
        node, monkeypatch) -> None:
    """
    Nenhum cluster bruto e diferente de cluster filtrado.

    Girar pode revelar geometria nova quando o SLAM simplesmente nao viu
    nenhuma fronteira ainda; nao adianta nada quando a fronteira existe e foi
    suprimida (ver `test_all_provisional_suppressions_are_released_once`,
    onde `raw_clusters` e nao-zero e o caminho e outro).
    """
    node._start(None, trigger(node))
    node._state = 'selecting'
    node._map_seq += 1
    monkeypatch.setattr(node, '_grid', lambda: object())
    monkeypatch.setattr(node, '_robot_pose', lambda: (0.0, 0.0, 0.0))

    def fake_extract(grid, stats=None, **_kwargs):
        if stats is not None:
            stats['raw_clusters'] = 0
        return []
    monkeypatch.setattr(
        maze_explorer_module, 'extract_frontiers', fake_extract)

    spins: list[int] = []
    monkeypatch.setattr(
        node, '_start_observation_recovery',
        lambda: spins.append(node._map_seq))

    node._begin_selection()

    assert spins == [node._map_seq]
    assert node._recovery_map_seq == node._map_seq
    assert node._message == 'nenhum cluster de fronteira bruto'
    assert node._barren_cycles == 0, (
        'a varredura ainda nao aconteceu; nao pode contar como ciclo baldio')


def test_a_repeated_zero_raw_cluster_map_does_not_spin_twice(
        node, monkeypatch) -> None:
    """Uma tentativa por versao de mapa -- ver o comentario em `__init__`."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    node._map_seq += 1
    monkeypatch.setattr(node, '_grid', lambda: object())
    monkeypatch.setattr(node, '_robot_pose', lambda: (0.0, 0.0, 0.0))

    def fake_extract(grid, stats=None, **_kwargs):
        if stats is not None:
            stats['raw_clusters'] = 0
        return []
    monkeypatch.setattr(
        maze_explorer_module, 'extract_frontiers', fake_extract)

    spins: list[int] = []
    monkeypatch.setattr(
        node, '_start_observation_recovery',
        lambda: spins.append(node._map_seq))

    node._begin_selection()
    # Forca nova extracao sem mapa novo, mesma tecnica das outras
    # invalidacoes de cache neste arquivo.
    node._epoch += 1
    node._begin_selection()

    assert spins == [node._map_seq], (
        'a segunda tentativa caiu sobre o MESMO mapa; nao pode repetir')


class _PendingSend:
    """Como `send_goal_async` devolve de verdade: Future ainda em voo."""

    def __init__(self) -> None:
        self.callback = None

    def add_done_callback(self, cb) -> None:
        self.callback = cb


class _AutoFireFuture:
    """Uma Future ja resolvida: dispara o callback registrado na hora."""

    def __init__(self, value) -> None:
        self._value = value

    def result(self):
        return self._value

    def add_done_callback(self, cb) -> None:
        cb(self)


class _SpinHandle:
    """O handle que `Spin.send_goal_async` aceita entrega ao callback."""

    def __init__(self, accepted: bool,
                 status: int = GoalStatus.STATUS_SUCCEEDED) -> None:
        self.accepted = accepted
        self._status = status

    def get_result_async(self):
        return _AutoFireFuture(SimpleNamespace(status=self._status))

    def cancel_goal_async(self) -> None:
        pass


def test_observation_recovery_round_trip_clears_pending_and_counts_barren(
        node, monkeypatch) -> None:
    """A varredura em si nao decide nada; so o proximo mapa novo decide."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    pending = _PendingSend()
    monkeypatch.setattr(
        node._spin_client, 'send_goal_async', lambda goal: pending)

    node._start_observation_recovery()
    assert node._recovery_pending is True
    assert node._recovery_attempts == 1

    pending.callback(_Future(_SpinHandle(accepted=True)))

    assert node._recovery_pending is False
    assert node._recovery_handle is None
    assert node._barren_cycles == 1


def test_a_refused_observation_recovery_counts_barren_at_once(
        node, monkeypatch) -> None:
    """O behavior_server pode recusar o giro (ex.: colisao iminente)."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    pending = _PendingSend()
    monkeypatch.setattr(
        node._spin_client, 'send_goal_async', lambda goal: pending)

    node._start_observation_recovery()
    pending.callback(_Future(_SpinHandle(accepted=False)))

    assert node._recovery_pending is False
    assert node._barren_cycles == 1


def test_cancel_stops_a_pending_observation_recovery(
        node, monkeypatch) -> None:
    """Cancelar a busca tem de parar o giro em voo, nao so a navegacao."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    pending = _PendingSend()
    monkeypatch.setattr(
        node._spin_client, 'send_goal_async', lambda goal: pending)
    node._start_observation_recovery()
    handle = _SpinHandle(accepted=True)
    cancelled: list[bool] = []
    handle.cancel_goal_async = lambda: cancelled.append(True)
    node._recovery_handle = handle
    node._recovery_pending = True

    node._cancel(None, trigger(node))

    assert cancelled == [True]
    assert node._recovery_pending is False
    assert node._recovery_handle is None


def test_a_stale_recovery_callback_is_ignored_after_cancel(
        node, monkeypatch) -> None:
    """Callback de uma rodada ja cancelada nao pode contar para a nova."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    pending = _PendingSend()
    monkeypatch.setattr(
        node._spin_client, 'send_goal_async', lambda goal: pending)
    node._start_observation_recovery()
    node._cancel(None, trigger(node))
    barren_before = node._barren_cycles

    pending.callback(_Future(_SpinHandle(accepted=True)))

    assert node._barren_cycles == barren_before


class _StubGoalHandle:
    """Handle de meta minimo -- so o suficiente para `_cancel_goal` funcionar."""

    def cancel_goal_async(self):
        return None


def _armed_for_navigating(node, clock) -> None:
    """Coloca `node` num estado 'navigating' com meta ja aceita por Nav2."""
    node._start(None, trigger(node))
    node._state = 'navigating'
    node._current = Frontier(x=5.0, y=5.0, cells=10, information_gain_m=1.0)
    node._goal_started_s = clock['t']
    node._goal_handle = _StubGoalHandle()  # meta ja aceita -- ver _on_nav_accepted


def test_navigation_watchdog_fires_after_the_stall_window(node) -> None:
    """Comando despachado, robo parado -- corta antes do prazo de 45 s."""
    clock = {'t': 0.0}
    node._now_s = lambda: clock['t']
    _armed_for_navigating(node, clock)
    node._robot_pose = lambda: (0.0, 0.0, 0.0)
    window = float(node.get_parameter('stall_window_s').value)

    node._tick()  # arma o relogio do vigia na primeira leitura
    assert node._state == 'navigating'

    clock['t'] = window - 1.0
    node._tick()
    assert node._state == 'navigating', 'ainda dentro da janela'

    clock['t'] = window + 1.0
    node._tick()
    assert node._state == 'selecting'
    assert 'vigia de movimento' in node._message
    assert (5.0, 5.0) in node._timed_out


def test_navigation_watchdog_resets_on_real_displacement(node) -> None:
    """Deslocamento real reinicia a janela -- nao e prazo fixo desde a meta."""
    clock = {'t': 0.0}
    node._now_s = lambda: clock['t']
    _armed_for_navigating(node, clock)
    window = float(node.get_parameter('stall_window_s').value)
    pose = {'p': (0.0, 0.0, 0.0)}
    node._robot_pose = lambda: pose['p']

    node._tick()
    clock['t'] = window - 1.0
    pose['p'] = (0.2, 0.0, 0.0)  # acima de stall_move_threshold_m (0.05 m)
    node._tick()
    assert node._state == 'navigating'

    clock['t'] = (window - 1.0) + (window - 1.0)
    node._tick()
    assert node._state == 'navigating', (
        'o relogio reiniciou no deslocamento; ainda nao pode ter estourado')


def test_navigation_watchdog_does_not_fire_on_legitimate_rotation(node) -> None:
    """
    R15a: girar em pé para encarar um corredor nao e travamento.

    So checar xy classificaria esta rotacao legitima (sem deslocamento) como
    o robo parado, cancelando uma meta que estava progredindo de verdade.
    """
    clock = {'t': 0.0}
    node._now_s = lambda: clock['t']
    _armed_for_navigating(node, clock)
    window = float(node.get_parameter('stall_window_s').value)
    pose = {'p': (0.0, 0.0, 0.0)}
    node._robot_pose = lambda: pose['p']

    node._tick()
    clock['t'] = window - 1.0
    pose['p'] = (0.0, 0.0, 0.3)  # gira 0.3 rad, xy parado
    node._tick()
    assert node._state == 'navigating'

    clock['t'] = (window - 1.0) + (window - 1.0)
    node._tick()
    assert node._state == 'navigating', (
        'rotacao real acima do limiar tem de reiniciar o relogio tambem')


def test_navigation_watchdog_handles_the_minus_pi_pi_wraparound(node) -> None:
    """Guinada cruzando de +pi para -pi e uma rotacao pequena, nao enorme."""
    clock = {'t': 0.0}
    node._now_s = lambda: clock['t']
    _armed_for_navigating(node, clock)
    window = float(node.get_parameter('stall_window_s').value)
    pose = {'p': (0.0, 0.0, math.pi - 0.01)}
    node._robot_pose = lambda: pose['p']

    node._tick()  # arma a baseline em (pi - 0.01)
    clock['t'] = window - 1.0
    pose['p'] = (0.0, 0.0, -math.pi + 0.01)  # cruzou o wraparound, diff real = 0.02
    node._tick()
    clock['t'] = window + 1.0
    node._tick()
    assert node._state == 'selecting', (
        'diff real de guinada (0.02 rad) fica abaixo do limiar -- '
        'sem o wraparound normalizado o vigia calcularia ~2*pi e nunca dispararia')
    assert 'vigia de movimento' in node._message


def test_navigation_watchdog_does_not_fire_before_goal_acceptance(node) -> None:
    """
    R15a: sem meta aceita (`_goal_handle is None`), nao ha o que travar.

    Antes desta correcao o relogio armava no despacho da meta (`_send_navigation`),
    contando a latencia de resposta do Nav2 -- ainda em voo -- como imobilidade.
    """
    clock = {'t': 0.0}
    node._now_s = lambda: clock['t']
    node._start(None, trigger(node))
    node._state = 'navigating'
    node._current = Frontier(x=5.0, y=5.0, cells=10, information_gain_m=1.0)
    node._goal_started_s = clock['t']
    node._goal_handle = None  # Nav2 ainda nao aceitou
    node._robot_pose = lambda: (0.0, 0.0, 0.0)
    window = float(node.get_parameter('stall_window_s').value)

    node._tick()
    clock['t'] = window + 1.0
    node._tick()
    assert node._state == 'navigating', (
        'sem meta aceita o vigia nao pode disparar')
    assert 'vigia de movimento' not in node._message


def test_navigation_watchdog_fires_when_accepted_goal_is_truly_still(node) -> None:
    """Meta aceita (`_goal_handle` setado) e robo genuinamente parado -- dispara."""
    clock = {'t': 0.0}
    node._now_s = lambda: clock['t']
    _armed_for_navigating(node, clock)
    node._robot_pose = lambda: (1.0, 2.0, 0.5)  # pose fixa, sem xy nem guinada
    window = float(node.get_parameter('stall_window_s').value)

    node._tick()
    clock['t'] = window + 1.0
    node._tick()
    assert node._state == 'selecting'
    assert 'vigia de movimento' in node._message


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
    node._exit_candidate_pose_map = (5.0, 5.0)
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
    assert node._exit_candidate_pose_map is None
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

    def counted(grid, stats=None, **kwargs):
        calls.append(1)
        # Real `extract_frontiers` sempre relata `raw_clusters`. Um valor
        # nao-zero aqui mantem estes testes de CACHE isolados do caminho de
        # recuperacao por varredura (R15), que tem os proprios testes
        # dedicados para o caso raw_clusters == 0.
        if stats is not None:
            stats['raw_clusters'] = 1
        return []

    monkeypatch.setattr(
        'demo_navigation.maze_explorer.extract_frontiers', counted)
    node._robot_pose = lambda: (0.0, 0.0, 0.0)
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
    """
    Mapa novo é informação nova: aí sim vale reextrair.

    O conteúdo tem de mudar de fato (R15) -- repetir a mesma grade não conta
    como mapa novo, ver `test_map_republication_does_not_invalidate_the_cache`
    logo abaixo.
    """
    changed = _map_message()
    changed.data[0] = 100
    selecting._begin_selection()
    selecting._on_map(changed)
    selecting._begin_selection()

    assert len(selecting.extract_calls) == 2


def test_map_republication_does_not_invalidate_the_cache(selecting):
    """
    `slam_toolbox` republica `/map` mesmo sem mudanca -- isso nao e mapa novo.

    Complementa o teste acima: aqui o CONTEUDO e identico ao que a fixture
    `selecting` ja usou para popular o cache, entao reextrair de novo custaria
    um core por nada.
    """
    selecting._begin_selection()
    selecting._on_map(_map_message())
    selecting._begin_selection()

    assert len(selecting.extract_calls) == 1


def test_on_map_only_advances_map_seq_on_real_content_change(node) -> None:
    """
    R15: `_map_seq` e contagem de CONTEUDO, nao de mensagem.

    Antes desta correcao, republicar um mapa identico ainda incrementava
    `_map_seq`, o que fazia `self._map_seq > self._last_provisional_map_seq`
    em `_begin_selection` liberar uma supressao provisoria sem nenhuma
    observacao nova ter chegado.
    """
    node._on_map(_map_message())
    seq_after_first = node._map_seq

    for _ in range(5):
        node._on_map(_map_message())
    assert node._map_seq == seq_after_first, (
        'republicacao identica nao pode avancar _map_seq')

    changed = _map_message()
    changed.data[0] = 100
    node._on_map(changed)
    assert node._map_seq == seq_after_first + 1, (
        'conteudo genuinamente novo tem de avancar _map_seq'
    )


def test_a_geometry_only_change_advances_map_seq(node) -> None:
    """
    R15a: um re-ancoramento do SLAM muda geometria, mesmo com celulas iguais.

    R15 hasheava so `message.data` -- um mapa com a mesma grade de celulas
    mas origem ou resolucao diferentes (por exemplo, apos um re-ancoramento)
    seria tratado como republicacao identica, perdendo a mudanca real.
    """
    node._on_map(_map_message())
    seq_after_first = node._map_seq

    moved_origin = _map_message()
    moved_origin.info.origin.position.x = 1.0
    node._on_map(moved_origin)
    assert node._map_seq == seq_after_first + 1, (
        'origem diferente, mesmas celulas, ainda e um mapa novo'
    )

    seq_after_origin = node._map_seq
    different_resolution = _map_message()
    different_resolution.info.origin.position.x = 1.0
    different_resolution.info.resolution = 0.10
    node._on_map(different_resolution)
    assert node._map_seq == seq_after_origin + 1, (
        'resolucao diferente, mesmas celulas, ainda e um mapa novo'
    )


def test_map_republication_does_not_release_a_provisional_recovery(
        node, monkeypatch) -> None:
    """
    O teste que o plano original pediu por nome.

    Mesmo conteudo, nova mensagem, nenhuma recuperacao.

    Sem a correcao de `_map_seq`, republicar `/map` 5 vezes (mesmo conteudo)
    bastava para `self._map_seq > self._last_provisional_map_seq` ficar
    verdadeiro e liberar a supressao provisoria -- exatamente o livelock que
    `_last_provisional_map_seq` foi criado para evitar, só que por mensagem
    em vez de por conteúdo.
    """
    frontier = Frontier(x=1.0, y=0.0, cells=20, information_gain_m=1.0)
    _run_provisionally_suppressed_selection(node, monkeypatch, [frontier])
    assert node._provisional_recovery_used is True

    # Uma observacao real de mapa, para sair do estado "nunca vi /map" do
    # rastreador de conteudo -- sem isto a PRIMEIRA chamada de `_on_map` do
    # teste sempre contaria como mudanca, mascarando o que se quer medir.
    node._on_map(_map_message())

    # Rearma a recuperacao (como uma chegada real faria, ver
    # `test_successful_motion_rearms_provisional_recovery`) e simula uma
    # NOVA recusa na mesma versao de mapa -- a precondicao real que
    # `_last_provisional_map_seq` existe para proteger.
    node._provisional_recovery_used = False
    node._refused.append((frontier.x, frontier.y))
    node._last_provisional_map_seq = node._map_seq

    for _ in range(5):
        node._on_map(_map_message())  # mesmo conteudo, mensagens novas
    node._begin_selection()

    assert node._refused == [(frontier.x, frontier.y)], (
        'republicacao identica nao pode liberar a supressao provisoria')
    assert node._candidates == []


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


def test_homing_entry_distance_is_latched_for_the_gate_measurement(
        node, monkeypatch) -> None:
    """
    O portao de distancia do homing precisa de um numero medido, nao de um chute.

    O status sai a 2 Hz e a entrada em `homing_exit` e instantanea, entao uma
    amostragem periodica perde o instante. A distancia da entrada e travada no
    momento da transicao; a corrente continua sendo amostrada.
    """
    node._start(None, trigger(node))
    node._state = 'selecting'
    monkeypatch.setattr(node, '_robot_pose', lambda: (0.0, 0.0, 0.0))
    monkeypatch.setattr(node, '_cancel_goal', lambda: None)
    for _ in range(int(node.get_parameter('homing_confirm_observations').value)):
        _marker_at(node, monkeypatch, 3.0)

    assert node._state == 'homing_exit'
    assert node._homing_entry_distance_m == 3.0
    assert node._homing_entries == 1
    assert node.published[-1]['homing_entry_distance_m'] == 3.0
    assert node.published[-1]['marker_distance_m'] == 3.0


def test_homing_entry_distance_is_not_overwritten_while_homing(
        node, monkeypatch) -> None:
    """A later partial view cannot move the target accepted by the gate."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    monkeypatch.setattr(node, '_robot_pose', lambda: (0.0, 0.0, 0.0))
    monkeypatch.setattr(node, '_cancel_goal', lambda: None)
    for _ in range(int(node.get_parameter('homing_confirm_observations').value)):
        _marker_at(node, monkeypatch, 3.0)

    accepted = node._exit_pose_map
    # A new observation swings far while homing. It remains observable as the
    # raw candidate, but cannot redirect the active approach.
    monkeypatch.setattr(node, '_robot_pose', lambda: (2.4, 0.8, 0.0))
    pose = PoseStamped()
    pose.header.frame_id = 'front_camera'
    pose.header.stamp.nanosec = 99
    pose.pose.position.x = 7.0
    node._on_exit_pose(pose)
    node._tick()

    assert node._homing_entries == 1
    assert node._homing_entry_distance_m == 3.0
    assert node._exit_pose_map == accepted
    assert node._exit_candidate_pose_map == (7.0, 0.0)
    assert node.published[-1]['marker_distance_m'] == pytest.approx(4.67)


def test_marker_distance_is_none_without_a_marker_or_a_pose(
        node, monkeypatch) -> None:
    """Sem marcador ou sem TF a medida e ausente, nunca zero."""
    monkeypatch.setattr(node, '_robot_pose', lambda: (0.0, 0.0, 0.0))
    assert node._distance_to_exit() is None

    node._exit_pose_map = (1.0, 1.0)
    monkeypatch.setattr(node, '_robot_pose', lambda: None)
    assert node._distance_to_exit() is None


def test_a_far_marker_is_recorded_but_does_not_capture_the_run() -> None:
    """
    Substitui `test_the_measurement_round_adds_no_homing_gate`; a medida existe.

    R7 mediu o erro de alcance por faixa contra o marcador do SDF em
    (-4,90, -2,60), 131 amostras:

        faixa estimada   razao est/real   erro absoluto medio
        0-2 m                 0,579              1,28 m
        2-3 m                 0,876              0,60 m
        3-4 m                 1,062              0,41 m
        4-6 m                 1,316              1,14 m
        acima de 6 m          1,813              3,08 m

    R7 entrou em homing a 7,35 m -- a pior faixa -- e como a aproximacao agora
    persiste, aquela observacao unica prendeu a corrida por 520 s em
    `homing_exit` sem nunca chegar. O portao e um MAXIMO: acima dele, registra
    o marcador e continua explorando.
    """


def _marker_at(node, monkeypatch, distance_m, stamp=None):
    """Deliver one distinct, identity-transformed camera observation."""
    monkeypatch.setattr(node, '_robot_pose', lambda: (0.0, 0.0, 0.0))
    monkeypatch.setattr(node, '_cancel_goal', lambda: None)
    transform = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=0.0, y=0.0),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
    monkeypatch.setattr(
        node._tf_buffer, 'lookup_transform', lambda *args: transform)
    if stamp is None:
        stamp = getattr(node, '_test_marker_stamp', 0) + 1
        node._test_marker_stamp = stamp
    pose = PoseStamped()
    pose.header.frame_id = 'front_camera'
    pose.header.stamp.nanosec = stamp
    pose.pose.position.x = distance_m
    node._on_exit_pose(pose)
    return pose


def test_a_marker_beyond_the_gate_never_enters_homing(node, monkeypatch) -> None:
    """7,35 m foi o que prendeu R7 por 520 s. Registrar sim, comprometer nao."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    far = float(node.get_parameter('homing_max_distance_m').value) + 1.0
    _marker_at(node, monkeypatch, far)

    for _ in range(10):
        node._tick()

    assert node._state != 'homing_exit'
    assert node._homing_entries == 0
    assert node.published[-1]['marker_distance_m'] == round(far, 2)
    assert node.published[-1]['marker_far_ignored'] == 1


def test_entering_homing_needs_more_than_one_near_observation(
        node, monkeypatch) -> None:
    """
    Uma amostra unica nao decide: em R7 a estimativa oscilou de 1,27 a 7,94 m.

    A histerese exige `homing_confirm_observations` observacoes proximas
    seguidas antes de cancelar a exploracao.
    """
    node._start(None, trigger(node))
    node._state = 'selecting'
    needed = int(node.get_parameter('homing_confirm_observations').value)
    assert needed >= 2, 'sem histerese o portao nao filtra a oscilacao medida'
    _marker_at(node, monkeypatch, 3.0)
    for _ in range(10):
        node._tick()
        assert node._state != 'homing_exit'

    for _ in range(needed - 1):
        _marker_at(node, monkeypatch, 3.0)
    assert node._state == 'homing_exit'
    assert node._homing_entry_distance_m == 3.0


def test_duplicate_source_stamp_is_not_a_second_confirmation(
        node, monkeypatch) -> None:
    """Transport duplication of one camera frame cannot satisfy the gate."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    _marker_at(node, monkeypatch, 3.0, stamp=42)
    _marker_at(node, monkeypatch, 3.0, stamp=42)

    assert node._marker_observations == 1
    assert node._near_marker_streak == 1
    assert node._state == 'selecting'


def test_a_far_observation_resets_the_hysteresis(node, monkeypatch) -> None:
    """Perto-longe-perto nao pode somar como se fossem seguidas."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    far = float(node.get_parameter('homing_max_distance_m').value) + 1.0

    _marker_at(node, monkeypatch, 3.0)
    _marker_at(node, monkeypatch, far)
    _marker_at(node, monkeypatch, 3.0)

    assert node._state != 'homing_exit'


def test_the_gate_sits_in_the_band_where_the_range_was_measured_good(
        node) -> None:
    """Acima de 4 m o erro medio medido em R7 passa de 1,1 m."""
    assert 2.0 < float(
        node.get_parameter('homing_max_distance_m').value) <= 4.0


def _homing_ready(node, monkeypatch, sent):
    """Um no em `homing_exit`, sem meta em voo, com a saida travada a 3 m."""
    node._start(None, trigger(node))
    node._state = 'homing_exit'
    node._pending = False
    node._goal_handle = None
    monkeypatch.setattr(node, '_robot_pose', lambda: (0.0, 0.0, 0.0))
    monkeypatch.setattr(
        node, '_send_navigation',
        lambda frontier, exploration: sent.append((frontier, exploration)))
    node._exit_pose_map = (3.0, 0.0)
    node._exit_candidate_pose_map = (3.0, 0.0)
    node._exit_seen_s = node._now_s()
    return node


def test_homing_survives_a_briefly_occluded_marker(node, monkeypatch) -> None:
    """
    Perder o marcador de vista nao e perder a saida.

    `_exit_pose_map` e uma coordenada travada no frame do mapa. A linha de visada
    serve para APRENDER onde fica a saida, nao para navegar ate ela -- andar por
    um corredor de labirinto quebra a visada por construcao. Abandonar a cada
    oclusao e o que deixou o homing 0 de 11 em campo (R2, rodada observada, arm B).
    """
    sent: list = []
    _homing_ready(node, monkeypatch, sent)
    stale_s = float(node.get_parameter('marker_stale_s').value)
    node._exit_seen_s = node._now_s() - stale_s - 1.0

    node._tick()

    assert node._state == 'homing_exit'
    assert sent, 'o homing precisa continuar a aproximacao com a pose travada'
    assert node.published[-1]['marker_visible'] is False


def test_blind_approach_goes_to_the_exit_not_to_a_half_metre_hop(
        node, monkeypatch) -> None:
    """
    Sem marcador fresco nao ha por que re-mirar, entao o passo curto so custa tempo.

    Com a visada, o passo de `homing_step_m` reaproveita cada nova deteccao para
    corrigir a mira. As cegas isso vira uma sequencia de metas retas de 0,5 m que
    o planejador recusa quando ha parede no caminho; uma meta unica deixa o Nav2
    contornar.
    """
    sent: list = []
    _homing_ready(node, monkeypatch, sent)
    stop = float(node.get_parameter('marker_stop_distance_m').value)
    node._exit_seen_s = node._now_s() - float(
        node.get_parameter('marker_stale_s').value) - 1.0

    node._tick()

    assert sent[-1][1] is False, 'aproximacao nao e exploracao'
    assert sent[-1][0].x == pytest.approx(3.0 - stop)

    sent.clear()
    node._exit_seen_s = node._now_s()
    node._tick()
    assert sent[-1][0].x == pytest.approx(
        float(node.get_parameter('homing_step_m').value))


def test_homing_gives_up_after_the_persistence_budget(node, monkeypatch) -> None:
    """
    A perseguicao as cegas e limitada, senao volta o bug que o prazo de frescor evitava.

    O contrato antigo era "sem marcador fresco, desiste ja". O novo e "sem
    marcador fresco, insiste por `homing_persistence_s` e depois desiste" -- o
    explorador nunca persegue a ultima pose vista para sempre.
    """
    sent: list = []
    _homing_ready(node, monkeypatch, sent)
    persistence_s = float(node.get_parameter('homing_persistence_s').value)
    node._exit_seen_s = node._now_s() - persistence_s - 1.0

    node._tick()

    assert node._state == 'selecting'
    assert not sent, 'estourado o orcamento, nao se despacha mais aproximacao'
    assert node._homing_abandons == 1
    assert node.published[-1]['homing_abandons'] == 1


def test_persistence_budget_outlives_the_freshness_deadline(node) -> None:
    """Se o orcamento fosse menor que o frescor, a insistencia nunca aconteceria."""
    assert float(node.get_parameter('homing_persistence_s').value) > float(
        node.get_parameter('marker_stale_s').value)


def test_homing_abandons_is_published_and_reset_by_start(node) -> None:
    """O contador separa "desistiu da aproximacao" de "meta falhou no Nav2"."""
    node._publish_status()
    assert node.published[-1]['homing_abandons'] == 0
    node._homing_abandons = 4
    node._start(None, trigger(node))
    assert node._homing_abandons == 0


def test_goal_timeout_is_sized_from_the_measured_goal_durations(node) -> None:
    """
    O prazo por meta e um orcamento, nao uma folga: cada estouro custa o valor cheio.

    Medido na rodada R5 (arm B, 21 metas): as 12 metas BEM SUCEDIDAS levaram de
    6,1 s a 35,1 s, e as 3 que falharam gastaram exatamente 90,0 s cada -- 270 s
    de um orcamento de 600 s, 45%, sem sair do lugar. A corrida terminou a 1,08 m
    do marcador por falta de tempo.

    O prazo tem de cobrir a pior meta que deu certo com margem, e tres estouros
    nao podem comer metade do orcamento total.
    """
    worst_successful_goal_s = 35.1     # R5, meta 2
    observed_timeouts = 3              # R5
    goal_timeout_s = float(node.get_parameter('goal_timeout_s').value)
    total_timeout_s = float(node.get_parameter('total_timeout_s').value)

    # Piso: nao pode cortar uma meta legitima. Teto: os tres estouros observados
    # precisam caber num quarto do orcamento, para que a maioria sobre para andar.
    assert goal_timeout_s > worst_successful_goal_s * 1.2
    assert observed_timeouts * goal_timeout_s <= total_timeout_s / 4


def test_homing_arrives_when_the_remaining_step_is_below_nav2_tolerance(
        node, monkeypatch) -> None:
    """
    Nao se comanda um deslocamento menor que a tolerancia de chegada do Nav2.

    Medido em R6: a aproximacao chegou a 0,75 m do marcador com
    `marker_stop_distance_m` em 0,70 -- 5 cm de falta. O passo restante de 5 cm
    e menor que `xy_goal_tolerance` (0,25 m), entao o Nav2 declara sucesso sem
    mover, o explorador ve 0,75 > 0,70 e manda de novo. O robo ficou 94 s
    parado ate o orcamento de persistencia estourar, e a exploracao foi embora.

    E a mesma armadilha que `min_frontier_distance_m = 0.35` ja resolve do lado
    das fronteiras desde R4; a aproximacao nunca ganhou a guarda equivalente.
    """
    sent: list = []
    _homing_ready(node, monkeypatch, sent)
    stop = float(node.get_parameter('marker_stop_distance_m').value)
    tolerance = float(node.get_parameter('nav_goal_tolerance_m').value)
    # Faltando menos que a tolerancia: mandar meta aqui e o laco de R6.
    node._exit_pose_map = (stop + tolerance * 0.5, 0.0)
    node._exit_seen_s = node._now_s()

    node._tick()

    assert node._state == 'completed'
    assert not sent, 'passo abaixo da tolerancia nao pode virar meta do Nav2'


def test_homing_arrives_at_exactly_the_nav2_tolerance(node, monkeypatch) -> None:
    """The equality boundary is also a Nav2 no-motion success."""
    sent: list = []
    _homing_ready(node, monkeypatch, sent)
    stop = float(node.get_parameter('marker_stop_distance_m').value)
    tolerance = float(node.get_parameter('nav_goal_tolerance_m').value)
    node._exit_pose_map = (stop + tolerance, 0.0)

    node._tick()

    assert node._state == 'completed'
    assert not sent


def test_homing_still_steps_when_the_remaining_distance_is_worth_commanding(
        node, monkeypatch) -> None:
    """A guarda acima nao pode engolir uma aproximacao legitima."""
    sent: list = []
    _homing_ready(node, monkeypatch, sent)
    stop = float(node.get_parameter('marker_stop_distance_m').value)
    tolerance = float(node.get_parameter('nav_goal_tolerance_m').value)
    node._exit_pose_map = (stop + tolerance * 3.0, 0.0)
    node._exit_seen_s = node._now_s()

    node._tick()

    assert node._state == 'homing_exit'
    assert sent


def test_the_homing_tolerance_matches_what_nav2_is_configured_with(node) -> None:
    """
    Duas copias do mesmo numero em arquivos diferentes divergem sozinhas.

    O explorador precisa saber a tolerancia de chegada do Nav2 para nao comandar
    passos que o controlador nao consegue distinguir de zero. Ele nao le o YAML
    do Nav2, entao este teste e o que mantem os dois valores iguais -- nos dois
    arquivos de parametros, o padrao e a variante de footprint.
    """
    import pathlib

    import yaml

    config = pathlib.Path(__file__).resolve().parents[1] / 'config'
    declared = float(node.get_parameter('nav_goal_tolerance_m').value)
    for name in ('nav2_params_go2.yaml', 'nav2_params_go2_footprint.yaml'):
        params = yaml.safe_load((config / name).read_text(encoding='utf-8'))
        checker = params['controller_server']['ros__parameters'][
            'general_goal_checker']
        assert declared == float(checker['xy_goal_tolerance']), name
