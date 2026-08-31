"""Frontier exploration executive for the Go2 maze demonstration."""

from __future__ import annotations

from dataclasses import asdict
import json
import math
import struct
import time
import zlib

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .frontier import extract_frontiers, Frontier, frontier_score, Grid, path_length


STATES = {
    'idle', 'waiting_map', 'selecting', 'navigating', 'homing_exit',
    'completed', 'failed', 'cancelled',
}


def _setback_point(
    poses, setback_m: float, min_travel_m: float = 0.0,
) -> tuple[float, float] | None:
    """
    Return a point `setback_m` back from a path's end, along the path itself.

    Walking backward along an already-validated plan keeps the result on a
    route the planner proved reachable -- unlike moving the frontier's own
    goal closer to a wall, which does not make Nav2's controller any more
    willing to track a path that close. Falls back to the path's own start
    point if the whole path is shorter than `setback_m`, and to `None` for an
    empty path (the caller already has the original point to fall back on).

    `min_travel_m` is the floor below which the result is discarded (`None`)
    instead of returned: on a short path, walking back `setback_m` from the
    end can land within Nav2's own `xy_goal_tolerance` of the robot's CURRENT
    pose, so `SimpleGoalChecker` calls the goal reached without the robot
    moving at all -- R11 (29/08) stalled exactly this way, motionless for
    minutes on an identical unchanging map. The caller falls back to the
    original (un-recessed) endpoint when this returns `None`.
    """
    if not poses:
        return None
    points = [(p.pose.position.x, p.pose.position.y) for p in poses]
    start = points[0]

    def far_enough(point: tuple[float, float]) -> bool:
        return math.hypot(point[0] - start[0], point[1] - start[1]) \
            >= min_travel_m

    if len(points) == 1:
        return points[0] if far_enough(points[0]) else None

    remaining = setback_m
    for i in range(len(points) - 1, 0, -1):
        x1, y1 = points[i]
        x0, y0 = points[i - 1]
        segment = math.hypot(x1 - x0, y1 - y0)
        if segment >= remaining:
            ratio = remaining / segment if segment > 0.0 else 0.0
            candidate = (x1 - (x1 - x0) * ratio, y1 - (y1 - y0) * ratio)
            return candidate if far_enough(candidate) else None
        remaining -= segment
    # The whole path is shorter than `setback_m`: there is no point on it
    # that is actually `setback_m` from the end. Falling back to `start`
    # here would hand back the robot's own current pose -- degenerate
    # regardless of `min_travel_m`, so this is always discarded.
    return None


def _map_fingerprint(message: OccupancyGrid) -> int:
    """
    Return a CRC32 over the map's geometry and cell contents together.

    R15 hashed only `message.data`, so a resolution/size/origin change (a
    SLAM re-anchor or a resize with identical cell values, for instance)
    would not advance `_map_seq` -- a real map change silently treated as a
    republication of the same one. Folding width, height, resolution and the
    full origin pose into the hash closes that gap.
    """
    info = message.info
    origin = info.origin
    header = struct.pack(
        '<IIfddddddd',
        info.width, info.height, info.resolution,
        origin.position.x, origin.position.y, origin.position.z,
        origin.orientation.x, origin.orientation.y,
        origin.orientation.z, origin.orientation.w,
    )
    return zlib.crc32(header + bytes(message.data))


class MazeExplorer(Node):
    """Choose reachable map frontiers and hand them to Nav2 one at a time."""

    def __init__(self) -> None:
        super().__init__('maze_explorer')
        self.declare_parameter('exploration_bt_xml', '')
        self.declare_parameter('total_timeout_s', 600.0)
        # 90 s, e NAO 180 s. A rodada 3 (29/08) subiu para 180 e o resultado
        # foi pior em tudo: 4,26 m contra 21,93 m, 3746 celulas contra 8915,
        # razao de trabalho 8,6% contra 41,7%, e nenhuma deteccao do marcador.
        #
        # A rodada 2 tinha expirado tres metas distantes em exatamente 90,0 s e
        # a leitura foi "o teto e curto demais". Era leitura errada: a rodada 3
        # travou 180 s numa meta a 0,4 m do robo. O teto nao esta cortando
        # travessia lenta, esta cortando travamento -- e dobra-lo so torna cada
        # travamento duas vezes mais caro.
        #
        # O que continua errado e a PERMANENCIA: uma meta que expira vai para a
        # blacklist dura e nunca volta. Ver docs/results/ml35-f5-exploration-r3.md.
        # Medido em R5 (arm B, 21 metas): as 12 metas que deram certo levaram
        # 6,1-35,1 s; as 3 que estouraram gastaram 90,0 s cada, 270 s de um
        # orcamento de 600 s. 45 s cobre a pior meta boa com 28% de margem e
        # corta pela metade o custo de cada meta travada.
        self.declare_parameter('goal_timeout_s', 45.0)
        self.declare_parameter('marker_stale_s', 2.0)
        self.declare_parameter('marker_stop_distance_m', 0.7)
        # Espelha `xy_goal_tolerance` do `general_goal_checker` do Nav2. Serve
        # para nao comandar passo que o controlador nao distingue de zero;
        # um teste de contrato mantem os dois valores iguais.
        self.declare_parameter('nav_goal_tolerance_m', 0.25)
        self.declare_parameter('homing_step_m', 0.5)
        # Orcamento da aproximacao as cegas. `marker_stale_s` diz quando o
        # marcador deixou de ser visto; este diz por quanto tempo ainda vale
        # caminhar ate a pose ja travada. Sem ele o homing desistia na
        # primeira parede que cortava a visada -- 0 de 11 aproximacoes em campo.
        self.declare_parameter('homing_persistence_s', 90.0)
        # Portao de ENTRADA, dimensionado com as 131 amostras de R7 contra o
        # marcador do SDF: o erro absoluto medio da estimativa e 0,41 m na faixa
        # 3-4 m, 1,14 m em 4-6 m e 3,08 m acima de 6 m. R7 comprometeu-se a
        # 7,35 m e a aproximacao, agora persistente, prendeu a corrida 520 s.
        self.declare_parameter('homing_max_distance_m', 4.0)
        # Histerese: a estimativa oscilou de 1,27 a 7,94 m na mesma corrida, e
        # uma amostra unica nao pode cancelar a exploracao.
        self.declare_parameter('homing_confirm_observations', 3)
        self.declare_parameter('blacklist_radius_m', 0.75)
        # Ciclos CONSECUTIVOS de selecao sem nenhum candidato viavel antes
        # de declarar falha. Existe porque aposentar fronteiras reprovadas
        # (ver `_on_path_result`) pode acabar engolindo todas elas, e o
        # robo parado nao gera mapa novo -- a situacao nunca se resolve
        # sozinha. A 1 Hz do `_tick`, 10 e ~10 s: folgado para a janela em
        # que o mapa ainda nao atualizou depois de uma chegada, e curto o
        # bastante para nao gastar o orcamento parado.
        self.declare_parameter('barren_selections_limit', 10)
        # Distancia minima entre o robo e uma fronteira para ela ser candidata.
        #
        # O `xy_goal_tolerance` do Nav2 e 0,25 m (`nav2_params_go2.yaml`). Uma
        # fronteira mais perto que isso faz o Nav2 devolver sucesso SEM que
        # nada se mova: a selecao volta ao mesmo ponto, o mapa nao muda, e o
        # ciclo se repete. Rodada 4 de 29/08: 565 vezes em 580 s, robo dentro
        # de uma caixa de 11 mm x 25 mm, mapa congelado em 2669 celulas.
        #
        # 0,35 m da folga sobre a tolerancia sem esconder fronteira util. O
        # corte e relativo a pose ATUAL e recalculado a cada ciclo -- nao e
        # anotacao, nao entra em nenhuma das tres listas de supressao, e andar
        # alguns centimetros devolve a fronteira a disputa sozinho.
        self.declare_parameter('min_frontier_distance_m', 0.35)
        # Raio, em `extract_frontiers`, que uma celula-alvo precisa manter
        # livre de qualquer celula ocupada para virar candidata (`has_clearance`
        # em frontier.py). O default anterior, 0,45 m, e maior que o meio-
        # comprimento do footprint (0,37 m, `nav2_params_go2.yaml`) e reprovava
        # celulas perto de vaos e cantos -- exatamente onde uma fronteira
        # estreita encontra a parede. 0,38 m mantem uma folga real sobre o
        # footprint (nao sobre `robot_radius`, que ja foi substituido) e deixa
        # o robo se aproximar mais da parede a frente antes de a fronteira
        # daquele lado ser descartada. Feedback de 29/08: o robo desistia cedo
        # demais perto de paredes e perdia aberturas.
        self.declare_parameter('frontier_wall_clearance_m', 0.38)
        # Alternates per cluster, and how far apart they must sit. R10
        # (29/08) died in 28.5 s because its one frontier cluster had exactly
        # one candidate point, that point was refused by the planner
        # (NO_VALID_PATH), and there was nothing else in the SAME cluster to
        # retry -- the whole cluster was lost over one unreachable point.
        # These let `extract_frontiers` hand back backup points from the same
        # cluster so a single bad point no longer costs the whole region.
        self.declare_parameter('frontier_max_alternates', 2)
        self.declare_parameter('frontier_alternate_spacing_m', 0.25)
        # After ComputePathToPose validates a candidate, navigate to a point
        # this far back from the endpoint ALONG THE RETURNED PATH, not to the
        # endpoint itself. Feedback 29/08: the robot gave up on a corridor
        # too early on meeting a wall ahead, missing the openings beside it --
        # but pulling the frontier's own goal placement closer to the wall
        # (see `frontier_wall_clearance_m` above) does not, on its own, make
        # Nav2's controller willing to track a path that close. A point set
        # back along a path the planner already proved reachable does not
        # have that problem: it is still on a validated route, just short of
        # its far end. The original endpoint is kept for scoring only
        # (`frontier_score`/`information_gain_m`) so scoring still reflects
        # the real frontier, not the shortened approach.
        self.declare_parameter('frontier_endpoint_setback_m', 0.40)
        # R15 (30/08/2026). Vigia de movimento durante `navigating`: meta
        # aceita mas o robo nao progride (nem translacao nem rotacao). Medido
        # em R13 (docs/results/ml35-f5-exploration-r13.md): 5 janelas reais de
        # imobilidade com comando, 10,2-24,2 s de duracao, e 4 das 7 metas
        # com timeout (45 s) tinham uma dessas janelas dentro.
        #
        # CORRECAO (revisao de codigo pos-R14c): a versao original deste
        # comentario dizia "15 s fica abaixo da mais curta das 5" -- errado,
        # 15 > 10,2. Na verdade 15 s so captura 2 das 5 janelas medidas em
        # R13 (15,1 e 24,2 s); as outras tres (11,7, 10,7 e 10,2 s) ficam
        # abaixo do limiar e NAO disparariam o vigia. Isto e uma escolha
        # deliberada (nao capturar toda pausa curta de replanejamento normal
        # como travamento), nao uma alegacao de cobertura total -- mas o
        # comentario anterior alegava cobertura total por engano. Ainda sobra
        # folga grande contra os 45 s de `goal_timeout_s`; o objetivo e agir
        # ANTES de esgotar o prazo da meta nas janelas mais longas, nao
        # substitui-lo nem capturar cada caso.
        self.declare_parameter('stall_window_s', 15.0)
        # Mesmo limiar de deslocamento que `find_stalled_navigating_windows`
        # em `scripts/exploration_trial.py` ja usa contra dado real de R13 --
        # os dois tem de concordar, ou o watchdog em campo e o diagnostico
        # offline classificariam a mesma corrida de jeitos diferentes.
        self.declare_parameter('stall_move_threshold_m', 0.05)
        # R15a (30/08/2026). Progresso ANGULAR equivalente ao de translacao
        # acima -- sem isto, uma rotacao legitima em pé (por exemplo, virar
        # para encarar um corredor) sem deslocamento xy seria classificada
        # como travamento. 0.05 rad (~2,9 graus) fica acima do ruido tipico
        # de localizacao com o robo parado e bem abaixo de qualquer rotacao
        # deliberada -- julgamento de codigo, ainda sem dado de HIL dedicado
        # a travamentos rotacionais (CLAUDE.md regra 7: nao alegar validacao
        # de hardware que nao foi feita).
        self.declare_parameter('stall_rotate_threshold_rad', 0.05)
        # Varredura de observacao quando NENHUM cluster de fronteira bruto
        # existe (nao quando existe mas foi filtrado -- girar nao revela
        # nada de novo nesse caso). ~60 graus: uma volta completa a
        # max_rotational_vel 0.12 rad/s (behavior_server, nav2_params_go2.yaml)
        # leva ~52 s, quase o orcamento de uma meta inteira; uma fatia menor,
        # repetida a cada versao de mapa que continuar sem cluster algum,
        # cobre o entorno progressivamente sem monopolizar o orcamento total.
        self.declare_parameter('recovery_spin_rad', 1.047)
        # R17 (30/08/2026). Cone de rumo (graus) dentro do qual uma fronteira
        # ainda conta como "adiante" -- ate 120 graus de desvio do rumo
        # estabelecido, o suficiente para curvas e corredores laterais sem
        # tratar toda mudanca de direcao como retorno. So o arco de 60 graus
        # de cada lado do sentido exatamente oposto (os 120 graus restantes
        # dos 360) conta como reversa. Enquanto existir ao menos uma
        # fronteira adiante alcancavel, nenhuma reversa e considerada --
        # prioridade explicita, nao uma penalidade suave.
        self.declare_parameter('forward_cone_deg', 120.0)
        # R17. Espacamento minimo entre breadcrumbs consecutivos, para nao
        # empilhar pontos quase identicos quando metas ficam proximas umas
        # das outras -- a pilha existe para marcar cruzamentos reais
        # (spawn -> corredor A -> cruzamento B -> ...), nao cada parada.
        self.declare_parameter('breadcrumb_min_spacing_m', 0.75)

        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            String, '/demo/exploration/status', transient)
        self.create_service(Trigger, '/demo/exploration/start', self._start)
        self.create_service(Trigger, '/demo/exploration/cancel', self._cancel)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, transient)
        self.create_subscription(
            PoseStamped, '/demo/perception/maze_exit/pose',
            self._on_exit_pose, 10)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._path_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose')
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        self._nav_cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self._spin_client = ActionClient(self, Spin, 'spin')

        self._state = 'idle'
        self._message = ''
        self._map: OccupancyGrid | None = None
        # Sequencia do mapa, e nao o proprio mapa, como chave de cache: comparar
        # duas OccupancyGrid celula a celula custaria mais que a extracao que o
        # cache existe para evitar.
        #
        # R15 (30/08/2026): so avanca quando o CONTEUDO muda (`_on_map` compara
        # um checksum, nao apenas conta mensagens). `slam_toolbox` republica
        # `/map` periodicamente mesmo sem mudanca real, e antes desta correcao
        # cada republicacao contava como "mapa novo" para
        # `map_seq > last_provisional_map_seq` em `_begin_selection` --
        # liberando uma supressao provisoria que nenhuma observacao nova
        # desmentiu. O teste
        # `test_map_republication_does_not_release_a_provisional_recovery`
        # cobre exatamente este caso.
        self._map_seq = 0
        self._map_content_hash: int | None = None
        self._selection_key: tuple[int, int, int] | None = None
        self._frontier_count = 0
        # Instrumentacao. Medida com relogio MONOTONICO, nunca com /clock: sob
        # `use_sim_time` o relogio de simulacao pode pausar, saltar ou correr
        # fora do tempo real, e o que se quer aqui e CPU gasta de verdade.
        self._frontier_extract_ms = 0.0
        self._frontier_cells = 0
        self._frontier_clusters = 0
        self._frontier_clusters_raw = 0
        self._path_requests = 0
        self._selection_cycle = 0
        self._near_skipped = 0
        self._current: Frontier | None = None
        self._blacklist: list[tuple[float, float]] = []
        # TRES listas, porque as tres falhas nao significam a mesma coisa.
        #
        # `_blacklist` e dura: o Nav2 recusou a meta, ou devolveu falha
        # explicita para ela. Isso e falha de EXECUCAO daquela fronteira, e
        # mapa novo nao a desmente.
        #
        # `_timed_out` e provisoria: a meta estourou `goal_timeout_s`. Isso
        # marca a TENTATIVA, nao a fronteira -- a rodada 3 de 29/08 gastou
        # 180 s numa meta a 0,4 m do robo, entao o teto corta travamento, e
        # travamento fala da pose, do costmap e do plano daquele instante. Na
        # rodada 2 tres expiracoes viraram tres pontos permanentes que
        # engoliram os quatro clusters restantes aos 570 s.
        #
        # `_refused` e provisoria: o planejador nao achou caminho AGORA.
        # `ExplorationGrid` roda com `allow_unknown: false`, entao toda
        # fronteira distante e reprovada enquanto o caminho ate ela
        # atravessar desconhecido -- e e exatamente isso que a exploracao vai
        # desfazer. Medido na rodada 1 de 29/08: as duas unicas reprovacoes
        # foram a 2,3 m e 2,7 m do robo, e aposenta-las de vez deixou 3
        # clusters e 157 celulas de fronteira real sem nenhum candidato
        # permitido. Reduzir o raio nao ajudaria: o ponto anotado E o
        # centroide do cluster.
        self._refused: list[tuple[float, float]] = []
        self._timed_out: list[tuple[float, float]] = []
        # Guards the provisional-recovery release below: a fresh entry must
        # not be released in the very same map generation that produced it.
        # R9 (29/08) re-selected and re-timed-out the identical coordinate
        # twice in a row (goals 7-8) because a recovery fired between them
        # with no map change in between -- releasing a suppression the map
        # has not yet had a chance to disprove.
        self._last_provisional_map_seq = -1
        # If provisional suppression covers every otherwise usable frontier,
        # release it once. A second dead end before real navigation progress
        # must count as barren instead of creating a refuse/release livelock.
        self._provisional_recovery_used = False
        self._provisional_recoveries = 0
        # Keep the raw visual candidate separate from the pose accepted for
        # homing. R7 measured estimates swinging from 1.27 to 7.94 m; a later
        # partial view must not overwrite a target which already passed the
        # entry gate.
        self._marker_distance_m: float | None = None
        self._homing_entry_distance_m: float | None = None
        self._homing_entries = 0
        self._barren_cycles = 0
        self._started_s = 0.0
        self._goal_started_s = 0.0
        self._epoch = 0
        self._pending = False
        self._goal_handle = None
        self._candidates: list[Frontier] = []
        self._candidate_index = 0
        # Which point of the CURRENT candidate frontier is under test: 0 is
        # its primary (x, y), 1..N its `alternates`. Reset whenever
        # `_candidate_index` moves to a new cluster.
        self._candidate_alt_index = 0
        self._best: tuple[float, Frontier, tuple[float, float]] | None = None
        # Telemetry of the most recent ComputePathToPose attempt, for the
        # status message -- without this, a refusal like R10's could only be
        # explained by reconstructing it offline after the fact.
        self._last_candidate_point: tuple[float, float] | None = None
        self._last_path_status: int | None = None
        self._last_path_error_code: int | None = None
        self._last_path_error_msg: str = ''
        self._last_path_planner_id: str = ''
        # The frontier's own endpoint vs. the point actually commanded to
        # Nav2 -- normally identical, but a setback point (see
        # `_setback_point`) makes them differ for exploration goals. Kept
        # separate so a HIL report can tell which one was used without
        # reconstructing it from the path afterwards.
        self._last_nav_original: tuple[float, float] | None = None
        self._last_nav_target: tuple[float, float] | None = None
        self._exit_candidate_pose_map: tuple[float, float] | None = None
        self._exit_pose_map: tuple[float, float] | None = None
        self._exit_seen_s = 0.0
        self._last_exit_observation: tuple[str, int] | None = None
        self._marker_observations = 0
        self._homing_failures = 0
        self._marker_far_ignored = 0
        self._near_marker_streak = 0
        self._homing_abandons = 0
        # Vigia de movimento (R15/R15a): ultima pose (x, y, yaw) e instante em
        # que o robo realmente progrediu (xy ou angular) desde a ACEITACAO da
        # meta ATUAL de `navigating` -- setado em `_on_nav_accepted`, nao no
        # despacho, para nao contar tempo de resposta do Nav2 como travamento.
        self._nav_last_pose: tuple[float, float, float] | None = None
        self._nav_last_progress_s = 0.0
        # Varredura de observacao (R15) quando nenhum cluster bruto existe.
        self._recovery_pending = False
        self._recovery_handle = None
        self._recovery_map_seq = -1
        self._recovery_attempts = 0
        # R17: exploracao direcional com backtracking por breadcrumbs.
        #
        # Rumo real (direcao do deslocamento, nao a guinada final) desde a
        # ultima meta concluida -- `None` ate a primeira, quando nao ha base
        # para classificar nada como "adiante" ou "reverso".
        self._current_heading: float | None = None
        # Pose do robo no despacho da meta ATUAL, para medir o deslocamento
        # real na chegada (`_update_heading`) -- nao a pose no fim, que so
        # diz onde parou, nao de onde veio.
        self._nav_departure_pose: tuple[float, float] | None = None
        # Pilha de poses seguras, uma por meta de exploracao concluida
        # (nao por retorno), espacadas por `breadcrumb_min_spacing_m`.
        # Consumida (removida) no momento em que um retorno comeca -- nunca
        # reutilizada, o que impede um ciclo entre dois pontos.
        self._breadcrumbs: list[tuple[float, float]] = []
        self._is_backtrack_goal = False
        self._backtrack_attempts = 0
        self._decision_mode: str | None = None
        self._forward_candidates_count = 0
        self._reverse_candidates_count = 0
        self._heading_delta_deg: float | None = None
        self.create_timer(1.0, self._tick)
        self._publish_status()

    def _start(self, _request, response):
        if self._state in {'waiting_map', 'selecting', 'navigating', 'homing_exit'}:
            response.success = False
            response.message = 'busca ja esta em andamento'
            return response
        self._epoch += 1
        self._state = 'waiting_map'
        self._message = 'aguardando mapa, TF e Nav2'
        self._started_s = self._now_s()
        self._blacklist.clear()
        self._refused.clear()
        self._timed_out.clear()
        self._last_provisional_map_seq = -1
        self._provisional_recovery_used = False
        self._provisional_recoveries = 0
        self._candidate_alt_index = 0
        self._last_candidate_point = None
        self._last_path_status = None
        self._last_path_error_code = None
        self._last_path_error_msg = ''
        self._last_path_planner_id = ''
        self._last_nav_original = None
        self._last_nav_target = None
        self._marker_distance_m = None
        self._homing_entry_distance_m = None
        self._exit_candidate_pose_map = None
        self._exit_pose_map = None
        self._exit_seen_s = 0.0
        self._last_exit_observation = None
        self._marker_observations = 0
        self._homing_entries = 0
        self._barren_cycles = 0
        self._homing_failures = 0
        self._marker_far_ignored = 0
        self._near_marker_streak = 0
        self._homing_abandons = 0
        self._nav_last_pose = None
        self._nav_last_progress_s = 0.0
        self._recovery_pending = False
        self._recovery_handle = None
        self._recovery_map_seq = -1
        self._recovery_attempts = 0
        self._current_heading = None
        self._nav_departure_pose = None
        self._breadcrumbs = []
        self._is_backtrack_goal = False
        self._backtrack_attempts = 0
        self._decision_mode = None
        self._forward_candidates_count = 0
        self._reverse_candidates_count = 0
        self._heading_delta_deg = None
        self._release_goal()
        if self._nav_cancel_client.service_is_ready():
            # Empty goal_info means every active NavigateToPose goal.  Starting
            # autonomous exploration must not race a manual cockpit goal.
            self._nav_cancel_client.call_async(CancelGoal.Request())
        self._publish_status()
        response.success = True
        response.message = 'busca iniciada'
        return response

    def _cancel(self, _request, response):
        self._epoch += 1
        self._cancel_goal()
        self._state = 'cancelled'
        self._message = 'busca cancelada pelo operador'
        self._publish_status()
        response.success = True
        response.message = self._message
        return response

    def _on_map(self, message: OccupancyGrid) -> None:
        # Guardar e contar, so. Um mapa novo NAO troca a meta em voo: quem
        # decide seleção é `_tick`, e ele só chama `_begin_selection` no estado
        # `selecting`. Reagir aqui faria o robô abandonar a fronteira a cada
        # publicação do SLAM.
        #
        # `_map_seq` só avança quando o mapa muda de verdade -- geometria OU
        # celulas, ver `_map_fingerprint` -- comentário junto da declaração
        # do campo em `__init__`.
        self._map = message
        content_hash = _map_fingerprint(message)
        if content_hash != self._map_content_hash:
            self._map_content_hash = content_hash
            self._map_seq += 1

    def _on_exit_pose(self, message: PoseStamped) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', message.header.frame_id, Time())
        except TransformException:
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        cosine, sine = math.cos(yaw), math.sin(yaw)
        x, y = message.pose.position.x, message.pose.position.y
        candidate = (
            translation.x + cosine * x - sine * y,
            translation.y + sine * x + cosine * y,
        )
        stamp_ns = message.header.stamp.sec * 1_000_000_000 \
            + message.header.stamp.nanosec
        observation = (message.header.frame_id, stamp_ns)
        if observation == self._last_exit_observation:
            return
        self._last_exit_observation = observation
        self._marker_observations += 1
        self._exit_candidate_pose_map = candidate
        self._exit_seen_s = self._now_s()
        self._marker_distance_m = self._distance_to_pose(candidate)

        # Count camera observations here, not timer cycles. A pose remains
        # fresh across several `_tick` calls; counting there allowed a single
        # bad frame to satisfy all three confirmations.
        if self._state not in {'waiting_map', 'selecting', 'navigating'} \
                or self._marker_distance_m is None:
            return
        if self._marker_distance_m > float(
                self.get_parameter('homing_max_distance_m').value):
            self._near_marker_streak = 0
            self._marker_far_ignored += 1
            return
        self._near_marker_streak += 1
        if self._near_marker_streak < int(
                self.get_parameter('homing_confirm_observations').value):
            return

        self._exit_pose_map = candidate
        self._homing_entry_distance_m = self._marker_distance_m
        self._homing_entries += 1
        self._epoch += 1
        self._cancel_goal()
        self._state = 'homing_exit'
        self._message = 'marcador da saida detectado'
        self._near_marker_streak = 0
        # Preserve the exact entry event for a slower external recorder.
        self._publish_status()

    def _distance_to_pose(
        self, target: tuple[float, float] | None,
    ) -> float | None:
        """Return robot-to-target map distance, or None without target/TF."""
        if target is None:
            return None
        robot = self._robot_pose()
        if robot is None:
            return None
        return round(math.hypot(target[0] - robot[0], target[1] - robot[1]), 2)

    def _distance_to_exit(self) -> float | None:
        """Return distance to the accepted homing target, if one exists."""
        return self._distance_to_pose(self._exit_pose_map)

    def _tick(self) -> None:
        if self._state not in {'waiting_map', 'selecting', 'navigating', 'homing_exit'}:
            self._publish_status()
            return
        now = self._now_s()
        if now - self._started_s >= float(
                self.get_parameter('total_timeout_s').value):
            self._fail('prazo total de exploracao excedido')
            return
        marker_fresh = self._exit_candidate_pose_map is not None \
            and now - self._exit_seen_s <= float(
                self.get_parameter('marker_stale_s').value)
        self._marker_distance_m = self._distance_to_pose(
            self._exit_candidate_pose_map)

        if self._state == 'waiting_map':
            if self._map is not None and self._robot_pose() is not None \
                    and self._path_client.server_is_ready() \
                    and self._nav_client.server_is_ready():
                self._state = 'selecting'
                self._message = ''
        elif self._state == 'selecting' and not self._pending:
            self._begin_selection()
        elif self._state == 'navigating':
            if now - self._goal_started_s >= float(
                    self.get_parameter('goal_timeout_s').value):
                self._timeout_current('meta de fronteira expirou')
            elif self._navigation_stalled(now):
                self._timeout_current(
                    'vigia de movimento: robo parado (sem progresso xy/angular)')
        elif self._state == 'homing_exit' and not self._pending \
                and self._goal_handle is None:
            if marker_fresh:
                self._send_homing_step()
            elif now - self._exit_seen_s <= float(
                    self.get_parameter('homing_persistence_s').value):
                # A saida ja esta travada em `_exit_pose_map`; a visada servia
                # para aprende-la, nao para chegar la. Segue as cegas.
                self._send_homing_step(blind=True)
            else:
                self._homing_abandons += 1
                self._state = 'selecting'
                self._message = 'marcador perdido; retomando fronteiras'
                self._exit_pose_map = None
                self._near_marker_streak = 0
        self._publish_status()

    def _navigation_stalled(self, now: float) -> bool:
        """
        Return True when an accepted goal has produced no real progress.

        Janela deslizante: deslocamento xy acima de `stall_move_threshold_m`
        OU rotacao acima de `stall_rotate_threshold_rad` reinicia o relogio
        -- uma rotacao legitima em pé (virar para encarar um corredor) nao e
        travamento so por nao andar em linha reta. So dispara depois de
        `stall_window_s` sem nenhum dos dois -- ver a justificativa com os
        numeros de R13 junto da declaracao dos parametros em `__init__`.

        So avalia depois que Nav2 aceitou a meta (`_goal_handle` setado em
        `_on_nav_accepted`) -- antes disso nao ha comando em execucao para
        travar, so uma chamada de servico ainda em voo.
        """
        if self._goal_handle is None:
            return False
        robot = self._robot_pose()
        if robot is None:
            return False
        x, y, yaw = robot
        if self._nav_last_pose is None:
            self._nav_last_pose = (x, y, yaw)
            self._nav_last_progress_s = now
            return False
        last_x, last_y, last_yaw = self._nav_last_pose
        moved = math.hypot(x - last_x, y - last_y)
        turned = abs(math.atan2(
            math.sin(yaw - last_yaw), math.cos(yaw - last_yaw)))
        if moved >= float(self.get_parameter('stall_move_threshold_m').value) \
                or turned >= float(
                    self.get_parameter('stall_rotate_threshold_rad').value):
            self._nav_last_pose = (x, y, yaw)
            self._nav_last_progress_s = now
            return False
        return now - self._nav_last_progress_s >= float(
            self.get_parameter('stall_window_s').value)

    def _grid(self) -> Grid | None:
        if self._map is None:
            return None
        origin = self._map.info.origin
        q = origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return Grid(
            self._map.info.width, self._map.info.height,
            self._map.info.resolution,
            origin.position.x, origin.position.y, yaw, self._map.data,
        )

    def _begin_selection(self) -> None:
        if self._recovery_pending:
            # Varredura de observacao em voo -- ver
            # `_start_observation_recovery`. Reextrair agora correria sobre o
            # mesmo mapa que a justificou.
            return
        grid = self._grid()
        robot = self._robot_pose()
        if grid is None or robot is None:
            self._state = 'waiting_map'
            return

        # Sem mapa novo, sem blacklist nova e sem epoca nova, a extracao daria
        # exatamente o mesmo resultado. Sem esta guarda, o caso "planner
        # rejeitou todas as fronteiras" deixa `_pending` em False e o `_tick`
        # reextrai o mapa INTEIRO a cada segundo, indefinidamente -- que era o
        # explorador segurando um core do AM69 sem produzir nada.
        #
        # A epoca entra na chave para que iniciar ou cancelar a busca force uma
        # extracao, mesmo que o mapa e a blacklist estejam iguais.
        # R17: a pilha de breadcrumbs entra na chave porque um retorno pode
        # mudar o resultado da selecao SEM mudar epoca, mapa ou supressao --
        # a mudanca real e a pose do robo apos o retorno. Sem isto, a
        # chegada ao breadcrumb reproduziria a MESMA chave da ultima falha
        # de selecao e cairia no atalho de "nada mudou, conta como estéril"
        # antes de sequer reextrair fronteiras da nova posicao.
        key = (self._epoch, self._map_seq,
               len(self._blacklist) + len(self._refused)
               + len(self._timed_out), len(self._breadcrumbs))
        if key == self._selection_key:
            # Nada mudou desde o ciclo anterior, entao nao ha o que reextrair --
            # mas tambem nao houve progresso, e ficar aqui e indistinguivel de
            # estar travado. Conta para o limite.
            self._note_barren_selection()
            return
        self._selection_key = key

        started = time.monotonic()
        extract_stats: dict = {}
        frontiers = extract_frontiers(
            grid,
            clearance_m=float(
                self.get_parameter('frontier_wall_clearance_m').value),
            max_alternates=int(
                self.get_parameter('frontier_max_alternates').value),
            alternate_spacing_m=float(
                self.get_parameter('frontier_alternate_spacing_m').value),
            stats=extract_stats,
        )
        self._frontier_extract_ms = round((time.monotonic() - started) * 1e3, 1)
        self._selection_cycle += 1
        self._frontier_clusters = len(frontiers)
        self._frontier_clusters_raw = extract_stats.get('raw_clusters', 0)
        self._frontier_cells = sum(item.cells for item in frontiers)

        # Apply permanent and geometric exclusions first. This intermediate
        # set tells whether provisional suppression alone caused a dead end.
        frontiers = [item for item in frontiers if not any(
            math.hypot(item.x - x, item.y - y) <= float(
                self.get_parameter('blacklist_radius_m').value)
            for x, y in self._blacklist)]
        # Depois da supressao e ANTES da ordenacao: a ordenacao e por
        # proximidade, entao sem este corte a fronteira degenerada seria sempre
        # a primeira candidata.
        near_limit = float(
            self.get_parameter('min_frontier_distance_m').value)
        reachable = [item for item in frontiers if math.hypot(
            item.x - robot[0], item.y - robot[1]) >= near_limit]
        self._near_skipped = len(frontiers) - len(reachable)
        provisional = list(self._refused) + list(self._timed_out)
        frontiers = [item for item in reachable if not any(
            math.hypot(item.x - x, item.y - y) <= float(
                self.get_parameter('blacklist_radius_m').value)
            for x, y in provisional)]

        # R4a ended with real clusters but zero permitted candidates: every
        # cluster was covered by provisional entries whose only release event
        # was reaching another frontier. Break that circular dependency once.
        # If the retried frontiers fail again before a successful arrival, the
        # normal barren limit terminates the run instead of clearing forever.
        if reachable and not frontiers and provisional \
                and not self._provisional_recovery_used \
                and self._map_seq > self._last_provisional_map_seq:
            self._refused.clear()
            self._timed_out.clear()
            self._provisional_recovery_used = True
            self._provisional_recoveries += 1
            frontiers = reachable
            self._message = 'released provisional frontier suppressions'
        frontiers.sort(key=lambda item: math.hypot(
            item.x - robot[0], item.y - robot[1]))
        self._frontier_count = len(frontiers)
        # R17: adiante vence sempre que existir -- so considera reversa
        # quando nao ha nenhuma fronteira adiante alcancavel.
        forward, reverse = self._split_forward_reverse(frontiers, robot)
        self._forward_candidates_count = len(forward)
        self._reverse_candidates_count = len(reverse)
        if forward:
            self._decision_mode = 'forward'
            selected = forward
        elif reverse:
            self._decision_mode = 'reverse'
            selected = reverse
        else:
            self._decision_mode = None
            selected = []
        self._candidates = selected[:8]
        self._candidate_index = 0
        self._candidate_alt_index = 0
        self._best = None
        if not self._candidates:
            # R15: classificacao honesta de por que nao ha candidato, em vez
            # de um unico rotulo 'nenhuma fronteira segura alcancavel' para
            # tres causas distintas -- ver `classify_stop_reason` em
            # `scripts/exploration_trial.py`, que ja separa estas contagens.
            if self._near_skipped:
                self._message = ('todas as fronteiras estao dentro da '
                                 'tolerancia de chegada')
            elif self._frontier_clusters_raw == 0:
                self._message = 'nenhum cluster de fronteira bruto'
            else:
                self._message = 'fronteiras existem mas foram filtradas'
            self._handle_no_usable_frontier()
            return
        self._validate_next()

    def _split_forward_reverse(
        self, frontiers: list[Frontier], robot: tuple[float, float, float],
    ) -> tuple[list[Frontier], list[Frontier]]:
        """
        Split frontiers into forward (within the heading cone) and reverse.

        R17: sem rumo estabelecido ainda (nenhuma meta de exploracao
        concluida nesta busca), trata tudo como adiante -- nao ha base para
        penalizar nada antes do primeiro deslocamento real.
        """
        if self._current_heading is None:
            return list(frontiers), []
        cone = float(self.get_parameter('forward_cone_deg').value)
        forward: list[Frontier] = []
        reverse: list[Frontier] = []
        for item in frontiers:
            bearing = math.atan2(item.y - robot[1], item.x - robot[0])
            delta = math.degrees(math.atan2(
                math.sin(bearing - self._current_heading),
                math.cos(bearing - self._current_heading)))
            (forward if abs(delta) <= cone else reverse).append(item)
        return forward, reverse

    def _handle_no_usable_frontier(self) -> None:
        """
        R17: sem candidato usavel -- backtrack por breadcrumb primeiro.

        Ordem: um breadcrumb ainda nao consumido e a opcao mais barata (nao
        gira, nao gasta orcamento de varredura) e a mais alinhada ao
        objetivo de so recuar quando de fato nao ha por onde seguir. A
        varredura de observacao so entra quando a pilha ja esvaziou, e
        exatamente nas mesmas condicoes de antes (nenhum cluster bruto,
        no maximo uma tentativa por versao de mapa) -- sem breadcrumbs
        disponiveis, o comportamento e identico ao de R15.
        """
        if self._breadcrumbs:
            self._start_backtrack()
            return
        if self._frontier_clusters_raw == 0 \
                and self._map_seq > self._recovery_map_seq:
            self._recovery_map_seq = self._map_seq
            self._start_observation_recovery()
            return
        self._note_barren_selection()

    def _start_backtrack(self) -> None:
        """
        Retorna ao breadcrumb mais recente em vez de declarar falha na hora.

        Consumido (retirado da pilha) no momento em que a navegacao de
        volta comeca, nao quando ela termina -- um retorno que falhe (Nav2
        recusa ou expira) nao pode ficar tentando o MESMO ponto para
        sempre. A pilha so encolhe, nunca reutiliza uma entrada ja
        retirada: e isso que impede um ciclo infinito entre dois pontos.
        """
        x, y = self._breadcrumbs.pop()
        self._backtrack_attempts += 1
        self._decision_mode = 'backtracking'
        frontier = Frontier(x=x, y=y, cells=0, information_gain_m=0.0)
        self._is_backtrack_goal = True
        self._send_navigation(frontier, exploration=True)

    def _start_observation_recovery(self) -> None:
        """
        Spin in place, within the limits `behavior_server` already validated.

        Gira dentro de `max_rotational_vel: 0.12` (`nav2_params_go2.yaml`),
        medido para nao derrubar o robo em recuperacao. So chamada quando
        nenhum cluster de fronteira bruto existe -- ver `_begin_selection`.
        """
        self._recovery_pending = True
        self._recovery_attempts += 1
        goal = Spin.Goal()
        goal.target_yaw = float(self.get_parameter('recovery_spin_rad').value)
        epoch = self._epoch
        future = self._spin_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done: self._on_recovery_accepted(done, epoch))

    def _on_recovery_accepted(self, future, epoch: int) -> None:
        if epoch != self._epoch:
            return
        handle = future.result()
        if not handle.accepted:
            self._recovery_pending = False
            self._note_barren_selection()
            return
        self._recovery_handle = handle
        handle.get_result_async().add_done_callback(
            lambda done: self._on_recovery_result(done, epoch))

    def _on_recovery_result(self, future, epoch: int) -> None:
        del future
        if epoch != self._epoch:
            return
        self._recovery_pending = False
        self._recovery_handle = None
        # A varredura em si nao decide nada; o proximo /map com conteudo
        # novo e que conta como progresso (`_map_seq`). Sem mapa novo, este
        # ciclo barren avanca em direcao ao limite normal -- uma varredura
        # que nao revelou nada nao pode girar para sempre.
        self._note_barren_selection()

    def _validate_next(self) -> None:
        if self._candidate_index >= len(self._candidates):
            if self._best is None:
                self._message = 'planner rejeitou todas as fronteiras'
                self._pending = False
                return
            _, frontier, target = self._best
            self._send_navigation(frontier, exploration=True, target=target)
            return
        frontier = self._candidates[self._candidate_index]
        points = ((frontier.x, frontier.y),) + frontier.alternates
        if self._candidate_alt_index >= len(points):
            # Every point of this cluster (primary and alternates) was
            # refused. O planejador REPROVOU esta fronteira. Sem aposenta-la,
            # ela volta identica no proximo ciclo, para sempre: foi o que
            # consumiu 459 s dos 600 s da fumaca de 28/08, com o mapa
            # congelado e um `ComputePathToPose` por segundo sobre a mesma
            # coordenada morta.
            #
            # Anotada pelo ponto PRIMARIO (o que identifica o cluster para as
            # supressoes por raio), nao pelo ultimo ponto tentado -- as
            # supressoes suprimem a REGIAO, nao um ponto especifico dentro
            # dela. `len(self._blacklist)`/`_refused` ja fazem parte da chave
            # de `_begin_selection`, entao o append sozinho ja forca uma
            # extracao nova no proximo ciclo.
            self._refused.append((frontier.x, frontier.y))
            self._last_provisional_map_seq = self._map_seq
            self._candidate_index += 1
            self._candidate_alt_index = 0
            self._validate_next()
            return
        point = points[self._candidate_alt_index]
        self._candidate_alt_index += 1
        goal = ComputePathToPose.Goal()
        goal.goal = self._pose(point[0], point[1], 0.0)
        goal.planner_id = 'ExplorationGrid'
        goal.use_start = False
        self._pending = True
        self._path_requests += 1
        self._last_candidate_point = point
        self._last_path_planner_id = goal.planner_id
        epoch = self._epoch
        future = self._path_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done: self._on_path_accepted(done, epoch, frontier, point))

    def _on_path_accepted(
        self, future, epoch: int, frontier: Frontier,
        point: tuple[float, float],
    ) -> None:
        if epoch != self._epoch:
            return
        handle = future.result()
        if not handle.accepted:
            self._pending = False
            self._validate_next()
            return
        handle.get_result_async().add_done_callback(
            lambda done: self._on_path_result(done, epoch, frontier, point))

    def _on_path_result(
        self, future, epoch: int, frontier: Frontier,
        point: tuple[float, float],
    ) -> None:
        if epoch != self._epoch:
            return
        wrapped = future.result()
        self._last_path_status = wrapped.status
        self._last_path_error_code = wrapped.result.error_code
        self._last_path_error_msg = wrapped.result.error_msg
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            route_m = path_length(wrapped.result.path.poses)
            score = frontier_score(frontier, route_m)
            if self._best is None or score > self._best[0]:
                # Floor: xy_goal_tolerance plus a margin, so the setback point
                # can never fall inside the radius Nav2's own goal checker
                # already treats as "arrived" -- see `_setback_point`'s
                # docstring for the R11 stall this guards against.
                setback = _setback_point(
                    wrapped.result.path.poses,
                    float(self.get_parameter(
                        'frontier_endpoint_setback_m').value),
                    min_travel_m=float(self.get_parameter(
                        'nav_goal_tolerance_m').value) + 0.10,
                )
                self._best = score, frontier, (setback or point)
            # This cluster already has a validated point; its remaining
            # alternates would only re-check the same region. Move on to the
            # next cluster instead of retrying them.
            self._candidate_index += 1
            self._candidate_alt_index = 0
        # A refusal here does NOT advance `_candidate_index`/append to
        # `_refused` -- `_validate_next` retries the next alternate of this
        # SAME frontier first, and only gives up on the whole cluster once
        # every point (primary and alternates) has been tried.
        self._pending = False
        self._validate_next()

    def _send_navigation(
        self, frontier: Frontier, exploration: bool,
        target: tuple[float, float] | None = None,
    ) -> None:
        robot = self._robot_pose()
        if robot is None:
            self._state = 'waiting_map'
            self._pending = False
            return
        # `target` is the point actually commanded -- normally the frontier's
        # own (x, y), but exploration goals may carry a setback point along
        # an already-validated path instead (see `_on_path_result`). Scoring
        # and suppression radii still key off `frontier.x/y`; only the
        # commanded pose changes.
        nav_x, nav_y = target if target is not None else (frontier.x, frontier.y)
        self._last_nav_original = (frontier.x, frontier.y)
        self._last_nav_target = (nav_x, nav_y)
        # R17: pose de partida desta meta, para medir o deslocamento REAL na
        # chegada (`_update_heading`) -- e o angulo entre o rumo estabelecido
        # e esta fronteira, para telemetria (`heading_delta_deg`).
        self._nav_departure_pose = (robot[0], robot[1])
        if exploration and self._current_heading is not None:
            bearing = math.atan2(
                frontier.y - robot[1], frontier.x - robot[0])
            self._heading_delta_deg = round(math.degrees(math.atan2(
                math.sin(bearing - self._current_heading),
                math.cos(bearing - self._current_heading))), 1)
        elif exploration:
            self._heading_delta_deg = None
        yaw = math.atan2(nav_y - robot[1], nav_x - robot[0])
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(nav_x, nav_y, yaw)
        if exploration:
            goal.behavior_tree = str(self.get_parameter('exploration_bt_xml').value)
        self._current = frontier
        self._pending = True
        self._barren_cycles = 0
        self._goal_started_s = self._now_s()
        # Vigia de movimento: NAO armar aqui. `send_goal_async` ainda esta em
        # voo -- armar so em `_on_nav_accepted`, quando Nav2 de fato aceitou a
        # meta, para nao contar o tempo de resposta da acao como travamento.
        self._state = 'navigating' if exploration else 'homing_exit'
        self._message = 'navegando para fronteira' if exploration \
            else 'aproximando marcador da saida'
        epoch = self._epoch
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done: self._on_nav_accepted(done, epoch, exploration))

    def _on_nav_accepted(self, future, epoch: int, exploration: bool) -> None:
        if epoch != self._epoch:
            return
        self._pending = False
        handle = future.result()
        if not handle.accepted:
            if exploration:
                self._blacklist_current('Nav2 recusou fronteira')
            else:
                self._homing_failed('Nav2 recusou aproximacao')
            return
        self._goal_handle = handle
        # Vigia de movimento: arma agora, no aceite -- nao no despacho (ver
        # `_send_navigation`). `None` descarta a pose da meta anterior, se
        # houver; o primeiro `_navigation_stalled` desta meta inicializa a
        # baseline.
        self._nav_last_pose = None
        self._nav_last_progress_s = self._now_s()
        handle.get_result_async().add_done_callback(
            lambda done: self._on_nav_result(done, epoch, exploration))

    def _on_nav_result(self, future, epoch: int, exploration: bool) -> None:
        if epoch != self._epoch:
            return
        status = future.result().status
        if exploration:
            if status != GoalStatus.STATUS_SUCCEEDED:
                self._blacklist_current(f'fronteira terminou com status {status}')
            else:
                # R17: captura ANTES de `_release_goal`, que zera a flag.
                # Um retorno concluido nao empilha um novo breadcrumb sobre
                # o ponto que acabou de ser retirado da pilha -- o rumo real
                # ainda e atualizado, so o registro de posicao e que muda.
                was_backtrack = self._is_backtrack_goal
                self._update_heading(push_breadcrumb=not was_backtrack)
                self._release_goal()
                # Chegar mudou pose, costmap e mapa, que sao exatamente os
                # tres motivos pelos quais o planejador reprovou e pelos quais
                # a meta travou. As duas supressoes provisorias caem juntas; a
                # blacklist dura fica.
                self._refused.clear()
                self._timed_out.clear()
                self._last_provisional_map_seq = -1
                self._provisional_recovery_used = False
                self._state = 'selecting'
                self._message = (
                    'retorno concluido; selecionando novamente' if was_backtrack
                    else 'fronteira alcancada; atualizando mapa')
        elif status == GoalStatus.STATUS_SUCCEEDED:
            self._release_goal()
            self._state = 'homing_exit'
            self._message = 'passo de aproximacao concluido'
        else:
            self._homing_failed(f'aproximacao terminou com status {status}')

    def _send_homing_step(self, blind: bool = False) -> None:
        robot = self._robot_pose()
        target = self._exit_pose_map
        if robot is None or target is None:
            return
        dx, dy = target[0] - robot[0], target[1] - robot[1]
        distance = math.hypot(dx, dy)
        stop = float(self.get_parameter('marker_stop_distance_m').value)
        # O que falta pode ser menor que a tolerancia de chegada do Nav2. Nesse
        # caso a meta seria satisfeita sem o robo andar, o explorador veria a
        # distancia inalterada e mandaria de novo -- 94 s parado a 0,75 m em R6.
        # Faltando menos que a tolerancia, ja se chegou.
        tolerance = float(self.get_parameter('nav_goal_tolerance_m').value)
        if distance - stop <= tolerance + 1e-9:
            self._state = 'completed'
            self._message = 'marcador alcancado; aguardando confirmacao de cruzamento'
            return
        # Com marcador fresco o passo curto reaproveita cada nova deteccao para
        # corrigir a mira. As cegas nao ha o que corrigir, e uma sequencia de
        # retas de 0,5 m so da ao planejador paredes para recusar: manda uma
        # meta unica e deixa o Nav2 contornar.
        step = distance - stop if blind else min(
            float(self.get_parameter('homing_step_m').value), distance - stop)
        ratio = step / distance
        frontier = Frontier(
            robot[0] + dx * ratio, robot[1] + dy * ratio, 0, 0.0)
        self._send_navigation(frontier, exploration=False)

    def _homing_failed(self, message: str) -> None:
        self._homing_failures += 1
        self._release_goal()
        if self._homing_failures >= 3:
            self._state = 'selecting'
            self._message = f'{message}; retomando exploracao'
            self._homing_failures = 0
            self._exit_pose_map = None
            self._near_marker_streak = 0
        else:
            self._state = 'homing_exit'
            self._message = message

    def _note_barren_selection(self) -> None:
        """Um ciclo de selecao que nao produziu meta. Falha se virar habito."""
        self._barren_cycles += 1
        if self._barren_cycles >= int(
                self.get_parameter('barren_selections_limit').value):
            self._fail('nenhuma fronteira segura alcancavel')

    def _timeout_current(self, message: str) -> None:
        """Meta estourou o teto: suprime a fronteira, mas nao para sempre."""
        if self._current is not None:
            self._timed_out.append((self._current.x, self._current.y))
            self._last_provisional_map_seq = self._map_seq
        self._epoch += 1
        self._cancel_goal()
        self._state = 'selecting'
        self._message = message

    def _blacklist_current(self, message: str) -> None:
        if self._current is not None:
            self._blacklist.append((self._current.x, self._current.y))
        self._epoch += 1
        self._cancel_goal()
        self._state = 'selecting'
        self._message = message

    def _cancel_goal(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        if self._recovery_handle is not None:
            self._recovery_handle.cancel_goal_async()
            self._recovery_handle = None
            self._recovery_pending = False
        self._release_goal()

    def _release_goal(self) -> None:
        self._goal_handle = None
        self._pending = False
        self._current = None
        # R17: ponto de saida unico para toda meta (sucesso, blacklist,
        # timeout e cancelamento passam por aqui) -- garante que a flag
        # nunca vaze de uma meta de retorno para a proxima meta normal.
        self._is_backtrack_goal = False

    def _update_heading(self, push_breadcrumb: bool) -> None:
        """
        Registra o rumo real (nao a guinada final) e, se pedido, um breadcrumb.

        R17: rumo = direcao do deslocamento desde o despacho desta meta
        (`_nav_departure_pose`), nao a orientacao final do robo -- um robo
        que chega de lado ou virado ainda estava indo NAQUELA direcao.
        Segmentos curtos demais (ruido de localizacao, nao deslocamento
        real) nao atualizam o rumo, para nao deixar uma chegada quase no
        lugar redefinir "adiante" ao acaso.
        """
        robot = self._robot_pose()
        if robot is None or self._nav_departure_pose is None:
            return
        dx = robot[0] - self._nav_departure_pose[0]
        dy = robot[1] - self._nav_departure_pose[1]
        if math.hypot(dx, dy) >= 0.05:
            self._current_heading = math.atan2(dy, dx)
        if push_breadcrumb:
            self._push_breadcrumb((robot[0], robot[1]))

    def _push_breadcrumb(self, pose: tuple[float, float]) -> None:
        spacing = float(self.get_parameter('breadcrumb_min_spacing_m').value)
        if self._breadcrumbs and math.hypot(
                pose[0] - self._breadcrumbs[-1][0],
                pose[1] - self._breadcrumbs[-1][1]) < spacing:
            return
        self._breadcrumbs.append(pose)

    def _fail(self, message: str) -> None:
        self._epoch += 1
        self._cancel_goal()
        self._state = 'failed'
        self._message = message
        self._publish_status()

    def _robot_pose(self) -> tuple[float, float, float] | None:
        try:
            transform = self._tf_buffer.lookup_transform('map', 'base', Time())
        except TransformException:
            return None
        translation, q = transform.transform.translation, transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return translation.x, translation.y, yaw

    def _pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_status(self) -> None:
        assert self._state in STATES
        now = self._now_s()
        elapsed = 0.0 if not self._started_s else now - self._started_s
        goal = None if self._current is None else asdict(self._current)
        payload = {
            'state': self._state,
            'elapsed_s': round(elapsed, 1),
            'frontier_count': self._frontier_count,
            'goal': goal,
            'blacklisted': len(self._blacklist),
            'refused': len(self._refused),
            'timed_out': len(self._timed_out),
            # Custo da busca, para o operador e para o gate de CPU. Estes cinco
            # campos sao aditivos: o cockpit ignora o que nao conhece.
            'frontier_extract_ms': self._frontier_extract_ms,
            'frontier_cells': self._frontier_cells,
            'frontier_clusters': self._frontier_clusters,
            # Raw cluster count, before the clearance/standoff candidate
            # search. Distinguishes "only one cluster ever existed" from
            # "several existed and the candidate filters ate the rest" --
            # the two look identical in `frontier_clusters` alone.
            'frontier_clusters_raw': self._frontier_clusters_raw,
            'candidates_checked': self._candidate_index,
            'path_requests': self._path_requests,
            'selection_cycle': self._selection_cycle,
            'near_frontiers_skipped': self._near_skipped,
            # The most recent ComputePathToPose attempt: which point, and
            # exactly how the planner answered. Without this, a refusal like
            # R10's (29/08) can only be explained by reconstructing it
            # offline against the frozen map after the fact.
            'candidate_point_x': (
                None if self._last_candidate_point is None
                else round(self._last_candidate_point[0], 3)),
            'candidate_point_y': (
                None if self._last_candidate_point is None
                else round(self._last_candidate_point[1], 3)),
            'last_path_status': self._last_path_status,
            'last_path_error_code': self._last_path_error_code,
            'last_path_error_msg': self._last_path_error_msg,
            'last_path_planner_id': self._last_path_planner_id,
            # Original frontier endpoint (scoring/information-gain) vs. the
            # point actually commanded to Nav2 -- differ only when a setback
            # point along an already-validated plan was used instead.
            'nav_original_x': (
                None if self._last_nav_original is None
                else round(self._last_nav_original[0], 3)),
            'nav_original_y': (
                None if self._last_nav_original is None
                else round(self._last_nav_original[1], 3)),
            'nav_target_x': (
                None if self._last_nav_target is None
                else round(self._last_nav_target[0], 3)),
            'nav_target_y': (
                None if self._last_nav_target is None
                else round(self._last_nav_target[1], 3)),
            'provisional_recoveries': self._provisional_recoveries,
            'barren_cycles': self._barren_cycles,
            'recovery_attempts': self._recovery_attempts,
            'marker_visible': (
                self._exit_candidate_pose_map is not None
                and now - self._exit_seen_s <= float(
                    self.get_parameter('marker_stale_s').value)
            ),
            # The raw candidate and the accepted/latching homing target are
            # separate so a partial view cannot silently move the target.
            'marker_distance_m': self._marker_distance_m,
            'marker_observations': self._marker_observations,
            'marker_confirmations': self._near_marker_streak,
            'marker_candidate_x': (
                None if self._exit_candidate_pose_map is None
                else round(self._exit_candidate_pose_map[0], 3)),
            'marker_candidate_y': (
                None if self._exit_candidate_pose_map is None
                else round(self._exit_candidate_pose_map[1], 3)),
            'marker_accepted_x': (
                None if self._exit_pose_map is None
                else round(self._exit_pose_map[0], 3)),
            'marker_accepted_y': (
                None if self._exit_pose_map is None
                else round(self._exit_pose_map[1], 3)),
            'homing_entry_distance_m': self._homing_entry_distance_m,
            'homing_entries': self._homing_entries,
            'homing_abandons': self._homing_abandons,
            'marker_far_ignored': self._marker_far_ignored,
            # R17: exploracao direcional com backtracking por breadcrumbs.
            # `decision_mode` e `None` ate a primeira selecao com
            # candidatos; o cockpit deve tratar isso como "ainda
            # selecionando", nao como um quarto modo.
            'decision_mode': self._decision_mode,
            'breadcrumbs': len(self._breadcrumbs),
            'backtrack_attempts': self._backtrack_attempts,
            'heading_delta_deg': self._heading_delta_deg,
            'forward_candidates': self._forward_candidates_count,
            'reverse_candidates': self._reverse_candidates_count,
            'message': self._message,
        }
        self._status_pub.publish(String(
            data=json.dumps(payload, separators=(',', ':'))))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MazeExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
