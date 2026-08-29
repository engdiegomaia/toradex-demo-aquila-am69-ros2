"""Frontier exploration executive for the Go2 maze demonstration."""

from __future__ import annotations

from dataclasses import asdict
import json
import math
import time

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
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
        self.declare_parameter('goal_timeout_s', 90.0)
        self.declare_parameter('marker_stale_s', 2.0)
        self.declare_parameter('marker_stop_distance_m', 0.7)
        self.declare_parameter('homing_step_m', 0.5)
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

        self._state = 'idle'
        self._message = ''
        self._map: OccupancyGrid | None = None
        # Sequencia do mapa, e nao o proprio mapa, como chave de cache: comparar
        # duas OccupancyGrid celula a celula custaria mais que a extracao que o
        # cache existe para evitar.
        self._map_seq = 0
        self._selection_key: tuple[int, int, int] | None = None
        self._frontier_count = 0
        # Instrumentacao. Medida com relogio MONOTONICO, nunca com /clock: sob
        # `use_sim_time` o relogio de simulacao pode pausar, saltar ou correr
        # fora do tempo real, e o que se quer aqui e CPU gasta de verdade.
        self._frontier_extract_ms = 0.0
        self._frontier_cells = 0
        self._frontier_clusters = 0
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
        self._barren_cycles = 0
        self._started_s = 0.0
        self._goal_started_s = 0.0
        self._epoch = 0
        self._pending = False
        self._goal_handle = None
        self._candidates: list[Frontier] = []
        self._candidate_index = 0
        self._best: tuple[float, Frontier] | None = None
        self._exit_pose_map: tuple[float, float] | None = None
        self._exit_seen_s = 0.0
        self._homing_failures = 0
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
        self._barren_cycles = 0
        self._homing_failures = 0
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
        self._map = message
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
        self._exit_pose_map = (
            translation.x + cosine * x - sine * y,
            translation.y + sine * x + cosine * y,
        )
        self._exit_seen_s = self._now_s()

    def _tick(self) -> None:
        if self._state not in {'waiting_map', 'selecting', 'navigating', 'homing_exit'}:
            self._publish_status()
            return
        now = self._now_s()
        if now - self._started_s >= float(
                self.get_parameter('total_timeout_s').value):
            self._fail('prazo total de exploracao excedido')
            return
        marker_fresh = self._exit_pose_map is not None and now - self._exit_seen_s \
            <= float(self.get_parameter('marker_stale_s').value)
        if marker_fresh and self._state != 'homing_exit':
            self._epoch += 1
            self._cancel_goal()
            self._state = 'homing_exit'
            self._message = 'marcador da saida detectado'

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
        elif self._state == 'homing_exit' and not self._pending \
                and self._goal_handle is None:
            if not marker_fresh:
                self._state = 'selecting'
                self._message = 'marcador perdido; retomando fronteiras'
            else:
                self._send_homing_step()
        self._publish_status()

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
        key = (self._epoch, self._map_seq,
               len(self._blacklist) + len(self._refused)
               + len(self._timed_out))
        if key == self._selection_key:
            # Nada mudou desde o ciclo anterior, entao nao ha o que reextrair --
            # mas tambem nao houve progresso, e ficar aqui e indistinguivel de
            # estar travado. Conta para o limite.
            self._note_barren_selection()
            return
        self._selection_key = key

        started = time.monotonic()
        frontiers = extract_frontiers(grid)
        self._frontier_extract_ms = round((time.monotonic() - started) * 1e3, 1)
        self._selection_cycle += 1
        self._frontier_clusters = len(frontiers)
        self._frontier_cells = sum(item.cells for item in frontiers)

        suppressed = (list(self._blacklist) + list(self._refused)
                      + list(self._timed_out))
        frontiers = [item for item in frontiers if not any(
            math.hypot(item.x - x, item.y - y) <= float(
                self.get_parameter('blacklist_radius_m').value)
            for x, y in suppressed)]
        # Depois da supressao e ANTES da ordenacao: a ordenacao e por
        # proximidade, entao sem este corte a fronteira degenerada seria sempre
        # a primeira candidata.
        near_limit = float(
            self.get_parameter('min_frontier_distance_m').value)
        reachable = [item for item in frontiers if math.hypot(
            item.x - robot[0], item.y - robot[1]) >= near_limit]
        self._near_skipped = len(frontiers) - len(reachable)
        frontiers = reachable
        frontiers.sort(key=lambda item: math.hypot(
            item.x - robot[0], item.y - robot[1]))
        self._frontier_count = len(frontiers)
        self._candidates = frontiers[:8]
        self._candidate_index = 0
        self._best = None
        if not self._candidates:
            self._message = ('todas as fronteiras estao dentro da '
                             'tolerancia de chegada') \
                if self._near_skipped else \
                'nenhuma fronteira segura alcancavel'
            self._note_barren_selection()
            return
        self._validate_next()

    def _validate_next(self) -> None:
        if self._candidate_index >= len(self._candidates):
            if self._best is None:
                self._message = 'planner rejeitou todas as fronteiras'
                self._pending = False
                return
            _, frontier = self._best
            self._send_navigation(frontier, exploration=True)
            return
        frontier = self._candidates[self._candidate_index]
        self._candidate_index += 1
        goal = ComputePathToPose.Goal()
        goal.goal = self._pose(frontier.x, frontier.y, 0.0)
        goal.planner_id = 'ExplorationGrid'
        goal.use_start = False
        self._pending = True
        self._path_requests += 1
        epoch = self._epoch
        future = self._path_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done: self._on_path_accepted(done, epoch, frontier))

    def _on_path_accepted(self, future, epoch: int, frontier: Frontier) -> None:
        if epoch != self._epoch:
            return
        handle = future.result()
        if not handle.accepted:
            self._pending = False
            self._validate_next()
            return
        handle.get_result_async().add_done_callback(
            lambda done: self._on_path_result(done, epoch, frontier))

    def _on_path_result(self, future, epoch: int, frontier: Frontier) -> None:
        if epoch != self._epoch:
            return
        wrapped = future.result()
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            route_m = path_length(wrapped.result.path.poses)
            score = frontier_score(frontier, route_m)
            if self._best is None or score > self._best[0]:
                self._best = score, frontier
        else:
            # O planejador REPROVOU esta fronteira. Sem aposenta-la, ela volta
            # identica no proximo ciclo, para sempre: foi o que consumiu 459 s
            # dos 600 s da fumaca de 28/08, com o mapa congelado e um
            # `ComputePathToPose` por segundo sobre a mesma coordenada morta.
            #
            # Nao usa `_blacklist_current`: aquele incrementa `_epoch` para
            # matar a meta em voo, e aqui ha uma rodada de validacao em
            # andamento cujos callbacks seguintes seriam descartados. So a
            # anotacao, sem trocar de epoca. `len(self._blacklist)` ja faz parte
            # da chave de `_begin_selection`, entao o append sozinho ja forca
            # uma extracao nova no proximo ciclo.
            self._refused.append((frontier.x, frontier.y))
        self._pending = False
        self._validate_next()

    def _send_navigation(self, frontier: Frontier, exploration: bool) -> None:
        robot = self._robot_pose()
        if robot is None:
            self._state = 'waiting_map'
            self._pending = False
            return
        yaw = math.atan2(frontier.y - robot[1], frontier.x - robot[0])
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(frontier.x, frontier.y, yaw)
        if exploration:
            goal.behavior_tree = str(self.get_parameter('exploration_bt_xml').value)
        self._current = frontier
        self._pending = True
        self._barren_cycles = 0
        self._goal_started_s = self._now_s()
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
                self._release_goal()
                # Chegar mudou pose, costmap e mapa, que sao exatamente os
                # tres motivos pelos quais o planejador reprovou e pelos quais
                # a meta travou. As duas supressoes provisorias caem juntas; a
                # blacklist dura fica.
                self._refused.clear()
                self._timed_out.clear()
                self._state = 'selecting'
                self._message = 'fronteira alcancada; atualizando mapa'
        elif status == GoalStatus.STATUS_SUCCEEDED:
            self._release_goal()
            self._state = 'homing_exit'
            self._message = 'passo de aproximacao concluido'
        else:
            self._homing_failed(f'aproximacao terminou com status {status}')

    def _send_homing_step(self) -> None:
        robot = self._robot_pose()
        target = self._exit_pose_map
        if robot is None or target is None:
            return
        dx, dy = target[0] - robot[0], target[1] - robot[1]
        distance = math.hypot(dx, dy)
        stop = float(self.get_parameter('marker_stop_distance_m').value)
        if distance <= stop:
            self._state = 'completed'
            self._message = 'marcador alcancado; aguardando confirmacao de cruzamento'
            return
        step = min(float(self.get_parameter('homing_step_m').value), distance - stop)
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
        self._release_goal()

    def _release_goal(self) -> None:
        self._goal_handle = None
        self._pending = False
        self._current = None

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
            'candidates_checked': self._candidate_index,
            'path_requests': self._path_requests,
            'selection_cycle': self._selection_cycle,
            'near_frontiers_skipped': self._near_skipped,
            'barren_cycles': self._barren_cycles,
            'marker_visible': (
                self._exit_pose_map is not None
                and now - self._exit_seen_s <= float(
                    self.get_parameter('marker_stale_s').value)
            ),
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
