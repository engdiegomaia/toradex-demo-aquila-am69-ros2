"""
Patrulha continua sob Nav2: manda METAS em ciclo, indefinidamente.

Roda na estacao x86 junto com o Nav2 (`nav_quadruped.launch.py`).

    ros2 run demo_bringup patrol_commander

## Como isto difere de demo_routine, e por que os dois nao podem coexistir

`demo_routine` publica `/demo/cmd_vel` direto: ela sabe a velocidade e nao sabe
onde o robo esta. Este no nao publica velocidade nenhuma -- ele manda meta pela
acao `navigate_to_pose` e deixa o Nav2 decidir a velocidade, o que e o que
permite desviar de obstaculo.

Rodar os dois ao mesmo tempo poe dois publicadores em `/demo/cmd_vel` (aqui via
`collision_monitor`, la direto). Isso NAO da erro: `twist_to_inputs` obedece a
ultima mensagem que chegou, alternando entre desvio e coreografia a 20 Hz. O robo
anda em espasmos e nenhum log explica. Escolha um dos dois.

## Por que as metas sao um ciclo e nao uma lista de waypoints do Nav2

O `waypoint_follower` do Nav2 tambem faria isto, e foi rejeitado: quando uma meta
da lista falha ele encerra a lista inteira, e numa exposicao um obstaculo mal
posicionado termina a demonstracao. Aqui uma meta que falha e ABANDONADA e o
ciclo segue para a proxima, que e o comportamento que uma exposicao precisa.
A contrapartida e que este no nao sabe dizer se o percurso completo foi cumprido
-- para isso use `nav2_simple_commander` num teste, nao este no.

## Os limites das metas nao sao arbitrarios

O costmap global e uma janela ROLANTE de 20 m sem mapa (ver
`nav2_params_go2.yaml`). Meta fora dela e ACEITA e depois falha perto da borda,
porque `allow_unknown: true` deixa o planejador tracar caminho pelo desconhecido.
`MAX_GOAL_RADIUS_M` rejeita essas metas na entrada, onde o erro ainda tem nome.

## Tempo limite por meta

Ao envelope medido do Go2 -- 0,15 m/s a frente, 0,12 rad/s de guinada -- 4 m
levam ~27 s no melhor caso, e um desvio dobra isso. `DEFAULT_GOAL_TIMEOUT_S` e
generoso de proposito: um limite curto cancela metas que estavam progredindo, o
que se parece com falha de navegacao e e falha de configuracao. Se voce apertar
este numero, meça primeiro.
"""

from dataclasses import dataclass
import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Raio maximo aceito para uma meta. A janela rolante do costmap global tem 20 m
# de lado, ou seja 10 m do centro; 8 m deixa 2 m de folga para o robo se afastar
# da origem durante o ciclo sem que a meta caia fora da janela.
MAX_GOAL_RADIUS_M = 8.0

# Tempo limite por meta, derivado da velocidade MEDIDA e nao escolhido a esmo.
#
# Medido em 20/08/2026 sob Nav2: velocidade media real de 0,021 m/s (pico 0,119).
# A perna mais longa do percurso default tem 4,27 m, o que da 203 s em linha reta;
# com o fator 1,3 de desvio, 264 s. 300 s cobre isso com folga.
#
# A media e muito menor que o pico porque o MPPI passa boa parte do tempo
# corrigindo rumo -- e a 0,12 rad/s de teto de guinada, corrigir rumo custa tempo
# em que quase nao se avanca. Nao encurte este prazo sem medir de novo: cancelar
# meta que estava progredindo parece falha de navegacao e e de configuracao.
DEFAULT_GOAL_TIMEOUT_S = 300.0

# Pausa entre metas. Existe pelo mesmo motivo do settle de `demo_routine`: o
# controlador de marcha precisa de um intervalo sem comando para assentar a
# postura, e mandar a proxima meta no instante em que a anterior termina nao da
# esse intervalo.
DEFAULT_SETTLE_S = 2.0


@dataclass(frozen=True)
class Goal:
    """Uma meta do ciclo, em metros e radianos, no frame do planejador."""

    x: float
    y: float
    yaw: float


# Percurso default: triangulo de tres metas, dimensionado contra os obstaculos de
# `quadruped_objects.sdf`. Cada meta e ALCANCAVEL e cada TRECHO exige desvio --
# as duas coisas medidas, nao estimadas.
#
# Os obstaculos daquele mundo: caixa (1.5, 0.0) meia-diagonal 0.21; cilindro
# (3.0, 0.45) r 0.18; caixa (3.0, -0.55) meia-diagonal 0.28; cilindro
# (4.5, 0.0) r 0.12. O raio circunscrito do Go2 e 0.383.
#
# Folga da RETA ao obstaculo mais proximo, por distancia ponto-segmento (nao
# pela distancia vertical num x escolhido, que superestima a folga):
#
#   (0,0)     -> (4, 1.5)   -0.068 m da caixa vermelha    BLOQUEADA
#   (4, 1.5)  -> (4, -1.5)  -0.003 m do cilindro amarelo  BLOQUEADA
#   (4, -1.5) -> (0,0)      -0.127 m da caixa azul        BLOQUEADA
#
# As tres retas estao bloqueadas, entao o desvio e obrigatorio -- que e o ponto
# do cenario. Se o robo andar em linha reta, ou o costmap esta vazio ou ele
# atravessou o obstaculo; as duas coisas sao falha.
#
# E as tres METAS sao folgadas: +0.887, +0.713 e +0.905 m. Meta apertada faz o
# Nav2 falhar por chegada impossivel, que se confunde com falha de desvio.
#
# O QUADRADO DE 3 m QUE PARECE OBVIO NAO SERVE, e vale registrar por que: a meta
# (3.0, 0.0) cai no vao entre o cilindro verde e a caixa azul. Esse vao tem
# 0.45-0.18 = 0.27 de um lado e -0.55+0.28 = -0.27 do outro, ou seja 0.54 m de
# largura livre, e o robo precisa de 2 x 0.383 = 0.77 m. A meta e inalcancavel, e
# o Nav2 a ACEITA e so falha depois de esgotar as recuperacoes -- o que se le como
# "o desvio nao funciona" e e uma meta impossivel.
#
# O yaw de cada meta e o rumo de CHEGADA -- a direcao em que o robo ja vem
# andando ao alcancar aquela meta -- e nao o rumo de saida para a meta seguinte.
#
# A diferenca custou uma corrida inteira. Com o yaw de saida, cada meta exigia
# giro PARADO de 110 a 139 graus na chegada: 16 a 20 s ao teto de 0,12 rad/s. E o
# giro nao fica parado -- medido em 20/08/2026, o robo chegou a 3,8 cm da meta
# (3.976, 1.470 contra 4.0, 1.5) e depois derivou 0,78 m em y girando para
# satisfazer a orientacao, saindo da tolerancia de posicao. A meta nunca fechou.
#
# Com o rumo de chegada, a orientacao ja esta satisfeita quando a posicao esta, e
# o giro para a meta seguinte acontece como parte do caminho seguinte -- andando,
# que e onde o Go2 gira melhor.
DEFAULT_WAYPOINTS = (
    Goal(4.0, 1.5, math.atan2(1.5, 4.0)),
    Goal(4.0, -1.5, -math.pi / 2.0),
    Goal(0.0, 0.0, math.atan2(1.5, -4.0)),
)


def parse_waypoints(flat: list) -> tuple:
    """
    Monta as metas validadas a partir da lista plana de parametro.

    O parametro chega plano -- [x, y, yaw, x, y, yaw, ...] -- porque o ROS 2 nao
    tem tipo de parametro para lista de listas. Isso torna facil errar o
    comprimento, e um comprimento errado silenciosamente desloca todas as metas
    seguintes, entao aqui isso e erro e nao aviso.
    """
    if len(flat) % 3 != 0:
        raise ValueError(
            'waypoints tem %d valores, que nao e multiplo de 3. O formato e '
            '[x, y, yaw, x, y, yaw, ...] em metros e radianos.' % len(flat))
    if not flat:
        raise ValueError('waypoints esta vazio; o ciclo nao teria meta nenhuma.')

    goals = []
    for index in range(0, len(flat), 3):
        x, y, yaw = (float(v) for v in flat[index:index + 3])
        distance = math.hypot(x, y)
        if distance > MAX_GOAL_RADIUS_M:
            raise ValueError(
                'meta %d esta a %.2f m da origem, acima do limite de %.1f m. A '
                'janela rolante do costmap global aceitaria essa meta e '
                'falharia perto da borda.'
                % (index // 3, distance, MAX_GOAL_RADIUS_M))
        goals.append(Goal(x, y, yaw))
    return tuple(goals)


def flatten(goals) -> list:
    """Devolve as metas no formato plano de parametro."""
    flat = []
    for goal in goals:
        flat.extend([goal.x, goal.y, goal.yaw])
    return flat


def to_pose(goal: Goal, frame_id: str, stamp) -> PoseStamped:
    """Monta o PoseStamped de uma meta, com o yaw como quaternion em z."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x = goal.x
    pose.pose.position.y = goal.y
    # Rotacao apenas em torno de z: o robo anda no plano. Escrever o quaternion
    # a mao evita depender de tf_transformations, que nao esta na imagem.
    pose.pose.orientation.z = math.sin(goal.yaw / 2.0)
    pose.pose.orientation.w = math.cos(goal.yaw / 2.0)
    return pose


class PatrolCommander(Node):
    """Manda metas do ciclo uma a uma, sem parar, ignorando as que falham."""

    def __init__(self) -> None:
        """Le os parametros, valida o percurso e abre o cliente da acao."""
        super().__init__('patrol_commander')

        self.declare_parameter('waypoints', flatten(DEFAULT_WAYPOINTS))
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('goal_timeout_s', DEFAULT_GOAL_TIMEOUT_S)
        self.declare_parameter('settle_s', DEFAULT_SETTLE_S)
        self.declare_parameter('loop', True)

        # Erro de parametro aborta a subida. Um percurso mal formado que virasse
        # aviso deixaria o no rodando sem mover o robo, que e o sintoma mais
        # caro de diagnosticar nesta pilha.
        self._goals = parse_waypoints(
            list(self.get_parameter('waypoints').value))
        self._frame = self.get_parameter('frame_id').value
        self._timeout = float(self.get_parameter('goal_timeout_s').value)
        self._settle = float(self.get_parameter('settle_s').value)
        self._loop = bool(self.get_parameter('loop').value)

        self._index = 0
        self._sent = 0
        self._succeeded = 0
        self._failed = 0
        self._goal_handle = None
        self._deadline = None
        # Entre send_goal_async e a resposta de aceitacao o handle ainda e None.
        # Sem esta bandeira o tick de 1 s reentra em _tick e manda OUTRA meta,
        # inundando o Nav2 com metas concorrentes -- e o Nav2 aceita, cancelando
        # implicitamente a anterior, entao o sintoma e o robo parado recebendo
        # meta nova toda vez que ia comecar a andar.
        self._pending = False

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 1 Hz e suficiente: este no supervisiona metas que levam dezenas de
        # segundos. Uma taxa alta so multiplicaria log.
        self._timer = self.create_timer(1.0, self._tick)

        self.get_logger().info(
            'patrulha com %d metas, timeout %.0f s, settle %.1f s, loop %s. '
            'NAO rode demo_routine junto: os dois viram publicadores de '
            '/demo/cmd_vel e o robo anda em espasmos.'
            % (len(self._goals), self._timeout, self._settle, self._loop))

    def _tick(self) -> None:
        """Manda a proxima meta, ou vigia o prazo da meta em curso."""
        if self._pending or self._goal_handle is not None:
            self._check_deadline()
            return

        if not self._client.server_is_ready():
            # Nao e erro: o Nav2 ainda esta ativando os nos de ciclo de vida.
            # Se persistir, o suspeito e autostart ou o /clock, nao este no.
            self.get_logger().info(
                'esperando a acao navigate_to_pose ficar pronta '
                '(Nav2 ativando)', throttle_duration_sec=10.0)
            return

        if self._index >= len(self._goals):
            if not self._loop:
                self.get_logger().info(
                    'percurso terminado: %d de %d metas cumpridas'
                    % (self._succeeded, self._sent))
                self._timer.cancel()
                return
            self._index = 0

        self._send(self._goals[self._index])
        self._index += 1

    def _send(self, goal: Goal) -> None:
        """Envia uma meta e arma o prazo dela."""
        message = NavigateToPose.Goal()
        message.pose = to_pose(goal, self._frame, self.get_clock().now().to_msg())

        self._sent += 1
        self.get_logger().info(
            'meta %d: x=%.2f y=%.2f yaw=%.0f deg em "%s"'
            % (self._sent, goal.x, goal.y, math.degrees(goal.yaw), self._frame))

        self._pending = True
        self._deadline = self._elapsed() + self._timeout
        future = self._client.send_goal_async(message)
        future.add_done_callback(self._on_accepted)

    def _on_accepted(self, future) -> None:
        """Guarda o handle da meta aceita, ou desiste dela se foi recusada."""
        self._pending = False
        handle = future.result()
        if not handle.accepted:
            # O Nav2 recusa meta cujo caminho ele nem tenta: fora da janela
            # rolante, ou dentro de obstaculo. Abandonar e seguir e proposital.
            self.get_logger().warning(
                'meta %d recusada pelo Nav2; seguindo para a proxima'
                % self._sent)
            self._failed += 1
            self._release()
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        """Registra o desfecho da meta e libera o ciclo."""
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._succeeded += 1
            self.get_logger().info(
                'meta %d cumprida (%d de %d)'
                % (self._sent, self._succeeded, self._sent))
        else:
            self._failed += 1
            # Status 5 = ABORTED, 6 = CANCELED. ABORTED aqui costuma ser
            # "recuperacoes esgotadas", que no Go2 quase sempre e obstaculo
            # dentro do raio inflado e nao falha do planejador.
            self.get_logger().warning(
                'meta %d terminou com status %d; abandonada, seguindo o ciclo '
                '(%d falhas)' % (self._sent, status, self._failed))
        self._release()

    def _check_deadline(self) -> None:
        """Cancela a meta em curso se ela passou do prazo."""
        if self._deadline is None or self._elapsed() < self._deadline:
            return
        if self._goal_handle is None:
            # Passou do prazo e a aceitacao nunca chegou. Nao ha o que cancelar;
            # solta o ciclo, senao ele trava aqui para sempre.
            self.get_logger().warning(
                'meta %d nunca foi aceita em %.0f s; soltando o ciclo'
                % (self._sent, self._timeout))
            self._failed += 1
            self._release()
            return
        self.get_logger().warning(
            'meta %d passou de %.0f s; cancelando. Se isto repetir, meça antes '
            'de encurtar o prazo: cancelar meta que progredia parece falha de '
            'navegacao e e de configuracao.' % (self._sent, self._timeout))
        self._goal_handle.cancel_goal_async()
        self._deadline = None

    def _release(self) -> None:
        """Libera o ciclo depois do settle, para a postura assentar."""
        self._goal_handle = None
        self._pending = False
        self._deadline = None
        # O settle e implementado como atraso do proximo envio, nao como pausa
        # bloqueante: bloquear o executor pararia os callbacks da acao.
        if self._settle > 0.0:
            self._timer.cancel()
            self._timer = self.create_timer(self._settle, self._resume)

    def _resume(self) -> None:
        """Volta a supervisao periodica depois do settle."""
        self._timer.cancel()
        self._timer = self.create_timer(1.0, self._tick)

    def _elapsed(self) -> float:
        """Segundos desde a epoca do relogio do no."""
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None) -> None:
    """Roda a patrulha até ser interrompida."""
    rclpy.init(args=args)
    node = PatrolCommander()
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
