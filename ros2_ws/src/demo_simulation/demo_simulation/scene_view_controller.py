"""
Câmera orbital das vistas de cena — o que os botões do cockpit movem.

Roda em: workstation x86 SOMENTE, no container `sim`. Ele fala com o serviço
`set_pose` do Gazebo, que só existe dentro do processo do simulador (regra 1 do
CLAUDE.md).

POR QUE ESTE NÓ EXISTE, EM VEZ DE O COCKPIT CHAMAR set_pose DIRETO

Porque `set_pose` só aceita pose ABSOLUTA, e "girar 15° para a esquerda" é uma
operação relativa. Alguém precisa saber onde a câmera está agora. Se esse
alguém fosse o navegador, teríamos o enquadramento declarado em dois lugares —
nos argumentos de `scene_cameras.launch.py` e outra vez em JavaScript — e eles
divergiriam no primeiro cenário novo. Pior: com `SIM_ARGS` reenquadrando as
câmeras para o labirinto, o primeiro clique num botão arrancaria a câmera do
enquadramento do labirinto para um default de armazém escrito no navegador.

Então o estado orbital vive aqui, ao lado das câmeras, semeado pelos MESMOS
parâmetros que as posicionaram no nascimento. O cockpit publica só um passo, e
não precisa saber nada de geometria.

MODELO ORBITAL

Uma câmera é (alvo T no chão, distância d, azimute a, inclinação p):

    P = T - d * (cos p * cos a, cos p * sin a, -sin p)

que se inverte, dado P e (p, a) com o alvo no plano z = 0:

    d = Pz / sin p        T = (Px, Py) + d * cos p * (cos a, sin a)

A vista de TOPO é o caso degenerado e não precisa de código próprio: com
p = pi/2 vem d = Pz e T = P, ou seja o alvo é o ponto sob a câmera, "zoom" vira
altura e "girar" vira rotação da imagem no próprio eixo. Um `if` para a vista
de topo seria um segundo modelo para manter em sincronia com o primeiro.

SEGUIR O ROBO

As duas vistas seguem o robo por default: o alvo da orbita passa a ser a pose do
robo mais o pan que o operador aplicou. Distancia, azimute e inclinacao nao sao
tocados, entao a vista iso mantem o enquadramento medido e desliza com o robo, e
a de topo (pitch = pi/2, alvo = ponto sob a camera) fica sobre ele.

Sem isto, no maze11 o robo sai do quadro da vista iso em poucos metros e o
operador perde justamente a testemunha independente da tela.

Seguir NAO desliga o pan: enquanto segue, os botoes de mover deslocam o OFFSET
em relacao ao robo, e nao um ponto do mundo. E por isso que segurar "mover para
a direita" continua fazendo o que diz, com o robo no lugar onde o operador o
deixou no quadro.

O interruptor e /demo/cockpit/scene/follow (std_srvs/SetBool), porque a vista
larga estatica — a que mostra o labirinto inteiro, com o enquadramento MEDIDO de
scene_cameras.launch.py — continua sendo um recurso e nao pode desaparecer sem
botao.

E o estado sai daqui, em /demo/cockpit/scene/following (std_msgs/Bool, latched),
nao do ultimo clique do cockpit. Mesma regra do rotulo de simulacao: o cockpit
pode ser recarregado, aberto em duas telas, ou aberto depois de alguem ter
desligado o seguimento pela linha de comando, e nos tres casos um botao pintado
pelo proprio clique estaria mentindo. Latched (TRANSIENT_LOCAL) para que uma aba
nova receba o valor sem esperar a proxima mudanca.

DE ONDE SAI A POSE DO ROBO, E A ARMADILHA QUE MORA NISSO

De /demo/odom. O que este no precisa e a pose no referencial do MUNDO, que e o
unico que o set_pose do Gazebo entende, e /demo/odom so coincide com ele por
sorte de configuracao:

  quadrupede   /go2/odom vem do gz-sim-odometry-publisher-system, que publica a
               pose exata do modelo no mundo (ground truth). Coincide sempre,
               inclusive com o `yaw:=1.5708` do maze11.

  diff-drive   /odom vem do plugin DiffDrive, que INTEGRA os encoders a partir
               de zero. A origem do odom e a pose de SPAWN, nao a do mundo.
               Coincide so enquanto x/y/yaw do spawn sao 0 — que e o default.

Por isso existem os parametros follow_offset_{x,y,yaw}: eles sao o odom -> mundo,
e simulation.launch.py os liga aos MESMOS x/y/yaw que spawnaram o robo.
quadruped.launch.py deixa em zero, de proposito.

Sem esse seed, um `x:=5` no diff-drive faria a camera seguir um fantasma 5 m ao
lado do robo — errado por um deslocamento constante, que e exatamente o tipo de
erro que se le como "a camera esta meio torta" e nao como "o referencial esta
errado".

CONTRATO

    /demo/cockpit/scene/cmd_view    geometry_msgs/TwistStamped   (entra)
    /demo/cockpit/scene/reset_view  std_srvs/Trigger             (entra)
    /demo/cockpit/scene/follow      std_srvs/SetBool             (entra)
    /demo/odom                      nav_msgs/Odometry            (entra)
    /demo/cockpit/scene/following   std_msgs/Bool                (sai, latched)
    /demo/sim/set_entity_pose       ros_gz_interfaces/SetEntityPose (sai)

O `header.frame_id` do TwistStamped escolhe a câmera: `scene_iso` ou
`scene_top`. Twist tem os seis graus de liberdade que a órbita precisa e é
mensagem padrão — o CLAUDE.md pede para não redefinir equivalentes do Twist:

    angular.z   azimute, rad          girar em torno do alvo
    angular.y   inclinação, rad       subir/descer o ponto de vista
    linear.x    aproximar, m          negativo = zoom in
    linear.y    deslocar lateral, m   pan no eixo direita/esquerda da imagem
    linear.z    deslocar frente, m    pan no eixo para dentro/fora da imagem

Passos vêm do cliente e não daqui de propósito: o tamanho do passo é decisão de
interface, e é a UI que sabe se o operador segurou o botão.
"""

import math

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
)
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

# Limites. A câmera não pode passar do zênite (a órbita perde o azimute) nem
# afundar até o chão (a vista vira uma parede de textura), e sem um teto de
# distância um operador com o dedo preso no botão manda a câmera para o espaço,
# de onde não há botão que a traga de volta — só o reset.
MIN_PITCH_RAD = 0.12
MAX_PITCH_RAD = math.pi / 2
MIN_DISTANCE_M = 1.0
MAX_DISTANCE_M = 80.0

# Alvo longe demais também é irrecuperável na prática.
MAX_TARGET_RADIUS_M = 60.0

# Quanto o alvo pode se afastar do ROBO enquanto a vista o segue. Menor que
# MAX_TARGET_RADIUS_M porque a pergunta é outra: ali é "não mande a câmera para
# o espaço", aqui é "não perca o robô de vista com o próprio botão de pan".
MAX_FOLLOW_OFFSET_M = 15.0

# Ritmo com que a pose seguida é reescrita no Gazebo. As câmeras de cena
# renderizam a 10 Hz (models/cockpit_scene_*.sdf), então empurrar mais rápido
# gasta chamada de serviço sem render novo para mostrar.
FOLLOW_PERIOD_S = 0.1

# Movimento abaixo disto não vale uma chamada de set_pose. Com o robô parado a
# odometria continua chegando a 50 Hz e trepida no último milímetro; sem esta
# banda morta o nó chamaria set_pose 10 vezes por segundo para sempre.
FOLLOW_DEADBAND_M = 0.02

# Duas coisas diferentes têm nomes parecidos, e confundi-las custa uma tarde:
#
#   scene_iso           o SENSOR, e o prefixo dos tópicos
#                       (/demo/cockpit/scene_iso/image_raw)
#   cockpit_scene_iso   o MODELO no Gazebo, que é o que `set_pose` move
#
# `set_pose` num nome de sensor devolve success=false e nada se mexe. Como as
# duas grafias existem de verdade no sistema, as duas são aceitas aqui e
# resolvidas para o nome do modelo — em vez de obrigar quem chama a saber qual
# das duas o Gazebo queria.
MODELS = {
    'scene_iso': 'cockpit_scene_iso',
    'scene_top': 'cockpit_scene_top',
}
# As duas grafias resolvem para a CHAVE da órbita, nunca para o nome do modelo:
# o nome do modelo é o que sai daqui em direção ao Gazebo, não o que indexa
# o estado. Trocar os dois lados faz toda mensagem legítima cair no ramo de
# 'câmera não existe' — com uma mensagem que lista justamente o nome enviado.
ALIASES = {**{key: key for key in MODELS},
           **{model: key for key, model in MODELS.items()}}
CAMERAS = tuple(MODELS)


class Orbit:
    """Estado orbital de UMA câmera, e a conversão de e para pose."""

    def __init__(self, x, y, z, pitch, yaw):
        self.home = (x, y, z, pitch, yaw)
        # Ultima pose do robo conhecida, ou None. Fica NA orbita e nao so no nó
        # porque `apply` precisa dela: sem isso, um pan aplicado enquanto segue
        # deixa `target` desatualizado até o próximo tique, e `position()` mente
        # nesse intervalo — que é justamente o instante em que o `_push` do
        # comando lê a pose para mandar ao Gazebo.
        self.anchor = None
        self.reset()

    def reset(self):
        x, y, z, pitch, yaw = self.home
        self.pitch = min(max(pitch, MIN_PITCH_RAD), MAX_PITCH_RAD)
        self.yaw = yaw
        # sin(pitch) nunca é zero por causa de MIN_PITCH_RAD; uma câmera na
        # horizontal não cruza o plano do chão e não tem alvo definido.
        self.distance = min(max(z / math.sin(self.pitch), MIN_DISTANCE_M),
                            MAX_DISTANCE_M)
        reach = self.distance * math.cos(self.pitch)
        self.target = (x + reach * math.cos(self.yaw),
                       y + reach * math.sin(self.yaw))
        # Pan acumulado ENQUANTO SEGUE, relativo ao robô. Zerar aqui é o que faz
        # "recentrar" significar a mesma coisa nos dois modos: parado, volta ao
        # enquadramento medido; seguindo, volta a ter o robô no centro.
        #
        # `anchor` NÃO é esquecido: recentrar é sobre enquadramento, não sobre
        # deixar de saber onde o robô está. Quem decide se o alvo volta para o
        # robô logo depois é o nó, e ele faz isso só quando está seguindo.
        self.follow_offset = (0.0, 0.0)

    def follow(self, anchor):
        """
        Guarda a pose do robô e reaponta o alvo. Chamado a cada tique.

        Só o ALVO se move: distância, azimute e inclinação são o enquadramento
        que alguém mediu, e seguir o robô não é motivo para mexer neles.
        """
        self.anchor = anchor
        self._retarget()

    def _retarget(self):
        """Alvo = robô + pan. Sem robô conhecido, o alvo fica onde está."""
        if self.anchor is None:
            return
        self.target = (self.anchor[0] + self.follow_offset[0],
                       self.anchor[1] + self.follow_offset[1])

    def position(self):
        reach = self.distance * math.cos(self.pitch)
        return (self.target[0] - reach * math.cos(self.yaw),
                self.target[1] - reach * math.sin(self.yaw),
                self.distance * math.sin(self.pitch))

    def apply(self, twist, following=False):
        """Aplica um passo relativo, já saturado nos limites."""
        self.yaw = _wrap(self.yaw + twist.angular.z)
        self.pitch = min(max(self.pitch + twist.angular.y, MIN_PITCH_RAD),
                         MAX_PITCH_RAD)
        self.distance = min(max(self.distance + twist.linear.x, MIN_DISTANCE_M),
                            MAX_DISTANCE_M)

        # Pan no referencial da IMAGEM, não do mundo: o operador está olhando a
        # tela e "para a direita" tem de ser para a direita na tela, qualquer
        # que seja o azimute. `forward` é a projeção da linha de visada no chão.
        forward = (math.cos(self.yaw), math.sin(self.yaw))
        right = (forward[1], -forward[0])
        dx = right[0] * twist.linear.y + forward[0] * twist.linear.z
        dy = right[1] * twist.linear.y + forward[1] * twist.linear.z

        # Seguindo, o pan move o OFFSET e não um ponto do mundo. Escrever no
        # alvo aqui seria escrever num campo que o próximo `follow()` sobrepõe
        # 100 ms depois — o botão de mover pareceria sem efeito, que é o pior
        # jeito de quebrar isto.
        if following:
            self.follow_offset = _clamp_radius(
                (self.follow_offset[0] + dx, self.follow_offset[1] + dy),
                MAX_FOLLOW_OFFSET_M,
            )
            # Reaponta AGORA, e não no próximo tique: quem chamou vai ler
            # position() em seguida para escrever a pose no Gazebo.
            self._retarget()
            return

        self.target = _clamp_radius((self.target[0] + dx, self.target[1] + dy),
                                    MAX_TARGET_RADIUS_M)


def _clamp_radius(point, limit):
    """Encolhe o vetor até o raio máximo, preservando a direção."""
    radius = math.hypot(point[0], point[1])
    if radius <= limit:
        return point
    scale = limit / radius
    return (point[0] * scale, point[1] * scale)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _quaternion(pitch, yaw):
    """RPY -> quaternion com roll = 0. Uma câmera de cena nunca tomba."""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return (-sp * sy, sp * cy, cp * sy, cp * cy)


class SceneViewController(Node):
    def __init__(self):
        super().__init__('scene_view_controller')

        # Os defaults DUPLICAM os de scene_cameras.launch.py, e é por isso que
        # o launch os repassa explicitamente: quem inicia por launch nunca cai
        # nestes valores. Eles existem só para `ros2 run` avulso não explodir.
        self.declare_parameter('iso_x', -3.0)
        self.declare_parameter('iso_y', 3.0)
        self.declare_parameter('iso_z', 2.4)
        self.declare_parameter('iso_pitch', 0.5150)
        self.declare_parameter('iso_yaw', -0.7854)
        self.declare_parameter('top_x', 0.0)
        self.declare_parameter('top_y', 0.0)
        self.declare_parameter('top_z', 6.0)
        self.declare_parameter('top_pitch', math.pi / 2)
        self.declare_parameter('top_yaw', math.pi / 2)

        # Seguir o robô. Ligado por default: perder o robô de vista é o modo de
        # falha comum do painel azul, e a vista larga estática continua a um
        # clique de distância (/demo/cockpit/scene/follow).
        self.declare_parameter('follow', True)
        self.declare_parameter('follow_topic', '/demo/odom')
        # odom -> mundo. Zero quando a odometria é ground truth (quadrúpede);
        # a pose de spawn quando ela é integrada dos encoders (diff-drive). Ver
        # a seção "DE ONDE SAI A POSE DO ROBÔ" no cabeçalho — este é o parâmetro
        # que o erro silencioso descrito lá tem como causa.
        self.declare_parameter('follow_offset_x', 0.0)
        self.declare_parameter('follow_offset_y', 0.0)
        self.declare_parameter('follow_offset_yaw', 0.0)

        def orbit(prefix):
            def value(name):
                return self.get_parameter(f'{prefix}_{name}').value

            return Orbit(value('x'), value('y'), value('z'),
                         value('pitch'), value('yaw'))

        self._orbits = {'scene_iso': orbit('iso'), 'scene_top': orbit('top')}

        # ReentrantCallbackGroup: a chamada a set_pose acontece DENTRO do
        # callback do tópico. Com o grupo mutuamente exclusivo default o
        # executor não roda a resposta do serviço enquanto o callback do tópico
        # não retorna, e o nó trava no primeiro botão — sem erro nenhum.
        group = ReentrantCallbackGroup()

        self._set_pose = self.create_client(
            SetEntityPose, '/demo/sim/set_entity_pose', callback_group=group)

        self.create_subscription(
            TwistStamped, '/demo/cockpit/scene/cmd_view', self._on_command, 10,
            callback_group=group)

        self.create_service(
            Trigger, '/demo/cockpit/scene/reset_view', self._on_reset,
            callback_group=group)

        # --- seguir o robô --------------------------------------------------
        self._following = bool(self.get_parameter('follow').value)
        self._seed = (self.get_parameter('follow_offset_x').value,
                      self.get_parameter('follow_offset_y').value,
                      self.get_parameter('follow_offset_yaw').value)
        # Nenhuma amostra ainda: até a primeira, seguir não muda nada e as duas
        # vistas ficam no enquadramento que o launch mediu. Um `(0, 0)` inicial
        # arrancaria as câmeras do labirinto para a origem do mundo antes de o
        # robô sequer publicar odometria.
        self._anchor = None

        follow_topic = self.get_parameter('follow_topic').value
        # SENSOR_DATA, o mesmo perfil com que demo_bringup/odom_tf.py já lê
        # este tópico — e esse é o único caminho de /demo/odom validado no
        # Aquila. Best-effort contra o publicador reliable da ponte é
        # compatível; escolher outro perfil aqui seria estrear uma combinação
        # de QoS na câmera, e QoS incompatível não dá erro, só silêncio.
        self.create_subscription(
            Odometry, follow_topic, self._on_odom, qos_profile_sensor_data,
            callback_group=group)

        self.create_service(
            SetBool, '/demo/cockpit/scene/follow', self._on_follow,
            callback_group=group)

        # Estado publicado, e latched. Ver a seção "SEGUIR O ROBO" no cabeçalho:
        # o botão do cockpit é pintado por isto e não pelo próprio clique.
        self._following_pub = self.create_publisher(
            Bool, '/demo/cockpit/scene/following',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._announce_following()

        # Timer, e não o próprio callback de odometria: a odometria chega a
        # 50 Hz e o render das câmeras é 10 Hz. Empurrar set_pose a 50 Hz
        # gastaria cinco chamadas de serviço por quadro renderizado.
        self.create_timer(FOLLOW_PERIOD_S, self._on_follow_tick,
                          callback_group=group)

        framing = ', '.join(
            f'{name} em ({x:.2f}, {y:.2f}, {z:.2f})'
            for name, (x, y, z) in
            ((n, o.position()) for n, o in self._orbits.items()))
        # rclpy nao tem logging no estilo printf: o RcutilsLogger aceita UMA
        # string. Passar args posicionais levanta TypeError na construcao do no,
        # que morre antes de existir para qualquer diagnostico.
        self.get_logger().info(
            f'controle de vista pronto: {framing}; '
            f'seguindo={self._following} por {follow_topic} '
            f'com seed odom->mundo {self._seed}')

    # --- entrada -----------------------------------------------------------

    def _on_command(self, message):
        requested = message.header.frame_id or 'scene_iso'
        name = ALIASES.get(requested)
        orbit = self._orbits.get(name) if name else None
        if orbit is None:
            # Nomear as opções: um frame_id errado é um erro de digitação no
            # cliente, e "câmera desconhecida" sem a lista manda a pessoa ler
            # código para descobrir o nome certo.
            self.get_logger().warning(
                f'câmera "{requested}" não existe; '
                f'use uma de {", ".join(CAMERAS)}')
            return
        orbit.apply(message.twist, following=self._is_following())
        self._apply_follow(orbit)
        self._push(name, orbit)

    def _on_reset(self, request, response):
        del request
        for name, orbit in self._orbits.items():
            orbit.reset()
            self._apply_follow(orbit)
            self._push(name, orbit)
        response.success = True
        response.message = 'vistas de cena de volta ao enquadramento inicial'
        return response

    def _on_odom(self, message):
        """
        Guarda a pose do robô já no referencial do mundo.

        A composição é `mundo = seed ∘ odom`, com o seed vindo dos parâmetros:
        rotaciona o ponto pelo yaw de spawn e depois translada. Aplicar só a
        translação estaria certo enquanto o spawn não gira — e é justamente o
        maze11 que gira (`yaw:=1.5708`).
        """
        position = message.pose.pose.position
        sx, sy, syaw = self._seed
        if syaw:
            cos_yaw, sin_yaw = math.cos(syaw), math.sin(syaw)
            x = cos_yaw * position.x - sin_yaw * position.y
            y = sin_yaw * position.x + cos_yaw * position.y
        else:
            x, y = position.x, position.y
        self._anchor = (x + sx, y + sy)

    def _on_follow(self, request, response):
        self._following = bool(request.data)
        self._announce_following()
        if not self._following:
            # Desligar deixa as câmeras EXATAMENTE onde estão, mirando o último
            # ponto seguido. Voltar ao enquadramento inicial é o que "recentrar"
            # faz, e fazer as duas coisas neste botão tiraria do operador a
            # única forma de congelar a vista que está boa.
            response.success = True
            response.message = 'vistas de cena paradas onde estão'
            return response

        # Ao religar, o pan acumulado no modo livre não tem sentido como offset
        # em relação ao robô: ele foi medido contra o mundo. Zerar é o que faz
        # "seguir" voltar a significar "robô no centro".
        for name, orbit in self._orbits.items():
            orbit.follow_offset = (0.0, 0.0)
            self._apply_follow(orbit)
            self._push(name, orbit)
        response.success = True
        response.message = 'vistas de cena seguindo o robô'
        return response

    def _on_follow_tick(self):
        if not self._is_following():
            return
        for name, orbit in self._orbits.items():
            before = orbit.position()
            self._apply_follow(orbit)
            after = orbit.position()
            # Banda morta sobre a POSIÇÃO DA CÂMERA e não sobre a do robô: na
            # vista iso um passo do robô move a câmera do mesmo tanto, mas na de
            # topo com pitch = pi/2 há razões para os dois números divergirem, e
            # o que decide se vale uma chamada é o que a câmera faz.
            if math.dist(before, after) < FOLLOW_DEADBAND_M:
                continue
            self._push(name, orbit)

    # --- estado ------------------------------------------------------------

    def _announce_following(self):
        self._following_pub.publish(Bool(data=self._following))

    def _is_following(self):
        """Seguir de verdade exige alvo: sem odometria não há o que seguir."""
        return self._following and self._anchor is not None

    def _apply_follow(self, orbit):
        if self._is_following():
            orbit.follow(self._anchor)

    # --- saída -------------------------------------------------------------

    def _push(self, name, orbit):
        if not self._set_pose.service_is_ready():
            # A ponte de serviços sobe junto com o Gazebo e pode demorar. Dizer
            # isso é melhor que enfileirar chamadas que ninguém vai atender.
            self.get_logger().warning(
                '/demo/sim/set_entity_pose ainda não existe; '
                'a ponte ros_gz do controle da simulação subiu?')
            return

        x, y, z = orbit.position()
        qx, qy, qz, qw = _quaternion(orbit.pitch, orbit.yaw)

        request = SetEntityPose.Request()
        request.entity.name = MODELS[name]
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = z
        request.pose.orientation.x = qx
        request.pose.orientation.y = qy
        request.pose.orientation.z = qz
        request.pose.orientation.w = qw

        future = self._set_pose.call_async(request)
        future.add_done_callback(lambda done: self._log_result(name, done))

    def _log_result(self, name, future):
        try:
            result = future.result()
        except Exception as error:  # noqa: BLE001 - queremos qualquer falha no log
            self.get_logger().error(f'set_pose de {name} falhou: {error}')
            return
        if not result.success:
            # O Gazebo devolve success=false quando o modelo não existe, e é
            # exatamente o que acontece quando o mundo subiu sem as câmeras.
            self.get_logger().warning(
                f'o Gazebo recusou mover "{name}"; o modelo foi spawnado?')


def main(args=None):
    rclpy.init(args=args)
    node = SceneViewController()
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
