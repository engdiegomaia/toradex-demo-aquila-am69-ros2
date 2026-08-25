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

CONTRATO

    /demo/cockpit/scene/cmd_view    geometry_msgs/TwistStamped   (entra)
    /demo/cockpit/scene/reset_view  std_srvs/Trigger             (entra)
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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import rclpy
from ros_gz_interfaces.srv import SetEntityPose
from std_srvs.srv import Trigger

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

    def position(self):
        reach = self.distance * math.cos(self.pitch)
        return (self.target[0] - reach * math.cos(self.yaw),
                self.target[1] - reach * math.sin(self.yaw),
                self.distance * math.sin(self.pitch))

    def apply(self, twist):
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
        tx = self.target[0] + right[0] * twist.linear.y + forward[0] * twist.linear.z
        ty = self.target[1] + right[1] * twist.linear.y + forward[1] * twist.linear.z

        radius = math.hypot(tx, ty)
        if radius > MAX_TARGET_RADIUS_M:
            scale = MAX_TARGET_RADIUS_M / radius
            tx, ty = tx * scale, ty * scale
        self.target = (tx, ty)


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

        def orbit(prefix):
            value = lambda name: self.get_parameter(f'{prefix}_{name}').value
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

        framing = ', '.join(
            f'{name} em ({x:.2f}, {y:.2f}, {z:.2f})'
            for name, (x, y, z) in
            ((n, o.position()) for n, o in self._orbits.items()))
        # rclpy nao tem logging no estilo printf: o RcutilsLogger aceita UMA
        # string. Passar args posicionais levanta TypeError na construcao do no,
        # que morre antes de existir para qualquer diagnostico.
        self.get_logger().info(f'controle de vista pronto: {framing}')

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
        orbit.apply(message.twist)
        self._push(name, orbit)

    def _on_reset(self, request, response):
        del request
        for name, orbit in self._orbits.items():
            orbit.reset()
            self._push(name, orbit)
        response.success = True
        response.message = 'vistas de cena de volta ao enquadramento inicial'
        return response

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
