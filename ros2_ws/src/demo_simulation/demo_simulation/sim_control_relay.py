"""
Fachada std_srvs para o controle do simulador.

Roda em: workstation x86 SOMENTE, no container `sim`, ao lado do
`sim_control_bridge`.

    /demo/sim/play             std_srvs/Trigger   (entra)
    /demo/sim/pause            std_srvs/Trigger   (entra)
    /demo/sim/reset            std_srvs/Trigger   (entra)
    /demo/sim/control          ros_gz_interfaces/srv/ControlWorld   (sai)
    /demo/sim/set_entity_pose  ros_gz_interfaces/srv/SetEntityPose  (sai)

POR QUE O RESET NAO USA MAIS `reset.all`

`reset.all` APAGA O ROBO. Medido em 26/08/2026, mundo `quadruped_maze11`, pilha
de pe, com uma unica chamada a /demo/sim/reset:

    topico                      antes      depois
    /joint_states             999 Hz       morto
    /demo/imu                 996 Hz       morto
    /demo/odom               49,6 Hz       morto
    /demo/scan                 10 Hz        10 Hz
    /demo/camera/image_raw     10 Hz        10 Hz
    /clock                    999 Hz       997 Hz

    $ gz model -m demo_robot
    No model named <demo_robot> was found

O robo e INSERIDO no mundo depois da carga, por `ros_gz_sim create`
(quadruped.launch.py). `reset.all` devolve o mundo ao SDF de origem, e o SDF de
origem nao contem o robo -- nem as duas cameras de cena, que tambem sao
inseridas. O que sobra e um mundo sem planta.

E o modo de falha e o pior possivel: o relogio continua andando a 999 Hz, o
lidar e a camera continuam publicando a 10 Hz (o Gazebo deixa os sensores
orfaos publicando), entao o cockpit fica INTEIRO verde -- video, cena, mapa,
relogio -- apontando para um robo que nao existe mais. Nada em log nenhum diz
que a planta foi apagada. Recuperar exige reiniciar o container `sim`.

O QUE O RESET FAZ AGORA

Teleporta o robo para a pose de nascimento do cenario, via o mesmo
`set_entity_pose` que as cameras de cena ja usam. Medido no mesmo dia:

    depois do teleporte:  /joint_states 999 Hz, /demo/imu 982 Hz, /demo/odom 50 Hz
    pose lida em /demo/odom: x=1.000 (comandado x=1.0)

A planta sobrevive inteira e o robo reassenta na altura de marcha por conta
propria. O relogio NAO volta a zero, e isso e deliberado: um salto de tempo
para tras invalida o buffer de TF do Nav2 e o `controller_manager`, e nada no
que o operador quer de um reset ("poe o robo no inicio") pede isso.

O costmap acumulado NAO e limpo aqui. Quem limpa e /demo/nav/reset, que o
cockpit expoe no proprio botao de reiniciar navegacao -- a granularidade
separada e o que permite recolocar o robo sem derrubar o Nav2.

POR QUE NÃO CHAMAR ControlWorld DIRETO DO NAVEGADOR

Foi a primeira tentativa e ela falha, com uma mensagem que vale registrar:

    call_service InvalidModuleException: Unable to import ros_gz_interfaces.srv
    from package ros_gz_interfaces

O rosbridge monta o pedido importando o pacote de interfaces DENTRO do próprio
container, e o container do cockpit não tem `ros_gz_interfaces` — nem deveria.
No M3 o cockpit é servido pelo Aquila, e no modo `deploy` não existe Gazebo
nenhum: instalar as interfaces do simulador ali seria carregar para o módulo a
definição de uma coisa que, naquele modo, não existe.

Então a fronteira do navegador fala std_srvs, que é núcleo do ROS e está em
qualquer container, e a tradução para o vocabulário do Gazebo acontece aqui —
do lado que já tem o Gazebo. É a mesma escolha que o `scene_view_controller`
faz para as câmeras: o navegador manda intenção, o simulador tem o tipo.

O `success=False` da resposta é o caminho útil desta fachada. Sem ela, um
simulador parado devolvia uma exceção de import para a UI, e o operador via
"falha ao pausar" com a causa errada.
"""

import math
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.srv import ControlWorld, SetEntityPose
from std_srvs.srv import Trigger

CONTROL_SERVICE = '/demo/sim/control'
SET_POSE_SERVICE = '/demo/sim/set_entity_pose'

# Quanto esperar a ponte responder. O Gazebo responde a ControlWorld em poucos
# milissegundos; este limite existe para o caso em que o serviço não existe do
# outro lado, e nele o que importa é falhar rápido o suficiente para que o botão
# não pareça travado.
CALL_TIMEOUT_S = 3.0


def _request(action: str) -> ControlWorld.Request:
    """
    Traduz a intenção para o vocabulário do WorldControl.

    Só play e pause: `reset` não passa mais por aqui. Ver o bloco POR QUE O
    RESET NAO USA MAIS `reset.all` no cabeçalho — a variante `reset.all` apaga
    o robô, e `time_only` salta o relógio para trás sem mover o robô, que é
    pior do que não fazer nada.
    """
    request = ControlWorld.Request()
    if action == 'play':
        request.world_control.pause = False
    elif action == 'pause':
        request.world_control.pause = True
    else:
        raise ValueError(f'ação desconhecida no WorldControl: {action}')
    return request


class SimControlRelay(Node):

    def __init__(self) -> None:
        super().__init__('sim_control_relay')

        # Grupo reentrante: os callbacks de Trigger BLOQUEIAM esperando a
        # resposta do ControlWorld. No grupo mutuamente exclusivo padrão essa
        # espera impediria o executor de processar a própria resposta, e todo
        # clique expiraria.
        self._group = ReentrantCallbackGroup()

        self._client = self.create_client(
            ControlWorld, CONTROL_SERVICE, callback_group=self._group,
        )
        self._pose_client = self.create_client(
            SetEntityPose, SET_POSE_SERVICE, callback_group=self._group,
        )

        # Pose de nascimento e nome do modelo. Vêm de PARÂMETRO, e quem os
        # preenche é sim_control.launch.py a partir de `scenarios.spawn_pose`:
        # o mesmo lugar de onde saem o `-x/-y/-Y` do `create` que nasceu o robô.
        # Cravar (0, 0) aqui devolveria o robô para a origem em qualquer mundo
        # cuja área útil não está na origem — o labirinto é exatamente esse
        # caso, e o erro seria silencioso (o robô reaparece dentro de parede).
        self.declare_parameter('robot_name', 'demo_robot')
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        # Não é a altura de marcha: é a altura de NASCIMENTO, a mesma do
        # `-z` do create. Um quadrúpede posto no nível do chão interpenetra o
        # solo e cai antes de o controlador estabilizar.
        self.declare_parameter('spawn_z', 0.5)
        self.declare_parameter('spawn_yaw', 0.0)

        for action in ('play', 'pause'):
            self.create_service(
                Trigger,
                f'/demo/sim/{action}',
                self._handler(action),
                callback_group=self._group,
            )
        self.create_service(
            Trigger, '/demo/sim/reset', self._handle_reset,
            callback_group=self._group,
        )

        self.get_logger().info(
            f'fachada de simulação pronta: play/pause -> {CONTROL_SERVICE}, '
            f'reset -> {SET_POSE_SERVICE} '
            f'(teleporta {self._robot_name()} para a pose de nascimento)'
        )

    def _robot_name(self) -> str:
        return str(self.get_parameter('robot_name').value)

    def _spawn(self) -> tuple:
        """(x, y, z, yaw) de nascimento, lidos no instante do clique."""
        return (
            float(self.get_parameter('spawn_x').value),
            float(self.get_parameter('spawn_y').value),
            float(self.get_parameter('spawn_z').value),
            float(self.get_parameter('spawn_yaw').value),
        )

    def _handle_reset(self, request, response):
        """
        Devolve o robô à pose de nascimento SEM apagá-lo.

        Não zera o relógio e não toca no mundo. O costmap acumulado é assunto
        de /demo/nav/reset, que tem botão próprio no cockpit.
        """
        del request
        if not self._pose_client.wait_for_service(timeout_sec=CALL_TIMEOUT_S):
            response.success = False
            response.message = (
                f'{SET_POSE_SERVICE} não respondeu; a ponte ros_gz do controle '
                'da simulação subiu?'
            )
            return response

        x, y, z, yaw = self._spawn()
        name = self._robot_name()

        pose_request = SetEntityPose.Request()
        pose_request.entity.name = name
        pose_request.pose.position.x = x
        pose_request.pose.position.y = y
        pose_request.pose.position.z = z
        # Guinada pura: um quadrúpede reposto com roll ou pitch cai.
        pose_request.pose.orientation.z = math.sin(yaw / 2.0)
        pose_request.pose.orientation.w = math.cos(yaw / 2.0)

        future = self._pose_client.call_async(pose_request)
        if not _wait(future, CALL_TIMEOUT_S):
            response.success = False
            response.message = f'{SET_POSE_SERVICE} expirou ao repor {name}'
            return response

        result = future.result()
        # O Gazebo devolve success=False quando NÃO EXISTE entidade com esse
        # nome. É o caminho útil desta resposta: diz ao operador que o modelo
        # não está no mundo, em vez de deixar o botão silencioso.
        response.success = bool(result and result.success)
        response.message = (
            f'{name} reposto em x={x:.3f} y={y:.3f} yaw={yaw:.4f}'
            if response.success
            else f'o Gazebo recusou repor {name}: existe um modelo com esse nome?'
        )
        if not response.success:
            self.get_logger().warning(response.message)
        return response

    def _handler(self, action: str):
        def handle(request, response):
            del request
            if not self._client.wait_for_service(timeout_sec=CALL_TIMEOUT_S):
                response.success = False
                response.message = (
                    f'{CONTROL_SERVICE} não respondeu; o container sim está no ar?'
                )
                return response

            future = self._client.call_async(_request(action))
            if not _wait(future, CALL_TIMEOUT_S):
                response.success = False
                response.message = f'{CONTROL_SERVICE} expirou ao {action}'
                return response

            result = future.result()
            response.success = bool(result and result.success)
            response.message = (
                f'{action} aplicado' if response.success
                else f'o Gazebo recusou {action}'
            )
            return response

        return handle


def _wait(future, timeout_s: float) -> bool:
    """
    Espera o future sem girar o executor.

    `spin_until_future_complete` e `spin_once` NÃO servem aqui: já estamos
    dentro de um callback do executor, e girá-lo de novo sobre o mesmo nó é
    reentrância sobre o objeto errado. Quem completa este future é outra thread
    do MultiThreadedExecutor — o grupo reentrante existe exatamente para
    permitir isso — então o que resta a esta thread é dormir e olhar.

    O relógio é o de parede, de propósito: este código pode estar esperando o
    resultado de PAUSAR a simulação, e o relógio simulado é a única coisa que
    com certeza não vai avançar depois disso.
    """
    deadline = time.monotonic() + timeout_s
    while not future.done():
        if time.monotonic() > deadline:
            return False
        time.sleep(0.01)
    return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimControlRelay()
    # Multithreaded para valer o grupo reentrante acima: com o executor de uma
    # thread só, o callback bloqueado seria a única thread disponível.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
