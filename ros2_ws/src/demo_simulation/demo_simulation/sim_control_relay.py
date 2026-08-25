"""
Fachada std_srvs para o controle do simulador.

Roda em: workstation x86 SOMENTE, no container `sim`, ao lado do
`sim_control_bridge`.

    /demo/sim/play     std_srvs/Trigger   (entra)
    /demo/sim/pause    std_srvs/Trigger   (entra)
    /demo/sim/reset    std_srvs/Trigger   (entra)
    /demo/sim/control  ros_gz_interfaces/srv/ControlWorld   (sai, para a ponte)

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

import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.srv import ControlWorld
from std_srvs.srv import Trigger

CONTROL_SERVICE = '/demo/sim/control'

# Quanto esperar a ponte responder. O Gazebo responde a ControlWorld em poucos
# milissegundos; este limite existe para o caso em que o serviço não existe do
# outro lado, e nele o que importa é falhar rápido o suficiente para que o botão
# não pareça travado.
CALL_TIMEOUT_S = 3.0


def _request(action: str) -> ControlWorld.Request:
    """
    Traduz a intenção para o vocabulário do WorldControl.

    `reset.all` reinicia tempo E modelos. É a única variante que devolve o robô
    à pose inicial, que é o que "resetar a simulação" significa para quem aperta
    o botão — `time_only` reinicia o relógio e deixa o robô onde estava, o que
    é pior do que não fazer nada.
    """
    request = ControlWorld.Request()
    if action == 'play':
        request.world_control.pause = False
    elif action == 'pause':
        request.world_control.pause = True
    elif action == 'reset':
        request.world_control.reset.all = True
    else:
        raise ValueError(f'ação desconhecida: {action}')
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

        for action in ('play', 'pause', 'reset'):
            self.create_service(
                Trigger,
                f'/demo/sim/{action}',
                self._handler(action),
                callback_group=self._group,
            )

        self.get_logger().info(
            'fachada de simulação pronta: /demo/sim/{play,pause,reset} -> '
            f'{CONTROL_SERVICE}'
        )

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
