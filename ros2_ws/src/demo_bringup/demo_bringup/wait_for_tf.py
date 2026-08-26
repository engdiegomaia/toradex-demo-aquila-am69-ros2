"""
Bloqueia ate uma aresta de TF existir de verdade, depois sai 0.

Roda no host x86 em learn, e no Aquila AM69 (arm64) em hil. Só CPU.

    ros2 run demo_bringup wait_for_tf --ros-args \
        -p parent_frame:=odom -p child_frame:=base -p timeout_s:=120.0

POR QUE ESTE NO EXISTE (ML3.5 F5, 26/08/2026)

Ele e irmao do `wait_for_clock`, e nasceu da mesma falha, um nivel adiante.

MEDIDO NO AQUILA AM69: depois de um `module.sh up`, o `local_costmap` passou 61 s
imprimindo

    Could not find a connection between 'odom' and 'base' because they are not
    part of the same tree. Tf has two or more unconnected trees.

e entao:

    Failed to activate local_costmap because transform from base to odom did not
    become available before timeout
    Failed to change state for node: controller_server
    Failed to bring up all requested nodes. Aborting bringup.

**O gerenciador de ciclo de vida aborta em DEFINITIVO e nao tenta de novo.** O
container fica de pe, os topicos todos aparecem, `scripts/module.sh verify`
retorna 0 -- e toda meta e recusada com "Action server is inactive", porque
`bt_navigator` nunca saiu de INACTIVE. Nada no caminho diz "TF".

A aresta em falta e `odom -> base`, que o `odom_tf` so publica quando a PRIMEIRA
mensagem de `/demo/odom` chega -- e essa mensagem vem do simulador, na OUTRA
maquina. Ou seja: o Nav2 estava apostando na velocidade de descoberta do DDS
entre containers. E a aposta as vezes perde. Depois do episodio medido a mesma
aresta estava viva a 50 Hz; o que faltou foi ordem, nao capacidade.

`wait_for_clock` documenta exatamente este raciocinio para o relogio, e o
`quadruped.launch.py` ja encadeia a subida dele por `OnProcessExit`, "cada elo
condicionado ao anterior terminar, e nao a tempo decorrido". Este no leva a mesma
disciplina para a TF, que era o elo que faltava.

POR QUE `Time()` E NAO O RELOGIO DO NO

A consulta usa `rclpy.time.Time()`, que em tf2 significa "o instante comum mais
recente" e nao depende do relogio deste no. Por isso ele roda com
`use_sim_time: False` de proposito: sem relogio a consultar, ele nao assina
`/clock` -- e assinar `/clock` a ~870 Hz para nao usar nenhuma mensagem foi
medido em 26/08 como 35-40% de um nucleo por no. Ver
`docs/results/ml35-f5-clock-fanout.md` e `test_sim_time_scope.py`.
"""

import sys

import rclpy
from rclpy.node import Node
import tf2_ros


# Passo de sondagem. 0.2 s e barato e mantem a latencia de destravamento bem
# abaixo do proprio tempo de descoberta do DDS.
POLL_PERIOD_S = 0.2


class WaitForTf(Node):
    """Sonda `can_transform` ate a aresta existir ou o prazo expirar."""

    def __init__(self) -> None:
        super().__init__('wait_for_tf')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base')
        self.declare_parameter('timeout_s', 120.0)

        self.parent = self.get_parameter('parent_frame').value
        self.child = self.get_parameter('child_frame').value
        self.timeout_s = float(self.get_parameter('timeout_s').value)

        self.buffer = tf2_ros.Buffer()
        # spin_thread=True: este no gira o executor a mao no laco abaixo, e o
        # listener precisa de uma thread propria para encher o buffer.
        self.listener = tf2_ros.TransformListener(self.buffer, self, spin_thread=True)

    def available(self) -> bool:
        return self.buffer.can_transform(
            self.parent, self.child, rclpy.time.Time())


def main(args=None) -> int:
    rclpy.init(args=args)
    node = WaitForTf()
    # Prazo em tempo de PAREDE, e nao simulado: este no existe justamente para o
    # caso em que o tempo simulado ainda nao atravessou a fronteira. Medir o
    # prazo no relogio que pode estar parado seria esperar para sempre -- o
    # mesmo argumento do `_wait` em nav_control_relay.py.
    import time
    deadline = time.monotonic() + node.timeout_s
    node.get_logger().info(
        'esperando TF %s -> %s (prazo %.0f s de tempo de parede)'
        % (node.parent, node.child, node.timeout_s))

    try:
        while rclpy.ok():
            if node.available():
                node.get_logger().info(
                    'TF %s -> %s disponivel. Liberando a subida do Nav2.'
                    % (node.parent, node.child))
                return 0
            if time.monotonic() > deadline:
                node.get_logger().error(
                    'TF %s -> %s NAO apareceu em %.0f s. Subir o Nav2 agora faz '
                    'o local_costmap falhar a ativacao e o gerenciador ABORTAR o '
                    'bringup em definitivo -- toda meta seria recusada com '
                    '"Action server is inactive". Confira se /demo/odom atravessa '
                    'a fronteira e se o odom_tf esta de pe.'
                    % (node.parent, node.child, node.timeout_s))
                return 1
            rclpy.spin_once(node, timeout_sec=POLL_PERIOD_S)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 1


if __name__ == '__main__':
    sys.exit(main())
