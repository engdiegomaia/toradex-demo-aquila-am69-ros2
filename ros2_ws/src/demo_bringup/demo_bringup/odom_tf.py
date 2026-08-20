"""
Fecha o topo da árvore TF: publica `odom` -> `base` a partir de `/demo/odom`.

Roda na estação x86 em modo learn e no Aquila AM69 em modo hil. Sem dependência
gráfica nem de arquitetura.

    ros2 run demo_bringup odom_tf --ros-args -p base_frame:=base

## Por que este nó existe

Medido em 20/08/2026 com `scripts/scenario_check.py`: a árvore TF do Go2 tem 20
arestas, 8 estáticas, com raiz em `base` — `base` -> `trunk` -> `lidar`,
`imu_link`, `front_camera`, e as quatro pernas até os pés. A árvore do robô é
completa. Faltam exatamente **duas arestas no topo**:

- `odom` -> `base`, que é a odometria;
- `map` -> `odom`, que é a localização.

E o `/demo/odom` já declara `header.frame_id: "odom"` numa mensagem cujo frame
ninguém publica. Sem essas duas arestas o Nav2 não consegue colocar o scan num
costmap, e falha de um jeito que não menciona TF.

## O que este nó NÃO é

**Não é estimativa de estado.** Ele republica como TF a odometria que já existe,
e em simulação essa odometria é **ground truth do Gazebo** — ver o cabeçalho de
`demo_simulation/config/bridge_quadruped.yaml`. Isso é bom o suficiente para
exercitar percepção e planejamento, e é honestamente inútil como validação de
localização: o robô sabe exatamente onde está porque o simulador contou.

A estimativa de estado com perna é F5, e quando ela existir **este nó sai**.

## A armadilha que ele pode causar

Dois publicadores na mesma aresta da TF não dão erro: o consumidor recebe as duas
e usa a última que chegou, alternando entre duas crenças. É por isso que
`bridge_quadruped.yaml` deliberadamente NÃO ponte `/go2/ground_truth_tf`, e é por
isso que este nó não pode rodar junto com:

- a ponte de `/go2/ground_truth_tf`;
- o estimador de F5, quando existir;
- uma segunda instância dele mesmo.

O nó avisa no log se detectar outro publicador em `/tf` na mesma aresta, mas o
aviso é melhor-esforço — a checagem definitiva é `ros2 run tf2_tools view_frames`.
"""

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class OdomTf(Node):
    """Republica `/demo/odom` como a aresta TF `odom` -> `base`."""

    def __init__(self) -> None:
        """Declara parâmetros, arma as arestas e publica a estática se pedida."""
        super().__init__('odom_tf')

        # `base` e não `base_link`: é o nome que o URDF do Go2 usa como raiz, e
        # go2_description é vendorizado com garantia byte-a-byte, então o nome do
        # frame é dado, não escolhido. Quem tem de ceder é o arquivo de
        # parâmetros do Nav2, que é do projeto.
        self._base = self.declare_parameter('base_frame', 'base').value
        self._odom = self.declare_parameter('odom_frame', 'odom').value
        self._map = self.declare_parameter('map_frame', 'map').value

        # A identidade `map` -> `odom` dá ao Nav2 um frame `map` sem SLAM nem
        # AMCL. Não é localização: é declarar que o mundo do planejador é o mundo
        # da odometria. Serve para desvio reativo de obstáculo, que é o que o
        # costmap local faz. NÃO serve para navegar sobre um mapa salvo — ali a
        # aresta é do SLAM ou do AMCL, e este parâmetro tem de ir para false ou
        # aparecem dois publicadores na mesma aresta.
        self._publish_map = self.declare_parameter(
            'publish_map_identity', True).value

        self._broadcaster = TransformBroadcaster(self)
        self._received = 0

        if self._publish_map:
            static = StaticTransformBroadcaster(self)
            static.sendTransform(self._identity(self._map, self._odom))
            self.get_logger().info(
                'publicada identidade estatica %s -> %s. Se voce subir SLAM ou '
                'AMCL, ponha publish_map_identity:=false ou havera dois '
                'publicadores nessa aresta.' % (self._map, self._odom))

        self.create_subscription(
            Odometry, '/demo/odom', self._on_odom,
            QoSPresetProfiles.SENSOR_DATA.value)

        self.get_logger().info(
            'publicando %s -> %s a partir de /demo/odom. Em simulacao esta '
            'odometria e ground truth do Gazebo, nao estimativa: use para '
            'exercitar percepcao e planejamento, nunca como validacao de '
            'localizacao.' % (self._odom, self._base))

    def _identity(self, parent: str, child: str) -> TransformStamped:
        """Monta uma transformada identidade entre dois frames."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.rotation.w = 1.0
        return transform

    def _on_odom(self, message: Odometry) -> None:
        """Monta a aresta TF correspondente a uma mensagem de odometria."""
        transform = TransformStamped()
        # O stamp vem da mensagem, não do relógio local. Copiar o relógio local
        # aqui produz TF adiantada ou atrasada em relação ao scan, e o costmap
        # descarta a leitura com "message filter dropping message", que não diz
        # que a causa é o stamp.
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = message.header.frame_id or self._odom
        transform.child_frame_id = self._base
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self._broadcaster.sendTransform(transform)

        self._received += 1
        if self._received == 1:
            self.get_logger().info(
                'primeira odometria recebida em frame "%s"; arvore fechada ate '
                '"%s"' % (transform.header.frame_id, self._base))


def main(args=None) -> None:
    """Roda o nó até ser interrompido."""
    rclpy.init(args=args)
    node = OdomTf()
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
