#!/usr/bin/env python3
"""
Le o costmap local e relata o perfil de custo transversal ao rumo do robo.

USA `costmap_raw` (nav2_msgs/Costmap), NAO `costmap` (OccupancyGrid).
O topico `costmap` e uma REESCALA: o Costmap2DPublisher do Nav2 mapeia
254 (letal) -> 100, 253 (inscrito) -> 99, 255 (desconhecido) -> -1 e o resto
para 1..98. Comparar com 253 ali nunca casa, e o resultado sai como "nenhuma
celula de colisao" num corredor cercado de parede. `costmap_raw` traz 0..254.

O que interessa e a faixa INSCRITA (253): o CostCritic do MPPI, com
consider_footprint false, trata custo >= 253 como COLISAO. A largura da faixa
que NAO e colisao e a largura util do corredor para o otimizador.
"""
import math
import sys

from nav2_msgs.msg import Costmap
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
import tf2_ros

INSCRIBED = 253
LETHAL = 254
UNKNOWN = 255


class Probe(Node):
    """Uma leitura do costmap local, mais a pose do robo por TF."""

    def __init__(self):
        super().__init__('costmap_probe')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.grid = None
        qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Costmap, '/local_costmap/costmap_raw',
                                 self._on_grid, qos)
        # O TransformListener NAO entra aqui. /tf e de alta taxa, e com
        # spin_once cada iteracao trata UM item: os callbacks de TF consomem
        # todas as iteracoes e a mensagem do costmap nunca e selecionada.
        self.buf = None
        self.listener = None

    def start_tf(self):
        """Liga o TF so depois do costmap, para nao competir por spin_once."""
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

    def _on_grid(self, msg):
        self.grid = msg


def main():
    rclpy.init()
    node = Probe()
    for _ in range(300):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.grid is not None:
            break
    if node.grid is None:
        print('sem costmap em /local_costmap/costmap_raw')
        return 1

    node.start_tf()
    # `base`, nao `base_link`: o URDF do Go2 nao tem base_link.
    for _ in range(200):
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.buf.can_transform('odom', 'base', rclpy.time.Time()):
            break

    meta = node.grid.metadata
    res = meta.resolution
    w, h = meta.size_x, meta.size_y
    ox, oy = meta.origin.position.x, meta.origin.position.y
    data = node.grid.data

    try:
        tf = node.buf.lookup_transform('odom', 'base', rclpy.time.Time())
    except Exception as exc:                                # noqa: BLE001
        print(f'sem TF odom->base: {exc}')
        return 1
    rx = tf.transform.translation.x
    ry = tf.transform.translation.y
    q = tf.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def cost_at(x, y):
        cx = int((x - ox) / res)
        cy = int((y - oy) / res)
        if not (0 <= cx < w and 0 <= cy < h):
            return None
        return data[cy * w + cx]

    print(f'costmap {w}x{h} @ {res:.3f} m, origem ({ox:.2f}, {oy:.2f})')
    print(f'robo em ({rx:.2f}, {ry:.2f}) yaw {math.degrees(yaw):.1f} deg')
    print(f'custo na celula do robo: {cost_at(rx, ry)}')

    # Perfil TRANSVERSAL ao rumo: e nele que a largura util aparece.  Perfil ao
    # longo do rumo nao mostra a parede lateral.
    nx, ny = -math.sin(yaw), math.cos(yaw)
    print('\nperfil transversal (esquerda -> direita), passo 0.05 m:')
    row, free, collide = [], 0, 0
    for i in range(-16, 17):
        d = i * 0.05
        c = cost_at(rx + nx * d, ry + ny * d)
        row.append(f'{d:+.2f}:{c}')
        if c == 0:
            free += 1
        if c is not None and INSCRIBED <= c <= LETHAL:
            collide += 1
    for i in range(0, len(row), 6):
        print('  ' + '  '.join(row[i:i + 6]))
    print(f'\nfaixa de custo ZERO      : {free * 0.05:.2f} m')
    print(f'faixa >= 253 (COLISAO)   : {collide * 0.05:.2f} m'
          f' de 1.60 m sondados')

    # Fracao do costmap que o critico chama de colisao: e o que decide se a
    # media ponderada das 1000 amostras tem trajetoria boa para pesar.
    known = [v for v in data if v != UNKNOWN]
    bad = [v for v in known if v >= INSCRIBED]
    lethal = [v for v in known if v == LETHAL]
    zero = [v for v in known if v == 0]
    print(f'\ncelulas conhecidas       : {len(known)}')
    print(f'  == 0 (livre)           : {len(zero)}'
          f'  ({100.0 * len(zero) / max(1, len(known)):.1f}%)')
    print(f'  >= 253 (colisao)       : {len(bad)}'
          f'  ({100.0 * len(bad) / max(1, len(known)):.1f}%)')
    print(f'  == 254 (letal)         : {len(lethal)}')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
