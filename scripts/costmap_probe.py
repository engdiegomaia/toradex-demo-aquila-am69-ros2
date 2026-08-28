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
import argparse
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

# Extensao SONDADA, em metros.  Sao estes numeros que definem a medicao; o passo
# sai da resolucao do costmap em tempo de execucao.  0,80 m para cada lado cobre
# o corredor de 1,20 m do maze11 com margem, e 4,00 m a frente cobre o alcance
# em que o L1 marca obstaculo.
TRANSVERSE_HALF_SPAN_M = 0.80
FORWARD_SPAN_M = 4.00


class Probe(Node):
    """Uma leitura do costmap local, mais a pose do robo por TF."""

    def __init__(self, topic):
        super().__init__('costmap_probe')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.grid = None
        qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Costmap, topic,
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
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--scope', choices=('local', 'global'), default='local',
        help='costmap a sondar (default: local)')
    args = parser.parse_args()
    topic = f'/{args.scope}_costmap/costmap_raw'

    rclpy.init()
    node = Probe(topic)
    for _ in range(300):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.grid is not None:
            break
    if node.grid is None:
        print(f'sem costmap em {topic}')
        return 1

    node.start_tf()
    # `base`, nao `base_link`: o URDF do Go2 nao tem base_link.
    for _ in range(200):
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.buf.can_transform(node.grid.header.frame_id, 'base',
                                  rclpy.time.Time()):
            break

    meta = node.grid.metadata
    res = meta.resolution
    w, h = meta.size_x, meta.size_y
    ox, oy = meta.origin.position.x, meta.origin.position.y
    data = node.grid.data

    try:
        frame = node.grid.header.frame_id
        tf = node.buf.lookup_transform(frame, 'base', rclpy.time.Time())
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

    print(f'{topic} frame={frame} {w}x{h} @ {res:.3f} m, '
          f'origem ({ox:.2f}, {oy:.2f})')
    print(f'robo em ({rx:.2f}, {ry:.2f}) yaw {math.degrees(yaw):.1f} deg')
    print(f'custo na celula do robo: {cost_at(rx, ry)}')

    # Perfil TRANSVERSAL ao rumo: e nele que a largura util aparece.  Perfil ao
    # longo do rumo nao mostra a parede lateral.
    nx, ny = -math.sin(yaw), math.cos(yaw)
    # O passo vem do costmap, NAO de um 0.05 cravado.  O costmap global passou a
    # 10 cm em 28/08/2026 (janela de 40 m com a mesma grade de 400 celulas por
    # eixo); um passo fixo de 5 cm amostraria cada celula duas vezes e as linhas
    # de "faixa" abaixo reportariam METADE da distancia real.  O que e constante
    # nesta medicao sao os limites em METROS, nao a contagem de celulas.
    print(f'\nperfil transversal (esquerda -> direita), passo {res:.3f} m:')
    steps = max(1, int(round(TRANSVERSE_HALF_SPAN_M / res)))
    row, free, collide = [], 0, 0
    for i in range(-steps, steps + 1):
        d = i * res
        c = cost_at(rx + nx * d, ry + ny * d)
        row.append(f'{d:+.2f}:{c}')
        if c == 0:
            free += 1
        if c is not None and INSCRIBED <= c <= LETHAL:
            collide += 1
    for i in range(0, len(row), 6):
        print('  ' + '  '.join(row[i:i + 6]))
    print(f'\nfaixa de custo ZERO      : {free * res:.2f} m')
    print(f'faixa >= 253 (COLISAO)   : {collide * res:.2f} m'
          f' de {2.0 * TRANSVERSE_HALF_SPAN_M:.2f} m sondados')

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

    # O achado que abriu esta medicao foi uma celula inscrita a 0,55 m em +y
    # do mundo, embora a geometria offline indique pista livre por 3,47 m.
    # Imprimir apenas o perfil transversal ao rumo nao reproduz esse achado.
    print(f'\nperfil em +y do frame do costmap, passo {res:.3f} m:')
    last = object()
    runs = []
    first_collision = None
    first_lethal = None
    for i in range(int(round(FORWARD_SPAN_M / res)) + 1):
        d = i * res
        c = cost_at(rx, ry + d)
        if c != last:
            runs.append((d, c))
            last = c
        if first_collision is None and c is not None and INSCRIBED <= c <= LETHAL:
            first_collision = (d, c)
        if first_lethal is None and c == LETHAL:
            first_lethal = d
    print('  mudancas: ' + '  '.join(f'{d:.2f}m:{c}' for d, c in runs))
    print('  primeira >= 253: '
          + (f'{first_collision[0]:.2f} m (custo {first_collision[1]})'
             if first_collision else 'nenhuma ate 4.00 m'))
    print('  primeira == 254: '
          + (f'{first_lethal:.2f} m' if first_lethal is not None
             else 'nenhuma ate 4.00 m'))

    # Uma parede lateral produz muitas celulas >=253 alinhadas ao corredor; um
    # retorno da propria perna produz um pequeno aglomerado junto ao robo. As
    # celulas mais proximas tornam essa diferenca visivel sem depender do RViz.
    nearest = []
    for cy in range(h):
        for cx in range(w):
            c = data[cy * w + cx]
            if INSCRIBED <= c <= LETHAL:
                x = ox + (cx + 0.5) * res
                y = oy + (cy + 0.5) * res
                dx, dy = x - rx, y - ry
                nearest.append((math.hypot(dx, dy), x, y, c,
                                math.degrees(math.atan2(dy, dx))))
    nearest.sort()
    print('\n10 celulas >=253 mais proximas:')
    for distance, x, y, cost, bearing in nearest[:10]:
        print(f'  d={distance:.3f} m  ({x:+.2f},{y:+.2f})  '
              f'rumo={bearing:+.1f} deg  custo={cost}')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
