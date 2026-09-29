#!/usr/bin/env python3
"""
Verifica o contrato de tópicos de um cenário de simulação e imprime um relatório.

Roda no host x86, contra a simulação em container (`--network=host`,
`ROS_DOMAIN_ID=69`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`). Não publica nada:
é só leitura, então pode rodar junto com qualquer roteiro de movimento.

    python3 tools/diagnostics/scenario_check.py --seconds 20
    python3 tools/diagnostics/scenario_check.py --seconds 20 --json relatorio.json

O que ele mede, e por quê cada coisa está aqui:

- **Taxa de cada tópico.** Um tópico presente mas a 0 Hz é o modo de falha mais
  comum e mais silencioso desta simulação: a bridge sobe, o `ros2 topic list`
  mostra o nome, e nada nunca chega. Contar mensagens numa janela é a única
  verificação que pega isso.
- **`/clock`.** Se o tempo de simulação não avança, todo nó com `use_sim_time`
  congela sem erro. É a primeira coisa a olhar quando "nada acontece".
- **Conteúdo do lidar**, não só a taxa: um scan de 100% de infinitos é o que se
  vê quando o mundo não tem nada na altura do plano de varredura, e isso é
  indistinguível de sensor quebrado se só se olhar Hz.
- **Geometria da imagem** contra o `camera_info`. Divergência aqui é o que
  quebra qualquer inferência a jusante, e é invisível na taxa.
- **A aresta `odom` -> `base_link` da TF.** Sabidamente ausente hoje (bloqueio
  de F5). Está aqui para que o relatório diga isso explicitamente em vez de
  alguém descobrir de novo depurando o Nav2.

Nenhum resultado deste script vale para o Aquila AM69 real: ele mede a
simulação no host.
"""

import argparse
import json
import math
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSPresetProfiles, QoSProfile,
                       ReliabilityPolicy)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection2DArray

# Taxa mínima aceitável por tópico. Não são alvos de projeto: são o piso abaixo
# do qual o consumidor a jusante quebra de forma observável.
#
#   /clock      -- abaixo de ~50 Hz o tempo de simulação anda aos saltos e os
#                  temporizadores de qualquer nó com use_sim_time tremem.
#   /demo/odom  -- 10 Hz é o que o Nav2 assume para odometria.
#   /demo/scan  -- 5 Hz é o piso do costmap; abaixo disso o obstáculo aparece
#                  depois de o robô já estar nele.
#   /demo/imu   -- o controlador roda a 500 Hz e consome IMU internamente; o que
#                  a bridge expõe é diagnóstico, então 50 Hz basta.
FLOORS = {
    '/clock': 50.0,
    '/demo/odom': 10.0,
    '/demo/scan': 5.0,
    '/demo/camera/image_raw': 5.0,
    '/demo/camera/camera_info': 1.0,
    '/demo/imu': 50.0,
}

# Estes podem estar legitimamente ausentes: só existem se o nó correspondente
# estiver rodando. Ausência é informação, não falha.
OPTIONAL = {'/demo/perception/detections', '/demo/cmd_vel', '/tf'}


class ScenarioCheck(Node):
    """Conta mensagens e guarda a última de cada tópico do contrato."""

    def __init__(self, seconds: float) -> None:
        """Assina todo o contrato e arma a janela de medição."""
        super().__init__('scenario_check')
        self._seconds = seconds
        self.counts: dict = {}
        self.last: dict = {}
        self.tf_edges: set = set()

        sensor = QoSPresetProfiles.SENSOR_DATA.value
        self._sub(Clock, '/clock', 10)
        self._sub(Odometry, '/demo/odom', sensor)
        self._sub(LaserScan, '/demo/scan', sensor)
        self._sub(Image, '/demo/camera/image_raw', sensor)
        self._sub(CameraInfo, '/demo/camera/camera_info', sensor)
        self._sub(Imu, '/demo/imu', sensor)
        self._sub(Detection2DArray, '/demo/perception/detections', 10)
        self._sub(Twist, '/demo/cmd_vel', 10)
        self.counts['/tf'] = 0
        self.create_subscription(TFMessage, '/tf', self._on_tf, 100)
        # /tf_static usa durabilidade TRANSIENT_LOCAL: as arestas fixas sao
        # publicadas UMA vez, na subida do robot_state_publisher, e retidas para
        # quem assinar depois. Com QoS padrao (VOLATILE) um assinante tardio nao
        # recebe nada e conclui que a arvore nao tem as juntas fixas -- foi
        # exatamente o erro que este script cometeu na primeira versao.
        self.create_subscription(
            TFMessage, '/tf_static', self._on_tf_static,
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.tf_static_edges: set = set()

        self._wall0 = time.monotonic()

    def _sub(self, msg_type, topic, qos) -> None:
        self.counts[topic] = 0
        self.create_subscription(
            msg_type, topic, lambda m, t=topic: self._on(t, m), qos)

    def _on(self, topic: str, message) -> None:
        self.counts[topic] += 1
        self.last[topic] = message

    def _on_tf(self, message: TFMessage) -> None:
        self.counts['/tf'] += 1
        for transform in message.transforms:
            self.tf_edges.add(
                (transform.header.frame_id, transform.child_frame_id))

    def _on_tf_static(self, message: TFMessage) -> None:
        for transform in message.transforms:
            edge = (transform.header.frame_id, transform.child_frame_id)
            self.tf_edges.add(edge)
            self.tf_static_edges.add(edge)

    def done(self) -> bool:
        """Diz se a janela de medição fechou."""
        return time.monotonic() - self._wall0 >= self._seconds

    def elapsed(self) -> float:
        """Devolve o tempo de parede decorrido na janela."""
        return time.monotonic() - self._wall0


def scan_report(scan: LaserScan) -> dict:
    """Resume um scan: quantos retornos são válidos e qual a faixa medida."""
    valid = [r for r in scan.ranges
             if not math.isinf(r) and not math.isnan(r)
             and scan.range_min <= r <= scan.range_max]
    return {
        'feixes': len(scan.ranges),
        'validos': len(valid),
        'validos_pct': 100.0 * len(valid) / len(scan.ranges) if scan.ranges else 0.0,
        'min_m': min(valid) if valid else None,
        'max_m': max(valid) if valid else None,
        'faixa_sensor_m': [scan.range_min, scan.range_max],
    }


def image_report(image: Image, info) -> dict:
    """Resume a imagem e confronta a geometria com o camera_info."""
    out = {
        'largura': image.width,
        'altura': image.height,
        'encoding': image.encoding,
        'bytes': len(image.data),
    }
    if image.data:
        amostra = image.data[::max(1, len(image.data) // 4096)]
        out['intensidade_media'] = round(sum(amostra) / len(amostra), 1)
    if info is not None:
        out['camera_info'] = [info.width, info.height]
        out['geometria_confere'] = (info.width == image.width
                                   and info.height == image.height)
    return out


def build_report(node: ScenarioCheck) -> dict:
    """Monta o relatório completo a partir do que foi contado."""
    window = node.elapsed()
    topics = {}
    for topic, count in sorted(node.counts.items()):
        hz = count / window if window > 0 else 0.0
        floor = FLOORS.get(topic)
        if count == 0:
            estado = 'AUSENTE' if topic in OPTIONAL else 'SEM DADOS'
        elif floor is None:
            estado = 'ok'
        else:
            estado = 'ok' if hz >= floor else 'LENTO'
        topics[topic] = {'msgs': count, 'hz': round(hz, 1),
                         'piso_hz': floor, 'estado': estado}

    report = {'janela_s': round(window, 1), 'topicos': topics}

    clock = node.last.get('/clock')
    if clock is not None:
        report['sim_time_s'] = round(
            clock.clock.sec + clock.clock.nanosec / 1e9, 2)

    odom = node.last.get('/demo/odom')
    if odom is not None:
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y ** 2 + q.z ** 2))
        report['pose'] = {'x': round(p.x, 3), 'y': round(p.y, 3),
                          'z': round(p.z, 3),
                          'yaw_deg': round(math.degrees(yaw), 1)}
        report['frame_odom'] = odom.header.frame_id or '(vazio)'

    scan = node.last.get('/demo/scan')
    if scan is not None:
        report['scan'] = scan_report(scan)

    image = node.last.get('/demo/camera/image_raw')
    if image is not None:
        report['camera'] = image_report(
            image, node.last.get('/demo/camera/camera_info'))

    report['tf_arestas'] = sorted('%s -> %s' % e for e in node.tf_edges)
    report['tf_estaticas'] = sorted(
        '%s -> %s' % e for e in node.tf_static_edges)
    report['tf_fecha_odom'] = any(
        pai == 'odom' for pai, _ in node.tf_edges)
    filhos = {c for _, c in node.tf_edges}
    pais = {p for p, _ in node.tf_edges}
    report['tf_raizes'] = sorted(pais - filhos)
    return report


def print_report(report: dict) -> int:
    """Imprime o relatório e devolve o número de problemas encontrados."""
    problemas = 0
    print('\n=== contrato de tópicos (janela de %.1f s) ===' % report['janela_s'])
    print('%-32s %8s %8s %10s' % ('tópico', 'msgs', 'Hz', 'estado'))
    for topic, info in report['topicos'].items():
        piso = '' if info['piso_hz'] is None else ' (piso %g)' % info['piso_hz']
        print('%-32s %8d %8.1f %10s%s'
              % (topic, info['msgs'], info['hz'], info['estado'], piso))
        if info['estado'] in ('SEM DADOS', 'LENTO'):
            problemas += 1

    if 'sim_time_s' in report:
        print('\ntempo de simulação: %.2f s' % report['sim_time_s'])
    if 'pose' in report:
        p = report['pose']
        print('pose: x=%.3f y=%.3f z=%.3f yaw=%.1f deg  (frame "%s")'
              % (p['x'], p['y'], p['z'], p['yaw_deg'], report['frame_odom']))

    if 'scan' in report:
        s = report['scan']
        faixa = ('min %.2f max %.2f m' % (s['min_m'], s['max_m'])
                 if s['min_m'] is not None else 'nenhum retorno válido')
        print('\nlidar: %d feixes, %d válidos (%.0f%%), %s'
              % (s['feixes'], s['validos'], s['validos_pct'], faixa))
        if s['validos'] == 0:
            print('  -> nada na altura do plano de varredura. Num mundo com '
                  'paredes isso é defeito; no mundo vazio é o esperado.')

    if 'camera' in report:
        c = report['camera']
        print('\ncâmera: %dx%d %s, %d bytes, intensidade média %s'
              % (c['largura'], c['altura'], c['encoding'], c['bytes'],
                 c.get('intensidade_media', '?')))
        if c.get('geometria_confere') is False:
            print('  -> PROBLEMA: camera_info diz %s, imagem diz %dx%d'
                  % (c['camera_info'], c['largura'], c['altura']))
            problemas += 1

    print('\nTF: %d arestas (%d estaticas), raiz(es): %s'
          % (len(report['tf_arestas']), len(report.get('tf_estaticas', [])),
             ', '.join(report.get('tf_raizes', [])) or '(nenhuma)'))
    for edge in report['tf_arestas'][:12]:
        print('  ', edge)
    if len(report['tf_arestas']) > 12:
        print('   ... e %d outras' % (len(report['tf_arestas']) - 12))
    if not report['tf_fecha_odom']:
        print('  -> a árvore NÃO tem frame "odom". Conhecido e esperado hoje: '
              'a estimativa de estado com perna é F5. Nav2 com localização '
              'absoluta não sobe sem isso.')

    print('\n%s: %d problema(s)' % ('FALHOU' if problemas else 'PASSOU', problemas))
    return problemas


def main() -> int:
    """Roda a janela de medição e imprime o relatório."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20.0,
                        help='duração da janela de medição, em segundos de parede')
    parser.add_argument('--json', help='também grava o relatório neste arquivo')
    args = parser.parse_args()

    rclpy.init()
    node = ScenarioCheck(args.seconds)
    while rclpy.ok() and not node.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    report = build_report(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    if args.json:
        with open(args.json, 'w') as handle:
            json.dump(report, handle, indent=2, ensure_ascii=False)
    return 1 if print_report(report) else 0


if __name__ == '__main__':
    sys.exit(main())
