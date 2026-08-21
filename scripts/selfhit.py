#!/usr/bin/env python3
"""
Decide se o lidar ve o PROPRIO robo, sem depender de onde o robo esta.

Retorno numa peca do robo tem raio E rumo constantes no frame do lidar, porque a
peca e rigida em relacao ao sensor. Retorno em parede varia de raio conforme o
robo se move. Entao: por setor de rumo, o desvio-padrao do raio minimo ao longo
do tempo separa os dois casos, e nao e preciso mundo vazio para concluir.

POR QUE ISSO MERECE UM SCRIPT

Auto-colisao de lidar entra no costmap como obstaculo colado no robo, e o
controlador conclui que esta emparedado. O sintoma e o robo recuar ou travar sem
nada em log, e a suspeita natural cai no controlador, que esta certo.

LIMITE, E ELE E SERIO

Com o robo PARADO o teste nao conclui: parede tambem da desvio zero. O script
mede o quanto o robo andou na janela e AVISA quando o resultado nao vale. Uma
leitura de rumo aparentemente fixo com robo parado ja levou a um diagnostico
errado de auto-colisao neste projeto (21/08/2026) -- o que havia era uma parede
reta a 0,372 m, confirmada depois porque os raios ajustavam d/cos(theta-theta0).

Roda no host x86, contra a simulacao ja de pe.

    python3 scripts/selfhit.py
"""

import time

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

BINS = 72                 # setores de 5 graus
NEAR_M = 1.0              # so o que esta perto pode ser peca do robo
SECONDS = 25.0
# Abaixo deste desvio o raio e constante para o que a simulacao resolve.
FIXED_RANGE_SD_M = 0.01
# Caminho minimo para o teste ter poder de separacao.
MIN_PATH_M = 0.15
# Presenca minima para o setor entrar no relatorio.
MIN_PRESENCE = 0.8


class SelfHit(Node):
    """Acumula, por setor de rumo, o raio minimo de cada nuvem."""

    def __init__(self) -> None:
        super().__init__('selfhit')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.per_cloud: list = []
        self.poses: list = []
        sensor = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(PointCloud2, '/demo/scan_cloud',
                                 self._on_cloud, sensor)
        self.create_subscription(Odometry, '/demo/odom', self._on_odom, sensor)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.poses.append((p.x, p.y))

    def _on_cloud(self, msg: PointCloud2) -> None:
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        if pts.size == 0:
            return
        # Raio de nao-retorno vem como inf/nan e tem de sair antes de qualquer
        # estatistica, ou min/std viram inf sem avisar.
        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts) == 0:
            return
        radius = np.hypot(pts[:, 0], pts[:, 1])
        bearing = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
        near = radius < NEAR_M
        row = np.full(BINS, np.nan)
        if near.any():
            index = ((bearing[near] + 180.0) / (360.0 / BINS)).astype(int) % BINS
            for sector in range(BINS):
                pick = index == sector
                if pick.any():
                    row[sector] = radius[near][pick].min()
        self.per_cloud.append(row)


def main() -> int:
    rclpy.init()
    node = SelfHit()
    start = time.time()
    while time.time() - start < SECONDS:
        rclpy.spin_once(node, timeout_sec=0.05)

    if len(node.per_cloud) < 20:
        print('poucas nuvens; a simulacao esta publicando /demo/scan_cloud?')
        return 1

    grid = np.vstack(node.per_cloud)
    conclusive = True
    if len(node.poses) > 1:
        poses = np.array(node.poses)
        net = float(np.hypot(*(poses[-1] - poses[0])))
        path = float(np.sum(np.hypot(*np.diff(poses, axis=0).T)))
        print(f'nuvens {len(grid)}   robo andou {path:.2f} m de caminho, '
              f'{net:.2f} m liquidos')
        if path < MIN_PATH_M:
            conclusive = False
            print('AVISO: robo quase parado. Com robo parado, parede tambem da '
                  'desvio zero, e o teste NAO separa os dois casos. Mova o robo '
                  '(mande uma meta) e repita.')

    print(f'\nsetores com retorno em >= {MIN_PRESENCE:.0%} das nuvens '
          f'(r < {NEAR_M} m):')
    print('  rumo             presenca   raio medio   desvio    veredito')
    fixed = 0
    for sector in range(BINS):
        column = grid[:, sector]
        seen = np.isfinite(column)
        presence = seen.mean()
        if presence < MIN_PRESENCE:
            continue
        low = -180.0 + sector * (360.0 / BINS)
        sd = float(column[seen].std())
        is_fixed = sd < FIXED_RANGE_SD_M
        fixed += int(is_fixed)
        verdict = 'PECA DO ROBO' if is_fixed else 'parede/ambiente'
        print(f'  [{low:+6.0f},{low + 5:+6.0f})    {presence * 100:5.0f}%'
              f'   {column[seen].mean():8.3f} m  {sd:7.4f}   {verdict}')

    print(f'\nsetores com raio constante: {fixed}')
    if not conclusive:
        print('INCONCLUSIVO: ver o aviso acima. Nao trate isso como '
              'auto-colisao.')
    elif fixed:
        print('Raio constante com o robo em movimento e auto-colisao. Filtre '
              'no obstacle_layer (obstacle_min_range) ou na origem do sensor.')
    else:
        print('Nenhum setor com raio constante: o lidar NAO ve o proprio robo.')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
