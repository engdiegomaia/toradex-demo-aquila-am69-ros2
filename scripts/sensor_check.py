#!/usr/bin/env python3
"""
Confere a integracao de lidar e odometria com o Nav2. Somente leitura.

Responde tres perguntas que decidem se o robo esta travando por SENSOR ou por
DECISAO do controlador:

1. O lidar acerta o proprio robo? Ponto de auto-colisao entra no costmap como
   obstaculo colado no robo, e o MPPI conclui que esta emparedado. Isso apareceria
   como recuo constante -- exatamente o sintoma reclamado.
2. A odometria e coerente com a TF que o Nav2 consome? Aqui /demo/odom e ground
   truth do Gazebo, entao o que se testa e a costura odom -> base, nao deriva.
3. As taxas sustentam o laco de 10 Hz que o MPPI pede? Nuvem lenta faz o costmap
   local envelhecer e o controlador planejar contra parede que ja saiu.
"""
import math
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros

# Raio circunscrito do tronco do Go2. Ponto de lidar mais perto do que isto, e
# estavel em rumo, e candidato a auto-colisao.
TRUNK_RADIUS_M = 0.383
SECONDS = 20.0


class Check(Node):
    """Coleta nuvem, scan, odom e TF por alguns segundos."""

    def __init__(self):
        super().__init__('sensor_check')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.cloud_n = 0
        self.scan_n = 0
        self.odom_n = 0
        self.cmd_n = 0
        self.near_bearings: list = []
        self.near_count = 0
        self.total_points = 0
        self.z_lo = float('inf')
        self.z_hi = float('-inf')
        self.r_lo = float('inf')
        self.scan_lo = float('inf')
        self.scan_inf = 0
        self.scan_total = 0
        self.odom = None
        self.cmd_neg = 0
        self.cmd_pos = 0
        self.cmd_zero = 0

        sensor = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(PointCloud2, '/demo/scan_cloud',
                                 self._on_cloud, sensor)
        self.create_subscription(LaserScan, '/demo/scan', self._on_scan, sensor)
        self.create_subscription(Odometry, '/demo/odom', self._on_odom, sensor)
        self.create_subscription(Twist, '/demo/cmd_vel_si', self._on_cmd, 10)
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

    def _on_cloud(self, msg):
        self.cloud_n += 1
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        if pts.size == 0:
            return
        self.total_points += len(pts)
        self.z_lo = min(self.z_lo, float(pts[:, 2].min()))
        self.z_hi = max(self.z_hi, float(pts[:, 2].max()))
        rng = np.hypot(pts[:, 0], pts[:, 1])
        finite = rng[np.isfinite(rng)]
        if finite.size:
            self.r_lo = min(self.r_lo, float(finite.min()))
        near = rng < TRUNK_RADIUS_M
        if near.any():
            self.near_count += int(near.sum())
            # Rumo dos pontos proximos: auto-colisao tem rumo FIXO, parede nao.
            self.near_bearings.extend(
                np.degrees(np.arctan2(pts[near, 1], pts[near, 0])).tolist())

    def _on_scan(self, msg):
        self.scan_n += 1
        self.scan_total += len(msg.ranges)
        vals = [r for r in msg.ranges if math.isfinite(r) and r > 0.0]
        self.scan_inf += len(msg.ranges) - len(vals)
        if vals:
            self.scan_lo = min(self.scan_lo, min(vals))

    def _on_odom(self, msg):
        self.odom_n += 1
        self.odom = msg

    def _on_cmd(self, msg):
        self.cmd_n += 1
        if msg.linear.x < -0.002:
            self.cmd_neg += 1
        elif msg.linear.x > 0.002:
            self.cmd_pos += 1
        else:
            self.cmd_zero += 1


def main():
    rclpy.init()
    node = Check()
    start = time.time()
    while time.time() - start < SECONDS:
        rclpy.spin_once(node, timeout_sec=0.05)
    dt = time.time() - start

    print(f'janela de {dt:.1f} s de tempo real\n')
    print('TAXAS')
    print(f'  /demo/scan_cloud   {node.cloud_n / dt:5.1f} Hz'
          f'  ({node.cloud_n} msgs)')
    print(f'  /demo/scan         {node.scan_n / dt:5.1f} Hz'
          f'  ({node.scan_n} msgs)')
    print(f'  /demo/odom         {node.odom_n / dt:5.1f} Hz')
    print(f'  /demo/cmd_vel_si   {node.cmd_n / dt:5.1f} Hz')

    print('\nLIDAR')
    if node.cloud_n:
        print(f'  pontos por nuvem   {node.total_points // node.cloud_n}')
        print(f'  z na nuvem         {node.z_lo:+.3f} .. {node.z_hi:+.3f} m')
        print(f'  alcance minimo     {node.r_lo:.3f} m')
    if node.scan_n:
        print(f'  scan minimo        {node.scan_lo:.3f} m')
        print(f'  scan sem retorno   '
              f'{100.0 * node.scan_inf / max(1, node.scan_total):.0f}%')

    print('\nAUTO-COLISAO')
    print(f'  pontos com r < {TRUNK_RADIUS_M} m: {node.near_count}')
    if node.near_bearings:
        arr = np.array(node.near_bearings)
        print(f'  rumo desses pontos : media {arr.mean():+.1f} deg,'
              f' desvio {arr.std():.1f} deg')
        print('  desvio pequeno (< 10 deg) com muitos pontos = auto-colisao;')
        print('  desvio grande = parede vista de perto, que e legitimo.')
    else:
        print('  nenhum. O lidar NAO ve o proprio robo.')

    print('\nODOMETRIA E TF')
    if node.odom is not None:
        p = node.odom.pose.pose.position
        print(f'  /demo/odom frame   {node.odom.header.frame_id}'
              f' -> {node.odom.child_frame_id}')
        print(f'  posicao            ({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f})')
        try:
            tf = node.buf.lookup_transform('odom', 'base', rclpy.time.Time())
            t = tf.transform.translation
            err = math.hypot(t.x - p.x, t.y - p.y)
            print(f'  TF odom->base      ({t.x:+.3f}, {t.y:+.3f}, {t.z:+.3f})')
            print(f'  erro odom vs TF    {err:.4f} m'
                  f'   {"OK" if err < 0.05 else "DIVERGENTE"}')
        except Exception as exc:                            # noqa: BLE001
            print(f'  TF odom->base      AUSENTE: {exc}')
        for parent, child in (('map', 'odom'), ('odom', 'base')):
            ok = node.buf.can_transform(parent, child, rclpy.time.Time())
            print(f'  {parent} -> {child:<5}      {"presente" if ok else "AUSENTE"}')

    print('\nCOMANDO (o sintoma reclamado)')
    if node.cmd_n:
        tot = node.cmd_n
        print(f'  vx > 0 (frente)    {100.0 * node.cmd_pos / tot:3.0f}%')
        print(f'  vx < 0 (re)        {100.0 * node.cmd_neg / tot:3.0f}%')
        print(f'  vx = 0 (parado)    {100.0 * node.cmd_zero / tot:3.0f}%')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
