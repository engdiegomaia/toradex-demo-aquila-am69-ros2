#!/usr/bin/env python3
"""
Checks the lidar and odometry integration with Nav2. Read-only.

Answers three questions that decide whether the robot is stalling because of a
SENSOR or because of a controller DECISION:

1. Does the lidar hit the robot itself? A self-collision point enters the
   costmap as an obstacle glued to the robot, and MPPI concludes it is boxed
   in. That would show up as constant backing up -- exactly the reported
   symptom.
2. Is the odometry consistent with the TF that Nav2 consumes? Here /demo/odom
   is Gazebo ground truth, so what is tested is the odom -> base seam, not
   drift.
3. Do the rates sustain the 10 Hz loop MPPI asks for? A slow cloud makes the
   local costmap go stale and the controller plan against a wall that is
   already gone.
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

# Circumscribed radius of the Go2 trunk. A lidar point closer than this that is
# also stable in bearing is a self-collision candidate.
TRUNK_RADIUS_M = 0.383
SECONDS = 20.0


class Check(Node):
    """Collects cloud, scan, odom and TF for a few seconds."""

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
            # Bearing of the near points: self-collision has a FIXED bearing, a wall does not.
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

    print(f'{dt:.1f} s wall-clock window\n')
    print('RATES')
    print(f'  /demo/scan_cloud   {node.cloud_n / dt:5.1f} Hz'
          f'  ({node.cloud_n} msgs)')
    print(f'  /demo/scan         {node.scan_n / dt:5.1f} Hz'
          f'  ({node.scan_n} msgs)')
    print(f'  /demo/odom         {node.odom_n / dt:5.1f} Hz')
    print(f'  /demo/cmd_vel_si   {node.cmd_n / dt:5.1f} Hz')

    print('\nLIDAR')
    if node.cloud_n:
        print(f'  points per cloud    {node.total_points // node.cloud_n}')
        print(f'  z in cloud          {node.z_lo:+.3f} .. {node.z_hi:+.3f} m')
        print(f'  minimum range      {node.r_lo:.3f} m')
    if node.scan_n:
        print(f'  scan minimum       {node.scan_lo:.3f} m')
        print(f'  scan no return     '
              f'{100.0 * node.scan_inf / max(1, node.scan_total):.0f}%')

    print('\nSELF-COLLISION')
        print(f'  points with r < {TRUNK_RADIUS_M} m: {node.near_count}')
    if node.near_bearings:
        arr = np.array(node.near_bearings)
        print(f'  bearing of points  : mean {arr.mean():+.1f} deg,'
              f' std dev {arr.std():.1f} deg')
        print('  small std dev (< 10 deg) with many points = self-collision;')
        print('  large std dev = wall seen up close, which is legitimate.')
    else:
        print('  none. The lidar does NOT see the robot itself.')

    print('\nODOMETRY AND TF')
    if node.odom is not None:
        p = node.odom.pose.pose.position
        print(f'  /demo/odom frame   {node.odom.header.frame_id}'
              f' -> {node.odom.child_frame_id}')
        print(f'  position           ({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f})')
        try:
            tf = node.buf.lookup_transform('odom', 'base', rclpy.time.Time())
            t = tf.transform.translation
            err = math.hypot(t.x - p.x, t.y - p.y)
            print(f'  TF odom->base      ({t.x:+.3f}, {t.y:+.3f}, {t.z:+.3f})')
            print(f'  odom vs TF error  {err:.4f} m'
                  f'   {"OK" if err < 0.05 else "DIVERGENT"}')
        except Exception as exc:                            # noqa: BLE001
            print(f'  TF odom->base      MISSING: {exc}')
        for parent, child in (('map', 'odom'), ('odom', 'base')):
            ok = node.buf.can_transform(parent, child, rclpy.time.Time())
            print(f'  {parent} -> {child:<5}      {"present" if ok else "MISSING"}')

    print('\nCOMMAND (the reported symptom)')
    if node.cmd_n:
        tot = node.cmd_n
        print(f'  vx > 0 (forward)   {100.0 * node.cmd_pos / tot:3.0f}%')
        print(f'  vx < 0 (reverse)   {100.0 * node.cmd_neg / tot:3.0f}%')
        print(f'  vx = 0 (stopped)   {100.0 * node.cmd_zero / tot:3.0f}%')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
