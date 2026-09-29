#!/usr/bin/env python3
"""
Checks the topic contract of a simulation scenario and prints a report.

Runs on the x86 host, against the containerised simulation (`--network=host`,
`ROS_DOMAIN_ID=69`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`). It publishes
nothing: it is read-only, so it can run alongside any motion script.

    python3 tools/diagnostics/scenario_check.py --seconds 20
    python3 tools/diagnostics/scenario_check.py --seconds 20 --json report.json

What it measures, and why each item is here:

- **Rate of each topic.** A topic that is present but at 0 Hz is the most
  common and quietest failure mode of this simulation: the bridge comes up,
  `ros2 topic list` shows the name, and nothing ever arrives. Counting
  messages over a window is the only check that catches it.
- **`/clock`.** If simulation time does not advance, every node with
  `use_sim_time` freezes without an error. It is the first thing to look at
  when "nothing happens".
- **Lidar content**, not just the rate: a scan that is 100% infinities is what
  you see when the world has nothing at the height of the scan plane, and that
  is indistinguishable from a broken sensor if you only look at Hz.
- **Image geometry** against `camera_info`. A mismatch here breaks any
  downstream inference and is invisible in the rate.
- **The TF `odom` -> `base_link` edge.** Known to be missing today (F5
  blocker). It is here so the report says so explicitly instead of someone
  rediscovering it while debugging Nav2.

No result from this script applies to the real Aquila AM69: it measures the
simulation on the host.
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

# Minimum acceptable rate per topic. These are not design targets: they are the
# floor below which the downstream consumer visibly breaks.
#
#   /clock      -- below ~50 Hz simulation time advances in jumps and the
#                  timers of any node with use_sim_time jitter.
#   /demo/odom  -- 10 Hz is what Nav2 assumes for odometry.
#   /demo/scan  -- 5 Hz is the costmap floor; below it the obstacle appears
#                  after the robot is already inside it.
#   /demo/imu   -- the controller runs at 500 Hz and consumes the IMU
#                  internally; what the bridge exposes is diagnostic, so
#                  50 Hz is enough.
FLOORS = {
    '/clock': 50.0,
    '/demo/odom': 10.0,
    '/demo/scan': 5.0,
    '/demo/camera/image_raw': 5.0,
    '/demo/camera/camera_info': 1.0,
    '/demo/imu': 50.0,
}

# These may legitimately be absent: they only exist if the corresponding node
# is running. Absence is information, not a failure.
OPTIONAL = {'/demo/perception/detections', '/demo/cmd_vel', '/tf'}


class ScenarioCheck(Node):
    """Counts messages and keeps the latest one of each contract topic."""

    def __init__(self, seconds: float) -> None:
        """Subscribes to the whole contract and arms the measurement window."""
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
        # /tf_static uses TRANSIENT_LOCAL durability: the fixed edges are
        # published ONCE, when robot_state_publisher comes up, and retained for
        # whoever subscribes later. With the default QoS (VOLATILE) a late
        # subscriber receives nothing and concludes the tree has no fixed
        # joints -- exactly the mistake this script made in its first version.
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
        """Return whether the measurement window has closed."""
        return time.monotonic() - self._wall0 >= self._seconds

    def elapsed(self) -> float:
        """Return the wall-clock time elapsed in the window."""
        return time.monotonic() - self._wall0


def scan_report(scan: LaserScan) -> dict:
    """Summarise a scan: how many returns are valid and the measured range."""
    valid = [r for r in scan.ranges
             if not math.isinf(r) and not math.isnan(r)
             and scan.range_min <= r <= scan.range_max]
    return {
        'beams': len(scan.ranges),
        'valid': len(valid),
        'valid_pct': 100.0 * len(valid) / len(scan.ranges) if scan.ranges else 0.0,
        'min_m': min(valid) if valid else None,
        'max_m': max(valid) if valid else None,
        'sensor_range_m': [scan.range_min, scan.range_max],
    }


def image_report(image: Image, info) -> dict:
    """Summarise the image and check its geometry against camera_info."""
    out = {
        'width': image.width,
        'height': image.height,
        'encoding': image.encoding,
        'bytes': len(image.data),
    }
    if image.data:
        sample = image.data[::max(1, len(image.data) // 4096)]
        out['mean_intensity'] = round(sum(sample) / len(sample), 1)
    if info is not None:
        out['camera_info'] = [info.width, info.height]
        out['geometry_matches'] = (info.width == image.width
                                   and info.height == image.height)
    return out


def build_report(node: ScenarioCheck) -> dict:
    """Build the full report from what was counted."""
    window = node.elapsed()
    topics = {}
    for topic, count in sorted(node.counts.items()):
        hz = count / window if window > 0 else 0.0
        floor = FLOORS.get(topic)
        if count == 0:
            state = 'ABSENT' if topic in OPTIONAL else 'NO DATA'
        elif floor is None:
            state = 'ok'
        else:
            state = 'ok' if hz >= floor else 'SLOW'
        topics[topic] = {'msgs': count, 'hz': round(hz, 1),
                         'floor_hz': floor, 'state': state}

    report = {'window_s': round(window, 1), 'topics': topics}

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
        report['frame_odom'] = odom.header.frame_id or '(empty)'

    scan = node.last.get('/demo/scan')
    if scan is not None:
        report['scan'] = scan_report(scan)

    image = node.last.get('/demo/camera/image_raw')
    if image is not None:
        report['camera'] = image_report(
            image, node.last.get('/demo/camera/camera_info'))

    report['tf_edges'] = sorted('%s -> %s' % e for e in node.tf_edges)
    report['tf_static_edges'] = sorted(
        '%s -> %s' % e for e in node.tf_static_edges)
    report['tf_closes_odom'] = any(
        parent == 'odom' for parent, _ in node.tf_edges)
    children = {c for _, c in node.tf_edges}
    parents = {p for p, _ in node.tf_edges}
    report['tf_roots'] = sorted(parents - children)
    return report


def print_report(report: dict) -> int:
    """Print the report and return the number of problems found."""
    problems = 0
    print('\n=== topic contract (%.1f s window) ===' % report['window_s'])
    print('%-32s %8s %8s %10s' % ('topic', 'msgs', 'Hz', 'state'))
    for topic, info in report['topics'].items():
        floor = '' if info['floor_hz'] is None else ' (floor %g)' % info['floor_hz']
        print('%-32s %8d %8.1f %10s%s'
              % (topic, info['msgs'], info['hz'], info['state'], floor))
        if info['state'] in ('NO DATA', 'SLOW'):
            problems += 1

    if 'sim_time_s' in report:
        print('\nsimulation time: %.2f s' % report['sim_time_s'])
    if 'pose' in report:
        p = report['pose']
        print('pose: x=%.3f y=%.3f z=%.3f yaw=%.1f deg  (frame "%s")'
              % (p['x'], p['y'], p['z'], p['yaw_deg'], report['frame_odom']))

    if 'scan' in report:
        s = report['scan']
        span = ('min %.2f max %.2f m' % (s['min_m'], s['max_m'])
                 if s['min_m'] is not None else 'no valid return')
        print('\nlidar: %d beams, %d valid (%.0f%%), %s'
              % (s['beams'], s['valid'], s['valid_pct'], faixa))
        if s['valid'] == 0:
            print('  -> nothing at the height of the scan plane. In a world with '
                  'walls this is a defect; in the empty world it is expected.')

    if 'camera' in report:
        c = report['camera']
        print('\ncamera: %dx%d %s, %d bytes, mean intensity %s'
              % (c['width'], c['height'], c['encoding'], c['bytes'],
                 c.get('mean_intensity', '?')))
        if c.get('geometry_matches') is False:
            print('  -> PROBLEM: camera_info says %s, image says %dx%d'
                  % (c['camera_info'], c['width'], c['height']))
            problems += 1

    print('\nTF: %d edges (%d static), root(s): %s'
          % (len(report['tf_edges']), len(report.get('tf_static_edges', [])),
             ', '.join(report.get('tf_roots', [])) or '(none)'))
    for edge in report['tf_edges'][:12]:
        print('  ', edge)
    if len(report['tf_edges']) > 12:
        print('   ... and %d more' % (len(report['tf_edges']) - 12))
    if not report['tf_closes_odom']:
        print('  -> the tree has NO "odom" frame. Known and expected today: '
              'legged state estimation is F5. Nav2 with absolute '
              'localisation does not come up without it.')

    print('\n%s: %d problem(s)' % ('FAILED' if problems else 'PASSED', problems))
    return problems


def main() -> int:
    """Run the measurement window and print the report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20.0,
                        help='duration of the measurement window, in wall-clock seconds')
    parser.add_argument('--json', help='also write the report to this file')
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
