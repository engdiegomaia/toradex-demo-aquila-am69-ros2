"""
Operational telemetry of the target for the cockpit.

The cockpit panel should not depend on raw `/rosout` to answer the
operational question: "which axes is the Aquila commanding right now, and how
much resource is it spending?". `/rosout` mixes host, Gazebo, Nav2, bridge,
repeated warnings, and DDS discovery noise. This node publishes two dedicated
channels, both `std_msgs/String` with JSON, so rosbridge can consume them
without a custom message:

    /demo/target/ops_log   short events about vx/vy/wz and the stick
    /demo/target/status    CPU, memory, temperature, and load of the target

It runs alongside navigation, so in HIL mode these numbers are the AM69's. In
learn mode they are the workstation's, which is still useful for development.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import socket
import time
from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _safe_float(value: object) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _read_proc_stat() -> Optional[tuple[int, int]]:
    try:
        fields = Path('/proc/stat').read_text(encoding='utf-8').splitlines()[0].split()
    except OSError:
        return None
    if not fields or fields[0] != 'cpu':
        return None
    values = [int(value) for value in fields[1:]]
    if len(values) < 5:
        return None
    idle = values[3] + values[4]
    total = sum(values)
    return idle, total


def _read_meminfo() -> dict[str, Optional[float]]:
    try:
        lines = Path('/proc/meminfo').read_text(encoding='utf-8').splitlines()
    except OSError:
        return {'mem_total_mb': None, 'mem_used_mb': None, 'mem_percent': None}

    values: dict[str, float] = {}
    for line in lines:
        if ':' not in line:
            continue
        key, raw = line.split(':', 1)
        amount = _safe_float(raw.strip().split()[0])
        if amount is not None:
            values[key] = amount

    total_kb = values.get('MemTotal')
    available_kb = values.get('MemAvailable')
    if not total_kb or available_kb is None:
        return {'mem_total_mb': None, 'mem_used_mb': None, 'mem_percent': None}
    used_kb = max(0.0, total_kb - available_kb)
    return {
        'mem_total_mb': total_kb / 1024.0,
        'mem_used_mb': used_kb / 1024.0,
        'mem_percent': 100.0 * used_kb / total_kb,
    }


def _read_temperature_c() -> Optional[float]:
    values: list[float] = []
    for path in Path('/sys/class/thermal').glob('thermal_zone*/temp'):
        try:
            raw = path.read_text(encoding='utf-8').strip()
        except OSError:
            continue
        value = _safe_float(raw)
        if value is None:
            continue
        # Kernel thermal zones commonly expose millidegrees Celsius.
        if value > 1000.0:
            value /= 1000.0
        if -40.0 <= value <= 140.0:
            values.append(value)
    return max(values) if values else None


def _load_average() -> tuple[Optional[float], Optional[float], Optional[float]]:
    try:
        return os.getloadavg()
    except OSError:
        return None, None, None


def _direction(vx: float, vy: float, wz: float) -> str:
    parts: list[str] = []
    if abs(vx) >= 0.01:
        parts.append('forward' if vx > 0 else 'reverse')
    if abs(vy) >= 0.01:
        parts.append('strafe left' if vy > 0 else 'strafe right')
    if abs(wz) >= 0.02:
        parts.append('turn ccw' if wz > 0 else 'turn cw')
    return 'stopped' if not parts else ' + '.join(parts)


class TargetMonitor(Node):
    """Publish direct actuation logs and resource telemetry for the target."""

    def __init__(self) -> None:
        super().__init__('target_monitor')
        self._ops_pub = self.create_publisher(String, '/demo/target/ops_log', 10)
        self._status_pub = self.create_publisher(String, '/demo/target/status', 10)

        self.create_subscription(Twist, '/demo/cmd_vel_si', self._on_si, 10)
        self.create_subscription(Twist, '/demo/cmd_vel', self._on_stick, 10)
        self.create_subscription(Odometry, '/demo/odom', self._on_odom, 10)

        self._hostname = socket.gethostname()
        self._last_cpu = _read_proc_stat()
        self._last_si: Optional[Twist] = None
        self._last_stick: Optional[Twist] = None
        self._last_odom: Optional[Odometry] = None
        self._last_si_at = 0.0
        self._last_stick_at = 0.0
        self._last_odom_at = 0.0
        self._last_summary = ''
        self._last_summary_at = 0.0

        self.create_timer(0.5, self._publish_ops_if_changed)
        self.create_timer(2.0, self._publish_status)
        self.get_logger().info(
            'publishing target actuation on /demo/target/ops_log and '
            'resources on /demo/target/status')

    def _on_si(self, message: Twist) -> None:
        self._last_si = message
        self._last_si_at = time.monotonic()

    def _on_stick(self, message: Twist) -> None:
        self._last_stick = message
        self._last_stick_at = time.monotonic()

    def _on_odom(self, message: Odometry) -> None:
        self._last_odom = message
        self._last_odom_at = time.monotonic()

    def _cpu_percent(self) -> Optional[float]:
        current = _read_proc_stat()
        previous = self._last_cpu
        self._last_cpu = current
        if current is None or previous is None:
            return None
        idle_delta = current[0] - previous[0]
        total_delta = current[1] - previous[1]
        if total_delta <= 0:
            return None
        return max(0.0, min(100.0, 100.0 * (1.0 - idle_delta / total_delta)))

    def _publish_status(self) -> None:
        load1, load5, load15 = _load_average()
        payload = {
            'host': self._hostname,
            'stamp': time.time(),
            'cpu_percent': self._cpu_percent(),
            'temp_c': _read_temperature_c(),
            'load1': load1,
            'load5': load5,
            'load15': load15,
            'uptime_s': time.monotonic(),
            **_read_meminfo(),
        }
        self._status_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))

    def _publish_ops_if_changed(self) -> None:
        now = time.monotonic()
        si = self._last_si
        if si is None or now - self._last_si_at > 2.5:
            summary = 'no recent SI command from Nav2'
            details = {'fresh': False}
        else:
            vx = si.linear.x
            vy = si.linear.y
            wz = si.angular.z
            direction = _direction(vx, vy, wz)
            summary = (
                f'target axes: {direction}; '
                f'vx={vx:+.3f} m/s vy={vy:+.3f} m/s wz={wz:+.3f} rad/s')
            details = {
                'fresh': True,
                'vx_mps': vx,
                'vy_mps': vy,
                'wz_radps': wz,
                'direction': direction,
            }

        stick = self._last_stick
        if stick is not None and now - self._last_stick_at <= 2.5:
            summary += (
                f'; stick lx={stick.linear.x:+.2f} '
                f'ly={stick.linear.y:+.2f} rz={stick.angular.z:+.2f}')
            details.update({
                'stick_lx': stick.linear.x,
                'stick_ly': stick.linear.y,
                'stick_rz': stick.angular.z,
            })

        odom = self._last_odom
        if odom is not None and now - self._last_odom_at <= 2.5:
            pos = odom.pose.pose.position
            summary += f'; odom x={pos.x:+.2f} y={pos.y:+.2f}'
            details.update({'odom_x': pos.x, 'odom_y': pos.y})

        should_publish = (
            summary != self._last_summary
            or now - self._last_summary_at >= (1.0 if details.get('fresh') else 5.0)
        )
        if not should_publish:
            return

        self._last_summary = summary
        self._last_summary_at = now
        payload = {
            'stamp': time.time(),
            'level': 'info',
            'node': 'target_monitor',
            'msg': summary,
            **details,
        }
        self._ops_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetMonitor()
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
