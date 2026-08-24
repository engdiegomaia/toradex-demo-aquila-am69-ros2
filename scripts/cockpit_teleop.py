#!/usr/bin/env python3
"""ROS publisher driven through stdin by the standalone host cockpit.

This helper runs inside the viz container, where rclpy and the project DDS
configuration already exist. Input is a tiny line protocol:

    CMD <linear.x> <linear.y> <angular.z>
    STOP
    QUIT

Values are normalized stick commands for /demo/cmd_vel. The helper publishes at
10 Hz only while fresh input is arriving and emits zero when input expires.
"""

from __future__ import annotations

import math
import sys
import threading
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


COMMAND_TIMEOUT = 0.40
COMMAND_LIMIT = 0.50


def parse_command(line: str):
    """Parse and validate one cockpit command line."""
    fields = line.strip().split()
    if fields == ['STOP']:
        return 'stop', (0.0, 0.0, 0.0)
    if fields == ['QUIT']:
        return 'quit', (0.0, 0.0, 0.0)
    if len(fields) != 4 or fields[0] != 'CMD':
        raise ValueError('expected CMD vx vy wz, STOP or QUIT')
    values = tuple(float(value) for value in fields[1:])
    if not all(math.isfinite(value) for value in values):
        raise ValueError('command values must be finite')
    if any(abs(value) > COMMAND_LIMIT for value in values):
        raise ValueError(f'command exceeds stick limit {COMMAND_LIMIT}')
    return 'command', values


class CommandState:
    """Thread-safe fresh-command state shared with the ROS timer."""

    def __init__(self):
        self.lock = threading.Lock()
        self.command = (0.0, 0.0, 0.0)
        self.deadline = 0.0
        self.stop_requested = False

    def update(self, command) -> None:
        with self.lock:
            self.command = command
            self.deadline = time.monotonic() + COMMAND_TIMEOUT

    def stop(self) -> None:
        with self.lock:
            self.command = (0.0, 0.0, 0.0)
            self.deadline = 0.0

    def quit(self) -> None:
        with self.lock:
            self.command = (0.0, 0.0, 0.0)
            self.deadline = 0.0
            self.stop_requested = True

    def sample(self):
        with self.lock:
            fresh = time.monotonic() <= self.deadline
            return self.command if fresh else (0.0, 0.0, 0.0)

    def should_quit(self) -> bool:
        with self.lock:
            return self.stop_requested


def read_commands(state: CommandState) -> None:
    """Read until QUIT or EOF; malformed input stops motion."""
    for line in sys.stdin:
        try:
            action, command = parse_command(line)
        except (TypeError, ValueError) as error:
            print(f'cockpit_teleop: comando recusado: {error}', file=sys.stderr)
            state.stop()
            continue
        if action == 'command':
            state.update(command)
        elif action == 'stop':
            state.stop()
        else:
            state.quit()
            return
    state.quit()


def make_twist(command) -> Twist:
    """Build one normalized-stick Twist message."""
    message = Twist()
    message.linear.x, message.linear.y, message.angular.z = command
    return message


def main() -> None:
    """Publish fresh commands at 10 Hz and always finish with zero."""
    state = CommandState()
    reader = threading.Thread(target=read_commands, args=(state,), daemon=True)
    reader.start()

    rclpy.init()
    node = Node('cockpit_teleop')
    publisher = node.create_publisher(Twist, '/demo/cmd_vel', 10)
    last_command = None

    def publish_sample():
        nonlocal last_command
        command = state.sample()
        if command != (0.0, 0.0, 0.0) or last_command != command:
            publisher.publish(make_twist(command))
            last_command = command

    timer = node.create_timer(0.1, publish_sample)
    try:
        while rclpy.ok() and not state.should_quit():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        timer.cancel()
        zero = make_twist((0.0, 0.0, 0.0))
        for _ in range(3):
            publisher.publish(zero)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
