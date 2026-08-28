"""Simulation-only oracle that confirms a real crossing of the maze exit."""

from __future__ import annotations

import math
from pathlib import Path

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def crosses_opening(
    previous: tuple[float, float],
    current: tuple[float, float],
    *,
    boundary_y: float,
    opening_x: float,
    half_width: float,
) -> bool:
    """Return whether a motion segment crosses outward through the opening."""
    x0, y0 = previous
    x1, y1 = current
    if not (y0 >= boundary_y and y1 < boundary_y and y1 != y0):
        return False
    alpha = (boundary_y - y0) / (y1 - y0)
    crossing_x = x0 + alpha * (x1 - x0)
    return abs(crossing_x - opening_x) <= half_width


class MazeEscapeValidator(Node):
    """Latch success only after crossing the opening and clearing its boundary."""

    BOUNDARY_Y = -0.90
    OPENING_X = -4.90
    OPENING_HALF_WIDTH = 0.60
    ROBOT_RADIUS = 0.38

    def __init__(self) -> None:
        super().__init__('maze_escape_validator')
        self.declare_parameter('world', '')
        self._enabled = Path(str(self.get_parameter('world').value)).name \
            == 'quadruped_maze11.sdf'
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            Bool, '/demo/maze/escaped', qos)
        self._previous: tuple[float, float] | None = None
        self._crossed_opening = False
        self._escaped = False
        self.create_subscription(Odometry, '/demo/odom', self._on_odom, 20)
        self._publish()

    def _publish(self) -> None:
        self._publisher.publish(Bool(data=self._escaped))

    def _on_odom(self, message: Odometry) -> None:
        if not self._enabled:
            return
        position = message.pose.pose.position
        current = (float(position.x), float(position.y))

        # A simulation reset teleports the robot back to the official spawn.
        if self._escaped and math.hypot(*current) < 0.5:
            self._crossed_opening = False
            self._escaped = False
            self._publish()

        if self._previous is not None:
            if crosses_opening(
                self._previous, current,
                boundary_y=self.BOUNDARY_Y,
                opening_x=self.OPENING_X,
                half_width=self.OPENING_HALF_WIDTH,
            ):
                self._crossed_opening = True

        fully_outside = current[1] <= self.BOUNDARY_Y - self.ROBOT_RADIUS
        if self._crossed_opening and fully_outside and not self._escaped:
            self._escaped = True
            self._publish()
            self.get_logger().info('maze exit crossing confirmed')
        self._previous = current


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MazeEscapeValidator()
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
