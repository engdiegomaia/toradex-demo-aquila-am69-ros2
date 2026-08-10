"""
Heartbeat subscriber — L1 learning node.

Subscribes to /demo/system/heartbeat and logs receipt. Parses the counter
embedded in the payload and flags any gap. Detecting gaps here is the whole
point: `ros2 topic list` proves discovery, but only counter continuity proves
actual message delivery (see CLAUDE.md rule 2 for why that distinction matters).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HeartbeatSubscriber(Node):
    """Log heartbeat receipt and detect counter gaps."""

    TOPIC_NAME = 'system/heartbeat'

    def __init__(self) -> None:
        super().__init__('heartbeat_subscriber')
        self._last_counter: int | None = None
        self._received = 0
        self._gaps = 0
        self._subscription = self.create_subscription(
            String, self.TOPIC_NAME, self._on_message, 10)
        self.get_logger().info(
            f'heartbeat_subscriber up: topic={self.TOPIC_NAME}')

    def _on_message(self, msg: String) -> None:
        counter = self._parse_counter(msg.data)
        self._received += 1

        if counter is None:
            self.get_logger().warn(f'unparseable payload: {msg.data!r}')
            return

        if self._last_counter is not None:
            expected = self._last_counter + 1
            if counter != expected:
                self._gaps += 1
                self.get_logger().warn(
                    f'gap: expected count={expected}, got count={counter} '
                    f'(total gaps={self._gaps})')
        self._last_counter = counter

        if self._received % 10 == 0:
            self.get_logger().info(
                f'received={self._received} last_count={counter} gaps={self._gaps}')

    @staticmethod
    def _parse_counter(payload: str) -> int | None:
        if not payload.startswith('count='):
            return None
        try:
            return int(payload.removeprefix('count='))
        except ValueError:
            return None


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HeartbeatSubscriber()
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
