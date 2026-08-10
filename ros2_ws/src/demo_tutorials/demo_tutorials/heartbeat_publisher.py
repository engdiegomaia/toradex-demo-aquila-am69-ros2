"""
Heartbeat publisher — L1 learning node.

Publishes an incrementing counter on /demo/system/heartbeat at a rate controlled
by the `rate_hz` ROS parameter. The rate can be changed live via `ros2 param set`.
"""

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String


class HeartbeatPublisher(Node):
    """Publish a counter at a configurable rate on /demo/system/heartbeat."""

    TOPIC_NAME = 'system/heartbeat'
    DEFAULT_RATE_HZ = 1.0

    def __init__(self) -> None:
        super().__init__('heartbeat_publisher')

        self.declare_parameter('rate_hz', self.DEFAULT_RATE_HZ)
        rate_hz = self._read_rate_hz()

        self._publisher = self.create_publisher(String, self.TOPIC_NAME, 10)
        self._counter = 0
        self._timer = self.create_timer(1.0 / rate_hz, self._on_tick)

        self.add_on_set_parameters_callback(self._on_parameter_change)

        self.get_logger().info(
            f'heartbeat_publisher up: topic={self.TOPIC_NAME} rate={rate_hz:.2f} Hz')

    def _read_rate_hz(self) -> float:
        rate = float(self.get_parameter('rate_hz').value)
        if rate <= 0.0:
            raise ValueError(f'rate_hz must be > 0, got {rate}')
        return rate

    def _on_tick(self) -> None:
        msg = String()
        msg.data = f'count={self._counter}'
        self._publisher.publish(msg)
        self._counter += 1

    def _on_parameter_change(self, params: list[Parameter]) -> SetParametersResult:
        for param in params:
            if param.name == 'rate_hz':
                new_rate = float(param.value)
                if new_rate <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f'rate_hz must be > 0, got {new_rate}')
                self._timer.cancel()
                self._timer = self.create_timer(1.0 / new_rate, self._on_tick)
                self.get_logger().info(f'rate_hz changed to {new_rate:.2f} Hz')
        return SetParametersResult(successful=True)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HeartbeatPublisher()
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
