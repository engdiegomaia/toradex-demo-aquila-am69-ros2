"""
AddTwoInts service server — L1 learning node.

The canonical ROS 2 service example, wired to /demo/add_two_ints under the
project namespace.
"""

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class AddTwoIntsServer(Node):
    """Serve AddTwoInts requests."""

    SERVICE_NAME = 'add_two_ints'

    def __init__(self) -> None:
        super().__init__('add_two_ints_server')
        self._service = self.create_service(
            AddTwoInts, self.SERVICE_NAME, self._on_request)
        self.get_logger().info(
            f'add_two_ints_server up: service={self.SERVICE_NAME}')

    def _on_request(
        self,
        request: AddTwoInts.Request,
        response: AddTwoInts.Response,
    ) -> AddTwoInts.Response:
        response.sum = request.a + request.b
        self.get_logger().info(
            f'{request.a} + {request.b} = {response.sum}')
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AddTwoIntsServer()
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
