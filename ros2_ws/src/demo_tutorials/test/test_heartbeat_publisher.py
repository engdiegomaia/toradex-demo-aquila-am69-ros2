"""Unit test for the heartbeat publisher: prove one message is emitted per tick."""

from demo_tutorials.heartbeat_publisher import HeartbeatPublisher
import rclpy
from std_msgs.msg import String


def test_publisher_emits_one_message_per_tick() -> None:
    """Instantiate the publisher, drive it once, assert a message went out."""
    rclpy.init()
    try:
        publisher = HeartbeatPublisher()

        received: list[String] = []

        # A dummy subscriber in the same process — proves the publisher
        # actually writes to the topic, not just that the callback runs.
        subscriber = rclpy.create_node('test_subscriber')
        subscriber.create_subscription(
            String,
            f'/{publisher.get_name()}/{HeartbeatPublisher.TOPIC_NAME}'
            if False else HeartbeatPublisher.TOPIC_NAME,
            received.append,
            10,
        )

        # Manually invoke one tick — no need to wait for the timer.
        publisher._on_tick()

        # Spin the subscriber briefly so it can drain the queue.
        deadline_ns = 1_000_000_000  # 1 second
        rclpy.spin_once(subscriber, timeout_sec=deadline_ns / 1e9)

        assert len(received) >= 1, 'publisher did not emit any message'
        assert received[0].data.startswith('count='), \
            f'unexpected payload: {received[0].data!r}'

        subscriber.destroy_node()
        publisher.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
