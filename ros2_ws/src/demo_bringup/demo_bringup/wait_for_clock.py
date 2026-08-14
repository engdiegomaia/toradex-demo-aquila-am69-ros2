"""
Block until simulation time is actually advancing, then exit 0.

Runs on: x86 host in learn mode, Aquila AM69 (arm64) in hil mode. CPU-only, no
graphical or architecture-specific dependency.

    ros2 run demo_bringup wait_for_clock --ros-args -p timeout_s:=120.0

Why this exists (ML3.5 F1)
--------------------------
learn.launch.py orders the demo with wall-clock timers: spawn at t=12s, bridge
at t=15s, perception at t=20s, Nav2 at t=25s. Inside a single process tree that
works, because all four marks are measured from the same start.

Across a container boundary they stop meaning anything. `docker compose up`
starts `sim` and `nav` concurrently, so t=25s in the nav container is counted
from when *nav* started, not from when Gazebo finished loading the warehouse
world. The delay becomes a bet on the other container's progress — and a cold
image pull, a slower host, or a heavier world loses that bet with no diagnostic.

The failure it prevents is specifically silent. Nav2 started before /clock is
publishing brings its lifecycle nodes up against transforms that carry no
timestamps; they stall waiting, and nothing in the logs names the clock as the
cause. That trap is documented in learn.launch.py and in docs/guia-operacao.md.

So the timer is replaced by the precondition it was standing in for: this node
waits for /clock to exist AND to advance, then exits, letting the rest of the
launch file proceed.

Two ticks, not one
------------------
Receiving a single /clock message is not sufficient evidence that time is
moving. A paused Gazebo publishes a constant timestamp forever, and `gz sim`
without `-r` starts paused. One message would clear a check that "the clock is
alive" while every downstream node still blocks on a clock that never advances —
trading a silent failure for a different silent failure. Requiring a strictly
greater second sample distinguishes running from paused.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock


def is_advancing(previous_ns: int | None, current_ns: int) -> bool:
    """
    Return True when `current_ns` proves the clock moved past `previous_ns`.

    Pure decision logic, kept free of ROS plumbing so the paused-simulator case
    is unit-testable without a live graph.

    The first sample can never prove advancement, so `previous_ns=None` is
    always False. Equal stamps mean a paused simulator. Time going backwards
    means the simulation was reset; that is not advancement either, and treating
    it as such would let Nav2 start against a clock that just jumped
    backwards — TF would hold only stale transforms.
    """
    if previous_ns is None:
        return False
    return current_ns > previous_ns


def to_nanoseconds(clock_msg: Clock) -> int:
    """Flatten a Clock message to a single comparable integer."""
    return clock_msg.clock.sec * 1_000_000_000 + clock_msg.clock.nanosec


class ClockWaiter(Node):
    """Exit once /clock is observed advancing, or fail after a timeout."""

    CLOCK_TOPIC = '/clock'

    DEFAULT_TIMEOUT_S = 120.0

    def __init__(self) -> None:
        super().__init__('wait_for_clock')

        # Bounded on purpose. An unbounded wait turns "sim never came up" into a
        # container sitting silent forever, which reads as a hang instead of a
        # failure; compose then reports nothing useful. On expiry this node
        # exits non-zero so the orchestrator surfaces it.
        self.declare_parameter('timeout_s', self.DEFAULT_TIMEOUT_S)

        self._timeout_s = float(self.get_parameter('timeout_s').value)
        self._validate_timeout(self._timeout_s)

        # /clock is published with TRANSIENT_LOCAL durability and best-effort
        # reliability by ros_gz_bridge. A reliable+volatile subscription is the
        # classic QoS mismatch: `ros2 topic list` shows /clock, `ros2 topic hz`
        # shows traffic, and this callback never fires.
        clock_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._subscription = self.create_subscription(
            Clock, self.CLOCK_TOPIC, self._on_clock, clock_qos)

        self._previous_ns: int | None = None
        self._advancing = False

        self.get_logger().info(
            f'waiting for {self.CLOCK_TOPIC} to advance '
            f'(timeout {self._timeout_s:.0f}s)')

    @property
    def advancing(self) -> bool:
        """Report whether two samples have proven the clock moving forward."""
        return self._advancing

    @property
    def timeout_s(self) -> float:
        return self._timeout_s

    def _on_clock(self, msg: Clock) -> None:
        current_ns = to_nanoseconds(msg)

        if is_advancing(self._previous_ns, current_ns):
            self._advancing = True
            self.get_logger().info(
                f'{self.CLOCK_TOPIC} is advancing (t={current_ns / 1e9:.3f}s) '
                f'— releasing the launch')
        elif self._previous_ns is not None and current_ns == self._previous_ns:
            # Logged at most once per repeated stamp; a paused simulator is the
            # single most likely reason to sit here, and `gz sim` without -r
            # starts paused.
            self.get_logger().warn(
                f'{self.CLOCK_TOPIC} is publishing but not advancing '
                f'(t={current_ns / 1e9:.3f}s) — is Gazebo paused? '
                f'gz sim needs -r to start unpaused.',
                once=True,
            )

        self._previous_ns = current_ns

    @staticmethod
    def _validate_timeout(timeout_s: float) -> None:
        if timeout_s <= 0.0:
            raise ValueError(f'timeout_s must be > 0, got {timeout_s}')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ClockWaiter()

    # Deliberately not rclpy.spin(): this node's contract is to terminate. It
    # spins in slices so the timeout is enforced against the node's own steady
    # clock rather than against sim time, which is the thing being waited for.
    deadline = node.get_clock().now().nanoseconds + int(node.timeout_s * 1e9)
    exit_code = 0

    try:
        while rclpy.ok() and not node.advancing:
            if node.get_clock().now().nanoseconds >= deadline:
                node.get_logger().error(
                    f'timed out after {node.timeout_s:.0f}s waiting for '
                    f'{node.CLOCK_TOPIC} to advance. Check that the sim '
                    f'container is up, that ROS_DOMAIN_ID matches on both '
                    f'sides, and that the ros_gz_bridge is publishing /clock.')
                exit_code = 1
                break
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # Non-zero on timeout so `docker compose` and `ros2 launch` both treat a
    # missing simulator as the failure it is instead of continuing into Nav2.
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
