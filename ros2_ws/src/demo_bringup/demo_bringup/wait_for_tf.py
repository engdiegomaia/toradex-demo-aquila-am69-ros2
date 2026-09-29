"""
Block until a TF edge really exists, then exit 0.

Runs on the x86 host in learn, and on the Aquila AM69 (arm64) in hil. CPU
only.

    ros2 run demo_bringup wait_for_tf --ros-args \
        -p parent_frame:=odom -p child_frame:=base -p timeout_s:=120.0

WHY THIS NODE EXISTS (ML3.5 F5, 26/08/2026)

It is a sibling of `wait_for_clock`, and was born from the same failure, one
level further out.

MEASURED ON THE AQUILA AM69: after a `module.sh up`, `local_costmap` spent 61 s
printing

    Could not find a connection between 'odom' and 'base' because they are not
    part of the same tree. Tf has two or more unconnected trees.

and then:

    Failed to activate local_costmap because transform from base to odom did not
    become available before timeout
    Failed to change state for node: controller_server
    Failed to bring up all requested nodes. Aborting bringup.

**The lifecycle manager aborts PERMANENTLY and does not retry.** The
container stays up, every topic appears, `scripts/module.sh verify` returns
0 -- and every goal is rejected with "Action server is inactive", because
`bt_navigator` never left INACTIVE. Nothing along the way says "TF".

The missing edge is `odom -> base`, which `odom_tf` only publishes once the
FIRST `/demo/odom` message arrives -- and that message comes from the
simulator, on the OTHER machine. In other words: Nav2 was betting on the
speed of DDS discovery between containers. And the bet sometimes loses. After
the measured episode, the same edge was alive at 50 Hz; what was missing was
ordering, not capacity.

`wait_for_clock` documents exactly this reasoning for the clock, and
`quadruped.launch.py` already chains its startup through `OnProcessExit`,
"each link conditioned on the previous one finishing, not on elapsed time."
This node brings the same discipline to TF, which was the missing link.

WHY `Time()` INSTEAD OF THE NODE CLOCK

The query uses `rclpy.time.Time()`, which in tf2 means "the most recent
common instant" and does not depend on this node's clock. That is why it
deliberately runs with `use_sim_time: False`: with no clock to query, it does
not subscribe to `/clock` -- and subscribing to `/clock` at ~870 Hz without
using a single message was measured on 26/08 as 35-40% of a core per node.
See `docs/results/ml35-f5-clock-fanout.md` and `test_sim_time_scope.py`.
"""

import sys

import rclpy
from rclpy.node import Node
import tf2_ros


# Polling step. 0.2 s is cheap and keeps the unblocking latency well below
# the DDS discovery time itself.
POLL_PERIOD_S = 0.2


class WaitForTf(Node):
    """Poll `can_transform` until the edge exists or the deadline expires."""

    def __init__(self) -> None:
        super().__init__('wait_for_tf')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base')
        self.declare_parameter('timeout_s', 120.0)

        self.parent = self.get_parameter('parent_frame').value
        self.child = self.get_parameter('child_frame').value
        self.timeout_s = float(self.get_parameter('timeout_s').value)

        self.buffer = tf2_ros.Buffer()
        # The spin_once in the loop below feeds the listener and keeps this
        # node under a single executor.
        self.listener = tf2_ros.TransformListener(self.buffer, self)

    def available(self) -> bool:
        return self.buffer.can_transform(
            self.parent, self.child, rclpy.time.Time())


def main(args=None) -> int:
    rclpy.init(args=args)
    node = WaitForTf()
    # Deadline in WALL time, not simulated: this node exists precisely for
    # the case where sim time has not yet crossed the machine boundary.
    # Measuring the deadline against a clock that might be stalled would
    # wait forever -- the same argument as `_wait` in nav_control_relay.py.
    import time
    deadline = time.monotonic() + node.timeout_s
    node.get_logger().info(
        'waiting for TF %s -> %s (deadline %.0f s of wall time)'
        % (node.parent, node.child, node.timeout_s))

    try:
        while rclpy.ok():
            if node.available():
                node.get_logger().info(
                    'TF %s -> %s available. Releasing Nav2 startup.'
                    % (node.parent, node.child))
                return 0
            if time.monotonic() > deadline:
                node.get_logger().error(
                    'TF %s -> %s did NOT appear within %.0f s. Bringing up '
                    'Nav2 now makes local_costmap fail activation and the '
                    'lifecycle manager ABORT bringup permanently -- every '
                    'goal would be rejected with "Action server is '
                    'inactive". Check whether /demo/odom crosses the '
                    'machine boundary and whether odom_tf is up.'
                    % (node.parent, node.child, node.timeout_s))
                return 1
            rclpy.spin_once(node, timeout_sec=POLL_PERIOD_S)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 1


if __name__ == '__main__':
    sys.exit(main())
