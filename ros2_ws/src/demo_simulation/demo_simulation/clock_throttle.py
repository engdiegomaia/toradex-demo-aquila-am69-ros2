#!/usr/bin/env python3
"""
Republishes `/clock` at a fixed rate, from the Gazebo raw clock.

Runs on the x86 host, inside the simulator container, next to Gazebo.

READ THIS FIRST: THROTTLING IS OFF BY DEFAULT
==============================================

`rate_hz: 0` (the launch default) is pass-through. This node was created to
throttle `/clock` and reduce load on the Aquila, and the test REFUTED the
idea. Measured on 21/08/2026, hil mode, same world and same goals:

    /clock     nav container CPU    average speed    peak cmd_vx
    ~750 Hz    470% of 800%         0.0251 m/s        0.138 m/s
     100 Hz    324% of 800%         0.0039 m/s        0.003 m/s

CPU really did drop. Navigation died along with it: the robot spent 180 s
spinning in place, `cmd_wz` active in 1721 of 1800 samples and `cmd_vx` at
zero. The exact mechanism of how 10 ms granularity breaks the MPPI has not
been isolated -- what is measured is the causal relationship. CPU savings
that make the robot stop walking is not optimization.

The correct attack on the SAME cost is to compose Nav2 into a single process:
one subscription to `/clock` instead of thirteen, and intra-process
communication instead of DDS. That is in
`demo_bringup/launch/nav_quadruped.launch.py`, in the `nav2_container` block.

THIS NODE IS NOT IN ANY LAUNCH FILE. Running `ros2 run` alone is not enough
to use it: `bridge_quadruped.yaml` publishes `/clock` DIRECTLY, so bringing
up this node without changing the bridge creates two publishers on the same
topic -- a silent failure, the robot moves oddly and nothing in the log
names the clock as the cause. Repeating the A/B takes two changes:

    1. in demo_simulation/config/bridge_quadruped.yaml, change the clock's
       `ros_topic_name` from "/clock" to "/demo/clock_raw";
    2. bring up this node with `rate_hz` set to the value under test (0 =
       pass-through, which reproduces the baseline with the extra hop
       already in place).

It was kept out of the default path because it is a pass-through with no
function after the A/B, not because the hop was measured as expensive: in
pass-through, the difference fell WITHIN run-to-run noise (0.0202 m/s with
the hop, 0.0232 without, same configuration and same world). Anyone looking
here for the explanation of a speed drop will not find it -- the hop is not
the cause.

The diagnosis below still holds -- it is the cost that actually exists.

THE COST THE 1 kHz CLOCK REALLY IMPOSES
========================================

The Go2 world uses `<max_step_size>0.001</max_step_size>` because the gait
needs a 1 ms physics step. Gazebo publishes `/clock` on every step, so the
bridge delivers ~1000 messages per second.

On the x86 host that is absorbable. On the Aquila AM69 it is not, and the
failure does NOT look like a clock failure. Measured on 21/08/2026, hil mode,
Nav2 on the module:

    /clock on the host        989 Hz
    /clock on the module      870 Hz
    nav container CPU         660% of 800% available
    load average               15 to 23, with 8 cores
    odom_tf                   87% of one core
    cmd_vel_si_to_stick       89% of one core

`odom_tf` and `cmd_vel_si_to_stick` are trivial Python republishers. The only
high-rate thing either of them processes is `/clock`, because
`use_sim_time: true` makes EVERY Nav2 node subscribe to that topic: ~13 nodes
x 870 Hz = ~11 thousand deliveries per second on a Cortex-A72. The visible
symptom is the robot navigating slowly, with a normal peak `cmd_vx` (0.138)
and a near-zero MEAN (0.0067) -- the controller is starved, not mistuned.

Nothing in the log names the clock. The nodes just get slow.

WHY THROTTLING HERE DOES NOT THREATEN THE GAIT
================================================

This robot's `controller_manager` runs INSIDE the Gazebo process, via
`gz_quadruped_hardware`, and is stepped by the physics loop -- not by
`/clock`. Verified in `quadruped.launch.py`, which comments on exactly this.
So the gait's 1 kHz loop stays at 1 kHz with the clock published at 100 Hz.

The consumers of `/clock` are the nodes with `use_sim_time`, and for them
100 Hz gives 10 ms of granularity, comfortable for Nav2, TF, and for tests
(which sample at 10 Hz).

THE OUTPUT QoS IS BEST-EFFORT, ON PURPOSE
==========================================

ROS 2's own `rclcpp::ClockQoS` is KeepLast(1) best-effort, and that is the
right choice here: losing one clock message is harmless, because the next
one arrives in 10 ms. RELIABLE clock delivery over Wi-Fi costs
retransmission and an ACK per message, for no gain. This node publishes with
the same policy.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSProfile, QoSReliabilityPolicy)
from rosgraph_msgs.msg import Clock

# Equivalent to ROS 2's ClockQoS: the last value is the only one that matters.
CLOCK_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class ClockThrottle(Node):
    """Holds the latest raw clock and republishes it on a REAL-time timer."""

    def __init__(self) -> None:
        super().__init__('clock_throttle')
        # NEVER true use_sim_time on this node: it PRODUCES /clock. Following
        # the simulated clock itself would leave it waiting on itself, and
        # the symptom would be the simulation coming up with no time
        # advancing at all -- the same lockup that `wait_for_clock` exists to
        # diagnose. The launch file is what passes `False`; declaring it here
        # raises ParameterAlreadyDeclaredException, because rclpy's Node
        # ALREADY declares use_sim_time on its own.
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('input_topic', '/demo/clock_raw')

        rate = float(self.get_parameter('rate_hz').value)
        source = str(self.get_parameter('input_topic').value)

        self._latest = None
        self._in = 0
        self._out = 0
        # rate_hz = 0 is PASS-THROUGH: every message leaves as it arrived,
        # with its original timestamp. It exists for arm A of an A/B test --
        # to prove that a behaviour change came, or did not come, from the
        # throttling. This cannot be imitated with a high rate: a timer at
        # 2000 Hz republishes the LAST message repeatedly and delivers MORE
        # traffic than Gazebo produces, which measures something else.
        self._bypass = rate <= 0.0

        self.create_subscription(Clock, source, self._on_clock, CLOCK_QOS)
        self.pub = self.create_publisher(Clock, '/clock', CLOCK_QOS)
        if not self._bypass:
            # Real-time timer: with use_sim_time false, create_timer uses the
            # system clock, which is what is wanted to pace the output.
            self.create_timer(1.0 / rate, self._tick)
        self.create_timer(10.0, self._report)

        if self._bypass:
            self.get_logger().warning(
                'PASS-THROUGH from %s -> /clock (rate_hz=0). No throttling: '
                'the Aquila receives the ~880 Hz from Gazebo. Test mode, '
                'not operating mode.' % source)
        else:
            self.get_logger().info(
                'republishing %s -> /clock at %.0f Hz. The physics step does '
                'NOT change: the gait is stepped by the Gazebo loop, not by '
                '/clock.' % (source, rate))

    def _on_clock(self, msg: Clock) -> None:
        self._latest = msg
        self._in += 1
        if self._bypass:
            self.pub.publish(msg)
            self._out += 1

    def _tick(self) -> None:
        # No raw clock yet: publish nothing. Publishing zero would make the
        # nodes with use_sim_time jump to t=0 and the whole TF tree would
        # sit in the past.
        if self._latest is None:
            return
        self.pub.publish(self._latest)
        self._out += 1

    def _report(self) -> None:
        # The in/out ratio is what proves the throttling took effect.
        # Without it, "I changed the rate and nothing happened" cannot be
        # distinguished from "the rate never changed".
        self.get_logger().info(
            'clock: in %d msg, out %d msg over the last 10 s '
            '(~%.0f Hz -> ~%.0f Hz)'
            % (self._in, self._out, self._in / 10.0, self._out / 10.0))
        self._in = 0
        self._out = 0


def main(args=None) -> None:
    """Bring up the node and spin until interrupted."""
    rclpy.init(args=args)
    node = ClockThrottle()
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
