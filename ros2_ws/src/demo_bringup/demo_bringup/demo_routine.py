"""
Continuous motion choreography for exhibiting the quadruped demo.

Runs on: either machine. It speaks only `/demo/cmd_vel`, which is the public
side of the topic contract, so it is machine-agnostic by construction -- the
same node drives Gazebo on the x86 host or a real driver on the module. That is
also why it lives in `demo_bringup` and not in `demo_simulation`: the simulation
package is *part of the plant* (see its `twist_to_inputs` note), while this is a
commander, standing in for the producer Nav2 will eventually be.

WHAT IT IS FOR

An exhibition loop: the robot walks a repeating pattern indefinitely so the demo
can be left running in front of an audience. It is not a test harness -- for
measurement use `tools/evaluation/gait_trial.sh`, which records evidence and aborts on a
fallen robot.

THREE CONSTRAINTS THAT ARE NOT OBVIOUS

1. Publish at or above ~7 Hz, and this node uses 20 Hz. `twist_to_inputs` ages
   every command against `_CMD_TIMEOUT_S = 0.3 s`, so a slower publisher makes
   the robot stutter between walking and stopping.

2. Silence IS the stop command, and it is the whole point of the settle phase.
   Publishing a zero Twist would keep the command fresh and hold the gait FSM in
   a "commanded to stand still" state; not publishing lets the watchdog expire,
   which is the path a finished Nav2 goal or a dead link takes, and it is what
   puts StateTrotting into HOLD where the posture settle runs. So a settle here
   is implemented as *not publishing*, deliberately.

3. Every segment is specified in SI in the body frame and converted here. The
   conversion is not identity and the envelope is not the one the message type
   suggests -- see the constants below.
"""

from dataclasses import dataclass
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

# Twist -> stick -> SI, mirroring twist_to_inputs (unit gain on the stick, then
# StateTrotting's invNormalize against its own symmetric limits) so that:
#     v_x = 0.4 * linear.x,  v_y = 0.3 * linear.y,  w_z = 0.5 * angular.z
# with every stick clamped to 0.5. Kept as named constants because a wrong
# factor here does not fail -- the robot just moves at a speed nobody asked for.
_STICK_CLAMP = 0.5
_VX_PER_STICK = 0.4  # m/s per unit of ly
_VY_PER_STICK = 0.3  # m/s per unit of lx
_WZ_PER_STICK = 0.5  # rad/s per unit of rx

# Reachable envelope, which is what the stick clamp allows.
VX_MAX = _STICK_CLAMP * _VX_PER_STICK  # 0.20 m/s
VY_MAX = _STICK_CLAMP * _VY_PER_STICK  # 0.15 m/s
WZ_MAX = _STICK_CLAMP * _WZ_PER_STICK  # 0.25 rad/s

# Defaults are the MEASURED operating points, not the envelope edges.
#
# 0.10 m/s is the validated forward point (docs/results/ml35-f4-parcial.md).
# 0.10 rad/s of yaw is below the ~0.13 rad/s where the balance QP's yaw moment
# saturates, measured in phase 2 -- asking for more buys nothing and drags feet.
# Do not lower the forward speed thinking it is safer: below ~0.05 m/s the step
# length goes millimetric under an 8 cm foot lift and the robot marches in place
# and destabilises. That is measured too.
CRUISE_MPS = 0.10
STRAFE_MPS = 0.08
TURN_RPS = 0.10

# Tracking measured on 20/08/2026, three passes of the routine against real
# /demo/odom (scratchpad exp/csv/routine.csv). These are the numbers that make
# the pattern close; without them the choreography walks out of the display
# area.
#
#   in-place turn: 23.4 deg measured against 24.06 requested -> 0.973
#   arc:           46.5 deg measured against 46.98 requested -> 0.990
#
# Yaw is under-tracked by 1-3%, so a turn commanded by time falls short.
# Commanding the angle divided by these gains is what closes the figure.
YAW_TRACKING_SPOT = 0.973
YAW_TRACKING_ARC = 0.990

# The gait walks backward 36% FASTER than forward with the same command:
# 0.132-0.139 m/s measured against 0.10 commanded, versus 0.098-0.102 for
# forward. Not compensated here because the choreography below does not use
# any forward/reverse pair -- it closes by geometry, not by error
# cancellation. Recorded because any mirrored pair someone adds will need it.
AFT_OVERSPEED = 1.36


@dataclass(frozen=True)
class Segment:
    """One commanded movement of the choreography, in SI, body frame."""

    name: str
    vx: float
    vy: float
    wz: float
    duration_s: float


def _spot_turn(name: str, degrees: float, turn: float) -> 'Segment':
    """Build an in-place turn that actually sweeps `degrees`, sign included."""
    seconds = abs(math.radians(degrees)) / turn / YAW_TRACKING_SPOT
    return Segment(name, 0.0, 0.0, turn if degrees > 0 else -turn, seconds)


def _arc(name: str, degrees: float, cruise: float, turn: float) -> 'Segment':
    """Build a walking arc that actually sweeps `degrees`, sign included."""
    seconds = abs(math.radians(degrees)) / turn / YAW_TRACKING_ARC
    return Segment(name, cruise, 0.0, turn if degrees > 0 else -turn, seconds)


def default_choreography(cruise: float = CRUISE_MPS,
                         strafe: float = STRAFE_MPS,
                         turn: float = TURN_RPS,
                         move_s: float = 8.0) -> tuple:
    """
    Build the exhibition pattern: what the robot can do, in a closed loop.

    Closure is by GEOMETRY, not by cancelling errors. Every element returns the
    robot to where it started on its own:

      - the box: four equal sides with a 90 deg left turn between them;
      - the circle: four quarter-arcs in the same direction, 360 deg total;
      - the strafe pair: left then right, same duration.

    The box is also what cancels the lateral crab documented in
    ../../../docs/results/ml35-postura-parada.md: the robot creeps about 2% of
    forward distance to its left, and four sides 90 deg apart point that creep
    in four opposing directions, so it sums to roughly zero instead of
    accumulating.

    What was measured NOT to work, and must not come back: pairing
    `arc-left(+cruise, +turn)` with `arc-right(+cruise, -turn)`. Flipping
    the sign of wz is not the mirror of an arc -- the mirror is the time reverse,
    (v, w) -> (-v, -w). The sign-flipped pair traced an S and walked the robot
    1.32 m in x and 0.89 m in y PER PASS, off the display area in minutes
    (measured 20/08/2026, three passes).

    Residual drift is now dominated by the 1-3% yaw tracking error left after
    YAW_TRACKING_*, which makes the whole figure precess slowly about its own
    centre rather than translate away. Precession keeps the robot on stage;
    translation does not. Absolute pose is still not closed by this node -- that
    is navigation's job by contract, and the robot has no absolute reference.
    """
    box = []
    for index in range(4):
        box.append(Segment('forward %d/4' % (index + 1), cruise, 0.0, 0.0, move_s))
        box.append(_spot_turn('turn 90 deg %d/4' % (index + 1), 90.0, turn))
    circle = [_arc('arc 90 deg %d/4' % (index + 1), 90.0, cruise, turn)
              for index in range(4)]
    strafe_s = move_s * 0.5
    return tuple(box + circle + [
        Segment('left side', 0.0, strafe, 0.0, strafe_s),
        Segment('right side', 0.0, -strafe, 0.0, strafe_s),
    ])


class Schedule:
    """
    Timeline of segments separated by settle gaps, resolvable by elapsed time.

    Free of ROS and of clocks on purpose: `elapsed` is passed in, so the
    ordering is unit-testable without a simulator.
    """

    def __init__(self, segments, settle_s: float, loop: bool = True) -> None:
        """Interleave a settle gap after every segment."""
        self._steps: list = []
        for segment in segments:
            if segment.duration_s > 0.0:
                self._steps.append((segment, segment.duration_s))
            if settle_s > 0.0:
                self._steps.append((None, settle_s))
        self._total = sum(duration for _, duration in self._steps)
        self._loop = loop

    @property
    def total_s(self) -> float:
        """Length of one pass through the pattern, settles included."""
        return self._total

    @property
    def segment_count(self) -> int:
        """Count the commanded movements, settles excluded."""
        return sum(1 for segment, _ in self._steps if segment is not None)

    def at(self, elapsed: float):
        """
        Return (segment_or_None, finished).

        A `None` segment means settling: the caller must publish nothing, which
        is what stops the robot and lets the posture settle run.
        """
        if self._total <= 0.0:
            return None, True
        if elapsed < 0.0:
            return None, False
        if elapsed >= self._total:
            if not self._loop:
                return None, True
            elapsed = elapsed % self._total

        remaining = elapsed
        for segment, duration in self._steps:
            if remaining < duration:
                return segment, False
            remaining -= duration
        # Only reachable on floating-point edges at the very end of the pass.
        return None, False


def to_twist(vx: float, vy: float, wz: float) -> Twist:
    """
    Convert a body-frame SI command into the Twist the bridge expects.

    Clamps to the reachable envelope rather than letting `twist_to_inputs` clamp
    silently, so a mis-specified choreography is visible here.
    """
    message = Twist()
    message.linear.x = max(-_STICK_CLAMP, min(_STICK_CLAMP, vx / _VX_PER_STICK))
    message.linear.y = max(-_STICK_CLAMP, min(_STICK_CLAMP, vy / _VY_PER_STICK))
    message.angular.z = max(-_STICK_CLAMP, min(_STICK_CLAMP, wz / _WZ_PER_STICK))
    return message


class DemoRoutine(Node):
    """Publish a looping choreography on /demo/cmd_vel for an exhibition."""

    def __init__(self) -> None:
        """Declare parameters, wire the publisher, and arm the tick."""
        super().__init__('demo_routine')

        self._rate_hz = self.declare_parameter('rate_hz', 20.0).value
        self._settle_s = self.declare_parameter('settle_s', 5.0).value
        self._move_s = self.declare_parameter('move_s', 8.0).value
        self._cruise = self.declare_parameter('cruise_mps', CRUISE_MPS).value
        self._strafe = self.declare_parameter('strafe_mps', STRAFE_MPS).value
        self._turn = self.declare_parameter('turn_rps', TURN_RPS).value
        self._loop = self.declare_parameter('loop', True).value
        self._min_z = self.declare_parameter('min_z', 0.28).value
        self._stand_z = self.declare_parameter('stand_z', 0.30).value

        self._schedule = Schedule(
            default_choreography(self._cruise, self._strafe, self._turn, self._move_s),
            self._settle_s, self._loop)

        self._publisher = self.create_publisher(Twist, '/demo/cmd_vel', 10)
        self.create_subscription(
            Odometry, '/demo/odom', self._on_odom,
            QoSPresetProfiles.SENSOR_DATA.value)

        self._z = None
        self._started_at = None
        self._last_label = None
        self._down = False

        self.create_timer(1.0 / self._rate_hz, self._tick)
        self.get_logger().info(
            'demo routine: %d segments, %.1f s per movement, %.1f s of '
            'posture settle between them, %.1f s per pass. Waiting for the '
            'robot to stand up (z >= %.2f m).'
            % (self._schedule.segment_count, self._move_s, self._settle_s,
               self._schedule.total_s, self._stand_z))

    def _on_odom(self, message: Odometry) -> None:
        """Track body height, the cheapest fallen-robot check available."""
        self._z = message.pose.pose.position.z

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _tick(self) -> None:
        """Publish the active segment, or nothing while settling."""
        if self._z is None:
            return

        # Wait for the stand-up sequence twist_to_inputs drives. Commanding
        # before the FSM reaches "fixed stand" is the documented way to get a
        # robot that never enters trotting.
        if self._started_at is None:
            if self._z < self._stand_z:
                return
            self._started_at = self._now()
            self.get_logger().info('robot standing; starting the routine')
            return

        # A fallen robot is not something to keep commanding. Stop publishing and
        # let the controller's RECOVER work; resume when it is back up.
        if self._z <= self._min_z:
            if not self._down:
                self.get_logger().warn(
                    'robot fallen (z = %.3f m): commands suspended until it '
                    'stands back up' % self._z)
                self._down = True
            return
        if self._down:
            self.get_logger().info('robot standing again; resuming the routine')
            self._down = False
            self._started_at = self._now()

        segment, finished = self._schedule.at(self._now() - self._started_at)
        if finished:
            self._log_phase('routine finished')
            return

        if segment is None:
            # Settle: publish nothing. Silence is the stop command.
            self._log_phase('posture settle')
            return

        self._log_phase(segment.name)
        self._publisher.publish(to_twist(segment.vx, segment.vy, segment.wz))

    def _log_phase(self, label: str) -> None:
        """Log once per phase change, so the console reads as a narration."""
        if label != self._last_label:
            self.get_logger().info(label)
            self._last_label = label


def main(args=None) -> None:
    """Spin the routine until interrupted."""
    rclpy.init(args=args)
    node = DemoRoutine()
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
