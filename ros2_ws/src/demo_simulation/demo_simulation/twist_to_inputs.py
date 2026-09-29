"""
Translate /demo/cmd_vel into what the gait controller actually accepts.

Runs on: x86 workstation ONLY, in the `sim` container, next to Gazebo.

WHY THIS NODE EXISTS

unitree_guide_controller does not subscribe to geometry_msgs/Twist. It
subscribes to control_input_msgs/Inputs on /control_input, which models a
gamepad: a `command` button code plus four normalized stick axes in [-1, 1].
Nothing in the stack speaks that message, and nothing should have to — the
topic contract (CLAUDE.md) says the plant is driven by /demo/cmd_vel, and Nav2
must not learn that the robot has legs.

So this node owns three jobs that the diff-drive plant never needed:

1. Walk the gait state machine up to FIXEDSTAND. A quadruped does not accept
   velocity commands from a cold start; it has to stand up first. TROTTING is
   entered only when a non-zero Twist arrives, so an idle robot remains in the
   stable stand controller.
2. Convert Twist to normalized stick axes once trotting.
3. Own command freshness. Inputs is a gamepad message: it has no timeout, and
   the controller acts on the last value it received, forever. A publisher that
   simply stops -- `ros2 topic pub` killed by `timeout`, a Nav2 goal that ends,
   a crashed node -- therefore leaves the robot walking on a command nobody is
   sending any more, and the log looks identical to normal operation. This node
   publishes on a fixed tick and zeroes the sticks once the last Twist is older
   than _CMD_TIMEOUT_S, so silence means stop.

Freshness lives here and not in the controller on purpose: the controller
cannot tell "the operator wants zero" from "the link died", and this node is
the only place that sees the arrival times.

F4 MAPPING

Twist carries SI units; Inputs carries normalized stick positions.  A tempting
mapping is to divide by the hard-coded StateTrotting limits (0.4 m/s forward,
0.3 m/s lateral and 0.5 rad/s yaw).  Execution disproved that mapping: a modest
linear.x=0.03 becomes ly=0.075 and made the Go2 fall from z=0.355 m to 0.073 m
in two seconds, without a controller error.

F3 proved normalized axes up to 0.03 stable.  F4 therefore preserves unit gain
and clamps every stick to that measured envelope.  This is deliberately slow,
but it preserves the public Twist contract and fails safe when Nav2 requests a
higher velocity.  F5 must calibrate the command-to-motion relationship against
legged odometry before widening the envelope.  Lateral and yaw signs are
negated because the upstream controller negates those axes internally.
"""

from control_input_msgs.msg import Inputs
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

# Gait FSM button codes, read from the controller's own sources (not guessed):
# StatePassive::checkChange, StateFixedDown::checkChange and
# StateFixedStand::checkChange in unitree_guide_controller/src/FSM/.
#   PASSIVE --(2)--> FIXEDDOWN --(2)--> FIXEDSTAND --(4)--> TROTTING
_CMD_NONE = 0
_CMD_STAND_STEP = 2
_CMD_START_TROT = 4

# Wall-clock seconds to hold between FSM transitions.
#
# LOAD-BEARING, and 5 s is not padding. Each FSM state gates its own exit on
# percent_ >= 1.5 (roughly 1.2 s of sim time), and the robot also has to
# physically settle from the spawn drop. The F2 spike first fired all three
# transitions inside ~2 s of wall time; the robot ended up collapsed at
# z ~= 0.07 m, which is indistinguishable from "the robot never stood" unless
# you check the pose directly. Nothing in the log says anything is wrong.
_TRANSITION_HOLD_S = 5.0
_TICK_PERIOD_S = 1.0
_TICKS_PER_TRANSITION = int(_TRANSITION_HOLD_S / _TICK_PERIOD_S)

# Maximum normalized stick magnitude.
#
# This was 0.03, "the envelope proven stable in F3".  That number is void: it
# was measured while the gait never activated, so it describes how hard the
# balance controller could shove a robot with four feet planted, not how fast
# it can walk.
#
# The gait has a design point and 0.03 is nowhere near it.  Step length is
# roughly v * (t_swing * (1 - phase) + t_stance / 2) ~= 0.34 * v, and the
# command reaches the placement law only through k_x * (v_body - v_goal) with
# k_x = 0.005.  At stick 0.03 -> 0.012 m/s that is a 4 mm step requested by a
# 20 um shift of the foot target, under an 8 cm foot lift: all of the
# disturbance of stepping and none of the momentum.  Hence a robot that marches
# in place and lets any yaw drift feed on itself.
#
# 0.5 -> 0.2 m/s, a step of about 7 cm, which is what a trot of this period is
# shaped for.  Nav2 still fails safe: anything faster is clamped here.
_SAFE_STICK_LIMIT = 0.5

# Command freshness. The tick has to be several times faster than the timeout,
# otherwise the age measured at each tick is dominated by the tick itself; the
# timeout in turn has to tolerate the slowest publisher the demo uses, which is
# the 10 Hz `ros2 topic pub` of the operation guide (0.1 s between messages).
_CONTROL_PERIOD_S = 0.05
_CMD_TIMEOUT_S = 0.3

# The FSM handshake is wider than one message, and the tick above will close it
# if you let it. The controller does not act on messages: the subscription
# writes into a single control_inputs_ struct and the update loop reads whatever
# is there when it runs. Publishing `command=4` once and then a control message
# 50 ms later means the 4 can be overwritten before any update sees it -- and
# the failure is silent, because the sticks keep streaming, the bridge logs
# "fixed stand -> trotting", and the robot just stands there in fixed stand.
# Measured on 18/08/2026, exactly that way.
#
# Repeating the start command costs nothing: StateTrotting::checkChange treats
# anything that is not 1 (passive) or 2 (fixed stand) as "stay trotting". So it
# is repeated across a window instead of sent once.
_START_TROT_HOLD_S = 0.5
_START_TROT_TICKS = int(_START_TROT_HOLD_S / _CONTROL_PERIOD_S)


# Service that replays the FSM ramp-up after a teleport.
#
# WHY THIS EXISTS
#
# The cockpit reset teleports the robot (sim_control_relay). StateTrotting does
# not notice: the HOLD reference -- pcd_ and yaw_cmd_, see
# StateTrotting::captureBodyReference -- is captured ONCE behind a latch that
# only a clean walking command releases. After the teleport the controller keeps
# chasing the pose the robot had BEFORE, and the yaw axis, limited near 5.3 N.m,
# cannot serve that target: measured on 2026-08-26, Mz on the rail on 100% of
# ticks with a constant residual, the robot dragged 0.87 m away from the spawn
# point and ending up COLLAPSED at z=0.131 m versus 0.353 m walking height.
# Nothing in any log flags it; the cockpit stays green.
#
# StateTrotting::enter() re-anchors both at the current pose. So the fix is to
# leave TROTTING and come back -- and the place for that is here, not in the
# controller: /control_input has ONE writer per project, and the reason is
# written under _START_TROT_HOLD_S.
#
# WHY TWO SERVICES, NOT ONE
#
# The first version re-anchored AFTER teleporting, in a single service. It works
# with the robot at rest and fails with Nav2 driving, which is the real case.
# Measured on 2026-08-26, reset during a live /demo/cmd_vel stream at 10 Hz:
#
#     t[s]      x        y       z   yaw[deg]
#      0.1   0.215    0.031   0.337     84.2     <- reset requested
#      1.1   0.483   -0.024   0.162     90.6     <- COLLAPSED, and slides 0.27 m
#     40.0   0.307   -0.084   0.159     82.7     <- still collapsed, writhing
#
# `SetEntityPose` repositions the body and PRESERVES VELOCITY. A robot walking at
# ~0.2 m/s with its legs mid-swing is dropped from 0.15 m still travelling, and
# falls.
#
# So the correct order is to STOP BEFORE teleporting, not to re-anchor after:
#
#     hold   -> FIXEDSTAND, axes centred; the robot plants its feet and stops
#     (sim_control_relay teleports here, with the robot at rest)
#     resume -> settles and returns to TROTTING, whose enter() re-anchors at
#               the new pose
HOLD_SERVICE = '/demo/gait/hold'
RESUME_SERVICE = '/demo/gait/resume'


class _RestandSequence:
    """
    Hold the robot in FIXEDSTAND, then return it to TROTTING re-anchored.

    Pure on purpose, like _CommandGate: `now` is passed as an argument, so the
    sequence is testable without ROS.

    The `holding` state has no deadline: it is ended by `resume`, called by the
    relay after the teleport has finished. A deadline here would be a race
    against the other process's service call.
    """

    def __init__(
        self,
        settle_s: float = _TRANSITION_HOLD_S,
        trot_ticks: int = _START_TROT_TICKS,
    ) -> None:
        self._settle_s = settle_s
        self._trot_ticks = trot_ticks
        self._phase = 'idle'
        self._until = 0.0
        self._left = 0

    @property
    def active(self) -> bool:
        """While true, the axes stay centred: see next_command."""
        return self._phase != 'idle'

    @property
    def holding(self) -> bool:
        """Return True between `hold` and `resume`: the teleport window."""
        return self._phase in ('stand', 'holding')

    def hold(self) -> None:
        """Leave TROTTING for FIXEDSTAND and wait there."""
        self._phase = 'stand'

    def resume(self, now: float) -> None:
        """Settle and return to TROTTING. Ignored if no hold is in progress."""
        if not self.holding:
            return
        self._phase = 'settling'
        self._until = now + self._settle_s
        self._left = self._trot_ticks

    def next_command(self, now: float) -> int:
        """Command byte for this tick, consuming the sequence."""
        if self._phase == 'stand':
            # Exactly ONE 2, never repeated. If held, it takes FIXEDSTAND to
            # FIXEDDOWN and the robot LIES DOWN -- StateFixedStand::checkChange,
            # case 2. It is the difference between resettling and collapsing
            # the robot.
            self._phase = 'holding'
            return _CMD_STAND_STEP

        if self._phase == 'holding':
            # Explicit zeros, not silence: the byte stored in the controller
            # persists, and ceasing to publish would leave the `2` above in
            # force until the next tick that published anything.
            return _CMD_NONE

        if self._phase == 'settling':
            if now < self._until:
                return _CMD_NONE
            self._phase = 'trot'

        if self._phase == 'trot':
            if self._left > 0:
                self._left -= 1
                return _CMD_START_TROT
            self._phase = 'idle'

        return _CMD_NONE


class _CommandGate:
    """
    Hold the last velocity command and decide whether it is still valid.

    Deliberately free of ROS and of wall clocks: `now` is passed in, so the
    staleness rule is unit-testable and the node stays free to feed it sim
    time (which is what it does -- the whole stack runs on /clock, and a paused
    simulator must not age a command).
    """

    def __init__(self, timeout_s: float = _CMD_TIMEOUT_S) -> None:
        self._timeout_s = timeout_s
        self._sticks = (0.0, 0.0, 0.0)
        self._stamp: float | None = None

    def record(self, twist: Twist, now: float) -> None:
        """Take a new command and stamp its arrival."""
        message = _twist_to_inputs(twist)
        self._sticks = (message.lx, message.ly, message.rx)
        self._stamp = now

    def is_stale(self, now: float) -> bool:
        """Return whether the last command is too old to act on."""
        if self._stamp is None:
            return True
        return now - self._stamp > self._timeout_s

    def sample(self, now: float) -> Inputs:
        """Return the command to publish now: the last one, or centered sticks."""
        message = Inputs()
        message.command = _CMD_NONE
        if not self.is_stale(now):
            message.lx, message.ly, message.rx = self._sticks
        return message


class _StartLatch:
    """Repeat the trot start command for a window, then go quiet."""

    def __init__(self, ticks: int = _START_TROT_TICKS) -> None:
        self._ticks = ticks
        self._left = 0

    def arm(self) -> None:
        self._left = self._ticks

    def next_command(self) -> int:
        """Return the command byte for this tick, consuming one repeat."""
        if self._left <= 0:
            return _CMD_NONE
        self._left -= 1
        return _CMD_START_TROT


class TwistToInputs(Node):
    """Bridge /demo/cmd_vel -> /control_input, driving the gait FSM first."""

    def __init__(self) -> None:
        super().__init__('twist_to_inputs')

        self._publisher = self.create_publisher(Inputs, '/control_input', 10)
        self._subscription = self.create_subscription(
            Twist, '/demo/cmd_vel', self._on_twist, 10,
        )

        self._stage = 'settling'
        self._ticks = 0
        self._timer = self.create_timer(_TICK_PERIOD_S, self._advance_gait_fsm)

        # Command path: the subscription only records, the tick publishes.  A
        # single writer keeps "what the controller last heard" a function of
        # time alone, which is what makes the watchdog meaningful.
        self._gate = _CommandGate()
        self._start_latch = _StartLatch()
        self._restand = _RestandSequence()
        self._stale = True
        self._control_timer = self.create_timer(
            _CONTROL_PERIOD_S, self._publish_control_input,
        )

        # The caller is sim_control_relay, one call on each side of the
        # teleport. See HOLD_SERVICE above. Trigger and not a topic because the
        # party doing the reset needs to know whether anyone was on the other
        # end: a reset that teleports without stopping and re-anchoring the
        # robot leaves it collapsed, and that is exactly the silent failure
        # mode to be avoided.
        self._hold_service = self.create_service(
            Trigger, HOLD_SERVICE, self._on_hold,
        )
        self._resume_service = self.create_service(
            Trigger, RESUME_SERVICE, self._on_resume,
        )

        self.get_logger().info(
            'twist_to_inputs up: /demo/cmd_vel -> /control_input. '
            'Standing the robot up before accepting velocity commands.'
        )

    def _advance_gait_fsm(self) -> None:
        """
        Walk PASSIVE -> FIXEDDOWN -> FIXEDSTAND, then wait for motion.

        Wall-clock paced on purpose — see _TRANSITION_HOLD_S.
        """
        self._ticks += 1
        if self._ticks < _TICKS_PER_TRANSITION:
            return
        self._ticks = 0

        if self._stage == 'settling':
            self._send_command(_CMD_STAND_STEP)
            self._stage = 'fixed_down'
            self.get_logger().info('gait FSM: passive -> fixed down')
        elif self._stage == 'fixed_down':
            self._send_command(_CMD_STAND_STEP)
            self._stage = 'fixed_stand'
            self.get_logger().info('gait FSM: fixed down -> fixed stand')
        elif self._stage == 'fixed_stand':
            self._timer.cancel()
            self.get_logger().info(
                'gait FSM: fixed stand. Waiting for a non-zero /demo/cmd_vel '
                'before entering trotting.'
            )

    def _send_command(self, command: int) -> None:
        message = Inputs()
        message.command = command
        self._publisher.publish(message)

    def _on_twist(self, twist: Twist) -> None:
        """Record the command and its arrival time; publishing is the tick's job."""
        self._gate.record(twist, self._now())

        if self._stage == 'fixed_stand':
            if not _has_motion_command(twist):
                return
            # Arm, do not publish: the tick is the only writer of control
            # messages, which is what makes the repeat above reliable.
            self._start_latch.arm()
            self._stage = 'trotting'
            self.get_logger().info(
                'gait FSM: fixed stand -> trotting. Now driven by /demo/cmd_vel.'
            )

        # Velocity commands before TROTTING are not queued, they are dropped:
        # _publish_control_input only publishes once trotting. Forwarding them
        # would inject axis values while the robot is still standing up and
        # knock it over mid-transition.

    def _publish_control_input(self) -> None:
        """Publish the current command, or centered sticks if it went stale."""
        if self._stage != 'trotting':
            return

        now = self._now()

        if self._restand.active:
            # Axes centred throughout the sequence. Injecting an axis value
            # into a robot that is standing up is what knocks it over mid
            # transition -- the same reason commands received before TROTTING
            # are discarded, not queued, in _on_twist.
            message = Inputs()
            message.command = self._restand.next_command(now)
            self._publisher.publish(message)
            return
        stale = self._gate.is_stale(now)
        if stale != self._stale:
            # Worth a line: this is the difference between "the operator asked
            # for zero" and "nobody is publishing", and the two look identical
            # from the controller side.
            self.get_logger().info(
                'cmd_vel watchdog: stale, holding position'
                if stale else 'cmd_vel watchdog: command stream is live'
            )
            self._stale = stale

        message = self._gate.sample(now)
        message.command = self._start_latch.next_command()
        self._publisher.publish(message)

    def _on_hold(self, request, response):
        """
        Take the robot to FIXEDSTAND and keep it there, still, for the teleport.

        Responds immediately: whoever waits for the robot to stop is the relay,
        which is the one that knows when it will teleport.
        """
        del request
        response.success = True
        if self._stage != 'trotting':
            # Nothing to hold: the robot has not yet entered TROTTING, and the
            # enter() that will happen when it does already captures the new
            # pose.
            response.message = f'gait in {self._stage}: nothing to hold'
            return response

        self._restand.hold()
        response.message = 'gait: trotting -> fixed stand, robot still'
        self.get_logger().info(response.message)
        return response

    def _on_resume(self, request, response):
        """
        Settle and return to TROTTING, whose enter() re-anchors pcd_ and yaw_cmd_.

        Responds immediately and executes on the following ticks: the sequence
        takes _TRANSITION_HOLD_S + _START_TROT_HOLD_S, and blocking the service
        for that long would make the cockpit reset button look stuck.
        """
        del request
        response.success = True
        if not self._restand.holding:
            response.message = 'gait: no hold in progress, nothing to resume'
            return response

        self._restand.resume(self._now())
        response.message = 'gait: fixed stand -> trotting, pose re-anchored'
        self.get_logger().info(response.message)
        return response

    def _now(self) -> float:
        """Seconds on the node clock (sim time here — see the launch file)."""
        return self.get_clock().now().nanoseconds * 1e-9


def _twist_to_inputs(twist: Twist) -> Inputs:
    """Convert SI velocity commands to the controller's normalized axes."""
    message = Inputs()
    message.command = _CMD_NONE
    # StateTrotting maps ly directly, but negates lx and rx.
    message.ly = _to_safe_stick(twist.linear.x)
    message.lx = -_to_safe_stick(twist.linear.y)
    message.rx = -_to_safe_stick(twist.angular.z)
    message.ry = 0.0
    return message


def _has_motion_command(twist: Twist) -> bool:
    """Return whether a Twist requests translation or yaw motion."""
    return any((
        float(twist.linear.x),
        float(twist.linear.y),
        float(twist.linear.z),
        float(twist.angular.x),
        float(twist.angular.y),
        float(twist.angular.z),
    ))


def _to_safe_stick(value: float) -> float:
    """Apply unit gain and clamp to the empirically stable stick envelope."""
    return max(-_SAFE_STICK_LIMIT, min(_SAFE_STICK_LIMIT, float(value)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwistToInputs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
