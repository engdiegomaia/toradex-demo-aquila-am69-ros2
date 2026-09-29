"""
Continuous patrol under Nav2: sends GOALS in a cycle, indefinitely.

Runs on the x86 workstation alongside Nav2 (`nav_quadruped.launch.py`).

    ros2 run demo_bringup patrol_commander

## How this differs from demo_routine, and why the two cannot coexist

`demo_routine` publishes `/demo/cmd_vel` directly: it knows the velocity and
does not know where the robot is. This node does not publish any velocity --
it sends a goal through the `navigate_to_pose` action and lets Nav2 decide the
velocity, which is what allows obstacle avoidance.

Running both at once puts two publishers on `/demo/cmd_vel` (here via
`collision_monitor`, there directly). This does NOT raise an error:
`twist_to_inputs` obeys whichever message arrived last, alternating between
avoidance and choreography at 20 Hz. The robot moves in spasms and no log
explains it. Pick one of the two.

## Why the goals are a cycle and not a Nav2 waypoint list

Nav2's `waypoint_follower` would also do this, and was rejected: when a goal in
the list fails it ends the whole list, and in an exhibition a badly placed
obstacle would end the demo. Here a goal that fails is ABANDONED and the cycle
moves to the next one, which is the behavior an exhibition needs. The
trade-off is that this node cannot tell whether the full route was completed
-- for that use `nav2_simple_commander` in a test, not this node.

## The goal limits are not arbitrary

The global costmap is a ROLLING 20 m window with no map (see
`nav2_params_go2.yaml`). A goal outside it is ACCEPTED and then fails near the
edge, because `allow_unknown: true` lets the planner trace a path through the
unknown. `MAX_GOAL_RADIUS_M` rejects those goals at the door, where the error
still has a name.

## Timeout per goal

At the Go2's measured envelope -- 0.15 m/s forward, 0.12 rad/s yaw -- 4 m
takes ~27 s in the best case, and an avoidance detour doubles that.
`DEFAULT_GOAL_TIMEOUT_S` is generous on purpose: a short limit cancels goals
that were making progress, which looks like a navigation failure and is a
configuration failure. If you tighten this number, measure first.
"""

from dataclasses import dataclass
import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Maximum accepted radius for a goal. The global costmap's rolling window is
# 20 m on a side, i.e. 10 m from the center; 8 m leaves 2 m of margin for the
# robot to drift from the origin during the cycle without the goal falling
# outside the window.
MAX_GOAL_RADIUS_M = 8.0

# Timeout per goal, derived from the MEASURED speed rather than picked
# arbitrarily.
#
# Measured on 20/08/2026 under Nav2: real average speed of 0.021 m/s (peak
# 0.119). The longest leg of the default route is 4.27 m, which is 203 s in a
# straight line; with the 1.3 detour factor, 264 s. 300 s covers this with
# margin.
#
# The average is much lower than the peak because MPPI spends a good part of
# the time correcting heading -- and at a 0.12 rad/s yaw ceiling, correcting
# heading costs time in which the robot barely advances. Do not shorten this
# deadline without measuring again: canceling a goal that was making progress
# looks like a navigation failure and is a configuration one.
DEFAULT_GOAL_TIMEOUT_S = 300.0

# Pause between goals. Exists for the same reason as `demo_routine`'s settle:
# the gait controller needs a command-free interval to settle its posture,
# and sending the next goal the instant the previous one ends does not give
# it that interval.
DEFAULT_SETTLE_S = 2.0


@dataclass(frozen=True)
class Goal:
    """One goal of the cycle, in meters and radians, in the planner's frame."""

    x: float
    y: float
    yaw: float


# Default route: a triangle of three goals, sized against the obstacles in
# `quadruped_objects.sdf`. Every goal is REACHABLE and every LEG requires
# avoidance -- both measured, not estimated.
#
# That world's obstacles: box (1.5, 0.0) half-diagonal 0.21; cylinder
# (3.0, 0.45) r 0.18; box (3.0, -0.55) half-diagonal 0.28; cylinder
# (4.5, 0.0) r 0.12. The Go2's circumscribed radius is 0.383.
#
# Clearance from the STRAIGHT LINE to the nearest obstacle, by point-to-segment
# distance (not by vertical distance at a chosen x, which overestimates the
# clearance):
#
#   (0,0)     -> (4, 1.5)   -0.068 m from the red box       BLOCKED
#   (4, 1.5)  -> (4, -1.5)  -0.003 m from the yellow cylinder BLOCKED
#   (4, -1.5) -> (0,0)      -0.127 m from the blue box      BLOCKED
#
# All three straight lines are blocked, so avoidance is mandatory -- which is
# the point of the scenario. If the robot walks in a straight line, either the
# costmap is empty or it drove through the obstacle; both are failures.
#
# And all three GOALS have margin: +0.887, +0.713, and +0.905 m. A tight goal
# makes Nav2 fail on an impossible arrival, which is easily mistaken for an
# avoidance failure.
#
# THE OBVIOUS-LOOKING 3 m SQUARE DOES NOT WORK, and it is worth recording why:
# the goal (3.0, 0.0) falls in the gap between the green cylinder and the blue
# box. That gap is 0.45-0.18 = 0.27 on one side and -0.55+0.28 = -0.27 on the
# other, i.e. 0.54 m of free width, and the robot needs 2 x 0.383 = 0.77 m.
# The goal is unreachable, and Nav2 ACCEPTS it and only fails after exhausting
# its recoveries -- which reads as "avoidance is not working" and is really an
# impossible goal.
#
# Each goal's yaw is the ARRIVAL bearing -- the direction the robot is already
# walking in when it reaches that goal -- not the departure bearing toward the
# next goal.
#
# The difference cost an entire run. With the departure bearing, every goal
# required a STATIONARY turn of 110 to 139 degrees on arrival: 16 to 20 s at
# the 0.12 rad/s ceiling. And the turn does not stay in place -- measured on
# 20/08/2026, the robot got to within 3.8 cm of the goal (3.976, 1.470 versus
# 4.0, 1.5) and then drifted 0.78 m in y while turning to satisfy the
# orientation, drifting out of the position tolerance. The goal never closed.
#
# With the arrival bearing, the orientation is already satisfied once the
# position is, and the turn toward the next goal happens as part of the next
# leg -- while walking, which is where the Go2 turns best.
DEFAULT_WAYPOINTS = (
    Goal(4.0, 1.5, math.atan2(1.5, 4.0)),
    Goal(4.0, -1.5, -math.pi / 2.0),
    Goal(0.0, 0.0, math.atan2(1.5, -4.0)),
)


def parse_waypoints(flat: list) -> tuple:
    """
    Build the validated goals from the flat parameter list.

    The parameter arrives flat -- [x, y, yaw, x, y, yaw, ...] -- because
    ROS 2 has no parameter type for a list of lists. This makes it easy to
    get the length wrong, and a wrong length silently shifts every following
    goal, so here that is an error, not a warning.
    """
    if len(flat) % 3 != 0:
        raise ValueError(
            'waypoints has %d values, which is not a multiple of 3. The '
            'format is [x, y, yaw, x, y, yaw, ...] in meters and radians.'
            % len(flat))
    if not flat:
        raise ValueError('waypoints is empty; the cycle would have no goal at all.')

    goals = []
    for index in range(0, len(flat), 3):
        x, y, yaw = (float(v) for v in flat[index:index + 3])
        distance = math.hypot(x, y)
        if distance > MAX_GOAL_RADIUS_M:
            raise ValueError(
                'goal %d is %.2f m from the origin, above the limit of %.1f '
                'm. The global costmap rolling window would accept this '
                'goal and fail near its edge.'
                % (index // 3, distance, MAX_GOAL_RADIUS_M))
        goals.append(Goal(x, y, yaw))
    return tuple(goals)


def flatten(goals) -> list:
    """Return the goals in the flat parameter format."""
    flat = []
    for goal in goals:
        flat.extend([goal.x, goal.y, goal.yaw])
    return flat


def to_pose(goal: Goal, frame_id: str, stamp) -> PoseStamped:
    """Build a goal's PoseStamped, with yaw as a quaternion about z."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x = goal.x
    pose.pose.position.y = goal.y
    # Rotation about z only: the robot moves in the plane. Writing the
    # quaternion by hand avoids depending on tf_transformations, which is not
    # in the image.
    pose.pose.orientation.z = math.sin(goal.yaw / 2.0)
    pose.pose.orientation.w = math.cos(goal.yaw / 2.0)
    return pose


class PatrolCommander(Node):
    """Send the cycle's goals one at a time, without stopping, ignoring failures."""

    def __init__(self) -> None:
        """Read the parameters, validate the route, and open the action client."""
        super().__init__('patrol_commander')

        self.declare_parameter('waypoints', flatten(DEFAULT_WAYPOINTS))
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('goal_timeout_s', DEFAULT_GOAL_TIMEOUT_S)
        self.declare_parameter('settle_s', DEFAULT_SETTLE_S)
        self.declare_parameter('loop', True)

        # A parameter error aborts startup. A malformed route that turned
        # into a warning would leave the node running without moving the
        # robot, which is the most expensive symptom to diagnose in this
        # stack.
        self._goals = parse_waypoints(
            list(self.get_parameter('waypoints').value))
        self._frame = self.get_parameter('frame_id').value
        self._timeout = float(self.get_parameter('goal_timeout_s').value)
        self._settle = float(self.get_parameter('settle_s').value)
        self._loop = bool(self.get_parameter('loop').value)

        self._index = 0
        self._sent = 0
        self._succeeded = 0
        self._failed = 0
        self._goal_handle = None
        self._deadline = None
        # Between send_goal_async and the acceptance response the handle is
        # still None. Without this flag the 1 s tick would re-enter _tick and
        # send ANOTHER goal, flooding Nav2 with concurrent goals -- and Nav2
        # accepts them, implicitly canceling the previous one, so the symptom
        # is a stopped robot getting a new goal every time it was about to
        # start walking.
        self._pending = False

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 1 Hz is enough: this node supervises goals that take tens of
        # seconds. A higher rate would only multiply the log.
        self._timer = self.create_timer(1.0, self._tick)

        self.get_logger().info(
            'patrol with %d goals, timeout %.0f s, settle %.1f s, loop %s. '
            'Do NOT run demo_routine at the same time: both become '
            'publishers of /demo/cmd_vel and the robot moves in spasms.'
            % (len(self._goals), self._timeout, self._settle, self._loop))

    def _tick(self) -> None:
        """Send the next goal, or watch the deadline of the one in progress."""
        if self._pending or self._goal_handle is not None:
            self._check_deadline()
            return

        if not self._client.server_is_ready():
            # Not an error: Nav2 is still activating its lifecycle nodes. If
            # this persists, the suspect is autostart or /clock, not this
            # node.
            self.get_logger().info(
                'waiting for the navigate_to_pose action to become ready '
                '(Nav2 activating)', throttle_duration_sec=10.0)
            return

        if self._index >= len(self._goals):
            if not self._loop:
                self.get_logger().info(
                    'route finished: %d of %d goals completed'
                    % (self._succeeded, self._sent))
                self._timer.cancel()
                return
            self._index = 0

        self._send(self._goals[self._index])
        self._index += 1

    def _send(self, goal: Goal) -> None:
        """Send a goal and arm its deadline."""
        message = NavigateToPose.Goal()
        message.pose = to_pose(goal, self._frame, self.get_clock().now().to_msg())

        self._sent += 1
        self.get_logger().info(
            'goal %d: x=%.2f y=%.2f yaw=%.0f deg in "%s"'
            % (self._sent, goal.x, goal.y, math.degrees(goal.yaw), self._frame))

        self._pending = True
        self._deadline = self._elapsed() + self._timeout
        future = self._client.send_goal_async(message)
        future.add_done_callback(self._on_accepted)

    def _on_accepted(self, future) -> None:
        """Keep the handle of an accepted goal, or give up on it if it was rejected."""
        self._pending = False
        handle = future.result()
        if not handle.accepted:
            # Nav2 rejects a goal whose path it does not even attempt:
            # outside the rolling window, or inside an obstacle. Abandoning
            # it and moving on is deliberate.
            self.get_logger().warning(
                'goal %d rejected by Nav2; moving on to the next one'
                % self._sent)
            self._failed += 1
            self._release()
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        """Record the goal's outcome and release the cycle."""
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._succeeded += 1
            self.get_logger().info(
                'goal %d completed (%d of %d)'
                % (self._sent, self._succeeded, self._sent))
        else:
            self._failed += 1
            # Status 5 = ABORTED, 6 = CANCELED. ABORTED here is usually
            # "recoveries exhausted", which on the Go2 is almost always an
            # obstacle inside the inflated radius, not a planner failure.
            self.get_logger().warning(
                'goal %d ended with status %d; abandoned, continuing the '
                'cycle (%d failures)' % (self._sent, status, self._failed))
        self._release()

    def _check_deadline(self) -> None:
        """Cancel the goal in progress if it is past its deadline."""
        if self._deadline is None or self._elapsed() < self._deadline:
            return
        if self._goal_handle is None:
            # Past the deadline and acceptance never arrived. There is
            # nothing to cancel; release the cycle, or it would get stuck
            # here forever.
            self.get_logger().warning(
                'goal %d was never accepted within %.0f s; releasing the '
                'cycle' % (self._sent, self._timeout))
            self._failed += 1
            self._release()
            return
        self.get_logger().warning(
            'goal %d exceeded %.0f s; canceling. If this repeats, measure '
            'before shortening the deadline: canceling a goal that was '
            'progressing looks like a navigation failure and is a '
            'configuration one.' % (self._sent, self._timeout))
        self._goal_handle.cancel_goal_async()
        self._deadline = None

    def _release(self) -> None:
        """Release the cycle after the settle, so posture can settle."""
        self._goal_handle = None
        self._pending = False
        self._deadline = None
        # The settle is implemented as a delay on the next send, not as a
        # blocking pause: blocking the executor would stop the action's
        # callbacks.
        if self._settle > 0.0:
            self._timer.cancel()
            self._timer = self.create_timer(self._settle, self._resume)

    def _resume(self) -> None:
        """Return to periodic supervision after the settle."""
        self._timer.cancel()
        self._timer = self.create_timer(1.0, self._tick)

    def _elapsed(self) -> float:
        """Seconds since the node clock's epoch."""
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None) -> None:
    """Run the patrol until interrupted."""
    rclpy.init(args=args)
    node = PatrolCommander()
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
