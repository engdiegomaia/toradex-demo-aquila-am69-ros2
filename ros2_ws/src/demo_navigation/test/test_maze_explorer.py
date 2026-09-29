"""
State machine and status contract of the exploration executive.

What is under test is not the frontier algorithm -- that lives in
`frontier.py` and has its own tests. It is the part the cockpit and Nav2
see: the state vocabulary, the published JSON, and the three transitions
whose failure costs an entire acceptance run -- two simultaneous searches, a
frontier goal that returns forever, and an old marker treated as fresh.
"""

import json
import math
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from demo_navigation import maze_explorer as maze_explorer_module
from demo_navigation.frontier import Frontier
from demo_navigation.maze_explorer import MazeExplorer, STATES
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from std_srvs.srv import Trigger


@pytest.fixture
def node():
    """Capture status from a MazeExplorer instead of publishing it."""
    rclpy.init()
    explorer = MazeExplorer()
    published: list[dict] = []
    explorer._status_pub.publish = lambda message: published.append(
        json.loads(message.data))
    explorer.published = published
    yield explorer
    explorer.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def trigger(node) -> Trigger.Response:
    """Return the empty service response delivered by rclpy to the callback."""
    del node
    return Trigger.Response()


def test_starts_idle_so_the_launch_never_moves_the_robot(node) -> None:
    """Keep the node idle alongside Nav2 until it receives a request."""
    assert node._state == 'idle'


def test_status_carries_every_field_the_cockpit_reads(node) -> None:
    """The HUD reads these fields by name; a missing one blanks part of the screen."""
    node._publish_status()
    payload = node.published[-1]
    for field in ('state', 'elapsed_s', 'frontier_count', 'goal',
                  'blacklisted', 'marker_visible', 'message'):
        assert field in payload, field
    assert payload['state'] in STATES


def test_status_reports_the_goal_in_flight_not_just_a_count(node) -> None:
    """Without the goal's coordinates, the operator doesn't know where it went."""
    node._current = Frontier(x=1.5, y=-2.25, cells=12, information_gain_m=0.6)
    node._publish_status()
    goal = node.published[-1]['goal']
    assert goal['x'] == 1.5 and goal['y'] == -2.25
    assert goal['cells'] == 12


def test_every_reachable_state_is_in_the_published_vocabulary(node) -> None:
    """`_publish_status` asserts this; the test guarantees the assertion is possible."""
    for state in ('idle', 'waiting_map', 'selecting', 'navigating',
                  'homing_exit', 'completed', 'failed', 'cancelled'):
        node._state = state
        node._publish_status()
        assert node.published[-1]['state'] == state


def test_start_takes_the_robot_out_of_idle(node) -> None:
    """Only the service arms the search -- never the map arriving on its own."""
    response = node._start(None, trigger(node))
    assert response.success is True
    assert node._state == 'waiting_map'


def test_start_refuses_a_second_run_while_one_is_in_flight(node) -> None:
    """
    Two searches on the same Nav2 preempt each other and the log doesn't flag it.

    The navigate_to_pose server accepts only one goal: the second aborts the
    first, whose callback arrives later and overwrites the new one's state.
    This is the same feedback that nav_trial documents under "concurrent
    goals".
    """
    node._start(None, trigger(node))
    for state in ('waiting_map', 'selecting', 'navigating', 'homing_exit'):
        node._state = state
        response = node._start(None, trigger(node))
        assert response.success is False, state
        assert node._state == state


def test_start_is_allowed_again_after_a_terminal_state(node) -> None:
    """A run that has ended must not lock the cockpit forever."""
    for state in ('completed', 'failed', 'cancelled', 'idle'):
        node._state = state
        assert node._start(None, trigger(node)).success is True


def test_start_clears_the_blacklist_of_the_previous_run(node) -> None:
    """The blacklist is valid within a run, not across cold starts."""
    node._blacklist.append((1.0, 2.0))
    node._homing_failures = 2
    node._start(None, trigger(node))
    assert node._blacklist == []
    assert node._homing_failures == 0


def test_cancel_is_idempotent(node) -> None:
    """The cockpit may call it twice; the Nav2 reset gets called along with it."""
    node._start(None, trigger(node))
    first = node._cancel(None, trigger(node))
    second = node._cancel(None, trigger(node))
    assert first.success is True and second.success is True
    assert node._state == 'cancelled'


def test_cancel_retires_the_epoch_so_late_callbacks_are_ignored(node) -> None:
    """
    The epoch is what stops a stale callback from resurrecting the search.

    Without it, the cancelled goal's result arrives after the `cancel`,
    finds the node in `cancelled` and returns it to `selecting` -- the robot
    starts moving again after the operator has ordered it to stop.
    """
    node._start(None, trigger(node))
    stale = node._epoch
    node._cancel(None, trigger(node))
    assert node._epoch != stale


def test_blacklisting_returns_to_selecting_and_remembers_the_failure(node) -> None:
    """A frontier that failed cannot be the immediate next choice."""
    node._state = 'navigating'
    node._current = Frontier(x=3.0, y=4.0, cells=10, information_gain_m=0.5)
    node._blacklist_current('frontier ended with status 6')
    assert node._state == 'selecting'
    assert (3.0, 4.0) in node._blacklist
    assert node._current is None


def test_blacklist_survives_within_the_run_and_filters_by_radius(node) -> None:
    """
    The filter is by radius, not by exact coordinate equality.

    The frontier reappears shifted by a few centimeters on every map update;
    comparing exact coordinates would make the blacklist never match, and
    the robot would return to the same wall until the total deadline runs
    out.
    """
    radius = float(node.get_parameter('blacklist_radius_m').value)
    node._blacklist.append((3.0, 4.0))
    near = Frontier(x=3.0 + radius / 2.0, y=4.0, cells=10,
                    information_gain_m=0.5)
    far = Frontier(x=3.0 + radius * 2.0, y=4.0, cells=10,
                   information_gain_m=0.5)
    assert _is_blacklisted(node, near) is True
    assert _is_blacklisted(node, far) is False


def _is_blacklisted(node, frontier) -> bool:
    """Apply the same predicate as `_begin_selection` to candidates."""
    radius = float(node.get_parameter('blacklist_radius_m').value)
    return any(math.hypot(frontier.x - x, frontier.y - y) <= radius
               for x, y in node._blacklist)


def _is_suppressed(node, frontier) -> bool:
    """Combine both lists as the candidate filter does."""
    radius = float(node.get_parameter('blacklist_radius_m').value)
    return any(math.hypot(frontier.x - x, frontier.y - y) <= radius
               for x, y in list(node._blacklist) + list(node._refused)
               + list(node._timed_out))


class _Wrapped:
    """What `get_result_async()` delivers: status plus the action's result."""

    def __init__(self, status: int) -> None:
        self.status = status
        # Nav2 action servers populate `.result` even for an aborted goal
        # (error_code/error_msg plus an empty path) -- `None` here would be a
        # test-double gap, not a real possibility, so default to the shape a
        # real ComputePathToPose result actually has.
        self.result = SimpleNamespace(
            error_code=0, error_msg='', path=SimpleNamespace(poses=[]))


class _Future:
    """An already-resolved future, the way rclpy calls the callback."""

    def __init__(self, value) -> None:
        self._value = value

    def result(self):
        return self._value


def test_a_frontier_the_planner_refuses_is_retired(node) -> None:
    """
    The failure mode that consumed 77% of the budget in the 28/08 smoke test.

    `_blacklist_current` only fires when Nav2 REFUSES the goal or when the
    dispatched goal times out. A frontier whose `ComputePathToPose` REJECTS
    never went through there: `_best` stayed `None`, the message became
    "planner rejected all frontiers", and the same dead candidate was
    offered again on the next cycle -- 459 times in a row, with the map
    frozen.
    """
    node._start(None, trigger(node))
    node._state = 'selecting'
    frontier = Frontier(x=-3.06, y=0.28, cells=70, information_gain_m=0.9)
    node._candidates = [frontier]
    node._candidate_index = 0
    node._candidate_alt_index = 1
    node._best = None

    node._on_path_result(_Future(_Wrapped(GoalStatus.STATUS_ABORTED)),
                         node._epoch, frontier, (frontier.x, frontier.y))

    assert _is_suppressed(node, frontier) is True, (
        'frontier rejected by the planner keeps being offered')


def test_retiring_a_refused_frontier_does_not_retire_the_epoch(node) -> None:
    """
    The normal blacklist changes epoch; this one must NOT.

    `_blacklist_current` increments `_epoch` on purpose, to invalidate the
    callback of the goal that was in flight. Here there is no goal in
    flight: there is a validation round in progress, and changing the epoch
    in the middle of it makes `_on_path_result` of the following candidates
    return early. Validation would stop halfway and the cycle would die
    silently.
    """
    node._start(None, trigger(node))
    node._state = 'selecting'
    frontier = Frontier(x=1.0, y=1.0, cells=10, information_gain_m=0.5)
    node._candidates = [frontier]
    node._candidate_index = 1
    node._candidate_alt_index = 1
    epoch = node._epoch

    node._on_path_result(_Future(_Wrapped(GoalStatus.STATUS_ABORTED)),
                         node._epoch, frontier, (frontier.x, frontier.y))

    assert node._epoch == epoch


def test_a_frontier_the_planner_accepts_is_not_retired(node) -> None:
    """The guard must not retire the happy path along with it."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    frontier = Frontier(x=2.0, y=2.0, cells=10, information_gain_m=0.5)
    node._candidates = [frontier]
    node._candidate_index = 1

    path = SimpleNamespace(path=SimpleNamespace(poses=[]),
                           error_code=0, error_msg='')
    wrapped = _Wrapped(GoalStatus.STATUS_SUCCEEDED)
    wrapped.result = path
    node._candidate_index = 0
    node._on_path_result(_Future(wrapped), node._epoch, frontier,
                         (frontier.x, frontier.y))

    assert node._blacklist == [] and node._refused == []


def test_a_planner_refusal_is_provisional_and_lifts_when_the_map_grows(
        node) -> None:
    """
    "Unreachable now" is not "unreachable forever", and the map is the difference.

    `ExplorationGrid` runs with `allow_unknown: false`, so a distant frontier
    is rejected because the PATH to it crosses unknown space -- not because
    the frontier is bad. Measured in round 1 (29/08): the only two refusals
    were (0.07, 3.20) and (-2.93, 0.15), 2.3 m and 2.7 m from the robot, and
    retiring them for good killed the distant half of the maze. What was
    left was 3 clusters and 157 cells of real frontier with zero candidates
    allowed.

    Reducing the radius doesn't solve it: the annotated point IS the
    cluster's centroid, so any radius greater than zero kills the very
    cluster that produced it. What has to change is the permanence.
    """
    node._start(None, trigger(node))
    node._state = 'selecting'
    far = Frontier(x=-2.93, y=0.15, cells=80, information_gain_m=1.2)
    node._candidates = [far]
    node._candidate_index = 0
    node._candidate_alt_index = 1

    node._on_path_result(_Future(_Wrapped(GoalStatus.STATUS_ABORTED)),
                         node._epoch, far, (far.x, far.y))
    assert _is_suppressed(node, far) is True

    node._current = Frontier(x=0.0, y=0.5, cells=10, information_gain_m=0.5)
    node._on_nav_result(_Future(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED)), node._epoch, True)

    assert _is_suppressed(node, far) is False, (
        'reaching a goal changes the map; the previous refusal has to expire')


def test_reaching_a_goal_does_not_lift_a_hard_blacklist(node) -> None:
    """
    The hard blacklist stays: it records EXECUTION failure, not map failure.

    Nav2 returning an explicit failure for that goal says something about
    that frontier that a new map does not disprove. Mixing up the lists
    brings back the 28/08 smoke test's livelock through another path.

    Goal timeout is NO LONGER an example of this: round 3 showed that it
    marks a stuck attempt, not an invalid frontier. See
    `test_a_timed_out_frontier_is_not_hard_blacklisted`.
    """
    node._start(None, trigger(node))
    node._state = 'navigating'
    node._current = Frontier(x=3.0, y=4.0, cells=10, information_gain_m=0.5)
    node._blacklist_current('frontier ended with status 6')
    epoch = node._epoch

    node._current = Frontier(x=0.0, y=0.5, cells=10, information_gain_m=0.5)
    node._on_nav_result(_Future(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED)), epoch, True)

    assert (3.0, 4.0) in node._blacklist


def test_the_goal_timeout_leaves_room_for_more_than_one_goal(node) -> None:
    """
    The per-goal deadline and the total deadline are not independent.

    Too short, and it times out goals that were making progress, and every
    timeout sends the frontier to the HARD blacklist -- that is what
    killed round 2, with 3 timeouts swallowing the remaining 4 clusters. Too
    long, and one bad goal consumes the entire run. The useful floor is to
    fit at least three times into the total budget.
    """
    goal = float(node.get_parameter('goal_timeout_s').value)
    total = float(node.get_parameter('total_timeout_s').value)
    assert goal * 3 <= total, (
        f'{goal} s per goal does not fit three times in a {total} s budget')


def test_the_goal_timeout_is_never_raised_again(node) -> None:
    """
    Locks in a REJECTED experiment so no one repeats it.

    Round 2 timed out three distant goals at exactly 90.0 s, which read as
    "the ceiling is too short". Round 3 raised it to 180 s and made
    everything worse: 4.26 m against 21.93 m, 3746 cells against 8915, work
    ratio 8.6% against 41.7%, and no marker detection -- because it stalled
    for 180 s on a goal 0.4 m from the robot. The ceiling cuts off a stall,
    not slow traversal.

    This test was born as `== 90.0` to bar that increase. The REAL limit it
    defends is the ceiling: lowering it moves in the same direction as what
    round 3 measured. The floor is in
    `test_goal_timeout_is_sized_from_the_measured_goal_durations`, which
    uses R5's duration distribution; the two together pin down the value.

    Evidence: docs/results/ml35-f5-exploration-r3.md.
    """
    assert float(node.get_parameter('goal_timeout_s').value) <= 90.0, (
        '180 s was measured and REJECTED in round 3 -- read '
        'docs/results/ml35-f5-exploration-r3.md before trying again. What '
        "still needs fixing is the blacklist's permanence, not the ceiling.")


def _timed_out_frontier(node):
    """Create a frontier that timed out, as `_tick` would."""
    frontier = Frontier(x=-3.295, y=0.428, cells=35, information_gain_m=1.75)
    node._start(None, trigger(node))
    node._state = 'navigating'
    node._current = frontier
    node._timeout_current('frontier goal timed out')
    return frontier


def test_a_timed_out_frontier_is_not_hard_blacklisted(node) -> None:
    """
    Timing out a goal marks the ATTEMPT, not the frontier. Round 3 proved this.

    A goal 0.4 m from the robot consumed a full 180 s: the ceiling cuts off
    a stall, and a stall speaks to the pose, the costmap, and the plan of
    that instant -- none of that is permanent. In round 2, three timeouts
    turned into three permanent points that swallowed the remaining four
    clusters by 570 s, and the run died with real frontier still available.

    Evidence: docs/results/ml35-f5-exploration-r{2,3}.md.
    """
    frontier = _timed_out_frontier(node)

    assert node._blacklist == [], (
        'a goal timeout must not enter the hard blacklist')
    assert (frontier.x, frontier.y) in node._timed_out
    assert node._state == 'selecting'


def test_a_timed_out_frontier_is_suppressed_at_once(node) -> None:
    """Provisional does not mean lax: the next goal has to be a different one."""
    frontier = _timed_out_frontier(node)
    assert _is_suppressed(node, frontier) is True


def test_map_republication_does_not_release_a_timed_out_frontier(node) -> None:
    """
    What releases it is progress, not time or a message.

    `slam_toolbox` republishes `/map` every 1 s whether the map changes or
    not. If republication cleared the suppression, the stuck frontier would
    come back every second and the 28/08 livelock would be back through
    another path.
    """
    frontier = _timed_out_frontier(node)
    for _ in range(5):
        node._on_map(OccupancyGrid())
    assert _is_suppressed(node, frontier) is True


def test_reaching_another_frontier_releases_a_timed_out_frontier(node) -> None:
    """Reaching somewhere else changes pose, costmap and plan -- the three reasons."""
    frontier = _timed_out_frontier(node)

    node._current = Frontier(x=0.0, y=0.5, cells=10, information_gain_m=0.5)
    node._on_nav_result(_Future(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED)), node._epoch, True)

    assert _is_suppressed(node, frontier) is False


def test_an_explicit_nav2_failure_is_still_hard_in_this_round(node) -> None:
    """
    One policy per round. Nav2's failure result stays hard.

    Changing both permanences in the same round would make the result
    unreadable: there would be no way to tell which of the two produced the
    difference.
    """
    node._start(None, trigger(node))
    node._state = 'navigating'
    node._current = Frontier(x=3.0, y=4.0, cells=10, information_gain_m=0.5)
    node._on_nav_result(_Future(SimpleNamespace(
        status=GoalStatus.STATUS_ABORTED)), node._epoch, True)

    assert (3.0, 4.0) in node._blacklist
    assert node._timed_out == []


def test_a_new_run_clears_every_suppression_list(node) -> None:
    """Suppression is valid within a run, never across cold starts."""
    node._blacklist.append((1.0, 2.0))
    node._refused.append((3.0, 4.0))
    node._timed_out.append((5.0, 6.0))
    node._state = 'failed'

    node._start(None, trigger(node))

    assert node._blacklist == []
    assert node._refused == []
    assert node._timed_out == []


def _run_selection(node, monkeypatch, frontiers, robot=(0.0, 0.0, 0.0)):
    """
    Run `_begin_selection` over a fixed set, with no TF, grid, or Nav2.

    What is under test is the filtering, not the extraction or the dispatch,
    so the three external collaborators get out of the way.
    """
    node._start(None, trigger(node))
    node._state = 'selecting'
    node._map_seq += 1
    monkeypatch.setattr(node, '_grid', lambda: object())
    monkeypatch.setattr(node, '_robot_pose', lambda: robot)
    monkeypatch.setattr(node, '_validate_next', lambda: None)

    def fake_extract(grid, stats=None, **_kwargs):
        # Real `extract_frontiers` always reports `raw_clusters`. A
        # non-zero value here (matching the frontiers actually given) keeps
        # these near/far filtering tests isolated from the zero-raw-cluster
        # recovery path (R15) and the R17 backtrack-then-spin fallback
        # inside `_handle_no_usable_frontier`, both of which have their own
        # dedicated tests.
        if stats is not None:
            stats['raw_clusters'] = len(frontiers) or 1
        return list(frontiers)

    monkeypatch.setattr(
        maze_explorer_module, 'extract_frontiers', fake_extract)
    node._begin_selection()


def test_a_frontier_inside_the_goal_tolerance_is_never_dispatched(
        node, monkeypatch) -> None:
    """
    Round 4's failure mode, attacked at the cause.

    Nav2's `xy_goal_tolerance` is 0.25 m. A frontier 0.20 m from the robot
    makes Nav2 return success without anything moving; selection goes back
    to the same point and the cycle repeats. In round 4 this ran 565 times
    in 580 s with the robot inside an 11 mm x 25 mm box.

    Evidence: docs/results/ml35-f5-exploration-r4.md.
    """
    near = Frontier(x=0.20, y=0.0, cells=8, information_gain_m=0.4)
    far = Frontier(x=2.0, y=0.0, cells=90, information_gain_m=4.5)
    _run_selection(node, monkeypatch, [near, far])

    assert near not in node._candidates
    assert node._near_skipped == 1


def test_the_next_frontier_out_is_selected_instead(node, monkeypatch) -> None:
    """Discarding the near one must let exploration keep going, not stop."""
    near = Frontier(x=0.20, y=0.0, cells=8, information_gain_m=0.4)
    far = Frontier(x=2.0, y=0.0, cells=90, information_gain_m=4.5)
    _run_selection(node, monkeypatch, [near, far])

    assert node._candidates == [far]
    assert node._frontier_count == 1


def test_a_frontier_exactly_at_the_limit_stays_eligible(
        node, monkeypatch) -> None:
    """The limit is inclusive; otherwise the cutoff becomes an ambiguous dead band."""
    limit = float(node.get_parameter('min_frontier_distance_m').value)
    edge = Frontier(x=limit, y=0.0, cells=20, information_gain_m=1.0)
    _run_selection(node, monkeypatch, [edge])

    assert node._candidates == [edge]
    assert node._near_skipped == 0


def test_skipping_a_near_frontier_never_suppresses_it(
        node, monkeypatch) -> None:
    """
    The cutoff is relative to the CURRENT pose, never a permanent annotation.

    An overly permanent list was all it took to kill round 1. Walking a few
    centimeters has to return the frontier to contention on its own.
    """
    near = Frontier(x=0.20, y=0.0, cells=8, information_gain_m=0.4)
    _run_selection(node, monkeypatch, [near])

    assert node._blacklist == []
    assert node._refused == []
    assert node._timed_out == []

    _run_selection(node, monkeypatch, [near], robot=(-1.0, 0.0, 0.0))
    assert node._candidates == [near]


def test_a_selection_with_only_near_frontiers_counts_as_no_progress(
        node, monkeypatch) -> None:
    """
    Ending up with no candidates due to proximity counts as no progress.

    If the cycle didn't count, a robot surrounded only by frontiers within
    tolerance would sit silently in `selecting` until the total deadline.
    """
    near = Frontier(x=0.20, y=0.0, cells=8, information_gain_m=0.4)
    _run_selection(node, monkeypatch, [near])

    assert node._candidates == []
    assert node._barren_cycles == 1


def test_status_reports_how_many_near_frontiers_were_skipped(
        node, monkeypatch) -> None:
    """Without the metric in the status, the discard is invisible in run analysis."""
    near = Frontier(x=0.20, y=0.0, cells=8, information_gain_m=0.4)
    far = Frontier(x=2.0, y=0.0, cells=90, information_gain_m=4.5)
    _run_selection(node, monkeypatch, [near, far])
    node._publish_status()

    assert node.published[-1]['near_frontiers_skipped'] == 1


def _pose_at(x: float, y: float) -> PoseStamped:
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    return pose


def test_setback_point_walks_back_from_the_path_end() -> None:
    """A straight 1 m path recessed by 0.4 m lands 0.6 m from the start."""
    path = [_pose_at(0.0, 0.0), _pose_at(1.0, 0.0)]
    point = maze_explorer_module._setback_point(path, setback_m=0.4)
    assert point == pytest.approx((0.6, 0.0))


def test_setback_point_discarded_when_it_would_fall_inside_goal_tolerance(
) -> None:
    """
    R11 (29/08): the robot never moved, for minutes, on an unchanging map.

    A short path (candidate close to the robot) recessed by the default
    0.40 m setback landed ~0.23 m from the robot's own current pose --
    inside Nav2's 0.25 m `xy_goal_tolerance`. `SimpleGoalChecker` called the
    goal reached without the robot moving at all, so the map never grew and
    the identical frontier kept getting re-selected forever. The fix is a
    `min_travel_m` floor: a setback point this close to the path's start is
    discarded (`None`) instead of returned, so the caller falls back to the
    original, already-validated endpoint.
    """
    path = [_pose_at(0.0, 0.0), _pose_at(0.6, 0.0)]
    point = maze_explorer_module._setback_point(
        path, setback_m=0.4, min_travel_m=0.35)
    assert point is None


def test_setback_point_kept_when_it_clears_the_travel_floor() -> None:
    """The same geometry with a floor it actually clears is kept, not dropped."""
    path = [_pose_at(0.0, 0.0), _pose_at(0.6, 0.0)]
    point = maze_explorer_module._setback_point(
        path, setback_m=0.4, min_travel_m=0.15)
    assert point == pytest.approx((0.2, 0.0))


def test_setback_point_on_a_path_shorter_than_the_setback_is_discarded(
) -> None:
    """
    A too-short path is discarded, not collapsed to the robot's own pose.

    It used to fall back to the path's own start point -- the robot's
    current pose, an even more degenerate target than the R11 stall.
    `min_travel_m` defaults to 0.0, but the start point is by definition
    zero distance from itself, so it is always discarded.
    """
    path = [_pose_at(0.0, 0.0), _pose_at(0.1, 0.0)]
    assert maze_explorer_module._setback_point(path, setback_m=0.4) is None


def test_setback_point_handles_empty_and_single_pose_paths() -> None:
    assert maze_explorer_module._setback_point([], setback_m=0.4) is None
    single = [_pose_at(2.0, 3.0)]
    assert maze_explorer_module._setback_point(
        single, setback_m=0.4) == pytest.approx((2.0, 3.0))
    assert maze_explorer_module._setback_point(
        single, setback_m=0.4, min_travel_m=5.0) is None


def test_r4a_leaves_the_r4_timeout_policy_alone(node) -> None:
    """One variable per round: timeout persistence is not changed here."""
    frontier = _timed_out_frontier(node)
    assert node._blacklist == []
    assert (frontier.x, frontier.y) in node._timed_out


def _run_provisionally_suppressed_selection(
        node, monkeypatch, frontiers, *, hard=False) -> None:
    """Run one selection where every real frontier starts suppressed."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    node._map_seq += 1
    monkeypatch.setattr(node, '_grid', lambda: object())
    monkeypatch.setattr(node, '_robot_pose', lambda: (0.0, 0.0, 0.0))
    monkeypatch.setattr(node, '_validate_next', lambda: None)

    def fake_extract(grid, stats=None, **_kwargs):
        # Real `extract_frontiers` always reports `raw_clusters` -- a
        # provisional/hard dead end here means a cluster WAS observed and
        # then suppressed, not that none existed (see
        # `test_a_zero_raw_cluster_selection_starts_an_observation_recovery`
        # for that other case).
        if stats is not None:
            stats['raw_clusters'] = len(frontiers)
        return list(frontiers)
    monkeypatch.setattr(
        maze_explorer_module, 'extract_frontiers', fake_extract)
    targets = node._blacklist if hard else node._refused
    targets.extend((item.x, item.y) for item in frontiers)
    node._begin_selection()


def test_all_provisional_suppressions_are_released_once(
        node, monkeypatch) -> None:
    """A provisional-only dead end gets one chance to explore again."""
    frontiers = [
        Frontier(x=1.0, y=0.0, cells=20, information_gain_m=1.0),
        Frontier(x=2.0, y=0.0, cells=30, information_gain_m=1.5),
    ]
    _run_provisionally_suppressed_selection(node, monkeypatch, frontiers)

    assert node._refused == []
    assert node._timed_out == []
    assert node._blacklist == []
    assert node._candidates == frontiers
    assert node._provisional_recoveries == 1
    assert node._provisional_recovery_used is True
    assert node._barren_cycles == 0


def test_provisional_recovery_cannot_repeat_without_progress(
        node, monkeypatch) -> None:
    """Repeated refusal after recovery counts barren instead of livelocking."""
    frontier = Frontier(x=1.0, y=0.0, cells=20, information_gain_m=1.0)
    _run_provisionally_suppressed_selection(node, monkeypatch, [frontier])

    node._refused.append((frontier.x, frontier.y))
    node._map_seq += 1
    node._begin_selection()

    assert node._refused == [(frontier.x, frontier.y)]
    assert node._candidates == []
    assert node._provisional_recoveries == 1
    assert node._barren_cycles == 1


def test_hard_blacklist_is_never_released_by_deadlock_recovery(
        node, monkeypatch) -> None:
    """Recovery must not resurrect a frontier with an execution failure."""
    frontier = Frontier(x=1.0, y=0.0, cells=20, information_gain_m=1.0)
    _run_provisionally_suppressed_selection(
        node, monkeypatch, [frontier], hard=True)

    assert node._blacklist == [(frontier.x, frontier.y)]
    assert node._candidates == []
    assert node._provisional_recoveries == 0
    assert node._barren_cycles == 1


def test_successful_motion_rearms_provisional_recovery(node) -> None:
    """Only a reached exploration goal permits another recovery attempt."""
    node._provisional_recovery_used = True
    node._state = 'navigating'
    node._current = Frontier(x=1.0, y=0.0, cells=20, information_gain_m=1.0)

    node._on_nav_result(_Future(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED)), node._epoch, True)

    assert node._provisional_recovery_used is False


def test_a_barren_selection_fails_the_run_instead_of_idling(node) -> None:
    """
    Retire frontiers without a terminal condition and one livelock replaces another.

    With the correction above, the blacklist can eventually consume every
    frontier. The old code wrote "no reachable safe frontier" and stayed in
    `selecting` forever — silently, indistinguishable from working. A stopped
    robot produces no new map, so the situation never resolves on its own: this
    is a failure and must be reported.
    """
    node._start(None, trigger(node))
    node._state = 'selecting'
    limit = int(node.get_parameter('barren_selections_limit').value)

    for _ in range(limit):
        assert node._state == 'selecting'
        node._note_barren_selection()

    assert node._state == 'failed'
    assert 'frontier' in node._message


def test_dispatching_a_goal_clears_the_barren_streak(node) -> None:
    """The count tracks CONSECUTIVE cycles; dispatching a goal resets it."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    node._note_barren_selection()
    node._note_barren_selection()
    assert node._barren_cycles == 2
    node._barren_cycles = 0  # what `_send_navigation` does when dispatching
    node._note_barren_selection()
    assert node._barren_cycles == 1
    assert node._state == 'selecting'


def test_a_zero_raw_cluster_selection_starts_an_observation_recovery(
        node, monkeypatch) -> None:
    """
    No raw clusters is different from filtered clusters.

    Spinning can reveal new geometry when SLAM has not seen any frontier yet;
    it does not help when frontiers exist but were suppressed (see
    `test_all_provisional_suppressions_are_released_once`, where `raw_clusters`
    is nonzero and the code takes a different path).
    """
    node._start(None, trigger(node))
    node._state = 'selecting'
    node._map_seq += 1
    monkeypatch.setattr(node, '_grid', lambda: object())
    monkeypatch.setattr(node, '_robot_pose', lambda: (0.0, 0.0, 0.0))

    def fake_extract(grid, stats=None, **_kwargs):
        if stats is not None:
            stats['raw_clusters'] = 0
        return []
    monkeypatch.setattr(
        maze_explorer_module, 'extract_frontiers', fake_extract)

    spins: list[int] = []
    monkeypatch.setattr(
        node, '_start_observation_recovery',
        lambda: spins.append(node._map_seq))

    node._begin_selection()

    assert spins == [node._map_seq]
    assert node._recovery_map_seq == node._map_seq
    assert node._message == 'no raw frontier cluster'
    assert node._barren_cycles == 0, (
        'the scan has not happened yet; this cannot count as a barren cycle')


def test_a_repeated_zero_raw_cluster_map_does_not_spin_twice(
        node, monkeypatch) -> None:
    """One attempt per map version — see the comment in `__init__`."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    node._map_seq += 1
    monkeypatch.setattr(node, '_grid', lambda: object())
    monkeypatch.setattr(node, '_robot_pose', lambda: (0.0, 0.0, 0.0))

    def fake_extract(grid, stats=None, **_kwargs):
        if stats is not None:
            stats['raw_clusters'] = 0
        return []
    monkeypatch.setattr(
        maze_explorer_module, 'extract_frontiers', fake_extract)

    spins: list[int] = []
    monkeypatch.setattr(
        node, '_start_observation_recovery',
        lambda: spins.append(node._map_seq))

    node._begin_selection()
    # Force a new extraction without a new map, using the same technique as
    # the other cache invalidations in this file.
    node._epoch += 1
    node._begin_selection()

    assert spins == [node._map_seq], (
        'the second attempt saw the SAME map; it must not repeat')


class _PendingSend:
    """Mimics the real `send_goal_async` result: a Future still in flight."""

    def __init__(self) -> None:
        self.callback = None

    def add_done_callback(self, cb) -> None:
        self.callback = cb


class _AutoFireFuture:
    """An already-resolved Future: fires the callback as soon as it is added."""

    def __init__(self, value) -> None:
        self._value = value

    def result(self):
        return self._value

    def add_done_callback(self, cb) -> None:
        cb(self)


class _SpinHandle:
    """The handle accepted by `Spin.send_goal_async` passes to the callback."""

    def __init__(self, accepted: bool,
                 status: int = GoalStatus.STATUS_SUCCEEDED) -> None:
        self.accepted = accepted
        self._status = status

    def get_result_async(self):
        return _AutoFireFuture(SimpleNamespace(status=self._status))

    def cancel_goal_async(self) -> None:
        pass


def test_observation_recovery_round_trip_clears_pending_and_counts_barren(
        node, monkeypatch) -> None:
    """The scan itself decides nothing; only the next new map does."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    pending = _PendingSend()
    monkeypatch.setattr(
        node._spin_client, 'send_goal_async', lambda goal: pending)

    node._start_observation_recovery()
    assert node._recovery_pending is True
    assert node._recovery_attempts == 1

    pending.callback(_Future(_SpinHandle(accepted=True)))

    assert node._recovery_pending is False
    assert node._recovery_handle is None
    assert node._barren_cycles == 1


def test_a_refused_observation_recovery_counts_barren_at_once(
        node, monkeypatch) -> None:
    """O behavior_server pode recusar o giro (ex.: colisao iminente)."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    pending = _PendingSend()
    monkeypatch.setattr(
        node._spin_client, 'send_goal_async', lambda goal: pending)

    node._start_observation_recovery()
    pending.callback(_Future(_SpinHandle(accepted=False)))

    assert node._recovery_pending is False
    assert node._barren_cycles == 1


def test_cancel_stops_a_pending_observation_recovery(
        node, monkeypatch) -> None:
    """Cancelling exploration must stop an in-flight spin, not just navigation."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    pending = _PendingSend()
    monkeypatch.setattr(
        node._spin_client, 'send_goal_async', lambda goal: pending)
    node._start_observation_recovery()
    handle = _SpinHandle(accepted=True)
    cancelled: list[bool] = []
    handle.cancel_goal_async = lambda: cancelled.append(True)
    node._recovery_handle = handle
    node._recovery_pending = True

    node._cancel(None, trigger(node))

    assert cancelled == [True]
    assert node._recovery_pending is False
    assert node._recovery_handle is None


def test_a_stale_recovery_callback_is_ignored_after_cancel(
        node, monkeypatch) -> None:
    """A callback from a cancelled round must not count toward the new one."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    pending = _PendingSend()
    monkeypatch.setattr(
        node._spin_client, 'send_goal_async', lambda goal: pending)
    node._start_observation_recovery()
    node._cancel(None, trigger(node))
    barren_before = node._barren_cycles

    pending.callback(_Future(_SpinHandle(accepted=True)))

    assert node._barren_cycles == barren_before


class _StubGoalHandle:
    """Minimal goal handle — just enough for `_cancel_goal` to work."""

    def cancel_goal_async(self):
        return None


def _armed_for_navigating(node, clock) -> None:
    """Put `node` in `navigating` with a goal already accepted by Nav2."""
    node._start(None, trigger(node))
    node._state = 'navigating'
    node._current = Frontier(x=5.0, y=5.0, cells=10, information_gain_m=1.0)
    node._goal_started_s = clock['t']
    node._goal_handle = _StubGoalHandle()  # goal accepted; see _on_nav_accepted


def test_navigation_watchdog_fires_after_the_stall_window(node) -> None:
    """Goal dispatched, robot stopped — abort before the 45 s deadline."""
    clock = {'t': 0.0}
    node._now_s = lambda: clock['t']
    _armed_for_navigating(node, clock)
    node._robot_pose = lambda: (0.0, 0.0, 0.0)
    window = float(node.get_parameter('stall_window_s').value)

    node._tick()  # arm the watchdog clock on the first reading
    assert node._state == 'navigating'

    clock['t'] = window - 1.0
    node._tick()
    assert node._state == 'navigating', 'still within the window'

    clock['t'] = window + 1.0
    node._tick()
    assert node._state == 'selecting'
    assert 'movement watchdog' in node._message
    assert (5.0, 5.0) in node._timed_out


def test_navigation_watchdog_resets_on_real_displacement(node) -> None:
    """Real displacement resets the window; it is not fixed from goal dispatch."""
    clock = {'t': 0.0}
    node._now_s = lambda: clock['t']
    _armed_for_navigating(node, clock)
    window = float(node.get_parameter('stall_window_s').value)
    pose = {'p': (0.0, 0.0, 0.0)}
    node._robot_pose = lambda: pose['p']

    node._tick()
    clock['t'] = window - 1.0
    pose['p'] = (0.2, 0.0, 0.0)  # above stall_move_threshold_m (0.05 m)
    node._tick()
    assert node._state == 'navigating'

    clock['t'] = (window - 1.0) + (window - 1.0)
    node._tick()
    assert node._state == 'navigating', (
        'the clock reset on displacement; the window must not have elapsed yet')


def test_navigation_watchdog_does_not_fire_on_legitimate_rotation(node) -> None:
    """
    R15a: turning in place to face a corridor is not a stall.

    Checking only xy would classify this legitimate rotation (with no
    translation) as a stopped robot, cancelling a goal that was making progress.
    """
    clock = {'t': 0.0}
    node._now_s = lambda: clock['t']
    _armed_for_navigating(node, clock)
    window = float(node.get_parameter('stall_window_s').value)
    pose = {'p': (0.0, 0.0, 0.0)}
    node._robot_pose = lambda: pose['p']

    node._tick()
    clock['t'] = window - 1.0
    pose['p'] = (0.0, 0.0, 0.3)  # rotates 0.3 rad, xy unchanged
    node._tick()
    assert node._state == 'navigating'

    clock['t'] = (window - 1.0) + (window - 1.0)
    node._tick()
    assert node._state == 'navigating', (
        'real rotation above the threshold must reset the clock too')


def test_navigation_watchdog_handles_the_minus_pi_pi_wraparound(node) -> None:
    """Yaw crossing from +pi to -pi is a small rotation, not a large one."""
    clock = {'t': 0.0}
    node._now_s = lambda: clock['t']
    _armed_for_navigating(node, clock)
    window = float(node.get_parameter('stall_window_s').value)
    pose = {'p': (0.0, 0.0, math.pi - 0.01)}
    node._robot_pose = lambda: pose['p']

    node._tick()  # arm the baseline at (pi - 0.01)
    clock['t'] = window - 1.0
    pose['p'] = (0.0, 0.0, -math.pi + 0.01)  # crossed wraparound; actual diff = 0.02
    node._tick()
    clock['t'] = window + 1.0
    node._tick()
    assert node._state == 'selecting', (
        'actual yaw difference (0.02 rad) is below threshold; without normalized '
        'wraparound the watchdog would calculate ~2*pi and never fire')
    assert 'movement watchdog' in node._message


def test_navigation_watchdog_does_not_fire_before_goal_acceptance(node) -> None:
    """
    R15a: without an accepted goal (`_goal_handle is None`), there is nothing to stall.

    Before this correction, the clock armed when the goal was dispatched
    (`_send_navigation`), counting Nav2 response latency — while still in flight —
    as immobility.
    """
    clock = {'t': 0.0}
    node._now_s = lambda: clock['t']
    node._start(None, trigger(node))
    node._state = 'navigating'
    node._current = Frontier(x=5.0, y=5.0, cells=10, information_gain_m=1.0)
    node._goal_started_s = clock['t']
    node._goal_handle = None  # Nav2 has not accepted yet
    node._robot_pose = lambda: (0.0, 0.0, 0.0)
    window = float(node.get_parameter('stall_window_s').value)

    node._tick()
    clock['t'] = window + 1.0
    node._tick()
    assert node._state == 'navigating', (
        'without an accepted goal, the watchdog must not fire')
    assert 'movement watchdog' not in node._message


def test_navigation_watchdog_fires_when_accepted_goal_is_truly_still(node) -> None:
    """Accepted goal (`_goal_handle` set) and truly stopped robot — fire."""
    clock = {'t': 0.0}
    node._now_s = lambda: clock['t']
    _armed_for_navigating(node, clock)
    node._robot_pose = lambda: (1.0, 2.0, 0.5)  # fixed pose, no xy or yaw change
    window = float(node.get_parameter('stall_window_s').value)

    node._tick()
    clock['t'] = window + 1.0
    node._tick()
    assert node._state == 'selecting'
    assert 'movement watchdog' in node._message


def test_homing_returns_to_exploration_after_three_failures(node) -> None:
    """Persisting with an unreachable marker consumes the run's total deadline."""
    node._state = 'homing_exit'
    for _ in range(2):
        node._homing_failed('approach ended with status 6')
        assert node._state == 'homing_exit'
    node._homing_failed('approach ended with status 6')
    assert node._state == 'selecting'
    assert node._homing_failures == 0


def test_marker_goes_stale_and_stops_counting_as_visible(node) -> None:
    """
    A stale pose is indistinguishable from a current one unless its age is checked.

    The detector publishes only after confirmation; stopping publication means
    it has lost sight of the marker. Without a timeout, the explorer would stay
    in `homing_exit`, chasing the last seen pose forever.
    """
    stale_s = float(node.get_parameter('marker_stale_s').value)
    node._exit_candidate_pose_map = (5.0, 5.0)
    node._exit_seen_s = node._now_s()
    node._publish_status()
    assert node.published[-1]['marker_visible'] is True

    node._exit_seen_s = node._now_s() - stale_s - 1.0
    node._publish_status()
    assert node.published[-1]['marker_visible'] is False


def test_total_timeout_fails_the_run_instead_of_running_forever(node) -> None:
    """The total deadline makes HIL acceptance a measurement, not a wait."""
    node._start(None, trigger(node))
    node._started_s = node._now_s() - float(
        node.get_parameter('total_timeout_s').value) - 1.0
    node._tick()
    assert node._state == 'failed'
    assert node.published[-1]['state'] == 'failed'


def test_tick_is_inert_once_the_run_is_over(node) -> None:
    """A terminal state must not start dispatching goals again on its own."""
    for state in ('idle', 'completed', 'failed', 'cancelled'):
        node._state = state
        node._tick()
        assert node._state == state


def test_selection_waits_for_the_map_instead_of_planning_blind(node) -> None:
    """Without a map or TF, selecting a frontier means selecting nothing."""
    node._state = 'selecting'
    node._map = None
    node._begin_selection()
    assert node._state == 'waiting_map'


def test_grid_reads_resolution_and_origin_from_the_live_map(node) -> None:
    """Replacing the origin with zero puts every frontier in the wrong place."""
    grid_message = OccupancyGrid()
    grid_message.info.width = 4
    grid_message.info.height = 3
    grid_message.info.resolution = 0.05
    grid_message.info.origin.position.x = -1.25
    grid_message.info.origin.position.y = 2.5
    grid_message.info.origin.orientation.w = 1.0
    grid_message.data = [0] * 12
    node._map = grid_message

    grid = node._grid()
    assert (grid.width, grid.height) == (4, 3)
    assert grid.resolution == pytest.approx(0.05)
    assert (grid.origin_x, grid.origin_y) == (-1.25, 2.5)
    assert grid.origin_yaw == pytest.approx(0.0)


def test_exit_pose_without_tf_is_dropped_rather_than_used_raw(node) -> None:
    """
    The detector pose is in the camera frame and is unusable in `map`.

    Using it without transforming would send the robot to a goal a few metres
    from the camera ITSELF, in the wrong frame — a plausible-looking but wrong
    target, the worst kind of failure here.
    """
    pose = PoseStamped()
    pose.header.frame_id = 'front_camera'
    pose.pose.position.x = 3.0
    node._on_exit_pose(pose)
    assert node._exit_candidate_pose_map is None
    assert node._exit_pose_map is None


# --- frontier selection cost -----------------------------------------------
#
# Measured on this x86 host with a maze11-sized SLAM map (234 x 284 cells), 95%
# explored: `extract_frontiers` took 158.6 ms and was called on every 1 Hz tick
# while in `selecting` with no pending goal — exactly the "planner rejected all
# frontiers" case. On the AM69 this would pin a core without producing anything.
# The Stage 4 gate requires p95 below 100 ms.

def _map_message(width: int = 4, height: int = 3) -> OccupancyGrid:
    message = OccupancyGrid()
    message.info.width = width
    message.info.height = height
    message.info.resolution = 0.05
    message.info.origin.orientation.w = 1.0
    message.data = [0] * (width * height)
    return message


@pytest.fixture
def selecting(node, monkeypatch):
    """Prepare a node for selection and count extraction instead of timing it."""
    calls: list[int] = []

    def counted(grid, stats=None, **kwargs):
        calls.append(1)
        # Real `extract_frontiers` always reports `raw_clusters`. A nonzero
        # value here keeps these CACHE tests isolated from the scan-recovery
        # path (R15), which has dedicated tests for raw_clusters == 0.
        if stats is not None:
            stats['raw_clusters'] = 1
        return []

    monkeypatch.setattr(
        'demo_navigation.maze_explorer.extract_frontiers', counted)
    node._robot_pose = lambda: (0.0, 0.0, 0.0)
    node._on_map(_map_message())
    node._state = 'selecting'
    node.extract_calls = calls
    return node


def test_selection_is_not_recomputed_while_map_and_blacklist_stand(selecting):
    """Without a new map, extraction would return the same result and cost a core."""
    selecting._begin_selection()
    for _ in range(5):
        selecting._begin_selection()

    assert len(selecting.extract_calls) == 1
    assert selecting._selection_cycle == 1


def test_a_new_map_invalidates_the_selection_cache(selecting):
    """
    A new map is new information: then it is worth extracting again.

    The content must actually change (R15) — repeating the same grid does not
    count as a new map; see `test_map_republication_does_not_invalidate_the_cache`
    below.
    """
    changed = _map_message()
    changed.data[0] = 100
    selecting._begin_selection()
    selecting._on_map(changed)
    selecting._begin_selection()

    assert len(selecting.extract_calls) == 2


def test_map_republication_does_not_invalidate_the_cache(selecting):
    """
    `slam_toolbox` republishes `/map` even without a change — that is not a new map.

    Complements the test above: here the CONTENT is identical to what the
    `selecting` fixture already used to populate the cache, so extracting again
    would waste a core.
    """
    selecting._begin_selection()
    selecting._on_map(_map_message())
    selecting._begin_selection()

    assert len(selecting.extract_calls) == 1


def test_on_map_only_advances_map_seq_on_real_content_change(node) -> None:
    """
    R15: `_map_seq` counts CONTENT, not messages.

    Before this correction, republishing an identical map still incremented
    `_map_seq`, causing `self._map_seq > self._last_provisional_map_seq` in
    `_begin_selection` to release a provisional suppression without any new
    observation arriving.
    """
    node._on_map(_map_message())
    seq_after_first = node._map_seq

    for _ in range(5):
        node._on_map(_map_message())
    assert node._map_seq == seq_after_first, (
        'an identical republication must not advance _map_seq')

    changed = _map_message()
    changed.data[0] = 100
    node._on_map(changed)
    assert node._map_seq == seq_after_first + 1, (
        'genuinely new content must advance _map_seq'
    )


def test_a_geometry_only_change_advances_map_seq(node) -> None:
    """
    R15a: SLAM re-anchoring changes geometry, even with identical cells.

    R15 hashed only `message.data` — a map with the same cell grid but a
    different origin or resolution (for example, after re-anchoring) would be
    treated as an identical republication, losing the real change.
    """
    node._on_map(_map_message())
    seq_after_first = node._map_seq

    moved_origin = _map_message()
    moved_origin.info.origin.position.x = 1.0
    node._on_map(moved_origin)
    assert node._map_seq == seq_after_first + 1, (
        'different origin, same cells, still a new map'
    )

    seq_after_origin = node._map_seq
    different_resolution = _map_message()
    different_resolution.info.origin.position.x = 1.0
    different_resolution.info.resolution = 0.10
    node._on_map(different_resolution)
    assert node._map_seq == seq_after_origin + 1, (
        'different resolution, same cells, still a new map'
    )


def test_map_republication_does_not_release_a_provisional_recovery(
        node, monkeypatch) -> None:
    """
    The test explicitly requested by name in the original plan.

    Same content, new message, no recovery.

    Without the `_map_seq` correction, republishing `/map` five times (same
    content) was enough to make `self._map_seq > self._last_provisional_map_seq`
    true and release the provisional suppression — exactly the livelock
    `_last_provisional_map_seq` was meant to prevent, except counted by message
    instead of content.
    """
    frontier = Frontier(x=1.0, y=0.0, cells=20, information_gain_m=1.0)
    _run_provisionally_suppressed_selection(node, monkeypatch, [frontier])
    assert node._provisional_recovery_used is True

    # A real map observation, to move the content tracker out of its "never
    # seen /map" state — without this, the test's FIRST `_on_map` call would
    # always count as a change, masking what we want to measure.
    node._on_map(_map_message())

    # Re-arm recovery (as a real arrival would; see
    # `test_successful_motion_rearms_provisional_recovery`) and simulate a NEW
    # refusal on the same map version — the real precondition
    # `_last_provisional_map_seq` exists to protect.
    node._provisional_recovery_used = False
    node._refused.append((frontier.x, frontier.y))
    node._last_provisional_map_seq = node._map_seq

    for _ in range(5):
        node._on_map(_map_message())  # same content, new messages
    node._begin_selection()

    assert node._refused == [(frontier.x, frontier.y)], (
        'identical republication must not release provisional suppression')
    assert node._candidates == []


def test_a_new_blacklist_entry_invalidates_the_selection_cache(selecting):
    """A rejected frontier changes the result even when the map is unchanged."""
    selecting._begin_selection()
    selecting._blacklist.append((1.0, 1.0))
    selecting._begin_selection()

    assert len(selecting.extract_calls) == 2


def test_restarting_the_run_invalidates_the_selection_cache(selecting):
    """
    Starting or cancelling exploration must force extraction.

    The epoch is part of the key for this reason: without it, a `start` right
    after a `cancel`, with the same map and a cleared blacklist, would inherit
    the previous run's cache and the explorer would sit idle waiting for a new map.
    """
    selecting._begin_selection()
    selecting._epoch += 1
    selecting._begin_selection()

    assert len(selecting.extract_calls) == 2


def test_a_map_update_while_navigating_does_not_replace_the_goal(selecting):
    """Reacting to /map in flight would make the robot abandon each frontier on every map."""
    goal = Frontier(x=2.0, y=3.0, cells=12, information_gain_m=1.0)
    selecting._current = goal
    selecting._state = 'navigating'
    selecting._started_s = selecting._now_s()
    selecting._goal_started_s = selecting._now_s()

    selecting._on_map(_map_message())
    selecting._tick()

    assert selecting._current == goal
    assert selecting._state == 'navigating'
    assert selecting.extract_calls == []


def test_status_carries_the_cost_of_the_search(selecting):
    """These fields are needed to prove the Stage 4 CPU gate."""
    selecting._begin_selection()
    selecting._publish_status()
    payload = selecting.published[-1]

    for field in ('frontier_extract_ms', 'frontier_cells', 'frontier_clusters',
                  'candidates_checked', 'path_requests', 'selection_cycle'):
        assert field in payload, field
    assert payload['selection_cycle'] == 1


def test_extraction_is_timed_on_a_monotonic_clock(selecting, monkeypatch):
    """
    Extraction time must NOT come from `/clock`.

    With `use_sim_time`, the simulation clock pauses, jumps, and runs out of
    sync with real time — all three have been observed in this project. We want
    actual CPU time spent here, which exists only on the monotonic clock.
    """
    ticks = iter([100.0, 100.25])
    monkeypatch.setattr(
        'demo_navigation.maze_explorer.time.monotonic', lambda: next(ticks))

    selecting._begin_selection()

    assert selecting._frontier_extract_ms == pytest.approx(250.0)


def test_homing_entry_distance_is_latched_for_the_gate_measurement(
        node, monkeypatch) -> None:
    """
    The homing distance gate needs a measured value, not a guess.

    Status publishes at 2 Hz and entry into `homing_exit` is instantaneous, so
    periodic sampling misses the moment. Entry distance is latched at the
    transition; the current distance continues to be sampled.
    """
    node._start(None, trigger(node))
    node._state = 'selecting'
    monkeypatch.setattr(node, '_robot_pose', lambda: (0.0, 0.0, 0.0))
    monkeypatch.setattr(node, '_cancel_goal', lambda: None)
    for _ in range(int(node.get_parameter('homing_confirm_observations').value)):
        _marker_at(node, monkeypatch, 3.0)

    assert node._state == 'homing_exit'
    assert node._homing_entry_distance_m == 3.0
    assert node._homing_entries == 1
    assert node.published[-1]['homing_entry_distance_m'] == 3.0
    assert node.published[-1]['marker_distance_m'] == 3.0


def test_homing_entry_distance_is_not_overwritten_while_homing(
        node, monkeypatch) -> None:
    """A later partial view cannot move the target accepted by the gate."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    monkeypatch.setattr(node, '_robot_pose', lambda: (0.0, 0.0, 0.0))
    monkeypatch.setattr(node, '_cancel_goal', lambda: None)
    for _ in range(int(node.get_parameter('homing_confirm_observations').value)):
        _marker_at(node, monkeypatch, 3.0)

    accepted = node._exit_pose_map
    # A new observation swings far while homing. It remains observable as the
    # raw candidate, but cannot redirect the active approach.
    monkeypatch.setattr(node, '_robot_pose', lambda: (2.4, 0.8, 0.0))
    pose = PoseStamped()
    pose.header.frame_id = 'front_camera'
    pose.header.stamp.nanosec = 99
    pose.pose.position.x = 7.0
    node._on_exit_pose(pose)
    node._tick()

    assert node._homing_entries == 1
    assert node._homing_entry_distance_m == 3.0
    assert node._exit_pose_map == accepted
    assert node._exit_candidate_pose_map == (7.0, 0.0)
    assert node.published[-1]['marker_distance_m'] == pytest.approx(4.67)


def test_marker_distance_is_none_without_a_marker_or_a_pose(
        node, monkeypatch) -> None:
    """Without a marker or TF, the measurement is absent, never zero."""
    monkeypatch.setattr(node, '_robot_pose', lambda: (0.0, 0.0, 0.0))
    assert node._distance_to_exit() is None

    node._exit_pose_map = (1.0, 1.0)
    monkeypatch.setattr(node, '_robot_pose', lambda: None)
    assert node._distance_to_exit() is None


def test_a_far_marker_is_recorded_but_does_not_capture_the_run() -> None:
    """
    Replaces `test_the_measurement_round_adds_no_homing_gate`; the measurement exists.

    R7 measured range error by band against the SDF marker at
    (-4.90, -2.60), using 131 samples:

        estimated band   estimate/actual ratio   mean absolute error
        0-2 m                 0.579              1.28 m
        2-3 m                 0.876              0.60 m
        3-4 m                 1.062              0.41 m
        4-6 m                 1.316              1.14 m
        above 6 m             1.813              3.08 m

    R7 entered homing at 7.35 m — the worst band — and because the approach now
    persists, that single observation trapped the run in `homing_exit` for 520 s
    without reaching the marker. The gate is a MAXIMUM: beyond it, record the
    marker and continue exploring.
    """


def _marker_at(node, monkeypatch, distance_m, stamp=None):
    """Deliver one distinct, identity-transformed camera observation."""
    monkeypatch.setattr(node, '_robot_pose', lambda: (0.0, 0.0, 0.0))
    monkeypatch.setattr(node, '_cancel_goal', lambda: None)
    transform = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=0.0, y=0.0),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
    monkeypatch.setattr(
        node._tf_buffer, 'lookup_transform', lambda *args: transform)
    if stamp is None:
        stamp = getattr(node, '_test_marker_stamp', 0) + 1
        node._test_marker_stamp = stamp
    pose = PoseStamped()
    pose.header.frame_id = 'front_camera'
    pose.header.stamp.nanosec = stamp
    pose.pose.position.x = distance_m
    node._on_exit_pose(pose)
    return pose


def test_a_marker_beyond_the_gate_never_enters_homing(node, monkeypatch) -> None:
    """7.35 m trapped R7 for 520 s. Record the marker, but do not commit to homing."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    far = float(node.get_parameter('homing_max_distance_m').value) + 1.0
    _marker_at(node, monkeypatch, far)

    for _ in range(10):
        node._tick()

    assert node._state != 'homing_exit'
    assert node._homing_entries == 0
    assert node.published[-1]['marker_distance_m'] == round(far, 2)
    assert node.published[-1]['marker_far_ignored'] == 1


def test_entering_homing_needs_more_than_one_near_observation(
        node, monkeypatch) -> None:
    """
    A single sample is not decisive: in R7 the estimate oscillated from 1.27 to 7.94 m.

    Hysteresis requires `homing_confirm_observations` consecutive near
    observations before cancelling exploration.
    """
    node._start(None, trigger(node))
    node._state = 'selecting'
    needed = int(node.get_parameter('homing_confirm_observations').value)
    assert needed >= 2, 'without hysteresis the gate does not filter measured oscillation'
    _marker_at(node, monkeypatch, 3.0)
    for _ in range(10):
        node._tick()
        assert node._state != 'homing_exit'

    for _ in range(needed - 1):
        _marker_at(node, monkeypatch, 3.0)
    assert node._state == 'homing_exit'
    assert node._homing_entry_distance_m == 3.0


def test_duplicate_source_stamp_is_not_a_second_confirmation(
        node, monkeypatch) -> None:
    """Transport duplication of one camera frame cannot satisfy the gate."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    _marker_at(node, monkeypatch, 3.0, stamp=42)
    _marker_at(node, monkeypatch, 3.0, stamp=42)

    assert node._marker_observations == 1
    assert node._near_marker_streak == 1
    assert node._state == 'selecting'


def test_a_far_observation_resets_the_hysteresis(node, monkeypatch) -> None:
    """Near-far-near observations must not count as consecutive."""
    node._start(None, trigger(node))
    node._state = 'selecting'
    far = float(node.get_parameter('homing_max_distance_m').value) + 1.0

    _marker_at(node, monkeypatch, 3.0)
    _marker_at(node, monkeypatch, far)
    _marker_at(node, monkeypatch, 3.0)

    assert node._state != 'homing_exit'


def test_the_gate_sits_in_the_band_where_the_range_was_measured_good(
        node) -> None:
    """Above 4 m, the mean error measured in R7 exceeds 1.1 m."""
    assert 2.0 < float(
        node.get_parameter('homing_max_distance_m').value) <= 4.0


def _homing_ready(node, monkeypatch, sent):
    """Place the node in `homing_exit` with its exit fixed 3 m away."""
    node._start(None, trigger(node))
    node._state = 'homing_exit'
    node._pending = False
    node._goal_handle = None
    monkeypatch.setattr(node, '_robot_pose', lambda: (0.0, 0.0, 0.0))
    monkeypatch.setattr(
        node, '_send_navigation',
        lambda frontier, exploration: sent.append((frontier, exploration)))
    node._exit_pose_map = (3.0, 0.0)
    node._exit_candidate_pose_map = (3.0, 0.0)
    node._exit_seen_s = node._now_s()
    return node


def test_homing_survives_a_briefly_occluded_marker(node, monkeypatch) -> None:
    """
    Losing sight of the marker is not losing the exit.

    `_exit_pose_map` is a fixed coordinate in the map frame. Line of sight is
    used to LEARN where the exit is, not to navigate to it — walking through a
    maze corridor naturally breaks line of sight. Abandoning homing at every
    occlusion is what left field homing at 0 of 11 (R2, observed run, arm B).
    """
    sent: list = []
    _homing_ready(node, monkeypatch, sent)
    stale_s = float(node.get_parameter('marker_stale_s').value)
    node._exit_seen_s = node._now_s() - stale_s - 1.0

    node._tick()

    assert node._state == 'homing_exit'
    assert sent, 'homing must continue approaching the fixed pose'
    assert node.published[-1]['marker_visible'] is False


def test_blind_approach_goes_to_the_exit_not_to_a_half_metre_hop(
        node, monkeypatch) -> None:
    """
    Without a fresh marker there is no reason to re-aim, so short steps only waste time.

    With line of sight, each new detection lets the `homing_step_m` step correct
    the aim. Blindly, this becomes a sequence of straight 0.5 m goals that the
    planner rejects when a wall is in the way; one goal lets Nav2 route around it.
    """
    sent: list = []
    _homing_ready(node, monkeypatch, sent)
    stop = float(node.get_parameter('marker_stop_distance_m').value)
    node._exit_seen_s = node._now_s() - float(
        node.get_parameter('marker_stale_s').value) - 1.0

    node._tick()

    assert sent[-1][1] is False, 'approach is not exploration'
    assert sent[-1][0].x == pytest.approx(3.0 - stop)

    sent.clear()
    node._exit_seen_s = node._now_s()
    node._tick()
    assert sent[-1][0].x == pytest.approx(
        float(node.get_parameter('homing_step_m').value))


def test_homing_gives_up_after_the_persistence_budget(node, monkeypatch) -> None:
    """
    Bound blind pursuit so the freshness timeout bug cannot return.

    The old contract was "without a fresh marker, give up immediately." The new
    one is "without a fresh marker, persist for `homing_persistence_s`, then give
    up" — the explorer never chases the last seen pose forever.
    """
    sent: list = []
    _homing_ready(node, monkeypatch, sent)
    persistence_s = float(node.get_parameter('homing_persistence_s').value)
    node._exit_seen_s = node._now_s() - persistence_s - 1.0

    node._tick()

    assert node._state == 'selecting'
    assert not sent, 'once the budget expires, no more approach goal is dispatched'
    assert node._homing_abandons == 1
    assert node.published[-1]['homing_abandons'] == 1


def test_persistence_budget_outlives_the_freshness_deadline(node) -> None:
    """If the budget were shorter than the freshness timeout, persistence could never happen."""
    assert float(node.get_parameter('homing_persistence_s').value) > float(
        node.get_parameter('marker_stale_s').value)


def test_homing_abandons_is_published_and_reset_by_start(node) -> None:
    """The counter distinguishes "approach abandoned" from "Nav2 goal failed."""
    node._publish_status()
    assert node.published[-1]['homing_abandons'] == 0
    node._homing_abandons = 4
    node._start(None, trigger(node))
    assert node._homing_abandons == 0


def test_goal_timeout_is_sized_from_the_measured_goal_durations(node) -> None:
    """
    The per-goal timeout is a budget, not slack: each timeout costs the full amount.

    Measured in round R5 (arm B, 21 goals): the 12 SUCCESSFUL goals took 6.1 to
    35.1 s, and the three failures each used exactly 90.0 s — 270 s of a 600 s
    budget, 45%, without moving. The run ended 1.08 m from the marker due to
    lack of time.

    The timeout must cover the slowest successful goal with margin, and three
    timeouts must not consume half of the total budget.
    """
    worst_successful_goal_s = 35.1     # R5, meta 2
    observed_timeouts = 3              # R5
    goal_timeout_s = float(node.get_parameter('goal_timeout_s').value)
    total_timeout_s = float(node.get_parameter('total_timeout_s').value)

    # Lower bound: do not cut off a legitimate goal. Upper bound: the three
    # observed timeouts must fit in a quarter of the budget, leaving most of it
    # for movement.
    assert goal_timeout_s > worst_successful_goal_s * 1.2
    assert observed_timeouts * goal_timeout_s <= total_timeout_s / 4


def test_homing_arrives_when_the_remaining_step_is_below_nav2_tolerance(
        node, monkeypatch) -> None:
    """
    Do not command a displacement smaller than Nav2's goal tolerance.

    Measured in R6: the approach reached 0.75 m from the marker with
    `marker_stop_distance_m` at 0.70 — 5 cm short. The remaining 5 cm step is
    less than `xy_goal_tolerance` (0.25 m), so Nav2 reports success without
    moving; the explorer sees 0.75 > 0.70 and sends it again. The robot stayed
    still for 94 s until the persistence budget expired and exploration ended.

    This is the same trap that `min_frontier_distance_m = 0.35` has prevented
    for frontiers since R4; the approach never had an equivalent guard.
    """
    sent: list = []
    _homing_ready(node, monkeypatch, sent)
    stop = float(node.get_parameter('marker_stop_distance_m').value)
    tolerance = float(node.get_parameter('nav_goal_tolerance_m').value)
    # Less than one tolerance remains: sending a goal here caused the R6 loop.
    node._exit_pose_map = (stop + tolerance * 0.5, 0.0)
    node._exit_seen_s = node._now_s()

    node._tick()

    assert node._state == 'completed'
    assert not sent, 'a step below tolerance must not become a Nav2 goal'


def test_homing_arrives_at_exactly_the_nav2_tolerance(node, monkeypatch) -> None:
    """The equality boundary is also a Nav2 no-motion success."""
    sent: list = []
    _homing_ready(node, monkeypatch, sent)
    stop = float(node.get_parameter('marker_stop_distance_m').value)
    tolerance = float(node.get_parameter('nav_goal_tolerance_m').value)
    node._exit_pose_map = (stop + tolerance, 0.0)

    node._tick()

    assert node._state == 'completed'
    assert not sent


def test_homing_still_steps_when_the_remaining_distance_is_worth_commanding(
        node, monkeypatch) -> None:
    """The guard above must not suppress a legitimate approach."""
    sent: list = []
    _homing_ready(node, monkeypatch, sent)
    stop = float(node.get_parameter('marker_stop_distance_m').value)
    tolerance = float(node.get_parameter('nav_goal_tolerance_m').value)
    node._exit_pose_map = (stop + tolerance * 3.0, 0.0)
    node._exit_seen_s = node._now_s()

    node._tick()

    assert node._state == 'homing_exit'
    assert sent


def test_the_homing_tolerance_matches_what_nav2_is_configured_with(node) -> None:
    """
    Two copies of the same number in different files can drift apart.

    The explorer needs Nav2's goal tolerance so it does not command steps the
    controller cannot distinguish from zero. It does not read Nav2's YAML, so
    this test keeps the two values aligned across both parameter files, the
    default and the footprint variant.
    """
    import pathlib

    import yaml

    config = pathlib.Path(__file__).resolve().parents[1] / 'config'
    declared = float(node.get_parameter('nav_goal_tolerance_m').value)
    for name in ('nav2_params_go2.yaml', 'nav2_params_go2_footprint.yaml'):
        params = yaml.safe_load((config / name).read_text(encoding='utf-8'))
        checker = params['controller_server']['ros__parameters'][
            'general_goal_checker']
        assert declared == float(checker['xy_goal_tolerance']), name


# R17 -- directional exploration with breadcrumb backtracking.
#
# Under test: forward/reverse classification by heading cone (with hard
# priority, not a score penalty), the breadcrumb stack, and unified recovery
# dispatch in `_handle_no_usable_frontier`. Reuses the same doubles (`_Future`,
# `_StubGoalHandle`, `_run_selection`) as the rest of the file — the goal is to
# test the state machine, not reinvent test doubles.

def _run_selection_with_heading(
        node, monkeypatch, frontiers, heading, robot=(0.0, 0.0, 0.0),
        breadcrumbs=()):
    """
    Like `_run_selection`, but with heading (and breadcrumbs) already set.

    `_start()` clears `_breadcrumbs`, so this test applies breadcrumbs AFTER it,
    never before.
    """
    node._start(None, trigger(node))
    node._state = 'selecting'
    node._map_seq += 1
    node._current_heading = heading
    node._breadcrumbs = list(breadcrumbs)
    monkeypatch.setattr(node, '_grid', lambda: object())
    monkeypatch.setattr(node, '_robot_pose', lambda: robot)
    monkeypatch.setattr(node, '_validate_next', lambda: None)

    def fake_extract(grid, stats=None, **_kwargs):
        if stats is not None:
            stats['raw_clusters'] = len(frontiers) or 1
        return list(frontiers)

    monkeypatch.setattr(
        maze_explorer_module, 'extract_frontiers', fake_extract)
    node._begin_selection()


def test_a_forward_frontier_beats_a_rearward_one_with_a_higher_score(
        node, monkeypatch) -> None:
    """
    Minimum test 1: forward wins even with lower information gain.

    Priority is hard, not a score penalty — the frontier behind the robot
    (with 5x more information gain) never competes on score: it is removed from
    the candidate list before `_validate_next` runs.
    """
    ahead = Frontier(x=1.0, y=0.0, cells=10, information_gain_m=0.5)
    behind = Frontier(x=-1.0, y=0.0, cells=90, information_gain_m=4.5)
    _run_selection_with_heading(
        node, monkeypatch, [ahead, behind], heading=0.0)

    assert node._decision_mode == 'forward'
    assert node._candidates == [ahead]
    assert behind not in node._candidates


def test_a_rearward_frontier_is_accepted_when_it_is_the_only_one(
        node, monkeypatch) -> None:
    """Minimum test 2: when nothing is ahead, reverse becomes eligible."""
    behind = Frontier(x=-1.0, y=0.0, cells=90, information_gain_m=4.5)
    _run_selection_with_heading(node, monkeypatch, [behind], heading=0.0)

    assert node._decision_mode == 'reverse'
    assert node._candidates == [behind]


def test_a_filtered_cycle_starts_a_breadcrumb_return_not_a_failure(
        node, monkeypatch) -> None:
    """
    Minimum test 3: a candidate-free cycle with a breadcrumb available does not fail.

    R17 does not wait ten barren cycles as R15 did — with a breadcrumb on the
    stack, the first cycle without a usable frontier dispatches the return.
    `_barren_cycles` stays untouched: this cycle produced a goal, just not a
    new exploration goal.
    """
    sent: list[tuple[float, float]] = []
    node._send_navigation = lambda frontier, exploration=True, target=None: \
        sent.append((frontier.x, frontier.y))
    # Existing frontiers that are all filtered (e.g. within goal tolerance)
    # must take the same recovery path, so the input list is empty here.
    _run_selection_with_heading(
        node, monkeypatch, [], heading=0.0, breadcrumbs=[(2.0, 0.0)])

    assert sent == [(2.0, 0.0)]
    assert node._is_backtrack_goal is True
    assert node._decision_mode == 'backtracking'
    assert node._backtrack_attempts == 1
    assert node._breadcrumbs == []
    assert node._barren_cycles == 0


def test_reaching_the_breadcrumb_runs_selection_again(node) -> None:
    """
    Minimum test 4: reaching a breadcrumb must run selection again.

    A successful return goal must go back to `selecting` (not `completed` or
    stuck in `navigating`), with its dedicated message — and without pushing a
    new breadcrumb for the point just popped from the stack to get there.
    """
    node._start(None, trigger(node))
    node._state = 'navigating'
    node._robot_pose = lambda: (2.0, 0.0, 0.0)
    node._current = Frontier(x=2.0, y=0.0, cells=0, information_gain_m=0.0)
    node._is_backtrack_goal = True
    node._nav_departure_pose = (0.0, 0.0)
    node._breadcrumbs = []

    node._on_nav_result(_Future(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED)), node._epoch, True)

    assert node._state == 'selecting'
    assert 'return complete' in node._message
    assert node._is_backtrack_goal is False
    assert node._breadcrumbs == [], (
        'completed return must not push a breadcrumb onto itself')


def test_a_completed_frontier_saves_its_departure_as_the_breadcrumb(
        node) -> None:
    """The first return must retrace its path, not target the current pose again."""
    node._start(None, trigger(node))
    node._state = 'navigating'
    node._robot_pose = lambda: (2.0, 0.0, 0.0)
    node._current = Frontier(x=2.0, y=0.0, cells=10, information_gain_m=0.5)
    node._nav_departure_pose = (0.5, 0.0)

    node._on_nav_result(_Future(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED)), node._epoch, True)

    assert node._breadcrumbs == [(0.5, 0.0)]


def test_a_consumed_breadcrumb_cannot_cause_a_loop(node) -> None:
    """
    Minimum test 5: the stack only shrinks — it never offers the same point again.

    Two consecutive returns must consume two distinct points and empty the
    stack; neither can reappear for a third return.
    """
    node._start(None, trigger(node))
    node._state = 'selecting'
    node._robot_pose = lambda: (0.0, 0.0, 0.0)
    node._breadcrumbs = [(1.0, 1.0), (2.0, 2.0)]
    sent: list[tuple[float, float]] = []
    node._send_navigation = lambda frontier, exploration=True, target=None: \
        sent.append((frontier.x, frontier.y))

    node._start_backtrack()
    assert node._breadcrumbs == [(1.0, 1.0)]
    node._start_backtrack()
    assert node._breadcrumbs == []
    assert sent == [(2.0, 2.0), (1.0, 1.0)], (
        'the two returns must target different points in LIFO order')

    # Empty stack: a third call must not invent a third return.
    node._frontier_clusters_raw = 5  # does not trigger observation scan
    node._handle_no_usable_frontier()
    assert node._barren_cycles == 1, (
        'without a remaining breadcrumb, this counts as a barren cycle, not a return')


def test_no_breadcrumbs_and_no_frontiers_terminates_normally(node) -> None:
    """
    Minimum test 6: without a breadcrumb or frontier, exploration ends via `_fail`.

    Without this, the robot would remain in `selecting` until `total_timeout_s`.
    Breadcrumb recovery must not become a new way to silently stall when there
    is truly nothing left to do.
    """
    node._start(None, trigger(node))
    node._state = 'selecting'
    node._breadcrumbs = []
    node._frontier_clusters_raw = 5  # does not trigger observation scan
    limit = int(node.get_parameter('barren_selections_limit').value)

    for _ in range(limit):
        node._handle_no_usable_frontier()

    assert node._state == 'failed'
    assert node._message == 'no safe frontier reachable'


def test_cancel_also_cancels_an_in_progress_breadcrumb_return(node) -> None:
    """Minimum test 7 (cancellation): cancelling must stop an in-flight return."""
    node._start(None, trigger(node))
    node._state = 'navigating'
    node._current = Frontier(x=2.0, y=0.0, cells=0, information_gain_m=0.0)
    node._is_backtrack_goal = True
    handle = _StubGoalHandle()
    cancelled: list[bool] = []
    handle.cancel_goal_async = lambda: cancelled.append(True)
    node._goal_handle = handle

    node._cancel(None, trigger(node))

    assert cancelled == [True]
    assert node._state == 'cancelled'
    assert node._is_backtrack_goal is False


def test_timeout_also_cancels_an_in_progress_breadcrumb_return(node) -> None:
    """Minimum test 7 (timeout): a return goal also has a deadline."""
    node._start(None, trigger(node))
    node._state = 'navigating'
    node._current = Frontier(x=2.0, y=0.0, cells=0, information_gain_m=0.0)
    node._is_backtrack_goal = True
    handle = _StubGoalHandle()
    cancelled: list[bool] = []
    handle.cancel_goal_async = lambda: cancelled.append(True)
    node._goal_handle = handle

    node._timeout_current('frontier goal timed out')

    assert cancelled == [True]
    assert node._state == 'selecting'
    assert node._is_backtrack_goal is False
