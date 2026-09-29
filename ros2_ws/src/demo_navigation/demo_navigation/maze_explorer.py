"""Frontier exploration executive for the Go2 maze demonstration."""

from __future__ import annotations

from dataclasses import asdict
import json
import math
import struct
import time
import zlib

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .frontier import extract_frontiers, Frontier, frontier_score, Grid, path_length


STATES = {
    'idle', 'waiting_map', 'selecting', 'navigating', 'homing_exit',
    'completed', 'failed', 'cancelled',
}


def _setback_point(
    poses, setback_m: float, min_travel_m: float = 0.0,
) -> tuple[float, float] | None:
    """
    Return a point `setback_m` back from a path's end, along the path itself.

    Walking backward along an already-validated plan keeps the result on a
    route the planner proved reachable -- unlike moving the frontier's own
    goal closer to a wall, which does not make Nav2's controller any more
    willing to track a path that close. Falls back to the path's own start
    point if the whole path is shorter than `setback_m`, and to `None` for an
    empty path (the caller already has the original point to fall back on).

    `min_travel_m` is the floor below which the result is discarded (`None`)
    instead of returned: on a short path, walking back `setback_m` from the
    end can land within Nav2's own `xy_goal_tolerance` of the robot's CURRENT
    pose, so `SimpleGoalChecker` calls the goal reached without the robot
    moving at all -- R11 (29/08) stalled exactly this way, motionless for
    minutes on an identical unchanging map. The caller falls back to the
    original (un-recessed) endpoint when this returns `None`.
    """
    if not poses:
        return None
    points = [(p.pose.position.x, p.pose.position.y) for p in poses]
    start = points[0]

    def far_enough(point: tuple[float, float]) -> bool:
        return math.hypot(point[0] - start[0], point[1] - start[1]) \
            >= min_travel_m

    if len(points) == 1:
        return points[0] if far_enough(points[0]) else None

    remaining = setback_m
    for i in range(len(points) - 1, 0, -1):
        x1, y1 = points[i]
        x0, y0 = points[i - 1]
        segment = math.hypot(x1 - x0, y1 - y0)
        if segment >= remaining:
            ratio = remaining / segment if segment > 0.0 else 0.0
            candidate = (x1 - (x1 - x0) * ratio, y1 - (y1 - y0) * ratio)
            return candidate if far_enough(candidate) else None
        remaining -= segment
    # The whole path is shorter than `setback_m`: there is no point on it
    # that is actually `setback_m` from the end. Falling back to `start`
    # here would hand back the robot's own current pose -- degenerate
    # regardless of `min_travel_m`, so this is always discarded.
    return None


def _map_fingerprint(message: OccupancyGrid) -> int:
    """
    Return a CRC32 over the map's geometry and cell contents together.

    R15 hashed only `message.data`, so a resolution/size/origin change (a
    SLAM re-anchor or a resize with identical cell values, for instance)
    would not advance `_map_seq` -- a real map change silently treated as a
    republication of the same one. Folding width, height, resolution and the
    full origin pose into the hash closes that gap.
    """
    info = message.info
    origin = info.origin
    header = struct.pack(
        '<IIfddddddd',
        info.width, info.height, info.resolution,
        origin.position.x, origin.position.y, origin.position.z,
        origin.orientation.x, origin.orientation.y,
        origin.orientation.z, origin.orientation.w,
    )
    return zlib.crc32(header + bytes(message.data))


class MazeExplorer(Node):
    """Choose reachable map frontiers and hand them to Nav2 one at a time."""

    def __init__(self) -> None:
        super().__init__('maze_explorer')
        self.declare_parameter('exploration_bt_xml', '')
        self.declare_parameter('total_timeout_s', 600.0)
        # 90 s, and NOT 180 s. Round 3 (29/08) raised it to 180 and the result
        # was worse across the board: 4.26 m against 21.93 m, 3746 cells
        # against 8915, work ratio 8.6% against 41.7%, and no marker
        # detection at all.
        #
        # Round 2 had timed out three distant goals at exactly 90.0 s, and the
        # reading was "the ceiling is too short". That reading was wrong:
        # round 3 got stuck for 180 s on a goal 0.4 m from the robot. The
        # ceiling is not cutting off slow traversal, it is cutting off a
        # stall -- and doubling it only makes each stall twice as expensive.
        #
        # What is still wrong is PERMANENCE: a goal that times out goes to the
        # hard blacklist and never comes back. See
        # docs/results/ml35-f5-exploration-r3.md. Measured in R5 (arm B, 21
        # goals): the 12 goals that succeeded took 6.1-35.1 s; the 3 that
        # timed out spent 90.0 s each, 270 s out of a 600 s budget. 45 s
        # covers the worst good goal with a 28% margin and halves the cost of
        # every stuck goal.
        self.declare_parameter('goal_timeout_s', 45.0)
        self.declare_parameter('marker_stale_s', 2.0)
        self.declare_parameter('marker_stop_distance_m', 0.7)
        # Mirrors `xy_goal_tolerance` of Nav2's `general_goal_checker`. This
        # keeps it from commanding a step the controller can't distinguish
        # from zero; a contract test keeps the two values equal.
        self.declare_parameter('nav_goal_tolerance_m', 0.25)
        self.declare_parameter('homing_step_m', 0.5)
        # Budget for the blind approach. `marker_stale_s` says when the
        # marker stopped being seen; this one says for how long it is still
        # worth walking toward the already-latched pose. Without it, homing
        # gave up at the first wall that cut the line of sight -- 0 of 11
        # approaches in the field.
        self.declare_parameter('homing_persistence_s', 90.0)
        # ENTRY gate, sized from R7's 131 samples against the SDF marker: the
        # estimate's mean absolute error is 0.41 m in the 3-4 m range, 1.14 m
        # at 4-6 m, and 3.08 m above 6 m. R7 committed at 7.35 m, and the
        # approach, now persistent, tied up the run for 520 s.
        self.declare_parameter('homing_max_distance_m', 4.0)
        # Hysteresis: the estimate swung from 1.27 to 7.94 m within the same
        # run, and a single sample must not be able to cancel exploration.
        self.declare_parameter('homing_confirm_observations', 3)
        # Fail-safe for the blind approach: a large body tilt means the Go2 is
        # no longer in a trustworthy walking posture.  Cancel the active goal
        # instead of continuing to command toward a latched marker pose.
        # Normal trotting tilt is well below this value; the threshold is kept
        # deliberately conservative because this guard is only active during
        # homing and is not a navigation controller.
        self.declare_parameter('homing_max_tilt_deg', 15.0)
        self.declare_parameter('blacklist_radius_m', 0.75)
        # CONSECUTIVE selection cycles with no viable candidate before
        # declaring failure. Exists because retiring refused frontiers (see
        # `_on_path_result`) can end up swallowing all of them, and a
        # stationary robot does not produce a new map -- the situation never
        # resolves itself. At `_tick`'s 1 Hz, 10 is ~10 s: loose enough for
        # the window where the map has not yet updated after an arrival, and
        # short enough to not burn the budget while stalled.
        self.declare_parameter('barren_selections_limit', 10)
        # Minimum distance between the robot and a frontier for it to be a
        # candidate.
        #
        # Nav2's `xy_goal_tolerance` is 0.25 m (`nav2_params_go2.yaml`). A
        # frontier closer than that makes Nav2 return success WITHOUT
        # anything moving: selection goes back to the same point, the map
        # doesn't change, and the cycle repeats. Round 4 of 29/08: 565 times
        # in 580 s, robot inside an 11 mm x 25 mm box, map frozen at 2669
        # cells.
        #
        # 0.35 m gives margin over the tolerance without hiding a useful
        # frontier. The cutoff is relative to the CURRENT pose and
        # recomputed every cycle -- it is not an annotation, it does not
        # enter any of the three suppression lists, and walking a few
        # centimeters returns the frontier to contention on its own.
        self.declare_parameter('min_frontier_distance_m', 0.35)
        # Radius, in `extract_frontiers`, that a target cell must keep clear
        # of any occupied cell to become a candidate (`has_clearance` in
        # frontier.py). The previous default, 0.45 m, is larger than half the
        # footprint's length (0.37 m, `nav2_params_go2.yaml`) and rejected
        # cells near gaps and corners -- exactly where a narrow frontier
        # meets the wall. 0.38 m keeps real margin over the footprint (not
        # over `robot_radius`, which has already been replaced) and lets the
        # robot get closer to the wall ahead before the frontier on that side
        # gets discarded. Feedback from 29/08: the robot was giving up too
        # early near walls and missing openings.
        self.declare_parameter('frontier_wall_clearance_m', 0.38)
        # Alternates per cluster, and how far apart they must sit. R10
        # (29/08) died in 28.5 s because its one frontier cluster had exactly
        # one candidate point, that point was refused by the planner
        # (NO_VALID_PATH), and there was nothing else in the SAME cluster to
        # retry -- the whole cluster was lost over one unreachable point.
        # These let `extract_frontiers` hand back backup points from the same
        # cluster so a single bad point no longer costs the whole region.
        self.declare_parameter('frontier_max_alternates', 2)
        self.declare_parameter('frontier_alternate_spacing_m', 0.25)
        # After ComputePathToPose validates a candidate, navigate to a point
        # this far back from the endpoint ALONG THE RETURNED PATH, not to the
        # endpoint itself. Feedback 29/08: the robot gave up on a corridor
        # too early on meeting a wall ahead, missing the openings beside it --
        # but pulling the frontier's own goal placement closer to the wall
        # (see `frontier_wall_clearance_m` above) does not, on its own, make
        # Nav2's controller willing to track a path that close. A point set
        # back along a path the planner already proved reachable does not
        # have that problem: it is still on a validated route, just short of
        # its far end. The original endpoint is kept for scoring only
        # (`frontier_score`/`information_gain_m`) so scoring still reflects
        # the real frontier, not the shortened approach.
        self.declare_parameter('frontier_endpoint_setback_m', 0.40)
        # R15 (30/08/2026). Movement watchdog during `navigating`: goal
        # accepted but the robot makes no progress (neither translation nor
        # rotation). Measured in R13
        # (docs/results/ml35-f5-exploration-r13.md): 5 real windows of
        # commanded stillness, 10.2-24.2 s long, and 4 of the 7 goals that
        # timed out (45 s) had one of these windows inside them.
        #
        # CORRECTION (code review post-R14c): the original version of this
        # comment said "15 s stays below the shortest of the 5" -- wrong,
        # 15 > 10.2. In fact 15 s only catches 2 of the 5 windows measured in
        # R13 (15.1 and 24.2 s); the other three (11.7, 10.7, and 10.2 s)
        # fall below the threshold and would NOT trigger the watchdog. This
        # is a deliberate choice (not capturing every short pause of normal
        # replanning as a stall), not a claim of total coverage -- but the
        # previous comment mistakenly claimed total coverage. There is still
        # a large margin left against `goal_timeout_s`'s 45 s; the goal is to
        # act BEFORE the goal's deadline runs out in the longer windows, not
        # to replace it or catch every case.
        self.declare_parameter('stall_window_s', 15.0)
        # Same displacement threshold that
        # `find_stalled_navigating_windows` in
        # `tools/evaluation/exploration_trial.py` already uses against real
        # R13 data -- the two have to agree, or the field watchdog and the
        # offline diagnostic would classify the same run differently.
        self.declare_parameter('stall_move_threshold_m', 0.05)
        # R15a (30/08/2026). ANGULAR progress, equivalent to the translation
        # one above -- without it, a legitimate rotation in place (e.g.
        # turning to face a corridor) with no xy displacement would be
        # classified as a stall. 0.05 rad (~2.9 degrees) sits above typical
        # localization noise with the robot stationary and well below any
        # deliberate rotation -- a code-level judgment call, still without
        # dedicated HIL data on rotational stalls (CLAUDE.md rule 7: never
        # claim hardware validation that was not performed).
        self.declare_parameter('stall_rotate_threshold_rad', 0.05)
        # Observation sweep when NO raw frontier cluster exists at all (not
        # when one exists but was filtered out -- spinning reveals nothing
        # new in that case). ~60 degrees: a full turn at max_rotational_vel
        # 0.12 rad/s (behavior_server, nav2_params_go2.yaml) takes ~52 s,
        # almost an entire goal's budget; a smaller slice, repeated on every
        # map version that keeps having no cluster at all, covers the
        # surroundings progressively without monopolizing the total budget.
        self.declare_parameter('recovery_spin_rad', 1.047)
        # R17 (30/08/2026). Heading cone (degrees) within which a frontier
        # still counts as "forward" -- up to 120 degrees of deviation from
        # the established heading, enough for turns and side corridors
        # without treating every change of direction as a return. Only the
        # 60-degree arc on each side of the exact opposite direction (the
        # remaining 120 of the 360 degrees) counts as reverse. As long as at
        # least one reachable forward frontier exists, no reverse one is
        # considered -- an explicit priority, not a soft penalty.
        self.declare_parameter('forward_cone_deg', 120.0)
        # R17. Minimum spacing between consecutive breadcrumbs, so as not to
        # stack nearly identical points when goals end up close to one
        # another -- the stack exists to mark real intersections
        # (spawn -> corridor A -> intersection B -> ...), not every stop.
        self.declare_parameter('breadcrumb_min_spacing_m', 0.75)

        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            String, '/demo/exploration/status', transient)
        self.create_service(Trigger, '/demo/exploration/start', self._start)
        self.create_service(Trigger, '/demo/exploration/cancel', self._cancel)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, transient)
        self.create_subscription(
            PoseStamped, '/demo/perception/maze_exit/pose',
            self._on_exit_pose, 10)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._path_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose')
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        self._nav_cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self._spin_client = ActionClient(self, Spin, 'spin')

        self._state = 'idle'
        self._message = ''
        self._map: OccupancyGrid | None = None
        # Map sequence, and not the map itself, as the cache key: comparing
        # two OccupancyGrids cell by cell would cost more than the extraction
        # the cache exists to avoid.
        #
        # R15 (30/08/2026): only advances when the CONTENT changes (`_on_map`
        # compares a checksum, not just a message count). `slam_toolbox`
        # republishes `/map` periodically even without a real change, and
        # before this fix every republication counted as a "new map" for
        # `map_seq > last_provisional_map_seq` in `_begin_selection` --
        # releasing a provisional suppression that no new observation had
        # disproved. The test
        # `test_map_republication_does_not_release_a_provisional_recovery`
        # covers exactly this case.
        self._map_seq = 0
        self._map_content_hash: int | None = None
        self._selection_key: tuple[int, int, int] | None = None
        self._frontier_count = 0
        # Instrumentation. Measured with a MONOTONIC clock, never with
        # /clock: under `use_sim_time` the simulation clock can pause, jump,
        # or run out of step with real time, and what matters here is actual
        # CPU time spent.
        self._frontier_extract_ms = 0.0
        self._frontier_cells = 0
        self._frontier_clusters = 0
        self._frontier_clusters_raw = 0
        self._path_requests = 0
        self._selection_cycle = 0
        self._near_skipped = 0
        self._current: Frontier | None = None
        self._blacklist: list[tuple[float, float]] = []
        # THREE lists, because the three kinds of failure don't mean the
        # same thing.
        #
        # `_blacklist` is hard: Nav2 refused the goal, or returned an
        # explicit failure for it. That is an EXECUTION failure of that
        # frontier, and a new map does not disprove it.
        #
        # `_timed_out` is provisional: the goal exceeded `goal_timeout_s`.
        # This marks the ATTEMPT, not the frontier -- round 3 of 29/08 spent
        # 180 s on a goal 0.4 m from the robot, so the ceiling cuts off a
        # stall, and a stall speaks to the pose, the costmap, and the plan of
        # that instant. In round 2, three timeouts turned into three
        # permanent points that swallowed the remaining four clusters by
        # 570 s.
        #
        # `_refused` is provisional: the planner found no path RIGHT NOW.
        # `ExplorationGrid` runs with `allow_unknown: false`, so every
        # distant frontier is refused while the path to it crosses unknown
        # space -- and that is exactly what exploration is going to undo.
        # Measured in round 1 of 29/08: the only two refusals were 2.3 m and
        # 2.7 m from the robot, and retiring them for good left 3 clusters
        # and 157 cells of real frontier with no candidate allowed at all.
        # Reducing the radius would not help: the annotated point IS the
        # cluster's centroid.
        self._refused: list[tuple[float, float]] = []
        self._timed_out: list[tuple[float, float]] = []
        # Guards the provisional-recovery release below: a fresh entry must
        # not be released in the very same map generation that produced it.
        # R9 (29/08) re-selected and re-timed-out the identical coordinate
        # twice in a row (goals 7-8) because a recovery fired between them
        # with no map change in between -- releasing a suppression the map
        # has not yet had a chance to disprove.
        self._last_provisional_map_seq = -1
        # If provisional suppression covers every otherwise usable frontier,
        # release it once. A second dead end before real navigation progress
        # must count as barren instead of creating a refuse/release livelock.
        self._provisional_recovery_used = False
        self._provisional_recoveries = 0
        # Keep the raw visual candidate separate from the pose accepted for
        # homing. R7 measured estimates swinging from 1.27 to 7.94 m; a later
        # partial view must not overwrite a target which already passed the
        # entry gate.
        self._marker_distance_m: float | None = None
        self._homing_entry_distance_m: float | None = None
        self._homing_entries = 0
        self._barren_cycles = 0
        self._started_s = 0.0
        self._goal_started_s = 0.0
        self._epoch = 0
        self._pending = False
        self._goal_handle = None
        self._candidates: list[Frontier] = []
        self._candidate_index = 0
        # Which point of the CURRENT candidate frontier is under test: 0 is
        # its primary (x, y), 1..N its `alternates`. Reset whenever
        # `_candidate_index` moves to a new cluster.
        self._candidate_alt_index = 0
        self._best: tuple[float, Frontier, tuple[float, float]] | None = None
        # Telemetry of the most recent ComputePathToPose attempt, for the
        # status message -- without this, a refusal like R10's could only be
        # explained by reconstructing it offline after the fact.
        self._last_candidate_point: tuple[float, float] | None = None
        self._last_path_status: int | None = None
        self._last_path_error_code: int | None = None
        self._last_path_error_msg: str = ''
        self._last_path_planner_id: str = ''
        # The frontier's own endpoint vs. the point actually commanded to
        # Nav2 -- normally identical, but a setback point (see
        # `_setback_point`) makes them differ for exploration goals. Kept
        # separate so a HIL report can tell which one was used without
        # reconstructing it from the path afterwards.
        self._last_nav_original: tuple[float, float] | None = None
        self._last_nav_target: tuple[float, float] | None = None
        self._exit_candidate_pose_map: tuple[float, float] | None = None
        self._exit_pose_map: tuple[float, float] | None = None
        self._exit_seen_s = 0.0
        self._last_exit_observation: tuple[str, int] | None = None
        self._marker_observations = 0
        self._homing_failures = 0
        self._marker_far_ignored = 0
        self._near_marker_streak = 0
        self._homing_abandons = 0
        # Movement watchdog (R15/R15a): last pose (x, y, yaw) and instant at
        # which the robot actually made progress (xy or angular) since the
        # ACCEPTANCE of the CURRENT `navigating` goal -- set in
        # `_on_nav_accepted`, not at dispatch, so as not to count Nav2's
        # response time as a stall.
        self._nav_last_pose: tuple[float, float, float] | None = None
        self._nav_last_progress_s = 0.0
        # Observation sweep (R15) when no raw cluster exists.
        self._recovery_pending = False
        self._recovery_handle = None
        self._recovery_map_seq = -1
        self._recovery_attempts = 0
        # R17: directional exploration with breadcrumb backtracking.
        #
        # Real heading (direction of displacement, not the final yaw) since
        # the last completed goal -- `None` until the first one, when there
        # is no basis to classify anything as "forward" or "reverse".
        self._current_heading: float | None = None
        # Robot pose at the dispatch of the CURRENT goal, to measure the
        # real displacement on arrival (`_update_heading`) -- not the pose
        # at the end, which only says where it stopped, not where it came
        # from.
        self._nav_departure_pose: tuple[float, float] | None = None
        # Stack of safe poses, one per completed exploration goal (not per
        # return), spaced by `breadcrumb_min_spacing_m`. Each entry is the
        # DEPARTURE pose of the completed goal: on reaching a dead end, the
        # top points to where the robot was before entering it, never to
        # its own current pose.
        # Consumed (removed) the moment a return begins -- never reused,
        # which prevents a cycle between two points.
        self._breadcrumbs: list[tuple[float, float]] = []
        self._is_backtrack_goal = False
        self._backtrack_attempts = 0
        self._decision_mode: str | None = None
        self._forward_candidates_count = 0
        self._reverse_candidates_count = 0
        self._heading_delta_deg: float | None = None
        self.create_timer(1.0, self._tick)
        self._publish_status()

    def _start(self, _request, response):
        if self._state in {'waiting_map', 'selecting', 'navigating', 'homing_exit'}:
            response.success = False
            response.message = 'search already in progress'
            return response
        self._epoch += 1
        self._state = 'waiting_map'
        self._message = 'waiting for map, TF and Nav2'
        self._started_s = self._now_s()
        self._blacklist.clear()
        self._refused.clear()
        self._timed_out.clear()
        self._last_provisional_map_seq = -1
        self._provisional_recovery_used = False
        self._provisional_recoveries = 0
        self._candidate_alt_index = 0
        self._last_candidate_point = None
        self._last_path_status = None
        self._last_path_error_code = None
        self._last_path_error_msg = ''
        self._last_path_planner_id = ''
        self._last_nav_original = None
        self._last_nav_target = None
        self._marker_distance_m = None
        self._homing_entry_distance_m = None
        self._exit_candidate_pose_map = None
        self._exit_pose_map = None
        self._exit_seen_s = 0.0
        self._last_exit_observation = None
        self._marker_observations = 0
        self._homing_entries = 0
        self._barren_cycles = 0
        self._homing_failures = 0
        self._marker_far_ignored = 0
        self._near_marker_streak = 0
        self._homing_abandons = 0
        self._nav_last_pose = None
        self._nav_last_progress_s = 0.0
        self._recovery_pending = False
        self._recovery_handle = None
        self._recovery_map_seq = -1
        self._recovery_attempts = 0
        self._current_heading = None
        self._nav_departure_pose = None
        self._breadcrumbs = []
        self._is_backtrack_goal = False
        self._backtrack_attempts = 0
        self._decision_mode = None
        self._forward_candidates_count = 0
        self._reverse_candidates_count = 0
        self._heading_delta_deg = None
        self._release_goal()
        if self._nav_cancel_client.service_is_ready():
            # Empty goal_info means every active NavigateToPose goal.  Starting
            # autonomous exploration must not race a manual cockpit goal.
            self._nav_cancel_client.call_async(CancelGoal.Request())
        self._publish_status()
        response.success = True
        response.message = 'search started'
        return response

    def _cancel(self, _request, response):
        self._epoch += 1
        self._cancel_goal()
        self._state = 'cancelled'
        self._message = 'search cancelled by operator'
        self._publish_status()
        response.success = True
        response.message = self._message
        return response

    def _on_map(self, message: OccupancyGrid) -> None:
        # Store and count, that's all. A new map does NOT swap the in-flight
        # goal: `_tick` is the one that decides selection, and it only calls
        # `_begin_selection` in the `selecting` state. Reacting here would
        # make the robot abandon the frontier on every SLAM publication.
        #
        # `_map_seq` only advances when the map truly changes -- geometry OR
        # cells, see `_map_fingerprint` -- comment next to the field's
        # declaration in `__init__`.
        self._map = message
        content_hash = _map_fingerprint(message)
        if content_hash != self._map_content_hash:
            self._map_content_hash = content_hash
            self._map_seq += 1

    def _on_exit_pose(self, message: PoseStamped) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', message.header.frame_id, Time())
        except TransformException:
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        cosine, sine = math.cos(yaw), math.sin(yaw)
        x, y = message.pose.position.x, message.pose.position.y
        candidate = (
            translation.x + cosine * x - sine * y,
            translation.y + sine * x + cosine * y,
        )
        stamp_ns = message.header.stamp.sec * 1_000_000_000 \
            + message.header.stamp.nanosec
        observation = (message.header.frame_id, stamp_ns)
        if observation == self._last_exit_observation:
            return
        self._last_exit_observation = observation
        self._marker_observations += 1
        self._exit_candidate_pose_map = candidate
        self._exit_seen_s = self._now_s()
        self._marker_distance_m = self._distance_to_pose(candidate)

        # Count camera observations here, not timer cycles. A pose remains
        # fresh across several `_tick` calls; counting there allowed a single
        # bad frame to satisfy all three confirmations.
        if self._state not in {'waiting_map', 'selecting', 'navigating'} \
                or self._marker_distance_m is None:
            return
        if self._marker_distance_m > float(
                self.get_parameter('homing_max_distance_m').value):
            self._near_marker_streak = 0
            self._marker_far_ignored += 1
            return
        self._near_marker_streak += 1
        if self._near_marker_streak < int(
                self.get_parameter('homing_confirm_observations').value):
            return

        self._exit_pose_map = candidate
        self._homing_entry_distance_m = self._marker_distance_m
        self._homing_entries += 1
        self._epoch += 1
        self._cancel_goal()
        self._state = 'homing_exit'
        self._message = 'exit marker detected'
        self._near_marker_streak = 0
        # Preserve the exact entry event for a slower external recorder.
        self._publish_status()

    def _distance_to_pose(
        self, target: tuple[float, float] | None,
    ) -> float | None:
        """Return robot-to-target map distance, or None without target/TF."""
        if target is None:
            return None
        robot = self._robot_pose()
        if robot is None:
            return None
        return round(math.hypot(target[0] - robot[0], target[1] - robot[1]), 2)

    def _distance_to_exit(self) -> float | None:
        """Return distance to the accepted homing target, if one exists."""
        return self._distance_to_pose(self._exit_pose_map)

    def _tick(self) -> None:
        if self._state not in {'waiting_map', 'selecting', 'navigating', 'homing_exit'}:
            self._publish_status()
            return
        now = self._now_s()
        if now - self._started_s >= float(
                self.get_parameter('total_timeout_s').value):
            self._fail('total exploration deadline exceeded')
            return
        marker_fresh = self._exit_candidate_pose_map is not None \
            and now - self._exit_seen_s <= float(
                self.get_parameter('marker_stale_s').value)
        self._marker_distance_m = self._distance_to_pose(
            self._exit_candidate_pose_map)

        if self._state == 'homing_exit':
            tilt_deg = self._robot_tilt_deg()
            if tilt_deg is not None and tilt_deg > float(
                    self.get_parameter('homing_max_tilt_deg').value):
                self._fail(
                    'homing interrupted by body tilt: '
                    f'{tilt_deg:.1f} degrees')
                return

        if self._state == 'waiting_map':
            if self._map is not None and self._robot_pose() is not None \
                    and self._path_client.server_is_ready() \
                    and self._nav_client.server_is_ready():
                self._state = 'selecting'
                self._message = ''
        elif self._state == 'selecting' and not self._pending:
            self._begin_selection()
        elif self._state == 'navigating':
            if now - self._goal_started_s >= float(
                    self.get_parameter('goal_timeout_s').value):
                self._timeout_current('frontier goal timed out')
            elif self._navigation_stalled(now):
                self._timeout_current(
                    'movement watchdog: robot stationary (no xy/angular progress)')
        elif self._state == 'homing_exit' and not self._pending \
                and self._goal_handle is None:
            if marker_fresh:
                self._send_homing_step()
            elif now - self._exit_seen_s <= float(
                    self.get_parameter('homing_persistence_s').value):
                # The exit is already latched in `_exit_pose_map`; the line
                # of sight served to learn it, not to reach it. Keep going
                # blind.
                self._send_homing_step(blind=True)
            else:
                self._homing_abandons += 1
                self._state = 'selecting'
                self._message = 'marker lost; resuming frontiers'
                self._exit_pose_map = None
                self._near_marker_streak = 0
        self._publish_status()

    def _navigation_stalled(self, now: float) -> bool:
        """
        Return True when an accepted goal has produced no real progress.

        Sliding window: xy displacement above `stall_move_threshold_m` OR
        rotation above `stall_rotate_threshold_rad` resets the clock -- a
        legitimate rotation in place (turning to face a corridor) is not a
        stall just for not moving in a straight line. Only fires after
        `stall_window_s` with neither of the two -- see the justification
        with R13's numbers next to the parameters' declaration in
        `__init__`.

        Only evaluated after Nav2 has accepted the goal (`_goal_handle` set
        in `_on_nav_accepted`) -- before that there is no command in
        execution to stall, only a service call still in flight.
        """
        if self._goal_handle is None:
            return False
        robot = self._robot_pose()
        if robot is None:
            return False
        x, y, yaw = robot
        if self._nav_last_pose is None:
            self._nav_last_pose = (x, y, yaw)
            self._nav_last_progress_s = now
            return False
        last_x, last_y, last_yaw = self._nav_last_pose
        moved = math.hypot(x - last_x, y - last_y)
        turned = abs(math.atan2(
            math.sin(yaw - last_yaw), math.cos(yaw - last_yaw)))
        if moved >= float(self.get_parameter('stall_move_threshold_m').value) \
                or turned >= float(
                    self.get_parameter('stall_rotate_threshold_rad').value):
            self._nav_last_pose = (x, y, yaw)
            self._nav_last_progress_s = now
            return False
        return now - self._nav_last_progress_s >= float(
            self.get_parameter('stall_window_s').value)

    def _grid(self) -> Grid | None:
        if self._map is None:
            return None
        origin = self._map.info.origin
        q = origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return Grid(
            self._map.info.width, self._map.info.height,
            self._map.info.resolution,
            origin.position.x, origin.position.y, yaw, self._map.data,
        )

    def _begin_selection(self) -> None:
        if self._recovery_pending:
            # Observation sweep in flight -- see
            # `_start_observation_recovery`. Re-extracting now would run over
            # the same map that justified it.
            return
        grid = self._grid()
        robot = self._robot_pose()
        if grid is None or robot is None:
            self._state = 'waiting_map'
            return

        # With no new map, no new blacklist and no new epoch, extraction
        # would give exactly the same result. Without this guard, the
        # "planner rejected all frontiers" case leaves `_pending` at False
        # and `_tick` re-extracts the ENTIRE map every second, indefinitely --
        # which was the explorer holding an AM69 core with nothing to show
        # for it.
        #
        # The epoch enters the key so that starting or cancelling the search
        # forces an extraction, even if the map and the blacklist are
        # unchanged.
        # R17: the breadcrumb stack enters the key because a return can
        # change the selection result WITHOUT changing epoch, map, or
        # suppression -- the real change is the robot's pose after the
        # return. Without this, arriving at the breadcrumb would reproduce
        # the SAME key as the last selection failure and would fall into the
        # "nothing changed, counts as barren" shortcut before even
        # re-extracting frontiers from the new position.
        key = (self._epoch, self._map_seq,
               len(self._blacklist) + len(self._refused)
               + len(self._timed_out), len(self._breadcrumbs))
        if key == self._selection_key:
            # Nothing changed since the previous cycle, so there is nothing
            # to re-extract -- but there was also no progress, and staying
            # here is indistinguishable from being stuck. Counts toward the
            # limit.
            self._note_barren_selection()
            return
        self._selection_key = key

        started = time.monotonic()
        extract_stats: dict = {}
        frontiers = extract_frontiers(
            grid,
            clearance_m=float(
                self.get_parameter('frontier_wall_clearance_m').value),
            max_alternates=int(
                self.get_parameter('frontier_max_alternates').value),
            alternate_spacing_m=float(
                self.get_parameter('frontier_alternate_spacing_m').value),
            stats=extract_stats,
        )
        self._frontier_extract_ms = round((time.monotonic() - started) * 1e3, 1)
        self._selection_cycle += 1
        self._frontier_clusters = len(frontiers)
        self._frontier_clusters_raw = extract_stats.get('raw_clusters', 0)
        self._frontier_cells = sum(item.cells for item in frontiers)

        # Apply permanent and geometric exclusions first. This intermediate
        # set tells whether provisional suppression alone caused a dead end.
        frontiers = [item for item in frontiers if not any(
            math.hypot(item.x - x, item.y - y) <= float(
                self.get_parameter('blacklist_radius_m').value)
            for x, y in self._blacklist)]
        # After suppression and BEFORE sorting: sorting is by proximity, so
        # without this cutoff the degenerate frontier would always be the
        # first candidate.
        near_limit = float(
            self.get_parameter('min_frontier_distance_m').value)
        reachable = [item for item in frontiers if math.hypot(
            item.x - robot[0], item.y - robot[1]) >= near_limit]
        self._near_skipped = len(frontiers) - len(reachable)
        provisional = list(self._refused) + list(self._timed_out)
        frontiers = [item for item in reachable if not any(
            math.hypot(item.x - x, item.y - y) <= float(
                self.get_parameter('blacklist_radius_m').value)
            for x, y in provisional)]

        # R4a ended with real clusters but zero permitted candidates: every
        # cluster was covered by provisional entries whose only release event
        # was reaching another frontier. Break that circular dependency once.
        # If the retried frontiers fail again before a successful arrival, the
        # normal barren limit terminates the run instead of clearing forever.
        if reachable and not frontiers and provisional \
                and not self._provisional_recovery_used \
                and self._map_seq > self._last_provisional_map_seq:
            self._refused.clear()
            self._timed_out.clear()
            self._provisional_recovery_used = True
            self._provisional_recoveries += 1
            frontiers = reachable
            self._message = 'released provisional frontier suppressions'
        frontiers.sort(key=lambda item: math.hypot(
            item.x - robot[0], item.y - robot[1]))
        self._frontier_count = len(frontiers)
        # R17: forward wins whenever it exists -- reverse is only considered
        # when there is no reachable forward frontier at all.
        forward, reverse = self._split_forward_reverse(frontiers, robot)
        self._forward_candidates_count = len(forward)
        self._reverse_candidates_count = len(reverse)
        if forward:
            self._decision_mode = 'forward'
            selected = forward
        elif reverse:
            self._decision_mode = 'reverse'
            selected = reverse
        else:
            self._decision_mode = None
            selected = []
        self._candidates = selected[:8]
        self._candidate_index = 0
        self._candidate_alt_index = 0
        self._best = None
        if not self._candidates:
            # R15: honest classification of why there is no candidate,
            # instead of a single 'no safe frontier reachable' label for
            # three distinct causes -- see `classify_stop_reason` in
            # `tools/evaluation/exploration_trial.py`, which already
            # separates these counts.
            if self._near_skipped:
                self._message = ('all frontiers are within arrival '
                                 'tolerance')
            elif self._frontier_clusters_raw == 0:
                self._message = 'no raw frontier cluster'
            else:
                self._message = 'frontiers exist but were filtered out'
            self._handle_no_usable_frontier()
            return
        self._validate_next()

    def _split_forward_reverse(
        self, frontiers: list[Frontier], robot: tuple[float, float, float],
    ) -> tuple[list[Frontier], list[Frontier]]:
        """
        Split frontiers into forward (within the heading cone) and reverse.

        R17: with no heading established yet (no exploration goal completed
        in this search), treats everything as forward -- there is no basis
        to penalize anything before the first real displacement.
        """
        if self._current_heading is None:
            return list(frontiers), []
        cone = float(self.get_parameter('forward_cone_deg').value)
        forward: list[Frontier] = []
        reverse: list[Frontier] = []
        for item in frontiers:
            bearing = math.atan2(item.y - robot[1], item.x - robot[0])
            delta = math.degrees(math.atan2(
                math.sin(bearing - self._current_heading),
                math.cos(bearing - self._current_heading)))
            (forward if abs(delta) <= cone else reverse).append(item)
        return forward, reverse

    def _handle_no_usable_frontier(self) -> None:
        """
        R17: no usable candidate -- breadcrumb backtrack first.

        Order: a not-yet-consumed breadcrumb is the cheapest option
        (doesn't spin, doesn't spend sweep budget) and the one most aligned
        with the goal of only backing off when there truly is nowhere else
        to go. The observation sweep only comes in once the stack is
        already empty, and under exactly the same conditions as before (no
        raw cluster, at most one attempt per map version) -- with no
        breadcrumbs available, the behavior is identical to R15's.
        """
        if self._breadcrumbs:
            self._start_backtrack()
            return
        if self._frontier_clusters_raw == 0 \
                and self._map_seq > self._recovery_map_seq:
            self._recovery_map_seq = self._map_seq
            self._start_observation_recovery()
            return
        self._note_barren_selection()

    def _start_backtrack(self) -> None:
        """
        Return to the most recent breadcrumb instead of declaring failure outright.

        Consumed (popped from the stack) the moment the return navigation
        starts, not when it ends -- a return that fails (Nav2 refuses or
        times out) must not keep trying the SAME point forever. The stack
        only shrinks, never reuses an entry already popped: that is what
        prevents an infinite cycle between two points.
        """
        x, y = self._breadcrumbs.pop()
        self._backtrack_attempts += 1
        self._decision_mode = 'backtracking'
        frontier = Frontier(x=x, y=y, cells=0, information_gain_m=0.0)
        self._is_backtrack_goal = True
        self._send_navigation(frontier, exploration=True)

    def _start_observation_recovery(self) -> None:
        """
        Spin in place, within the limits `behavior_server` already validated.

        Spins within `max_rotational_vel: 0.12` (`nav2_params_go2.yaml`),
        measured so as not to knock the robot over during recovery. Only
        called when no raw frontier cluster exists -- see
        `_begin_selection`.
        """
        self._recovery_pending = True
        self._recovery_attempts += 1
        goal = Spin.Goal()
        goal.target_yaw = float(self.get_parameter('recovery_spin_rad').value)
        epoch = self._epoch
        future = self._spin_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done: self._on_recovery_accepted(done, epoch))

    def _on_recovery_accepted(self, future, epoch: int) -> None:
        if epoch != self._epoch:
            return
        handle = future.result()
        if not handle.accepted:
            self._recovery_pending = False
            self._note_barren_selection()
            return
        self._recovery_handle = handle
        handle.get_result_async().add_done_callback(
            lambda done: self._on_recovery_result(done, epoch))

    def _on_recovery_result(self, future, epoch: int) -> None:
        del future
        if epoch != self._epoch:
            return
        self._recovery_pending = False
        self._recovery_handle = None
        # The sweep itself decides nothing; it is the next /map with new
        # content that counts as progress (`_map_seq`). With no new map,
        # this barren cycle advances toward the normal limit -- a sweep that
        # revealed nothing cannot spin forever.
        self._note_barren_selection()

    def _validate_next(self) -> None:
        if self._candidate_index >= len(self._candidates):
            if self._best is None:
                self._message = 'planner rejected all frontiers'
                self._pending = False
                return
            _, frontier, target = self._best
            self._send_navigation(frontier, exploration=True, target=target)
            return
        frontier = self._candidates[self._candidate_index]
        points = ((frontier.x, frontier.y),) + frontier.alternates
        if self._candidate_alt_index >= len(points):
            # Every point of this cluster (primary and alternates) was
            # refused. The planner REJECTED this frontier. Without retiring
            # it, it comes back identical on the next cycle, forever: that
            # is what consumed 459 s of 600 s in the 28/08 smoke test, with
            # the map frozen and one `ComputePathToPose` per second against
            # the same dead coordinate.
            #
            # Annotated by the PRIMARY point (the one that identifies the
            # cluster for the radius-based suppressions), not by the last
            # point tried -- the suppressions suppress the REGION, not a
            # specific point within it. `len(self._blacklist)`/`_refused`
            # are already part of `_begin_selection`'s key, so the append
            # alone already forces a fresh extraction on the next cycle.
            self._refused.append((frontier.x, frontier.y))
            self._last_provisional_map_seq = self._map_seq
            self._candidate_index += 1
            self._candidate_alt_index = 0
            self._validate_next()
            return
        point = points[self._candidate_alt_index]
        self._candidate_alt_index += 1
        goal = ComputePathToPose.Goal()
        goal.goal = self._pose(point[0], point[1], 0.0)
        goal.planner_id = 'ExplorationGrid'
        goal.use_start = False
        self._pending = True
        self._path_requests += 1
        self._last_candidate_point = point
        self._last_path_planner_id = goal.planner_id
        epoch = self._epoch
        future = self._path_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done: self._on_path_accepted(done, epoch, frontier, point))

    def _on_path_accepted(
        self, future, epoch: int, frontier: Frontier,
        point: tuple[float, float],
    ) -> None:
        if epoch != self._epoch:
            return
        handle = future.result()
        if not handle.accepted:
            self._pending = False
            self._validate_next()
            return
        handle.get_result_async().add_done_callback(
            lambda done: self._on_path_result(done, epoch, frontier, point))

    def _on_path_result(
        self, future, epoch: int, frontier: Frontier,
        point: tuple[float, float],
    ) -> None:
        if epoch != self._epoch:
            return
        wrapped = future.result()
        self._last_path_status = wrapped.status
        self._last_path_error_code = wrapped.result.error_code
        self._last_path_error_msg = wrapped.result.error_msg
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            route_m = path_length(wrapped.result.path.poses)
            score = frontier_score(frontier, route_m)
            if self._best is None or score > self._best[0]:
                # Floor: xy_goal_tolerance plus a margin, so the setback point
                # can never fall inside the radius Nav2's own goal checker
                # already treats as "arrived" -- see `_setback_point`'s
                # docstring for the R11 stall this guards against.
                setback = _setback_point(
                    wrapped.result.path.poses,
                    float(self.get_parameter(
                        'frontier_endpoint_setback_m').value),
                    min_travel_m=float(self.get_parameter(
                        'nav_goal_tolerance_m').value) + 0.10,
                )
                self._best = score, frontier, (setback or point)
            # This cluster already has a validated point; its remaining
            # alternates would only re-check the same region. Move on to the
            # next cluster instead of retrying them.
            self._candidate_index += 1
            self._candidate_alt_index = 0
        # A refusal here does NOT advance `_candidate_index`/append to
        # `_refused` -- `_validate_next` retries the next alternate of this
        # SAME frontier first, and only gives up on the whole cluster once
        # every point (primary and alternates) has been tried.
        self._pending = False
        self._validate_next()

    def _send_navigation(
        self, frontier: Frontier, exploration: bool,
        target: tuple[float, float] | None = None,
    ) -> None:
        robot = self._robot_pose()
        if robot is None:
            self._state = 'waiting_map'
            self._pending = False
            return
        # `target` is the point actually commanded -- normally the frontier's
        # own (x, y), but exploration goals may carry a setback point along
        # an already-validated path instead (see `_on_path_result`). Scoring
        # and suppression radii still key off `frontier.x/y`; only the
        # commanded pose changes.
        nav_x, nav_y = target if target is not None else (frontier.x, frontier.y)
        self._last_nav_original = (frontier.x, frontier.y)
        self._last_nav_target = (nav_x, nav_y)
        # R17: this goal's departure pose, to measure the REAL displacement
        # on arrival (`_update_heading`) -- and the angle between the
        # established heading and this frontier, for telemetry
        # (`heading_delta_deg`).
        self._nav_departure_pose = (robot[0], robot[1])
        if exploration and self._current_heading is not None:
            bearing = math.atan2(
                frontier.y - robot[1], frontier.x - robot[0])
            self._heading_delta_deg = round(math.degrees(math.atan2(
                math.sin(bearing - self._current_heading),
                math.cos(bearing - self._current_heading))), 1)
        elif exploration:
            self._heading_delta_deg = None
        yaw = math.atan2(nav_y - robot[1], nav_x - robot[0])
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(nav_x, nav_y, yaw)
        if exploration:
            goal.behavior_tree = str(self.get_parameter('exploration_bt_xml').value)
        self._current = frontier
        self._pending = True
        self._barren_cycles = 0
        self._goal_started_s = self._now_s()
        # Movement watchdog: do NOT arm here. `send_goal_async` is still in
        # flight -- arm only in `_on_nav_accepted`, when Nav2 has actually
        # accepted the goal, so as not to count the action's response time
        # as a stall.
        self._state = 'navigating' if exploration else 'homing_exit'
        self._message = 'navigating to frontier' if exploration \
            else 'approaching exit marker'
        epoch = self._epoch
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done: self._on_nav_accepted(done, epoch, exploration))

    def _on_nav_accepted(self, future, epoch: int, exploration: bool) -> None:
        if epoch != self._epoch:
            return
        self._pending = False
        handle = future.result()
        if not handle.accepted:
            if exploration:
                self._blacklist_current('Nav2 refused frontier')
            else:
                self._homing_failed('Nav2 refused approach')
            return
        self._goal_handle = handle
        # Movement watchdog: arms now, on acceptance -- not on dispatch (see
        # `_send_navigation`). `None` discards the previous goal's pose, if
        # any; the first `_navigation_stalled` of this goal initializes the
        # baseline.
        self._nav_last_pose = None
        self._nav_last_progress_s = self._now_s()
        handle.get_result_async().add_done_callback(
            lambda done: self._on_nav_result(done, epoch, exploration))

    def _on_nav_result(self, future, epoch: int, exploration: bool) -> None:
        if epoch != self._epoch:
            return
        status = future.result().status
        if exploration:
            if status != GoalStatus.STATUS_SUCCEEDED:
                self._blacklist_current(f'frontier ended with status {status}')
            else:
                # R17: captured BEFORE `_release_goal`, which clears the
                # flag. A completed return does not stack a new breadcrumb
                # on the point that was just popped from the stack -- the
                # real heading is still updated, only the position record
                # changes.
                was_backtrack = self._is_backtrack_goal
                self._update_heading(push_breadcrumb=not was_backtrack)
                self._release_goal()
                # Arriving changed pose, costmap and map, which are exactly
                # the three reasons the planner rejected and the goal
                # stalled. Both provisional suppressions drop together; the
                # hard blacklist stays.
                self._refused.clear()
                self._timed_out.clear()
                self._last_provisional_map_seq = -1
                self._provisional_recovery_used = False
                self._state = 'selecting'
                self._message = (
                    'return complete; selecting again' if was_backtrack
                    else 'frontier reached; updating map')
        elif status == GoalStatus.STATUS_SUCCEEDED:
            self._release_goal()
            self._state = 'homing_exit'
            self._message = 'approach step complete'
        else:
            self._homing_failed(f'approach ended with status {status}')

    def _send_homing_step(self, blind: bool = False) -> None:
        robot = self._robot_pose()
        target = self._exit_pose_map
        if robot is None or target is None:
            return
        dx, dy = target[0] - robot[0], target[1] - robot[1]
        distance = math.hypot(dx, dy)
        stop = float(self.get_parameter('marker_stop_distance_m').value)
        # What's left may be smaller than Nav2's arrival tolerance. In that
        # case the goal would be satisfied without the robot moving, the
        # explorer would see the distance unchanged and send another one --
        # 94 s stuck at 0.75 m in R6. With less than the tolerance left,
        # arrival has already happened.
        tolerance = float(self.get_parameter('nav_goal_tolerance_m').value)
        if distance - stop <= tolerance + 1e-9:
            self._state = 'completed'
            self._message = 'marker reached; awaiting crossing confirmation'
            return
        # With a fresh marker, the short step reuses each new detection to
        # correct the aim. Blind, there's nothing to correct, and a sequence
        # of 0.5 m straight legs only gives the planner walls to refuse:
        # send a single goal and let Nav2 route around.
        step = distance - stop if blind else min(
            float(self.get_parameter('homing_step_m').value), distance - stop)
        ratio = step / distance
        frontier = Frontier(
            robot[0] + dx * ratio, robot[1] + dy * ratio, 0, 0.0)
        self._send_navigation(frontier, exploration=False)

    def _homing_failed(self, message: str) -> None:
        self._homing_failures += 1
        self._release_goal()
        if self._homing_failures >= 3:
            self._state = 'selecting'
            self._message = f'{message}; resuming exploration'
            self._homing_failures = 0
            self._exit_pose_map = None
            self._near_marker_streak = 0
        else:
            self._state = 'homing_exit'
            self._message = message

    def _note_barren_selection(self) -> None:
        """Fail a selection cycle that produces no goal repeatedly."""
        self._barren_cycles += 1
        if self._barren_cycles >= int(
                self.get_parameter('barren_selections_limit').value):
            self._fail('no safe frontier reachable')

    def _timeout_current(self, message: str) -> None:
        """Goal exceeded the ceiling: suppresses the frontier, but not forever."""
        if self._current is not None:
            self._timed_out.append((self._current.x, self._current.y))
            self._last_provisional_map_seq = self._map_seq
        self._epoch += 1
        self._cancel_goal()
        self._state = 'selecting'
        self._message = message

    def _blacklist_current(self, message: str) -> None:
        if self._current is not None:
            self._blacklist.append((self._current.x, self._current.y))
        self._epoch += 1
        self._cancel_goal()
        self._state = 'selecting'
        self._message = message

    def _cancel_goal(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        if self._recovery_handle is not None:
            self._recovery_handle.cancel_goal_async()
            self._recovery_handle = None
            self._recovery_pending = False
        self._release_goal()

    def _release_goal(self) -> None:
        self._goal_handle = None
        self._pending = False
        self._current = None
        # R17: single exit point for every goal (success, blacklist, timeout
        # and cancellation all pass through here) -- guarantees the flag
        # never leaks from a return goal into the next normal goal.
        self._is_backtrack_goal = False

    def _update_heading(self, push_breadcrumb: bool) -> None:
        """
        Record the real heading (not the final yaw) and, if asked, a breadcrumb.

        R17: heading = direction of displacement since this goal was
        dispatched (`_nav_departure_pose`), not the robot's final
        orientation -- a robot that arrives sideways or turned was still
        heading in THAT direction. Segments that are too short (localization
        noise, not real displacement) do not update the heading, so as not
        to let an arrival that is almost in place redefine "forward" at
        random.
        """
        robot = self._robot_pose()
        if robot is None or self._nav_departure_pose is None:
            return
        dx = robot[0] - self._nav_departure_pose[0]
        dy = robot[1] - self._nav_departure_pose[1]
        if math.hypot(dx, dy) >= 0.05:
            self._current_heading = math.atan2(dy, dx)
        if push_breadcrumb:
            # Save the start of the segment travelled. Using the arrival pose
            # would create a first "return" to the position the robot is
            # already at, spending time before actually backing off.
            self._push_breadcrumb(self._nav_departure_pose)

    def _push_breadcrumb(self, pose: tuple[float, float]) -> None:
        spacing = float(self.get_parameter('breadcrumb_min_spacing_m').value)
        if self._breadcrumbs and math.hypot(
                pose[0] - self._breadcrumbs[-1][0],
                pose[1] - self._breadcrumbs[-1][1]) < spacing:
            return
        self._breadcrumbs.append(pose)

    def _fail(self, message: str) -> None:
        self._epoch += 1
        self._cancel_goal()
        self._state = 'failed'
        self._message = message
        self._publish_status()

    def _robot_pose(self) -> tuple[float, float, float] | None:
        try:
            transform = self._tf_buffer.lookup_transform('map', 'base', Time())
        except TransformException:
            return None
        translation, q = transform.transform.translation, transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return translation.x, translation.y, yaw

    def _robot_tilt_deg(self) -> float | None:
        """Return the body's tilt from the latest ``map -> base`` TF."""
        try:
            transform = self._tf_buffer.lookup_transform('map', 'base', Time())
        except TransformException:
            return None
        q = transform.transform.rotation
        # Angle between the body's +z axis and the world's +z axis.  Roll and
        # pitch are intentionally combined: either direction can precede a
        # fall, while yaw must not affect this safety check.
        cos_tilt = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        cos_tilt = max(-1.0, min(1.0, cos_tilt))
        return math.degrees(math.acos(cos_tilt))

    def _pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_status(self) -> None:
        assert self._state in STATES
        now = self._now_s()
        elapsed = 0.0 if not self._started_s else now - self._started_s
        goal = None if self._current is None else asdict(self._current)
        payload = {
            'state': self._state,
            'elapsed_s': round(elapsed, 1),
            'frontier_count': self._frontier_count,
            'goal': goal,
            'blacklisted': len(self._blacklist),
            'refused': len(self._refused),
            'timed_out': len(self._timed_out),
            # Search cost, for the operator and for the CPU gate. These five
            # fields are additive: the cockpit ignores what it doesn't know.
            'frontier_extract_ms': self._frontier_extract_ms,
            'frontier_cells': self._frontier_cells,
            'frontier_clusters': self._frontier_clusters,
            # Raw cluster count, before the clearance/standoff candidate
            # search. Distinguishes "only one cluster ever existed" from
            # "several existed and the candidate filters ate the rest" --
            # the two look identical in `frontier_clusters` alone.
            'frontier_clusters_raw': self._frontier_clusters_raw,
            'candidates_checked': self._candidate_index,
            'path_requests': self._path_requests,
            'selection_cycle': self._selection_cycle,
            'near_frontiers_skipped': self._near_skipped,
            # The most recent ComputePathToPose attempt: which point, and
            # exactly how the planner answered. Without this, a refusal like
            # R10's (29/08) can only be explained by reconstructing it
            # offline against the frozen map after the fact.
            'candidate_point_x': (
                None if self._last_candidate_point is None
                else round(self._last_candidate_point[0], 3)),
            'candidate_point_y': (
                None if self._last_candidate_point is None
                else round(self._last_candidate_point[1], 3)),
            'last_path_status': self._last_path_status,
            'last_path_error_code': self._last_path_error_code,
            'last_path_error_msg': self._last_path_error_msg,
            'last_path_planner_id': self._last_path_planner_id,
            # Original frontier endpoint (scoring/information-gain) vs. the
            # point actually commanded to Nav2 -- differ only when a setback
            # point along an already-validated plan was used instead.
            'nav_original_x': (
                None if self._last_nav_original is None
                else round(self._last_nav_original[0], 3)),
            'nav_original_y': (
                None if self._last_nav_original is None
                else round(self._last_nav_original[1], 3)),
            'nav_target_x': (
                None if self._last_nav_target is None
                else round(self._last_nav_target[0], 3)),
            'nav_target_y': (
                None if self._last_nav_target is None
                else round(self._last_nav_target[1], 3)),
            'provisional_recoveries': self._provisional_recoveries,
            'barren_cycles': self._barren_cycles,
            'recovery_attempts': self._recovery_attempts,
            'marker_visible': (
                self._exit_candidate_pose_map is not None
                and now - self._exit_seen_s <= float(
                    self.get_parameter('marker_stale_s').value)
            ),
            # The raw candidate and the accepted/latching homing target are
            # separate so a partial view cannot silently move the target.
            'marker_distance_m': self._marker_distance_m,
            'marker_observations': self._marker_observations,
            'marker_confirmations': self._near_marker_streak,
            'marker_candidate_x': (
                None if self._exit_candidate_pose_map is None
                else round(self._exit_candidate_pose_map[0], 3)),
            'marker_candidate_y': (
                None if self._exit_candidate_pose_map is None
                else round(self._exit_candidate_pose_map[1], 3)),
            'marker_accepted_x': (
                None if self._exit_pose_map is None
                else round(self._exit_pose_map[0], 3)),
            'marker_accepted_y': (
                None if self._exit_pose_map is None
                else round(self._exit_pose_map[1], 3)),
            'homing_entry_distance_m': self._homing_entry_distance_m,
            'homing_entries': self._homing_entries,
            'homing_abandons': self._homing_abandons,
            'marker_far_ignored': self._marker_far_ignored,
            # R17: directional exploration with breadcrumb backtracking.
            # `decision_mode` is `None` until the first selection with
            # candidates; the cockpit should treat that as "still
            # selecting", not as a fourth mode.
            'decision_mode': self._decision_mode,
            'breadcrumbs': len(self._breadcrumbs),
            'backtrack_attempts': self._backtrack_attempts,
            'heading_delta_deg': self._heading_delta_deg,
            'forward_candidates': self._forward_candidates_count,
            'reverse_candidates': self._reverse_candidates_count,
            'message': self._message,
        }
        self._status_pub.publish(String(
            data=json.dumps(payload, separators=(',', ':'))))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MazeExplorer()
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
