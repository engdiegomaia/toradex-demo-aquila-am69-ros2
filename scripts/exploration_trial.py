#!/usr/bin/env python3
"""
Passive recorder for one autonomous maze-exit run.

Runs on the x86 host, inside the `tools` container, against a stack that is
already up (`ROS_DOMAIN_ID=69`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`):

    docker compose -f docker/compose.host.yml --profile tools up -d tools
    docker compose -f docker/compose.host.yml exec -T tools \
        /usr/local/bin/entrypoint.sh python3 - out.csv --seconds 660 \
        < scripts/exploration_trial.py

WHY THIS SCRIPT EXISTS, AND WHY IT IS NOT `nav_trial.py`

`nav_trial.py` DRIVES: it owns the goal list and sends `navigate_to_pose`
itself. That is the right instrument when the question is "does the robot cross
faster", because the route is the independent variable.

Here the route is the DEPENDENT variable. `maze_explorer` chooses its own
frontiers, and the point of the run is to find out which ones it picks and why.
A recorder that sent goals would be measuring itself. This script therefore
publishes NOTHING and calls no action server. It only listens.

WHAT IT RECORDS, AND WHY EACH FIELD IS HERE

- **`/demo/exploration/status` verbatim, expanded into columns.** The explorer
  already publishes frontier cost, cluster counts, candidate index and
  selection cycle. Re-deriving them here would measure a different program.
- **Map coverage from `/map`, not from the costmap.** The costmap inflates and
  rolls; `/map` is the grid the frontier search actually reads, so coverage and
  frontier count have to come from the same grid or they cannot be compared.
  Both the known-cell COUNT and the percentage are recorded: slam_toolbox grows
  the grid as it maps, so the percentage can fall while mapping progresses.
- **`/demo/cmd_vel_si`, not `/demo/cmd_vel`.** SI at the collision-monitor
  output. `/demo/cmd_vel` carries stick position and is not comparable across
  configurations that change `vx_max`.
- **Falls by odometry `z` and body tilt, every row.** Any number measured after
  the robot fell describes a body being dragged.
- **Goal transitions reconstructed from status, with the explorer's own
  message.** `_on_nav_result` folds the Nav2 `GoalStatus` into the status
  message; that string is the only place the per-goal outcome is published, so
  the goals CSV parses it rather than inventing a second source of truth.
- **First detection and homing start as timestamps, not booleans.** "Perception
  fired" is not a result; WHEN it fired relative to the goal sequence is.

The recorder measures the simulated plant on the host and the Nav2/perception
stack on the Aquila. Those numbers describe that distributed stack. They do not
validate leg odometry, a physical Go2, thermals, or isolated module performance
(`CLAUDE.md` rules 5 and 7).

ROS IMPORTS ARE DEFERRED INTO `build_recorder()` ON PURPOSE. The pure helpers
below are unit-tested without a sourced Jazzy environment, exactly as
`tf_lidar_probe.py` does it.
"""

import argparse
import csv
import json
import math
import re
import sys
import time


# The explorer folds the Nav2 GoalStatus into its status message. Parsing a
# published string is deliberate coupling, so it is isolated to one place and a
# miss degrades to a recorded 'unknown' rather than a wrong number.
_STATUS_CODE = re.compile(r'status (\d+)')

# nav2_msgs GoalStatus.STATUS_SUCCEEDED. The explorer only prints a numeric
# code when the goal did NOT succeed.
GOAL_STATUS_SUCCEEDED = 4

# Success is WHITELISTED, not inferred from "no error code in the message".
#
# The explorer reports several failures that carry no numeric status at all --
# `meta de fronteira expirou` is the one that mattered: a goal killed by the
# explorer's own 90 s timeout was being scored as a completed goal, which is
# the acceptance number pointing the wrong way. Anything unrecognised is
# therefore counted as a failure, because a metric that has to be wrong should
# be wrong pessimistically.
SUCCESS_MESSAGES = (
    'fronteira alcancada',
    'passo de aproximacao concluido',
    'marcador alcancado',
)

ROW_FIELDS = [
    'sim_s', 'wall_s', 'state', 'x', 'y', 'z', 'yaw_deg', 'tilt_deg',
    'cmd_vx', 'cmd_wz', 'path_m', 'map_known_pct', 'map_known_cells',
    'frontier_count', 'frontier_cells', 'frontier_clusters',
    'frontier_extract_ms', 'candidates_checked', 'path_requests',
    'selection_cycle', 'blacklisted', 'marker_visible', 'detections_n',
    'exit_pose_seen', 'escaped', 'message',
]

GOAL_FIELDS = [
    'goal_index', 'goal_x', 'goal_y', 'phase', 'sent_sim_s', 'elapsed_s',
    'outcome', 'nav2_status', 'message',
]


# --- pure helpers ----------------------------------------------------------

def quat_to_yaw_tilt(qx: float, qy: float, qz: float, qw: float) -> tuple:
    """Return ``yaw_deg, tilt_deg`` for an odometry orientation."""
    yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                     1.0 - 2.0 * (qy * qy + qz * qz))
    # Tilt is the angle between body +z and world +z. Sign is irrelevant: a
    # fall is a fall in either direction.
    cos_tilt = 1.0 - 2.0 * (qx * qx + qy * qy)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, cos_tilt))))
    return math.degrees(yaw), tilt


def parse_goal_outcome(message: str) -> tuple:
    """Return ``outcome, nav2_status`` for the explorer's closing message.

    The explorer prints ``... status N`` only on a goal that Nav2 ended, so a
    numeric code is a Nav2 failure. An empty message means the goal was still
    open when the run ended, which is neither a success nor a failure and must
    not be counted as either. Everything else is scored against
    ``SUCCESS_MESSAGES``; see the note there for why the default is failure.
    """
    if not message:
        return 'open', ''
    match = _STATUS_CODE.search(message)
    if match:
        return 'failed', int(match.group(1))
    if any(marker in message for marker in SUCCESS_MESSAGES):
        return 'ok', GOAL_STATUS_SUCCEEDED
    return 'failed', ''


def percentile(values: list, fraction: float) -> float:
    """Nearest-rank percentile; avoids a numpy dependency in the tools image."""
    if not values:
        return float('nan')
    ordered = sorted(values)
    index = min(len(ordered) - 1,
                max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def _floats(rows: list, key: str) -> list:
    return [float(row[key]) for row in rows if row.get(key) not in ('', None)]


def summarise(rows: list, goals: list, marks: dict) -> dict:
    """Reduce the recorded rows to the acceptance numbers of the F5 protocol."""
    if not rows:
        return {'samples': 0}
    extract = _floats(rows, 'frontier_extract_ms')
    vx = [abs(value) for value in _floats(rows, 'cmd_vx')]
    tilt = _floats(rows, 'tilt_deg')
    height = _floats(rows, 'z')
    known = _floats(rows, 'map_known_cells')
    sim_span = rows[-1]['sim_s'] - rows[0]['sim_s']
    wall_span = rows[-1]['wall_s'] - rows[0]['wall_s']
    return {
        'samples': len(rows),
        'sim_span_s': round(sim_span, 1),
        'wall_span_s': round(wall_span, 1),
        'rtf': round(sim_span / wall_span, 3) if wall_span else None,
        'escaped': bool(marks.get('escaped')),
        'escaped_sim_s': marks.get('escaped_sim_s'),
        'final_state': rows[-1]['state'],
        'final_message': rows[-1]['message'],
        'path_m': round(float(rows[-1]['path_m'] or 0.0), 2),
        'map_known_cells_first': known[0] if known else None,
        'map_known_cells_last': known[-1] if known else None,
        'frontier_extract_ms_p50': round(percentile(extract, 0.50), 2),
        'frontier_extract_ms_p95': round(percentile(extract, 0.95), 2),
        'frontier_extract_ms_max': round(max(extract), 2) if extract else None,
        'vx_work_ratio': round(
            sum(1 for value in vx if value > 0.01) / len(vx), 4) if vx else None,
        'vx_mean_abs': round(sum(vx) / len(vx), 4) if vx else None,
        'tilt_max_deg': round(max(tilt), 2) if tilt else None,
        'z_min_m': round(min(height), 4) if height else None,
        'goals_total': len(goals),
        'goals_ok': sum(1 for goal in goals if goal['outcome'] == 'ok'),
        'goals_failed': sum(1 for goal in goals if goal['outcome'] == 'failed'),
        'goals_homing': sum(1 for goal in goals if goal['phase'] == 'homing'),
        'blacklisted_final': rows[-1]['blacklisted'],
        'first_detection_sim_s': marks.get('first_detection_sim_s'),
        'first_exit_pose_sim_s': marks.get('first_exit_pose_sim_s'),
        'exit_pose_frame': marks.get('exit_pose_frame', ''),
        'homing_started_sim_s': marks.get('homing_started_sim_s'),
    }


def write_csv(path: str, fieldnames: list, rows: list) -> None:
    with open(path, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def goals_csv_path(out_csv: str) -> str:
    """Companion path for the per-goal file, matching the F5 evidence layout."""
    if out_csv.endswith('.csv'):
        return f'{out_csv[:-4]}-goals.csv'
    return f'{out_csv}-goals.csv'


# --- ROS recorder ----------------------------------------------------------

def build_recorder(period_s: float):
    """Import ROS and return an ``ExplorationRecorder`` instance.

    Imports live here so the pure helpers above can be tested without a sourced
    Jazzy environment.
    """
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, QoSPresetProfiles, QoSProfile,
                           ReliabilityPolicy)
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Bool, String
    from vision_msgs.msg import Detection2DArray

    class ExplorationRecorder(Node):
        """One row per sample, one row per explorer goal. Publishes nothing."""

        def __init__(self) -> None:
            super().__init__('exploration_trial')
            latched = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            sensor = QoSPresetProfiles.SENSOR_DATA.value

            self.status: dict = {}
            self.odom = None
            self.cmd = None
            self.escaped = False
            self.detections_n = 0
            self.map_known = 0
            self.map_total = 0
            self.exit_pose_frame = ''
            self.exit_pose = None

            self.sim_s = 0.0
            self.first_detection_sim_s = None
            self.first_exit_pose_sim_s = None
            self.homing_started_sim_s = None
            self.escaped_sim_s = None
            self.path_m = 0.0
            self._last_xy = None

            self.rows: list = []
            self.goals: list = []
            self._open_goal = None
            self._goal_index = 0

            self.create_subscription(Clock, '/clock', self._on_clock, sensor)
            self.create_subscription(
                String, '/demo/exploration/status', self._on_status, latched)
            self.create_subscription(Odometry, '/demo/odom', self._on_odom, 20)
            self.create_subscription(Twist, '/demo/cmd_vel_si', self._on_cmd, 20)
            self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
            self.create_subscription(
                Detection2DArray, '/demo/perception/maze_exit/detections',
                self._on_detections, 10)
            self.create_subscription(
                PoseStamped, '/demo/perception/maze_exit/pose',
                self._on_exit_pose, 10)
            self.create_subscription(
                Bool, '/demo/maze/escaped', self._on_escaped, latched)

            self._wall0 = time.monotonic()
            self.create_timer(period_s, self.sample)

        # --- subscriptions -------------------------------------------------

        def _on_clock(self, message) -> None:
            self.sim_s = message.clock.sec + message.clock.nanosec / 1e9

        def _on_odom(self, message) -> None:
            self.odom = message
            position = message.pose.pose.position
            current = (position.x, position.y)
            if self._last_xy is not None:
                self.path_m += math.dist(self._last_xy, current)
            self._last_xy = current

        def _on_cmd(self, message) -> None:
            self.cmd = message

        def _on_map(self, message) -> None:
            self.map_total = len(message.data)
            self.map_known = sum(1 for value in message.data if value >= 0)

        def _on_detections(self, message) -> None:
            self.detections_n = len(message.detections)
            if message.detections and self.first_detection_sim_s is None:
                self.first_detection_sim_s = self.sim_s

        def _on_exit_pose(self, message) -> None:
            self.exit_pose = message
            self.exit_pose_frame = message.header.frame_id
            if self.first_exit_pose_sim_s is None:
                self.first_exit_pose_sim_s = self.sim_s

        def _on_escaped(self, message) -> None:
            if message.data and not self.escaped:
                self.escaped_sim_s = self.sim_s
            self.escaped = message.data

        def _on_status(self, message) -> None:
            try:
                self.status = json.loads(message.data)
            except json.JSONDecodeError:
                # A malformed status must not end the run; the raw string is
                # preserved so the CSV shows what actually arrived.
                self.status = {'state': 'unparsed',
                               'message': message.data[:200]}
                return
            self._track_goal()

        # --- goal bookkeeping ----------------------------------------------

        def _track_goal(self) -> None:
            """Turn the status stream into one row per attempted goal."""
            state = self.status.get('state', '')
            if state == 'homing_exit' and self.homing_started_sim_s is None:
                self.homing_started_sim_s = self.sim_s

            goal = self.status.get('goal')
            key = None if goal is None else (
                round(float(goal.get('x', 0.0)), 3),
                round(float(goal.get('y', 0.0)), 3))
            open_key = None if self._open_goal is None else (
                self._open_goal['goal_x'], self._open_goal['goal_y'])
            if key == open_key:
                return

            if self._open_goal is not None:
                self.close_goal()
            if key is not None:
                self._open_goal = {
                    'goal_index': self._goal_index,
                    'goal_x': key[0],
                    'goal_y': key[1],
                    'phase': ('homing' if state == 'homing_exit'
                              else 'exploration'),
                    'sent_sim_s': round(self.sim_s, 3),
                }
                self._goal_index += 1

        def close_goal(self) -> None:
            record = self._open_goal
            self._open_goal = None
            if record is None:
                return
            message = str(self.status.get('message', ''))
            outcome, nav2_status = parse_goal_outcome(message)
            record.update({
                'elapsed_s': round(self.sim_s - record['sent_sim_s'], 3),
                'outcome': outcome,
                'nav2_status': nav2_status,
                'message': message,
            })
            self.goals.append(record)

        # --- sampling --------------------------------------------------------

        def sample(self) -> None:
            row = {name: '' for name in ROW_FIELDS}
            row['sim_s'] = round(self.sim_s, 3)
            row['wall_s'] = round(time.monotonic() - self._wall0, 3)
            row['state'] = self.status.get('state', '')
            row['message'] = str(self.status.get('message', ''))[:120]

            if self.odom is not None:
                position = self.odom.pose.pose.position
                orientation = self.odom.pose.pose.orientation
                yaw, tilt = quat_to_yaw_tilt(
                    orientation.x, orientation.y, orientation.z, orientation.w)
                row.update({
                    'x': round(position.x, 4), 'y': round(position.y, 4),
                    'z': round(position.z, 4), 'yaw_deg': round(yaw, 2),
                    'tilt_deg': round(tilt, 2),
                })
            if self.cmd is not None:
                row['cmd_vx'] = round(self.cmd.linear.x, 4)
                row['cmd_wz'] = round(self.cmd.angular.z, 4)

            row['path_m'] = round(self.path_m, 3)
            if self.map_total:
                row['map_known_cells'] = self.map_known
                row['map_known_pct'] = round(
                    100.0 * self.map_known / self.map_total, 2)

            for name in ('frontier_count', 'frontier_cells',
                         'frontier_clusters', 'frontier_extract_ms',
                         'candidates_checked', 'path_requests',
                         'selection_cycle', 'blacklisted', 'marker_visible'):
                if name in self.status:
                    row[name] = self.status[name]

            row['detections_n'] = self.detections_n
            row['exit_pose_seen'] = int(self.exit_pose is not None)
            row['escaped'] = int(self.escaped)
            self.rows.append(row)

        def marks(self) -> dict:
            return {
                'escaped': self.escaped,
                'escaped_sim_s': self.escaped_sim_s,
                'first_detection_sim_s': self.first_detection_sim_s,
                'first_exit_pose_sim_s': self.first_exit_pose_sim_s,
                'exit_pose_frame': self.exit_pose_frame,
                'homing_started_sim_s': self.homing_started_sim_s,
            }

    return ExplorationRecorder()


def main(argv: list | None = None) -> int:
    parser = argparse.ArgumentParser(description='Record one exploration run.')
    parser.add_argument('out_csv')
    parser.add_argument('--seconds', type=float, default=660.0,
                        help='wall-clock ceiling for the recording')
    parser.add_argument('--hz', type=float, default=2.0,
                        help='sample rate of the per-row CSV')
    parser.add_argument('--stop-on-escape', action='store_true',
                        help='end as soon as /demo/maze/escaped latches true')
    arguments = parser.parse_args(argv)

    import rclpy

    rclpy.init()
    node = build_recorder(1.0 / arguments.hz)
    deadline = time.monotonic() + arguments.seconds
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if arguments.stop_on_escape and node.escaped:
                node.sample()  # one more row, so the escape is in the CSV
                break
    except KeyboardInterrupt:
        pass

    node.close_goal()
    write_csv(arguments.out_csv, ROW_FIELDS, node.rows)
    goals_path = goals_csv_path(arguments.out_csv)
    write_csv(goals_path, GOAL_FIELDS, node.goals)

    print(json.dumps(summarise(node.rows, node.goals, node.marks()),
                     indent=2, default=str))
    print(f'rows  -> {arguments.out_csv}', file=sys.stderr)
    print(f'goals -> {goals_path}', file=sys.stderr)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
