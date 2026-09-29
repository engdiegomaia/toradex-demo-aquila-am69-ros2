"""Unit tests for the pure helpers of tools/evaluation/exploration_trial.py.

These run WITHOUT ROS. That is the reason `exploration_trial` imports `rclpy`
and the message packages inside `build_recorder()` instead of at module scope —
importing the module here must not require a sourced Jazzy environment.
"""

from __future__ import annotations

import csv
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / 'tools' / 'evaluation'
sys.path.insert(0, str(SCRIPTS))

from exploration_trial import (  # noqa: E402
    GOAL_FIELDS,
    GOAL_STATUS_SUCCEEDED,
    OccupancyGridInfo,
    ROW_FIELDS,
    classify_stop_reason,
    find_stalled_navigating_windows,
    goals_csv_path,
    lateral_wall_clearance_m,
    occupancy_grid_to_dict,
    parse_goal_outcome,
    path_metrics,
    percentile,
    pose_entered_region,
    quat_to_yaw_tilt,
    snapshot_json_path,
    summarise,
    write_csv,
)


def _row(**overrides) -> dict:
    row = {name: '' for name in ROW_FIELDS}
    row.update({
        'sim_s': 0.0, 'wall_s': 0.0, 'state': 'navigating',
        'z': 0.35, 'tilt_deg': 0.5, 'cmd_vx': 0.0, 'path_m': 0.0,
        'map_known_cells': 100, 'blacklisted': 0, 'message': '',
    })
    row.update(overrides)
    return row


def _goal(**overrides) -> dict:
    goal = {
        'goal_index': 0, 'goal_x': 1.0, 'goal_y': 2.0,
        'phase': 'exploration', 'sent_sim_s': 0.0, 'elapsed_s': 1.0,
        'outcome': 'ok', 'nav2_status': GOAL_STATUS_SUCCEEDED, 'message': '',
    }
    goal.update(overrides)
    return goal


# --- orientation -----------------------------------------------------------

def test_quat_to_yaw_tilt_reads_level_heading() -> None:
    # 90 degrees of yaw, no roll or pitch: the spawn attitude of the maze runs.
    yaw, tilt = quat_to_yaw_tilt(0.0, 0.0, 0.7071068, 0.7071068)

    assert yaw == pytest.approx(90.0, abs=1e-3)
    assert tilt == pytest.approx(0.0, abs=1e-3)


def test_quat_to_yaw_tilt_reports_a_fall_as_large_tilt() -> None:
    # 90 degrees of roll: the body +z axis is horizontal.
    _, tilt = quat_to_yaw_tilt(0.7071068, 0.0, 0.0, 0.7071068)

    assert tilt == pytest.approx(90.0, abs=1e-3)


def test_quat_to_yaw_tilt_is_unsigned_in_tilt() -> None:
    """A fall to either side has to read as a fall, not cancel out."""
    _, left = quat_to_yaw_tilt(0.3826834, 0.0, 0.0, 0.9238795)
    _, right = quat_to_yaw_tilt(-0.3826834, 0.0, 0.0, 0.9238795)

    assert left == pytest.approx(right, abs=1e-6)
    assert left == pytest.approx(45.0, abs=1e-3)


# --- goal outcome parsing --------------------------------------------------

def test_parse_goal_outcome_reads_the_explorers_failure_code() -> None:
    outcome, status = parse_goal_outcome('fronteira terminou com status 6')

    assert outcome == 'failed'
    assert status == 6


def test_parse_goal_outcome_accepts_the_explorers_success_message() -> None:
    outcome, status = parse_goal_outcome('fronteira alcancada; atualizando mapa')

    assert outcome == 'ok'
    assert status == GOAL_STATUS_SUCCEEDED


def test_parse_goal_outcome_scores_the_explorer_timeout_as_a_failure() -> None:
    """`meta de fronteira expirou` carries no numeric code but is a failure.

    Inferring success from "no error code present" scored a goal killed by the
    explorer's own 90 s timeout as a completed goal.
    """
    outcome, status = parse_goal_outcome('meta de fronteira expirou')

    assert outcome == 'failed'
    assert status == ''


@pytest.mark.parametrize('message', [
    'Nav2 recusou fronteira',
    'planner rejeitou todas as fronteiras',
    'nenhuma fronteira segura alcancavel',
])
def test_parse_goal_outcome_defaults_unknown_messages_to_failure(message) -> None:
    """An acceptance metric that must be wrong should be wrong pessimistically."""
    outcome, _ = parse_goal_outcome(message)

    assert outcome == 'failed'


def test_parse_goal_outcome_does_not_score_a_goal_that_never_closed() -> None:
    """A goal still open when the run ends is neither a success nor a failure.

    Counting it either way would move the acceptance number by one goal for a
    result the run never observed.
    """
    outcome, status = parse_goal_outcome('')

    assert outcome == 'open'
    assert status == ''


# --- percentile ------------------------------------------------------------

def test_percentile_uses_nearest_rank() -> None:
    values = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]

    assert percentile(values, 0.50) == 5.0
    assert percentile(values, 0.95) == 10.0


def test_percentile_of_nothing_is_nan_not_zero() -> None:
    """Zero would read as "extraction was instant" on a run that never ran."""
    assert percentile([], 0.95) != percentile([], 0.95)  # NaN != NaN


# --- summary ---------------------------------------------------------------

def test_summarise_reports_the_escape_and_its_timestamp() -> None:
    rows = [_row(sim_s=0.0, wall_s=0.0), _row(sim_s=120.0, wall_s=124.0)]
    marks = {'escaped': True, 'escaped_sim_s': 118.4}

    summary = summarise(rows, [], marks)

    assert summary['escaped'] is True
    assert summary['escaped_sim_s'] == 118.4
    assert summary['sim_span_s'] == 120.0
    assert summary['rtf'] == pytest.approx(0.968, abs=1e-3)


def test_summarise_counts_forward_work_not_peak_speed() -> None:
    rows = [
        _row(sim_s=0.0, wall_s=0.0, cmd_vx=0.0, state='navigating'),
        _row(sim_s=1.0, wall_s=1.0, cmd_vx=0.10, state='navigating'),
        _row(sim_s=2.0, wall_s=2.0, cmd_vx=0.0, state='navigating'),
        _row(sim_s=3.0, wall_s=3.0, cmd_vx=0.10, state='navigating'),
    ]

    summary = summarise(rows, [], {})

    # No terminal state: the active window is the whole recording, so the
    # two views agree.
    assert summary['recording_vx_work_ratio'] == 0.5
    assert summary['recording_vx_mean_abs'] == pytest.approx(0.05)
    assert summary['active_vx_work_ratio'] == 0.5
    assert summary['active_vx_mean_abs'] == pytest.approx(0.05)
    assert summary['active_duration_s'] == 3.0


def test_summarise_excludes_post_failure_samples_from_active_metrics() -> None:
    """The R8 defect: a run that fails early keeps recording zeros.

    The explorer fails at sim_s=2 with real work happening at cmd_vx=0.10
    before that. 600 s of post-failure zeros must not dilute `active_*`,
    only `recording_*`.
    """
    rows = [
        _row(sim_s=0.0, wall_s=0.0, cmd_vx=0.10, state='navigating'),
        _row(sim_s=1.0, wall_s=1.0, cmd_vx=0.10, state='navigating'),
        _row(sim_s=2.0, wall_s=2.0, cmd_vx=0.0, state='failed'),
        _row(sim_s=602.0, wall_s=602.0, cmd_vx=0.0, state='failed'),
        _row(sim_s=1202.0, wall_s=1202.0, cmd_vx=0.0, state='failed'),
    ]

    summary = summarise(rows, [], {})

    # The active window is inclusive of the terminal sample itself (the
    # failure at sim_s=2), so its trailing zero counts once, not 600s worth.
    assert summary['active_vx_work_ratio'] == pytest.approx(2 / 3, abs=1e-4)
    assert summary['active_vx_mean_abs'] == pytest.approx(0.0667, abs=1e-3)
    assert summary['active_duration_s'] == 2.0
    # The full-window view is still dominated by the post-failure zeros --
    # that is expected, and exactly why it must not be read alone.
    assert summary['recording_vx_work_ratio'] == pytest.approx(0.4)
    assert summary['recording_vx_mean_abs'] < summary['active_vx_mean_abs']


def test_summarise_active_window_stops_at_first_terminal_sample() -> None:
    """A second, later terminal-looking sample must not extend the window."""
    rows = [
        _row(sim_s=0.0, state='waiting_map', cmd_vx=0.0),
        _row(sim_s=1.0, state='selecting', cmd_vx=0.0),
        _row(sim_s=2.0, state='navigating', cmd_vx=0.20),
        _row(sim_s=3.0, state='completed', cmd_vx=0.0),
        _row(sim_s=4.0, state='completed', cmd_vx=0.0),
    ]

    summary = summarise(rows, [], {})

    assert summary['active_duration_s'] == 3.0
    assert summary['state_durations_s']['navigating'] == 1.0


def test_summarise_of_a_run_with_no_active_state_does_not_break() -> None:
    """A recording that never left `idle` has no active window to report."""
    rows = [
        _row(sim_s=0.0, wall_s=0.0, state='idle', cmd_vx=0.0),
        _row(sim_s=1.0, wall_s=1.0, state='idle', cmd_vx=0.0),
    ]

    summary = summarise(rows, [], {})

    assert summary['active_vx_work_ratio'] is None
    assert summary['active_vx_mean_abs'] is None
    assert summary['active_duration_s'] is None
    assert summary['recording_vx_work_ratio'] == 0.0


def test_summarise_reports_state_durations_for_the_full_recording() -> None:
    rows = [
        _row(sim_s=0.0, state='waiting_map'),
        _row(sim_s=1.5, state='navigating'),
        _row(sim_s=4.5, state='navigating'),
        _row(sim_s=5.0, state='completed'),
    ]

    summary = summarise(rows, [], {})

    assert summary['state_durations_s'] == {
        'waiting_map': 1.5, 'navigating': 3.5,
    }


def test_summarise_separates_homing_goals_from_exploration_goals() -> None:
    goals = [
        _goal(goal_index=0, phase='exploration', outcome='ok'),
        _goal(goal_index=1, phase='exploration', outcome='failed'),
        _goal(goal_index=2, phase='homing', outcome='ok'),
    ]

    summary = summarise([_row()], goals, {})

    assert summary['goals_total'] == 3
    assert summary['goals_ok'] == 2
    assert summary['goals_failed'] == 1
    assert summary['goals_homing'] == 1


def test_summarise_tracks_known_cells_because_the_grid_grows() -> None:
    """slam_toolbox grows the grid, so a percentage can fall while mapping."""
    rows = [
        _row(sim_s=0.0, wall_s=0.0, map_known_cells=100),
        _row(sim_s=10.0, wall_s=10.0, map_known_cells=900),
    ]

    summary = summarise(rows, [], {})

    assert summary['map_known_cells_first'] == 100
    assert summary['map_known_cells_last'] == 900


def test_summarise_of_an_empty_run_does_not_invent_numbers() -> None:
    assert summarise([], [], {}) == {'samples': 0}


def test_summarise_reports_the_worst_tilt_and_lowest_body_height() -> None:
    rows = [
        _row(sim_s=0.0, wall_s=0.0, tilt_deg=0.4, z=0.35),
        _row(sim_s=1.0, wall_s=1.0, tilt_deg=61.0, z=0.08),
    ]

    summary = summarise(rows, [], {})

    assert summary['tilt_max_deg'] == 61.0
    assert summary['z_min_m'] == 0.08


# --- output layout ---------------------------------------------------------

def test_goals_csv_path_sits_next_to_the_row_csv() -> None:
    assert goals_csv_path('docs/results/run1.csv') == 'docs/results/run1-goals.csv'


def test_goals_csv_path_handles_a_name_without_the_extension() -> None:
    assert goals_csv_path('run1') == 'run1-goals.csv'


def test_write_csv_emits_the_declared_header_even_with_no_rows(tmp_path) -> None:
    """An empty run must still produce a parseable file, not a zero-byte one."""
    target = tmp_path / 'empty.csv'

    write_csv(str(target), GOAL_FIELDS, [])

    with open(target, newline='') as handle:
        assert next(csv.reader(handle)) == GOAL_FIELDS


# --- R13 snapshot capture ----------------------------------------------------

def test_snapshot_json_path_sits_next_to_the_row_csv() -> None:
    assert snapshot_json_path('out.csv') == 'out-map-snapshot.json'


def test_snapshot_json_path_handles_a_name_without_the_extension() -> None:
    assert snapshot_json_path('out') == 'out-map-snapshot.json'


def test_pose_entered_region_requires_both_bounds() -> None:
    assert pose_entered_region(-5.0, 1.0, -4.5, 2.0) is True
    assert pose_entered_region(-4.0, 1.0, -4.5, 2.0) is False  # x not past
    assert pose_entered_region(-5.0, 3.0, -4.5, 2.0) is False  # y not past


def test_pose_entered_region_is_false_exactly_on_the_boundary() -> None:
    """A pose sitting on the boundary has not yet arrived (see nav-goal-tolerance-trap)."""
    assert pose_entered_region(-4.5, 1.0, -4.5, 2.0) is False
    assert pose_entered_region(-5.0, 2.0, -4.5, 2.0) is False


def _fake_occupancy_grid(**overrides) -> SimpleNamespace:
    grid = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=123, nanosec=456), frame_id='map'),
        info=SimpleNamespace(
            resolution=0.05, width=3, height=2,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=-1.0, y=-2.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))),
        data=[-1, 0, 100, -1, 50, 0],
    )
    for key, value in overrides.items():
        setattr(grid, key, value)
    return grid


def test_occupancy_grid_to_dict_is_lossless() -> None:
    """R13 needs the exact grid `extract_frontiers` saw, not a summary of it."""
    payload = occupancy_grid_to_dict(_fake_occupancy_grid())

    assert payload['header'] == {
        'stamp_sec': 123, 'stamp_nanosec': 456, 'frame_id': 'map'}
    assert payload['info']['width'] == 3
    assert payload['info']['height'] == 2
    assert payload['info']['resolution'] == 0.05
    assert payload['info']['origin'] == {
        'x': -1.0, 'y': -2.0, 'z': 0.0,
        'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}
    assert payload['data'] == [-1, 0, 100, -1, 50, 0]


def test_occupancy_grid_to_dict_data_is_a_plain_list() -> None:
    """`json.dumps` must not choke on the ROS message's own array type."""
    payload = occupancy_grid_to_dict(_fake_occupancy_grid(data=(1, 2, 3)))

    assert isinstance(payload['data'], list)


# --- R13 zigzag/stall diagnostics -------------------------------------------

def test_classify_stop_reason_is_open_while_still_running() -> None:
    assert classify_stop_reason('navigating', '', None, None) == 'open'


def test_classify_stop_reason_recognises_total_timeout() -> None:
    result = classify_stop_reason(
        'failed', 'prazo total de exploracao excedido', 3, 2)
    assert result == 'total_timeout'


def test_classify_stop_reason_recognises_cancellation() -> None:
    assert classify_stop_reason(
        'cancelled', 'busca cancelada pelo operador', 3, 2) == 'cancelled'


def test_classify_stop_reason_splits_barren_by_raw_cluster_count() -> None:
    """R12's whole point: 'no cluster ever existed' vs 'all were filtered'."""
    message = 'nenhuma fronteira segura alcancavel'

    assert classify_stop_reason('failed', message, 0, 0) == \
        'barren_no_raw_frontiers'
    assert classify_stop_reason('failed', message, 4, 0) == \
        'barren_frontiers_filtered'
    assert classify_stop_reason('failed', message, 4, 2) == 'barren_other'


def test_classify_stop_reason_does_not_special_case_a_goal_timeout() -> None:
    """A per-goal timeout never ends the run by itself (see GOAL_FIELDS)."""
    assert classify_stop_reason(
        'failed', 'meta de fronteira expirou', 3, 2) == 'unknown'


def _navigating_row(sim_s, x, y, cmd_vx=0.2, cmd_wz=0.0) -> dict:
    return _row(state='navigating', sim_s=sim_s, x=x, y=y,
                cmd_vx=cmd_vx, cmd_wz=cmd_wz)


def _seconds(n: int) -> range:
    return range(0, n, 1)


def test_stall_windows_flags_a_command_with_no_motion() -> None:
    rows = [_navigating_row(t, 0.0, 0.0) for t in _seconds(15)]

    windows = find_stalled_navigating_windows(rows, min_stall_s=10.0)

    assert len(windows) == 1
    assert windows[0]['duration_s'] >= 10.0


def test_stall_windows_ignores_real_progress() -> None:
    """Steady net displacement must not read as a stall."""
    rows = [_navigating_row(t, 0.05 * t, 0.0) for t in _seconds(15)]

    assert find_stalled_navigating_windows(rows, min_stall_s=10.0) == []


def test_stall_windows_ignores_a_short_pause() -> None:
    rows = [_navigating_row(t, 0.0, 0.0) for t in _seconds(5)]

    assert find_stalled_navigating_windows(rows, min_stall_s=10.0) == []


def test_stall_windows_require_a_nonzero_command() -> None:
    """A goal-less pause (no command at all) is not the watchdog's target."""
    rows = [_navigating_row(t, 0.0, 0.0, cmd_vx=0.0, cmd_wz=0.0)
            for t in _seconds(15)]

    assert find_stalled_navigating_windows(rows, min_stall_s=10.0) == []


def test_stall_windows_only_count_the_navigating_state() -> None:
    rows = [_row(state='selecting', sim_s=t, x=0.0, y=0.0, cmd_vx=0.2)
            for t in _seconds(15)]

    assert find_stalled_navigating_windows(rows, min_stall_s=10.0) == []


def test_path_metrics_of_a_straight_path_is_one() -> None:
    metrics = path_metrics([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)])

    assert metrics['length_m'] == 2.0
    assert metrics['straightness'] == 1.0


def test_path_metrics_of_a_zigzag_path_is_below_one() -> None:
    metrics = path_metrics([(0.0, 0.0), (1.0, 0.3), (2.0, -0.3), (3.0, 0.0)])

    assert 0.0 < metrics['straightness'] < 1.0
    assert metrics['length_m'] > 3.0


def test_path_metrics_of_too_short_a_path_is_none() -> None:
    expected = {'length_m': None, 'straightness': None}
    assert path_metrics([(0.0, 0.0)]) == expected


def _flat_grid(width, height, resolution=0.1) -> OccupancyGridInfo:
    return OccupancyGridInfo(
        width=width, height=height, resolution=resolution,
        origin_x=-width * resolution / 2, origin_y=-height * resolution / 2,
        origin_yaw=0.0)


def test_lateral_wall_clearance_finds_walls_on_both_sides() -> None:
    """A robot driving along a north-south corridor, walls 0.5 m either side.

    The wall cells are two lines at fixed x = +/-0.5 m spanning every row
    (i.e. a corridor running along y). Facing 90 deg (+y, along the
    corridor) puts those walls to the robot's own left/right; facing 0 deg
    would put them straight ahead instead (see the heading-rotation test
    below), which is exactly the distinction this helper exists to make.
    """
    width, height, resolution = 40, 40, 0.05
    grid = _flat_grid(width, height, resolution)
    data = [0] * (width * height)
    centre_col = width // 2
    for row in range(height):
        data[row * width + centre_col - 10] = 100
        data[row * width + centre_col + 10] = 100

    left_m, right_m = lateral_wall_clearance_m(
        grid, data, robot_x=0.0, robot_y=0.0, robot_yaw_deg=90.0,
        max_probe_m=1.0)

    assert left_m == pytest.approx(0.5, abs=resolution)
    assert right_m == pytest.approx(0.5, abs=resolution)


def test_lateral_wall_clearance_reports_max_probe_when_clear() -> None:
    grid = _flat_grid(40, 40, 0.05)
    data = [0] * (40 * 40)

    left_m, right_m = lateral_wall_clearance_m(
        grid, data, robot_x=0.0, robot_y=0.0, robot_yaw_deg=0.0,
        max_probe_m=1.0)

    assert left_m == 1.0
    assert right_m == 1.0


def test_lateral_wall_clearance_rotates_with_robot_heading() -> None:
    """A 90 deg-rotated robot swaps which world direction is 'left'."""
    width, height, resolution = 40, 40, 0.05
    grid = _flat_grid(width, height, resolution)
    data = [0] * (width * height)
    centre_col = width // 2
    # A wall only to the world's +x side (straight ahead of a 0 deg robot).
    for row in range(height):
        data[row * width + centre_col + 10] = 100

    facing_east = lateral_wall_clearance_m(
        grid, data, robot_x=0.0, robot_y=0.0, robot_yaw_deg=0.0,
        max_probe_m=1.0)
    facing_north = lateral_wall_clearance_m(
        grid, data, robot_x=0.0, robot_y=0.0, robot_yaw_deg=90.0,
        max_probe_m=1.0)

    # Facing +x, the +x wall is neither left nor right -- both clear.
    assert facing_east == (1.0, 1.0)
    # Facing +y (90 deg), the +x wall is now to the robot's right.
    assert facing_north[1] < 1.0
    assert facing_north != facing_east
