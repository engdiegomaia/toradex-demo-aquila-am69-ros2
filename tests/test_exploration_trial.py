"""Unit tests for the pure helpers of scripts/exploration_trial.py.

These run WITHOUT ROS. That is the reason `exploration_trial` imports `rclpy`
and the message packages inside `build_recorder()` instead of at module scope —
importing the module here must not require a sourced Jazzy environment.
"""

from __future__ import annotations

import csv
from pathlib import Path
import sys

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))

from exploration_trial import (  # noqa: E402
    GOAL_FIELDS,
    GOAL_STATUS_SUCCEEDED,
    ROW_FIELDS,
    goals_csv_path,
    parse_goal_outcome,
    percentile,
    quat_to_yaw_tilt,
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
        _row(sim_s=0.0, wall_s=0.0, cmd_vx=0.0),
        _row(sim_s=1.0, wall_s=1.0, cmd_vx=0.10),
        _row(sim_s=2.0, wall_s=2.0, cmd_vx=0.0),
        _row(sim_s=3.0, wall_s=3.0, cmd_vx=0.10),
    ]

    summary = summarise(rows, [], {})

    assert summary['vx_work_ratio'] == 0.5
    assert summary['vx_mean_abs'] == pytest.approx(0.05)


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
