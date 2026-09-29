"""
Pure helpers behind the numbers quoted in the F5 round reports.

The reports lean on two derived quantities: the marker range ratio against SDF
ground truth, and a replay of `SimpleProgressChecker` used to size
`required_movement_radius`/`movement_time_allowance`. Both were computed ad hoc
before this module existed, which made every figure in the reports an assertion
rather than reproducible evidence. These tests pin the semantics that the
reports depend on.
"""

import importlib.util
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location(
    'analyse_exploration', ROOT / 'tools' / 'evaluation' / 'analyse_exploration.py')
analyse = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(analyse)


def _row(sim_s, x, y, visible=True, distance=None):
    return {
        'sim_s': str(sim_s), 'x': str(x), 'y': str(y),
        'marker_visible': 'True' if visible else 'False',
        'marker_distance_m': '' if distance is None else str(distance),
    }


def test_a_held_estimate_is_one_observation_not_many() -> None:
    """Status is published at 2 Hz; repeating the same value is not a new measurement.

    Counting each row would inflate the sample with copies of the same
    detection, which is exactly the reading error this session already made
    with the `timed_out` counter.
    """
    rows = [_row(0.0, 0.0, 0.0, True, 2.00),
            _row(0.5, 0.0, 0.0, True, 2.00),
            _row(1.0, 0.1, 0.0, True, 2.00),
            _row(1.5, 0.2, 0.0, True, 1.90)]

    assert len(analyse.ratio_pairs(rows)) == 2


def test_ratio_is_measured_against_the_world_file_marker() -> None:
    """Ground truth is the SDF pose, never another estimate."""
    rows = [_row(0.0, -4.90, -1.60, True, 0.5)]     # exactly 1 m from the marker

    (estimated, true), = analyse.ratio_pairs(rows)
    assert estimated == 0.5
    assert true == 1.0


def test_an_invisible_marker_contributes_no_pair() -> None:
    """`marker_distance_m` stays latched after the marker disappears."""
    rows = [_row(0.0, 0.0, 0.0, False, 3.0)]

    assert analyse.ratio_pairs(rows) == []


def test_buckets_split_on_the_ESTIMATED_range() -> None:
    """The gate decides using the estimate; bucketing by ground truth does not size it."""
    pairs = [(1.0, 2.0), (3.5, 3.4)]

    bands = {(b['low'], b['high']): b for b in analyse.bucket_ratios(pairs)}
    assert bands[(0.0, 2.0)]['n'] == 1
    assert bands[(3.0, 4.0)]['n'] == 1


def test_the_checker_resets_its_baseline_when_the_robot_moves() -> None:
    """A slow, continuous robot is never cut off; that is the plugin's contract."""
    track = [(t * 1.0, t * 0.35, 0.0) for t in range(40)]

    assert analyse.progress_checker_fires(track, 0.30, 25.0) is None


def test_the_checker_fires_on_a_robot_that_stops_moving() -> None:
    """Stopped within the radius, the deadline runs out and the goal is aborted."""
    track = [(t * 1.0, 0.01 * t, 0.0) for t in range(40)]

    fired = analyse.progress_checker_fires(track, 0.30, 25.0)
    assert fired is not None and fired >= 25.0


def test_replay_scores_expiries_and_never_credits_a_good_goal() -> None:
    """Cutting off a good goal is the cost of the parameter; it must be counted separately."""
    rows = [_row(t * 1.0, 0.0, 0.0) for t in range(40)]
    goals = [
        # 'expirou' is the literal substring tools/evaluation/analyse_exploration.py
        # matches on to classify an expiry; kept in Portuguese so the check
        # still fires (that module is out of scope for this translation pass).
        {'sent_sim_s': '0.0', 'elapsed_s': '39.0', 'outcome': 'failed',
         'message': 'meta de fronteira expirou'},
        {'sent_sim_s': '0.0', 'elapsed_s': '39.0', 'outcome': 'ok',
         'message': 'frontier reached'},
    ]

    result = analyse.replay_progress_checker([(rows, goals)], 0.30, 25.0)
    assert result['expiries'] == 1
    assert result['caught'] == 1
    assert result['false_aborts'] == 1
    assert result['saved_s'] > 0.0
