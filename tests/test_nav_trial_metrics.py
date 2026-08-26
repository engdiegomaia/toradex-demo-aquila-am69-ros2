"""Unit tests for timing recorded by scripts/nav_trial.py."""

from __future__ import annotations

from pathlib import Path
import sys

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))

from trial_timing import timing_spans, vx_metrics  # noqa: E402


def test_timing_spans_compares_sim_and_wall_over_same_interval() -> None:
    rows = [
        {'sim_s': 100.0, 'wall_s': 2.0},
        {'sim_s': 114.5, 'wall_s': 17.0},
    ]

    sim_span, wall_span, rtf = timing_spans(rows)

    assert sim_span == 14.5
    assert wall_span == 15.0
    assert rtf == pytest.approx(0.9666667)


@pytest.mark.parametrize(
    'rows',
    [
        [],
        [{'sim_s': 1.0, 'wall_s': 1.0}],
        [
            {'sim_s': 2.0, 'wall_s': 1.0},
            {'sim_s': 1.0, 'wall_s': 2.0},
        ],
        [
            {'sim_s': 1.0, 'wall_s': 2.0},
            {'sim_s': 2.0, 'wall_s': 2.0},
        ],
    ],
)
def test_timing_spans_rejects_invalid_measurements(rows) -> None:
    with pytest.raises(ValueError):
        timing_spans(rows)


def test_vx_metrics_uses_one_explicit_threshold_for_zero_and_work() -> None:
    rows = [{'cmd_vx': 0.0}, {'cmd_vx': 0.005}, {'cmd_vx': 0.0501},
            {'cmd_vx': -0.02}]

    zero, duty = vx_metrics(rows)

    assert zero == pytest.approx(0.5)
    assert duty == pytest.approx(0.25)


def test_vx_metrics_rejects_negative_threshold_and_empty_rows() -> None:
    with pytest.raises(ValueError):
        vx_metrics([], 0.005, 0.05)
    with pytest.raises(ValueError):
        vx_metrics([{'cmd_vx': 0.0}], -0.001, 0.05)
