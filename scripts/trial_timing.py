"""Dependency-free timing helpers shared by navigation trial tests."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any


def timing_spans(rows: Sequence[dict[str, Any]]) -> tuple[float, float, float]:
    """Return simulated span, wall span and RTF over the same samples."""
    if len(rows) < 2:
        raise ValueError('at least two samples are required')
    sim_span = float(rows[-1]['sim_s']) - float(rows[0]['sim_s'])
    wall_span = float(rows[-1]['wall_s']) - float(rows[0]['wall_s'])
    if sim_span < 0.0:
        raise ValueError('simulation time moved backwards')
    if wall_span <= 0.0:
        raise ValueError('wall time did not advance')
    return sim_span, wall_span, sim_span / wall_span


def vx_metrics(rows: Sequence[dict[str, Any]], zero_threshold: float = 0.005,
               work_threshold: float = 0.05) -> tuple[float, float]:
    """Return (near-zero fraction, forward-work fraction) for sampled cmd_vx.

    The two thresholds are deliberate and explicit.  ``0.005`` m/s is the
    historical near-zero band; ``0.05`` m/s is the forward-command band used
    by the existing F5 reports (negative/turning commands are not work).
    """
    if zero_threshold < 0.0 or work_threshold < 0.0:
        raise ValueError('thresholds must be non-negative')
    if not rows:
        raise ValueError('at least one sample is required')
    values = [float(row['cmd_vx']) for row in rows]
    zero = sum(abs(value) <= zero_threshold for value in values) / len(values)
    duty = sum(value > work_threshold for value in values) / len(values)
    return zero, duty
