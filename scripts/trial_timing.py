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
