"""Dependency-free timing helpers shared by navigation trial tests."""

from __future__ import annotations

from collections.abc import Sequence
import math
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


def path_metrics(points: Sequence[tuple[float, float]],
                 lookahead_m: float = 0.5) -> tuple[float, float]:
    """Return polyline length and initial lookahead bearing in degrees.

    The lookahead avoids treating tiny discretisation changes in the first
    NavFn cells as a route change. The bearing is measured from the first pose
    to the first pose at least ``lookahead_m`` along the polyline, or to the
    final pose when the whole path is shorter.
    """
    if lookahead_m <= 0.0:
        raise ValueError('lookahead must be positive')
    if len(points) < 2:
        return 0.0, math.nan

    length = 0.0
    target = None
    start_x, start_y = points[0]
    last_x, last_y = start_x, start_y
    for x, y in points[1:]:
        segment = math.hypot(x - last_x, y - last_y)
        length += segment
        if target is None and length >= lookahead_m:
            target = (x, y)
        last_x, last_y = x, y
    if target is None and length > 0.0:
        target = (last_x, last_y)
    if target is None:
        return 0.0, math.nan
    bearing = math.degrees(math.atan2(target[1] - start_y,
                                      target[0] - start_x))
    return length, bearing


def angular_distance_deg(a: float, b: float) -> float:
    """Smallest unsigned separation between two headings."""
    return abs((a - b + 180.0) % 360.0 - 180.0)


def plan_switch_count(rows: Sequence[dict[str, Any]],
                      length_delta_m: float = 1.0,
                      heading_delta_deg: float = 45.0,
                      max_age_s: float = 2.5) -> int:
    """Count large consecutive changes among fresh, valid global plans."""
    if min(length_delta_m, heading_delta_deg, max_age_s) < 0.0:
        raise ValueError('plan thresholds must be non-negative')
    previous = None
    switches = 0
    for row in rows:
        try:
            length = float(row['plan_length_m'])
            heading = float(row['plan_heading_deg'])
            age = float(row['plan_age_s'])
        except (KeyError, TypeError, ValueError):
            continue
        if not all(math.isfinite(value) for value in (length, heading, age)):
            continue
        if age > max_age_s:
            continue
        current = (length, heading)
        if previous is not None:
            if (abs(current[0] - previous[0]) > length_delta_m
                    or angular_distance_deg(current[1], previous[1])
                    > heading_delta_deg):
                switches += 1
        previous = current
    return switches


def goals_csv_path(csv_path: str) -> str:
    """Return the sibling CSV that carries one row per finished goal.

    The telemetry CSV is one row per SAMPLE; a goal outcome is one row per
    ACTION. Forcing both into one file either repeats the outcome on every
    sample or leaves most cells empty, and both make the gate unreadable. The
    sibling keeps each file with a single meaning and a stable schema.
    """
    root, dot, extension = csv_path.rpartition('.')
    if not dot or '/' in extension:
        return f'{csv_path}-metas.csv'
    return f'{root}-metas.{extension}'
