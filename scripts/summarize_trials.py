#!/usr/bin/env python3
"""Summarise interleaved navigation CSVs with median and observed range.

Usage::

    python3 scripts/summarize_trials.py baseline=run1.csv baseline=run2.csv \
        tuned=run3.csv

The command intentionally does not pool samples: one CSV is one replicate.
This keeps the n>=3 protocol honest when a run aborts or has a different
duration.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path

from trial_timing import timing_spans, vx_metrics


def read_trial(path: Path, threshold: float) -> dict[str, float | str]:
    with path.open(newline='', encoding='utf-8') as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) < 2:
        raise ValueError(f'{path}: at least two samples are required')
    sim, wall, rtf = timing_spans(rows)
    xs = [float(row['x']) for row in rows]
    ys = [float(row['y']) for row in rows]
    path_m = sum(math.hypot(x1 - x0, y1 - y0)
                 for x0, y0, x1, y1 in zip(xs, ys, xs[1:], ys[1:]))
    zero, duty = vx_metrics(rows, threshold, 0.05)
    return {
        'file': str(path),
        'velocity_mps': path_m / sim,
        'vx_zero': zero,
        'vx_duty': duty,
        'rtf': rtf,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trials', nargs='+', metavar='LABEL=CSV')
    parser.add_argument('--vx-zero-threshold', type=float, default=0.005)
    args = parser.parse_args(argv)
    groups: dict[str, list[dict[str, float | str]]] = {}
    for spec in args.trials:
        if '=' not in spec:
            parser.error(f'{spec!r}: use LABEL=CSV')
        label, raw_path = spec.split('=', 1)
        if not label or not raw_path:
            parser.error(f'{spec!r}: use LABEL=CSV')
        try:
            result = read_trial(Path(raw_path), args.vx_zero_threshold)
        except (OSError, ValueError, KeyError) as exc:
            parser.error(str(exc))
        groups.setdefault(label, []).append(result)

    print(f'zero: |cmd_vx| <= {args.vx_zero_threshold:.3f} m/s; work: cmd_vx > 0.050 m/s')
    print('condition  n  duty median [min,max]   vx~0 median [min,max]   '
          'velocity median [min,max]')
    for label, trials in groups.items():
        def values(key: str) -> list[float]:
            return [float(trial[key]) for trial in trials]
        duty = values('vx_duty')
        zero = values('vx_zero')
        velocity = values('velocity_mps')
        pct = lambda value: f'{100.0 * value:.1f}%'
        print(f'{label:<10} {len(trials):>2} '
              f'{pct(statistics.median(duty))} '
              f'[{pct(min(duty))},{pct(max(duty))}]   '
              f'{pct(statistics.median(zero))} '
              f'[{pct(min(zero))},{pct(max(zero))}]   '
              f'{statistics.median(velocity):.4f} '
              f'[{min(velocity):.4f},{max(velocity):.4f}] m/s')
    return 0


if __name__ == '__main__':
    sys.exit(main())
