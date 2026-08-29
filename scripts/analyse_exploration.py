#!/usr/bin/env python3
"""
Reproduce the derived numbers quoted in the F5 round reports.

Every figure in `docs/results/ml35-f5-*.md` that is not read straight off a CSV
comes from here, so a reviewer can regenerate it instead of trusting prose:

    scripts/analyse_exploration.py ratio docs/results/ml35-f5-exploration-r7.csv
    scripts/analyse_exploration.py progress docs/results/ml35-f5-exploration-r{5,6}.csv

WHY THIS EXISTS

The R7 report quoted "131 samples, ratio 0.478 -> 1.055" and the progress-checker
change quoted "6/9 expiries caught, 107 s returned, 0 good goals aborted". Both
were computed in a throwaway shell and were not reproducible from the tree. A
number nobody can regenerate is an assertion, not evidence.

WHAT THE NUMBERS MEAN, AND WHAT THEY DO NOT

`ratio` compares the marker range the perception stack published against the
true robot-to-marker distance, using the maze exit marker's pose from the world
file as ground truth. Samples are consecutive observations along ONE trajectory
at 2 Hz: they are strongly autocorrelated, so the bucket means describe the runs
that produced them and are NOT a confidence interval on the estimator.

`progress` replays `nav2_controller::SimpleProgressChecker` over recorded poses.
The checker resets its baseline whenever the robot moves `required_movement_radius`
and aborts when it fails to within `movement_time_allowance`. Replay cannot see
what Nav2 would have done DIFFERENTLY after an early abort -- it answers only
"would this goal have been cut, and when", which is what sizing the parameter
needs.

The pure functions below are unit-tested in `tests/test_analyse_exploration.py`
without a sourced Jazzy environment.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys

# `quadruped_maze11.sdf`: <model name="maze_exit_marker"><pose>-4.90 -2.60 ...
MARKER_XY = (-4.90, -2.60)

# Reported band edges, in metres of ESTIMATED range.
BUCKETS = ((0.0, 2.0), (2.0, 3.0), (3.0, 4.0), (4.0, 6.0), (6.0, float('inf')))


def ratio_pairs(rows: list, marker_xy: tuple = MARKER_XY) -> list:
    """Return ``(estimated_m, true_m)`` for every fresh marker observation.

    Consecutive rows repeating the same estimate are the SAME observation held
    by the 2 Hz status publisher, not new measurements, so they are collapsed.
    """
    pairs = []
    previous = None
    for row in rows:
        if row.get('marker_visible') != 'True':
            continue
        if row.get('marker_distance_m') in ('', None):
            continue
        estimated = float(row['marker_distance_m'])
        if round(estimated, 2) == previous:
            continue
        previous = round(estimated, 2)
        true = math.hypot(marker_xy[0] - float(row['x']),
                          marker_xy[1] - float(row['y']))
        pairs.append((estimated, true))
    return pairs


def bucket_ratios(pairs: list, buckets: tuple = BUCKETS) -> list:
    """Summarise estimate/truth by band of ESTIMATED range."""
    summary = []
    for low, high in buckets:
        chosen = [p for p in pairs if low <= p[0] < high]
        if not chosen:
            continue
        ratios = [e / t for e, t in chosen if t > 0.0]
        errors = [abs(e - t) for e, t in chosen]
        summary.append({
            'low': low,
            'high': high,
            'n': len(chosen),
            'ratio_mean': sum(ratios) / len(ratios),
            'ratio_min': min(ratios),
            'ratio_max': max(ratios),
            'abs_error_mean': sum(errors) / len(errors),
        })
    return summary


def progress_checker_fires(
    track: list, required_movement_radius: float, movement_time_allowance: float,
) -> float | None:
    """Replay SimpleProgressChecker; return seconds into the goal, or None.

    `track` is ``(t, x, y)`` ascending. The baseline pose resets every time the
    robot travels `required_movement_radius` from it; the checker aborts when
    that has not happened within `movement_time_allowance`.
    """
    if not track:
        return None
    base_t, base_x, base_y = track[0]
    for time_s, x, y in track[1:]:
        if math.hypot(x - base_x, y - base_y) >= required_movement_radius:
            base_t, base_x, base_y = time_s, x, y
        elif time_s - base_t >= movement_time_allowance:
            return time_s - track[0][0]
    return None


def goal_track(rows: list, sent_sim_s: float, elapsed_s: float) -> list:
    """Return the ``(t, x, y)`` samples recorded while one goal was active."""
    end = sent_sim_s + elapsed_s
    return [(float(r['sim_s']), float(r['x']), float(r['y']))
            for r in rows
            if r.get('x') not in ('', None)
            and sent_sim_s <= float(r['sim_s']) <= end]


def replay_progress_checker(
    runs: list, radius: float, allowance: float,
) -> dict:
    """Score one (radius, allowance) pair over ``[(rows, goals), ...]``."""
    caught = expiries = false_aborts = 0
    saved_s = 0.0
    for rows, goals in runs:
        for goal in goals:
            duration = float(goal['elapsed_s'])
            track = goal_track(rows, float(goal['sent_sim_s']), duration)
            fired = progress_checker_fires(track, radius, allowance)
            if 'expirou' in goal['message']:
                expiries += 1
                if fired is not None:
                    caught += 1
                    saved_s += duration - fired
            elif goal['outcome'] == 'ok' and fired is not None:
                false_aborts += 1
    return {
        'radius': radius, 'allowance': allowance, 'caught': caught,
        'expiries': expiries, 'saved_s': saved_s, 'false_aborts': false_aborts,
    }


def _read(path: str) -> list:
    with open(path, newline='', encoding='utf-8') as handle:
        return list(csv.DictReader(handle))


def _goals_path(rows_path: str) -> str:
    return rows_path[:-len('.csv')] + '-goals.csv'


def main(argv: list | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('ratio', 'progress'))
    parser.add_argument('csv', nargs='+', help='row CSVs from exploration_trial.py')
    args = parser.parse_args(argv)

    if args.mode == 'ratio':
        for path in args.csv:
            pairs = ratio_pairs(_read(path))
            if not pairs:
                print(f'{path}: no marker observations')
                continue
            ratios = [e / t for e, t in pairs]
            print(f'\n{path}: {len(pairs)} observations, '
                  f'ratio mean {sum(ratios) / len(ratios):.3f} '
                  f'[{min(ratios):.3f}, {max(ratios):.3f}]')
            print('  NOT independent: consecutive samples on one trajectory.')
            print(f'  {"band":>12} {"n":>4} {"ratio":>7} {"abs err":>9}')
            for row in bucket_ratios(pairs):
                high = '  inf' if row['high'] == float('inf') else f"{row['high']:5.1f}"
                print(f"  {row['low']:5.1f}-{high} {row['n']:>4} "
                      f"{row['ratio_mean']:>7.3f} {row['abs_error_mean']:>8.2f}m")
        return 0

    runs = [(_read(path), _read(_goals_path(path))) for path in args.csv]
    print(f'{"radius":>7} {"allowance":>10} {"caught":>12} {"saved":>9} {"false":>7}')
    for radius, allowance in ((0.20, 40.0), (0.30, 25.0), (0.20, 20.0), (0.50, 10.0)):
        result = replay_progress_checker(runs, radius, allowance)
        print(f"{result['radius']:>7.2f} {result['allowance']:>10.1f} "
              f"{result['caught']:>5}/{result['expiries']:<6} "
              f"{result['saved_s']:>8.1f}s {result['false_aborts']:>7}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
