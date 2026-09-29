#!/usr/bin/env python3
"""
Derives the maze EXIT ROUTE from the STL: internal waypoints + external goal.

Runs on the x86 host, offline, without ROS or Gazebo. Read-only.

    python3 tools/maze/maze_route.py --models ~/ros_maze_worlds/models maze11

WHY THIS SCRIPT EXISTS

`maze_fit.py` answers "this maze is usable, and the robot spawns here". It
suggests goals SCATTERED within a radius, which is what a patrol wants. It is
not what a CROSSING wants: to get out, the sequence of goals has to follow a
connected path to the opening, in the right order, and end OUTSIDE.

The opening cannot be picked by eye in Gazebo. A goal on a wall is ACCEPTED by
Nav2 and fails later, near the edge, where the error no longer has a name --
the same reason `MAZE11_GOALS` was generated rather than guessed.

THE SUCCESS CRITERION THIS MAKES POSSIBLE

`SUCCEEDED` on the last goal is NOT "left the maze": Nav2 declares success
within `xy_goal_tolerance` (0.25 m), and the tolerance alone does not say which
SIDE of the wall the robot stopped on. This script also emits the exit
boundary, so the trial can check CROSSING, which is geometric and does not
depend on Nav2 agreeing with itself.

COORDINATE CONVENTION

Same as `maze_fit.py` and `patrol_commander`: origin at the robot's spawn
cell, +x forward, +y to the left. The SDF pose already puts the maze in that
relationship -- see the header of `quadruped_maze11.sdf`.
"""

from __future__ import annotations

import argparse
import heapq
import math
from pathlib import Path

import numpy as np

import maze_fit


# Target spacing between waypoints. This is not aesthetics.
#
# The MPPI horizon is 96 x 0.1 x 0.15 = 1.44 m (see nav2_params_go2.yaml). A
# goal farther than the horizon leaves the cost field almost flat near the
# robot, which was measured on 20/08 as the cause of "command comes out as
# noise around zero". A goal much closer than that makes the goal_checker fire
# before the robot gains speed, and the crossing becomes a sequence of starts.
WAYPOINT_SPACING_M = 1.4

# How far the external goal sits BEYOND the opening. 1.0 m is more than the
# trunk's circumscribed radius (0.383) plus xy_goal_tolerance (0.25): it
# guarantees that satisfying the goal requires the whole body outside, not the
# tolerance just touching it.
EXTERNAL_MARGIN_M = 1.0

# Weight of the corridor-centre deviation in the path cost. At 0 Dijkstra cuts
# corners and scrapes walls; too high and it refuses a legitimate narrow
# corridor.
CENTRE_WEIGHT = 2.0

NEIGHBOURS = tuple(
    (dr, dc, math.hypot(dr, dc))
    for dr in (-1, 0, 1) for dc in (-1, 0, 1) if (dr, dc) != (0, 0)
)


def boundary_openings(component: np.ndarray) -> list[tuple[int, int]]:
    """Navigable cells touching the grid border: these are real openings."""
    rows, cols = component.shape
    found = []
    for c in range(cols):
        if component[0, c]:
            found.append((0, c))
        if component[rows - 1, c]:
            found.append((rows - 1, c))
    for r in range(rows):
        if component[r, 0]:
            found.append((r, 0))
        if component[r, cols - 1]:
            found.append((r, cols - 1))
    return found


def group_openings(cells: list) -> list:
    """
    Group contiguous border cells into a single gap.

    Counting CELLS instead of gaps would make the report say "10 openings" where
    there is one 0.50 m wide, and a reader would conclude an alternative route
    exists.
    """
    groups: list = []
    for cell in sorted(cells):
        for group in groups:
            if any(abs(cell[0] - other[0]) <= 1 and abs(cell[1] - other[1]) <= 1
                   for other in group):
                group.append(cell)
                break
        else:
            groups.append([cell])
    return groups


def dijkstra(component, clearance, resolution, start):
    """Shortest path weighted by wall clearance. Returns (cost, parent)."""
    best_clear = float(clearance[component].max())
    cost = {start: 0.0}
    parent: dict = {start: None}
    queue = [(0.0, start)]
    rows, cols = component.shape

    while queue:
        here, cell = heapq.heappop(queue)
        if here > cost.get(cell, math.inf):
            continue
        row, col = cell
        for drow, dcol, step in NEIGHBOURS:
            nrow, ncol = row + drow, col + dcol
            if not (0 <= nrow < rows and 0 <= ncol < cols):
                continue
            if not component[nrow, ncol]:
                continue
            # Penalise proximity to a wall without forbidding it.
            penalty = 1.0 + CENTRE_WEIGHT * (
                1.0 - float(clearance[nrow, ncol]) / best_clear)
            candidate = here + step * resolution * penalty
            if candidate < cost.get((nrow, ncol), math.inf):
                cost[(nrow, ncol)] = candidate
                parent[(nrow, ncol)] = cell
                heapq.heappush(queue, (candidate, (nrow, ncol)))
    return cost, parent


def trace(parent, target):
    path = []
    cell = target
    while cell is not None:
        path.append(cell)
        cell = parent[cell]
    return path[::-1]


def sample(path, clearance, resolution, spacing):
    """
    Sample the path every `spacing` metres, snapping to the corridor centre.

    Without the snap the waypoints fall wherever Dijkstra passed, which is
    reasonable but not the best local point: a goal 5 cm closer to the centre
    costs nothing and takes the robot out of the band where the CostCritic
    penalises it.
    """
    picked = [path[0]]
    walked = 0.0
    for previous, cell in zip(path, path[1:]):
        walked += math.hypot(cell[0] - previous[0], cell[1] - previous[1]) * resolution
        if walked < spacing:
            continue
        walked = 0.0
        row, col = cell
        window = clearance[max(row - 2, 0):row + 3, max(col - 2, 0):col + 3]
        offset = np.unravel_index(int(np.argmax(window)), window.shape)
        picked.append((max(row - 2, 0) + offset[0], max(col - 2, 0) + offset[1]))
    if picked[-1] != path[-1]:
        picked.append(path[-1])
    return picked


def build(name: str, models: Path, scale: float, resolution: float,
          spacing: float) -> dict:
    result = maze_fit.analyse(name, models, scale, resolution, start='se')
    component, _centred, clearance, gx, gy, srow, scol, res = result['_grid']

    def to_robot(cell):
        return (float(gx[cell[1]] - gx[scol]), float(gy[cell[0]] - gy[srow]))

    openings = boundary_openings(component)
    gaps = group_openings(openings)
    if not openings:
        raise SystemExit(f'{name}: no navigable opening on the grid border')

    cost, parent = dijkstra(component, clearance, res, (srow, scol))
    reachable = [cell for cell in openings if cell in cost]
    if not reachable:
        raise SystemExit(f'{name}: the opening exists but is not reachable from the start')
    exit_cell = min(reachable, key=lambda cell: cost[cell])

    path = trace(parent, exit_cell)
    raw_m = sum(math.hypot(b[0] - a[0], b[1] - a[1]) * res
                for a, b in zip(path, path[1:]))
    waypoints = [to_robot(cell) for cell in sample(path, clearance, res, spacing)]

    # The external goal goes out in the direction of the border the opening touches.
    rows, cols = component.shape
    if exit_cell[0] == 0:
        heading = (0.0, -1.0)
    elif exit_cell[0] == rows - 1:
        heading = (0.0, +1.0)
    elif exit_cell[1] == 0:
        heading = (-1.0, 0.0)
    else:
        heading = (+1.0, 0.0)

    opening = to_robot(exit_cell)
    external = (opening[0] + heading[0] * EXTERNAL_MARGIN_M,
                opening[1] + heading[1] * EXTERNAL_MARGIN_M)

    return {
        'name': name, 'grid': component.shape, 'resolution': res,
        'start_cell': (srow, scol), 'exit_cell': exit_cell,
        'gaps': len(gaps), 'gap_width_m': max(len(g) for g in gaps) * res,
        'reachable_cells': len(reachable),
        'weighted_cost_m': cost[exit_cell], 'path_m': raw_m,
        'opening': opening, 'opening_clearance': float(clearance[exit_cell]),
        'external': external, 'heading': heading,
        'waypoints': waypoints[1:],  # the first one is the start itself
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mazes', nargs='+')
    parser.add_argument('--models', type=Path,
                        default=Path.home() / 'ros_maze_worlds' / 'models')
    parser.add_argument('--scale', type=float, default=0.002)
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--spacing', type=float, default=WAYPOINT_SPACING_M)
    args = parser.parse_args()

    for name in args.mazes:
        r = build(name, args.models, args.scale, args.resolution, args.spacing)
        print(f"=== {r['name']} ===")
        print(f"grid {r['grid'][0]}x{r['grid'][1]} @ {r['resolution']} m")
        print(f"start cell {r['start_cell']}  exit cell {r['exit_cell']}")
        print(f"border gaps: {r['gaps']} "
              f"(widest with {r['gap_width_m']:.2f} m of centring freedom; "
              f"{r['reachable_cells']} cells reachable from the start)")
        print(f"internal path: {r['path_m']:.2f} m "
              f"(weighted cost {r['weighted_cost_m']:.2f})")
        print(f"opening at ({r['opening'][0]:.2f}, {r['opening'][1]:.2f}) "
              f"clearance {r['opening_clearance']:.2f} m")
        print(f"external goal ({r['external'][0]:.2f}, {r['external'][1]:.2f})")
        print(f"\n{len(r['waypoints'])} waypoints, ready for --goals:")
        print(';'.join(f'{x:.2f},{y:.2f}' for x, y in r['waypoints']))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
