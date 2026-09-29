#!/usr/bin/env python3
"""
Measures, offline, HOW MUCH Nav2 does not know when it receives a goal in this maze.

Runs on the x86 host, without ROS or Gazebo. Reads the STL only.

    python3 tools/maze/maze_geodesic.py --models ~/ros_maze_worlds/models maze11

WHY THIS SCRIPT EXISTS

`maze_fit.py` answers "this maze is usable and the goals land in corridors".
`maze_route.py` answers "this is the sequence to get OUT". Neither answers the
question the F5 investigation needed:

    is the goal the trial sends reachable along the STRAIGHT LINE, or does the
    global planner have to invent a path through space the robot has never seen?

The question matters because the quadruped's global costmap is a ROLLING WINDOW
with no static layer and no map (`nav2_params_go2.yaml`, global_costmap), and
NavFn runs with `allow_unknown: true`. Together these mean a goal behind a
not-yet-observed wall produces a plan THROUGH the wall, with no error, no
warning, and a perfect appearance in `/plan`.

This is the worst failure mode in this project: nothing flags it.
`compute_path_to_pose` returns `SUCCEEDED`, the path shows up in RViz and in the
cockpit, and the controller spends the whole trial trying to follow a straight
line that crosses masonry.

WHAT IT MEASURES

  straight    Euclidean distance spawn -> goal
  geodesic    shortest path through NAVIGABLE space (eroded by robot_radius)
  ratio       geodesic / straight.  1.0 = the straight line works.  >1 = the
              straight line lies.
  1st wall    where the straight line first touches a wall
  visible     fraction of the free space within lidar range that the robot sees
              from the spawn, with occlusion.  It is the size of the ignorance
              at time zero.

COORDINATE CONVENTION

Same as `maze_fit.py` and `MAZE11_GOALS` in `nav_trial.py`: origin at the
robot's spawn cell, axes of the `map` frame. The check at the start of
`analyse_goals` fails loudly if that relationship breaks -- a silently swapped
frame is the class of error this file exists to avoid.
"""

from __future__ import annotations

import argparse
import heapq
import math
from pathlib import Path

import numpy as np
from scipy import ndimage

import maze_fit


# Circumscribed radius of the Go2 trunk, equal to the `robot_radius` of both
# costmaps. Navigable space is the costmap eroded by it, not raw free space: a
# 0.20 m crack is free and not navigable, and a geodesic through it would be a
# route the planner never picks.
ROBOT_RADIUS_M = 0.383

# `obstacle_max_range` of the global_costmap. It is NOT the L1's declared range
# (10 m): what is not marked does not enter the costmap, so the range that
# matters here is the costmap's, not the sensor's.
LIDAR_RANGE_M = 8.0

# Resolution of both costmaps. Measuring at another resolution would give a
# different geodesic.
COSTMAP_RESOLUTION_M = 0.05

# Rays of the visibility raycast. 2880 = one every 0.125 degrees; at 8 m that
# is 1.7 cm of arc, below the 5 cm cell, so no corridor escapes through sparse
# angular sampling.
VISIBILITY_RAYS = 2880


def maze11_goals() -> list[tuple[float, float]]:
    """
    Read `MAZE11_GOALS` from the `nav_trial.py` source, via AST, not import.

    `nav_trial` imports `geometry_msgs` on its first useful line, so importing
    it here would force this script to have ROS -- and it exists precisely to
    run offline. Copying the tuple would create two sources of truth that
    silently diverge the day someone regenerates the goals with `maze_fit.py`.
    """
    import ast

    source = (Path(__file__).resolve().parents[1] / 'evaluation' / 'nav_trial.py').read_text()
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Assign):
            names = [t.id for t in node.targets if isinstance(t, ast.Name)]
            if 'MAZE11_GOALS' in names:
                return [tuple(v) for v in ast.literal_eval(node.value)]
    raise SystemExit('MAZE11_GOALS not found in nav_trial.py')


def navigable_mask(clearance: np.ndarray) -> np.ndarray:
    """Cells where the robot centre fits."""
    return clearance >= ROBOT_RADIUS_M


def geodesic_field(mask: np.ndarray, row: int, col: int,
                   resolution: float) -> np.ndarray:
    """8-connected Dijkstra over `mask`, in metres, from (row, col)."""
    dist = np.full(mask.shape, np.inf)
    dist[row, col] = 0.0
    diag = math.sqrt(2.0)
    steps = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, diag), (-1, 1, diag), (1, -1, diag), (1, 1, diag))
    queue = [(0.0, row, col)]
    rows, cols = mask.shape
    while queue:
        here, r, c = heapq.heappop(queue)
        if here > dist[r, c]:
            continue
        for dr, dc, weight in steps:
            rr, cc = r + dr, c + dc
            if 0 <= rr < rows and 0 <= cc < cols and mask[rr, cc]:
                there = here + weight * resolution
                if there < dist[rr, cc]:
                    dist[rr, cc] = there
                    heapq.heappush(queue, (there, rr, cc))
    return dist


def descend(dist: np.ndarray, mask: np.ndarray,
            row: int, col: int) -> list[tuple[int, int]]:
    """Walk back down the geodesic gradient to the origin."""
    path = [(row, col)]
    rows, cols = dist.shape
    while math.isfinite(dist[row, col]) and dist[row, col] > 0.0:
        best, br, bc = dist[row, col], row, col
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                rr, cc = row + dr, col + dc
                if 0 <= rr < rows and 0 <= cc < cols and mask[rr, cc]:
                    if dist[rr, cc] < best:
                        best, br, bc = dist[rr, cc], rr, cc
        if (br, bc) == (row, col):
            break
        row, col = br, bc
        path.append((row, col))
    return path


def visibility(free: np.ndarray, to_rc, resolution: float, reach: float,
               x0: float = 0.0, y0: float = 0.0) -> np.ndarray:
    """2D raycast with occlusion from (x0, y0). Marks the cell that was hit."""
    seen = np.zeros_like(free)
    rows, cols = free.shape
    samples = int(reach / (resolution / 2.0))
    for k in range(VISIBILITY_RAYS):
        angle = 2.0 * math.pi * k / VISIBILITY_RAYS
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        for i in range(1, samples + 1):
            d = i * resolution / 2.0
            r, c = to_rc(x0 + d * cos_a, y0 + d * sin_a)
            if not (0 <= r < rows and 0 <= c < cols):
                break
            seen[r, c] = True
            if not free[r, c]:
                break
    return seen


def first_wall_from(free: np.ndarray, to_rc, resolution: float,
                    x0: float, y0: float,
                    x1: float, y1: float) -> tuple[float, str]:
    """Distance to the first NON-free cell on the line (x0,y0) -> (x1,y1)."""
    dx, dy = x1 - x0, y1 - y0
    length = math.hypot(dx, dy)
    rows, cols = free.shape
    samples = max(1, int(length / (resolution / 2.0)))
    for i in range(1, samples + 1):
        t = i / samples
        r, c = to_rc(x0 + dx * t, y0 + dy * t)
        if not (0 <= r < rows and 0 <= c < cols):
            return t * length, 'outside grid'
        if not free[r, c]:
            return t * length, 'wall'
    return length, 'free'


def analyse_goals(name: str, models: Path, scale: float,
                  goals: list[tuple[float, float]],
                  chain: bool = False) -> dict:
    """
    Measure each goal from the spawn, or -- with `chain` -- from the previous goal.

    The distinction is not cosmetic. `MAZE11_GOALS` is a patrol: each goal is
    sent with the robot wherever the previous one left it, but they were chosen
    for spread and not for connectivity, so measuring from the spawn describes
    the first one well and the others badly. A `maze_route.py` route is the
    opposite: it only makes sense chained, and measuring it from the spawn
    invents walls the real leg never meets. Calling the wrong mode produces a
    plausible and false table.
    """
    result = maze_fit.analyse(name, models, scale, COSTMAP_RESOLUTION_M,
                              start='se')
    component, _centred, clearance, gx, gy, row0, col0, res = result['_grid']
    free = clearance > 0.0

    def to_rc(x: float, y: float) -> tuple[int, int]:
        return (int(round(row0 + y / res)), int(round(col0 + x / res)))

    def to_map(r: int, c: int) -> tuple[float, float]:
        return (float(gx[c] - gx[col0]), float(gy[r] - gy[row0]))

    # The spawn MUST be the origin of the goals' frame. If `maze_fit` ever
    # changes convention, this line stops the script instead of publishing
    # numbers measured in the wrong frame -- which would go unnoticed.
    origin = to_map(row0, col0)
    if abs(origin[0]) > 1e-9 or abs(origin[1]) > 1e-9:
        raise SystemExit(f'frame convention broke: spawn at {origin}, '
                         'expected (0, 0)')

    navigable = navigable_mask(clearance)
    if not navigable[row0, col0]:
        raise SystemExit('spawn is not navigable for robot_radius '
                         f'{ROBOT_RADIUS_M} m -- wrong pose or scale')
    labels, _ = ndimage.label(navigable)
    reachable = labels == labels[row0, col0]

    dist = geodesic_field(reachable, row0, col0, res)
    spawn_seen = visibility(free, to_rc, res, LIDAR_RANGE_M)
    views = {(row0, col0): spawn_seen}

    rows_f, cols_f = np.nonzero(free)
    coords = np.array([to_map(r, c) for r, c in zip(rows_f, cols_f)])
    in_range = np.hypot(coords[:, 0], coords[:, 1]) <= LIDAR_RANGE_M

    rows_out = []
    from_x, from_y = 0.0, 0.0
    fields = {(row0, col0): dist}
    for x, y in goals:
        r, c = to_rc(x, y)
        r_from, c_from = to_rc(from_x, from_y)
        if (r_from, c_from) not in fields:
            fields[(r_from, c_from)] = geodesic_field(
                reachable, r_from, c_from, res)
            views[(r_from, c_from)] = visibility(
                free, to_rc, res, LIDAR_RANGE_M, from_x, from_y)
        field = fields[(r_from, c_from)]
        seen = views[(r_from, c_from)]

        dx, dy = x - from_x, y - from_y
        straight = math.hypot(dx, dy)
        inside = 0 <= r < field.shape[0] and 0 <= c < field.shape[1]
        geo = float(field[r, c]) if inside else math.inf
        # The straight line is always measured from the origin of the LEG, so
        # the raycast is offset: `first_wall` walks from (0,0) to the delta, and
        # the result only holds if the origin is the spawn. For chained legs the
        # raycast has to start from the previous leg.
        wall_at, why = first_wall_from(free, to_rc, res,
                                       from_x, from_y, x, y)
        if math.isfinite(geo):
            path = descend(field, reachable, r, c)
            visible_frac = sum(1 for p in path if seen[p]) / len(path)
        else:
            path, visible_frac = [], float('nan')
        if chain:
            from_x, from_y = x, y
        rows_out.append({
            'goal': (x, y), 'straight': straight, 'geodesic': geo,
            'ratio': geo / straight if straight else math.nan,
            'wall_at': wall_at, 'why': why,
            'cells': len(path), 'visible_frac': visible_frac,
        })

    return {
        'name': name, 'scale': scale, 'resolution': res, 'chain': chain,
        'free_cells': int(free.sum()),
        'in_range_cells': int(in_range.sum()),
        'seen_cells': int((spawn_seen & free).sum()),
        'goals': rows_out,
    }


def report(data: dict) -> None:
    print(f"== {data['name']} @ scale {data['scale']}  "
          f"res {data['resolution']} m  robot_radius {ROBOT_RADIUS_M} m")
    seen_frac = 100.0 * data['seen_cells'] / max(1, data['in_range_cells'])
    print(f"   free space                  {data['free_cells']} cells")
    print(f"   within {LIDAR_RANGE_M:.0f} m of spawn       "
          f"{data['in_range_cells']} cells")
    print(f"   VISIBLE from spawn          {data['seen_cells']} cells "
          f"({seen_frac:.1f}% of what is in range)")
    print()
    origin = 'the previous goal' if data['chain'] else 'the spawn'
    print(f"   straight/geodesic/wall measured from {origin}")
    print(f"   {'goal':>16} {'straight':>8} {'geodesic':>10} {'ratio':>6} "
          f"{'1st wall':>10}  {'visible path':>12}")
    for row in data['goals']:
        x, y = row['goal']
        geo = row['geodesic']
        geo_s = f'{geo:10.2f}' if math.isfinite(geo) else f"{'unreach.':>10}"
        vis = row['visible_frac']
        vis_s = f'{100 * vis:11.1f}%' if vis == vis else f"{'-':>12}"
        print(f"   ({x:6.2f},{y:6.2f}) {row['straight']:7.2f} {geo_s} "
              f"{row['ratio']:6.2f} {row['wall_at']:8.2f} m {vis_s}")
    print()
    worst = max(r['ratio'] for r in data['goals'] if math.isfinite(r['ratio']))
    blocked = sum(1 for r in data['goals'] if r['why'] == 'wall')
    print(f"   VERDICT: {blocked} of {len(data['goals'])} goals have a wall on the "
          f"straight line; worst geodesic/straight ratio {worst:.2f}x")
    if blocked:
        print("   -> with `allow_unknown: true` and a rolling costmap without a "
              "map, the global\n"
              "      plan for these goals crosses an unobserved wall, with no error.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('maze', nargs='?', default='maze11')
    parser.add_argument('--models', type=Path,
                        default=Path.home() / 'ros_maze_worlds' / 'models')
    parser.add_argument('--scale', type=float, default=0.002,
                        help='same scale as the SDF (0.002 for maze11)')
    parser.add_argument('--goals', default='',
                        help='"x,y;x,y;..."; empty uses MAZE11_GOALS')
    parser.add_argument('--chain', action='store_true',
                        help='measure each leg from the PREVIOUS goal; use '
                             'for maze_route.py routes, never for a patrol')
    args = parser.parse_args()

    if args.goals:
        goals = [tuple(float(v) for v in pair.split(','))
                 for pair in args.goals.split(';')]
    else:
        goals = maze11_goals()

    report(analyse_goals(args.maze, args.models, args.scale, goals,
                         chain=args.chain))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
