#!/usr/bin/env python3
"""
Measures whether a ros_maze_worlds maze suits the Go2, and where to spawn in it.

Runs on the x86 host, offline, without ROS or Gazebo: it reads the STL
directly. Read-only.

    python3 tools/maze/maze_fit.py --models /path/to/ros_maze_worlds/models maze11
    python3 tools/maze/maze_fit.py --models ... maze10 maze11 --scale 0.002

It exists because the scale and pose of each maze are MEASURED numbers, and
without this script they would go back to being guesses. Reusing one maze's
pose in another puts the robot inside a wall, and Gazebo does not complain.

WHAT IT MEASURES, AND WHY EACH ITEM MATTERS

- **Corridor width.** The Go2 trunk is 0.70 x 0.31 m, circumscribed radius
  0.383 m, so it needs 0.77 m to pass while turning. At the upstream 0.001
  scale NO cell of the mazes fits the robot.
- **Wall height.** The L1 lidar sits ~0.306 m above the floor. A wall at the
  exact height of the scan plane goes in and out of the scan as the trunk
  oscillates, and that looks like a bridge defect, not geometry.
- **Navigable area in ONE connected component.** This is the number that
  decides whether a patrol is possible: two large pockets separated by a
  narrow corridor add up to area and are useless.
- **Spawn pose.** Chosen where there is the longest free run along +x (the
  first thing the robot does is walk forward) WITH corridor-centre clearance.
  Maximising the run alone pushes the robot into a corner.

HOW THE WALL FOOTPRINT IS OBTAINED

The mazes are extruded wall boxes. The horizontal faces of the STL are the top
and bottom of those boxes, so their projection onto XY is exactly the
footprint. Rasterising only the horizontal faces avoids having to close the
solid.

The default scale is 0.002, twice the upstream, because of the two
measurements above. See the header of
`demo_simulation/worlds/quadruped_maze.sdf` and
`docs/results/ml35-labirinto.md`.
"""

import argparse
import math
from pathlib import Path
import struct
import sys

import numpy as np
from scipy import ndimage


# Circumscribed radius of the Go2 trunk (0.70 x 0.31 m). The robot only fits
# through gaps larger than twice this.
TRUNK_RADIUS_M = 0.383

# Minimum clearance to consider a cell "corridor centre". A 1.20 m corridor has
# half-width 0.60; 0.55 accepts the centre and rejects anything hugging a
# wall.
CORRIDOR_CENTRE_CLEARANCE_M = 0.55

# Rasterisation resolution. 0.01 m resolves the 0.40 m thick wall and the
# 0.217 m margin at no relevant cost on these grids.
DEFAULT_RESOLUTION_M = 0.01


def load_triangles(path: Path) -> np.ndarray:
    """Read a binary or ASCII STL and return (n, 3, 3) vertices."""
    data = path.read_bytes()
    if data[:5].lower() == b'solid' and b'facet' in data[:2000]:
        vertices = [
            [float(x) for x in parts[1:4]]
            for parts in (line.split() for line in
                          data.decode('utf-8', 'ignore').splitlines())
            if parts[:1] == ['vertex']
        ]
        return np.asarray(vertices, dtype=float).reshape(-1, 3, 3)

    count = struct.unpack('<I', data[80:84])[0]
    out = np.zeros((count, 3, 3))
    for i in range(count):
        base = 84 + i * 50 + 12
        for j in range(3):
            out[i, j] = struct.unpack('<3f', data[base + j * 12:base + j * 12 + 12])
    return out


def wall_footprint(triangles: np.ndarray, resolution: float):
    """Rasterise the horizontal faces in XY. Returns (free, gx, gy, low, high)."""
    flat = triangles.reshape(-1, 3)
    low, high = flat.min(axis=0), flat.max(axis=0)
    nx = int(np.ceil((high[0] - low[0]) / resolution)) + 1
    ny = int(np.ceil((high[1] - low[1]) / resolution)) + 1
    gx = low[0] + (np.arange(nx) + 0.5) * resolution
    gy = low[1] + (np.arange(ny) + 0.5) * resolution
    mesh_x, mesh_y = np.meshgrid(gx, gy)

    occupied = np.zeros((ny, nx), dtype=bool)
    for triangle in triangles:
        normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
        if abs(normal[2]) < 1e-9:
            continue  # vertical face: it is the side of the wall, not the footprint
        a, b, c = triangle[:, 0:2]
        denominator = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(denominator) < 1e-12:
            continue
        bary_a = ((b[1] - c[1]) * (mesh_x - c[0])
                  + (c[0] - b[0]) * (mesh_y - c[1])) / denominator
        bary_b = ((c[1] - a[1]) * (mesh_x - c[0])
                  + (a[0] - c[0]) * (mesh_y - c[1])) / denominator
        inside = (bary_a >= -1e-9) & (bary_b >= -1e-9) & (1 - bary_a - bary_b >= -1e-9)
        occupied |= inside

    return ~occupied, gx, gy, low, high


def corridor_widths(free: np.ndarray, resolution: float, span: float) -> np.ndarray:
    """Contiguous free-space gaps, in both grid directions."""
    runs = []
    for grid in (free, free.T):
        for row in grid:
            length = 0
            for cell in row:
                if cell:
                    length += 1
                elif length:
                    runs.append(length * resolution)
                    length = 0
            if length:
                runs.append(length * resolution)
    widths = np.asarray(runs)
    # Drop sub-cell noise and the gap outside the maze, which is not a corridor.
    return widths[(widths > 0.05) & (widths < span * 0.5)]


def free_run_east(component: np.ndarray, resolution: float) -> np.ndarray:
    """Contiguous free run along +x from each cell of the component."""
    run = np.zeros(component.shape, dtype=float)
    for row in range(component.shape[0]):
        length = 0.0
        for col in range(component.shape[1] - 1, -1, -1):
            length = length + resolution if component[row, col] else 0.0
            run[row, col] = length
    return run


# Corners of the bounding box, as (sign in x, sign in y). "se" is the
# bottom-right corner seen from above with x to the right and y up.
CORNERS = {'se': (+1, -1), 'ne': (+1, +1), 'nw': (-1, +1), 'sw': (-1, -1)}

# Exit directions, with the corresponding spawn yaw in degrees.
HEADINGS = (('+x', 0, 1, 0), ('+y', 90, 0, 1),
            ('-x', 180, -1, 0), ('-y', -90, 0, -1))


def run_from(component, row, col, step_row, step_col, resolution) -> float:
    """Contiguous free lane from (row, col) in the given direction."""
    length = 0.0
    r, c = row + step_row, col + step_col
    while (0 <= r < component.shape[0] and 0 <= c < component.shape[1]
            and component[r, c]):
        length += resolution
        r += step_row
        c += step_col
    return length


def exits(component, row, col, resolution) -> list:
    """
    Free lane in the four directions, sorted from longest to shortest.

    It exists because the pose alone is not enough: the robot spawns with yaw
    0, facing +x, and the first thing it does is walk forward. In a corner of
    the maze +x is usually a wall -- and turning in place is exactly what this
    robot does worst (yaw ceiling of 0.13 rad/s). Spawning facing the exit is
    free: `quadruped.launch.py` accepts `yaw:=`.
    """
    measured = [
        (name, yaw, run_from(component, row, col, dy, dx, resolution))
        for name, yaw, dx, dy in HEADINGS
    ]
    return sorted(measured, key=lambda item: -item[2])


def choose_start(centred, run, gx, gy, low, high, mode: str):
    """
    Choose the spawn cell: longest run along +x, or a corner.

    `run` maximises free lane ahead, which is good for a straight-gait trial.
    A corner is what you want in a demonstration: the robot starts at one end
    and crosses the whole maze, instead of spawning in the middle of it.

    In either case the cell comes from `centred`, i.e. with corridor-centre
    clearance. Without it the robot spawns against a wall and the first step
    already scrapes.
    """
    if mode == 'run':
        return np.unravel_index(np.argmax(run * centred), run.shape)

    sign_x, sign_y = CORNERS[mode]
    target_x = high[0] if sign_x > 0 else low[0]
    target_y = high[1] if sign_y > 0 else low[1]
    rows, cols = np.nonzero(centred)
    if rows.size == 0:
        return np.unravel_index(np.argmax(run * centred), run.shape)
    reach = np.hypot(gx[cols] - target_x, gy[rows] - target_y)
    nearest = int(np.argmin(reach))
    return rows[nearest], cols[nearest]


def analyse(name: str, models: Path, scale: float, resolution: float,
            start: str = 'run') -> dict:
    """Measure a maze and return the verdict plus the recommended pose."""
    stl = models / name / 'meshes' / f'{name}.stl'
    if not stl.is_file():
        raise FileNotFoundError(stl)

    triangles = load_triangles(stl) * scale
    free, gx, gy, low, high = wall_footprint(triangles, resolution)
    span_x, span_y = high[0] - low[0], high[1] - low[1]

    widths = corridor_widths(free, resolution, span_x)
    clearance = ndimage.distance_transform_edt(free) * resolution

    fits = free & (clearance >= TRUNK_RADIUS_M)
    labels, count = ndimage.label(fits)
    if count == 0:
        return {
            'name': name, 'scale': scale, 'span': (span_x, span_y),
            'wall_height': high[2] - low[2], 'widths': widths,
            'components': 0, 'area': 0.0, 'pose': None,
        }
    sizes = ndimage.sum(np.ones_like(labels), labels, range(1, count + 1))
    component = labels == 1 + int(np.argmax(sizes))

    run = free_run_east(component, resolution)
    centred = component & (clearance >= CORRIDOR_CENTRE_CLEARANCE_M)
    row, col = choose_start(centred, run, gx, gy, low, high, start)

    rows, cols = np.nonzero(component)
    return {
        'name': name, 'scale': scale, 'span': (span_x, span_y), 'start': start,
        'wall_height': high[2] - low[2], 'widths': widths,
        'components': count, 'area': component.sum() * resolution ** 2,
        'pose': (-gx[col], -gy[row]),
        'run': run[row, col], 'clearance': clearance[row, col],
        'extent_x': (gx[cols.min()] - gx[col], gx[cols.max()] - gx[col]),
        'extent_y': (gy[rows.min()] - gy[row], gy[rows.max()] - gy[row]),
        'exits': exits(component, row, col, resolution),
        '_grid': (component, centred, clearance, gx, gy, row, col, resolution),
    }


def pick_goals(result: dict, count: int, max_radius: float,
               min_radius: float = 2.0) -> list:
    """
    Choose goals at corridor centres, spread out and within the radius.

    A goal on top of a wall is ACCEPTED by Nav2 and fails later, near the edge,
    where the error no longer has a name -- so it is chosen here, on the same
    grid that decided the pose, and not by eye in RViz.

    `max_radius` exists because patrol_commander rejects goals beyond
    MAX_GOAL_RADIUS_M (8.0 m) and the global costmap is a rolling window.
    """
    component, centred, clearance, gx, gy, row0, col0, resolution = result['_grid']
    rows, cols = np.nonzero(centred)
    if rows.size == 0:
        return []

    # Coordinates in the ROBOT frame: it spawns at cell (row0, col0).
    points = np.stack([gx[cols] - gx[col0], gy[rows] - gy[row0]], axis=1)
    reach = np.hypot(points[:, 0], points[:, 1])
    # min_radius: a goal less than 2 m from the start is not a crossing, and in
    # a patrol it becomes a stop that measures nothing.
    keep = (reach <= max_radius) & (reach >= min_radius)
    points, scores = points[keep], clearance[rows, cols][keep]
    if points.size == 0:
        return []

    # Greedy by distance: take the point farthest from those already chosen, so
    # the patrol crosses the maze instead of circling in one room.
    chosen = [points[int(np.argmax(reach[keep]))]]
    while len(chosen) < count:
        spread = np.min(
            [np.hypot(points[:, 0] - c[0], points[:, 1] - c[1]) for c in chosen],
            axis=0)
        candidate = int(np.argmax(spread + 0.05 * scores))
        if spread[candidate] < 1.5:
            break
        chosen.append(points[candidate])
    return [(float(x), float(y)) for x, y in chosen]


def report(result: dict, goals: int = 0, max_radius: float = 8.0,
           min_radius: float = 2.0) -> bool:
    """Print the assessment and return True if the maze is usable."""
    name, scale = result['name'], result['scale']
    span_x, span_y = result['span']
    widths = result['widths']
    needed = 2 * TRUNK_RADIUS_M

    print(f'== {name} @ scale {scale}   start "{result.get("start", "run")}"')
    print(f'   footprint               {span_x:.2f} x {span_y:.2f} m')
    print(f'   wall height             {result["wall_height"]:.2f} m'
          '   (lidar L1 a ~0.306 m)')
    if widths.size:
        median = float(np.median(widths))
        print(f'   median corridor         {median:.2f} m'
              f'   (needs {needed:.2f} m)')
        print(f'   margin per side         {(median - needed) / 2 * 100:.1f} cm')
    print(f'   navigable area          {result["area"]:.1f} m2'
          f'   in {result["components"]} component(s)')

    if result['pose'] is None:
        print('   VERDICT: UNSUITABLE -- no cell fits the robot at this scale')
        return False

    pose_x, pose_y = result['pose']
    print(f'   clearance at spawn      {result["clearance"]:.2f} m')
    print('   free lane per direction  ' + '  '.join(
        f'{name}={length:.2f}m' for name, _, length in result['exits']))
    best_name, best_yaw, best_run = result['exits'][0]
    print(f'   >>> yaw:={math.radians(best_yaw):.4f}'
          f'   ({best_yaw:+d} deg, exit {best_name}, {best_run:.2f} m free)')
    print(f'   extent (robot coords)   x[{result["extent_x"][0]:+.2f}, '
          f'{result["extent_x"][1]:+.2f}]  '
          f'y[{result["extent_y"][0]:+.2f}, {result["extent_y"][1]:+.2f}]')
    print(f'   >>> <pose>{pose_x:.3f} {pose_y:.3f} 0 0 0 0</pose>')

    # The wall has to clear the scan plane, not sit in it.
    ok = result['wall_height'] > 0.40 and (
        not widths.size or float(np.median(widths)) >= needed)
    print(f'   VERDICT: {"SUITABLE" if ok else "UNSUITABLE"}')

    if goals and '_grid' in result:
        picked = pick_goals(result, goals, max_radius, min_radius)
        print(f'   goals at corridor centres, robot coords, radius <= '
              f'{max_radius:.1f} m:')
        for index, (x, y) in enumerate(picked):
            print(f'      {index}: x={x:+.2f} y={y:+.2f}'
                  f'   ({np.hypot(x, y):.2f} m from origin)')
        if len(picked) < goals:
            print(f'      (only {len(picked)} of {goals} requested fit spread out '
                  'within the radius)')
        flat = ', '.join(f'{x:.2f}, {y:.2f}, 0.0' for x, y in picked)
        print(f'   waypoints:="[{flat}]"')

    return ok


def main() -> int:
    """Measure one or more mazes and return 1 if any is unsuitable."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mazes', nargs='+', help='names, e.g. maze10 maze11')
    parser.add_argument('--models', type=Path,
                        default=Path('/tmp/ros_maze_worlds/models'),
                        help='models/ directory of the ros_maze_worlds clone')
    parser.add_argument('--scale', type=float, default=0.002,
                        help='uniform scale applied to the STL (default 0.002)')
    parser.add_argument('--resolution', type=float, default=DEFAULT_RESOLUTION_M,
                        help='rasterisation resolution in m')
    parser.add_argument('--start', default='run',
                        choices=['run'] + sorted(CORNERS),
                        help='where the robot spawns: "run" = longest free lane '
                             'along +x; "se"/"ne"/"nw"/"sw" = maze corner '
                             '(se = bottom right)')
    parser.add_argument('--goals', type=int, default=0, metavar='N',
                        help='also suggest N goals at corridor centres, '
                             'ready for the patrol_commander waypoints:=')
    parser.add_argument('--min-goal-radius', type=float, default=2.0,
                        help='goals closer than this to the start are '
                             'discarded: they are not a crossing')
    parser.add_argument('--max-goal-radius', type=float, default=8.0,
                        help='maximum radius of the suggested goals; matches '
                             'MAX_GOAL_RADIUS_M in patrol_commander')
    args = parser.parse_args()

    if not args.models.is_dir():
        print(f'--models={args.models} is not a directory. Clone '
              'ros_maze_worlds and point to its models/ subdirectory.',
              file=sys.stderr)
        return 2

    failures = 0
    for maze in args.mazes:
        try:
            measured = analyse(maze, args.models, args.scale, args.resolution,
                               args.start)
            if not report(measured, args.goals, args.max_goal_radius,
                          args.min_goal_radius):
                failures += 1
        except FileNotFoundError as missing:
            print(f'{maze}: STL not found at {missing}', file=sys.stderr)
            failures += 1
        print()
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
