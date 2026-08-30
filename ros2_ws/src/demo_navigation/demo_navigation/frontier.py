"""ROS-free wavefront-frontier extraction for maze exploration."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Sequence


# Valor de `nav_msgs/OccupancyGrid` para celula nao observada. E o que separa
# "livre" de "nunca visto", e por isso a fronteira e definida por ele.
UNKNOWN = -1


@dataclass(frozen=True)
class Grid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: Sequence[int]


@dataclass(frozen=True)
class Frontier:
    x: float
    y: float
    cells: int
    information_gain_m: float
    # Extra candidate points from the SAME cluster, ordered by preference,
    # spaced >= alternate_spacing_m apart. A cluster with a genuinely
    # unreachable primary point does not have to be abandoned outright: the
    # caller can retry these before giving up on the whole cluster. Empty by
    # default so every existing construction (tests included) is unaffected.
    alternates: tuple[tuple[float, float], ...] = ()


def cell_to_world(grid: Grid, col: int, row: int) -> tuple[float, float]:
    local_x = (col + 0.5) * grid.resolution
    local_y = (row + 0.5) * grid.resolution
    cosine, sine = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    return (
        grid.origin_x + cosine * local_x - sine * local_y,
        grid.origin_y + sine * local_x + cosine * local_y,
    )


def world_to_cell(grid: Grid, x: float, y: float) -> tuple[int, int] | None:
    dx, dy = x - grid.origin_x, y - grid.origin_y
    cosine, sine = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    col, row = int(math.floor(local_x / grid.resolution)), int(
        math.floor(local_y / grid.resolution))
    if 0 <= col < grid.width and 0 <= row < grid.height:
        return col, row
    return None


def _frontier_cells(grid: Grid, free_max: int) -> set[tuple[int, int]]:
    """
    Return free cells that touch an unknown cell in any of the 8 directions.

    Written around the UNKNOWN cells rather than around every cell, because the
    unknown border is a thin curve while the grid is an area. Measured on an x86
    host over SLAM maps of the size this demo produces, against the previous
    per-cell neighbour generator:

        234 x 284,  35% explorado    51,9 ms -> 6,7 ms
        234 x 284,  95% explorado   146,0 ms -> 2,5 ms
        400 x 400,  95% explorado   322,0 ms -> 4,5 ms

    Same output, cell for cell -- the equality is asserted in the tests. The
    old version paid a generator object plus four bounds checks per neighbour
    for every cell in the map, which is where the whole cost lived.

    Note the shape of the win: the old cost GREW as the map filled in, because
    more cells passed the free test and reached the neighbour scan. This one
    SHRINKS, because the unknown border retreats as exploration proceeds. That
    matters on the module, where the worst moment used to be the late maze.
    """
    data, width, height = grid.data, grid.width, grid.height

    # Per row, the columns lying within one column of an unknown cell.
    dilated: list[set[int]] = []
    for base in range(0, width * height, width):
        near: set[int] = set()
        for col, value in enumerate(data[base:base + width]):
            if value != UNKNOWN:
                continue
            near.add(col)
            if col:
                near.add(col - 1)
            if col + 1 < width:
                near.add(col + 1)
        dilated.append(near)

    empty: set[int] = set()
    cells: set[tuple[int, int]] = set()
    for row in range(height):
        # Unioning three consecutive rows covers the vertical and the diagonal
        # neighbours at once; each row's set is already widened horizontally.
        near = dilated[row] \
            | (dilated[row - 1] if row else empty) \
            | (dilated[row + 1] if row + 1 < height else empty)
        base = row * width
        for col in near:
            if 0 <= data[base + col] <= free_max:
                cells.add((col, row))
    return cells


def extract_frontiers(
    grid: Grid,
    *,
    free_max: int = 20,
    occupied_min: int = 65,
    min_cells: int = 8,
    clearance_m: float = 0.45,
    standoff_m: float = 0.45,
    max_alternates: int = 2,
    alternate_spacing_m: float = 0.25,
    stats: dict | None = None,
) -> list[Frontier]:
    """
    Cluster free cells touching unknown and return safe inward goals.

    `stats`, when given a dict, is filled with `raw_clusters` (cluster count
    before the clearance/standoff candidate search) and `clusters_with_candidate`
    (how many of those actually produced a usable goal point) — telemetry to
    tell "only one cluster ever existed" apart from "several existed and the
    filters ate the rest", which look identical from the returned list alone.
    """
    if grid.width <= 0 or grid.height <= 0 or grid.resolution <= 0.0:
        if stats is not None:
            stats['raw_clusters'] = 0
            stats['clusters_with_candidate'] = 0
        return []
    if len(grid.data) != grid.width * grid.height:
        if stats is not None:
            stats['raw_clusters'] = 0
            stats['clusters_with_candidate'] = 0
        return []

    def index(col: int, row: int) -> int:
        return row * grid.width + col

    def neighbours(col: int, row: int):
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dc == 0 and dr == 0:
                    continue
                nc, nr = col + dc, row + dr
                if 0 <= nc < grid.width and 0 <= nr < grid.height:
                    yield nc, nr

    frontier_cells = _frontier_cells(grid, free_max)

    clusters: list[list[tuple[int, int]]] = []
    unseen = set(frontier_cells)
    while unseen:
        seed = unseen.pop()
        cluster = [seed]
        queue = deque([seed])
        while queue:
            col, row = queue.popleft()
            for neighbour in neighbours(col, row):
                if neighbour in unseen:
                    unseen.remove(neighbour)
                    cluster.append(neighbour)
                    queue.append(neighbour)
        if len(cluster) >= min_cells:
            clusters.append(cluster)

    if stats is not None:
        stats['raw_clusters'] = len(clusters)

    clearance_cells = max(1, math.ceil(clearance_m / grid.resolution))
    standoff_cells = max(1, round(standoff_m / grid.resolution))
    # Extra BFS depth past the standoff band so there is room for alternates
    # at a genuinely different depth, not just the same ring as the primary
    # point. Bounded, not unlimited: deep alternates would drift the goal
    # away from the frontier's actual information gain.
    alt_depth_cells = max(2, round(2.0 * alternate_spacing_m / grid.resolution))

    def has_clearance(col: int, row: int) -> bool:
        for dr in range(-clearance_cells, clearance_cells + 1):
            for dc in range(-clearance_cells, clearance_cells + 1):
                if dc * dc + dr * dr > clearance_cells * clearance_cells:
                    continue
                nc, nr = col + dc, row + dr
                if not (0 <= nc < grid.width and 0 <= nr < grid.height):
                    return False
                if grid.data[index(nc, nr)] >= occupied_min:
                    return False
        return True

    result: list[Frontier] = []
    for cluster in clusters:
        centre_col = sum(cell[0] for cell in cluster) / len(cluster)
        centre_row = sum(cell[1] for cell in cluster) / len(cluster)
        queue = deque((cell, 0) for cell in cluster)
        visited = set(cluster)
        candidates: list[tuple[int, int]] = []
        while queue:
            (col, row), distance = queue.popleft()
            if distance >= standoff_cells and has_clearance(col, row):
                candidates.append((col, row))
                continue
            if distance >= standoff_cells + 2 + alt_depth_cells:
                continue
            for nc, nr in neighbours(col, row):
                if (nc, nr) in visited:
                    continue
                if not 0 <= grid.data[index(nc, nr)] <= free_max:
                    continue
                visited.add((nc, nr))
                queue.append(((nc, nr), distance + 1))
        if not candidates:
            continue
        goal_col, goal_row = min(
            candidates,
            key=lambda cell: (
                (cell[0] - centre_col) ** 2 + (cell[1] - centre_row) ** 2,
                cell[1], cell[0],
            ),
        )
        x, y = cell_to_world(grid, goal_col, goal_row)

        # Alternates: the remaining candidates, nearest-to-centroid first,
        # greedily kept only if they sit >= alternate_spacing_m from the
        # primary point AND from every alternate already chosen. Spacing
        # matters more than raw proximity here -- three points crammed into
        # the same corner are one retry, not three.
        alternates: list[tuple[float, float]] = []
        chosen_points = [(x, y)]
        remaining_cells = sorted(
            (cell for cell in candidates if cell != (goal_col, goal_row)),
            key=lambda cell: (
                (cell[0] - centre_col) ** 2 + (cell[1] - centre_row) ** 2,
                cell[1], cell[0],
            ),
        )
        for cell in remaining_cells:
            if len(alternates) >= max_alternates:
                break
            wx, wy = cell_to_world(grid, *cell)
            if all(math.hypot(wx - px, wy - py) >= alternate_spacing_m
                   for px, py in chosen_points):
                alternates.append((wx, wy))
                chosen_points.append((wx, wy))

        result.append(Frontier(
            x=x,
            y=y,
            cells=len(cluster),
            information_gain_m=len(cluster) * grid.resolution,
            alternates=tuple(alternates),
        ))
    if stats is not None:
        stats['clusters_with_candidate'] = len(result)
    return sorted(result, key=lambda item: (-item.cells, item.y, item.x))


def path_length(poses) -> float:
    points = [(pose.pose.position.x, pose.pose.position.y) for pose in poses]
    return sum(math.hypot(x1 - x0, y1 - y0)
               for (x0, y0), (x1, y1) in zip(points, points[1:]))


def frontier_score(frontier: Frontier, route_m: float) -> float:
    """
    Higher is better; route cost still dominates tiny/noisy frontier clusters.

    The route penalty is `sqrt(route_m)`, not `route_m` itself. A linear
    penalty makes anything past a few meters permanently uncompetitive
    against any closer cluster regardless of its own size, which starves
    real exploration progress once the near, already-partly-explored rooms
    are gone: R9/R11 (29/08) both got pulled the same direction repeatedly
    and never pushed further down the one corridor that led toward the
    actual exit, because its clusters were both smaller AND farther and a
    linear penalty compounds those two disadvantages instead of just adding
    them. A sub-linear penalty keeps the same ordering for comparable
    distances (closer still wins, all else equal) while letting a distant
    cluster's own size matter again once the cheap options run out.
    """
    return frontier.information_gain_m - 0.5 * math.sqrt(route_m)
