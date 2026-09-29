import random

from demo_navigation.frontier import (
    _frontier_cells,
    cell_to_world,
    extract_frontiers,
    Frontier,
    frontier_score,
    Grid,
    UNKNOWN,
    world_to_cell,
)


def _frontier_cells_by_definition(grid, free_max=20):
    """
    Aplica a definicao literal: celula livre com um dos 8 vizinhos desconhecido.

    E a implementacao que `_frontier_cells` substituiu, mantida aqui como
    oraculo. A versao rapida trabalha em torno da borda desconhecida em vez de
    varrer celula a celula, e a unica coisa que autoriza essa troca e as duas
    concordarem exatamente.
    """
    cells = set()
    for row in range(grid.height):
        for col in range(grid.width):
            if not 0 <= grid.data[row * grid.width + col] <= free_max:
                continue
            for delta_row in (-1, 0, 1):
                for delta_col in (-1, 0, 1):
                    near_col, near_row = col + delta_col, row + delta_row
                    if delta_col == 0 and delta_row == 0:
                        continue
                    if not (0 <= near_col < grid.width
                            and 0 <= near_row < grid.height):
                        continue
                    if grid.data[near_row * grid.width + near_col] == UNKNOWN:
                        cells.add((col, row))
    return cells


def test_cell_world_round_trip_with_rotated_origin():
    grid = Grid(20, 20, 0.1, -2.0, 3.0, 0.4, [0] * 400)
    world = cell_to_world(grid, 7, 12)
    assert world_to_cell(grid, *world) == (7, 12)


def test_extracts_cluster_and_places_goal_in_known_free_space():
    width, height = 40, 24
    data = [-1] * (width * height)
    for row in range(2, 22):
        for col in range(2, 21):
            data[row * width + col] = 0
    grid = Grid(width, height, 0.05, 0.0, 0.0, 0.0, data)
    frontiers = extract_frontiers(grid)
    assert len(frontiers) == 1
    candidate = frontiers[0]
    cell = world_to_cell(grid, candidate.x, candidate.y)
    assert cell is not None
    assert data[cell[1] * width + cell[0]] == 0
    assert candidate.cells >= 8


def test_rejects_frontier_without_robot_clearance():
    width, height = 30, 20
    data = [-1] * (width * height)
    for row in range(1, 19):
        for col in range(1, 16):
            data[row * width + col] = 0
    # Occupied stripe leaves less than the required 0.45 m clearance.
    for row in range(height):
        data[row * width + 8] = 100
    grid = Grid(width, height, 0.05, 0.0, 0.0, 0.0, data)
    assert extract_frontiers(grid) == []


def test_a_large_open_cluster_offers_spaced_out_alternates():
    """
    A wide room gives room for backup points, not just one dead end.

    R10 (29/08) lost its only cluster over one unreachable point because
    `extract_frontiers` only ever offered one. On a room big enough to have
    real interior space, alternates should exist and sit apart from each
    other and from the primary point by at least `alternate_spacing_m`.

    The 40x24 room used by the other tests here is too tight for this: with
    the default clearance_m/standoff_m both needing a 9-cell margin, only a
    1-col x 2-row sliver of cells satisfies both at once (confirmed by
    direct inspection of extract_frontiers' internal candidate list), which
    is by construction too small to hold a second point 0.25 m away. This
    needs a room with real interior depth, not just any cluster.
    """
    width, height = 80, 60
    data = [-1] * (width * height)
    for row in range(5, 55):
        for col in range(5, 75):
            data[row * width + col] = 0
    grid = Grid(width, height, 0.05, 0.0, 0.0, 0.0, data)
    frontiers = extract_frontiers(
        grid, max_alternates=2, alternate_spacing_m=0.25)
    assert len(frontiers) == 1
    candidate = frontiers[0]
    assert len(candidate.alternates) >= 1
    points = [(candidate.x, candidate.y), *candidate.alternates]
    for i, (x0, y0) in enumerate(points):
        for x1, y1 in points[i + 1:]:
            assert ((x0 - x1) ** 2 + (y0 - y1) ** 2) ** 0.5 >= 0.25 - 1e-9


def test_a_thin_cluster_offers_no_alternates_without_crashing():
    """A room with only one candidate cell still returns cleanly."""
    width, height = 40, 24
    data = [-1] * (width * height)
    for row in range(2, 22):
        for col in range(2, 21):
            data[row * width + col] = 0
    grid = Grid(width, height, 0.05, 0.0, 0.0, 0.0, data)
    frontiers = extract_frontiers(
        grid, max_alternates=2, alternate_spacing_m=100.0)
    assert len(frontiers) == 1
    # No spacing this large can be satisfied twice in a room this size, so no
    # alternate qualifies -- the important part is this does not crash.
    assert frontiers[0].alternates == ()


def test_stats_reports_raw_clusters_separately_from_filtered_candidates():
    """
    Distinguish "only one cluster ever existed" from "the filter ate them".

    `frontier_clusters` alone cannot tell "only one existed" from "several
    existed and the candidate filters ate the rest" -- they look identical.
    `stats` exists so a caller can tell them apart.
    """
    width, height = 40, 24
    data = [-1] * (width * height)
    for row in range(2, 22):
        for col in range(2, 21):
            data[row * width + col] = 0
    grid = Grid(width, height, 0.05, 0.0, 0.0, 0.0, data)
    stats: dict = {}
    frontiers = extract_frontiers(grid, stats=stats)
    assert len(frontiers) == 1
    assert stats['raw_clusters'] == 1
    assert stats['clusters_with_candidate'] == 1

    # An impossible clearance eats the only candidate; the raw cluster count
    # must still show it existed before the filter ran.
    stats = {}
    assert extract_frontiers(grid, clearance_m=5.0, stats=stats) == []
    assert stats['raw_clusters'] == 1
    assert stats['clusters_with_candidate'] == 0


def test_frontier_score_trades_information_for_route_length():
    rich = Frontier(0.0, 0.0, 30, 1.5)
    small = Frontier(0.0, 0.0, 10, 0.5)
    assert frontier_score(rich, 2.0) > frontier_score(small, 2.0)
    assert frontier_score(rich, 5.0) < frontier_score(rich, 1.0)


def test_frontier_score_does_not_crush_a_small_but_far_cluster() -> None:
    """
    A linear route penalty used to starve the one real path to the exit.

    R9/R11 (29/08): the corridor toward the actual exit had clusters that
    were both smaller AND farther than the rooms back the way the robot
    came. A linear penalty compounds those two disadvantages instead of
    just adding them, so the far cluster never became competitive even
    once the near ones were fully explored.

    Reproduces R11's actual numbers: a 50-cell cluster 3.15 m away vs. a
    16-cell cluster 8.46 m away. The old linear `-0.5*route_m` penalty
    scored these 0.87 and -3.43 (a 4.3-point gap); the sub-linear penalty
    still prefers the closer one, but by far less.
    """
    close_big = Frontier(0.0, 0.0, 50, 50 * 0.05)
    far_small = Frontier(0.0, 0.0, 16, 16 * 0.05)
    close_score = frontier_score(close_big, 3.152)
    far_score = frontier_score(far_small, 8.455)
    assert close_score > far_score
    assert close_score - far_score < 2.5


def test_fast_sweep_agrees_with_the_literal_definition_on_random_grids():
    """
    The fast sweep can only replace the slow one if it gives the SAME set.

    Random grids with a fixed seed, including degenerate cases of a single
    column and a single row -- that is where the fast version could get it
    wrong, because it unions the row above and the row below and has to
    handle the edges.
    """
    rng = random.Random(20260828)
    shapes = [(1, 1), (1, 12), (12, 1), (2, 2), (7, 5), (23, 19), (40, 40)]
    for width, height in shapes:
        for _ in range(6):
            data = [rng.choice([UNKNOWN, 0, 0, 20, 60, 100])
                    for _ in range(width * height)]
            grid = Grid(width, height, 0.05, 0.0, 0.0, 0.0, data)
            assert _frontier_cells(grid, 20) == \
                _frontier_cells_by_definition(grid, 20), (width, height)


def test_fast_sweep_handles_grids_with_no_unknown_and_all_unknown():
    """A fully known map has no frontier; a fully unknown one doesn't either."""
    known = Grid(9, 7, 0.05, 0.0, 0.0, 0.0, [0] * 63)
    blank = Grid(9, 7, 0.05, 0.0, 0.0, 0.0, [UNKNOWN] * 63)

    assert _frontier_cells(known, 20) == set()
    assert _frontier_cells(blank, 20) == set()


def test_only_free_cells_can_become_frontier():
    """An occupied cell next to the unknown is NOT a frontier -- it's a wall."""
    # A single row, to make the neighborhood obvious.
    occupied_next_to_unknown = Grid(3, 1, 0.05, 0.0, 0.0, 0.0, [0, 80, UNKNOWN])
    free_next_to_unknown = Grid(3, 1, 0.05, 0.0, 0.0, 0.0, [UNKNOWN, 0, 0])

    assert _frontier_cells(occupied_next_to_unknown, 20) == set()
    assert _frontier_cells(free_next_to_unknown, 20) == {(1, 0)}
