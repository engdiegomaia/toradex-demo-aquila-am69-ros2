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


def test_frontier_score_trades_information_for_route_length():
    rich = Frontier(0.0, 0.0, 30, 1.5)
    small = Frontier(0.0, 0.0, 10, 0.5)
    assert frontier_score(rich, 2.0) > frontier_score(small, 2.0)
    assert frontier_score(rich, 5.0) < frontier_score(rich, 1.0)


def test_fast_sweep_agrees_with_the_literal_definition_on_random_grids():
    """
    A varredura rapida so pode substituir a lenta se der o MESMO conjunto.

    Grades aleatorias com semente fixa, incluindo casos degenerados de uma
    coluna e de uma linha -- e onde a versao rapida poderia errar, porque ela
    une a linha de cima e a de baixo e precisa tratar as bordas.
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
    """Mapa totalmente conhecido nao tem fronteira; totalmente desconhecido tambem nao."""
    known = Grid(9, 7, 0.05, 0.0, 0.0, 0.0, [0] * 63)
    blank = Grid(9, 7, 0.05, 0.0, 0.0, 0.0, [UNKNOWN] * 63)

    assert _frontier_cells(known, 20) == set()
    assert _frontier_cells(blank, 20) == set()


def test_only_free_cells_can_become_frontier():
    """Celula ocupada encostada no desconhecido NAO e fronteira -- e parede."""
    # Uma linha so, para deixar a vizinhanca obvia.
    occupied_next_to_unknown = Grid(3, 1, 0.05, 0.0, 0.0, 0.0, [0, 80, UNKNOWN])
    free_next_to_unknown = Grid(3, 1, 0.05, 0.0, 0.0, 0.0, [UNKNOWN, 0, 0])

    assert _frontier_cells(occupied_next_to_unknown, 20) == set()
    assert _frontier_cells(free_next_to_unknown, 20) == {(1, 0)}
