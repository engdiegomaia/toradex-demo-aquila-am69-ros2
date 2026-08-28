from demo_navigation.frontier import (
    cell_to_world,
    extract_frontiers,
    Frontier,
    frontier_score,
    Grid,
    world_to_cell,
)


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
