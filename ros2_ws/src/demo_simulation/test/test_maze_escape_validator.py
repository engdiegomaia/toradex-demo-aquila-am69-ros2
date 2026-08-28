from demo_simulation.maze_escape_validator import crosses_opening


def test_accepts_outward_crossing_through_opening():
    assert crosses_opening(
        (-4.9, -0.8), (-4.9, -1.0),
        boundary_y=-0.9, opening_x=-4.9, half_width=0.6)


def test_rejects_crossing_through_wall_or_in_wrong_direction():
    assert not crosses_opening(
        (-3.0, -0.8), (-3.0, -1.0),
        boundary_y=-0.9, opening_x=-4.9, half_width=0.6)
    assert not crosses_opening(
        (-4.9, -1.0), (-4.9, -0.8),
        boundary_y=-0.9, opening_x=-4.9, half_width=0.6)
