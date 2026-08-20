"""
Unit tests for the Nav2 patrol cycle.

Only the ROS-free parts: parsing the flat waypoint parameter, the goal-radius
guard, the yaw convention, and the quaternion. Those are where a silent error
either moves the robot somewhere nobody asked for or produces a goal the Nav2
accepts and then fails on -- both expensive to diagnose from the logs.
"""

import math

from demo_bringup.patrol_commander import (
    DEFAULT_WAYPOINTS,
    flatten,
    Goal,
    MAX_GOAL_RADIUS_M,
    parse_waypoints,
    to_pose,
)
import pytest


def test_parse_reads_triples_in_order():
    goals = parse_waypoints([1.0, 2.0, 0.5, -1.0, 0.0, -0.5])
    assert goals == (Goal(1.0, 2.0, 0.5), Goal(-1.0, 0.0, -0.5))


def test_parse_rejects_length_that_is_not_a_multiple_of_three():
    # Um comprimento errado desloca todas as metas seguintes em silencio, que e
    # exatamente o que este erro impede.
    with pytest.raises(ValueError, match='multiplo de 3'):
        parse_waypoints([1.0, 2.0, 0.5, -1.0])


def test_parse_rejects_empty_waypoints():
    with pytest.raises(ValueError, match='vazio'):
        parse_waypoints([])


def test_parse_rejects_goal_outside_the_rolling_window():
    # O Nav2 ACEITARIA esta meta e falharia perto da borda da janela rolante.
    beyond = MAX_GOAL_RADIUS_M + 1.0
    with pytest.raises(ValueError, match='acima do limite'):
        parse_waypoints([beyond, 0.0, 0.0])


def test_parse_accepts_a_goal_exactly_at_the_limit():
    goals = parse_waypoints([MAX_GOAL_RADIUS_M, 0.0, 0.0])
    assert goals[0].x == MAX_GOAL_RADIUS_M


def test_flatten_round_trips_through_parse():
    assert parse_waypoints(flatten(DEFAULT_WAYPOINTS)) == DEFAULT_WAYPOINTS


def test_default_waypoints_are_inside_the_rolling_window():
    for goal in DEFAULT_WAYPOINTS:
        assert math.hypot(goal.x, goal.y) <= MAX_GOAL_RADIUS_M


def test_every_default_yaw_is_the_bearing_it_arrives_on():
    # Rumo de CHEGADA, nao de saida. Com o de saida, medido em 20/08/2026, cada
    # meta exigia 110-139 graus de giro parado (16-20 s a 0,12 rad/s) e o robo
    # derivava 0,78 m em y durante o giro, saindo da tolerancia de posicao: ele
    # chegou a 3,8 cm da meta e ela reprovou por prazo.
    count = len(DEFAULT_WAYPOINTS)
    for index in range(count):
        goal = DEFAULT_WAYPOINTS[index]
        prev = DEFAULT_WAYPOINTS[index - 1]
        bearing = math.atan2(goal.y - prev.y, goal.x - prev.x)
        assert math.isclose(math.cos(goal.yaw - bearing), 1.0, abs_tol=1e-9), (
            'meta %d pede %.1f deg, mas chega vindo de %.1f deg'
            % (index, math.degrees(goal.yaw), math.degrees(bearing)))


def test_default_waypoints_close_the_cycle():
    # O ciclo repete indefinidamente; se a ultima meta nao voltar ao inicio, cada
    # volta desloca o percurso e o robo sai da area de exposicao.
    assert (DEFAULT_WAYPOINTS[-1].x, DEFAULT_WAYPOINTS[-1].y) == (0.0, 0.0)


def test_pose_carries_the_frame_and_position():
    pose = to_pose(Goal(1.5, -2.5, 0.0), 'map', None)
    assert pose.header.frame_id == 'map'
    assert pose.pose.position.x == pytest.approx(1.5)
    assert pose.pose.position.y == pytest.approx(-2.5)


def test_pose_quaternion_is_a_pure_yaw_rotation():
    for yaw in (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0, 1.234):
        pose = to_pose(Goal(0.0, 0.0, yaw), 'map', None)
        q = pose.pose.orientation
        assert q.x == 0.0 and q.y == 0.0
        assert math.hypot(q.z, q.w) == pytest.approx(1.0)
        # Recupera o yaw do quaternion e compara pelo cosseno da diferenca, para
        # que pi e -pi contem como iguais.
        recovered = 2.0 * math.atan2(q.z, q.w)
        assert math.cos(recovered - yaw) == pytest.approx(1.0, abs=1e-9)
