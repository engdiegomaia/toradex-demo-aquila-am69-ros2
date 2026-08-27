"""Guards for ``scripts/maze_geodesic.py``.

The script exists to answer one question — is the goal reachable by the straight
line the planner would draw? — and the two ways it can lie are both silent:

* it reads the maze in one coordinate convention and the goals in another, and
  prints a plausible table measured in the wrong frame;
* it measures a *sequential* route from the spawn instead of from the previous
  leg, inventing walls that no leg actually meets.

Both are guarded here. The STL lives outside the repository (see
``demo_simulation/scenarios.py``, ``external_models``), so the tests that need
geometry skip when it is absent instead of failing on a clean checkout.
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
MODELS = Path.home() / 'ros_maze_worlds' / 'models'
MAZE11_STL = MODELS / 'maze11' / 'meshes' / 'maze11.stl'

sys.path.insert(0, str(SCRIPTS))

needs_mesh = pytest.mark.skipif(
    not MAZE11_STL.is_file(),
    reason=f'malha externa ausente: {MAZE11_STL}')


@pytest.fixture(scope='module')
def geodesic():
    return pytest.importorskip('maze_geodesic')


def test_goals_are_read_from_nav_trial_not_copied(geodesic):
    """A tupla tem uma fonte so.

    Duplicar ``MAZE11_GOALS`` aqui ou no script cria duas verdades que divergem
    no dia em que alguem regenerar as metas com ``maze_fit.py`` -- e a tabela
    continuaria imprimindo, medindo metas que o ensaio nao manda mais.
    """
    source = (SCRIPTS / 'nav_trial.py').read_text(encoding='utf-8')
    literal = None
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Assign):
            if any(getattr(t, 'id', None) == 'MAZE11_GOALS'
                   for t in node.targets):
                literal = [tuple(v) for v in ast.literal_eval(node.value)]
    assert literal, 'MAZE11_GOALS sumiu de nav_trial.py'
    assert geodesic.maze11_goals() == literal


def test_reading_goals_does_not_require_ros(geodesic):
    """O script roda offline. Importar ``nav_trial`` puxaria geometry_msgs."""
    assert 'nav_trial' not in sys.modules
    geodesic.maze11_goals()
    assert 'nav_trial' not in sys.modules


def test_robot_radius_matches_the_costmaps(geodesic):
    """Geodesica erodida por um raio diferente do costmap descreve outro robo."""
    import yaml
    params = yaml.safe_load(
        (ROOT / 'ros2_ws/src/demo_navigation/config/nav2_params_go2.yaml')
        .read_text(encoding='utf-8'))
    for scope in ('local_costmap', 'global_costmap'):
        radius = params[scope][scope]['ros__parameters']['robot_radius']
        assert abs(radius - geodesic.ROBOT_RADIUS_M) < 0.01, (
            f'{scope}.robot_radius={radius} diverge de '
            f'ROBOT_RADIUS_M={geodesic.ROBOT_RADIUS_M}')


@needs_mesh
def test_patrol_goals_all_sit_behind_a_wall(geodesic):
    """O achado da secao 11. Se isto mudar, a secao 11 esta vencida."""
    data = geodesic.analyse_goals('maze11', MODELS, 0.002,
                                  geodesic.maze11_goals())
    blocked = [row for row in data['goals'] if row['why'] == 'parede']
    assert len(blocked) == len(data['goals']), (
        'alguma meta de patrulha deixou de ter parede na reta -- releia a '
        'secao 11 de docs/ml35/proximos-passos-navegacao.md antes de seguir')
    assert max(row['ratio'] for row in data['goals']) > 1.5


@needs_mesh
def test_chain_measures_from_the_previous_leg(geodesic):
    """O inverso, que e o que importa: sem --chain o numero seria outro.

    Uma perna de 1,4 m a partir da anterior nao pode medir 3 m de reta. Se os
    dois modos empatassem, ``--chain`` seria decorativo e a rota conectada
    apareceria como se atravessasse parede.
    """
    route = [(-1.50, 0.05), (-2.90, 0.10), (-3.30, 1.40), (-1.90, 1.75)]
    chained = geodesic.analyse_goals('maze11', MODELS, 0.002, route, chain=True)
    absolute = geodesic.analyse_goals('maze11', MODELS, 0.002, route)

    assert chained['chain'] is True and absolute['chain'] is False
    # A primeira perna parte do spawn nos dois modos, entao TEM de coincidir.
    assert chained['goals'][0]['straight'] == absolute['goals'][0]['straight']
    # As seguintes nao podem coincidir, ou o modo nao faz nada.
    assert [row['straight'] for row in chained['goals'][1:]] != \
           [row['straight'] for row in absolute['goals'][1:]]
    # E o ponto do achado: encadeada, a rota nao tem parede na reta.
    assert all(row['why'] == 'livre' for row in chained['goals'])
    assert all(row['ratio'] < 1.2 for row in chained['goals'])


@needs_mesh
def test_frame_convention_is_asserted_not_assumed(geodesic):
    """O spawn tem de cair na origem do frame das metas, e falhar alto se nao."""
    data = geodesic.analyse_goals('maze11', MODELS, 0.002, [(0.0, 0.0)])
    assert data['goals'][0]['geodesic'] == pytest.approx(0.0, abs=1e-9)
