"""Guards for ``tools/maze/maze_geodesic.py``.

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
SCRIPTS = ROOT / 'tools' / 'maze'
MODELS = Path.home() / 'ros_maze_worlds' / 'models'
MAZE11_STL = MODELS / 'maze11' / 'meshes' / 'maze11.stl'

sys.path.insert(0, str(SCRIPTS))

needs_mesh = pytest.mark.skipif(
    not MAZE11_STL.is_file(),
    reason=f'external mesh missing: {MAZE11_STL}')


@pytest.fixture(scope='module')
def geodesic():
    return pytest.importorskip('maze_geodesic')


def test_goals_are_read_from_nav_trial_not_copied(geodesic):
    """The tuple has a single source.

    Duplicating ``MAZE11_GOALS`` here or in the script creates two truths that
    diverge the day someone regenerates the goals with ``maze_fit.py`` -- and
    the table would keep printing, measuring goals the trial no longer sends.
    """
    source = (ROOT / 'tools' / 'evaluation' / 'nav_trial.py').read_text(encoding='utf-8')
    literal = None
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Assign):
            if any(getattr(t, 'id', None) == 'MAZE11_GOALS'
                   for t in node.targets):
                literal = [tuple(v) for v in ast.literal_eval(node.value)]
    assert literal, 'MAZE11_GOALS disappeared from nav_trial.py'
    assert geodesic.maze11_goals() == literal


def test_reading_goals_does_not_require_ros(geodesic):
    """The script runs offline. Importing ``nav_trial`` would pull in geometry_msgs."""
    assert 'nav_trial' not in sys.modules
    geodesic.maze11_goals()
    assert 'nav_trial' not in sys.modules


def test_robot_radius_matches_the_costmaps(geodesic):
    """A geodesic eroded by a radius smaller than the costmap's describes a different robot.

    Since the promotion of the polygonal footprint (29/08/2026) the costmap no
    longer declares `robot_radius`; the comparable floor is the CIRCUMSCRIBED
    radius of the polygon (sqrt(0.37^2+0.18^2) ~ 0.411 m), which includes a
    0.02 m margin over the measured chassis. The offline geodesic uses the raw
    chassis (`ROBOT_RADIUS_M`), so it MUST be LESS THAN OR EQUAL to the
    costmap's radius -- larger would describe a robot that the costmap
    protects less than the offline analysis assumes.
    """
    import math
    import yaml
    params = yaml.safe_load(
        (ROOT / 'ros2_ws/src/demo_navigation/config/nav2_params_go2.yaml')
        .read_text(encoding='utf-8'))
    for scope in ('local_costmap', 'global_costmap'):
        points = ast.literal_eval(
            params[scope][scope]['ros__parameters']['footprint'])
        circumscribed = max(math.hypot(x, y) for x, y in points)
        assert geodesic.ROBOT_RADIUS_M <= circumscribed, (
            f'{scope} footprint circumscribed radius {circumscribed} is '
            f'smaller than ROBOT_RADIUS_M={geodesic.ROBOT_RADIUS_M} -- the '
            'offline geodesic would describe a smaller robot than the '
            'costmap actually protects')


@needs_mesh
def test_patrol_goals_all_sit_behind_a_wall(geodesic):
    """The finding from section 11. If this changes, section 11 is obsolete."""
    data = geodesic.analyse_goals('maze11', MODELS, 0.002,
                                  geodesic.maze11_goals())
    blocked = [row for row in data['goals'] if row['why'] == 'wall']
    assert len(blocked) == len(data['goals']), (
        'some patrol goal no longer has a wall on the straight line -- '
        're-read section 11 of docs/ml35/proximos-passos-navegacao.md before continuing')
    assert max(row['ratio'] for row in data['goals']) > 1.5


@needs_mesh
def test_chain_measures_from_the_previous_leg(geodesic):
    """The inverse, which is what matters: without --chain the number would differ.

    A leg of 1.4 m from the previous one cannot measure 3 m in a straight
    line. If the two modes tied, ``--chain`` would be decorative and the
    connected route would appear to pass through a wall.
    """
    route = [(-1.50, 0.05), (-2.90, 0.10), (-3.30, 1.40), (-1.90, 1.75)]
    chained = geodesic.analyse_goals('maze11', MODELS, 0.002, route, chain=True)
    absolute = geodesic.analyse_goals('maze11', MODELS, 0.002, route)

    assert chained['chain'] is True and absolute['chain'] is False
    # The first leg starts from the spawn in both modes, so it MUST match.
    assert chained['goals'][0]['straight'] == absolute['goals'][0]['straight']
    # The following ones must not match, or the mode does nothing.
    assert [row['straight'] for row in chained['goals'][1:]] != \
           [row['straight'] for row in absolute['goals'][1:]]
    # And that's the point of the finding: chained, the route has no wall on the straight line.
    assert all(row['why'] == 'free' for row in chained['goals'])
    assert all(row['ratio'] < 1.2 for row in chained['goals'])


@needs_mesh
def test_frame_convention_is_asserted_not_assumed(geodesic):
    """The spawn must land on the origin of the goals' frame, and fail loudly if not."""
    data = geodesic.analyse_goals('maze11', MODELS, 0.002, [(0.0, 0.0)])
    assert data['goals'][0]['geodesic'] == pytest.approx(0.0, abs=1e-9)
