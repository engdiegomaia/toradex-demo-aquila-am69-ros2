"""
Actually builds the scene cameras launch description.

WHY THIS FILE EXISTS

`scenarios.py`'s tests check the TABLE; this one checks the WIRING. The
distinction is not academic: on 25/08/2026 the table was correct, the
structural guards passed, and the `sim` container died on startup with

    [ERROR] [launch]: Caught exception in launch (see debug for traceback):
    value='-13.0' is not an instance of <class 'float'>

because `ParameterValue(value_type=float)` converts a SUBSTITUTION to float,
not text to float, and the poses resolved by the OpaqueFunction were already
text. None of this is visible from static reading, and the cost of finding it
was a full image build cycle.

Running the description here costs milliseconds and catches this entire
class of bug: parameter type, undeclared argument, substitution in place of
an action.
"""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

import pytest

LAUNCH_DIR = Path(__file__).resolve().parents[1] / 'launch'


def _load(name: str):
    path = LAUNCH_DIR / name
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(module, **overrides) -> LaunchContext:
    """Context with the declared defaults, as launch would build it."""
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            value = ''
            if entity.default_value:
                value = entity.default_value[0].perform(context)
            context.launch_configurations[entity.name] = value
    context.launch_configurations.update(overrides)
    return context


@pytest.fixture(scope='module')
def scene_cameras():
    return _load('scene_cameras.launch.py')


def _params(module, world, **overrides) -> dict:
    """Parameters the scene_view_controller would receive for this world."""
    context = _context(module, world=world, **overrides)
    return module.controller_params(module._resolve(context, 'iso'),
                                    module._resolve(context, 'top'))


WORLDS = ('quadruped_maze11.sdf', 'warehouse.sdf', 'quadruped_empty.sdf', '')


@pytest.mark.parametrize('world', WORLDS)
def test_description_builds_for_every_world(scene_cameras, world):
    """Includes the empty world: the generic path must be a valid one."""
    nodes = scene_cameras._cameras(_context(scene_cameras, world=world))
    assert len(nodes) == 3, 'two cameras and the view controller'


def test_view_controller_params_are_real_floats(scene_cameras):
    """
    The exact 25/08 defect: pose as text inside ParameterValue.

    The node declares the ten poses as double. Text there brings down the
    description BEFORE any node comes up, and the message talks about type,
    not camera.
    """
    context = _context(scene_cameras, world='quadruped_maze11.sdf')
    params = scene_cameras.controller_params(
        scene_cameras._resolve(context, 'iso'),
        scene_cameras._resolve(context, 'top'))
    for name in ('iso_x', 'iso_y', 'iso_z', 'iso_pitch', 'iso_yaw',
                 'top_x', 'top_y', 'top_z', 'top_pitch', 'top_yaw'):
        assert isinstance(params[name], float), f'{name} is not float'


def test_maze_framing_reaches_the_view_controller(scene_cameras):
    """
    A correct table does not guarantee it REACHED the node.

    This is the counterpart of the structural test: that one checks the
    numbers exist in scenarios.py, this one checks the controller actually
    receives them. The warehouse framing in a maze world would point the
    blue panel at empty ground, with no error and no log.
    """
    params = _params(scene_cameras, '/algum/lugar/quadruped_maze11.sdf')
    assert params['top_x'] == pytest.approx(-4.855)
    assert params['top_y'] == pytest.approx(4.855)
    assert params['top_z'] == pytest.approx(13.0)
    assert params['iso_x'] == pytest.approx(-13.0)


def test_explicit_argument_beats_the_table(scene_cameras):
    """Probe from higher up without losing the other nine numbers."""
    params = _params(scene_cameras, 'quadruped_maze11.sdf',
                     scene_top_z='20.0')
    assert params['top_z'] == pytest.approx(20.0)
    # the others still follow the scenario
    assert params['top_x'] == pytest.approx(-4.855)


def test_warehouse_keeps_its_measured_framing(scene_cameras):
    """
    Use 6 m, not 12 m.

    At 12 m the camera ends up above the warehouse roof beams and the
    whole image turns into a beam, with the robot hidden behind it.
    """
    params = _params(scene_cameras, 'warehouse.sdf')
    assert params['top_z'] == pytest.approx(6.0)
