"""
Structural guards for the official scenario (maze).

Static checks on committed files, same reasoning as the other files in this
directory: each invariant here is cheap to break in a one-line edit and the
failure mode is a measurement that PASSES with numbers better than reality.

Run with:  python3 -m pytest tests/ -q
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SIMULATION = REPO_ROOT / 'ros2_ws' / 'src' / 'demo_simulation'
BRINGUP = REPO_ROOT / 'ros2_ws' / 'src' / 'demo_bringup'
SCENARIOS = SIMULATION / 'demo_simulation' / 'scenarios.py'
QUADRUPED_LAUNCH = SIMULATION / 'launch' / 'quadruped.launch.py'
SCENE_CAMERAS = SIMULATION / 'launch' / 'scene_cameras.launch.py'
SIM_LAUNCH = BRINGUP / 'launch' / 'sim.launch.py'
ROBOT_SELECTION = BRINGUP / 'demo_bringup' / 'robot_selection.py'
COMPOSE_HOST = REPO_ROOT / 'docker' / 'compose.host.yml'
MODELS_EXTRA_README = REPO_ROOT / 'docker' / 'models-extra' / 'README.md'

MAZE_WORLD = 'quadruped_maze11.sdf'


def _load(path: Path, name: str):
    """Import a module by path, so the tables are checked as VALUES."""
    sys.path.insert(0, str(path.parent.parent))
    try:
        module = __import__(f'{path.parent.name}.{name}', fromlist=[name])
    finally:
        sys.path.pop(0)
    return module


@pytest.fixture(scope='module')
def scenarios():
    return _load(SCENARIOS, 'scenarios')


@pytest.fixture(scope='module')
def selection():
    return _load(ROBOT_SELECTION, 'robot_selection')


def _code(path: Path) -> str:
    """Source with comments and docstrings stripped.

    A guard that matches its own documentation is worse than no guard: it passes
    while the behaviour is gone. Same helper reasoning as
    test_cockpit_web_contract.py.
    """
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
        if not isinstance(node, (ast.Module, ast.FunctionDef,
                                 ast.AsyncFunctionDef, ast.ClassDef)):
            continue
        body = node.body
        if (body and isinstance(body[0], ast.Expr)
                and isinstance(body[0].value, ast.Constant)
                and isinstance(body[0].value.value, str)):
            body.pop(0)
    return ast.unparse(tree)


# --- the official scenario ----------------------------------------------------

def test_maze_is_the_official_world_for_the_quadruped(selection):
    """The maze is the official scenario, per the operator's request on 25/08."""
    package, *parts = selection.official_world('quadruped')
    assert parts[-1] == MAZE_WORLD
    assert package == 'demo_simulation'


def test_diffdrive_keeps_the_warehouse_where_it_was_validated(selection):
    """
    The fallback does NOT inherit the maze.

    The diff-drive F6 gate is a goal from x=0 to x=1, measured in the
    warehouse. In the maze that goal lands on a wall: promoting the maze for
    both would swap the scenario of a test that passed for one it never ran
    on.
    """
    package, *parts = selection.official_world('diffdrive')
    assert parts[-1] == 'warehouse.sdf'
    assert package == 'nav2_minimal_tb4_sim'


def test_unknown_robot_has_no_official_world(selection):
    with pytest.raises(ValueError):
        selection.official_world('hexapod')


def test_quadruped_plant_defaults_to_the_maze():
    """
    The shortest path already brings up the official scenario.

    It used to be `quadruped_empty.sdf`, inherited from the F2 spike. With the
    quadruped becoming the default robot in F6, the old default made the
    shortest command measure open-field navigation with tuning meant for a
    1.20 m corridor.
    """
    code = _code(QUADRUPED_LAUNCH)
    assert MAZE_WORLD in code
    assert 'quadruped_empty.sdf' not in code


# --- framing derived from the world -------------------------------------

def test_maze_framing_matches_the_measured_geometry(scenarios):
    """
    The maze numbers are the measured ones, not fresh rounding.

    Center (-4.855, 4.855) came from the STL bbox read from the binary and
    multiplied by the 0.002 scale (tools/maze/maze_fit.py), not from the file
    name. The top view at 13 m covers 17.8 x 13.3 m, the maze's 11.6 m with
    margin.
    """
    top = scenarios.camera_pose(MAZE_WORLD, 'top')
    assert top[:3] == (-4.855, 4.855, 13.0)
    iso = scenarios.camera_pose(MAZE_WORLD, 'iso')
    assert iso[:3] == (-13.0, -3.0, 9.0)


def test_maze_spawn_faces_the_corridor(scenarios):
    """
    yaw 1.5708 spawns the robot facing the corridor, not the wall.

    With yaw 0 the first thing Nav2 has to do is a 90-degree turn inside a
    1.20 m corridor. That eats the first seconds of every trial and pollutes
    the comparison between conditions.
    """
    assert scenarios.spawn_pose(MAZE_WORLD)['yaw'] == pytest.approx(1.5708)


def test_scene_camera_defaults_are_empty_not_numeric():
    """
    Empty = derive from the world. A number here is the original silent failure.

    While world and framing were independent arguments, the wrong combination
    was the easiest to produce: maze world with warehouse camera points the
    blue panel at empty floor, with no error and no log.
    """
    code = _code(SCENE_CAMERAS)
    assert "default_value=''" in code
    # The poses cannot be hardcoded here: scenarios.py is what holds them.
    for number in ('-4.855', '13.0', '0.5150'):
        assert number not in code, (
            f'{number} became a launch default again; the framing must come '
            'from scenarios.py, or world and camera diverge again'
        )


def test_scene_cameras_reads_the_scenario_table():
    assert 'camera_pose' in _code(SCENE_CAMERAS)


def test_sim_launch_resolves_the_world_before_including_the_plant():
    """
    Launch scope: the parent wins, so the world must NOT be left empty here.

    A DeclareLaunchArgument in the included description does not override an
    inherited value. If sim.launch.py passed empty, scene_cameras.launch.py
    would inherit the empty value and fall back to the generic framing in a
    world whose usable area is not at the origin.
    """
    code = _code(SIM_LAUNCH)
    assert 'official_world' in code


# --- the external mesh guard -------------------------------------------

def test_missing_mesh_aborts_and_names_the_variable():
    """
    A missing mesh is a WARNING in Gazebo, never an error.

    Without a guard: the world loads, the maze is not there, the lidar sees
    no wall, Nav2 plans a straight line, and the goal ends SUCCEEDED faster
    than reality. The trial PASSES with numbers better than the truth.
    """
    code = _code(QUADRUPED_LAUNCH)
    assert 'missing_models' in code
    assert 'MAZE_MODELS' in code
    assert 'RuntimeError' in code


def test_maze_mesh_is_not_vendored(scenarios):
    """
    License TODO upstream, same blocker that swapped the A1 for the Go2.

    If the mesh is ever vendored, it comes out of `needs_models` and this
    test must be rewritten along with the legal basis -- not deleted.
    """
    assert scenarios.external_models(MAZE_WORLD) == ('maze11',)
    assert not list(SIMULATION.glob('models/maze11/**/*.stl'))


def test_models_extra_readme_exists():
    """
    The compose comment promises this file.

    The promise was false: the comment said "The default is an EMPTY
    directory in the repo ... See models-extra/README.md" and the directory
    did not exist. Documentation that points at nothing is worse than none.
    """
    assert MODELS_EXTRA_README.is_file()
    text = MODELS_EXTRA_README.read_text()
    assert 'MAZE_MODELS' in text
    assert 'ros_maze_worlds' in text


def test_compose_no_longer_pastes_the_framing_by_hand():
    """
    The fourth copy of the ten numbers came out of compose.

    It was a six-line SIM_ARGS, and pasting half of it gave maze world with
    warehouse camera.
    """
    text = COMPOSE_HOST.read_text()
    assert 'scene_iso_pitch:=' not in text
    assert 'MAZE_MODELS' in text


# --- stability: the route_server segfault ----------------------------

NAV_PARAMS = (REPO_ROOT / 'ros2_ws' / 'src' / 'demo_navigation' / 'config'
              / 'nav2_params_go2.yaml')
VENDORED_NAV = (REPO_ROOT / 'ros2_ws' / 'src' / 'demo_navigation' / 'launch'
                / 'nav2_vendored' / 'navigation_launch.py')


def test_route_server_does_not_build_the_rerouting_service():
    """
    Configuring `ReroutingService` brings down nav2_container with SIGSEGV.

    Measured on three paths: RESET+STARTUP via lifecycle_manager and a
    **normal cold start**, the latter intermittently. When it happens ALL the
    Nav2 servers die at once and the `nav` container is left with the `ros2`
    process alive and no children -- `docker compose ps` says "running" and
    the robot does not navigate.

    The demo does not use graph-based routing, so listing only
    `AdjustSpeedLimit` in `operations` means the problematic plugin is never
    built.
    """
    import yaml
    params = yaml.safe_load(NAV_PARAMS.read_text())
    operations = params['route_server']['ros__parameters']['operations']
    assert 'ReroutingService' not in operations
    # Empty list breaks the launch with "Expected 'value' to be one of [...]
    # but got '()'" -- trap 5 of the operations guide.
    assert operations, 'empty YAML list breaks the launch; leave one plugin'


def test_every_declared_route_operation_declares_its_plugin_type():
    """
    Overriding `operations` forces every plugin in the list to declare its TYPE.

    While the list comes from the default, nav2_route knows the types of its
    own defaults. The instant it is overridden, it starts requiring
    `<name>.plugin` -- and fails the entire bring-up with a message that does
    not say where the parameter is missing:

        [FATAL] [route_server]: Can not get 'plugin' param value for AdjustSpeedLimit
        [FATAL] [route_server]: Failed to configure route server: No 'plugin' param
        [ERROR] [lifecycle_manager_navigation]: Failed to bring up all requested nodes.

    Measured on 25/08/2026, on the first cold start with the segfault
    workaround applied. This test exists so that the next operation added to
    the list does not repeat the same failed bring-up.
    """
    import yaml
    section = yaml.safe_load(NAV_PARAMS.read_text())['route_server']
    params = section['ros__parameters']
    for name in params['operations']:
        assert name in params, (
            f'{name} is in operations and has no section of its own')
        assert params[name].get('plugin'), (
            f'{name} needs `plugin:` with the type, e.g. nav2_route::{name}')


def test_vendored_navigation_launch_still_owns_the_lifecycle_list():
    """
    The workaround above exists BECAUSE the list cannot be overridden via YAML.

    `navigation_launch.py` passes `{'node_names': lifecycle_nodes}` inline,
    and an inline parameter beats a parameter file. If upstream ever exposes
    the list as a launch argument, the workaround could become "do not bring
    up the route_server" and this test must change along with it.
    """
    text = VENDORED_NAV.read_text()
    assert "'node_names': lifecycle_nodes" in text
    assert "'route_server'" in text


# --- stability: DDS discovery ---------------------------------------

DDS_HOST = REPO_ROOT / 'docker' / 'cyclonedds' / 'host.xml'
ENV_SH = REPO_ROOT / 'scripts' / 'env.sh'
COMPOSE_HOST_TEXT = COMPOSE_HOST.read_text()


def test_host_dds_pins_loopback_alongside_autodetermine():
    """
    Same machine cannot depend on whatever autodetermine picks.

    With docker0, two bridges and tailscale0 up, the choice can land on an
    interface that does not route to the other participants. The result
    measured on 25/08/2026 was ASYMMETRIC delivery: `nav` received odom and
    scan from `sim`, and `sim` did NOT receive /demo/cmd_vel from `nav`. The
    robot trots in place with `sticks=(lx=0.0000 ...)` while Nav2 commands
    yaw on 95.6% of the samples, and no log line names DDS.
    """
    text = DDS_HOST.read_text()
    assert 'name="lo"' in text, (
        'without an explicit loopback interface, traffic between containers '
        'on the same machine goes back to depending on autodetermine'
    )
    assert 'autodetermine="true"' in text, (
        'the real interface must stay in the list, or hil mode loses the '
        'path to the Aquila'
    )


def test_rendered_dds_config_is_not_older_than_its_template():
    """
    Editing host.xml has no effect on the containers until `module.sh sync` runs.

    Compose mounts `host.rendered.xml`, generated from host.xml with the
    module address and this host's interface (the address does not go into
    git). On the 25/08/2026 bench the rendered file had been generated
    before, was byte-identical to the template, and stayed frozen while the
    template changed: the loopback fix was committed and the containers kept
    running without it.

    This test is SKIP when the rendered file does not exist -- on a fresh
    clone it does not exist yet, and that is not a defect of the commit.
    """
    rendered = REPO_ROOT / 'docker' / 'cyclonedds' / 'host.rendered.xml'
    if not rendered.is_file():
        pytest.skip('host.rendered.xml has not been generated yet (module.sh sync)')
    assert rendered.stat().st_mtime >= DDS_HOST.stat().st_mtime, (
        'host.rendered.xml is older than host.xml: run '
        '`scripts/module.sh sync` and recreate the containers, or the '
        'revised config is not the one running'
    )
    assert 'name="lo"' in rendered.read_text(), (
        'the rendered file lost the loopback interface'
    )


def test_render_step_keeps_loopback_and_replaces_autodetermine():
    """
    The awk in `render_host_config` replaces only the autodetermine line.

    If it ever starts rewriting the whole <Interfaces> block, the loopback
    line disappears from hil and the asymmetry comes back -- there, where it
    is more expensive to discover.
    """
    module_sh = (REPO_ROOT / 'scripts' / 'module.sh').read_text()
    assert 'NetworkInterface autodetermine=' in module_sh
    assert 'autodetermine' in module_sh


def test_host_tools_use_the_same_dds_config_as_the_containers():
    """
    `nav_trial.py` used to abort with "navigate_to_pose did not appear" while
    the nav log said "Managed nodes are active" and /demo/odom arrived at
    49 Hz: half the graph visible, half not.
    """
    text = ENV_SH.read_text()
    assert 'CYCLONEDDS_URI' in text
    assert 'cyclonedds/host.xml' in text


# --- stability: load navigation pays for without showing -----------------

SCENE_MODELS = (REPO_ROOT / 'ros2_ws' / 'src' / 'demo_simulation' / 'models')


def test_perception_layer_clears_in_both_costmaps():
    """
    `clearing: false` on the perception layer leaves a PERMANENT mark.

    `observation_persistence` empties the observation buffer, not the cells
    already written. Measured on 25/08/2026 with perception stopped for a
    minute: 169 lethal cells before clearing, 108 after `/demo/nav/reset` --
    61 cells that no live observation was sustaining. In a 1.20 m corridor
    with `robot_radius: 0.38`, that is the difference between having a path
    and not, and in a long-running demo it is monotonic degradation with no
    log at all.

    The TWO costmaps need to match: planning on a map that clears and
    controlling on one that does not clear gives a valid route with the
    controller refusing to follow it.
    """
    import yaml
    params = yaml.safe_load(NAV_PARAMS.read_text())
    found = 0
    for costmap in ('global_costmap', 'local_costmap'):
        layers = params[costmap][costmap]['ros__parameters']
        layer = layers.get('perception_layer')
        if layer is None:
            continue
        found += 1
        source = layer['observation_sources']
        assert layers['perception_layer'][source]['clearing'] is True, (
            f'{costmap}.perception_layer.{source}.clearing must be true')
    assert found == 2, 'both perception layers must exist'


def test_scene_cameras_keep_the_measured_aspect_and_match_each_other():
    """
    The framing of both scenes was MEASURED at 4:3.

    A fixed `horizontal_fov` with a different aspect ratio crops the
    vertical and de-frames both at once, with no error. And both cameras
    must have the same resolution: different resolutions make one cockpit
    panel arrive sharper than the other for no reason, and change the
    render cost of each.
    """
    import re
    sizes = {}
    for name in ('cockpit_scene_iso.sdf', 'cockpit_scene_top.sdf'):
        text = (SCENE_MODELS / name).read_text()
        width = int(re.search(r'<width>(\d+)</width>', text).group(1))
        height = int(re.search(r'<height>(\d+)</height>', text).group(1))
        assert width * 3 == height * 4, f'{name}: {width}x{height} is not 4:3'
        sizes[name] = (width, height)
    assert len(set(sizes.values())) == 1, f'divergent resolutions: {sizes}'
