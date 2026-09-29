import ast
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).parents[1]
NAV = ROOT / 'ros2_ws/src/demo_navigation'
SIM = ROOT / 'ros2_ws/src/demo_simulation'


def test_exploration_planner_never_crosses_unknown():
    for name in ('nav2_params_go2.yaml', 'params-align8.yaml'):
        params = yaml.safe_load((NAV / 'config' / name).read_text())
        planner = params['planner_server']['ros__parameters']
        assert planner['GridBased']['allow_unknown'] is True
        assert planner['ExplorationGrid']['allow_unknown'] is False
        assert 'ExplorationGrid' in planner['planner_plugins']


GLOBAL_COSTMAP_CELLS_PER_AXIS = 400


def _costmap(name, which):
    params = yaml.safe_load((NAV / 'config' / name).read_text())
    return params[which][which]['ros__parameters']


def test_global_costmap_window_covers_the_whole_maze_diagonal():
    # `quadruped_maze11.sdf` measures ~11.7 x 14.2 m, diagonal ~18.4 m. A 20 m
    # rolling window only reaches +/- 10 m from the robot, so with the robot in
    # one corner the opposite corner did not exist in the master grid -- not
    # for the planner, nor for the cockpit panel. 40 m covers the diagonal
    # from any point in the maze.
    for name in ('nav2_params_go2.yaml', 'params-align8.yaml'):
        costmap = _costmap(name, 'global_costmap')
        assert costmap['rolling_window'] is True, name
        assert costmap['width'] == 40, name
        assert costmap['height'] == 40, name
        assert costmap['resolution'] == 0.10, name


def test_widening_the_global_costmap_did_not_enlarge_the_master_grid():
    # The window doubled AND the resolution doubled, deliberately and in the
    # same round:
    #   before  20 / 0.05 = 400    after  40 / 0.10 = 400
    # This test exists to fail half of the change. Increasing `width` without
    # lowering `resolution` gives 800 cells per axis -- 4x the grid -- on the
    # same arm64 module that already shares CPU with SLAM and the explorer.
    for name in ('nav2_params_go2.yaml', 'params-align8.yaml'):
        costmap = _costmap(name, 'global_costmap')
        cells = costmap['width'] / costmap['resolution']
        assert cells == GLOBAL_COSTMAP_CELLS_PER_AXIS, name
        assert costmap['height'] / costmap['resolution'] == \
            GLOBAL_COSTMAP_CELLS_PER_AXIS, name


def test_local_costmap_keeps_five_centimetre_cells():
    # The local costmap is what decides close-range avoidance. The global's
    # 10 cm cells are for reach, and must not leak in here.
    for name in ('nav2_params_go2.yaml', 'params-align8.yaml'):
        costmap = _costmap(name, 'local_costmap')
        assert costmap['resolution'] == 0.05, name
        assert costmap['width'] == 6, name
        assert costmap['height'] == 6, name


def test_global_costmap_inflation_still_clears_the_robot_footprint():
    # `inflation_radius` is NOT part of this A/B round; it is here because
    # 10 cm per cell is only safe while the inflation stays larger than the
    # robot's footprint, with more than one cell of margin. 0.85 is the
    # GLOBAL costmap's value (0.55 is the local one's). Since the polygonal
    # footprint was promoted (29/08/2026) the shape is a rectangle, not a
    # circle; the comparable floor is the polygon's CIRCUMSCRIBED radius
    # (sqrt(0.37^2+0.18^2) ~ 0.411 m).
    for name in ('nav2_params_go2.yaml', 'params-align8.yaml'):
        costmap = _costmap(name, 'global_costmap')
        radius = costmap['inflation_layer']['inflation_radius']
        assert radius == 0.85, name
        points = ast.literal_eval(costmap['footprint'])
        circumscribed = max(math.hypot(x, y) for x, y in points)
        assert (radius - circumscribed) > costmap['resolution'], name


def test_starting_is_a_cockpit_state_and_never_a_ros_one():
    """
    `starting` covers the window between the click and the Aquila's first status.

    It exists ONLY in the cockpit. If it shows up in the `maze_explorer`
    vocabulary, the node starts publishing a state its own state machine does
    not handle, and `_publish_status`'s `assert self._state in STATES` stops
    protecting anything. The other half of this contract lives in
    hmi/test/exploration.test.js.
    """
    explorer = (NAV / 'demo_navigation/maze_explorer.py').read_text()
    store = (ROOT / 'hmi/js/panels/exploration.js').read_text()
    assert "'starting'" in store
    assert "'starting'" not in explorer


def test_slam_tf_is_restamped_for_distributed_hil_clock():
    params = yaml.safe_load((NAV / 'config/slam_params.yaml').read_text())
    slam = params['slam_toolbox']['ros__parameters']
    assert slam['restamp_tf'] is True
    assert slam['transform_timeout'] == 0.2


def test_slam_rasterises_the_grid_every_second():
    # 1.0 is a deliberate DEVIATION from upstream's 5.0 default
    # (`/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml`).
    #
    # 5.0 was tried on 28/08/2026 and reverted the same day. The argument was
    # CPU savings on the AM69; the measurement gave 31.4% -> 30.3% on
    # `async_slam_toolbox_node`, within noise
    # (docs/results/ml35-f5-tf-cpu-baseline.md section 4). With no savings on
    # one side of the scale, all that is left is the cost on the other: the
    # global costmap's `static_layer` ending up up to 5 s behind the wall SLAM
    # already knows about.
    #
    # The other four keys are here as an A/B LOCK, not out of taste: the round
    # that measures `map_update_interval`'s effect only means something if
    # they have not moved along with it.
    params = yaml.safe_load((NAV / 'config/slam_params.yaml').read_text())
    slam = params['slam_toolbox']['ros__parameters']
    assert slam['map_update_interval'] == 1.0
    assert slam['restamp_tf'] is True
    assert slam['transform_timeout'] == 0.2
    assert slam['transform_publish_period'] == 0.02
    assert slam['minimum_time_interval'] == 0.5


def test_exploration_bt_hardcodes_safe_planner_and_smoothing():
    path = NAV / 'behavior_trees/nav_to_pose_exploration.xml'
    root = ET.parse(path).getroot()
    compute = root.find('.//ComputePathToPose')
    assert compute is not None
    assert compute.attrib['planner_id'] == 'ExplorationGrid'
    smooth = root.find('.//SmoothPath')
    assert smooth is not None
    assert smooth.attrib['unsmoothed_path'] != smooth.attrib['smoothed_path']


def test_explorer_has_no_runtime_knowledge_of_maze_geometry():
    source = (NAV / 'demo_navigation/maze_explorer.py').read_text()
    for forbidden in ('maze_route', 'maze11', 'STL', '-4.90', '-0.90'):
        assert forbidden not in source


def test_exit_marker_is_visual_only_and_outside_opening():
    world = ET.parse(SIM / 'worlds/quadruped_maze11.sdf').getroot()
    marker = world.find(".//model[@name='maze_exit_marker']")
    assert marker is not None
    assert marker.find('.//collision') is None
    pose = [float(value) for value in marker.findtext('pose').split()]
    assert pose[:2] == [-4.90, -2.60]


def test_public_exploration_interfaces_are_stable():
    source = (NAV / 'demo_navigation/maze_explorer.py').read_text()
    for name in (
        '/demo/exploration/start', '/demo/exploration/cancel',
        '/demo/exploration/status', '/demo/perception/maze_exit/pose',
    ):
        assert name in source


def test_short_goal_gate_is_connected_and_bounded():
    source = (ROOT / 'tools/evaluation/nav_trial.py').read_text()
    assert 'MAZE11_SHORT_GOALS' in source
    assert "args.goals == 'maze11-short'" in source


def test_cockpit_owns_start_cancel_and_ground_truth_display():
    html = (ROOT / 'hmi/index.html').read_text()
    panel = (ROOT / 'hmi/js/panels/nav-panel.js').read_text()
    # The search decision lives in a DOM-free module so it can be tested by
    # `node --test` without stubbing a 2D context. The contract holds over both.
    store = (ROOT / 'hmi/js/panels/exploration.js').read_text()
    config = (ROOT / 'hmi/js/config.js').read_text()
    assert 'data-role="exploration-start"' in html
    assert 'data-role="exploration-cancel"' in html
    assert '/demo/exploration/start' in panel
    assert '/demo/exploration/cancel' in panel
    # Two doors for the manual goal -- the canvas click and the submit -- and
    # both have to be closed while the search is running.
    #
    # `explorationBusy()` and not `explorationActive()`: the latter only knows
    # the state published by the Aquila, and between the "start search" click
    # and the first status there is a window where the explorer has already
    # accepted the search but the cockpit does not know it yet. Closing the
    # doors with `isActive()` alone leaves that window open for a manual goal
    # to land on top of the search.
    assert panel.count('if (explorationBusy()) return') == 3
    assert '/demo/maze/escaped' in config
    assert 'EXIT CONFIRMED' in store
    # The success label can only come from ground truth, never from the
    # explorer's state: 'completed' says it got close to the marker, not that
    # the robot crossed the opening.
    assert 'mazeEscaped' in store


def test_gate_persists_outcome_and_error_code_per_goal():
    """The gate's verdict cannot live only in the runner's stdout."""
    source = (ROOT / 'tools/evaluation/nav_trial.py').read_text()
    # Outcome, Nav2 error code and route switches, per goal.
    for field in ('outcome', 'error_code', 'error_msg', 'plan_switches'):
        assert f"'{field}'" in source
    # The goal still in flight at the end of the trial has to be archived:
    # without this call a 3-goal gate ends up reporting 2.
    assert source.count('self.close_goal(') >= 2
    assert 'goals_csv_path' in source


def test_telemetry_rows_carry_the_goal_they_belong_to():
    """Without the stamp, the gate's three goals become a single series."""
    source = (ROOT / 'tools/evaluation/nav_trial.py').read_text()
    assert "'goal_index'" in source


# --- launch and dependency wiring -------------------------------------------
#
# Structural, with `ast`: a raw string match passes with a broken launch. What
# they catch only shows up LATER, in the arm64 container -- a node nobody
# starts, or an import the package.xml does not declare and the build's
# rosdep does not install.

def _node_launches(path):
    """Return {executable: {raw Node(...) kwargs}} from a launch file."""
    import ast
    tree = ast.parse((ROOT / path).read_text(encoding='utf-8'))
    found = {}
    for call in (n for n in ast.walk(tree) if isinstance(n, ast.Call)):
        if not isinstance(call.func, ast.Name) or call.func.id != 'Node':
            continue
        kwargs = {k.arg: k.value for k in call.keywords if k.arg}
        executable = kwargs.get('executable')
        if isinstance(executable, ast.Constant):
            found[executable.value] = ast.dump(call)
    return found


NAV_LAUNCH = 'ros2_ws/src/demo_bringup/launch/nav_quadruped.launch.py'
PERCEPTION_LAUNCH = 'ros2_ws/src/demo_perception/launch/perception.launch.py'
SIM_LAUNCH = 'ros2_ws/src/demo_simulation/launch/quadruped.launch.py'


def test_explorer_is_started_by_the_quadruped_navigation_launch():
    """A node nobody starts is a node that does not exist."""
    nodes = _node_launches(NAV_LAUNCH)
    assert 'maze_explorer' in nodes
    # Without the exploration tree path it would fall back to the default
    # tree, which uses GridBased with allow_unknown -- and the path would go
    # through unknown space.
    assert 'exploration_bt_xml' in nodes['maze_explorer']


def test_detector_runs_where_the_camera_is_consumed():
    """Perception runs on the module; the detector has to come up with it."""
    assert 'maze_exit_detector' in _node_launches(PERCEPTION_LAUNCH)


def test_ground_truth_validator_never_leaves_the_simulation():
    """
    The validator reads ground-truth odometry: it must NOT run on the robot's side.

    If it came up alongside navigation, the explorer would get indirect access
    to the truth it is supposed to discover on its own, and acceptance would
    not measure anything.
    """
    assert 'maze_escape_validator' in _node_launches(SIM_LAUNCH)
    assert 'maze_escape_validator' not in _node_launches(NAV_LAUNCH)
    assert 'maze_escape_validator' not in _node_launches(PERCEPTION_LAUNCH)


def test_explorer_and_detector_never_run_on_the_simulation_side():
    """Rule 1 in reverse: whatever decides navigation lives on the module."""
    sim_nodes = _node_launches(SIM_LAUNCH)
    assert 'maze_explorer' not in sim_nodes
    assert 'maze_exit_detector' not in sim_nodes


def test_every_new_node_has_a_console_script():
    """Without an entry point, launch finds the package but not the executable."""
    for package, executable in (
        ('demo_navigation', 'maze_explorer'),
        ('demo_perception', 'maze_exit_detector'),
        ('demo_simulation', 'maze_escape_validator'),
    ):
        setup = (ROOT / f'ros2_ws/src/{package}/setup.py').read_text()
        assert executable in setup, f'{package}: {executable}'


def test_package_manifests_declare_what_the_new_modules_import():
    """
    An import not declared in package.xml breaks in the container, not on the host.

    On the host the ROS overlay already has everything; the arm64 image
    installs exactly what the manifest asks for. This is the test that
    separates "works here" from "works on the Aquila".
    """
    import re
    expected = {
        'demo_navigation': ('std_msgs', 'geometry_msgs', 'nav_msgs',
                            'tf2_ros', 'nav2_msgs', 'action_msgs',
                            'std_srvs'),
        'demo_perception': ('geometry_msgs', 'sensor_msgs', 'vision_msgs'),
        'demo_simulation': ('std_msgs', 'nav_msgs'),
    }
    for package, dependencies in expected.items():
        manifest = (ROOT / f'ros2_ws/src/{package}/package.xml').read_text()
        declared = set(re.findall(r'<(?:exec_)?depend>([^<]+)</', manifest))
        missing = [name for name in dependencies if name not in declared]
        assert not missing, f'{package} does not declare {missing}'


def test_the_exploration_tree_is_installed_with_the_package():
    """The tree is read at runtime by bt_navigator inside the container."""
    setup = (ROOT / 'ros2_ws/src/demo_navigation/setup.py').read_text()
    assert 'behavior_trees' in setup


def test_perception_keeps_the_marker_out_of_the_costmap_pipeline():
    """
    The marker is a visual cue; turning it into an obstacle blocks the exit itself.

    `detections_to_cloud` subscribes to the project's contract topic. The exit
    detector publishes on another one, and that separation is what keeps the
    marker from showing up as an obstacle right in front of the opening.
    """
    detector = (ROOT / 'ros2_ws/src/demo_perception/demo_perception'
                / 'maze_exit_detector.py').read_text()
    assert "'/demo/perception/maze_exit/detections'" in detector
    assert "'/demo/perception/detections'" not in detector


def test_neither_perception_nor_frontier_knows_the_maze():
    """The isolation holds for the whole chain, not only for the executive."""
    for path in ('ros2_ws/src/demo_perception/demo_perception/maze_exit_detector.py',
                 'ros2_ws/src/demo_navigation/demo_navigation/frontier.py'):
        source = (ROOT / path).read_text()
        for forbidden in ('maze_route', 'maze11', '-4.90', '-0.90', 'waypoint'):
            assert forbidden not in source, f'{path}: {forbidden}'
