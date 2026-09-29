"""
Structural guards for the web cockpit (ML3.5 cockpit-web).

These are static checks on committed files, not runtime tests. They exist
because each invariant below is cheap to break in a one-line edit and expensive
to discover: the placement ones only fail on the Aquila, weeks later.

Run with:  python3 -m pytest tests/ -q
"""

from __future__ import annotations

import ast
import re
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
COCKPIT_DOCKERFILE = REPO_ROOT / 'docker' / 'cockpit' / 'Dockerfile'
HMI_DOCKERFILE = REPO_ROOT / 'docker' / 'hmi' / 'Dockerfile'
NGINX_TEMPLATE = REPO_ROOT / 'docker' / 'hmi' / 'default.conf.template'
COMPOSE_HOST = REPO_ROOT / 'docker' / 'compose.host.yml'
BUNDLE = REPO_ROOT / 'hmi'
SIMULATION = REPO_ROOT / 'ros2_ws' / 'src' / 'demo_simulation'
SCENE_CAMERAS_LAUNCH = SIMULATION / 'launch' / 'scene_cameras.launch.py'
SCENARIOS_TABLE = SIMULATION / 'demo_simulation' / 'scenarios.py'
PLANT_LAUNCHES = (
    SIMULATION / 'launch' / 'quadruped.launch.py',
    SIMULATION / 'launch' / 'simulation.launch.py',
)
BRIDGE_CONFIGS = (
    SIMULATION / 'config' / 'bridge_quadruped.yaml',
    SIMULATION / 'config' / 'bridge_warehouse.yaml',
)
GO2_DESCRIPTION = REPO_ROOT / 'ros2_ws' / 'src' / 'go2_description'

# Desktop-OpenGL / OGRE 2 packages. The cockpit image is the ONE role that is
# allowed to build for arm64 and reach the module, so it is the one place where
# adding a convenient debugging package silently violates CLAUDE.md rule 1.
DESKTOP_GL_PACKAGES = (
    'rviz2',
    'rviz-common',
    'ros-gz-sim',
    'gz-sim',
    'gazebo',
    'rqt-common-plugins',
    'rqt-image-view',
)


def test_cockpit_image_stays_free_of_desktop_opengl():
    """The one image allowed on the AM69 must not acquire an OGRE 2 dependency."""
    dockerfile = COCKPIT_DOCKERFILE.read_text()
    installs = '\n'.join(
        line for line in dockerfile.splitlines() if not line.lstrip().startswith('#')
    )
    for package in DESKTOP_GL_PACKAGES:
        assert package not in installs, (
            f'{package} would put desktop OpenGL on an arm64-capable image '
            '(CLAUDE.md rule 1)'
        )


def test_cockpit_image_carries_the_two_transports():
    """rosbridge without web_video_server means a camera panel that never fills."""
    dockerfile = COCKPIT_DOCKERFILE.read_text()
    assert 'rosbridge-suite' in dockerfile
    assert 'web-video-server' in dockerfile


def test_hmi_image_has_no_javascript_build_step():
    """plano-cockpit-web.md Decision 5: no npm supply chain travels to arm64."""
    dockerfile = _code_of(HMI_DOCKERFILE)
    for forbidden in ('npm ', 'yarn ', 'pnpm ', 'node_modules'):
        assert forbidden not in dockerfile, (
            f'{forbidden.strip()!r} reopens Decision 5 (bundle without build step)'
        )


def test_bundle_ships_no_vendored_dependencies():
    """A node_modules or a vendored roslib in the tree is the same decision."""
    assert not (BUNDLE / 'node_modules').exists()
    assert not (BUNDLE / 'package-lock.json').exists()
    vendored = [path.name for path in BUNDLE.rglob('roslib*.js')]
    assert vendored == [], f'vendored roslibjs found: {vendored}'


def test_bundle_has_no_hardcoded_hostname():
    """AGENTS.md §5.7. A literal host works on this desk and fails on the bench."""
    sources = [
        path
        for path in BUNDLE.rglob('*')
        if path.suffix in {'.js', '.html', '.css'} and 'test' not in path.parts
    ]
    assert sources, 'bundle sources not found — did the tree move?'
    for path in sources:
        text = path.read_text()
        for forbidden in ('ws://localhost', 'http://localhost', '127.0.0.1'):
            assert forbidden not in text, f'{path.name} hardcodes {forbidden}'


def test_cockpit_ports_are_configurable():
    """network_mode: host means these bind on the operator's workstation."""
    compose = COMPOSE_HOST.read_text()
    for variable in (
        'COCKPIT_ROSBRIDGE_PORT',
        'COCKPIT_VIDEO_PORT',
        'COCKPIT_HMI_PORT',
    ):
        assert f'${{{variable}:-' in compose, f'{variable} must have a compose default'


def test_nginx_template_restricts_envsubst():
    """
    Without the filter, envsubst blanks nginx's own $uri/$host.

    The result is a site that reports a valid configuration and 404s every
    request — a failure whose cause is not named anywhere in the logs.
    """
    assert 'NGINX_ENVSUBST_FILTER=^COCKPIT_' in HMI_DOCKERFILE.read_text()
    assert '$uri' in NGINX_TEMPLATE.read_text()


def _code_of(path: Path) -> str:
    """Source text with comment lines removed, so prose does not trip a check."""
    return '\n'.join(
        line for line in path.read_text().splitlines()
        if not line.lstrip().startswith('#')
    )


# --- F3b: scene cameras ----------------------------------------------------
#
# The blue panel's cameras belong to the world, never to the robot, and they
# must reach EVERY world rather than only the two that happen to be committed
# here. Both invariants are one careless edit away from being lost, and both
# fail silently: a dead panel, or a licence argument that no longer holds.


def test_scene_cameras_are_spawned_not_written_into_worlds():
    """
    The cameras must be spawnable models, not blocks inside worlds/*.sdf.

    docker/compose.host.yml does not pass `world:=`, so the default is
    nav2_minimal_tb4_sim's warehouse.sdf — an installed package file this repo
    cannot edit. Cameras written into the project's own worlds leave the blue
    panel dark in exactly the configuration the demo runs in.
    """
    for world in (SIMULATION / 'worlds').glob('*.sdf'):
        assert 'cockpit_scene' not in world.read_text(), (
            f'{world.name} carries a scene camera inline; it belongs in '
            'models/ and is spawned by scene_cameras.launch.py'
        )

    for name in ('cockpit_scene_iso.sdf', 'cockpit_scene_top.sdf'):
        assert (SIMULATION / 'models' / name).is_file(), f'model {name} missing'


def test_scene_camera_models_are_installed():
    """A model that setup.py does not install is a file the launch cannot find."""
    setup = (SIMULATION / 'setup.py').read_text()
    assert "glob('models/*.sdf')" in setup


def test_both_plants_include_the_scene_cameras():
    """
    The panel must not go dark when ROBOT_TYPE flips.

    Both plants are supported paths (CLAUDE.md), and a cockpit that only works
    for one of them is the kind of mode-dependent behaviour the project bans.
    """
    for launch in PLANT_LAUNCHES:
        assert 'scene_cameras.launch.py' in launch.read_text(), (
            f'{launch.name} does not include the shared scene-camera fragment'
        )


def test_scene_cameras_are_never_hung_on_the_vendored_robot():
    """
    go2_description is vendorized byte-for-byte to sustain the licence
    argument (its own README). Hanging a chase camera on the trunk would be the
    obvious shortcut and would quietly invalidate that.
    """
    if not GO2_DESCRIPTION.is_dir():
        return
    for source in GO2_DESCRIPTION.rglob('*'):
        if source.is_file() and source.suffix in {'.xacro', '.urdf', '.sdf', '.yaml'}:
            assert 'cockpit' not in source.read_text(errors='ignore'), (
                f'{source.relative_to(REPO_ROOT)} mentions the cockpit; the '
                'vendored package must stay untouched'
            )


def test_scene_topics_are_bridged_on_every_plant():
    """
    Image AND camera_info, on both bridges.

    The camera_info is not geometry here: it is the panel's heartbeat. An <img>
    on a multipart stream never reports its own liveness, so without it the
    panel shows "no signal" over running video.
    """
    for config in BRIDGE_CONFIGS:
        text = config.read_text()
        for view in ('scene_iso', 'scene_top'):
            for suffix in ('image_raw', 'camera_info'):
                topic = f'/demo/cockpit/{view}/{suffix}'
                assert topic in text, f'{topic} is not in {config.name}'


def test_scene_camera_framing_is_overridable():
    """
    No pose literal may be the only way to frame a world.

    maze11 is the one project world whose usable area is not centred on the
    origin, so a single set of defaults cannot suit every world. The framing
    therefore lives in demo_simulation/scenarios.py, indexed BY WORLD, and this
    launch file declares the five axes as arguments that override it.

    The table moved out of this file on 25/08/2026 (it used to be a `DEFAULTS`
    dict here). The invariant did not move: every axis stays reachable from the
    command line, and no world is stuck with another world's framing.

    Read the axis declarations through the source rather than by importing: the
    argument names are built with f-strings, so a grep for the literal name
    would pass on a file that declares nothing.
    """
    poses = ast.literal_eval(
        next(
            node.value
            for node in ast.parse(SCENARIOS_TABLE.read_text()).body
            if isinstance(node, ast.Assign)
            and any(getattr(t, 'id', None) == 'GENERIC' for t in node.targets)
        )
    )

    assert {'scene_iso', 'scene_top'} <= set(poses), (
        'both views need a generic framing entry'
    )
    for name in ('scene_iso', 'scene_top'):
        # x, y, z, pitch, yaw. Roll is deliberately absent.
        assert len(poses[name]) == 5, f'{name} pose is incomplete'
        for value in poses[name]:
            float(value)

    launch = SCENE_CAMERAS_LAUNCH.read_text()
    for axis in ('x', 'y', 'z', 'pitch', 'yaw'):
        assert f"scene_{{name}}_{{axis}}" in launch or (
            f"'{axis}'" in launch
        ), f'the {axis} axis is not declared as a launch argument'
    assert 'POSE_FIELDS' in launch, (
        'the five axes must be declared from a single list; declaring them '
        'one by one is how one of them silently goes missing'
    )


def test_scene_camera_models_carry_no_desktop_gl_assumption():
    """
    A camera sensor is render work inside the Gazebo process, which is host
    only. The models must not acquire anything that suggests otherwise, and the
    world dependency must stay written down where someone will read it.
    """
    for name in ('cockpit_scene_iso.sdf', 'cockpit_scene_top.sdf'):
        text = (SIMULATION / 'models' / name).read_text()
        assert '<static>true</static>' in text, f'{name} must be static'
        assert 'gz-sim-sensors-system' in text, (
            f'{name} does not document the sensors plugin dependency; without '
            'it the sensor exists and publishes nothing, with no error in the log'
        )


# --------------------------------------------------------------------------
# Simulation and camera control from the cockpit (operator request,
# 24/08/2026: "start the simulation from the cockpit, reset it, move, rotate
# and zoom the simulation screen").
# --------------------------------------------------------------------------

SIM_CONTROL_LAUNCH = SIMULATION / 'launch' / 'sim_control.launch.py'
SIM_CONTROL_RELAY = SIMULATION / 'demo_simulation' / 'sim_control_relay.py'
SIM_CONTROLS_JS = BUNDLE / 'js' / 'panels' / 'sim-controls.js'
VIEW_CONTROLS_JS = BUNDLE / 'js' / 'panels' / 'view-controls.js'
TOKENS_CSS = BUNDLE / 'css' / 'tokens.css'
INDEX_HTML = BUNDLE / 'index.html'


def _code_lines(source: str) -> list[str]:
    """
    Code lines, without comments.

    These files' headers EXPLAIN why the browser does not speak Gazebo's
    vocabulary, and cite the names while doing it. A naive text search would
    forbid exactly the documentation of the rule.
    """
    lines, in_block = [], False
    for raw in source.splitlines():
        line = raw.strip()
        if in_block:
            if '*/' in line:
                in_block = False
            continue
        if line.startswith('/*'):
            in_block = '*/' not in line
            continue
        if line.startswith('//'):
            continue
        lines.append(raw)
    return lines


def test_browser_never_speaks_gazebo_interfaces():
    """
    The browser calls std_srvs, never ros_gz_interfaces.

    rosbridge builds the request by importing the interfaces package inside
    the container itself, and the cockpit container does not have
    ros_gz_interfaces — in `deploy` mode, with no Gazebo at all, it would not
    even make sense to have it. The failure is an InvalidModuleException at
    click time, far from any test.
    """
    for path in (SIM_CONTROLS_JS, VIEW_CONTROLS_JS):
        code = '\n'.join(_code_lines(path.read_text(encoding='utf-8')))
        assert 'callService(' in code, f'{path.name} does not call any service'
        assert 'ros_gz_interfaces' not in code, (
            f'{path.name} references ros_gz_interfaces in code'
        )
        assert '/demo/sim/control' not in code, (
            f'{path.name} calls the service directly with the Gazebo type; use '
            'the sim_control_relay std_srvs facade'
        )
        assert 'world_control' not in code, (
            f'{path.name} builds a WorldControl; that vocabulary belongs to '
            'the simulator'
        )


def test_sim_control_relay_ships_with_the_bridge():
    """The facade and the bridge come up together: one without the other is a dead button."""
    launch = SIM_CONTROL_LAUNCH.read_text(encoding='utf-8')
    assert "executable='sim_control_relay'" in launch
    assert 'ros_gz_bridge' in launch

    entry_points = (SIMULATION / 'setup.py').read_text(encoding='utf-8')
    assert 'sim_control_relay = demo_simulation.sim_control_relay:main' in entry_points


def test_sim_control_relay_serves_the_three_actions():
    """The cockpit's three buttons need a service on the other side."""
    relay = SIM_CONTROL_RELAY.read_text(encoding='utf-8')
    # The PATHS, not the names: `play` and `pause` come out of a loop and
    # `reset` has its own handler, and the browser only knows the path.
    assert "f'/demo/sim/{action}'" in relay
    assert "for action in ('play', 'pause')" in relay
    assert "'/demo/sim/reset'" in relay, 'reset is not served by the facade'


def test_reset_must_not_delete_the_robot_again():
    """
    `reset.all` DELETES the plant, and the cockpit has no way to notice.

    The robot and the two scene cameras are inserted after the world loads
    (`ros_gz_sim create`); `reset.all` returns the world to its source SDF,
    which does not contain them. Measured on 26/08/2026: `/joint_states` 999 Hz
    -> dead, `/demo/imu` 996 Hz -> dead, `/demo/odom` 49.6 Hz -> dead, and
    `gz model -m demo_robot` answering `No model named <demo_robot>`.

    What makes this worth a guard is the APPEARANCE: the clock keeps running
    at 999 Hz and Gazebo leaves the orphaned sensors publishing at 10 Hz, so
    every cockpit panel turns green pointing at a plant that no longer exists.
    Nothing in the log flags it. Evidence in
    `docs/results/cockpit-reset-nao-destrutivo.md`.
    """
    relay = SIM_CONTROL_RELAY.read_text(encoding='utf-8')
    # The field, not the word: the header CITES `reset.all` on purpose, so the
    # next person knows why it is not being used. What must not exist is the
    # assignment.
    assert 'request.world_control.reset' not in relay, (
        'the facade must not write to any WorldControl reset field; '
        'reset.all deletes the robot and time_only jumps the clock backwards'
    )
    assert 'reset.all = True' not in relay
    # The new path: teleport through the same facade the scene cameras use.
    assert 'SetEntityPose' in relay
    assert "'/demo/sim/set_entity_pose'" in relay


def test_sim_state_label_comes_from_the_clock():
    """
    The state label must not be the echo of the last click.

    A dead simulator, an expired call, or a pause made through the Gazebo GUI
    produce exactly the case where the echo lies.
    """
    source = SIM_CONTROLS_JS.read_text(encoding='utf-8')
    assert "'/clock'" in source
    assert 'throttleRate' in source, (
        '/clock publishes at ~1 kHz; without throttling the full traffic goes '
        'to the browser'
    )


def test_view_pad_buttons_all_have_a_step():
    """A data-command without a step is a button that does nothing and never complains."""
    html = INDEX_HTML.read_text(encoding='utf-8')
    commands = set(re.findall(r'data-role="view" data-command="([a-z-]+)"', html))
    steps = set(re.findall(r"^  '([a-z-]+)':", VIEW_CONTROLS_JS.read_text(encoding='utf-8'),
                           flags=re.MULTILINE))
    assert commands, 'no camera button in the HTML'
    assert commands == steps, f'missing step: {commands - steps}; missing button: {steps - commands}'


def test_brand_palette_is_the_toradex_one():
    """
    The three brand values are exact and came from the team. A "just one
    shade darker" tweak to one of them is how an identity gets lost.
    """
    tokens = TOKENS_CSS.read_text(encoding='utf-8')
    for name, value in (
        ('--brand-blue', '#00508c'),
        ('--brand-green', '#96c837'),
        ('--brand-orange', '#ff5a00'),
    ):
        assert f'{name}: {value};' in tokens, f'{name} stopped being {value}'
    assert '--bg-root: #ffffff;' in tokens, 'the cockpit background is white'


def test_canvas_colours_come_from_the_tokens():
    """
    No canvas panel keeps its own colour.

    When the cockpit swapped the black background for the brand's white, the
    plane's cyan and the laser's yellow disappeared along with it. Colour
    duplicated in JavaScript is what makes the next theme swap fix four panels
    and forget two.
    """
    for name in ('nav-panel.js', 'detection-overlay.js'):
        source = (BUNDLE / 'js' / 'panels' / name).read_text(encoding='utf-8')
        literals = re.findall(r"= '(#[0-9a-fA-F]{3,8})'", source)
        assert not literals, f'{name} still has a literal colour: {literals}'
        assert 'readMapPalette' in source


def test_logos_are_present_and_have_alpha():
    """
    The brand marks are white ink with alpha, which is why the bar is blue.

    An opaque PNG here would turn into a white rectangle over the blue — and
    the test exists because the file is easy to replace with a flattened JPG.
    """
    for name in ('toradex.png', 'ros.png'):
        blob = (BUNDLE / 'img' / name).read_bytes()
        assert blob[:8] == b'\x89PNG\r\n\x1a\n', f'{name} is not a PNG'
        # Byte 25 of the IHDR is the color type; 6 = RGBA, 4 = grey+alpha.
        assert blob[25] in (4, 6), f'{name} has no alpha channel'

    html = INDEX_HTML.read_text(encoding='utf-8')
    assert 'img/toradex.png' in html and 'img/ros.png' in html
    assert 'alt="Toradex"' in html and 'alt="ROS 2"' in html, (
        'without alt, a kiosk with no image does not say whose demo this is'
    )


def test_stub_detections_are_not_drawn_over_the_video():
    """
    Nothing draws the stub's detections over the camera.

    Today's `demo_perception` sweeps a synthetic box across the image whether
    there is an object there or not. Over the video that turns into a
    rectangle wandering from one side to the other, and in a demo the viewer
    reads that as a real detection.

    What is NOT under test: detections continuing to arrive and feed the
    costmap's perception_layer. That is CLAUDE.md's topic contract and it did
    not change — what left was only the drawing. The overlay module stays in
    the bundle, tested, for when TIDL replaces the stub.
    """
    html = INDEX_HTML.read_text(encoding='utf-8')
    assert 'data-role="detections"' not in html, (
        'the detections canvas came back to the camera panel'
    )
    main = (BUNDLE / 'js' / 'main.js').read_text(encoding='utf-8')
    assert 'createDetectionOverlay' not in main
    assert (BUNDLE / 'js' / 'panels' / 'detection-overlay.js').exists(), (
        'the overlay must stay in the bundle: it comes back with TIDL'
    )


def test_scene_cameras_keep_the_measured_aspect_ratio():
    """
    Resolution can go up; the 4:3 ratio must not change.

    The horizontal_fov and the poses of both cameras were measured at this
    ratio (see the header of scene_cameras.launch.py). Going to 16:9 while
    keeping the hfov crops vertically and misframes both scenes at once — with
    no error at all, just a robot out of frame.
    """
    for name in ('cockpit_scene_iso.sdf', 'cockpit_scene_top.sdf'):
        sdf = (SIMULATION / 'models' / name).read_text(encoding='utf-8')
        width = int(re.search(r'<width>(\d+)</width>', sdf).group(1))
        height = int(re.search(r'<height>(\d+)</height>', sdf).group(1))
        assert width * 3 == height * 4, f'{name} stopped being 4:3 ({width}x{height})'


# --------------------------------------------------------------------------
# Camera following the robot, and navigation restart from the cockpit
# (operator request, 24/08/2026: bigger logo, reset the goal by restarting
# the navigation ROS stack on the Aquila, and a view tracked to the robot on
# both cameras).
# --------------------------------------------------------------------------

SCENE_VIEW_CONTROLLER = SIMULATION / 'demo_simulation' / 'scene_view_controller.py'
NAVIGATION = REPO_ROOT / 'ros2_ws' / 'src' / 'demo_navigation'
NAV_CONTROL_RELAY = NAVIGATION / 'demo_navigation' / 'nav_control_relay.py'
NAV_CONTROL_LAUNCH = NAVIGATION / 'launch' / 'nav_control.launch.py'
NAV_PANEL_JS = BUNDLE / 'js' / 'panels' / 'nav-panel.js'
PANELS_CSS = BUNDLE / 'css' / 'panels.css'
LAYOUT_CSS = BUNDLE / 'css' / 'layout.css'
# The two explicit navigation entrypoints. Both bring up the
# lifecycle_manager_navigation, so both must expose the restart facade — the
# way this breaks is the button working on one ROBOT_TYPE and not the other.
NAV_ENTRYPOINTS = (
    NAVIGATION / 'launch' / 'navigation.launch.py',
    REPO_ROOT / 'ros2_ws' / 'src' / 'demo_bringup' / 'launch'
    / 'nav_quadruped.launch.py',
)


def _python_code(source: str) -> str:
    """
    Python code without comments and without docstrings.

    Same reason as `_code_lines` above, one floor deeper: these nodes' headers
    EXPLAIN why localization is not reset, and cite the manager's name while
    doing it. A naive search would forbid the documentation of the rule.

    Via ast, not regex: a docstring with triple quotes inside an f-string is
    exactly the case a regex silently gets wrong.
    """
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if not isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef,
                                 ast.AsyncFunctionDef)):
            continue
        body = node.body
        if (body and isinstance(body[0], ast.Expr)
                and isinstance(body[0].value, ast.Constant)
                and isinstance(body[0].value.value, str)):
            node.body = body[1:] or [ast.Pass()]
    return ast.unparse(ast.fix_missing_locations(tree))


def test_scene_views_follow_the_robot_on_both_cameras():
    """
    Following applies to BOTH views, and the target is the robot.

    An implementation that only followed the iso view would pass any "does
    following exist" test and fail exactly on the iso/top button: the top
    view is the one the operator uses to watch the robot walk through the
    maze.
    """
    code = SCENE_VIEW_CONTROLLER.read_text(encoding='utf-8')
    assert 'def follow(self, anchor)' in code, (
        'the Orbit has no way to follow a moving target'
    )
    # The tick loop iterates self._orbits, which holds both cameras. A
    # literal `self._orbits['scene_iso']` here would be the regression.
    assert 'for name, orbit in self._orbits.items()' in code
    assert "'scene_iso'" in code and "'scene_top'" in code


def test_scene_follow_state_is_published_not_echoed():
    """
    The `follow` button is painted by the node, not by the click itself.

    Same rule as the simulation label: reloading the page, opening the
    cockpit on a second screen, or turning following off from the command
    line are three cases where the local click does not know the answer.
    """
    node = SCENE_VIEW_CONTROLLER.read_text(encoding='utf-8')
    assert "'/demo/cockpit/scene/following'" in node, (
        'the node does not publish the following state'
    )
    assert 'TRANSIENT_LOCAL' in node, (
        'without latched durability a new tab has no value until the next '
        'change'
    )

    code = '\n'.join(_code_lines(VIEW_CONTROLS_JS.read_text(encoding='utf-8')))
    assert 'FOLLOWING_TOPIC' in code and 'client.subscribe(' in code, (
        'the cockpit does not read the following state from any topic'
    )
    assert 'following = wanted' not in code, (
        'the button is being painted by the click; the value must come from '
        'the node'
    )


def test_follow_anchor_carries_the_odom_to_world_seed():
    """
    /demo/odom is not the pose in the world on both plants, and the difference is silent.

    On the quadruped it is Gazebo ground truth. On diff-drive the DiffDrive
    plugin integrates encoders from zero, so the odom origin is the SPAWN
    pose. Without the seed, an `x:=5` makes the camera follow a point 5 m
    beside the robot — and a `yaw:=` makes the error grow with distance.
    """
    node = SCENE_VIEW_CONTROLLER.read_text(encoding='utf-8')
    for name in ('follow_offset_x', 'follow_offset_y', 'follow_offset_yaw'):
        assert f"'{name}'" in node, f'{name} is not declared by the node'
    # The yaw must be APPLIED, not just declared: an ignored rotation is
    # exactly the error that grows with distance travelled.
    assert 'math.cos(syaw)' in node and 'math.sin(syaw)' in node, (
        'the yaw seed is declared and unused'
    )

    # The diff-drive plant passes the spawn pose; the quadruped plant passes
    # NOTHING, because it would add the pose twice on top of an odometry
    # that is already world-frame.
    diffdrive = (SIMULATION / 'launch' / 'simulation.launch.py').read_text(
        encoding='utf-8')
    assert "'follow_offset_x': LaunchConfiguration('x')" in diffdrive
    assert "'follow_offset_yaw': LaunchConfiguration('yaw')" in diffdrive

    quadruped = (SIMULATION / 'launch' / 'quadruped.launch.py').read_text(
        encoding='utf-8')
    assert 'follow_offset' not in quadruped, (
        'the quadruped plant has ground-truth odometry: a seed here would add '
        'the spawn pose twice'
    )


def test_nav_reset_facade_ships_with_both_navigation_paths():
    """
    The restart facade ships on BOTH navigation paths.

    Both bring up `lifecycle_manager_navigation`, so both can be restarted.
    If it only shipped on one, the cockpit button would work on one
    ROBOT_TYPE and not the other — with no error anywhere, because the
    service would simply not exist.
    """
    for path in NAV_ENTRYPOINTS:
        source = path.read_text(encoding='utf-8')
        assert 'nav_control.launch.py' in source, (
            f'{path.name} brings up Nav2 without the restart facade'
        )

    launch = NAV_CONTROL_LAUNCH.read_text(encoding='utf-8')
    assert "executable='nav_control_relay'" in launch

    entry_points = (NAVIGATION / 'setup.py').read_text(encoding='utf-8')
    assert (
        'nav_control_relay = demo_navigation.nav_control_relay:main'
        in entry_points
    ), 'the executable is not registered; the launch fails to find it'


def test_nav_reset_never_uses_reset_startup_because_it_segfaults():
    """
    Restart is PAUSE + RESUME, never RESET + STARTUP.

    Measured on 24/08/2026, learn, quadruped path: RESET followed by STARTUP
    kills the `component_container_isolated` with SIGSEGV (exit code -11),
    always on the second CONFIGURE of `route_server`, in "Configuring
    Rerouting service operation". Two attempts, two identical deaths. After
    that there is no navigation at all — only `docker compose restart nav`
    brings it back.

    `route_server` is in the vendored navigation_launch.py's `lifecycle_nodes`
    list, which is an upstream copy and must stay identical, and this project
    uses no routing at all. As long as it stays in the managed list, RESET is
    forbidden: a button meant to fix navigation that kills navigation is
    worse than no button.
    """
    relay = NAV_CONTROL_RELAY.read_text(encoding='utf-8')
    code = _python_code(relay)
    assert 'ManageLifecycleNodes.Request.PAUSE' in code
    assert 'ManageLifecycleNodes.Request.RESUME' in code
    for forbidden in ('Request.RESET', 'Request.STARTUP'):
        assert forbidden not in code, (
            f'{forbidden} goes back through route_server\'s CONFIGURE, which '
            'kills the container — see this test\'s docstring'
        )
    # The alternative to CONFIGURE: the costmap is cleared by the costmap
    # nodes' own services, without going through the lifecycle. Without this
    # the reset would leave the phantom obstacle that ruins a long demo.
    assert 'clear_entirely_global_costmap' in code
    assert 'clear_entirely_local_costmap' in code

    assert 'lifecycle_manager_localization' not in code, (
        'the facade is touching localization too; see its header'
    )
    # The timeout is measured on the WALL clock. This node runs with
    # use_sim_time and a deactivated Nav2 coexists with a stopped /clock:
    # measuring on simulated time turns "timed out" into "wait forever".
    assert 'time.monotonic()' in relay
    assert 'self.get_clock()' not in relay


def test_nav_reset_needs_two_clicks_and_the_browser_speaks_std_srvs():
    """
    Restarting is destructive and takes tens of seconds: two clicks, like the
    simulation reset.

    And the call is the std_srvs facade, not manage_nodes. Here `nav2_msgs`
    does exist in the cockpit container — it's the package NavigateToPose's
    goal uses — so the reason is not the Gazebo trap: it's that the sequence
    has an invalid state in the middle and cannot depend on the page staying
    open.
    """
    code = '\n'.join(_code_lines(NAV_PANEL_JS.read_text(encoding='utf-8')))
    assert "'/demo/nav/reset'" in code, 'the panel does not call the facade'
    assert 'manage_nodes' not in code, (
        'the browser is driving Nav2\'s lifecycle directly; an F5 in the '
        'middle leaves the stack deactivated'
    )
    assert 'RESET_ARM_MS' in code and "dataset.armed = 'true'" in code, (
        'restarting navigation is not gated behind confirmation'
    )


def test_nav_map_has_bounded_zoom_controls_that_do_not_send_goals():
    html = INDEX_HTML.read_text(encoding='utf-8')
    source = NAV_PANEL_JS.read_text(encoding='utf-8')
    view = (BUNDLE / 'js' / 'panels' / 'map-view.js').read_text(encoding='utf-8')

    assert 'data-role="nav-zoom-in"' in html
    assert 'data-role="nav-zoom-out"' in html
    assert 'DEFAULT_MAP_ZOOM = 0.5' in view
    assert 'MIN_MAP_ZOOM = 0.125' in view and 'MAX_MAP_ZOOM = 4' in view
    assert "stepMapZoom(state.zoom, direction)" in source
    # Defensive even though the controls are canvas siblings today: moving the
    # overlay during a layout refactor must not turn zoom into a navigation goal.
    zoom_handlers = source[source.index("zoomInButton?.addEventListener"):
                           source.index('// --- restart navigation')]
    assert zoom_handlers.count('event.stopPropagation()') == 2


def test_toradex_logo_doubled_and_the_bar_grew_with_it():
    """
    The Toradex mark is 2x the original, and the bar reserves height for it.

    The original 34 px was tuned on screen against the ROS logo; the request
    was to double the Toradex one. What this test guards is not the number:
    it is the coupling. `.bar` has overflow-x and not -y, so raising the logo
    without raising the grid row's minimum CROPS the mark, with no scrollbar
    and no error at all.
    """
    tokens = TOKENS_CSS.read_text(encoding='utf-8')
    toradex = int(re.search(r'--logo-toradex:\s*(\d+)px', tokens).group(1))
    bar_min = int(re.search(r'--bar-min-height:\s*(\d+)px', tokens).group(1))

    assert toradex >= 68, (
        f'the Toradex mark went back to {toradex}px; the request was at '
        'least 2x the original 34px'
    )
    assert bar_min >= toradex + 10, (
        f'the bar reserves {bar_min}px for a {toradex}px logo plus 10px of '
        'padding: the mark gets cropped'
    )

    # The height comes from the token on both sides. A literal px value on
    # either one is exactly how the two numbers drift apart.
    panels = PANELS_CSS.read_text(encoding='utf-8')
    assert 'height: var(--logo-toradex)' in panels
    layout = LAYOUT_CSS.read_text(encoding='utf-8')
    assert 'minmax(var(--bar-min-height)' in layout
    assert 'minmax(48px' not in layout, (
        'the bar row went back to a literal minimum, out of sync with the logo'
    )
