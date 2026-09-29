"""
Those who don't ask the time don't subscribe to the clock.

`use_sim_time: true` is NOT a harmless statement of intent: rclpy creates a
`/clock` subscription per node, regardless of whether the node's code ever
calls the clock. On the Aquila AM69 Gazebo publishes `/clock` at ~870 Hz
(1 ms physics step, required by the gait), and the measured price per node
that only receives and discards those messages was 35-40% of a core.

Measurement from 25/08/2026, `nav` container on the AM69, stack up and WITH NO
ACTIVE GOAL, sampling /proc/<tid>/stat per thread: idle floor of 367% of 800%,
of which 111% were three Python republishers that never call the clock even
once.

These tests are structural -- they read launch files and source with `ast`,
they do not bring up ROS. They lock the invariant in BOTH directions, and it
is the second one that matters:

  1. the listed nodes come up with use_sim_time literally False;
  2. those same nodes do NOT call get_clock() on the hot path.

Without (2) the test would become a rubber stamp: someone adds a timer that
reads the clock, the node starts reading WALL time thinking it reads
simulated time, and nothing flags it -- the stamps end up years in the
future and the costmap drops the reading with "message filter dropping
message", which never names the cause.
"""

from __future__ import annotations

import ast
from pathlib import Path


SRC = Path(__file__).resolve().parents[2]
QUADRUPED_LAUNCH = SRC / 'demo_bringup' / 'launch' / 'nav_quadruped.launch.py'
NAV_CONTROL_LAUNCH = SRC / 'demo_navigation' / 'launch' / 'nav_control.launch.py'

# executable -> source that implements it.
CLOCKLESS_NODES = {
    'cmd_vel_si_to_stick':
        SRC / 'demo_bringup' / 'demo_bringup' / 'cmd_vel_si_to_stick.py',
    'nav_control_relay':
        SRC / 'demo_navigation' / 'demo_navigation' / 'nav_control_relay.py',
    'odom_tf':
        SRC / 'demo_bringup' / 'demo_bringup' / 'odom_tf.py',
}

# odom_tf is the justified exception: it stamps the STATIC map -> odom edge
# with the clock, and tf2's static buffer returns the static transform for
# any instant queried -- that stamp never enters any lookup.
CLOCK_ALLOWED_IN = {'odom_tf': {'_identity'}}


def _node_params(launch_path: Path, executable: str) -> dict:
    """Extract the constant parameters of the requested Node(executable=...)."""
    tree = ast.parse(launch_path.read_text(encoding='utf-8'))

    for call in (n for n in ast.walk(tree) if isinstance(n, ast.Call)):
        kw = {k.arg: k.value for k in call.keywords if k.arg}
        found = kw.get('executable')
        if not isinstance(found, ast.Constant) or found.value != executable:
            continue

        params: dict[str, object] = {}
        for entry in getattr(kw.get('parameters'), 'elts', []):
            if not isinstance(entry, ast.Dict):
                continue
            for key, value in zip(entry.keys, entry.values):
                if not isinstance(key, ast.Constant):
                    continue
                # A constant becomes a value; LaunchConfiguration(...) becomes
                # the 'dynamic' marker, which is exactly what this test needs
                # to fail on.
                params[key.value] = (value.value if isinstance(value, ast.Constant)
                                     else 'dynamic')
        return params

    raise AssertionError(f'Node(executable={executable!r}) does not exist in {launch_path.name}')


def _clock_calls(source: Path) -> set[str]:
    """Functions in the file that call self.get_clock()."""
    tree = ast.parse(source.read_text(encoding='utf-8'))
    callers: set[str] = set()

    for func in (n for n in ast.walk(tree)
                 if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))):
        for node in ast.walk(func):
            if (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and node.func.attr == 'get_clock'):
                callers.add(func.name)
    return callers


def test_glue_nodes_do_not_follow_sim_time() -> None:
    for executable, launch in (('odom_tf', QUADRUPED_LAUNCH),
                               ('cmd_vel_si_to_stick', QUADRUPED_LAUNCH),
                               ('nav_control_relay', NAV_CONTROL_LAUNCH)):
        params = _node_params(launch, executable)
        assert params.get('use_sim_time') is False, (
            f'{executable} comes up with use_sim_time={params.get("use_sim_time")!r}. '
            f'It does not use the clock, and subscribing to /clock at ~870 Hz cost '
            f'35-40% of a core on the AM69.'
        )


def test_nodes_without_sim_time_do_not_read_the_clock() -> None:
    for executable, source in CLOCKLESS_NODES.items():
        offenders = _clock_calls(source) - CLOCK_ALLOWED_IN.get(executable, set())
        assert not offenders, (
            f'{executable} comes up without use_sim_time but calls get_clock() in '
            f'{sorted(offenders)}. That clock is the WALL clock: either the node goes '
            f'back to following /clock, or the call goes away.'
        )


def test_odom_tf_hot_path_copies_the_message_stamp() -> None:
    """The odom -> base edge must inherit the odometry's stamp, not the clock's."""
    tree = ast.parse(CLOCKLESS_NODES['odom_tf'].read_text(encoding='utf-8'))
    on_odom = next(n for n in ast.walk(tree)
                   if isinstance(n, ast.FunctionDef) and n.name == '_on_odom')

    assigns = [ast.unparse(n.value) for n in ast.walk(on_odom)
               if isinstance(n, ast.Assign)
               and any(ast.unparse(t).endswith('header.stamp') for t in n.targets)]
    assert assigns == ['message.header.stamp'], (
        f'_on_odom stamps the TF with {assigns}. It has to be the message stamp: '
        f'a local clock here makes the TF lead or lag the scan and the costmap '
        f'drops the reading without saying why.'
    )


def test_nav_control_launch_rejects_a_sim_time_argument() -> None:
    """An argument that is accepted and ignored is a silent failure; here it has to fail loudly."""
    tree = ast.parse(NAV_CONTROL_LAUNCH.read_text(encoding='utf-8'))
    declared = {
        call.args[0].value
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == 'DeclareLaunchArgument'
        and call.args and isinstance(call.args[0], ast.Constant)
    }
    assert 'use_sim_time' not in declared

    # By AST, not by text slice: the first version of this test matched the
    # COMMENT that explains the removal and passed the wrong tree.
    for caller in (QUADRUPED_LAUNCH,
                   SRC / 'demo_navigation' / 'launch' / 'navigation.launch.py'):
        includes = [
            call for call in ast.walk(ast.parse(caller.read_text(encoding='utf-8')))
            if isinstance(call, ast.Call)
            and isinstance(call.func, ast.Name)
            and call.func.id == 'IncludeLaunchDescription'
            and any(isinstance(n, ast.Constant) and n.value == 'nav_control.launch.py'
                    for n in ast.walk(call))
        ]
        assert len(includes) == 1, f'{caller.name}: {len(includes)} includes of nav_control'
        passed = {k.arg for k in includes[0].keywords if k.arg}
        assert 'launch_arguments' not in passed, (
            f'{caller.name} still passes launch_arguments to nav_control.launch.py, '
            f'which no longer declares use_sim_time.'
        )
