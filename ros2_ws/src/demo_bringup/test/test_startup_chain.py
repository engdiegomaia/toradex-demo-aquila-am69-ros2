"""
The quadruped Nav2 startup chain: ordering, not elapsed time.

Structural: they read the launch file and YAML with `ast`/`yaml`, they do not
bring up ROS.

What they protect cost bench time. On 26/08/2026, on the Aquila AM69, the five
nodes of `nav_quadruped.launch.py` were emitted TOGETHER. The `odom -> base`
edge took more than 60 s to cross the container boundary, the `local_costmap`
did not activate, and the lifecycle manager ABORTED the bringup for good. The
container stayed up, all topics appeared, `module.sh verify` returned 0, and
every goal was rejected with "Action server is inactive".
"""

from __future__ import annotations

import ast
from pathlib import Path

import yaml


SRC = Path(__file__).resolve().parents[2]
LAUNCH = SRC / 'demo_bringup' / 'launch' / 'nav_quadruped.launch.py'
PARAMS = SRC / 'demo_navigation' / 'config' / 'nav2_params_go2.yaml'
WAIT_FOR_TF = SRC / 'demo_bringup' / 'demo_bringup' / 'wait_for_tf.py'

# Emitting these together with the rest is exactly the measured defect.
MUST_BE_GATED = (
    'cloud_to_scan', 'slam', 'nav2_container', 'navigation', 'nav_control',
)


def _tree() -> ast.Module:
    return ast.parse(LAUNCH.read_text(encoding='utf-8'))


def _launch_description_elements() -> list[str]:
    """Names emitted DIRECTLY in LaunchDescription([...])."""
    for call in ast.walk(_tree()):
        if (isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
                and call.func.id == 'LaunchDescription' and call.args):
            return [e.id for e in call.args[0].elts if isinstance(e, ast.Name)]
    raise AssertionError('LaunchDescription([...]) not found')


def _node_params(executable: str) -> dict:
    for call in ast.walk(_tree()):
        if not isinstance(call, ast.Call):
            continue
        kw = {k.arg: k.value for k in call.keywords if k.arg}
        found = kw.get('executable')
        if not isinstance(found, ast.Constant) or found.value != executable:
            continue
        params: dict = {}
        for entry in getattr(kw.get('parameters'), 'elts', []):
            if not isinstance(entry, ast.Dict):
                continue
            for key, value in zip(entry.keys, entry.values):
                if isinstance(key, ast.Constant):
                    params[key.value] = (value.value
                                         if isinstance(value, ast.Constant)
                                         else 'dynamic')
        return params
    raise AssertionError(f'Node(executable={executable!r}) does not exist')


def test_nav2_is_not_emitted_alongside_everything_else() -> None:
    emitted = _launch_description_elements()
    for name in MUST_BE_GATED:
        assert name not in emitted, (
            f'{name} is emitted directly in LaunchDescription. That races it '
            f'against the odom -> base TF: if the TF is slow, the local_costmap '
            f'does not activate and the manager aborts the bringup FOREVER.'
        )


def test_the_edge_producer_starts_immediately() -> None:
    """`odom_tf` PRODUCES the edge the gate waits for; gating it deadlocks everything."""
    assert 'odom_tf' in _launch_description_elements()


def test_nav2_is_gated_on_the_tf_gate_finishing() -> None:
    handlers = [
        call for call in ast.walk(_tree())
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
        and call.func.id == 'OnProcessExit'
    ]
    assert len(handlers) >= 2, 'the wait_for_clock -> wait_for_tf -> Nav2 chain is gone'

    gated_on_tf = [
        h for h in handlers
        if any(isinstance(n, ast.Name) and n.id == 'wait_for_tf'
               for kw in h.keywords if kw.arg == 'target_action'
               for n in ast.walk(kw.value))
    ]
    assert gated_on_tf, 'nothing is gated on wait_for_tf finishing'

    following = ast.unparse(gated_on_tf[0])
    for name in MUST_BE_GATED:
        assert name in following, f'{name} is not behind the TF gate'


def test_the_gate_watches_the_frames_the_costmap_demands() -> None:
    """
    The gate must wait for the SAME edge that makes the costmap activate.

    If someone changes `robot_base_frame` in the YAML and not here, the gate
    releases with the wrong edge available and the defect comes back whole, now
    with a green test nearby.
    """
    params = _node_params('wait_for_tf')
    costmap = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    local = costmap['local_costmap']['local_costmap']['ros__parameters']

    assert params['parent_frame'] == local['global_frame']
    assert params['child_frame'] == local['robot_base_frame']


def test_tf_listener_and_polling_share_one_executor() -> None:
    """The listener and spin_once must not register the node in distinct executors."""
    tree = ast.parse(WAIT_FOR_TF.read_text(encoding='utf-8'))
    listener = next(
        call for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == 'TransformListener'
    )
    keywords = {kw.arg: kw.value for kw in listener.keywords if kw.arg}

    assert 'spin_thread' not in keywords
    assert any(
        isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == 'spin_once'
        for call in ast.walk(tree)
    )


def test_the_gate_fails_loud_instead_of_starting_nav2_anyway() -> None:
    """
    `OnProcessExit` fires on ANY exit, including errors.

    Via the AST, not `'returncode' in source`: the first version of this test
    did that and SURVIVED the mutation that replaced the `if` with `if True`,
    because the word kept appearing in the adjacent log message.
    """
    handler = next(
        (fn for fn in ast.walk(_tree())
         if isinstance(fn, ast.FunctionDef) and fn.name == '_on_exit'), None)
    assert handler, 'the gate exit callable is gone'

    branch = next((n for n in ast.walk(handler) if isinstance(n, ast.If)), None)
    assert branch, 'the gate does not branch: it releases Nav2 on any exit'

    condition = ast.unparse(branch.test)
    assert 'returncode' in condition and '0' in condition, (
        f'the gate condition is {condition!r}. It must look at the exit '
        f'code: a wait_for_tf that TIMED OUT cannot release Nav2, which is '
        f'exactly the original defect.'
    )

    # The failure branch can be in an `else` or right after the `if` that returns
    # early. Both forms are correct; what cannot be is the Shutdown being on the
    # SUCCESS path, or not existing.
    success_path = ast.unparse(ast.Module(body=branch.body, type_ignores=[]))
    whole = ast.unparse(handler)
    assert 'Shutdown(' in whole, (
        'the failure branch does not bring the launch down, so failure is silent again'
    )
    assert 'Shutdown(' not in success_path, (
        'the gate brings the launch down even when the prerequisite WAS satisfied'
    )
