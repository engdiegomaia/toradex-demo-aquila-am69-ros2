"""
Contracts for the joint_state_broadcaster decimation.

Exists because the change these tests protect has a failure mode that is cheap
to commit and expensive to diagnose: lowering the LOOP rate instead of the
BROADCASTER rate. Both edits are one line, live in the same file, and the
wrong one breaks the gait -- whose behavior was measured across four reverted
experiments (docs/results/ml35-f4-parcial.md) against a controller_manager at
1000 Hz.

What gets decimated is the /joint_states publisher, measured at ~1090 Hz on
/tf in the HIL run of 28/08/2026 (docs/results/ml35-f5-tf-cpu-baseline.md §6).
What is NOT decimated is everything else: loop at 1000, gait at 200, physics
at 1 ms, IMU and odometry left as they were.
"""

from __future__ import annotations

import ast
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / 'ros2_ws/src/demo_simulation/launch/quadruped.launch.py'
JSB_PARAMS = (ROOT / 'ros2_ws/src/demo_simulation/config'
              / 'joint_state_broadcaster.yaml')
GAIT_PARAMS = ROOT / 'ros2_ws/src/demo_simulation/config/gait_go2.yaml'
GAZEBO_CONFIG = ROOT / 'ros2_ws/src/go2_description/config/gazebo.yaml'
ROBOT_CONFIG = ROOT / 'ros2_ws/src/go2_description/config/robot_control.yaml'
WORLDS = sorted((ROOT / 'ros2_ws/src/demo_simulation/worlds').glob('*.sdf'))

# The control loop rate, which is the same as the 1 ms physics step.
LOOP_RATE_HZ = 1000
# The broadcaster rate after decimation. Matched to /demo/odom, which is the
# dynamic edge navigation expects at 50 Hz.
BROADCASTER_RATE_HZ = 50
# The gait rate, measured under ML3.5 F2/F4.
GAIT_RATE_HZ = 200


def _yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _launch_tree() -> ast.Module:
    return ast.parse(LAUNCH.read_text(encoding='utf-8'))


def _assignment(name: str) -> str:
    node = next(
        item for item in ast.walk(_launch_tree())
        if isinstance(item, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == name
                for target in item.targets)
    )
    return ast.unparse(node)


# --- the global rate does not move ---------------------------------------

def test_the_control_loop_still_runs_at_one_kilohertz() -> None:
    for path in (GAZEBO_CONFIG, ROBOT_CONFIG):
        params = _yaml(path)['controller_manager']['ros__parameters']
        assert params['update_rate'] == LOOP_RATE_HZ, path.name


def test_physics_still_steps_at_one_millisecond() -> None:
    # Decimating the broadcaster must not, by mistake, turn into a slower
    # world: it would be the same CPU saving through the wrong path, and
    # would invalidate the entire measured gait.
    assert WORLDS, 'no world found'
    for world in WORLDS:
        assert '<max_step_size>0.001</max_step_size>' in world.read_text(
            encoding='utf-8'), world.name


def test_the_gait_controller_keeps_the_rate_its_tuning_was_measured_against(
) -> None:
    for path in (GAZEBO_CONFIG, ROBOT_CONFIG):
        params = _yaml(path)['unitree_guide_controller']['ros__parameters']
        assert params['update_rate'] == GAIT_RATE_HZ, path.name


def test_the_two_vendored_configs_agree_on_the_rates_this_demo_uses() -> None:
    # gazebo.yaml and robot_control.yaml are the same stack in simulation and
    # on hardware. A rate that diverges between the two is a bug that only
    # shows up on the side nobody ran that week.
    #
    # The list is restricted to what THIS project loads. `ocs2_quadruped_controller`
    # already diverges upstream (500 Hz in simulation vs 200 on hardware) and
    # `rl_quadruped_controller` depends on .pt weights that were dropped when
    # go2_description was vendored. Including both here would mean adopting a
    # divergence that is not ours and that nobody on this side can resolve.
    gazebo, robot = _yaml(GAZEBO_CONFIG), _yaml(ROBOT_CONFIG)
    for section in ('controller_manager', 'unitree_guide_controller'):
        left = gazebo[section]['ros__parameters'].get('update_rate')
        right = robot[section]['ros__parameters'].get('update_rate')
        assert left == right, section


# --- only the broadcaster is decimated ------------------------------------

def test_the_broadcaster_is_decimated_to_fifty_hertz() -> None:
    params = _yaml(JSB_PARAMS)['joint_state_broadcaster']['ros__parameters']
    assert params['update_rate'] == BROADCASTER_RATE_HZ


def test_the_broadcaster_rate_divides_the_loop_rate_exactly() -> None:
    # The ControllerManager decimates by an integer factor. A rate that does
    # not divide 1000 is not rejected -- it is rounded, and /joint_states
    # comes out at a cadence that is not the one the YAML states.
    assert LOOP_RATE_HZ % BROADCASTER_RATE_HZ == 0


def test_the_broadcaster_param_file_touches_nothing_but_the_broadcaster(
) -> None:
    document = _yaml(JSB_PARAMS)
    assert list(document) == ['joint_state_broadcaster']
    assert list(document['joint_state_broadcaster']['ros__parameters']) == [
        'update_rate']


def test_no_project_param_file_decimates_the_imu_or_the_gait() -> None:
    # The scan covers the entire demo_simulation config directory, not a
    # list of known files, because the defect it is chasing is someone
    # adding a new file.
    config_dir = ROOT / 'ros2_ws/src/demo_simulation/config'
    for path in sorted(config_dir.glob('*.yaml')):
        document = _yaml(path)
        # bridge_quadruped.yaml is a LIST of bridges, not a map of
        # controllers. Skipping instead of blowing up keeps the scan useful
        # when the directory gains more files that are not param files.
        if not isinstance(document, dict):
            continue
        for controller, block in document.items():
            if controller == 'joint_state_broadcaster':
                continue
            params = (block or {}).get('ros__parameters', {})
            assert 'update_rate' not in params, f'{path.name}:{controller}'


def test_the_gait_tuning_file_carries_no_rate_at_all() -> None:
    # gait_go2.yaml is the other param file applied by the same mechanism. It
    # exists for gait tuning; a rate appearing there would start competing
    # with gazebo.yaml without anything flagging the difference.
    params = _yaml(GAIT_PARAMS)['unitree_guide_controller']['ros__parameters']
    assert 'update_rate' not in params
    assert 'joint_state_broadcaster' not in _yaml(GAIT_PARAMS)


def test_the_vendored_config_never_declares_a_broadcaster_rate() -> None:
    # go2_description is vendored and byte-identical to upstream, and that
    # guarantee is what backs the license argument. The decimation lives in
    # the project.
    for path in (GAZEBO_CONFIG, ROBOT_CONFIG):
        document = _yaml(path)
        block = document.get('joint_state_broadcaster')
        assert block is None or 'ros__parameters' not in block, path.name


# --- the launch applies the file, and only the file -----------------------

def test_the_spawner_applies_the_broadcaster_param_file() -> None:
    source = _assignment('joint_state_broadcaster')
    assert "'joint_state_broadcaster'" in source
    assert "'--param-file'" in source
    assert "LaunchConfiguration('jsb_params')" in source


def test_the_imu_broadcaster_is_left_at_the_inherited_rate() -> None:
    source = _assignment('imu_sensor_broadcaster')
    assert "'--param-file'" not in source


def test_the_gait_spawner_still_loads_only_its_own_tuning() -> None:
    source = _assignment('unitree_guide_controller')
    assert "LaunchConfiguration('gait_params')" in source
    assert "jsb_params" not in source


def test_the_decimation_is_overridable_and_defaults_to_the_project_file(
) -> None:
    source = _assignment('jsb_params_arg')
    assert "'jsb_params'" in source
    assert "'joint_state_broadcaster.yaml'" in source
    assert "'demo_simulation'" in source


def test_the_new_argument_is_actually_registered() -> None:
    # A DeclareLaunchArgument that does not enter the LaunchDescription does not
    # exist: the LaunchConfiguration fails at runtime, inside the container.
    description = next(
        ast.unparse(node) for node in ast.walk(_launch_tree())
        if isinstance(node, ast.Return)
        and isinstance(node.value, ast.Call)
        and getattr(node.value.func, 'id', None) == 'LaunchDescription'
    )
    assert 'jsb_params_arg' in description
