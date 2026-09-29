"""
Locks down the semantics of the cockpit reset button.

WHY THIS FILE EXISTS

The reset used to be `ControlWorld.reset.all`, and that variant DELETES THE
ROBOT. Measured on 26/08/2026 in the `quadruped_maze11` world, with a single
call to /demo/sim/reset:

    /joint_states  999 Hz -> dead         gz model -m demo_robot
    /demo/imu      996 Hz -> dead         =>  No model named <demo_robot>
    /demo/odom    49.6 Hz -> dead
    /demo/scan      10 Hz -> 10 Hz        (orphan sensor, keeps publishing)
    /clock         999 Hz -> 997 Hz

The robot is INSERTED after the world loads (`ros_gz_sim create`), and
`reset.all` returns the world to its source SDF -- which does not contain it.
The failure mode is the worst this project knows: the cockpit stays entirely
green (clock, camera, scene) pointing at a plant that no longer exists, with
not a single line of log.

Two guards, and neither needs Gazebo:

  1. the relay does not know how to assemble `reset` on the WorldControl --
     if anyone reopens that path, `_request('reset')` starts working again
     and the test fails;
  2. launch resolves the reset pose from the SCENARIO TABLE, not (0,0). In
     the maze (0,0) is not the origin of the usable area, and resetting
     there would put the robot back inside a wall -- silently;
  3. the reset STOPS the robot before teleporting and re-anchors it
     afterwards. The two following defects, measured on the same day, are
     both silent:
       - teleporting without re-anchoring: StateTrotting keeps chasing the
         previous pose, the yaw axis saturates at 100% of ticks, the robot
         drags 0.87 m and COLLAPSES to z=0.131 m against a 0.353 m gait
         height;
       - teleporting without stopping: `SetEntityPose` preserves VELOCITY,
         and a robot mid-gait is dropped 0.15 m while still travelling -- z
         from 0.337 m to 0.162 m in one second, with the cmd_vel stream
         still alive.
"""

import importlib.util
from pathlib import Path

from demo_simulation.scenarios import spawn_pose
from demo_simulation.sim_control_relay import _request

from launch import LaunchContext

import pytest

LAUNCH_DIR = Path(__file__).resolve().parents[1] / 'launch'
WORLDS_DIR = Path(__file__).resolve().parents[1] / 'worlds'


def _load(name: str):
    path = LAUNCH_DIR / name
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def sim_control():
    return _load('sim_control.launch.py')


def _context(world: str, **overrides) -> LaunchContext:
    """Context as the quadruped plant hands it to the included fragment."""
    context = LaunchContext()
    context.launch_configurations.update({
        'world': str(WORLDS_DIR / world),
        'robot_name': 'demo_robot',
        # Empty = "ask the table", same as the plant's ScenarioPose.
        'x': '', 'y': '', 'yaw': '',
        'height': '0.5',
    })
    context.launch_configurations.update(overrides)
    return context


# --- guard 1: the WorldControl no longer accepts reset ----------------------

def test_world_control_rejects_reset():
    with pytest.raises(ValueError):
        _request('reset')


@pytest.mark.parametrize('action, paused', [('play', False), ('pause', True)])
def test_play_and_pause_still_use_world_control(action, paused):
    request = _request(action)
    assert request.world_control.pause is paused
    # What must never happen: pausing/resuming by restarting the world.
    assert request.world_control.reset.all is False
    assert request.world_control.reset.model_only is False
    assert request.world_control.reset.time_only is False


# --- guard 2: the reset pose comes from the scenario table ------------------

def test_reset_uses_the_maze_scenario_pose(sim_control):
    world = 'quadruped_maze11.sdf'
    expected = spawn_pose(str(WORLDS_DIR / world))
    params = sim_control._reset_pose(_context(world))

    assert params['spawn_x'] == pytest.approx(expected['x'])
    assert params['spawn_y'] == pytest.approx(expected['y'])
    # The maze yaw is NOT zero: it is born facing the corridor. Resetting
    # with yaw 0 puts the robot facing the wall.
    assert params['spawn_yaw'] == pytest.approx(expected['yaw'])
    assert params['spawn_yaw'] != 0.0
    assert params['robot_name'] == 'demo_robot'


def test_explicit_arguments_override_the_table(sim_control):
    params = sim_control._reset_pose(
        _context('quadruped_maze11.sdf', x='2.5', y='-1.25', yaw='0.75'),
    )
    assert params['spawn_x'] == pytest.approx(2.5)
    assert params['spawn_y'] == pytest.approx(-1.25)
    assert params['spawn_yaw'] == pytest.approx(0.75)


def test_reset_height_matches_the_plant_height(sim_control):
    """The quadruped resets at its BIRTH height, not its gait height."""
    assert sim_control._reset_pose(
        _context('quadruped_maze11.sdf'))['spawn_z'] == pytest.approx(0.5)
    assert sim_control._reset_pose(
        _context('quadruped_maze11.sdf', height='0.43'),
    )['spawn_z'] == pytest.approx(0.43)


def test_plant_without_height_uses_the_diffdrive_default(sim_control):
    """
    `height` only exists on the quadruped plant.

    The diff-drive is born with `-z 0.1` hard-coded in `create`; resetting it
    to 0.5 m would be a gratuitous fall, and resetting a quadruped to 0.1 m
    drives its legs into the ground.
    """
    context = _context('quadruped_maze11.sdf')
    del context.launch_configurations['height']
    params = sim_control._reset_pose(context)
    assert params['spawn_z'] == pytest.approx(sim_control.DEFAULT_RESET_Z)
    assert params['spawn_z'] == pytest.approx(0.1)


# --- guard 3: teleporting without re-anchoring the gait ---------------------

RELAY = (
    Path(__file__).resolve().parents[1]
    / 'demo_simulation' / 'sim_control_relay.py'
).read_text(encoding='utf-8')


def test_robot_stops_before_teleport_and_resumes_afterward():
    """
    The order and content of this fix, not a matter of style.

    Stopping AFTER teleporting does not work: the teleport preserves
    velocity, and it is the robot mid-gait that falls. Re-anchoring BEFORE
    does not work either: StateTrotting::enter() reads the current pose,
    which is still the old one.
    """
    assert 'HOLD_SERVICE' in RELAY
    assert 'RESUME_SERVICE' in RELAY

    stop = RELAY.index('gait = self._hold_gait()')
    teleport = RELAY.index('pose_request.entity.name')
    resume = RELAY.index('self._resume_gait(gait)}')
    assert stop < teleport < resume


def test_reset_waits_for_the_robot_to_stop():
    """Without a wait, the hold is just a call: the robot is still moving."""
    assert 'GAIT_STOP_S' in RELAY
    assert 'time.sleep(GAIT_STOP_S)' in RELAY

    stop = RELAY.index('gait = self._hold_gait()')
    wait = RELAY.index('time.sleep(GAIT_STOP_S)')
    teleport = RELAY.index('pose_request.entity.name')
    assert stop < wait < teleport


def test_missing_gait_does_not_fail_the_reset():
    """
    On the differential-drive plant there is no gait, and that is a normal path.

    If re-anchoring became mandatory, the diffdrive reset would start
    failing -- and the cockpit would show an error on a reset that worked.
    """
    assert 'GAIT_TIMEOUT_S' in RELAY
    assert 'nothing to stop' in RELAY


def test_teleport_failure_does_not_leave_the_robot_in_fixed_stand():
    """
    Every exit path after the hold must resume.

    A robot left in FIXEDSTAND accepts no command at all, and nothing in the
    log says why the demo stopped responding.
    """
    resume_paths = RELAY.count('self._resume_gait(gait)')
    assert resume_paths == 3, (
        f'expected 3 resume paths (timeout, refusal, success), '
        f'found {resume_paths}'
    )


def test_reanchoring_failure_must_not_be_silent():
    """A reset that teleports and does not re-anchor leaves the robot dragging."""
    assert 'the gait was NOT re-anchored' in RELAY
