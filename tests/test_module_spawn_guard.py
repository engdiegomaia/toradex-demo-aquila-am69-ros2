"""Behavioral tests for scripts/module.sh's spawn-distance guard.

Recreating 'nav' restarts slam_toolbox, which anchors its map at whatever
pose the robot has right now. If /demo/sim/reset only runs AFTER that
restart, the map is anchored far from spawn and the next exploration round
dies in well under a minute (ComputePathToPose refusing everything with
error_code=208) -- reproduced in ML3.5 F5 R10 and R12 (see
docs/results/ml35-f5-exploration-r12.md). check_robot_near_spawn_before_nav_restart()
in scripts/module.sh is the guard against that; these tests exercise it
directly by stubbing _read_current_odom_xy, without touching a live
/demo/odom or the real module.
"""

from __future__ import annotations

from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_SH = REPO_ROOT / 'scripts' / 'module.sh'


def run_guard(force_spawn: str, odom_stub: str) -> subprocess.CompletedProcess:
    """Source module.sh, stub the odom read, and invoke the guard.

    `odom_stub` is the body of a bash function replacing
    `_read_current_odom_xy` -- e.g. `printf "5.0 0.1\\n"` for "far", or a
    bare `return 0` for "no odom available".
    """
    script = f'''
source "{MODULE_SH}"
_read_current_odom_xy() {{ {odom_stub}; }}
check_robot_near_spawn_before_nav_restart "{force_spawn}"
printf "GUARD_EXIT=%s\\n" "$?"
'''
    return subprocess.run(
        ['bash', '-c', script],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )


def test_position_near_spawn_is_allowed() -> None:
    result = run_guard('0', 'printf "0.0001 -0.0053\\n"')

    assert result.returncode == 0
    assert 'GUARD_EXIT=0' in result.stdout
    assert 'RECUSADO' not in result.stderr


def test_position_far_from_spawn_is_refused() -> None:
    result = run_guard('0', 'printf "5.0 0.1\\n"')

    assert result.returncode == 1
    assert 'RECUSADO' in result.stderr
    assert '5.00 m' in result.stderr


def test_boundary_at_exactly_one_meter_is_allowed() -> None:
    """The guard's own threshold is `distance > 1.0`, not `>=`."""
    result = run_guard('0', 'printf "1.0 0.0\\n"')

    assert result.returncode == 0
    assert 'GUARD_EXIT=0' in result.stdout


def test_force_spawn_bypasses_a_refusal() -> None:
    result = run_guard('1', 'printf "5.0 0.1\\n"')

    assert result.returncode == 0
    assert 'GUARD_EXIT=0' in result.stdout
    assert 'RECUSADO' not in result.stderr


def test_missing_odometry_is_allowed_through() -> None:
    """No /demo/odom yet (first bring-up): nothing to compare against."""
    result = run_guard('0', 'return 0')

    assert result.returncode == 0
    assert 'GUARD_EXIT=0' in result.stdout


def test_malformed_odometry_is_allowed_through_not_a_crash() -> None:
    """A value python3 can't parse must not abort the script under set -e."""
    result = run_guard('0', 'printf "not-a-number 0.0\\n"')

    assert result.returncode == 0
    assert 'GUARD_EXIT=0' in result.stdout


def test_refusal_message_prescribes_reset_before_up() -> None:
    result = run_guard('0', 'printf "5.0 0.1\\n"')

    assert 'ros2 service call /demo/sim/reset' in result.stderr
    reset_line = result.stderr.index('ros2 service call /demo/sim/reset')
    up_line = result.stderr.index('scripts/module.sh up', reset_line)
    assert reset_line < up_line, 'refusal must show reset BEFORE up, in that order'


def test_refusal_message_never_suggests_resetting_after_up() -> None:
    """--force-spawn must never be framed as 'I will reset afterward'.

    Resetting AFTER nav is recreated is exactly the sequence that produces
    the bug: slam_toolbox has already anchored on its first scan by the time
    a later reset's teleport happens, and the teleport does not undo an
    existing anchor.
    """
    result = run_guard('0', 'printf "5.0 0.1\\n"')
    text = result.stderr.lower()

    # The message explicitly disclaims the dangerous reading (a negation, not
    # a suggestion) -- that sentence is expected and is not what this test
    # guards against.
    assert 'nao significa' in text

    # The numbered action list is what an operator actually follows. Its
    # --force-spawn option must never pair "force" with "reset" as a
    # sequence to perform -- only the disclaimer above may combine them.
    force_spawn_option = text[text.index('2. force'):]
    assert 'reset' not in force_spawn_option


def test_force_spawn_is_independent_of_the_cmd_vel_force_flag() -> None:
    """scripts/module.sh:cmd_up must parse --force and --force-spawn separately."""
    script = MODULE_SH.read_text(encoding='utf-8')

    assert '--force-spawn) has_force_spawn=1' in script
    assert '--force) has_force=1' in script
    assert 'check_robot_near_spawn_before_nav_restart "${has_force_spawn}"' in script
