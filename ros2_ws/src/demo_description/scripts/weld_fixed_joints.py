#!/usr/bin/env python3
"""
Expand a xacro and strip <preserveFixedJoint>, so Gazebo welds fixed joints.

Runs on: x86 host only (it is part of the Gazebo-side pipeline).

    weld_fixed_joints.py demo_robot.urdf.xacro [key:=value ...] > robot.urdf
    xacro demo_robot.urdf.xacro | weld_fixed_joints.py > robot.urdf
    weld_fixed_joints.py --verbose demo_robot.urdf.xacro   # report count to stderr

On success this script is SILENT. launch's Command substitution aborts the entire
launch if a command writes anything to stderr, so a progress message here is
enough to take the simulation down with "executed command showed stderr output".
Pass --verbose only when running it by hand.

The first form exists because launch's `Command` substitution runs its string
through shlex.split rather than a shell, so a `|` in a Command would be passed to
xacro as a literal argument instead of creating a pipe. Given a file argument this
script runs xacro itself and filters the result, which keeps the whole thing a
single process for Command to call.

WHY THIS EXISTS
===============
The upstream TurtleBot 4 description tags 21 of its fixed joints with

    <gazebo reference="<joint>_joint">
      <preserveFixedJoint>true</preserveFixedJoint>
    </gazebo>

That tag tells Gazebo NOT to weld the joint's child into its parent's rigid body.
The result is a robot spawned as 13 independent physics bodies (shell, four tower
standoffs, sensor plate, lidar, camera, bumper, wheels, caster, base) held
together only by fixed-joint constraints.

The solver does not hold them rigidly. Under gravity and contact the parts settle
and drift, and the robot visibly comes apart — the tower leans, the plate and
lidar float off the standoffs. This is the "expanded parts" symptom.

It is NOT a geometry bug. The offsets and the per-visual rpy rotations in the
upstream model are correct; welding or not welding changes nothing about where the
parts are DRAWN, only whether physics is allowed to move them relative to each
other. That is why re-measuring mesh offsets never fixed it, and why swapping the
robot model did not either.

Upstream keeps the tag because preserved fixed joints are individually addressable
in Gazebo (useful for per-link contact sensors and for the bumper). This demo does
not need that, and needs a robot that stays in one piece.

WHAT IT DOES NOT TOUCH
======================
Only <gazebo> blocks whose sole content is <preserveFixedJoint> are removed.
Friction (<mu1>/<mu2>), materials, sensors and the system plugins are left alone —
those live in their own <gazebo reference="<link>"> blocks.

Verify after running:
    grep -c preserveFixedJoint   -> 0
    grep -c diff-drive-system    -> 1
"""

from __future__ import annotations

import re
import subprocess
import sys

# Matches a <gazebo reference="..."> block containing ONLY a preserveFixedJoint
# element (plus whitespace). Anything else in the block is left untouched, so a
# future upstream bump that adds a sibling element to one of these blocks keeps
# that element instead of silently losing it.
_PRESERVE_ONLY_BLOCK = re.compile(
    r'[ \t]*<gazebo\s+reference="[^"]*">\s*'
    r'<preserveFixedJoint>\s*true\s*</preserveFixedJoint>\s*'
    r'</gazebo>[ \t]*\n?',
    re.IGNORECASE,
)


def weld(urdf: str) -> tuple[str, int]:
    """Return (urdf without preserve-only blocks, number of blocks removed)."""
    welded, count = _PRESERVE_ONLY_BLOCK.subn('', urdf)
    return welded, count


def _read_source(argv: list[str]) -> str:
    """Expand a xacro if given a path, else read an already-expanded URDF."""
    if not argv:
        return sys.stdin.read()

    result = subprocess.run(
        ['xacro', *argv],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        # Surface xacro's own diagnostic verbatim; it names the offending file
        # and line, which a generic wrapper message would hide.
        print(result.stderr.rstrip(), file=sys.stderr)
        raise SystemExit(result.returncode)
    return result.stdout


def main() -> int:
    argv = [a for a in sys.argv[1:] if a != '--verbose']
    verbose = '--verbose' in sys.argv[1:]

    source = _read_source(argv)
    if not source.strip():
        print('weld_fixed_joints: empty URDF input', file=sys.stderr)
        return 1

    welded, removed = weld(source)

    # A leftover tag means a block carried extra siblings and was skipped by the
    # strict pattern above. Fail loudly: a partially welded robot still falls
    # apart, and doing it silently is the exact failure mode this guards against.
    leftover = welded.count('preserveFixedJoint')
    if leftover:
        print(
            f'weld_fixed_joints: {leftover} <preserveFixedJoint> tag(s) survived. '
            f'They are inside a <gazebo> block with other elements, which this '
            f'script will not delete blindly. Handle them explicitly.',
            file=sys.stderr,
        )
        return 1

    # Deliberately SILENT on success — no progress line, not even to stderr.
    #
    # launch's Command substitution treats ANY stderr output as a fatal error
    # ("executed command showed stderr output") and aborts the whole launch. A
    # friendly "welded N joints" message here is enough to take the simulation
    # down. Errors below still write to stderr, which is correct: those SHOULD
    # abort the launch.
    #
    # To see the count, run the script by hand with --verbose.
    if verbose:
        print(f'weld_fixed_joints: welded {removed} fixed joint(s)', file=sys.stderr)
    sys.stdout.write(welded)
    return 0


if __name__ == '__main__':
    sys.exit(main())
