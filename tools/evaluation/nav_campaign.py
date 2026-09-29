#!/usr/bin/env python3
"""
Runs an INTERLEAVED navigation A/B campaign and summarizes by median and range.

    python3 tools/evaluation/nav_campaign.py artifacts/campaign-align8 \
        --condition baseline='<command that reapplies the baseline>' \
        --condition align8='<command that applies the condition>' \
        --reps 3 --seconds 420

## Why this script exists

`nav_trial.py` measures ONE run. `summarize_trials.py` summarizes replicates
that already exist. What was missing was the middle: who decides the ORDER of
the runs and what happens between them. Without that the comparison stays
indefensible, and this project already paid for it.

The number that matters: under IDENTICAL configuration the average speed
varied **2.4×** between runs (`docs/results/ml35-f5-clock-fanout.md`). One run
per condition cannot tell effect from noise — not even when the effect is
real. Two protocol consequences follow, and both are implemented here:

- **Interleave, don't block.** The order is `A B A B A B`, never `A A A B B B`.
  Any drift over the course of the campaign — thermal, host cache, container
  memory, someone else using the network — falls equally on both conditions
  instead of turning into a difference between them.
- **Same initial state on every leg.** Between legs the robot returns to its
  birth pose and the costmap is cleared. Without that, leg 2 starts from
  wherever leg 1 stopped, and "condition B" comes to mean "condition B
  starting from a different place".

## The reset between legs has only been possible since 26/08/2026

`/demo/sim/reset` used to DELETE the robot from the world (`reset.all` returns
the world to the original SDF, which does not contain a model inserted by
`create`). A campaign that called reset between legs would measure, from leg 2
onward, a world with no robot — with the clock running and the orphaned
sensors still publishing, i.e. with nothing flagging it. See
`docs/results/cockpit-reset-nao-destrutivo.md`.

Today the reset teleports and the plant survives, and that is what makes this
loop honest. If you revert that fix, THIS script starts lying.

## How a condition is applied, and why it isn't an embedded `ros2 param set`

Each condition carries a SHELL COMMAND, supplied by whoever runs the campaign.
Deliberate: the MPPI critic weights are read in the controller's
`on_configure`, so a `ros2 param set` on
`FollowPath.PathAlignCritic.cost_weight` is accepted, reads the new value
back, and **may not change behavior**. A script that applied `param set` on
its own would produce an entire campaign comparing the condition against
itself, with perfect-looking evidence.

So the decision stays with the operator, and the reliable form is to recreate
the stack with the condition's YAML. Example, on the module:

    --condition align8='ssh torizon@<module> "cd /home/torizon/demo &&
        NAV2_PARAMS=/ws/src/demo_navigation/config/params-align8.yaml \
        docker compose -f compose.module.yml up -d --force-recreate nav"'

In a comparison, even `baseline` needs a command. Without one, after the first
`align8` leg the legs named baseline would keep using align8. The script
refuses that protocol instead of producing a false A/B.

The baseline must pass `NAV2_PARAMS=__robot_default__` to Compose. An empty
value produces the invalid argument `params_override:=` before the launch can
pick the robot's default.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shlex
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent

# Reset services. `sim/reset` puts the robot back at its birth pose without
# deleting anything; `nav/reset` clears the accumulated costmap. There are two
# because they live on different machines in hil mode, and the separate
# granularity is what lets you reposition the robot without taking Nav2 down.
SIM_RESET = '/demo/sim/reset'
NAV_RESET = '/demo/nav/reset'

# The three managed nodes whose lifecycle state decides whether a goal is
# accepted or rejected with "Action server is inactive". Same set that
# `module.sh verify` checks for bt_navigator (see that script for the
# 26/08/2026 incident that motivated the check).
READINESS_NODES = ('/bt_navigator', '/controller_server', '/planner_server')

# After teleporting, the quadruped settles to gait height on its own (0.5 m at
# birth -> ~0.35 m at gait, measured). Starting to measure during the fall
# would contaminate the leg's first sample window.
SETTLE_S = 6.0


def _run(command: list[str], timeout: float) -> tuple[int, str]:
    try:
        done = subprocess.run(
            command, capture_output=True, text=True, timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return 124, f'timed out after {timeout:.0f} s'
    return done.returncode, (done.stdout or '') + (done.stderr or '')


def call_trigger(service: str, timeout: float = 30.0) -> tuple[bool, str]:
    """Calls a std_srvs/Trigger and returns (accepted, raw text)."""
    code, output = _run(
        ['ros2', 'service', 'call', service, 'std_srvs/srv/Trigger', '{}'],
        timeout,
    )
    # `ros2 service call` exits 0 even when the service responded
    # success=False, so the exit code alone is not enough: the response
    # field is what decides.
    accepted = code == 0 and 'success=True' in output
    return accepted, output.strip()


def lifecycle_state(node: str, timeout: float = 10.0) -> str:
    """First word of `ros2 lifecycle get <node>` (`'active'`, `'inactive'`,
    `'unconfigured'`...), or `''` if the call fails or does not respond."""
    code, output = _run(['ros2', 'lifecycle', 'get', node], timeout)
    if code != 0:
        return ''
    line = output.strip().splitlines()[0] if output.strip() else ''
    return line.split()[0] if line else ''


def wait_for_managed_nodes(
    nodes: tuple[str, ...] = READINESS_NODES,
    *,
    timeout: float = 90.0,
    poll_interval: float = 2.0,
    verbose: bool = True,
) -> tuple[bool, float]:
    """
    Waits until every `node` in `nodes` responds `active`, or until `timeout`.

    Returns (ready, elapsed_seconds). The elapsed time is returned even when
    ready=True: slow bring-up is a signal worth recording, not only a failure
    reason. This exists because a topic existing does not mean the service
    works — after an `up`, the three managed nodes can be standing with the
    lifecycle manager aborted, and every goal gets silently rejected until
    someone notices (see `module.sh verify`, step 4/4).
    """
    start = time.monotonic()
    deadline = start + timeout
    pending = set(nodes)
    while True:
        for node in list(pending):
            if lifecycle_state(node) == 'active':
                pending.discard(node)
        elapsed = time.monotonic() - start
        if not pending:
            return True, elapsed
        if time.monotonic() >= deadline:
            if verbose:
                print(f'  managed nodes that did not activate: {", ".join(sorted(pending))}')
            return False, elapsed
        time.sleep(poll_interval)


def _append_manifest(out_dir: Path, **record) -> None:
    """Appends a line to the campaign manifest; never overwrites a previous
    leg, even when the campaign aborts on the next leg."""
    with (out_dir / 'manifesto.jsonl').open('a', encoding='utf-8') as handle:
        handle.write(json.dumps(record, ensure_ascii=False) + '\n')


def reset_between_legs(*, skip_nav: bool, verbose: bool = True) -> bool:
    """
    Returns robot and costmap to the initial state. False if something refused.

    A refusal is an ABORT, not a warning: a leg that starts from a different
    state than the others is not a replicate, and including it in the median
    ruins exactly the number the campaign exists to produce.
    """
    ok, detail = call_trigger(SIM_RESET)
    if verbose:
        print(f'  simulator reset: {"ok" if ok else "REFUSED"}')
    if not ok:
        print(f'  {detail}', file=sys.stderr)
        return False

    if not skip_nav:
        ok, detail = call_trigger(NAV_RESET, timeout=60.0)
        if verbose:
            print(f'  navigation reset: {"ok" if ok else "REFUSED"}')
        if not ok:
            print(f'  {detail}', file=sys.stderr)
            return False

    time.sleep(SETTLE_S)
    return True


def parse_condition(raw: str) -> tuple[str, str | None]:
    """`name` or `name=shell command`."""
    name, sep, command = raw.partition('=')
    name = name.strip()
    if not name:
        raise argparse.ArgumentTypeError(f'condition without a name: {raw!r}')
    return name, (command if sep else None)


def validate_conditions(conditions: list[tuple[str, str | None]]) -> None:
    """Requires an explicit reapplication on each side of a comparison."""
    if len(conditions) < 2:
        return
    missing = [name for name, command in conditions if not command]
    if missing:
        raise ValueError(
            'a comparison requires a command for every condition; without '
            'reapplying '
            + ', '.join(missing)
            + ', the leg inherits the previous configuration')


def leg_order(conditions: list[tuple[str, str | None]], reps: int):
    """
    The interleaved order, flattened: (rep, name, command).

    Exported so it can be tested without a simulator: the order IS the
    protocol, and a swapped loop turns the campaign into a blocked one
    without anything flagging it.
    """
    for rep in range(1, reps + 1):
        for name, command in conditions:
            yield rep, name, command


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('out_dir', help='directory for the CSVs and the summary')
    parser.add_argument('--condition', action='append', default=[],
                        metavar='NAME[=COMMAND]',
                        help='repeat for each condition; without =COMMAND it '
                             'applies nothing (reference)')
    parser.add_argument('--reps', type=int, default=3,
                        help='replicates per condition (protocol asks n >= 3)')
    parser.add_argument('--seconds', type=float, default=420.0,
                        help='duration of each leg, passed to nav_trial')
    parser.add_argument('--goals', default='maze11')
    parser.add_argument('--goal-timeout', type=float, default=90.0,
                        help='per-goal deadline forwarded to nav_trial')
    parser.add_argument('--skip-nav-reset', action='store_true',
                        help='do not call /demo/nav/reset between legs')
    parser.add_argument('--apply-timeout', type=float, default=300.0,
                        help='ceiling for a condition\'s command')
    parser.add_argument('--readiness-timeout', type=float, default=90.0,
                        help='ceiling for bt_navigator/controller_server/'
                             'planner_server to become active after '
                             'applying a condition')
    parser.add_argument('--dry-run', action='store_true',
                        help='print the leg order and exit')
    args = parser.parse_args(argv)

    if not args.condition:
        print('no condition: use --condition at least once',
              file=sys.stderr)
        return 2
    if args.reps < 3 and not args.dry_run:
        # Warning, not error: n < 3 is legitimate for debugging the loop
        # itself. What is not legitimate is DECIDING with it, and the
        # summary carries the n.
        print(f'WARNING: reps={args.reps} is below the protocol (n >= 3); '
              'the 2.4x spread under identical configuration still holds.')

    conditions = [parse_condition(raw) for raw in args.condition]
    try:
        validate_conditions(conditions)
    except ValueError as error:
        parser.error(str(error))
    legs = list(leg_order(conditions, args.reps))

    print(f'interleaved campaign: {len(conditions)} conditions x {args.reps} '
          f'replicates = {len(legs)} legs of {args.seconds:.0f} s')
    print(f'minimum track time: {len(legs) * args.seconds / 60:.0f} min '
          f'(not counting bring-up and reset)')
    print('order: ' + ' '.join(name for _, name, _ in legs))
    if args.dry_run:
        return 0

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    produced: list[tuple[str, Path]] = []

    for index, (rep, name, command) in enumerate(legs, start=1):
        csv_path = out_dir / f'{name}-r{rep}.csv'
        print(f'\n--- leg {index}/{len(legs)}: {name}, replicate {rep} ---')

        if command:
            print(f'  applying condition: {command}')
            code, output = _run(shlex.split(command), args.apply_timeout)
            if code != 0:
                print(f'  the condition FAILED (code {code}); aborting the '
                      f'campaign to avoid recording an invalid replicate\n{output}',
                      file=sys.stderr)
                return 1

        # `docker compose up -d --force-recreate` (the typical command for a
        # condition) returns before bt_navigator/controller_server/
        # planner_server become active. A leg that starts collecting in that
        # gap measures "Action server is inactive" instead of the condition's
        # actual behavior.
        ready, bringup_s = wait_for_managed_nodes(timeout=args.readiness_timeout)
        print(f'  managed node readiness: {"ok" if ready else "TIMEOUT"} '
              f'in {bringup_s:.1f}s')
        _append_manifest(
            out_dir, rep=rep, condition=name, command_applied=bool(command),
            readiness_ready=ready, readiness_seconds=round(bringup_s, 1),
        )
        if not ready:
            print(f'  managed nodes did not become active within '
                  f'{args.readiness_timeout:.0f}s; aborting the campaign',
                  file=sys.stderr)
            return 1

        if not reset_between_legs(skip_nav=args.skip_nav_reset):
            print('  reset refused; aborting the campaign', file=sys.stderr)
            return 1

        code, output = _run(
            [sys.executable, str(HERE / 'nav_trial.py'), str(csv_path),
             '--seconds', str(args.seconds), f'--goals={args.goals}',
             '--goal-timeout', str(args.goal_timeout)],
            timeout=args.seconds + 300.0,
        )
        print(output.strip()[-2000:])
        if not csv_path.exists():
            print(f'  the leg did not write {csv_path}; aborting',
                  file=sys.stderr)
            return 1
        produced.append((name, csv_path))

    print('\n===== summary by condition (median and observed range) =====')
    code, output = _run(
        [sys.executable, str(HERE / 'summarize_trials.py')]
        + [f'{name}={path}' for name, path in produced],
        timeout=120.0,
    )
    print(output.strip())

    summary = out_dir / 'resumo.txt'
    summary.write_text(output, encoding='utf-8')
    print(f'\nsummary written to {summary}')
    return code


if __name__ == '__main__':
    sys.exit(main())
