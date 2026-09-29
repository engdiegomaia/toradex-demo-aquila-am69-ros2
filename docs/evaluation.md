# Evaluation and Measurement

This page covers how navigation and exploration performance is measured.
Recorders never publish commands. They listen, so they can run alongside the
real stack without changing what they measure.

All tools live in [`tools/`](../tools/README.md) and need a ROS 2 environment:
the `tools` container, or a native Jazzy installation with
`source scripts/env.sh`. Write raw output to `artifacts/`. That directory is
ignored by git and is mounted into the host `tools` container at
`/ws/artifacts`.

## Measurement rules

1. **Performance numbers only count on the Aquila AM69.** Never draw CPU,
   latency, thermal or FPS conclusions from the host or from QEMU.
2. **Repeat and interleave.** On identical configurations the mean speed has
   varied 2.4× between runs. Compare conditions as `A B A B A B`, not
   `A A A B B B`, and report the median and the range.
3. **One variable per comparison.** Every other parameter must be identical,
   and the Nav2 container must be recreated between conditions.
4. **Record the setup.** Each result states the link type, the commit, the
   parameter file and whether perception ran.

## Tools

| Tool | Kind | Measures |
| --- | --- | --- |
| `tools/evaluation/exploration_trial.py` | Recorder | One autonomous exploration run: pose, velocity, explorer state, goals, map growth, detections, escape flag. Writes `<out>.csv` and `<out>-goals.csv`. |
| `tools/evaluation/analyse_exploration.py` | Analysis | Derived figures from exploration CSVs (marker-range ratio, progress-checker effect) |
| `tools/evaluation/nav_trial.py` | Driver | Sends a fixed goal list through `navigate_to_pose` and records the result |
| `tools/evaluation/nav_campaign.py` | Driver | Interleaved A/B campaign over `nav_trial.py`, applying a command between legs |
| `tools/evaluation/summarize_trials.py` | Analysis | Median and range over trial CSVs |
| `tools/evaluation/gait_trial.py` / `gait_trial.sh` | Driver (open loop) | Gait envelope in the standalone simulator. Publishes `/demo/cmd_vel`, so never run it with Nav2. |
| `tools/diagnostics/scenario_check.py` | Probe | Topic rates, `/clock`, lidar and camera content |
| `tools/diagnostics/sensor_check.py` | Probe | Lidar, odometry and Nav2 rates |
| `tools/diagnostics/tf_lidar_probe.py` | Probe | TF age and availability for the lidar chain |

## Recording an exploration run

With the stack up (see [running-the-demo.md](running-the-demo.md)):

```bash
docker compose -f docker/compose.host.yml --profile tools up -d tools
docker compose -f docker/compose.host.yml exec -T tools \
  /usr/local/bin/entrypoint.sh python3 - artifacts/run1.csv --seconds 660 \
  < tools/evaluation/exploration_trial.py
```

Start the exploration from the cockpit once the recorder reports that it is
listening.

## A/B campaign on navigation parameters

`nav_campaign.py` alternates conditions. Before each leg it runs a command that
applies the condition. For parameter variants, that command recreates the
module's `nav` container with a different `NAV2_PARAMS`, which must be a path
inside the `nav` image:

```bash
python3 tools/evaluation/nav_campaign.py artifacts/campaign-align8 \
  --condition baseline='ssh torizon@<module> "cd /home/torizon/demo &&
      NAV2_PARAMS=__robot_default__ \
      docker compose -f compose.module.yml up -d --force-recreate nav"' \
  --condition align8='ssh torizon@<module> "cd /home/torizon/demo &&
      NAV2_PARAMS=/ws/src/demo_navigation/config/params-align8.yaml \
      docker compose -f compose.module.yml up -d --force-recreate nav"' \
  --reps 3 --seconds 420
```

The baseline also needs an explicit command. Without it, the baseline legs
that follow an `align8` leg would still run with `align8`. The script refuses
such a protocol. Pass `__robot_default__` rather than an empty value, because
an empty value produces an invalid launch argument.

Reset the simulation before each leg, for the reason explained in
[hil-deployment.md](hil-deployment.md#restarting-a-run).

## Publishing a result

Raw output stays in `artifacts/`. A result that supports a decision goes into
the documentation with a short report. The report states what was measured,
on which machine, the setup and the conclusion, and it keeps the CSV next to
the report. Summaries belong in [validation.md](validation.md). Decisions
belong in an [ADR](decisions/README.md).
