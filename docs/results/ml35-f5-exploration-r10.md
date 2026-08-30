# ML3.5 F5 — round 10: died before the robot ever moved; inconclusive on the wall-clearance fix

Date: 29/08/2026, real HIL. Single variable against R9: `frontier_wall_clearance_m`
lowered from the previous hardcoded 0.45 m to 0.38 m in `maze_explorer.py` (see
`docs/results/ml35-f5-exploration-r9.md`, "Follow-up"). `nav`/`perception` rebuilt
natively on the module and force-recreated (fresh `slam_toolbox` map/pose-graph as a
side effect), `/demo/sim/reset` called, parameter read back live as `0.38` before
triggering. Recorder started before the single `/demo/exploration/start` call this time.

> **Verdict: FAIL, and INCONCLUSIVE for the wall-clearance fix.** The run reached its
> terminal `failed` state at **`wall_s` 28.5 s** — not 279 s as an earlier mid-run status
> poll misread (`elapsed_s` in the explorer's own status message runs on a different
> counter than the recorder's wall clock; the recorder's own `wall_s`/`sim_s` columns are
> authoritative and are what this report uses throughout). **The robot never moved**:
> `path_m` stayed at 0.06 m (noise) for the entire 660 s recording, `cmd_vx`/`cmd_wz`
> never left `0.0`, and `map_known_cells` never grew past its first-frame value of 1216
> (9.13 %). No fall (`tilt_deg` max 0.22°, unremarkable since the robot was stationary
> throughout). The goals CSV has a header and zero rows: no navigation goal was ever
> actually dispatched.

## What happened, from the recorder's own state-change log

| `wall_s` | `state` | `frontier_count` | `frontier_clusters` | `path_requests` | `refused` | `provisional_recoveries` | `barren_cycles` | message |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 15.5 | waiting_map | 0 | 0 | 0 | 0 | 0 | 0 | aguardando mapa, TF e Nav2 |
| 17.0 | selecting | 1 | 1 | 1 | 0 | 0 | 0 | |
| 18.0 | selecting | 1 | 1 | 2 | 0 | 1 | 0 | released provisional frontier suppressions |
| 19.0 | selecting | 0 | 1 | 2 | 1 | 1 | 1 | nenhuma fronteira segura alcancavel |
| 20.0–27.5 | selecting | 0 | 1 | 2 | 1 | 1 | 2→9 | nenhuma fronteira segura alcancavel (barren ticking up) |
| 28.5 | **failed** | 0 | 1 | 2 | 1 | 1 | 10 | nenhuma fronteira segura alcancavel |

Reading it: exactly **one** frontier cluster ever existed (the first one found near
spawn, 413 cells). Its candidate goal was checked twice (`path_requests` reaches 2), the
provisional-recovery mechanism released it once at 18.0 s, and it was refused again
immediately after — at which point there was no second cluster to fall back to, and the
barren counter ran to its limit (10) in nine seconds flat. The whole event is over by
28.5 s; the remaining ~630 s of the recording is the robot sitting at spawn while the
recorder pads out its fixed budget, exactly as designed for a terminal state.

## Metrics

| metric | value |
| --- | --- |
| samples / sim span / wall span | 1320 / 642.5 s / 659.5 s |
| `escaped` | **false**, at every sample |
| terminal `wall_s` (state → `failed`) | **28.5 s** |
| `path_m` (final) | 0.06 m — the robot did not move |
| `map_known_cells` / `map_known_pct` | 1216 / 9.13 % — unchanged for the entire run |
| goals dispatched | **0** (goals CSV: header only, no rows) |
| terminal counters | `refused=1 timed_out=0 blacklisted=0 provisional_recoveries=1 barren_cycles=10` |
| `frontier_clusters` (ever seen) | 1 |
| tilt max / z range | 0.22° / 0.3566–0.3569 m — no fall (robot stationary) |

## Reading it honestly

**Corrected after this file's first draft.** The first version of this section blamed
Nav2's `CostCritic`/MPPI for the refusal. That was wrong on the mechanics: R10 (and every
`_refused` entry `maze_explorer.py` records) happens at the **`ComputePathToPose`
stage — the global planner, before `NavigateToPose`/MPPI ever runs.** MPPI never sees a
goal that the global planner has already refused. The correction below replaces that
claim with an offline A/B diagnostic run directly against R10's own frozen final map
(the robot never moved after `failed`, so the live `/map` was still byte-identical to
the state that killed the run), using the exact `planner_id='ExplorationGrid'` and
`use_start=False` `maze_explorer.py` itself sends — not a guess, a real
`ComputePathToPose` action call:

| clearance_m | candidate | costmap cost (0–100) | `ComputePathToPose` status | path returned |
| --- | --- | --- | --- | --- |
| 0.38 | (-1.731, 8.175) | 52 | **6 (ABORTED)** | 0 poses |
| 0.45 | (-1.731, 8.175) | 52 | **6 (ABORTED)** | 0 poses |

Re-queried directly for the actual planner `error_code` (`nav2_msgs/action/ComputePathToPose` carries one): **`208` (`NO_VALID_PATH`)**, `error_msg` empty. Not `205`/`206` (`START_OCCUPIED`/`GOAL_OCCUPIED`), not `202`/`203`/`204` (TF or map-bounds). The planner genuinely searched and found no path in the known-cell graph — the exact category `ExplorationGrid`'s `allow_unknown: false` predicts for a map with an unresolved connectivity gap, not a goal-placement problem.

**Both clearance values select the identical candidate and get the identical refusal.**
`frontier_wall_clearance_m` made no difference to this run's outcome — this is exactly
the "ambos recusam" branch of the decision table this diagnostic was designed against:
a startup/single-candidate fragility, not a clearance effect. Two more checks against
the same snapshot confirm it is not a broken planner or a filter artefact:

- A `ComputePathToPose` call to a **different**, closer goal `(-0.05, 2.0)` on the same
  map **succeeded** (status 4, a 37-pose, 2.04 m path) — the planner and costmap are
  functioning normally in general; the failure is specific to reaching this frontier
  region, not a global planner outage.
- Re-running `extract_frontiers`'s raw BFS clustering step directly (before the
  clearance/standoff candidate filter) on the same map found **exactly one** raw cluster
  of 413 cells, centroid `(-3.04, 7.95)` — R10 did not lose other clusters to filtering;
  there genuinely was only one connected frontier region in this map.

So the honest state of this round is: **the sole frontier candidate sat in a region the
global planner could not reach from the robot's position, for a reason this diagnostic
does not yet pin down** (most likely a coverage/connectivity gap in the still-very-thin
map — only 9.13 % known at the moment of death — rather than anything
`frontier_wall_clearance_m` controls). An attempt to locate the exact break point along
the corridor by probing guessed intermediate coordinates was inconclusive (the guessed
points were not verified to lie on the real corridor centreline) and is not reported
here as a finding.
- **No fall, but there was no motion to fall from.** This run says nothing about mobility
  or the fiducial detector.

## What this round does and does not close

- Does **not** close Gate B.
- **Rules out** `frontier_wall_clearance_m` as the cause of R10's death — confirmed by a
  real, matched A/B `ComputePathToPose` call against the frozen map, not inference. No
  reason from this data to revert 0.38 back to 0.45.
- Does **not** explain why the sole reachable-looking frontier was actually unreachable —
  that remains open, and is a map-coverage/connectivity question, separate from wall
  clearance.
- Does **confirm** the rebuilt `nav` image, the redeployed `frontier_wall_clearance_m=0.38`
  parameter, and the recorder-first launch order all worked mechanically as intended.

## Recommended next step

Do not touch `frontier_wall_clearance_m` again based on this round — the diagnostic
above already answers that question. The real open item is structural: a single
frontier cluster whose only candidate point is unreachable currently kills the whole
run, regardless of what caused that one candidate to be unreachable. Per-cluster
multiple candidate points would **not** have saved this specific run (there was only one
cluster, and the whole region was equally unreachable), but does still address the
distinct pattern seen in R9 (the same single coordinate re-selected and re-refused twice,
goals 7–8). Fixing R10's failure mode specifically needs either better map coverage
before the first goal is attempted, or telemetry that can actually localise a
`ComputePathToPose` refusal (candidate coordinates, costmap cost, and ideally the
planner's own error message) instead of requiring an offline reconstruction like this
one after the fact.

## Limitations

HIL only; nothing here validates a physical Go2, leg odometry, thermals, or isolated
module performance (`CLAUDE.md` rules 5 and 7). One round, and a very short one at that.
