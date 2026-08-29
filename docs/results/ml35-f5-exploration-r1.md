# ML3.5 F5 — round 1: the livelock is gone, the blacklist is now too greedy

Date: 29/08/2026. Commit under test: `a554a25`. Baseline: the 28/08 smoke,
`ml35-f5-exploration-smoke.md`. Topology: **real HIL** — Gazebo Harmonic on the x86
host, Nav2 + SLAM + perception on the Aquila AM69, `ROS_DOMAIN_ID=69`,
`rmw_cyclonedds_cpp`, `ROBOT_TYPE=quadruped`, world `quadruped_maze11.sdf`.

Single variable against the baseline: **`_on_path_result` retires a frontier the
planner refuses**, plus the terminal condition that stops the retirement from turning
into a silent idle. Nothing else changed — no Nav2 parameter, no perception parameter,
no costmap change.

Raw samples: `ml35-f5-exploration-r1.csv` (1319 rows at 2 Hz).
Per-goal records: `ml35-f5-exploration-r1-goals.csv`.

> **Verdict: FAIL, and a different failure.** The livelock the fix targeted is gone —
> **459 s in `selecting` became 15 s, 463 selection cycles became 14, 466 path requests
> became 11, and 459 planner aborts became 2.** The map grew instead of freezing. The
> run then failed at a new wall: with 3 frontier clusters and 157 frontier cells still
> present, **zero** were permitted, because the 3 blacklisted points suppressed all of
> them. F5 stays open; round 2 is authorised with one variable.

---

## 1. Round 1 against the baseline

| metric | baseline (28/08) | round 1 (29/08) | |
| --- | --- | --- | --- |
| time in `selecting` | 459 s | **15 s** | 30× better |
| `selection_cycle` | 463 | **14** | |
| `path_requests` | 466 | **11** | |
| planner aborts in the log | 459 | **2** | |
| `map_known_cells` | frozen at 3564 | **1790 → 3698** | mapping resumed |
| goals dispatched | 5 | 4 | |
| goals reached | 4 | 3 | |
| how the run ended | budget exhausted at 600 s | **declared failed at 148 s** | |
| `escaped` | false | false | still FAIL |

The last row is the point of the terminal condition: the baseline burned 77% of the
budget looking busy. Round 1 said what was wrong and stopped.

### Preconditions, all met

Sim restarted (`/clock` from `sec: 0`), module stack restarted, fresh `/map` 88 wide,
robot at `(-0.002, 0.042, 0.347)`, `escaped=false`, explorer `idle`, Nav2
`is_active` → `success=True`, **TF 99.77%** over 45 s with RTF 0.967. Exploration was
started once, at `sim_s` 189, and no manual goal was sent at any point.

## 2. What the run did

| phase | `sim_s` | what happened |
| --- | --- | --- |
| startup | 177 – 190 | `idle` → `selecting`, first extraction: 3 clusters, 669 cells |
| productive | 190 – 327 | 4 goals dispatched, 3 reached, map 1790 → 3698 cells |
| wall | 328 – 337 | `nenhuma fronteira segura alcancavel`, 10 barren cycles → `failed` |

### Goals

| # | goal (x, y) | elapsed | outcome | message |
| --- | --- | --- | --- | --- |
| 0 | (-0.032, 1.142) | 12.1 s | ok | `fronteira alcancada; atualizando mapa` |
| 1 | (-0.183, 0.650) | 26.1 s | ok | `fronteira alcancada; atualizando mapa` |
| 2 | (-0.183, 0.650) | 5.1 s | ok | `fronteira alcancada; atualizando mapa` |
| 3 | (0.076, 0.563) | 90.0 s | **failed** | `meta de fronteira expirou` |

Goals 1 and 2 are still the same coordinate reached twice — the "reached frontier is
not retired" symptom survives, reduced from three repeats to two. It is **not** round
2's variable: it cost 5 s here, against the 148 s the run actually lost.

## 3. The new wall, located

At `sim_s` 328 the extraction reported `frontier_clusters=3`, `frontier_cells=157` —
real frontier still there — and `frontier_count=0` after filtering. Three blacklisted
points suppressed every cluster.

The bounded log window gives the two planner refusals verbatim, and they are the whole
story:

```text
[planner_server]: ExplorationGrid plugin failed to plan from (-0.16, 0.94)
                  to (0.07, 3.20): "Failed to create plan with tolerance of: 0.250000"
[planner_server]: ExplorationGrid plugin failed to plan from (-0.25, 0.80)
                  to (-2.93, 0.15): "Failed to create plan with tolerance of: 0.250000"
```

Both refused frontiers are **distant**: 2.3 m and 2.7 m from the robot. `ExplorationGrid`
runs with `allow_unknown: false` (locked by `test_maze_exploration_contract.py`), so a
frontier is refused while the *path to it* still crosses unmapped space — which is
precisely what exploring is about to fix. Retiring them permanently retired the far
half of the maze. The third point is goal 3, which timed out.

### The radius is not the variable, and the data says so

The plan predicted round 2 would be "a separate, smaller radius for planner
refusals". **The measurement refutes that.** The point written to the blacklist is the
frontier's own centroid, so any radius greater than zero kills the very cluster that
produced it. Shrinking `blacklist_radius_m` from 0.75 m would have recovered nothing
here.

What has to change is **permanence**, not reach. "Unreachable now" is not
"unreachable ever" when the planner refuses to cross unknown space.

## 4. Criteria from §2.3, scored

| criterion | measured | verdict |
| --- | --- | --- |
| `frontier_extract_ms` p95 < 100 ms | 93.8 ms (p50 68.1, max 116.3) | **pass** |
| no infinite repetition of the same selection | 14 cycles, 11 path requests | **pass** |
| map and coverage growing | 1790 → 3698 cells | **pass** |
| goals actually dispatched | 4, three reached | **pass** |
| no fall | max tilt 0.79°, `z` min 0.344 m | **pass** |
| no manual intervention | none after the single start call | **pass** |
| escape within 600 s | `escaped=false`, failed at 148 s | **FAIL** |

Bounded log scan over the run window: **0** TF extrapolation, **0** `worldToMap`,
**0** `invalid source`, **2** planner aborts. Distance travelled 2.97 m; `vx` work
ratio 5.96%.

TF was measured at 99.77% immediately **before** the run, not sampled during it. Say
that plainly rather than quoting the pre-run figure as an in-run result.

## 5. What this does not validate

- Gazebo on the x86 host, so nothing here says anything about a physical Go2
  (`CLAUDE.md` rule 7).
- No crossing was attempted: the robot never approached the exit, so
  `/demo/maze/escaped` was never exercised beyond reading `false`.
- `homing_exit` remains unexercised in a real run. It is proven only by the placed-robot
  perception gate in `ml35-f5-perception-aquila.md` §6.

## 6. Round 2, single variable

Make planner refusals **provisional**: hold them in their own list, and clear that list
whenever a goal is reached, because arriving is what changes the map that caused the
refusal. The hard blacklist — Nav2 refusing a goal, or a goal timing out — stays
permanent, since those record an execution failure that a new map does not contradict.

Not in round 2, deliberately: the repeated-frontier symptom (5 s), the 90 s
`goal_timeout_s` on an 8-cell frontier, and the `_map_seq` re-extraction guard — at 14
cycles it is no longer costing anything.
