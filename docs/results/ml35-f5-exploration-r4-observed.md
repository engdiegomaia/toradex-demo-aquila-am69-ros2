# ML3.5 F5 — observed round (29/08): the provisional recovery works; homing is what ends the run

Date: 29/08/2026. Topology: **real HIL** — Gazebo Harmonic on the x86 host, Nav2 + SLAM
+ perception on the Aquila AM69, `ROS_DOMAIN_ID=69`, `rmw_cyclonedds_cpp`,
`ROBOT_TYPE=quadruped`, world `quadruped_maze11.sdf` (11.60 × 11.60 m, 1.20 m corridors,
one connected component).

> **This is NOT the R4b protocol round.** The exploration run was already in flight when
> this session attached to the bench; it was not started by the recorder and the first
> ~290 s are not sampled. The preconditions were not established by this session and
> the run must not be counted toward the 3/3 cold-start campaign. Everything below is
> either (a) recorded from `sim_s` 660.7 onward or (b) measured directly against the
> terminal state, which is valid independently of how the run began.

Raw samples: `ml35-f5-exploration-r4-observed.csv` (840 rows at 2 Hz).
Per-goal records: `ml35-f5-exploration-r4-observed-goals.csv` (7 goals).

---

## 1. Verdict

| question | answer |
| --- | --- |
| Is the pending change deployed and running? | **Yes** — verified byte-identical host→module→container, and the live node publishes `provisional_recoveries`. |
| Was the provisional recovery exercised? | **Yes** — `provisional_recoveries` incremented 0 → 1 in the field. |
| Did the recovery behave to contract? | **Yes** — fired once, released only `_refused`/`_timed_out`, left `_blacklist` intact, did not livelock, terminated via `barren_cycles` as designed. |
| Did the round escape? | **No** — `/demo/maze/escaped` stayed false. `final_state = failed`, `final_message = "nenhuma fronteira segura alcancavel"`. |
| Is the recovery the remaining blocker? | **No.** It released the only candidates it could see, and all three were genuinely unreachable. |

**The recovery branch is exercised and passes its own contract. It is not what keeps F5
open.** The run ends on two distinct, independently measured defects described in §3
and §4.

## 2. Recorded facts

| metric | value |
| --- | --- |
| samples / sim span / wall span | 840 / 390.3 s / 419.5 s (RTF 0.931) |
| `path_m` over the recorded window | **3.71 m** |
| `map_known_cells` | 9674 → **9700** (+26 cells in 390 s) |
| `vx_work_ratio` | **9.76 %** (`vx_mean_abs` 0.005) |
| goals total / ok / failed | 7 / **1** / 6 |
| of which homing goals | **4, all failed** |
| `provisional_recoveries` | **1** |
| terminal counters | `blacklisted=0 refused=3 timed_out=0 frontier_count=0 frontier_clusters=4 barren_cycles=10` |
| `frontier_extract_ms` p50 / p95 / max | 112.3 / 225.7 / 297.9 ms |
| entered `failed` at | `sim_s` 735.173 |
| bounding box after `sim_s` 720 | x[-3.01, -2.78] y[1.28, 1.45] — **23 × 17 cm** |
| tilt max / z min | 1.77° / 0.3246 m — **no fall** |
| interventions | none during the run; all probes in §3–§4 ran after `failed` |

671 of 840 samples are in `failed`: the run died early and the recorder sampled a
stationary robot for the remaining ~5 minutes.

## 3. What ends the run: homing preempts healthy exploration and never succeeds

The per-goal record is the whole story:

| # | phase | sent `sim_s` | dur | outcome | message |
| --- | --- | --- | --- | --- | --- |
| 0 | exploration | 660.3 | 7.9 s | **ok** | fronteira alcancada |
| 1 | exploration | 669.0 | 28.0 s | failed | *preempted* — aproximando marcador |
| 2 | homing | 697.0 | **0.99 s** | failed | aproximando marcador |
| 3 | homing | 698.0 | **9.00 s** | failed | marcador perdido; retomando fronteiras |
| 4 | exploration | 709.0 | 7.0 s | failed | *preempted* — aproximando marcador |
| 5 | homing | 716.0 | **0.99 s** | failed | aproximando marcador |
| 6 | homing | 717.0 | **7.00 s** | failed | marcador perdido; retomando fronteiras |

Traced in the samples, the first cycle reads:

| `sim_s` | state | pose | `frontier_count` / `clusters` | marker |
| --- | --- | --- | --- | --- |
| 696.5 | `navigating` | (-1.93, 1.77) | **6 / 6** | false |
| 697.0 | `homing_exit` | (-1.99, 1.77) | 6 / 6 | **true** |
| 703.6 | `homing_exit` | (-2.27, 1.71) | 6 / 6 | true |
| 704.1 | `homing_exit` | (-2.29, 1.69) | 6 / 6 | **false** (occluded) |
| 707.3 | `selecting` | (-2.40, 1.59) | 6 / 6 | marcador perdido |
| 708.2 | `selecting` | (-2.40, 1.59) | **1 / 4** | — |

A healthy exploration goal with **six** frontier clusters is cancelled on marker
acquisition; homing travels 0.44 m in 10 s, the next wall occludes the marker, and the
explorer returns with the frontier set collapsed to **one**. The cycle repeats once and
the run dies.

**This reproduces round 2 exactly.** R2 recorded the marker first seen from (-3.15, 1.05)
with the marker at (-4.90, -2.60) — a **3.9 m line of sight through the maze opening** —
and both homing goals failing at **0.99 s and 8.0 s**. This round: **0.99/9.00 s** and
**0.99/7.00 s**. Same signature, two independent rounds.

**Homing has now been attempted 6 times across R2 and this round and has succeeded 0
times.** `estado-fases.md` already ranks this as open blocker #2 ("homing commits too
early"). It is now the *first* blocker, because the timeout-permanence blocker it was
ranked behind has since been fixed.

## 4. Why the fallback then finds nothing: three unreachable clusters and one that is filtered

Measured directly against the terminal state (robot at (-3.15, 1.29)), using the live
`/map` and the live `/global_costmap/costmap_raw`:

| # | goal | cells | costmap cost | reachable | dist to robot | suppressed by refusal (r=0.75) |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | (-1.93, 5.22) | 29 | **253 (inscribed → lethal)** | no | 4.11 m | yes |
| 1 | (-2.73, 8.32) | 28 | 243 | **no** (disconnected island) | 7.03 m | yes |
| 2 | **(-3.03, 1.52)** | 22 | 199 | **yes** | **0.26 m** | **no** |
| 3 | (-2.48, 8.32) | 9 | 237 | no | 7.05 m | yes |

Two independent confirmations that clusters 0, 1 and 3 are genuinely unreachable, not
mis-suppressed:

- `ComputePathToPose` aborts on them under **both** planners — `ExplorationGrid`
  (`allow_unknown: false`) *and* `GridBased` (`allow_unknown: true`, tolerance 0.5) —
  while short-range goals from the same pose succeed: (-3.0, 1.8) 12 poses, (-2.6, 2.3)
  17 poses, (-2.0, 3.0) 46 poses, (-2.5, 1.5) 10 poses. **The start side is healthy.**
- BFS over the global costmap from the robot: 888 passable cells, only **409 reachable**.
  The y≈8.3 clusters sit in a passable-but-disconnected island the lidar saw through an
  opening.

So the recovery in §1 did exactly what it was built to do and released precisely the
three frontiers that cannot be planned to. It cannot help, because:

> **Cluster 2 — the one genuinely reachable frontier — was never eligible for recovery
> at all.** The near filter (`min_frontier_distance_m = 0.35`) runs *before* the
> provisional filter, so `reachable` already excludes it at 0.26 m. The recovery
> restores `frontiers = reachable`, which is the post-near-filter list. A frontier
> inside the near limit is invisible to the recovery by construction.

A clearance sweep against the same terminal state confirms the near filter, not
clearance, is the constraint — raising `clearance_m` never adds a reachable goal:

| `clearance_m` = `standoff_m` | goals | non-lethal | **reachable** |
| --- | --- | --- | --- |
| 0.45 (current) | 4 | 3 | **1** |
| 0.50 / 0.55 / 0.60 | 4 | 3 | **1** |
| 0.65 | 2 | 2 | **1** |
| 0.70 | 1 | 1 | **1** |

Lowering `min_frontier_distance_m` below 0.26 m is **not** a safe remedy: Nav2's
`xy_goal_tolerance` is 0.25 m, and 0.35 was chosen in R4a precisely to stop the
instant-arrival loop that burned 565 selection cycles and 4508 path requests.

### Superseded in part by round 4b — read that first

`ml35-f5-exploration-r4b.md` found that NavFn refuses to plan from a start cell whose
global-costmap cost is `253` or above, and that a BFS like the one used above **relocates
the start** to a passable cell when the robot's own cell is blocked, which NavFn does not.

That does **not** overturn this section: here the short-range probes from the same pose
succeeded (12, 17, 46 and 10 poses), which proves this round's start pose *was* plannable
and that clusters 0, 1 and 3 were genuinely unreachable goals. It does mean the general
claim "refusals identify bad frontiers" is false — in round 4b the identical counter was
produced entirely by a bad *start*. Read `refused` as "the planner said no", never as
"this frontier is bad".

## 5. Recovery verdict against the handoff criteria

| criterion | result |
| --- | --- |
| recovery increments | **yes** (0 → 1) |
| dispatches a new goal | yes — the three released candidates were re-validated |
| produces material progress | **no** — all three are unreachable; 0.0 m gained |
| does not repeat before a successful arrival | **yes**, guard held |
| hard blacklist intact | **yes** (`blacklisted = 0` throughout) |
| does not block `barren_cycles` termination | **yes** — terminated at 10 barren cycles |

Mechanically the branch passes. It does not rescue the run, and on this evidence it was
never capable of doing so: the deadlock it clears was a *symptom*, and the frontier it
would need to release is excluded upstream of it.

## 6. Recorder gap found

`scripts/exploration_trial.py` does not sample `refused`, `timed_out`,
`near_frontiers_skipped` or `provisional_recoveries`; they exist on
`/demo/exploration/status` but not as CSV columns. Every suppression number in §2 and §4
had to be recovered from the live topic and from bounded log windows. This should be
fixed before R4b so the protocol fields the handoff asks for are actually recordable.

## 7. Limitations

HIL only. Gazebo plant on x86, Nav2/SLAM/perception on the AM69. Nothing here validates
a physical Go2, leg odometry, thermals or isolated module performance
(`CLAUDE.md` rules 5 and 7). The run was not started under the R4b protocol (see the
banner) and does not count toward acceptance.
