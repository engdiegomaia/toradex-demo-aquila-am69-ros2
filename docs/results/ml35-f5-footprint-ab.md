# ML3.5 F5 — footprint A/B: replacing `robot_radius` with the real trunk polygon

Date: 29/08/2026. Topology: **real HIL** — Gazebo Harmonic on the x86 host, Nav2 + SLAM
+ perception on the Aquila AM69, `ROS_DOMAIN_ID=69`, `rmw_cyclonedds_cpp`,
`ROBOT_TYPE=quadruped`, world `quadruped_maze11.sdf`.

This is a **controlled experiment, not a protocol round.** Nothing here counts toward the
3/3 cold-start campaign. The variant is **not promoted**: `nav2_params_go2.yaml` still
ships `robot_radius: 0.38` and the contract tests enforce that until this evidence is
accepted.

## 1. The two arms

| | arm A (control) | arm B (variant) |
| --- | --- | --- |
| params file | `nav2_params_go2.yaml` | `nav2_params_go2_footprint.yaml` |
| shape | `robot_radius: 0.38` — the **circumscribed** radius of the 0.70 × 0.31 m trunk | `footprint: [[0.37,0.18],[0.37,-0.18],[-0.37,-0.18],[-0.37,0.18]]` |
| applied to | `local_costmap` + `global_costmap` | same two, nothing else |
| `CostCritic.consider_footprint` | `false` | **`false`** — deliberately unchanged |

Single-variable is **enforced by test**, not asserted: `test_footprint_variant_changes_only_
the_two_costmap_shapes` loads both YAMLs, swaps the polygon back to `robot_radius: 0.38`
in the variant, and requires the two documents to compare equal. Six contract tests in
total (241 → 247).

`consider_footprint` was left `false` on purpose. `nav2_params_go2.yaml:537` records that
`true` alongside `robot_radius` dereferences an empty polygon and takes the whole
`nav2_container` down with SIGSEGV. Flipping it here would have been a second variable;
it is a separate follow-up now that a real polygon exists.

## 2. Arm B static validation — the collision-monitor coupling did not bite

The blocker the handoff warned about (`nav2_params.yaml`: footprint changes "MUST stay
false while the costmaps use `robot_radius`") did not materialise, because the variant
removes `robot_radius` rather than adding a polygon beside it.

| check | result |
| --- | --- |
| container survives bring-up | **Up, no SIGSEGV** |
| `planner_server` / `controller_server` / `bt_navigator` | all `active [3]` |
| "Inconsistent configuration in collision checking" warning | **absent** |
| footprint actually published | **yes**, on `/local_costmap/published_footprint` and `/global_costmap/published_footprint` — 4 points, extent 0.38 × 0.76 m in the map frame (the 0.36 × 0.74 body rectangle rotated by the ≈90° spawn yaw) |
| `collision_monitor` | node present, lifecycle `active [3]`, no error lines (`/collision_monitor_state` is event-driven and silent while stationary) |
| spawn own-cell cost | **96** |
| static corridor planning | **4/4 SUCCEEDED** — 24, 37, 34, 28 poses |

## 3. Cost cross-section during walking — the measurement that decides it

The robot's **own** global-costmap cell, sampled at 2 Hz while actually walking under
exploration. Arm A ran 150 s; arm B ran 150 s. Raw traces:
`costtrace-armA.csv`, `costtrace-armB.csv`.

| metric | arm A (`robot_radius` 0.38) | arm B (polygon) |
| --- | --- | --- |
| samples | 297 | 297 |
| distinct poses | 86 | **50** |
| min | 141 | **73** |
| median | 168 | **135** |
| p90 | 237 | **165** |
| p95 | 243 | **165** |
| **max** | **243** | **165** |
| % ≥ 243 | **7.1 %** | **0.0 %** |
| % ≥ 253 (fatal to NavFn) | 0.0 % | 0.0 % |
| **headroom from max to 253** | **10** | **88** |

Neither arm ever recorded a 253 sample, so this A/B does **not** show arm B preventing an
event that arm A produced. What it shows is the margin. Arm A spends 7.1 % of its walking
time one inflation step (243) below the value at which NavFn refuses every goal; arm B
never gets within 88 cost units of it. R4b's terminal 253 is a rare state, so the right
question is how close the distribution sits to the cliff — and arm B moves the whole
distribution away from it.

**Caveat, recorded rather than smoothed over:** arm B covered 50 distinct poses against
arm A's 86 over the same 297 samples, so arm B sampled less terrain and stood still more.
The comparison is not pose-matched, and a lower maximum could in part reflect visiting
fewer tight places. The direction is consistent across every statistic (min, median, p90,
p95, max all drop), which is hard to explain by coverage alone, but this is one run per
arm and should not be read as more than that.

## 4. Behaviour of the arm B run — it survived to the deadline instead of dying

The arm B exploration that produced the trace above kept running, and an incremental
status logger was attached at `elapsed_s` 472 (raw: `ml35-f5-footprint-ab-live.csv`,
214 rows, flushed per row). The run ended at **`elapsed_s` 600.0** with
`final_message = "prazo total de exploracao excedido"` — it hit `total_timeout_s`,
the explorer's own 600 s budget.

**That is a new failure mode.** Every previous round died of the planner: R4a deadlocked,
the 29/08 observed round collapsed from six frontier clusters to one, R4b went barren at
116 s. This run was still selecting frontiers and dispatching goals when the clock ran
out.

| `elapsed_s` | state | `fc` | `refused` | `timed_out` | event |
| --- | --- | --- | --- | --- | --- |
| 472.0 | `navigating` | 5 | **0** | 0 | logger attached; healthy |
| 503.0 | `selecting` | 5 | 0 | 1 | goal expired at `goal_timeout_s` = 90 s |
| 505.0 | `navigating` | 3 | 2 | 1 | new goal; two candidates refused in this cycle |
| 595.0 | `selecting` | 3 | 2 | **2** | second goal expired — another 90 s |
| 597.1 | `navigating` | 2 | 2 | 2 | new goal dispatched |
| 600.0 | **`failed`** | 2 | 2 | 2 | **total exploration deadline exceeded** |

Terminal counters: `blacklisted=0 refused=2 timed_out=2 near_skipped=0
provisional_recoveries=1 barren_cycles=0 selection_cycle=15 path_requests=45`.

Two observations that set the next priority:

- **`refused` reached only 2 in 600 s, and `barren_cycles` never left 0.** Under arm A,
  R4b booked 4 refusals and 10 barren cycles inside 116 s. The refusal storm did not
  happen. The start pose stayed plannable for the whole run, which is exactly what the
  polygon was meant to buy.
- **The budget went to goal timeouts.** The two visible expiries cost **180 s — 30 % of
  the entire 600 s budget** — in the last stretch alone, on top of five homing excursions
  earlier. Arm B did not fail to plan; it failed to finish in time.

`/demo/maze/escaped` was `false` throughout.

## 5. The homing measurement landed — and it exposed why homing never works

The same run finally collected the number the handoff has been waiting for since R4b:

```text
homing_entries            5
homing_entry_distance_m   3.06     (latched at the most recent transition)
marker_distance_m         12.36    (at the end; marker not visible)
```

The last of five homing entries was committed at **3.06 m**, consistent with round 2's
3.9 m glimpse through the maze opening. All five entries were over before the logger
attached at 472 s, so only the latched last distance survives; the other four are lost.

**Homing is now 0 for 11** across R2 (2), the observed round (4) and this run (5).

### Root cause, read directly from the code

`_send_homing_step` does not navigate to the exit. It walks toward it in
`homing_step_m` = 0.5 m hops, and after **every** hop `_tick` re-tests the marker against
`marker_stale_s` = 2.0 s:

```python
elif self._state == 'homing_exit' and not self._pending and self._goal_handle is None:
    if not marker_fresh:
        self._state = 'selecting'
        self._message = 'marcador perdido; retomando fronteiras'
    else:
        self._send_homing_step()
```

`self._exit_pose_map` is a **latched map coordinate**. Once it is known, Nav2 can drive to
it like any other goal — line of sight is needed to *learn* the exit's position, not to
navigate to it. But this branch discards the whole approach the moment the marker is not
visible in the current instant.

Walking a maze corridor breaks line of sight by construction: the robot completes a 0.5 m
hop, the next wall occludes the marker, `marker_fresh` is false, and homing abandons
itself. This is exactly the recorded signature — the observed round's per-goal CSV shows
homing goals failing at **0.99 s then 8–9 s** with "marcador perdido; retomando
fronteiras", and R2 shows 0.99 s / 8.0 s. Eleven attempts, one mechanism.

**This reorders the blockers.** The distance gate in the handoff (§6) prevents *wasting*
time on a far marker. It would not have produced an escape here, because the five entries
at ≈3 m were close enough to convert and still did not. What blocks `escaped = true` is
abandonment on occlusion, not premature entry.

One qualification: under arm B, a homing excursion is no longer *fatal*. In the observed
round a single excursion collapsed the frontier set from six clusters to one and the run
died. Here five entries happened and the explorer returned each time with five clusters
and `refused = 0`. Homing now wastes time rather than ending the run — which is why the
600 s budget, not the planner, is what stopped this attempt.

## 6. Verdict and what is still missing

Arm B is better on every measured axis and worse on none: no crash, no collision-monitor
inconsistency, footprint published correctly, static planning 4/4, a cost distribution
that never approaches the fatal threshold, 600 s of exploration with 2 refusals and zero
barren cycles, and a failure mode that moved from "the planner refuses everything" to
"the robot ran out of time".

Still outstanding before promotion into `nav2_params_go2.yaml`:

- a **clean smoke from t = 0** under arm B with the full recorder attached — this run was
  already in flight, so goals reached could not be counted from the status history and
  four of the five homing distances were lost;
- explicit confirmation that Nav2 **replans after lateral displacement**;
- the pose-coverage caveat in §3 (50 vs 86 distinct poses) addressed by a second trace, or
  accepted as a known limit.

And the next defect to fix is now identified with a code-level root cause and 11 field
observations: **homing must navigate to the latched exit pose instead of requiring
continuous visual contact.**

## 7. Limitations

HIL only. Gazebo plant on x86, Nav2/SLAM/perception on the AM69. Nothing here validates a
physical Go2, leg odometry, thermals or isolated module performance (`CLAUDE.md` rules 5
and 7). One experiment per arm; no cold-start campaign attempted.
