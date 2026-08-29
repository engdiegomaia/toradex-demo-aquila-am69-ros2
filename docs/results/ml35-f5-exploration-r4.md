# ML3.5 F5 — round 4: INCONCLUSIVE. The change never ran, and a different defect ate the run

Date: 29/08/2026. Baseline for this round: round 2, `ml35-f5-exploration-r2.md`.
Topology: **real HIL** — Gazebo Harmonic on the x86 host, Nav2 + SLAM + perception on
the Aquila AM69, `ROS_DOMAIN_ID=69`, `rmw_cyclonedds_cpp`, `ROBOT_TYPE=quadruped`,
world `quadruped_maze11.sdf`.

Single variable against round 2: **goal timeouts are provisional.** A timed-out
frontier goes to its own `_timed_out` list instead of the hard blacklist, is suppressed
immediately, is not released by ticks or `/map` republication, and is released when the
robot reaches some other frontier. Explicit Nav2 failures keep using `_blacklist`.

Raw samples: `ml35-f5-exploration-r4.csv` (1320 rows at 2 Hz).
Per-goal records: `ml35-f5-exploration-r4-goals.csv`.

> **Verdict: INCONCLUSIVE — the code path under test never executed.** `timed_out`
> stayed **0** for the entire run, alongside `refused` 0 and `blacklisted` 0. No goal
> ever reached `goal_timeout_s`, so `_timeout_current` was never called. **This run is
> not evidence for or against R4's change.**
>
> What it did surface is a **separate, pre-existing defect** that consumed the run and
> can consume any run: the explorer dispatched a goal it was already standing on, Nav2
> returned success immediately, and the loop repeated **565 times over 580 s with the
> robot motionless**. R4 must be re-run after that is fixed.

---

## 1. The change was not exercised — stated first, because it governs everything else

| status field | value throughout the run |
| --- | --- |
| `timed_out` | **0** |
| `refused` | 0 |
| `blacklisted` | 0 |

The explorer spent **6.5 s total** in `navigating` across a 641 s run. `goal_timeout_s`
is 90 s. No goal came within an order of magnitude of expiring, so the branch this round
exists to measure never ran.

The R4 diff is a no-op whenever no timeout fires: the new list stays empty, the term
added to the `_begin_selection` key is a constant 0, the added filter contributes
nothing, and clearing an empty list on arrival changes nothing. **The behaviour observed
here is round 2's code behaving differently on a different run, not a regression
introduced by round 4.** The unit tests cover the branch; the field measurement does not.

## 2. What actually happened

| metric | round 2 | round 4 |
| --- | --- | --- |
| distance travelled | 21.93 m | **0.82 m** |
| robot's bounding box after `sim_s` 60 | crossed the maze | **11 mm × 25 mm** |
| `map_known_cells` | 1710 → 8915 | 1779 → 2669, then frozen from `sim_s` 99 |
| time in `selecting` | ~13 s | **610 s** |
| time in `navigating` | most of the run | **6.5 s** |
| `selection_cycle` | 10 | **565** |
| `path_requests` | 33 | **4508** |
| `vx` work ratio | 41.71% | **1.01%** |
| `escaped` | false | false |

The robot sat at `x ∈ [-0.076, -0.065]`, `y ∈ [0.538, 0.563]` from `sim_s` 60 to the
600 s deadline. `frontier_count` held at 11 and `frontier_cells` at 556, unchanged,
because nothing moved and therefore nothing was mapped.

### The loop, per cycle

1. `_begin_selection` extracts 11 clusters and takes the nearest 8 as candidates.
2. `_validate_next` walks all 8 — that is the 8 path requests per cycle, 4508 / 565.
3. The best is `(-0.132, 0.741)`, roughly **0.2 m** from the robot.
4. `_send_navigation` dispatches it; Nav2 reports success almost at once, because the
   robot is already inside `xy_goal_tolerance` of that pose.
5. `_on_nav_result` writes `fronteira alcancada; atualizando mapa`, clears the
   provisional lists, and returns to `selecting`.
6. The robot has not moved, so the map is identical and the same 11 frontiers come back.

`frontiers.sort(key=distance from robot)` makes step 3 likely whenever any frontier
lands within goal tolerance — which is exactly what happens when the robot stops beside
unexplored space.

### Why the barren detector did not catch it

`_send_navigation` sets `self._barren_cycles = 0`. A goal *was* dispatched every cycle,
so the streak never accumulated and `barren_cycles` read 0 for the whole run. **The
no-progress detector is keyed on dispatch, not on motion**, and a dispatch that moves
the robot zero metres satisfies it. That is the gap that let a 580 s stall look healthy.

## 3. Criteria from §2.3, scored

| criterion | measured | verdict |
| --- | --- | --- |
| no infinite repetition of the same selection | 565 identical cycles | **FAIL** |
| map and coverage growing | frozen at 2669 cells for 540 s | **FAIL** |
| goals actually dispatched | dispatched constantly, moved 0.82 m | **FAIL** |
| `frontier_extract_ms` p95 < 100 ms | 269.0 ms (p50 217.8, max 311.8) | **FAIL** |
| no fall | max tilt 0.45°, `z` min 0.344 m | pass |
| no manual intervention | none after the single start call | pass |
| escape within 600 s | `escaped=false`, `prazo total de exploracao excedido` | **FAIL** |

Preconditions were met identically to rounds 2 and 3: fresh `/map` 88 × 86, robot at
`(-0.002, 0.042, 0.347)`, `escaped=false`, Nav2 `is_active` → `success=True`, started
once at `sim_s` 54, no manual goal, RTF 0.973.

## 4. What this does not validate

- Nothing about R4's variable. Re-run it after §5 lands.
- Nothing about a physical Go2 — Gazebo on the x86 host (`CLAUDE.md` rule 7).
- Nothing about the homing entry condition; the marker was never seen.

## 5. What has to happen before R4 can be measured

**A goal must not count as progress when the robot did not move.** Two shapes, one
variable each; they are independent of R4's change and of each other:

1. **Do not select a frontier the robot is already at.** Drop candidates closer than
   the effective goal tolerance during selection, so the degenerate goal is never
   dispatched.
2. **Key the no-progress detector on motion.** Let `_send_navigation` clear the barren
   streak only when the previous goal actually displaced the robot, so a
   dispatch-and-instant-arrival loop trips `barren_selections_limit` instead of hiding
   behind it.

Shape 1 removes the cause; shape 2 makes the class of failure self-reporting. Both are
worth having, and per the protocol they are separate rounds.

`frontier_extract_ms` also breached its criterion again at 269 ms p95 — but at 565
cycles it cost roughly 123 s of CPU across the run, which is no longer negligible the
way it was in round 2. It stops being a footnote if selection keeps running this often.
