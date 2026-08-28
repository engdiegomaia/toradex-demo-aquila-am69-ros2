# ML3.5 F5 — autonomous exploration smoke test: the explorer livelocks on a frontier the planner will not plan to

Date: 28/08/2026. Commit under test: `dd98a89`. Topology: **real HIL** — Gazebo
Harmonic on the x86 host, Nav2 + SLAM + perception on the Aquila AM69,
`ROS_DOMAIN_ID=69`, `rmw_cyclonedds_cpp`, `ROBOT_TYPE=quadruped`,
world `quadruped_maze11.sdf`.

Raw samples: `ml35-f5-exploration-smoke.csv` (1319 rows at 2 Hz).
Per-goal records: `ml35-f5-exploration-smoke-goals.csv`.
Recorder: `scripts/exploration_trial.py` (passive; publishes nothing).

> **Verdict: FAIL. `escaped=false`, final state `failed` at the 600 s deadline.
> The robot travelled 2.73 m. It spent 459 s of the 600 s budget — 77% — in
> `selecting`, re-requesting a path to the same unreachable frontier once per
> second, 459 times, and never dispatched another goal.**
>
> Per the protocol this keeps F5 open and blocks the three cold starts. No tuning
> was applied; this document classifies, as §2.4 of the plan requires.

---

## 1. Preconditions, all met before the start call

| precondition | evidence |
| --- | --- |
| simulation restarted | `docker compose restart sim`; `/clock` restarted from `sec: 0` |
| Nav2, SLAM, perception restarted | `module.sh down` then `up`; containers recreated |
| no saved map or pose graph | fresh `/map` 88 × 85 (the pre-restart grid was 122 × 226) |
| robot at the initial pose | `/demo/odom` `(-0.004, 0.070)`, `z = 0.347`, standing |
| Nav2 active | `lifecycle_manager_navigation/is_active` → `success=True` |
| TF ≥ 99.5% | 99.74% over 389 samples / 40 s |
| `escaped` false | `/demo/maze/escaped` → `data: false` |

Exploration was started **once**, at 20:20:37 wall / `sim_s` 297, with
`ros2 service call /demo/exploration/start` → `success=True, message='busca iniciada'`.
No manual goal was sent after the start, and none was sent during the run.

## 2. What the run did

| phase | `sim_s` | duration | what happened |
| --- | --- | --- | --- |
| startup | 276 – 295 | 19 s | `idle` → `waiting_map` → `selecting` |
| productive | 295 – 436 | 141 s | 5 goals dispatched, 4 reached, 1 timed out |
| **livelock** | **436 – 895** | **459 s** | **`selecting`, no goal dispatched, map frozen** |
| deadline | 895 – 907 | 12 s | `failed` — `prazo total de exploracao excedido` |

### Goals

| # | goal (x, y) | elapsed | outcome | explorer message |
| --- | --- | --- | --- | --- |
| 0 | (-0.032, 1.125) | 9.0 s | ok | `fronteira alcancada; atualizando mapa` |
| 1 | (-0.134, 0.732) | 19.0 s | ok | `fronteira alcancada; atualizando mapa` |
| 2 | (-0.134, 0.732) | 8.0 s | ok | `fronteira alcancada; atualizando mapa` |
| 3 | (-0.134, 0.732) | 9.1 s | ok | `fronteira alcancada; atualizando mapa` |
| 4 | (-0.584, 0.232) | 90.0 s | **failed** | `meta de fronteira expirou` (explorer `goal_timeout_s`) |

Goals 1, 2 and 3 are the **same coordinate, re-selected three times**. Reaching it did
not remove it as a frontier, so the explorer went back to it — the "same frontier
repeated" symptom from the plan's classification table, visible before the livelock
proper begins.

## 3. Root cause of the livelock — located, and not a mystery

From `sim_s` 436 to 895, every recorded sample carries state `selecting` and the
message `planner rejeitou todas as fronteiras` (940 of 940 samples). Over that window:

- `frontier_count` = 1, `frontier_cells` = 70, `frontier_clusters` = 1 — constant;
- `candidates_checked` = 1 per cycle — the one surviving frontier;
- `selection_cycle` 36 → 463 and `path_requests` 39 → 466 — one of each per second;
- `blacklisted` = 1 — constant;
- `map_known_cells` frozen at 3564 — mapping stopped completely.

The module log gives the reason, 459 times, identically:

```text
[planner_server]: ExplorationGrid plugin failed to plan from (-0.18, 0.12) to
                  (-3.06, 0.28): "Failed to create plan with tolerance of: 0.250000"
[planner_server]: [compute_path_to_pose] [ActionServer] Aborting handle.
```

Two independent defects combine:

**(a) The surviving frontier is unreachable and is never retired.**
`maze_explorer._blacklist_current()` is called only when Nav2 *refuses* a goal or when
a dispatched goal times out. A frontier whose `ComputePathToPose` **fails validation**
never enters the blacklist: `_on_path_result` simply leaves `_best` as `None`,
`_validate_next` sets `planner rejeitou todas as fronteiras`, and the candidate is
offered again on the next cycle, unchanged, forever.

**(b) The re-extraction guard does not hold against a republished map.**
`_begin_selection` guards on `key = (self._epoch, self._map_seq, len(self._blacklist))`
— and its own comment states the guard exists to stop exactly this busy loop. But
`_map_seq` counts `/map` **messages**, not `/map` **content**. `slam_toolbox`
republishes at `map_update_interval = 1.0 s` whether or not anything changed, so the
key changes every second, the guard never fires, and the full frontier extraction runs
again over an identical grid. That is 459 extractions at 61.9 ms median / 76.8 ms p95
producing an identical result each time — the measured cost of `maze_explorer` sitting
at 76.8% of a core while the robot stood still.

Defect (b) is what makes (a) expensive; (a) is what makes the run fail.

## 4. Criteria from §2.3, scored

| criterion | measured | verdict |
| --- | --- | --- |
| `frontier_extract_ms` p95 < 100 ms | 76.8 ms (p50 61.9, max 93.7) | **pass** |
| no infinite repetition of the same selection | 459 identical selections | **FAIL** |
| map and coverage growing | frozen at 3564 cells for 459 s | **FAIL** |
| goals actually dispatched | 5 in 600 s, none after `sim_s` 436 | **FAIL** |
| no TF regression | 99.93% (2867 samples / 300 s), was 99.74% | **pass** |
| no fall | max tilt 0.65°, `z` min 0.344 m | **pass** |
| no manual intervention | none after the single start call | **pass** |
| escape within 600 s | `escaped=false`, `failed` at deadline | **FAIL** |

Prohibited-error scan over a 15-minute log window: **0** TF extrapolation, **0**
`worldToMap`, **0** `invalid source`. The only repeated warning is the 459 planner
aborts above.

## 5. Mobility, measured separately from the livelock

While the robot actually had a goal (`navigating`, 284 samples = 142 s):

| metric | value |
| --- | --- |
| forward-work ratio in `vx` | 0.306 |
| mean \|vx\| | 0.0125 m/s |
| peak \|vx\| | 0.128 m/s |
| path travelled | 2.73 m |

Over the whole run the ratio drops to 0.068 and mean \|vx\| to 0.0028 m/s — but that
average is dominated by the 459 s with no goal at all, and quoting it as a gait result
would be wrong. The honest statement is: **with a goal, the robot moves at ~0.019 m/s
of net path rate; without one, it does not move.**

At that rate 600 s of continuous driving buys roughly 11 m of path. Whether that is
enough to clear this maze is unknown and untested, because no run has yet kept the
robot driving for 600 s. Fixing the livelock is a prerequisite to asking the mobility
question at all, which is why it is the single next variable.

Real-time factor 0.956; the run consumed 631 s of simulated time in 659 s of wall time.

## 6. Classification, per the plan's table

| symptom observed | plan's next investigation |
| --- | --- |
| `ComputePath` rejects candidates | costmap or selection |
| same frontier repeated | cache/blacklist |
| goals accepted, little movement | MPPI / trajectory decision |

The first two are the same defect and are localised to `maze_explorer` §3(a)/(b)
above. The third is real but second-order: it accounts for 142 s of the run, the
livelock for 459 s.

## 7. Single next variable

Retire a frontier whose `ComputePathToPose` fails, so the explorer stops re-offering
it — `maze_explorer._on_path_result` / `_validate_next`. Keying the re-extraction
guard on map content rather than message count is the natural companion, but it is a
CPU fix, not a correctness fix, and belongs to a separate round.

This was **not applied in this session**. The protocol requires one variable per
round, re-run under the identical protocol, and this run is the baseline that change
has to be measured against.

## 8. What this does not validate

- Nothing here describes a physical Go2. The plant is Gazebo on the x86 host
  (`CLAUDE.md` rules 5 and 7).
- The CPU and TF numbers are real AM69 measurements, but of this distributed HIL
  stack, not of the module in isolation.
- Perception's positive path was never exercised: the robot never came within sight of
  the exit marker. See `ml35-f5-perception-aquila.md`.
