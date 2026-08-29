# ML3.5 F5 — round 4b: instrumentation round; the planner refuses from the START pose

Date: 29/08/2026. Topology: **real HIL** — Gazebo Harmonic on the x86 host, Nav2 + SLAM
+ perception on the Aquila AM69, `ROS_DOMAIN_ID=69`, `rmw_cyclonedds_cpp`,
`ROBOT_TYPE=quadruped`, world `quadruped_maze11.sdf`.

Single variable against the 29/08 observed round: **none — this round adds measurement
only.** `marker_distance_m`, `homing_entry_distance_m` (latched at the transition) and
`homing_entries` were added to `/demo/exploration/status`, and the recorder gained the
seven suppression columns it was silently missing. `test_the_measurement_round_adds_no_homing_gate`
fails if anyone adds the homing distance gate before the measurement exists.

Raw samples: `ml35-f5-exploration-r4b.csv`. Per-goal records:
`ml35-f5-exploration-r4b-goals.csv`.

Preconditions, all verified before the single start call:

- fresh SLAM map and pose graph — `/map` was 88 × 86 cells at start (the previous run
  ended at 126 × 190); no saved map loaded;
- spawn pose `(-0.002, 0.042)`, matching rounds 2–4a (`(0.00, 0.04)`);
- `planner_server`, `controller_server`, `bt_navigator`, `behavior_server` all `active [3]`;
- explorer `idle` with every counter at zero and the new fields present;
- deployed `maze_explorer.py` byte-identical host → module → container
  (`41131efa6ba89bba`), source mount, no arm64 rebuild;
- exactly one `/demo/exploration/start`; no manual goal, teleop or reposition.

> **Verdict: FAIL on escape, and the round did NOT collect the measurement it exists
> for.** `homing_entries = 0` — the exit marker was never detected, so
> `homing_entry_distance_m` stayed `null`. Per the handoff rule, the homing gate branch
> is **not exercised**, neither passed nor failed.
>
> The round did, however, find the defect that has been mis-attributed for four rounds:
> **Nav2 refuses every plan because the robot's own cell is `253`
> (`INSCRIBED_INFLATED_OBSTACLE`) in the global costmap.** The frontiers were never the
> problem.

---

## 1. Recorded summary

| metric | value |
| --- | --- |
| samples / sim span / wall span | 1320 / 635.2 s / 659.5 s (RTF 0.963) |
| `escaped` | **false** |
| `final_state` / `final_message` | `failed` / "nenhuma fronteira segura alcancavel" |
| run ended at | `sim_s` **241.3** (explorer elapsed ≈ 116 s) |
| `path_m` | **2.74 m** |
| `map_known_cells` | 1782 → **3681** (mapping did work) |
| `vx_work_ratio` / `vx_mean_abs` | **3.16 %** / 0.0024 |
| goals total / ok / failed / homing | 2 / 1 / 1 / **0** |
| terminal counters | `blacklisted=0 refused=4 timed_out=0 near_skipped=0 provisional_recoveries=1 barren_cycles=10` |
| `frontier_extract_ms` p50 / p95 / max | 101.2 / **101.2** / 114.5 ms (R4a p95 was 146.9) |
| tilt max / z min | 0.59° / 0.3434 m — **no fall** |
| `homing_entries` / `homing_entry_distance_m` | **0 / null — measurement not collected** |

## 2. What happened

| `sim_s` | state | counters | event |
| --- | --- | --- | --- |
| 125.4 | `waiting_map` | — | single start call accepted |
| 126.4 | `selecting` | — | |
| 128.3 | `navigating` | `fc=3 cl=3` | goal 0 → (-0.032, 1.141) |
| 138.5 | `selecting` | `fc=1 cl=2` | **goal 0 reached in 10.1 s** |
| 139.5 | `navigating` | `fc=1 cl=2` | goal 1 → (-2.034, 0.066) |
| 229.3 | `selecting` | `to=1` | **goal 1 expired at 90.0 s** — "meta de fronteira expirou" |
| 241.3 | **`failed`** | `fc=0 cl=4 refused=4 to=0 rec=1 barren=10` | 10 barren cycles |

The shape of the ending is the whole finding. Goal 1's target `(-2.034, 0.066)` was
**accepted and planned to** at `sim_s` 139.5 — so the start pose was plannable then. The
robot then spent **90 s** failing to arrive while `vx_work_ratio` sat at 3.16 % and total
travel reached only 2.74 m: it walked into a pose it could not drive out of. Twelve
seconds after the timeout, the planner refused **that same coordinate** along with three
others, and the run went barren.

`provisional_recoveries` incremented a second time in the field and again behaved to
contract: it cleared the single `timed_out` entry and the refusals, restored the
candidates, left `blacklisted = 0`, and did not repeat. The restored candidates were
refused again within seconds and the barren limit ended the run.

## 3. The refusals are not about the frontiers

Measured against the terminal state, robot at `(-0.52, 0.36)`:

```text
EXACT ROBOT CELL COST = 253
7x7 neighbourhood, robot row marked:
   254  254  254  253  253  253  243
   253  253  253  253  253  253  237
   253  253  253  253  253  253  221
   253  253  253  253  253  231  199   <== robot
   243  243  243  237  221  199  199
   199  199  199  199  221  237  243
   237  221  199  231  253  253  253
```

`253` is `INSCRIBED_INFLATED_OBSTACLE`. **NavFn treats `253` and above as lethal and
refuses to plan from such a start, whatever the goal is.** That single fact explains
every observation:

- The five goals refused during the run — `(-2.03, 0.07)`, `(-3.08, 0.26)`,
  `(-3.18, 0.16)`, `(-1.78, 0.31)`, `(-2.53, 0.16)` — are **passable and reachable** in
  the very costmap the planner uses (costs 174, 131, 152, 168, 195; all in the robot's
  BFS-connected component). Replanning to all five *after* the run still aborts.
- A breadth-first search finds paths to them only because it **relocates the start** to
  a nearby passable cell when the robot's own cell is blocked. NavFn does not do that.
  That relocation is what made the observed round's §4 analysis attribute the failures
  to the goals.
- Every frontier is refused in the same cycle, because they all share one cause.

**`_refused` is therefore the wrong bookkeeping for this failure.** The planner is
rejecting the *start*, and the explorer records the rejection against the *frontier*.
Every cluster gets blamed for a robot-pose problem, all clusters end up suppressed, and
the provisional recovery cannot help: releasing them just re-refuses them from the same
bad pose. The R4a deadlock, the observed round's collapse and this round's barren
ending are all the same defect wearing three different counters.

## 4. Why the robot ends up inside inflation

Pure geometry, from numbers already measured in `ml35-labirinto.md`:

| quantity | value |
| --- | --- |
| corridor (maze11 at scale 0.002) | **1.20 m** |
| `robot_radius` in `nav2_params_go2.yaml` | **0.38 m** (circumscribed radius of the 0.70 × 0.31 m trunk) |
| plannable corridor width | 1.20 − 2(0.38) = **0.44 m** |
| lateral tolerance before the start cell goes inscribed | **± 0.22 m** from the centreline |

`global_costmap` resolution is 0.10 m, so one cell of discretisation consumes a further
45 % of that ± 0.22 m budget.

> **CORRECTION, measured the same day — the inference below was wrong.** This section
> originally argued that a walking quadruped sways more than ± 0.22 m and therefore
> "spends much of its time legitimately inside inscribed space". A direct cost trace of
> the robot's **own** global-costmap cell, sampled at 2 Hz for 150 s of actual walking
> under the unchanged default (`robot_radius: 0.38`), says otherwise:
>
> ```text
> armA-robot_radius-0.38
> samples          297  (86 distinct poses)
> own-cell cost    min 141   median 168   max 243
> cost >= 253      0/297 = 0.0%
> ```
>
> The centre cell **never** reached 253 during normal gait, though the maximum of 243
> shows it does come close. The ± 0.22 m arithmetic stands; the conclusion drawn from it
> does not. R4b's 253 reading is therefore **a rare terminal state the robot falls into**
> — plausibly after the 90 s goal timeout left it wedged with no progress — **not a
> routine consequence of sway.** The defect is still real and still fatal when it
> happens: from a 253 cell NavFn refuses every goal. But it is intermittent, which also
> explains why some runs (round 2, and a 29/08 run that reached ~5 m with `refused=0`)
> explore perfectly well on the identical configuration.
>
> Consequence for the remedy: a larger clearance is **margin against a rare fatal state**,
> not a fix for a constant one, and any A/B must be judged over a distribution of poses
> rather than a single sample. Raw trace: `costtrace-armA.csv`.

Two remedies, both single-variable and both already measured elsewhere — **neither
applied in this round**:

| candidate | plannable width | lateral tolerance | evidence |
| --- | --- | --- | --- |
| maze scale 0.002 → **0.0025** | 1.50 − 0.76 = 0.74 m | **± 0.37 m** | `ml35-labirinto.md`: 73.3 m², 1 connected component |
| explicit footprint polygon instead of `robot_radius` (inscribed 0.155 m) | 1.20 − 0.31 = 0.89 m | **± 0.445 m** | trunk is 0.31 m wide; 0.38 is the *circumscribed* radius |

The second is the more principled — a 0.38 m circle around a 0.31 m wide robot discards
23 cm of real corridor on every side — but `nav2_params.yaml` already warns that
`use_polygon`/footprint changes interact with the collision monitor and "MUST stay false
while the costmaps use `robot_radius`". That coupling has to be read before touching it.

## 5. Against the handoff's recovery criteria

| criterion | result |
| --- | --- |
| recovery increments | **yes** (0 → 1, at `t = 105.8 s`) |
| dispatches new goals | yes — four candidates restored |
| produces material progress | **no** — refused again 13 s later |
| does not repeat before a successful arrival | **yes**, guard held |
| hard blacklist intact | **yes** (`blacklisted = 0`) |
| does not block barren termination | **yes** (10 barren cycles) |

Twice exercised, twice to contract, twice unable to help — because it operates on a
counter that is recording the wrong cause.

## 6. Measurement outcome

| field | result |
| --- | --- |
| `homing_entries` | **0** |
| `homing_entry_distance_m` | **null — NOT COLLECTED** |
| `marker_distance_m` | null throughout; marker never detected |
| `provisional_recoveries` | 1 |

The homing gate still has no measured distance behind it. The only measured value
remains round 2's **3.9 m** glimpse. The instrumentation is correct, deployed and
verified live; the run simply never got far enough to see the marker. Collecting it
requires a run that survives past the refusal storm, which §2 and §3 now explain.

## 7. Limitations

HIL only. Gazebo plant on x86, Nav2/SLAM/perception on the AM69. Nothing here validates
a physical Go2, leg odometry, thermals or isolated module performance (`CLAUDE.md`
rules 5 and 7). One failed start keeps F5 open; no cold-start campaign was attempted.
