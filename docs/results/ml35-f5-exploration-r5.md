# ML3.5 F5 — round 5: the best exploration yet; it ran out of clock 1.08 m from the exit

Date: 29/08/2026. Topology: **real HIL** — Gazebo Harmonic on the x86 host, Nav2 + SLAM +
perception on the Aquila AM69, `ROS_DOMAIN_ID=69`, `rmw_cyclonedds_cpp`,
`ROBOT_TYPE=quadruped`, world `quadruped_maze11.sdf`.

Two variables against R4b, both justified by measurement rather than tuning:

1. **arm B costmap shape** — `NAV2_PARAMS=nav2_params_go2_footprint.yaml`, the trunk
   polygon instead of `robot_radius: 0.38` (evidence: `ml35-f5-footprint-ab.md`);
2. **bounded blind homing** — `homing_persistence_s = 90 s` plus a full blind approach,
   replacing "abandon the approach the instant the marker is not visible".

Raw samples: `ml35-f5-exploration-r5.csv`. Per-goal records:
`ml35-f5-exploration-r5-goals.csv`.

Preconditions, all verified before the single start call:

- fresh SLAM map and pose graph — `/map` 88 × 86 cells at start, no saved map loaded;
- `planner_server`, `controller_server`, `bt_navigator`, `behavior_server` **and**
  `collision_monitor` all `active [3]`;
- explorer `idle`, every counter zero, `homing_abandons` present;
- deployed `maze_explorer.py` md5-identical host → module → container
  (`998531d5…`), source mount, no arm64 rebuild;
- exactly one `/demo/exploration/start`; no manual goal, teleop or reposition.

> **Verdict: FAIL on escape — and the closest the demo has ever come.**
> `/demo/maze/escaped` stayed false. The run ended on `total_timeout_s` while in
> `homing_exit`, **1.08 m from the exit marker and still closing**. The blocker is no
> longer the planner and no longer homing: it is the clock.

---

## 1. Recorded summary, against the previous best

| metric | R4b (arm A) | R2 (previous best travel) | **R5** |
| --- | --- | --- | --- |
| `path_m` | 2.74 | 21.93 | **41.44** |
| `map_known_cells` | 1782 → 3681 | → 8915 | 1754 → **13378** |
| `vx_work_ratio` | 3.16 % | 41.7 % | **60.4 %** |
| goals total / ok | 2 / **1** | — | 21 / **12** |
| `refused` / `barren_cycles` | 4 / 10 | — | **1 / 0** |
| run length | dead at 116 s | — | full **600 s** |
| `homing_entries` / `homing_abandons` | 0 / — | 2 / — | 1 / **0** |
| tilt max / z min | 0.59° / 0.343 | — | 0.92° / 0.339 — **no fall** |

RTF 0.97, wall span 699.5 s. `frontier_extract_ms` p50/p95/max 151.8 / 221.3 / 275.0.

## 2. Homing: the persistence fix worked

| field | value |
| --- | --- |
| `homing_entries` | 1 |
| **`homing_abandons`** | **0** |
| `homing_entry_distance_m` | **1.85** |
| `marker_distance_m` min / max | **1.08** / 2.47 |
| first detection / first exit pose | `sim_s` 702.005 / 702.006 |
| homing started | `sim_s` 703.018 |

Homing entered **once**, at 1.85 m, and **never abandoned** — where the previous eleven
attempts across R2, the observed round and arm B all ended in "marcador perdido;
retomando fronteiras". The robot closed from 1.85 m to **1.08 m with the marker not
visible**, which is precisely the behaviour the fix was built for: `_exit_pose_map` is a
latched map coordinate, and line of sight is needed to learn the exit, not to reach it.

It needed roughly 0.38 m more to hit `marker_stop_distance_m = 0.7` and declare
`completed`. It had no clock left.

## 3. Where the 600 s went — and the one number that decides the next change

The per-goal record splits cleanly in two:

| | goals | total time |
| --- | --- | --- |
| **succeeded** | 12 | 219.9 s — durations **6.1 s to 35.1 s** |
| **expired at `goal_timeout_s`** | **3** | **270.0 s — exactly 90.0 s each** |

Three stalled goals consumed **45 % of the entire budget** without moving the robot, while
**every single successful goal finished in 35.1 s or less.** The ceiling was covering
nothing that was actually being used.

```text
 5 exploration   sent 244.0   90.0s  failed   meta de fronteira expirou
13 exploration   sent 467.0   90.0s  failed   meta de fronteira expirou
14 exploration   sent 559.0   90.0s  failed   meta de fronteira expirou
```

**Change applied for R6: `goal_timeout_s` 90 → 45 s.** 45 s clears the worst successful
goal (35.1 s) by 28 % and halves the cost of a stalled one — 135 s instead of 270 s, a
**135 s saving** on this run's evidence. Homing began at `elapsed` ≈ 584 s and needed
≈ 60 s more; 135 s back is enough for the escape, with margin.

The R3 experiment is not being re-run: raising the ceiling to 180 s was measured and
failed badly (4.26 m vs 21.93 m). The guard test that pinned it now asserts
`<= 90.0` — never raise — and a second test derives the floor from R5's duration
distribution, so the value is pinned from both sides by measurement.

Minor bookkeeping note: the terminal `timed_out` counter reads 2 while the goals CSV shows
three 90.0 s expiries. `_timed_out` is a suppression list keyed by frontier, not a tally of
events, so it is not expected to equal the number of expiries. The per-goal CSV is the
authority for how time was spent.

## 4. What is now proven and what is not

Proven in this run: the arm B costmap keeps the start pose plannable for a full 600 s
(`refused = 1`, `barren_cycles = 0`); the explorer sustains 60 % work ratio and 41.44 m;
homing converts a sighting into a sustained approach; no falls.

Not proven: escape. `/demo/maze/escaped` has still never been true, no cold-start campaign
has been attempted, and the footprint variant is **not promoted** — the default
`nav2_params_go2.yaml` still ships `robot_radius: 0.38`.

## 5. Limitations

HIL only. Gazebo plant on x86, Nav2/SLAM/perception on the AM69. Nothing here validates a
physical Go2, leg odometry, thermals or isolated module performance (`CLAUDE.md` rules 5
and 7). One run; a smoke exposes defects and cannot prove reliability.
