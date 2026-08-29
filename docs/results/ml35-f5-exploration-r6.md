# ML3.5 F5 — round 6: homing works, and it is walking to a marker that is not there

Date: 29/08/2026. Topology: **real HIL** — Gazebo Harmonic on the x86 host, Nav2 + SLAM +
perception on the Aquila AM69, `ROS_DOMAIN_ID=69`, `rmw_cyclonedds_cpp`,
`ROBOT_TYPE=quadruped`, world `quadruped_maze11.sdf`, arm B costmap
(`nav2_params_go2_footprint.yaml`).

Single variable against R5: **`goal_timeout_s` 90 → 45 s**, sized from R5's measured goal
durations (12 successful goals, 6.1–35.1 s; 3 expiries at 90.0 s each).

Raw samples: `ml35-f5-exploration-r6.csv`. Per-goal: `ml35-f5-exploration-r6-goals.csv`.

Preconditions verified before the single start call: fresh `/map` 88 × 86; five lifecycle
nodes `active [3]`; explorer `idle`, all counters zero; deployed `maze_explorer.py`
md5-identical host → module → container (`4a5d2f5d…`); live params confirmed on the node
(`goal_timeout_s = 45.0`, `homing_persistence_s = 90.0`); exactly one
`/demo/exploration/start`.

> **Verdict: FAIL on escape, and the round found the defect that has been hiding behind
> every homing failure. The exit pose that perception publishes is roughly twice too
> close.** Homing did everything it was asked to; it walked to a phantom.

---

## 1. Recorded summary

| metric | R5 | **R6** |
| --- | --- | --- |
| `path_m` | 41.44 | 33.02 |
| `map_known_cells` | → 13378 | → 11617 |
| `vx_work_ratio` | 60.4 % | 48.9 % |
| goals total / ok | 21 / 12 | 24 / 9 |
| `refused` / `timed_out` / `barren_cycles` | 1 / 2 / 0 | **0 / 0 / 0** |
| `provisional_recoveries` | 0 | 2 |
| `homing_entries` / `homing_abandons` | 1 / 0 | 1 / **1** |
| `homing_entry_distance_m` | 1.85 | 2.36 |
| `marker_distance_m` min | 1.08 | **0.75** |
| tilt max / z min | 0.92° / 0.339 | 0.92° / 0.341 — **no fall** |

**CORRECTION — the timeout change returned nothing, and this report first claimed it did.**
The terminal `timed_out` counter reads 0, and that was misread as success. `_timed_out` is a
suppression list keyed by frontier, not a tally of events — the same trap this project
documented in the R5 report one round earlier. Counting the per-goal CSV instead:

| | expiries | each | total |
| --- | --- | --- | --- |
| R5 (`goal_timeout_s` 90) | 3 | 90.0 s | **270 s** |
| R6 (`goal_timeout_s` 45) | **6** | 45.0 s | **270 s** |

Halving the ceiling exactly doubled the number of expiries and returned **zero** budget. The
reason is conceptual: a fixed ceiling measures elapsed time, not progress, so the time lost
to goals that cannot be reached is invariant under scaling it. Successful goals did shrink
(12 → 9), so the change was not free either.

## 2. The finding: the estimated exit pose is ~2× too close

The marker is a static model in the world file, so ground truth is exact:

```xml
<model name="maze_exit_marker">
  <pose>-4.90 -2.60 0.60 0 0 0</pose>
```

At the detection instant the recorder has the robot's pose, so the comparison is direct:

| quantity | value |
| --- | --- |
| robot pose at detection (`sim_s` 312.1) | **(−2.45, 1.53)**, yaw −170.7° |
| **true** robot→marker distance | **4.80 m** |
| **estimated** distance (`homing_entry_distance_m`) | **2.36 m** |
| ratio estimated / true | **0.49** |
| marker bearing relative to the robot's heading | **+50°** (FOV is ±60°) |

The same error explains the terminal state. The closest approach was pose
**(−3.43, −0.06)** reporting `marker_distance_m = 0.75`; the true distance from there to
(−4.90, −2.60) is **2.93 m**. The robot was never near the exit.

### Two causes checked and excluded

`maze_exit_detector.marker_pose` estimates range from apparent width:
`range_m = fx * marker_width_m / width_px`.

1. **Resolution / calibration mismatch — excluded.** `/demo/camera/image_raw` and
   `/demo/camera/camera_info` both report 640 × 480, `fx = 184.836`, `cx = 320.0`.
   `marker_width_m = 0.8` matches the SDF panel exactly (0.80 × 0.03 × 0.80).
2. **Off-axis pinhole geometry — insufficient.** The panel's normal lies along ±y, so at
   this pose the incidence angle is ≈ 30.7° and the bearing is 50°. A rectilinear
   correction predicts `est/true = cos β / cos(incidence) = 0.75`. The observed ratio is
   **0.49**. Geometry accounts for at most half the error.

So `width_px` is roughly twice what the panel should subtend, for a reason not yet
identified. The leading hypothesis, **not yet tested**, is in `find_bbox`: it takes the
global min/max over *every* magenta pixel in the frame rather than the bounding box of one
connected component. Any second magenta region — a reflection, emissive bleed onto an
adjacent wall (`emissive 0.35 0 0.35`), or the panel glimpsed twice through an opening —
merges into one bbox and inflates `width_px`, which collapses the range.

### The measurement that would settle it

One sample is one sample. Park the robot at several known poses with the marker visible at
varied bearing and range, and log `width_px`, the estimated range and the true range from
odometry. If the error tracks the number of magenta regions rather than the bearing, the
bbox is the cause; if it tracks bearing, the projection model is.

## 3. Second defect, fixed: a homing step below Nav2's goal tolerance

Independently of the phantom, R6 reproduced the R4 instant-arrival pathology inside homing.
The robot sat at pose **(−3.63, 0.22) — byte-identical for 94 s** — with
`marker_distance_m` frozen at 0.93, until `homing_persistence_s` expired
(`homing_abandons` 0 → 1) and exploration wandered off to 6.71 m.

The mechanism: `marker_stop_distance_m` is 0.70 and Nav2's `xy_goal_tolerance` is 0.25. At
0.75 m the remaining step is 0.05 m — far below the tolerance — so Nav2 reports success
without the robot moving, the explorer still sees 0.75 > 0.70, and it commands the same
step again. The frontier side has had a guard for this since R4
(`min_frontier_distance_m = 0.35 > 0.25`); the approach never got the equivalent.

**Fixed:** a new `nav_goal_tolerance_m` parameter mirrors Nav2's `xy_goal_tolerance`, and
`_send_homing_step` declares arrival when `distance - stop < tolerance` instead of
commanding a step the controller cannot distinguish from zero. A contract test keeps the
two numbers equal across both params files, since the explorer does not read the Nav2 YAML.
Three tests added (71 → 74).

**This fix cannot produce an escape on its own.** With the exit pose wrong by ~2×, an
earlier arrival latch just declares `completed` sooner, at a place that is not the exit.
Order of work: the perception range first, this guard behind it.

## 4. Why reaching the marker is the escape

`maze_escape_validator` latches only after the robot crosses `y = −0.90` within
`x ∈ [−5.50, −4.30]` and then clears `y ≤ −1.28`. The marker at (−4.90, −2.60) sits 1.7 m
*outside* that boundary, so a homing run that genuinely reaches it necessarily crosses.
That is why the phantom matters so much: correct homing to a correct pose is the escape.

## 5. Limitations

HIL only. Gazebo plant on x86, Nav2/SLAM/perception on the AM69. Nothing here validates a
physical Go2, leg odometry, thermals or isolated module performance (`CLAUDE.md` rules 5
and 7). The range comparison rests on **one** detection sample; §2 states the campaign that
would confirm it.
