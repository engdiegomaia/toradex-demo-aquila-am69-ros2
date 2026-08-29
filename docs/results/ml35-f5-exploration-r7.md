# ML3.5 F5 — round 7: the range scale error is fixed; a far commitment eats the run

Date: 29/08/2026. Topology: **real HIL** — Gazebo Harmonic on the x86 host, Nav2 + SLAM +
perception on the Aquila AM69, `ROS_DOMAIN_ID=69`, `rmw_cyclonedds_cpp`,
`ROBOT_TYPE=quadruped`, world `quadruped_maze11.sdf`, arm B costmap.

Two changes against R6, in orthogonal subsystems, each independently instrumented:

1. **`magenta_bbox` now returns the largest connected region** instead of the global
   min/max over every magenta pixel in the frame (8-connectivity on the sampled lattice,
   dependency-free — the arm64 perception image gains nothing);
2. **progress checker `0.20 m / 40 s` → `0.30 m / 25 s`**, sized by simulating
   `SimpleProgressChecker` over all 45 recorded goals of R5+R6.

`demo_perception`'s Python is now bind-mounted on the module like the explorer's, so a
detector edit costs a sync and a restart instead of a QEMU rebuild
(`tests/test_module_params_mount.py` locks it).

Raw samples: `ml35-f5-exploration-r7.csv`. Per-goal: `ml35-f5-exploration-r7-goals.csv`.

> **Verdict: FAIL on escape. The scale error is fixed — the estimate/truth ratio moved from
> 0.478 to 1.055 — but the estimate is now noisy rather than biased, the explorer committed
> to homing on a 7.35 m reading, and the persistence fix from R5 held that commitment for
> 520 s.** Fixing "gives up too early" without an entry gate produced "never gives up".

---

## 1. The range estimate: bias fixed, variance exposed

Ground truth is the SDF model at `(-4.90, -2.60)`; every sample with `marker_visible`
gives an estimate/truth pair.

**These pairs are NOT independent.** They are consecutive observations along one
trajectory, sampled at 2 Hz, and are strongly autocorrelated — 131 samples is not 131
degrees of freedom. Treat the bucket means as a description of the runs that produced them,
not as a confidence interval. Reproduce them with
`scripts/analyse_exploration.py ratio docs/results/ml35-f5-exploration-r7.csv`.

| | samples | ratio mean | range |
| --- | --- | --- | --- |
| R5 + R6 (global bbox) | 12 | **0.478** | 0.402 – 0.539 |
| **R7 (connected components)** | **131** | **1.055** | 0.474 – 2.403 |

**The systematic ~2.1× underestimate is gone.** That confirms the R6 hypothesis: a second
magenta region was merging into one bounding box, inflating `width_px`, and since
`range_m = fx * marker_width_m / width_px`, an inflated width collapses the range.

What replaces it is a range-dependent error:

| estimated range | n | ratio mean | mean absolute error |
| --- | --- | --- | --- |
| 0 – 2 m | 19 | 0.579 | 1.28 m |
| 2 – 3 m | 39 | 0.876 | 0.60 m |
| **3 – 4 m** | **43** | **1.062** | **0.41 m** |
| 4 – 6 m | 14 | 1.316 | 1.14 m |
| > 6 m | 16 | 1.813 | **3.08 m** |

Accuracy peaks in the 3–4 m band and degrades in both directions, badly beyond 6 m. The
overestimate at range is the signature of **partial visibility**: a panel glimpsed through
a maze opening presents less than its 0.80 m width, and a narrower blob reads as farther.
That is the failure mode a width-based estimator cannot fix, and it is the measured
argument for a fiducial marker (four corners and a PnP solve, which fails closed instead of
returning a confident wrong range) rather than a philosophical one.

## 2. How the run died: one far reading captured it

| `elapsed_s` | state | `marker_distance_m` | event |
| --- | --- | --- | --- |
| 103.6 | **`homing_exit`** | 3.86 | entered at `homing_entry_distance_m` = **7.35 m** |
| 150.6 – 434.6 | `homing_exit` | 7.15 / 4.18 / 1.86 / 7.94 / 7.55 / 7.34 / 7.29 | estimate swinging |
| 481.6 – 575.7 | `homing_exit` | 1.27 / 3.11 / 3.11 | still approaching |
| 600.0 | `failed` | 3.34 | total exploration deadline |

Terminal counters: `goals 7, ok 1, homing 4, refused 1, timed_out 1, barren 0,
homing_entries 1, homing_abandons 0`.

The explorer spent **520 of 600 s in `homing_exit`** and never explored again. Entry was
committed at 7.35 m — the worst-accuracy band, ~3 m of error — and `homing_persistence_s`,
which exists so a corridor wall cannot abort a good approach, faithfully kept a bad one
alive.

**This is a self-inflicted interaction and worth stating plainly.** R5's persistence fix
was correct and is still correct; shipping it without an entry gate is what let a single
bad observation own the run. The two belong together.

## 3. Fix applied for R8: a measured entry gate with hysteresis

- `homing_max_distance_m = 4.0` — above the band where R7 measured mean absolute error of
  0.41 m, below the 4–6 m band's 1.14 m. A marker beyond it is **recorded and ignored**,
  incrementing the new `marker_far_ignored` counter; exploration continues.
- `homing_confirm_observations = 3` — the estimate swung 1.27 → 7.94 m within one run, so a
  single sample must not cancel exploration. A far observation resets the streak.

`test_the_measurement_round_adds_no_homing_gate`, which existed to stop anyone adding a
gate before the measurement existed, has been **replaced** by
`test_a_far_marker_is_recorded_but_does_not_capture_the_run`, carrying the bucket table
above as its rationale. Four tests cover the gate (74 → 78).

## 4. The progress checker change is not yet judged

R7 produced only 7 goals and spent most of the run in homing, so the checker had almost no
exploration to act on. `timed_out` finished at 1. **No conclusion either way** — it needs a
run that actually explores. Its sizing evidence remains the R5+R6 simulation:

| radius | allowance | expiries caught | time returned | good goals falsely aborted |
| --- | --- | --- | --- | --- |
| 0.20 | 40 s (previous) | 2/9 | 29 s | 0 |
| **0.30** | **25 s** | **6/9** | **107 s** | **0** |
| 0.50 | 10 s (Nav2 default) | 9/9 | 427 s | **9 — all of them** |

The Nav2 default would abort every successful goal in both rounds, which is the empirical
confirmation of the note already in `nav2_params_go2.yaml`: at 0.12 rad/s of yaw, a 90°
reorientation takes 13 s at near-zero forward speed.

## 5. Limitations

HIL only. Gazebo plant on x86, Nav2/SLAM/perception on the AM69. Nothing here validates a
physical Go2, leg odometry, thermals or isolated module performance (`CLAUDE.md` rules 5
and 7). Two variables changed in this round; they are separable only because the range
ratio and the goal-expiry accounting are independent measurements.
