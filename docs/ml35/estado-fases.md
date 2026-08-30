# ML3.5 — phase status

Continuity document. Whoever takes this project in a new session reads **this file
first**, then [`implementation-handoff.md`](implementation-handoff.md) for the current
execution sequence. `guia-ml35-docker.md` remains the architecture specification.

Update the table and phase section when closing each gate.

---

## Objective of ML3.5

Replace the diff-drive with a A1 quadruped with real leg locomotion** (ROS 2 Jazzy +
Gazebo Harmonic), with each part of the system in its own container and the host x86 /
Aquila module explicit since the first phase.

This is the **C** option of a choice of three, made at the declared cost: weeks of work,
uncertain result. The discarded alternatives are in "Decisions" below.

---

## Current situation

| Phase | Name | Status | Commit |
|---|---|---|---|
| **F0** | Return point, ML3.1 | **Completed** 14/08/2026 | `3885f2e` |
| **F1** | Container baseline diff-drive | **Completed** 14/08/2026 | `5d95934` |
| **F2** | Spike Go2 inside the container `sim` | **Completed** 14/08/2026 | (disposable spike, uncommitted) |
| **F3** | Go2 in the project tree (was "retarget A1") | **Completed** 17/08/2026 | `db4e6f3`, `ae3d9a1` |
| **F4** | Contract crossing container border | **Completed** 24/08/2026 | contract and perception revalidated on Go2 headless |
| **F5** | Nav2 on legs + HIL mode | 🟡 **in progress** 30/08/2026 | **PASSED:** TF (99.94%), global costmap window, map update, gait, **short stability gate**, the **perception gate on the Aquila** (60/60 detections, pose and timestamped TF), autonomous exploration reaching the exit's own corridor (`R12`: `(-5.077, 1.469)`, west of the exit's `x=-4.90`), and a **positioned (non-acceptance) validation of the AprilTag fiducial pipeline** — render, detection, TF, fail-closed framing, and target latch all confirmed with real HIL evidence. **FIXED THIS SESSION:** the R9/R11 northward scoring bias, the R11 goal-tolerance false-arrival stall, and a `nav`-container-restart-before-`sim-reset` map-anchor corruption bug (R10/R12) — the last now has an automatic `scripts/module.sh up` refusal, not just a documented sequence. **FAILED:** `escaped` is still false in every round; the positioned AprilTag test additionally surfaced an unexplained quadruped fall during blind homing near the opening. **NOT REACHED:** crossing performance gate, three cold starts, and determining why exploration does not continue south of `(-5.08, 1.47)` (coverage vs. clearance vs. selection vs. execution — undiagnosed). See `docs/ml35/implementation-handoff.md` §0 for the full session-close handoff, `docs/results/ml35-f5-exploration-r{9,10,11,12}.md` and `ml35-f5-apriltag-positioned.md`. |
| **F6** | Selectable Fallback and Tests | **Completed** 24/08/2026 | cold start + goal `SUCCEEDED` on both robots |

### 29/08 (R9) — recorder data-loss fixed, full 660 s run captured, wall-clearance suspect diagnosed

R9 was run twice. The first attempt lost all time-series data to a `tools` container
missing a `docs/results` bind mount (a service-level `volumes:` key silently replacing
rather than merging the `*common` anchor's mount list); fixed in `compose.host.yml`. The
re-run is real and complete: 1320 samples over the full 660 s budget, no fall
(`tilt_deg` max 1.53°), `escaped` stayed false, final state `failed` /
"nenhuma fronteira segura alcancavel" at 48.6 % map coverage. 9 of 14 goals (64 %) ended
in the 45 s per-goal timeout rather than arrival, including the same frontier
`(-2.15, 3.12)` timing out twice in a row. Full analysis in
`docs/results/ml35-f5-exploration-r9.md`.

Live feedback while this run was in progress: the robot appears to abandon a forward path
too early on encountering a wall ahead, missing openings and side corridors. Diagnosed
(not yet HIL-tested): `frontier.py`'s `extract_frontiers` required every frontier goal to
clear 0.45 m from any occupied cell (`clearance_m`, `has_clearance`) — larger than the
robot's own footprint half-length (0.37 m). Added `frontier_wall_clearance_m` as a
declared parameter on `maze_explorer.py`, default lowered to 0.38 m, threaded into the
`extract_frontiers` call. Single isolated variable; the next round must run with this and
nothing else changed before drawing any conclusion. Not yet rebuilt into the `nav`
container or deployed to the module.

**Follow-up (R10, same day):** rebuilt `nav`/`perception` natively on the module,
redeployed, parameter confirmed live at 0.38. R10 died at `wall_s` 28.5 s — the robot
never moved (`path_m` 0.06 m, zero goals dispatched) because only one frontier cluster
ever existed and its sole candidate was refused.

**Corrected after an offline A/B diagnostic against R10's own frozen map** (no new HIL
run needed — the robot never moved, so `/map` was unchanged): a real `ComputePathToPose`
call with `maze_explorer.py`'s exact `planner_id='ExplorationGrid'` returns the
**identical candidate and the identical ABORTED result at both `clearance_m=0.38` and
0.45**. `frontier_wall_clearance_m` is **ruled out** as R10's cause — this is the
"ambos recusam" branch of the decision table: a startup/single-candidate fragility, not
a clearance effect. No reason to revert to 0.45. A separate check confirmed only one
raw frontier cluster genuinely existed (not several eaten by filters), and the planner
works normally on a closer, different goal on the same map — so the open question was why
that one candidate's region was unreachable (at the time, thought likely map coverage,
not clearance). Full diagnostic in `docs/results/ml35-f5-exploration-r10.md`.

**Update (30/08, after R12's diagnosis):** R10's own preconditions recreated `nav`
*before* calling `/demo/sim/reset` — exactly the ordering R12 later identified as
corrupting `slam_toolbox`'s map anchor. R10's sole candidate, in a cluster centred at
`(-3.04, 7.95)`, sits almost exactly where R9 had ended (`y≈7.49`), not near the real
spawn. Same signature as R12's stale-anchor failure (`error_code=208`, one small
disconnected cluster, known cells far from `(0,0)` despite a correct `map`→`odom` TF).
**R10 was in all likelihood the first reproduction of the nav-restart-before-sim-reset
bug, not an independent map-coverage/connectivity fragility** — the A/B result ruling out
`frontier_wall_clearance_m` still stands, only the explanation of *why* the candidate was
unreachable changes. Detail in `docs/results/ml35-f5-exploration-r10.md`'s own follow-up.

**Follow-up (same day) — structural fixes from the R10/R9 diagnosis, implemented in code,
HIL pending (no R11 run yet):**

- **Per-cluster multi-candidate retry.** `frontier.py`'s `extract_frontiers` now returns
  up to `frontier_max_alternates` (default 2) extra candidate points per cluster
  (`Frontier.alternates`), spaced >= `frontier_alternate_spacing_m` (default 0.25 m) apart,
  found by widening the existing inward BFS search depth. `maze_explorer.py`'s
  `_validate_next`/`_on_path_accepted`/`_on_path_result` now try every point of a cluster
  (primary, then alternates) via `ComputePathToPose` before retiring the whole cluster to
  `_refused` — a cluster is no longer lost over a single unreachable point. This would
  **not** have saved R10 itself (its one cluster's whole region was unreachable, not just
  one point within it — see the diagnostic above) but directly targets R9's distinct
  goals-7/8 pattern (same coordinate re-selected and re-refused).
- **Recessed navigation endpoint along the validated plan.** After a `ComputePathToPose`
  success, `_setback_point()` walks back `frontier_endpoint_setback_m` (default 0.40 m)
  from the path's end, along the path itself, and that point — not the raw frontier
  endpoint — is what gets commanded to `NavigateToPose`. The original endpoint is kept
  only for `frontier_score`/information-gain. Preferred over further lowering
  `frontier_wall_clearance_m` for R9's near-wall timeout pattern, per explicit direction.
- **Map-generation-gated provisional recovery.** `_last_provisional_map_seq` now gates the
  `_refused`/`_timed_out` release in `_begin_selection`: a suppression can only be lifted
  in a LATER map generation than the one that produced it, closing the R9 goals-7/8 hole
  where an identical just-expired coordinate got retried in the same cycle.
- **Telemetry added to `/demo/exploration/status`:** `frontier_clusters_raw` (raw cluster
  count before the clearance/standoff filter, vs. `frontier_clusters` after — tells apart
  "only one ever existed" from "several existed and the filter ate the rest"),
  `candidate_point_x/y` + `last_path_status`/`last_path_error_code`/`last_path_error_msg`
  (the most recent `ComputePathToPose` attempt and exactly how the planner answered — no
  more reconstructing a refusal offline after the fact, as R10 required),
  `last_path_planner_id`, and `nav_original_x/y` vs `nav_target_x/y` (frontier endpoint vs.
  the point actually commanded, which differ only when a setback point was used).
- Tests: `test_frontier.py` gained 3 new tests (alternates spacing, thin-cluster
  no-crash, raw-vs-filtered stats), `test_maze_explorer.py`'s existing tests updated for
  the new `_on_path_result(..., point)` signature. 81/81 passing
  (`test_frontier.py` + `test_maze_explorer.py`), flake8/pydocstyle clean relative to this
  change (only pre-existing baseline warnings remain, none touching the new code).
**Follow-up (R11, 29-30/08) — rebuilt, deployed, HIL round run; one regression found and
fixed mid-round; healthiest round so far, exit still not found:**

The first deploy attempt stalled completely — robot motionless for minutes on an
unchanging map. Root cause: a setback point recessed 0.40 m from a short path landed only
~0.23 m from the robot's own current pose, inside Nav2's 0.25 m `xy_goal_tolerance`, so
`SimpleGoalChecker` called the goal reached without any real motion — the same failure
class R4 already fixed once for a frontier's own endpoint, reintroduced through the new
setback point. Fixed with a `min_travel_m` floor on `_setback_point()`
(`nav_goal_tolerance_m + 0.10`): a setback candidate that close to the path's start is
discarded and the caller falls back to the original endpoint. 6 new unit tests added
(88/88 passing). Rebuilt and redeployed a second time; the clean rerun that followed:

- **`path_m` 30.91 m, map coverage 51.6 %, 14/19 goals (74 %) succeeded** — vs R9's
  34.78 m / 48.6 % / 36 % and R10's 0.06 m / 9.13 % / 0 goals. `navigating` occupied 523
  of 554 active seconds (94 % duty cycle). No fall (`tilt_deg` max 1.46°).
- Final state `failed` / "nenhuma fronteira segura alcancavel" (barren-out, not a
  timeout or a stall) after 10 barren cycles, 1 provisional-recovery use.
- Exit marker never seen. **Repeats R9's pattern**: the robot explored heavily northward
  (`y` up to 9.3-10.0) and never got near R9's reported opening at `(-4.90, -0.90)`,
  southwest of spawn (this round's trajectory only reached `x ∈ [-3.66, 0.28]`,
  `y ∈ [-0.36, 9.99]`). Two consecutive rounds with the same directional bias — a real
  pattern now, cause not yet isolated (candidate distribution vs. `frontier_score`'s
  distance term vs. genuine map topology, not distinguished by this round).
- Full record in `docs/results/ml35-f5-exploration-r11.md`.

**Follow-up (R12, 29-30/08) — northward bias diagnosed and fixed, exit still not found:**

Diagnosed offline against R11's own frozen final map (no new HIL round needed for the
diagnosis): the two clusters closest to the reported exit direction scored -3.4 against
the northern rooms' +0.9 to +1.3 on `frontier_score`, purely because a linear
`-0.5*route_m` route penalty compounds "smaller" and "farther" into permanent
uncompetitiveness. Fixed by changing the penalty to `-0.5*sqrt(route_m)` (sub-linear) —
1 new unit test, 89/89 passing.

**A second, unrelated bug surfaced before the reported run**: recreating the `nav`
container while the robot was still sitting wherever R11 left it (`y≈9.3`) let
`slam_toolbox` anchor its first scan there before `/demo/sim/reset` ran, corrupting the
map with an orphaned patch disconnected from spawn — died in 43.3 s, not counted. Fix at
the time was **operational only**: `/demo/sim/reset` must run *before* recreating `nav`,
never after.

**Update (30/08):** fixed in code, not just operationally. `scripts/module.sh up` now
calls `check_robot_near_spawn_before_nav_restart()` and refuses (exit 1) to recreate
`nav`/`perception` when `/demo/odom` shows the robot more than 1 m from spawn, printing
the correct reset-then-up sequence. Bypass is `--force-spawn`, deliberately a separate
flag from the pre-existing `--force` (cmd_vel collision guard) — `--force-spawn` means
only "the SLAM anchor doesn't matter here", never "I'll reset afterward" (resetting after
is the bug). Documented as `docs/guia-completo.md` trap 20. Committed `7f03df7`, verified
live against the module in both the refuse-path (5.05 m from spawn) and the proceed-path
(after a proper reset).

The corrected re-run: `path_m` 33.6 m, no fall, and — the real result — **goal 13
reached `(-5.077, 1.469)` successfully**, west of the reported exit's own `x=-4.90` and
the farthest any round has gotten. Final state `failed` / "prazo total de exploracao
excedido" (the 600 s `total_timeout_s` firing while still actively finding frontiers,
**not** a barren-out like every prior round) — a genuinely different, healthier failure
mode. After reaching the far west, the robot swung back to explore remaining northern
frontiers instead of continuing south; whether that's because more real unexplored area
was north, or because the corridor south of there is clearance-filtered like R11's walled
cluster, is not distinguished by this round. Full record in
`docs/results/ml35-f5-exploration-r12.md`.

**Still not done:** determining whether continuing south of `(-5.08, 1.47)` needs a
clearance fix or is simply not yet where the map's largest remaining frontier is. (The
reset-before-restart sequencing fix is now done — see the 30/08 update above.)

**Update (30/08, R13) — diagnostic-only round: the reported zigzag reads as MPPI
oscillation around an essentially straight plan, not a bent plan or a runaway gait; 4 of
7 goal-timeouts had the robot genuinely stuck, not just slow.** Same config as R12
(nothing behavioural changed), new instrumentation only: `scripts/exploration_trial.py`
gained a `/plan` subscription with `plan_straightness`/`plan_length_m`, a
`/local_costmap/costmap` probe for `wall_left_m`/`wall_right_m`, an automatic
`classify_stop_reason()` (total_timeout / two barren sub-types / cancelled), and
`find_stalled_navigating_windows()` (command present, no real displacement, >= 10 s). A
real bug was caught and fixed before the valid round: the new `/plan` subscription used
`/map`'s TRANSIENT_LOCAL QoS, but `nav2_planner` publishes `/plan` RELIABLE/VOLATILE —
the mismatch silently delivered zero messages (confirmed live by the
`incompatible QoS ... DURABILITY` warning) until fixed to a plain-depth profile.

Clean full run, no manual intervention beyond the mandated reset -> `module.sh up` ->
single `/demo/exploration/start`: 642.2 s span, 34.55 m, 16/23 goals, tilt max 1.32°
(**zero falls**), ended on `prazo total de exploracao excedido` (healthy, same pattern as
R12 — never went barren). Full record: `docs/results/ml35-f5-exploration-r13.md` +
CSVs.

Zigzag decision tree, resolved with data instead of assumption: the global plan is
mostly straight (median `plan_straightness` 0.96, only 0.7% of samples below 0.7 —
possibility 1 weak). Isolating genuine straight-corridor samples (straightness > 0.97,
both walls sensed), the MPPI still commands `|cmd_wz| > 0.02 rad/s` 82.0% of the time,
and the left/right wall-clearance asymmetry there splits nearly 50/50 by direction
(40.1% left-favoured, 36.0% right-favoured) rather than sitting on one fixed side —
textbook oscillation-around-a-straight-reference, i.e. **possibility 2**. A field
comparison of commanded `cmd_wz` against the body's own realised yaw rate (756 samples,
`state=navigating`, meaningfully-commanded) gave a median ratio of **0.80** (mild
under-execution), not the ~1.37 overshoot the `foot_placement.k_yaw=0.35` A/B hypothesis
in the R14 plan expects — same direction 97.2% of the time, ratio > 1.15 in only 9.8% of
samples. This does **not** disprove the k_yaw hypothesis (the measurement is a crude 2 Hz
comparison with no lag compensation, explicitly not a substitute for R14's own controlled
A/B), but it also does not corroborate it, which is why the report recommends R14 try the
MPPI-critic branch (PathAlignCritic/PathAngleCritic weight, `offset_from_furthest`, replan
rate) before the `k_yaw` A/B, as a evidence-weighted suggestion rather than an override of
the plan's stated order.

Stop classification: of the plan's seven categories, three were observed this round —
per-goal timeout (7/23 goals), the eventual total-timeout (final state), and "navigating
with a command but no real displacement" (5 windows found by
`find_stalled_navigating_windows`, 10.2-24.2 s each). **4 of the 7 goal-timeouts (goals 2,
6, 8, 11) contain one of these stall windows** — goal 11 alone shows the robot pinned
against a wall at a constant 0.05 m right-side clearance for close to 20 s combined across
two windows. This is real, measured evidence that R15's proposed movement watchdog (cancel
before burning the full 45 s) has genuine work to do, not a hypothetical one. Goals 9 and
21 also timed out but produced no detected stall window — open, not investigated further
this round. `frontier_clusters_raw` exceeded `frontier_clusters` on every `selecting`
sample with data (typical loss ~25-40%) but never reached zero — the clearance filter is
demonstrably active but was not this round's stop cause (`barren_cycles_final = 0`
throughout).

**Does not close**: which R14 variable to change first (evidence-weighted suggestion
only, not a controlled A/B); the two unexplained goal-9/21 timeouts; `_map_seq` counting
messages instead of content (untouched — this round never depended on re-extracting over
a republished-identical map).

**Update (30/08, R14) — first MPPI-side A/B (`PathAlignCritic.offset_from_furthest`
20→10), measured negative/inconclusive and reverted.** Per R13's report, tried the
MPPI-critic branch before `k_yaw`, single variable. Same real-HIL round, same
precondition sequence, mandated. Result did **not** support the hypothesis: on the
identical "genuine straight corridor" subset R13 defined, median `|cmd_wz|` rose
(0.052→0.079 rad/s) instead of falling, `plan_straightness` got worse (fraction below 0.9:
12.4%→39.1%), real-yaw peak-to-peak amplitude per ~5 s window got worse (median
10.7°→16.9°), and the wall-clearance asymmetry changed character from "oscillates ~50/50"
(R13's own signature for possibility 2) to "fixed left bias" (63% left) — not the pattern
the hypothesis predicted if the shorter reference had helped. Goal completion also fell
(70%→56%), and the run ended early via `barren_other` (500 s, barren-cycle limit) rather
than reaching the healthy `total_timeout` R13 and R12 both hit. Zero falls in both rounds
(tilt max 1.22° vs 1.32°); minimum wall clearance was actually better this round (0.10 m
vs 0.05 m). All of this is **n=1 per side and confounded by different maze geometry
explored each run** (R14 dispatched 16 goals vs R13's 23) — not a controlled repeat, so
treat as a real but not fully clean negative result. Full numbers:
`docs/results/ml35-f5-exploration-r14.md`. `offset_from_furthest` reverted to 20 in the
same commit round, so the next MPPI-side attempt starts from R13's known baseline instead
of stacking under an unproven change. Remaining untried MPPI-side candidates: PathAlignCritic
weight, PathAngleCritic weight, replan/plan-substitution frequency — none attempted yet;
which one to try next (or whether to fall back to the plan's original `k_yaw` A/B) is an
open decision, not yet made.

**Update (30/08, R15) — the four centralisation-adjacent software fixes, implemented and
unit-tested, not yet HIL-validated.** All four land in `maze_explorer.py` together (no
hardware run required to validate the logic itself, only to validate real-world behaviour
in R16):

1. `_map_seq` now advances on `/map` **content** change (crc32 of `.data`), not per
   message — `slam_toolbox` republishing an identical map no longer satisfies
   `map_seq > last_provisional_map_seq` and wrongly releases a provisional suppression
   nothing actually disproved. New test:
   `test_map_republication_does_not_release_a_provisional_recovery` (the exact test the
   original plan asked for by name), plus a direct content-vs-message test on `_on_map`.
2. Movement watchdog during `navigating`: cancels the goal as a provisional timeout after
   `stall_window_s` (15 s) without `stall_move_threshold_m` (0.05 m, same threshold R13's
   own offline `find_stalled_navigating_windows` already validated) of real displacement —
   instead of waiting out the full `goal_timeout_s` (45 s). Sized directly off R13's 5
   measured real stalls (10.2-24.2 s, all above the 15 s window with margin).
3. Zero-raw-cluster observation recovery: when NO raw frontier cluster exists at all (not
   "existed but got filtered" — that path is unchanged), dispatches Nav2's own `Spin`
   behavior through `behavior_server`, which already runs at the hardware-validated
   `max_rotational_vel: 0.12` (the value measured to not fall the robot in recovery).
   Limited to one attempt per distinct (content-based) map version.
4. Honest barren classification: the single "nenhuma fronteira segura alcancavel" message
   split into three, matching the raw-vs-filtered distinction `classify_stop_reason`
   (R13) already reads from the final row's counts.

100/100 tests pass (`colcon test --packages-select demo_navigation`; 87 of them in
`test_maze_explorer.py`, up from 65), flake8-clean relative to the prior commit on both
touched files.

**Does not close**: whether R15's watchdog/recovery actually help exploration reach the
southwest exit region on real hardware (that is R16's job, integrated with whichever
config R14's decision lands on); which MPPI-side variable to try next; the homing-fall
investigation near the exit opening (still untouched, no round this far has approached
that area).

**Update (30/08) — item 6, AprilTag positioned HIL validation, done (explicitly
non-acceptance-counted, no exploration ran):** robot teleported directly to vantage
points in front of the exit marker (safe hold-gait/set_entity_pose/resume-gait sequence,
always kept north of `BOUNDARY_Y` with margin). Confirmed with real HIL evidence: the
printed tag texture renders correctly in Gazebo; `cv2.aruco` finds id 0 reliably at
~2.5 m (14/20 frames, reprojection error <= 0.48 px) but only marginally at ~3.6 m (1/21
frames) — a real range-dependent reliability curve worth weighing against
`homing_max_distance_m=4.0`; `map -> front_camera` TF resolves; adverse framing (yaw swept
to -140°) always fails closed (never a wrong-but-confident pose), though only the coarser
"no tag found" path was empirically triggered, not the finer "partially out of frame"
one; and the 29/08 audit's target-latch fix holds under real detections
(`marker_accepted_x/y` stayed fixed at `(-3.503, -2.75)` for ~90 s while
`marker_candidate_x/y` reflected the last live sighting). **Unplanned finding:** during
the latch test's blind-homing phase, the quadruped tipped over near the opening
(`(-5.057, -1.255)`, large roll/pitch, `/demo/maze/escaped` read `false` — confirmed, not
inferred, and very close to the escape threshold). Not investigated further; recovered
cleanly via `/demo/sim/reset`. This is a new, real risk for whoever next drives homing
through the exit opening, not previously known. Full record in
`docs/results/ml35-f5-apriltag-positioned.md`.

### 29/08 (fiducial) — the exit marker gets a printed AprilTag, magenta stays as fallback

**Implemented in code, HIL pending.** The magenta-panel range estimate is unbiased but
noisy with distance (R7: 0.41 m mean error at 3-4 m, 3.08 m above 6 m — see the R7/R8
entry below); a fiducial fails **CLOSED** instead of returning a confident wrong range.
`maze_exit_detector.py` gained a `detector_backend` parameter (`fiducial` by default,
`magenta` as a configurable fallback — never both publishing at once) and a pure
`find_fiducial`/`fiducial_pose`/`fiducial_diagnostics` path using `cv2.aruco`'s **legacy
functional API** (`getPredefinedDictionary`, `DetectorParameters_create()`,
`detectMarkers()`, `estimatePoseSingleMarkers()`) — this project's OpenCV is 4.6.0, which
predates the newer `ArucoDetector` class. Dictionary `DICT_APRILTAG_36h11`, id 0, 0.64 m
tag core inside the existing 0.80 m magenta panel (`quadruped_maze11.sdf`'s
`maze_exit_marker`, pose unchanged). The tag texture is generated deterministically by
`scripts/generate_maze_exit_marker.py` (not downloaded, regenerate instead of
hand-editing the PNG) and installed with the package via `demo_simulation`'s `setup.py`.

The detector still does not know the maze — the tag's dictionary/id/size are declared
parameters with in-code defaults, not read from the scenario, and the isolation contract
test (`test_neither_perception_nor_frontier_knows_the_maze`) still passes. The explorer's
existing 3-observation confirmation gate (see the "(auditoria)" entry just below) is
untouched by this change; it consumes whichever backend is active through the same
`/demo/perception/maze_exit/pose` topic.

Tests: 22 new/updated in `demo_perception` (pure-function reprojection-error checks,
node-level pose/diagnostics publishing, backend switching, clipped/wrong-id rejection),
5 structural checks on the SDF/texture asset in `demo_simulation`. Root suite unaffected
(261 passed). **Not yet run**: real detection against the actual Gazebo-rendered texture
(only a synthetic OpenCV-drawn frame was exercised), and the container image has not been
rebuilt to re-verify `cv2.aruco` resolves the legacy API at runtime with this new code
path — the interpreter-level spike for that API predates this change.

### 29/08 (auditoria) — o portao nao tinha histerese; dois defeitos corrigidos

Revisao independente do trabalho de R7/R8 achou **dois defeitos funcionais que os testes
nao pegavam porque os testes codificavam a mesma semantica errada.**

1. **As "tres confirmacoes" eram tres ciclos do timer, nao tres observacoes.** O contador
   subia em `_tick` (1 Hz) enquanto as deteccoes chegam em `_on_exit_pose`, e uma pose fica
   fresca por `marker_stale_s` = 2 s -- entao **um unico quadro ruim satisfazia as tres
   confirmacoes**. O portao nao dava histerese nenhuma, e `marker_far_ignored` contava
   ciclos do timer. Agora a confirmacao so avanca em `_on_exit_pose`, uma vez por mensagem,
   deduplicada por `(frame_id, stamp_ns)`.
2. **O portao protegia so a entrada, nao o alvo.** Toda observacao nova sobrescrevia
   `_exit_pose_map`, inclusive durante `homing_exit`, entao a oscilacao de 1,27 a 7,94 m
   medida em R7 podia deslocar o alvo depois da entrada. Separado em
   `_exit_candidate_pose_map` (observacao bruta, sempre publicada) e `_exit_pose_map` (alvo
   aceito, **travado durante a tentativa**).

Tambem: borda de igualdade da tolerancia (`<=` com margem numerica); numeros derivados agora
reproduziveis por `scripts/analyse_exploration.py` com helpers testados; instrumentacao de
percepcao em `/demo/perception/maze_exit/diagnostics` (contagem de regioes, bbox, razao de
aspecto, larguras de todas as regioes) e do explorador (`marker_observations`,
`marker_confirmations`, pose candidata e pose aceita).

**Duas afirmacoes exageradas foram corrigidas:** os 131 pares de R7 sao amostras
consecutivas de uma trajetoria a 2 Hz, fortemente autocorrelacionadas -- nao sao 131 graus
de liberdade; e R5/R8 nao sao "a mesma configuracao", so mesma topologia e footprint.

Testes 80 + 39 + 256. **Nem o portao nem a guarda de tolerancia foram exercitados em HIL.**

**Correcao (29/08, segunda passada): a "queda de mobilidade" de 10x era artefato do
gravador, nao fisica.** `scripts/exploration_trial.py` media `vx_mean_abs`/`vx_work_ratio`
sobre a janela inteira de gravacao; em R8 o explorador falhou aos 76,5 s mas o gravador
seguiu ate 670 s, entao ~594 s de zeros pos-falha diluiram a media por ~8,8x. Com o
gravador agora separando `active_*` (da primeira amostra em estado ativo ate a primeira
amostra terminal) de `recording_*` (arquivo inteiro), R8 recalculado direto do CSV mostra
`active_vx_work_ratio` 55,5% e `active_vx_mean_abs` 0,036 m/s contra 66,5% / 0,055 m/s de
R5 — uma diferenca de ordinaria variancia entre corridas, nao um colapso de uma ordem de
grandeza. R8 morreu cedo (76,6 s de janela ativa) por causa do modo esteril, nao por
mobilidade. Ver `docs/results/ml35-f5-exploration-r8.md`.


### 29/08 (R7/R8) — a estimativa de alcance deixa de ser enviesada; o portao de homing entra

**R7 — o erro de escala da percepcao esta corrigido.** `magenta_bbox` tomava min/max global
sobre TODO pixel magenta do quadro, entao qualquer segunda regiao magenta entrava na mesma
caixa, inflava `width_px` e, como `range_m = fx * marker_width_m / width_px`, encolhia a
distancia. Trocado pela maior regiao CONEXA (8-vizinhos na grade amostrada, sem dependencia
nova na imagem arm64), a razao estimado/real contra o marcador do SDF em (-4,90, -2,60)
passou de **0,478** (12 amostras, R5+R6) para **1,055** (131 amostras). A calibracao ja
tinha sido descartada: `horizontal_fov` 2,094 rad em 640 px da fx 184,75 contra os 184,836
publicados.

O que sobra e erro dependente da distancia: erro absoluto medio de **0,41 m** na faixa
3-4 m, 1,14 m em 4-6 m e **3,08 m acima de 6 m**. A superestimativa longe e a assinatura de
**visibilidade parcial** -- painel visto por uma abertura mostra menos que seus 0,80 m e
uma mancha mais estreita le como mais longe. Nenhum estimador por largura resolve isso; a
resposta estrutural e um marcador fiducial (AprilTag/ArUco: quatro cantos e PnP, que falha
fechado em vez de devolver alcance errado com confianca). Isso mexe na premissa do demo
descrita no proprio SDF, entao e decisao de produto, nao correcao de bug.

R7 falhou por interacao propria: entrou em homing a **7,35 m** -- a pior faixa -- e como a
persistencia de R5 nunca desiste, uma unica observacao ruim prendeu a corrida **520 de
600 s** em `homing_exit`. Consertar "desiste cedo demais" sem portao de entrada produz
"nunca desiste". Os dois pertencem a mesma mudanca.

**Portao implantado:** `homing_max_distance_m` 4,0 m (acima da faixa onde o erro medido e
0,41 m), `homing_confirm_observations` 3 (a estimativa oscilou de 1,27 a 7,94 m na mesma
corrida) e o contador `marker_far_ignored`. `test_the_measurement_round_adds_no_homing_gate`
foi SUBSTITUIDO por `test_a_far_marker_is_recorded_but_does_not_capture_the_run`.

**R8 — inconclusivo.** Morreu no modo esteril: UMA meta expirou (45,0 s), a recuperacao
provisoria soltou o que podia, os dois candidatos restantes foram recusados,
`frontier_count` foi a zero e o limite de ciclos esteris encerrou. A rodada mal andou --
**3,46 m** e razao de trabalho de **6,3%**, contra 41,44 m e 60,4% de R5 na mesma
configuracao -- entao tinha explorado pouco e tinha poucas fronteiras a perder. O marcador nunca foi
detectado (`homing_entries = 0`), entao **nem o portao nem a correcao de percepcao foram
exercitados**, e o progress checker segue sem julgamento por duas rodadas. Assinatura de
R4b: o robo anda para uma pose de onde nao consegue planejar. Arm B tornou isso raro, nao
impossivel -- R5/R6/R7 terminaram com `refused` <= 2 e sobreviveram porque ainda tinham
fronteiras; R8 tinha duas.

**Correcao de um erro desta sessao:** o relatorio de R6 afirmou que `goal_timeout_s` 90 -> 45
funcionou. Nao funcionou -- R6 produziu 6 expiracoes de 45,0 s = 270 s, exatamente os
3 x 90 s de R5. O contador terminal `timed_out` e uma lista de supressao, nao contagem de
eventos; conte no CSV por meta. Prazo fixo mede tempo decorrido, nao progresso.


### 29/08 (R5/R6) — homing fixed twice, and the exit marker turns out to be a phantom

Two protocol rounds under arm B, each one measured single variable. Evidence:
`docs/results/ml35-f5-exploration-r5.md`, `ml35-f5-exploration-r6.md` and their CSVs.

**R5 — the best exploration the demo has produced, stopped by the clock.** 41.44 m
travelled (previous best 21.93 m), 13 378 cells mapped (8 915), `vx_work_ratio` 60.4 %,
12 of 21 goals reached, `refused = 1`, `barren_cycles = 0`, no falls. Homing entered once
at 1.85 m, **never abandoned** (`homing_abandons = 0`), and closed to **1.08 m with the
marker not visible** — the persistence fix doing exactly its job after eleven consecutive
field failures. It ran out of `total_timeout_s` mid-approach.

Where the 600 s went: 12 successful goals took **6.1–35.1 s** each; 3 stalled goals took
**exactly 90.0 s** each — 270 s, **45 % of the budget**, with the robot not moving. So
`goal_timeout_s` 90 → 45 (clears the worst good goal by 28 %). **It returned nothing:** R6
produced 6 expiries of 45,0 s = 270 s, exactly R5's 3 × 90 s. A fixed ceiling measures
elapsed time, not progress, so the loss is invariant under scaling it. The terminal
`timed_out` counter reading 0 is a suppression list, not an event tally -- count the
per-goal CSV.

**R6 — the defect behind every homing failure.** The marker is a static SDF model at
`(-4.90, -2.60)`, so ground truth is exact. At detection the robot was at `(-2.45, 1.53)`:
true distance **4.80 m**, perception's estimate **2.36 m**, ratio **0.49**, marker 50°
off-axis. At closest approach the robot was at `(-3.43, -0.06)` reporting 0.75 m while
truly **2.93 m** away. **Homing has been walking to a point that is not the exit.**

Two causes checked and excluded: image and `camera_info` agree (640 × 480, `fx` 184.836,
`marker_width_m` 0.8 matches the SDF panel); and off-axis projection predicts 0.75, not
0.49. So `width_px` is ~2× the panel's true subtense. Leading hypothesis, **untested**:
`find_bbox` takes the global min/max over every magenta pixel rather than one connected
component, so a second magenta region merges into the bbox and collapses the range. One
sample only — the confirming campaign is specified in the R6 report §2.

This is now the **top blocker**, because `maze_escape_validator` latches only on crossing
`y = -0.90` within `x ∈ [-5.50, -4.30]` and clearing `y ≤ -1.28`, and the marker sits 1.7 m
outside that boundary: homing correctly to a correct pose *is* the escape.

**Also fixed in R6:** the robot froze at a byte-identical pose for **94 s**. With
`marker_stop_distance_m` 0.70 against Nav2's `xy_goal_tolerance` 0.25, a 0.05 m remaining
step succeeds without motion and is re-commanded forever — the R4 instant-arrival trap,
which the frontier side has guarded since R4 via `min_frontier_distance_m = 0.35`. New
`nav_goal_tolerance_m` mirrors the Nav2 value (contract test keeps them equal in both
params files) and arrival is declared when `distance - stop < tolerance`. It cannot produce
an escape while the marker pose is wrong; it only stops the freeze.

Tests 74 (package) + 247 (root). Nothing committed.


### 29/08 (latest) — the footprint polygon unblocks the planner; homing is now the blocker

Two findings, one experiment and one code defect. Evidence:
`docs/results/ml35-f5-footprint-ab.md`, `costtrace-arm{A,B}.csv`,
`ml35-f5-footprint-ab-live.csv`.

**1. Replacing `robot_radius` with the real trunk polygon removes the 253 failure mode.**
`robot_radius: 0.38` is the *circumscribed* radius of a 0.70 × 0.31 m body, so a 0.38 m
circle around a 0.31 m wide robot discards 23 cm of corridor per side. The variant
`nav2_params_go2_footprint.yaml` swaps both costmaps to a
`[±0.37, ±0.18]` rectangle and changes nothing else — enforced by a contract test that
reverts the polygon and requires the two YAMLs to compare equal (241 → 247 root tests).

A 2 Hz trace of the robot's own global-costmap cell during 150 s of walking:

| | arm A (`robot_radius`) | arm B (polygon) |
| --- | --- | --- |
| median / max own-cell cost | 168 / **243** | 135 / **165** |
| % of samples at ≥ 243 | **7.1 %** | **0.0 %** |
| headroom to the fatal 253 | **10** | **88** |

The feared collision-monitor coupling did not bite — the variant *removes* `robot_radius`
instead of adding a polygon beside it. No SIGSEGV, no "Inconsistent configuration in
collision checking", footprint published on both costmap topics, `collision_monitor`
`active [3]`, static corridor planning 4/4 SUCCEEDED.

Behaviourally the failure mode changed completely: R4b under arm A died at **116 s** with
`refused=4, barren_cycles=10`; arm B ran the **full 600 s** and ended on
`prazo total de exploracao excedido` with `refused=2, barren_cycles=0`. The planner stopped
being the limit.

Caveats kept on the record: neither arm ever sampled 253, so this is **margin**, not a
prevented event; and arm B covered 50 distinct poses vs arm A's 86, so the traces are not
pose-matched. **Promoted in code on 29/08/2026, HIL final pending** — `nav2_params_go2.yaml`
and `params-align8.yaml` now ship the footprint polygon in both costmaps instead of
`robot_radius`; `nav2_params_go2_footprint.yaml` is kept, byte-equivalent to the default,
only because Compose/docs/prior commands still reference it. A clean smoke from t = 0 and
a lateral-displacement replan check under the promoted default are still to run in HIL.

**2. Homing is 0 for 11, and the cause is in the code.** `_send_homing_step` walked to the
exit in 0.5 m hops and after each hop dropped the entire approach unless the marker was
visible in that instant (`marker_stale_s = 2.0`). But `_exit_pose_map` is a latched map
coordinate — line of sight is needed to *learn* the exit, not to reach it, and a maze
corridor breaks line of sight by construction. Hence the twice-recorded signature: homing
goals failing at 0.99 s then 8–9 s with "marcador perdido".

Fixed in the host tree (deployed, **not committed**): `homing_persistence_s` (90 s) lets
homing keep navigating to the latched pose while the marker is stale, and when blind it
sends the **full approach** rather than a straight-line 0.5 m hop so Nav2 can route around
walls. Past the budget it gives up and increments the new `homing_abandons` counter. Five
tests (65 → 70), one of which asserts the budget outlives the freshness deadline, so the
old "never chase the last-seen pose forever" property survives — bounded instead of
instant.

The homing **entry** gate still has only a partial measurement: `homing_entry_distance_m`
latched **3.06 m** on the last of arm B's five entries (R2's glimpse was 3.9 m); the other
four were lost because the logger attached after them.


### 29/08 (late) — the planner refuses from the START pose; frontiers were never the cause

Evidence: `docs/results/ml35-f5-exploration-r4-observed.md` (observed round, does NOT
count toward acceptance) and `ml35-f5-exploration-r4b.md` (protocol round), with CSVs
beside them.

- **The provisional recovery is field-validated, twice.** `provisional_recoveries`
  incremented in both runs, released only `_refused`/`_timed_out`, left the hard
  blacklist at 0, did not livelock, and terminated through `barren_cycles` as designed.
  It passes its own contract and it does **not** rescue either run.
- **Root cause found, and it is the start pose, not the goals.** At the R4b terminal
  state the robot's own global-costmap cell reads **253 (`INSCRIBED_INFLATED_OBSTACLE`)**.
  NavFn refuses to plan from a start at 253 or above, whatever the goal is. The five
  goals refused during the run are all passable (costs 131–195) and inside the robot's
  own connected component; replanning to them afterwards still aborts. A BFS finds
  paths only because it relocates the start to a passable cell — NavFn does not, and
  that relocation is what made earlier analysis blame the frontiers.
- **Therefore `_refused` records the wrong cause.** The planner rejects the *start*;
  the explorer books the rejection against the *frontier*. Every cluster is blamed for
  one robot-pose problem, all clusters end up suppressed, and the recovery cannot help
  because releasing them re-refuses them from the same bad pose. The R4a deadlock, the
  observed round's frontier collapse and the R4b barren ending are **one defect wearing
  three counters**.
- **The geometry says this is expected.** maze11 corridors are 1.20 m and
  `robot_radius` is 0.38 m (circumscribed for the 0.70 × 0.31 m trunk), leaving
  1.20 − 0.76 = **0.44 m** plannable — **± 0.22 m** off the centreline before the start
  cell goes inscribed, with a 0.10 m costmap cell eating a further 45 % of that. A
  walking quadruped sways more than that. Two measured, unapplied remedies: maze scale
  0.0025 (corridor 1.50 m → ± 0.37 m, already measured at 73.3 m² / 1 component in
  `ml35-labirinto.md`), or an explicit footprint polygon instead of `robot_radius`
  (inscribed 0.155 m → ± 0.445 m) — the latter is coupled to the collision monitor and
  `nav2_params.yaml` warns about it explicitly.
- **Homing remains 0 for 6.** Across round 2 and the observed round, six homing
  attempts, zero successes, with matching signatures (0.99 s then 8–9 s). In the
  observed round homing **preempted a healthy six-frontier exploration goal** and gave
  it back collapsed to one. Ranked behind the start-pose defect only because the
  refusal storm now kills runs before the marker is ever seen.
- **Instrumentation added, measurement NOT yet collected.** `marker_distance_m`,
  `homing_entry_distance_m` (latched at the transition, because status is 2 Hz and the
  entry is instantaneous) and `homing_entries` now publish on
  `/demo/exploration/status`; the recorder gained the seven suppression columns it was
  silently missing. R4b ended with `homing_entries = 0`, so the gate branch is **not
  exercised** and the only measured marker distance is still round 2's **3.9 m**.
  `test_the_measurement_round_adds_no_homing_gate` fails if the gate is added first.

**Next single variable**, in order: fix the start-pose refusal (geometry or footprint),
because until Nav2 can plan from wherever the gait leaves the robot, no frontier policy
and no homing gate can be evaluated at all.

### 29/08 — the robot explores the maze and finds the exit marker on its own

Four things closed and one refused. Evidence: `docs/results/ml35-f5-perception-aquila.md`
§6 and `ml35-f5-exploration-r{1,2,3}.md`, with per-run CSVs beside them.

- **Perception gate CLOSED.** The 28/08 shortfall was line of sight, not software.
  With the robot placed at the exit region — `gait/hold` → `set_entity_pose` →
  `gait/resume`, never a raw teleport, because `SetEntityPose` preserves velocity —
  the detector returned **60/60 non-empty detections**, `class_id maze_exit` at score
  1.0, bbox 149 × 153 px, pose `(0.992, 0.011, 0.000)` in `front_camera`, and
  `front_camera` resolvable to `map`, `odom` and `base` at the pose timestamp. TF held
  **99.83%**, so CPU did not interfere and `sample_stride` stays at 4. One quantified
  limitation: range reads **21% short** (0.992 m against 1.254 m true) because the
  panel's emissive material blooms 26% past its geometric edge.

- **Round 1 — the livelock is gone.** Retiring frontiers the planner refuses turned
  459 s in `selecting` into 15 s, 463 selection cycles into 14, and 459 planner aborts
  into 2. The run then failed fast and explicitly on a new wall instead of burning the
  budget looking busy.

- **Round 2 — the demonstration essentially works.** Making planner refusals
  *provisional* (cleared on arrival, because `ExplorationGrid` runs `allow_unknown:
  false` and refuses distant frontiers only until the path is mapped) produced
  **21.93 m travelled, 8915 map cells, 13 goals with 7 reached, 41.7% work ratio**, and
  the **first autonomous detection of the exit marker in this project** at `sim_s`
  402.9, with `homing_exit` entered at 403.0.

- **Round 3 — REJECTED, and locked.** Raising `goal_timeout_s` 90 → 180 made
  everything worse (4.26 m, 3746 cells, 8.6% work ratio, no detection) because a goal
  **0.4 m away** consumed the full 180 s. The ceiling cuts stalls, not slow
  traversals. Reverted to 90 s;
  `test_the_goal_timeout_stays_at_the_value_that_was_measured_best` now blocks the
  repeat.

**What still blocks the escape, in order.** (1) The **permanence** of the timeout
blacklist: a stalled goal is retired for the whole run, and in round 2 three such
entries swallowed the last four frontier clusters at `sim_s` 570. (2) **Homing commits
too early**: the marker was first seen from 3.9 m through the opening, and the next
wall occluded it 9 s later. (3) `frontier_extract_ms` p95 reached 201 ms on the bigger
map, over the 100 ms criterion — real, but under 2 s across the whole run.

**Infrastructure.** `compose.module.yml` now mounts
`demo_navigation/demo_navigation` over the symlink target, the same trick already used
for `config/`. An explorer edit costs `module.sh sync` plus a container restart instead
of a native arm64 rebuild of `base` and then `nav`. Verified live on the AM69.

### 28/08 (night) — perception transport passes on the module; the exploration smoke fails on an explorer livelock

Evidence: **`docs/results/ml35-f5-perception-aquila.md`** and
**`docs/results/ml35-f5-exploration-smoke.md`**, with their CSVs next door.
Recorder added in `scripts/exploration_trial.py` (+ `tests/test_exploration_trial.py`).
Commit under test: `dd98a89`, images unchanged since `8676941` (description strings only).

**Gate A — perception on the Aquila: transport PASSES, positive detection NOT REACHED.**

* `/demo/camera/camera_info` reaches the module at 9.70 Hz, so the pose estimator has
  intrinsics. That was the first listed blocker and it is cleared.
* `/demo/perception/maze_exit/detections` publishes at ~9.6 Hz, one message per image.
  The detector consumes `/demo/perception/image_in`, the module-local `republish` of
  the compressed camera topic — not `image_raw`.
* The `image_in` rate reads 7.0 Hz, *below* the detection rate. That ordering is
  impossible and is a measurement artefact: `ros2 topic hz` on an uncompressed image
  topic inside the module is itself the load. **Do not quote 7.0 Hz as the detector
  input rate**; 9.7 Hz is the honest figure.
* No positive detection was obtained. The exit marker sits 1.70 m *outside* the maze,
  in line with the opening at `x = -4.90, y = -0.90`; from inside it is visible only
  from close to that opening. A bounded 430° in-place sweep produced 445 detection
  messages, all empty, and the exit region was entirely unknown in `/map`. The robot
  never got line of sight, so this is a mobility failure, not a perception one.
* Detector CPU is 89.1% of one core (`sample_stride=4`) and module load average
  reached 19.68 on 8 cores — but TF held at **99.93%** during the run and no
  `invalid source` appeared. **`sample_stride` was therefore left at 4**: the
  protocol only authorises changing it if CPU interferes, and by its own criteria it
  did not.

**Gate B — exploration smoke: FAIL. `escaped=false`, `failed` at the 600 s deadline.**

* Preconditions all met and verified: fresh 88 × 85 `/map` (no saved pose graph),
  robot at spawn, Nav2 `is_active`, TF 99.74%, `escaped=false`. Started **once**.
* 5 goals dispatched in the first 141 s — 4 reached, 1 killed by the explorer's own
  90 s timeout. Goals 1–3 were the **same coordinate re-selected three times**.
* Then **459 s — 77% of the budget — livelocked in `selecting`**, message
  `planner rejeitou todas as fronteiras` on 940 of 940 samples, `map_known_cells`
  frozen at 3564, one `ComputePathToPose` request per second, all aborting with
  `ExplorationGrid plugin failed to plan from (-0.18, 0.12) to (-3.06, 0.28)`.
* Total path 2.73 m. No fall (max tilt 0.65°). No TF regression. Zero prohibited
  errors (`extrapolation`, `worldToMap`, `invalid source`) in a 15-minute window.
  `frontier_extract_ms` p95 = 76.8 ms, inside the 100 ms budget.

**Root cause, located.** Two defects in `maze_explorer` compound:

1. `_blacklist_current()` runs only when Nav2 *refuses* a goal or a dispatched goal
   times out. A frontier whose `ComputePathToPose` **fails validation** is never
   retired — `_on_path_result` leaves `_best` at `None` and the same candidate is
   re-offered forever. This is what fails the run.
2. `_begin_selection`'s guard keys on `_map_seq`, which counts `/map` **messages**,
   not **content**. `slam_toolbox` republishes every 1.0 s regardless, so the guard
   never fires and a full frontier extraction runs each second over an identical
   grid. Its own comment says the guard exists to prevent exactly this. This is what
   makes the livelock cost 76.8% of a core.

**Single next variable:** retire a frontier whose path validation fails
(`_on_path_result` / `_validate_next`). **Not applied in this session** — the protocol
requires one variable per round re-run under the identical protocol, and this run is
the baseline it must be measured against. The map-content guard is a CPU fix and
belongs to a separate round.

**The three cold starts remain blocked.** A smoke pass is their precondition, and F5
stays open.

**Mobility, stated honestly.** With a goal active the robot manages a forward-work
ratio of 0.306 and ~0.019 m/s of net path rate; the whole-run figures (0.068 and
0.0028 m/s) are dominated by the 459 s with no goal at all and must not be quoted as
a gait result. Whether ~11 m of path in 600 s clears this maze is still **untested**,
because no run has yet kept the robot driving that long.

### 28/08 (late) — the TF gate is closed; speed is not

Evidence: **`docs/results/ml35-f5-ab-joint-states.md`** and
**`ml35-f5-portao-tres-metas.md`**, with the CSVs next door. Committees `a7dc097`
(decimation), `235ac1f` (probe), `d7efd30` (reversion of the map), `becac77` and
`fa1d012` (evidence).

* *The root cause was `/tf` a 1090 Hz.** The `controller_manager` runs the 1000 Hz
because physics runs at 1000 Hz, and a controller without its own `update_rate` inherits
that rate. `joint_state_broadcaster` published `/joint_states` to 1 kHz,
`robot_state_publisher` converted each sample into a `TFMessage`, and eleven subscribers
unserialized the result — across the Ethernet, because `robot_state_publisher` runs on
HOST and navigation runs on the Aquila. Nav2 does not consume any of this: the edges she
uses are FIXAS together and already come out once in `/tf_static`. The 1090 Hz were the
twelve joints of the PERNAS.

The correction is `update_rate: 50` in the broadcaster, by the spawner
(`demo_simulation/config/joint_state_broadcaster.yaml`), exact factor decimation 20.
Lasso, march, physics, IMU and untouched odometry, and there is structural testing for
each of them — the cheap failure mode is to lower the Lasso rate instead of the
BROADCASTER, two editions of a line in the same file.

**A/B paired, a variable, same protocol in both arms** (reveria `sim` → restart module →
wait SLAM → stabilize → measure):

|  | 1000 Hz | 50 Hz |
| --- | ---: | ---: |
| `/joint_states` | 986,1 Hz | 45,1 Hz |
| `/tf` | 1054,5 Hz | 144,6 Hz (−86,3%) |
| `odom <- lidar` available | 94,75% | **99,94%** |
| module load | 26,90 | 18,52 |
| `nav2_container` | 298% | 240% |
| `maze_explorer` | 67,6% | **76,0%** |

* * The attribution of the edge was closed: `odom <- base` and `odom <- lidar` gave the
exact same number in both arms. The compound chain loses nothing but what the dynamic
edge loses. And the sampler of 200 Hz gives the mechanism: in the arm The maximum
between separate stamps was 120 ms in a 20 ms publisher — five cycles lost at once. It
was not delivery burst; it was the `odom_tf` not being staggered in time to stamp. This
also corrects the morning assignment (100,00% / 95,30% / 78,11%), which
compared three separate executions of different durations: the qualitative conclusion
was right, the numbers were not comparable with each other.

* *One of my hypotheses was REPROVADA and is registered as such.** I had said that the
~68% of an IDLE `maze_explorer` were its `TransformListener` deserialising
1090 messages per second. With the 86% smaller flow it ROSE, to
76,0%. The plausible explanation is starvation — with the load dropping from
26,9 to 18,5, a previously disputed knot turns at ease — but this is hypothesis, not
measurement. Profile his threads (Method of `ml35-f5-clock-fanout.md`) is the next step
if the goal is CPU.

**`map_update_interval` returned to 1.0**, in an independent round. He had gone to 5.0
in this same session by economy of CPU; the economy was measured and did not exist
(31,4% → 30,3% in `async_slam_toolbox`, within noise). In return the cost is 1 pp,
symmetrical — small variation and operationally irrelevant, in both directions.
`/map` climbs from 0,2 to 1,000 Hz and `static_layer` stops staying until 5 s
behind the wall SLAM already knows.

* *Gate gate, 3 racing 180 s in `maze11-short`:**

|  | 1 | 2 | 3 |
| --- | ---: | ---: | ---: |
| goals achieved | 9/10 | 8/10 | 9/10 |
| first three | Okay ok ok ok | Okay ok ok ok | Okay ok ok ok |
| peak tilt | 1,17° | 1,06° | 1,18° |
| minimum clearance | 0,448 m | 0,448 m | 0,448 m |
| average speed | 0,0383 | 0,0342 | 0,0447 m/s |

Limited log scan, 12 min: **zero** `worldToMap`, **zero** `invalid source`, **zero**
extrapolation of TF. Five out of six criteria pass.

**The GATE CONTRACT WAS SPLIT, and that's what closes this session.** The
limit of 0,05 m/s came from a TRAVESSIA test and was being charged with targets
separated by 0,5 m, where the average route includes acceptance, acceleration,
deceleration by the goal checker, reacquisition and the return route of the recycled
sequence. This measures stability and latency of goals, not crossing. The correction is
not to lower the limit until it passes — it is to separate:

- ** Short port of ESTABILIDADE**, charged from `maze11-short`: three goals
  `SUCCEEDED`, each within 45 s, zero Nav2 errors, zero extrapolations, `worldToMap`
  and `invalid source`, zero falls, zero major route changes. **APROVADO** — Worst
  target 27,9 s, and the durations per target (send to send) are 12,4/4,8/5,1,
  16,5/15,8/5,8 and 27,9/21,3/5,3 s;
- **port of DESEMPENHO crossing**, with targets separated by at least the
  horizon of MPPI, or preferably the autonomous exit itself in 600 s. The limit of
  0,05 m/s still goes there, untouched. ** NO EXECUTADO. **

`maze11-short` (0,0383 / 0,0342 / 0,0447 m/s) are registered and ** are not a
criterion**. They are also not regression: the baseline of Maze11 in `gait_go2.yaml` is
0,0399 and these give average 0,0391. What has improved is the working ratio in vx,
from 6,2% to 15,6–22,3% — the robot spends two to three times more time
with effective advancement, and this has NOT become average speed. It is exactly the
distance between the limit of MACHINE, which this session attacked and closed, and the
limit of DECISION DE TRAJETO, isolated in `ml35-f5-clock-fanout.md` and still standing.

The 2 race's deadline has fallen on the QUARTA goal, already in recycled sequence,
outside the three-party contract. It remains a sign of variability — one in thirty goals
closed — and does not invalidate the short gate.

* *The march was verified by these same three races: ** peak tilt 1,06–1,18°, carcass
clearance 0,448 m, zero falls, zero `cmd_vx` negative. Deciding the broadcaster didn't
degrade the floor.

**`Control loop missed` is metric, not blocking.** The track from 8,6–10,4 Hz alone does
not report frequency or severity. The farm smoke shall record total warnings, warnings
per minute, greater consecutive sequence and correlation with zero stop or command. Only
profile `nav2_container` and `maze_explorer` if there is sustained sequence below the
desired frequency WITH correlated stops.

**The `maze_explorer` to 76% idle does not justify profiling now.** 76% of a core is
cost, not functional failure, and idle value is not the right measurement of Step 4 —
the extraction of borders only runs in the state `selecting`. The right measurement is
the smoke. Profile only if it shows extraction above 100 ms, controller losing cycles
continuously, exploration without selecting new frontiers, TF regressing, load
preventing perception, or exit time incompatible with 600 s.

* *Next real lock: perception in Aquila.** Confirm with limited capture:
`/demo/camera/camera_info` reaching the module, image effectively processed, detection
in at least 3 of 5 frames, exit pose in the correct frame,
detector CPU, TF remaining ≥99,5% and controller without material degradation. If
the detector CPU interferes, increase **only** `sample_stride` and repeat — do not touch
the MPPI together.

* *Agreed sequence until closing: ** Contract/documentation → perception → exploitation
smoke → diagnosis only if smoking fails → three cold matches → final report and
cleaning. Do not increase `vx_max` before this: the carcass clearance of ~6,5 cm remains
small.

* *New armadilla, which cost a whole race of 180 s.** Shortly after recreating the `sim`
container, `/clock` appears in the graph but does not deliver a message to a subscriber
NOVO for a few minutes. Any script with `use_sim_time: True` that climbs into this
window reads a stopped clock: RTF 0,000, cloud age −240 s, 100% of
stamps "in the future". The columns that did not use the knot clock remained valid, but
the race was discarded and redone. Before measuring, confirm real delivery (`ros2 topic
hz /clock`), not presence in the graph — and note that the map of SLAM is `/map`, not
`/demo/map`.

### 28/08 — autonomous search implemented; stability gate still REPROVADO

Evidence: **`docs/results/ml35-f5-busca-autonoma.md`** (`PENDING EXECUTION`).

The autonomous output demonstration of the labyrinth is**implemented from end to end and
installed**, and **no acceptance race was performed**. The two sentences are valid at
the same time, and the second is the one that decides if the phase closes.

**Close (host):**

| Ask. | Where it spins |
|---|---|
| `frontier.py` + `maze_explorer` (borders, blacklist, deadlines, JSON) | module |
| `ExplorationGrid` (`allow_unknown: false`) + `nav_to_pose_exploration.xml` | module |
| `maze_exit_detector` (magenta panel, confirmation 3 by 5) | module |
| magenta panel on `quadruped_maze11.sdf` | host |
| `maze_escape_validator` → `/demo/maze/escaped` | **host, only simulation** |
| buttons and HUD search in cockpit | cockpit |

Host suites: contract **150**, `demo_navigation` **25**, `demo_perception` **33**,
cockpit **169**. None of them measure navigation.

* * The gate remains the lock, and it failed. ** Last race
(`artifacts/maze11-short-gate.csv`): 37,1 s, 0,69 m, **0,0185 m/s**, `cmd_vx` non-null
in 18,7% of the samples — **below the floor of 0,05 m/s**, and without
3/3 targets. The previous race, before `restamp_tf: true`, had the
robot**frozen** (`cmd_vx` zero in 150 s). The parameter unlocked the command and **did
not close the gate**.

`restamp_tf` was verified as a real parameter of Jazzy `slam_toolbox`
(`slam_toolbox_common.hpp:177`, and `restamp_tf: false` in the five
`mapper_params_*.yaml` of `/opt/ros/jazzy/share`) — it is not YAML ignored silently.
`transform_timeout` is at 0,2 as required.

**`nav_trial.py` started archiving the evidence by goal.** Before the outcome of each
action died in the stdout and the goal in flight at the end of the trial was never
recorded — a gate of 3 goals reported 2. Now comes out a CSV brother `<csv>-metas.csv`
with target, outcome, `status`, `error_code`/`error_msg` of Nav2 and route
changes **for that** goal, and each telemetry sample carries `goal_index`.

* * Open risk that precedes any conclusion about perception: ** Camera RAW no longer
crosses the wire since `ml35-f5-camera-comprimida.md`. `SetRemap` by
`demo_bringup/launch/perception.launch.py` automatically reconnects the detector image,
but `/demo/camera/camera_info` is not remapped**. Without it the detector publishes
detection and never publishes pose — silent failure. Check `ros2 topic hz /demo/camera/camera_info` **in the
module** before blaming the vision.

### 27/08 (Part 3) — mechanism found: the global plan alternates at 1 Hz. `clearing: false` REJECTED

Evidence: **`docs/results/ml35-f5-memoria-costmap.md`**.

* *Causes root, reading `/plan` every 5 s in a stuck target (0,0) → (0,8):**

| t | length | initial course |
| ---: | ---: | ---: |
| +5 s / +10 s | **11,49 m** | 173° — true route |
| +15 s / +20 s | **8,59 m** | 89° — cross wall not seen |
| +25 s / +30 s | 8,66 / 8,81 m | 35° / 18° |

The 11,5 m match the offline geodesic (12,23 m). The 8,6 m only exist because
`allow_unknown: true` makes the unknown cheap. **The MPPI receives a path that reverses
90–180° every second** — then rotates without translating. Completes the finding of
part 2: there it was proven that the symptom disappears with good goal; here is the
mechanism by which the bad goal produces it.

**A failed experiment — do not repeat.** `clearing: false` on the global `obstacle_layer`
("memory map") improved margin (movement 0,19 → 1,04 m, working ratio 0,0% →
3,9%) and **did not move the mechanism**: the plan continued alternating
11,66 ↔ 8,77 m, zero goals. Reading `costmap_raw`, with it enabled **150 of 161
straight cells to the goal were left in 255 (unknown)**.

`clearing` is not "forget obstacle" — it is raytrace, and raytrace is the only mechanism
that makes LIVRE unknown in this layer. Turning it off leaves the map permanently
unknown and makes the shortcut More attractive. Correction worsens the cause it attacks.
Locked by `tests/test_module_params_mount.py`.

* * What solves** is to keep busy * and* free, and the obstacle layer has a button only
for the two. It is a map of `slam_toolbox` on `static_layer` (already defined and
inert), with `allow_unknown: true` maintained. Real lock: `pointcloud_to_laserscan`
requires **rebuild arm64 native in the module**.

**Infrastructure delivered:** `compose.module.yml` mounts
`ros2_ws/src/demo_navigation/config` over `/ws/src/demo_navigation/config` (the **final target of the
symlink**, not the path installed — mounting on the installed would be quiet). **
Parameter in the module came to cost `sync`, not `build`.**

* *Open Anomalia:** the global costmap brand first cell ≥ 253 to **0,55 m in +y**, where
`maze_fit.py` measures **3,47 m freeway**. Measure before running SLAM — a persistent
map would inherit the error definitively.

### 27/08 (Part 2) — PROVEN in HIL: 8 of 8 goals met, working ratio 0,0% → 37,5%

Evidence: **`docs/results/ml35-f5-rota-conectada.md`** + the two CSVs next door. A/B
with minutes apart, real **HIL** (Nav2 on the Aquila AM69), same images, same
parameters, nothing rebuilt. **Only variable: the geometry of the goal.**

| metric | route connected | control — patrol (0; 8) |
| --- | ---: | ---: |
| ** Working Reason `vx`** | **37,5%** | **0,0%** |
| `cmd_vx`  | 12,9% | 98,6% |
| liquid displacement | **7,11 m** | 0,19 m |
| path efficiency | 57,2% | 16,2% |
| **mets completed** | **8 of 8** | 0 in 120 s |
| liquid drift from yaw | **+1,1°** in 240 s | **−186,8°** in 120 s |

The working reason has never gone beyond 8,6% in any condition tested in this project
(CPU, `/clock`, MPPI sampling, compressed camera, Ethernet, BT correction). It was
37,5% without touching anything but the goal. The 57,2% efficiency reproduces the
57% measured on the host on 21/08 — the module has always been able to do so.

* * The unidirectional spin of §10 is not a controller defect. With valid plan the
`cmd_wz` alternates 45,2% / 51,3% and the drift is +1,1° in four minutes. With meta
behind wall back to be unidirectional — **and with the inverted sign** in relation to
§10, which kills the family "critic asymmetry" / "sign error in yaw":
signal error does not change signal.

* *Consequence to the gate. ** "Goal Nav2 `SUCCEEDED` with the leg robot, Nav2 in the
module" was completed eight times in a race**. What I failed was the 8 m protocol on
patrol targets. The F5 gate has to be rewritten on connected route or on persisted map
before being charged again.

**A real deficit remains, now measurable:** 37,5% and 0,0454 m/s average are still below `vx_max` 0,15
m/s. Critical tune only makes sense from here.

* *New armadilla:**Every target of the Maze11 has negative `x`, and `--goals -1.50,...`
is read by the argparse as flag — the script prints `usage` and exits **0**. With
`2>/dev/null` turns silent race that does nothing. Always use `--goals=`. Documented in
`nav_trial.py` itself.

### 27/08 (Part 1) — the test targets are behind the wall; the global plan crosses the wall

Evidence and numbers: `docs/ml35/proximos-passos-navegacao.md` §11. Measure **offline**,
without bench, without ROS and without Gazebo — read only the STL of the Maze11, with
the new tool `scripts/maze_geodesic.py` (6 guards in `tests/test_maze_geodesic.py`, three
verified by mutation).

* *The four `MAZE11_GOALS` have a straight wall. The geodesic for navigable space is
1,53× the 4,22× the straight, and from spam the robot sees, with occlusion, **19,6%**
the free space within the 8 m of `obstacle_max_range`.

With `global_costmap` escalator **without `static_layer` and without map** and the NavFn
in `allow_unknown: true`, the plan of these goals crosses unobserved wall — `SUCCEEDED`,
beautiful path in the RViz and in the cockpit, **zero error or log**. Corroborates with
data already in the repository: the path measured in §8 had ~7,0 m for a goal
whose real route is 12,23 m and whose own straight is 8,00 m.

**Method consequence:** §§7–10 measured MPPI with an invalid entry. This does not reopen the
five refuted hypotheses of §1, but no conclusion about critics survives — the critical
off test goes down from priority.

* * The ordination that ¢Ü9 searched for bearing and did not find** is by distance to
the first wall on the line: 0,96 m → 0,00 m displacement; 3,88 m → 0,07 m;
3,90 m → 0,10 m. The cut falls on the horizon of MPPI (1,44 m).

* *Next step, cheap and decisive, in the host, without touching the image: ** Rotate the
connected route of `maze_route.py` (0 of 9 legs with wall on the straight,
100% visible in all, against 4 of 4 blocked on patrol). Command ready at §11.

* *Persisting the map — operator's request — is not a new front: it is connecting what
is already in the tree.** Go2 `static_layer` is already set and inert with the procedure
next door, and the diff-drive path already navigates over `maps/warehouse.{pgm,yaml}`.
The two real locks: `slam_params.yaml` has `base_frame: base_link` (parameter,
non-architecture) and `slam_toolbox` consumes `LaserScan`, while Go2's `/demo/scan` is
the degenerate ring that delta 3 of `nav2_params_go2.yaml` has already measured as **zero
obstacles**. The good data is `/demo/scan_cloud`, and flattening it is
`ros-jazzy-pointcloud-to-laserscan` (touch, 2.0.2 in Jazzy's apt, not yet in any image).
**No AMCL** in this topology: Gazebo's odometry is true of terrain and `map`→`odom` is
already the identity of `odom_tf`.

### 26/08 (late) — cockpit reset, target telemetry, campaign protocol

Full evidence: `docs/results/cockpit-reset-nao-destrutivo.md`.

* * Closed serious defect: the cockpit reset button erased the robot. `/demo/sim/reset`
used `ControlWorld.reset.all`, which returns the world to SDF of origin — and the robot
and the two scene cameras are INSERIDOS after loading (`ros_gz_sim create`), so they are
not on it. Measured: `/joint_states` 999 Hz → dead, `/demo/imu` 996 Hz → dead,
`/demo/odom` 49,6 Hz → dead, `gz model -m demo_robot` → `No model named <demo_robot>`.

The failure mode was the worst of this project: clock followed 999 Hz and orphaned
sensors at 10 Hz, then **The cockpit became whole green pointing to a nonexistent
plant** without a log line. Recover required rebooting of `sim`.

Now the reset TELEPORTA the robot for the birth pose of the scenario, by the same
`/demo/sim/set_entity_pose` that the cameras already used. Verified: robot lap from
(2,0; −1,5) to (0,00003; −0,010), `/joint_states` 1000 Hz, `/demo/imu` 974 Hz,
`/demo/odom` 49,9 Hz, and the five models follow in the world. The clock **does not**
return to zero, on purpose: a jump backwards would invalidate Nav2's TF buffer and
`controller_manager`.

> "Resent alone" was written here and was false — corrected in session
> Next, see below.

* *Telemetry of the target in the cockpit, measured in the actual AM69. `target_monitor`
publishes `/demo/target/status` (CPU, memory, temperature, load) and
`/demo/target/ops_log` (axes commanded in SI, stick and odom, in short text).
The log panel no longer depends on raw `/rosout` — `/rosout` remains subscribed only as
filtered reservation for warnings and errors. Temperature checked against sensor:
34,974 °C reported against `thermal_zone1/6` reading 34498 thousandths at the same
time; the seven zones between 32,1 and 34,5 °C. Cost of the node: **4,3% of a core** in
800% available (`use_sim_time: False` keeps it cheap — it does not sign `/clock`).

**The screens survive the two restarts.** Probe with the rosbridge client of the cockpit
itself, a panel subscriber: `sim` restore and restore of the application on the target
do not lose any panel, and WebSocket does not fall (in this topology `cockpit` and
`hmi` run on the host). The only zero is `/demo/cmd_vel_si` without active target,
which **is not a defect** — Nav2 came up `Managed nodes are active` and `velocity_smoother` only
publishes after the first target; the new channel says so in text.

One hypothesis was tested and discarded**: reconnect the `<img>` of MJPEG after the
publisher returns. Measured with `curl` in the same response HTTP through a restort of
`sim`, bytes grow unbroken (1,01 MB → 4,61 MB).
`web_video_server` keeps registration and response open. Don't spend code on it.

* * Campaign protocol delivered: `scripts/nav_campaign.py`** It is the missing link
between `nav_trial.py` (a race) and `summarize_trials.py` (resume replicatas): decides
ORDEM and what happens between legs. Intercala `A B A B A B` instead of blocking, and
replace robot and costmap before each leg. **It is only possible because of the repair
of the reset above** — a campaign that called the old reset between legs would measure,
from leg 2 onward, a world without robot and with nothing accusing. Ten
guards in `tests/test_nav_campaign.py`, including the reverse (the blocked order has to fail) and the
fact that `ros2 service call` exits 0 even with `success=False`.

45 s smoke leg to prove the loop: work by `cmd_vx` **0,0%**, `vx` It reproduces the
symptom; n=1 and 45 s do not decide anything.

**Two things this session didn't do:**

- * *The cockpit was not opened in a browser.** There is no Chrome on this machine and the
  MCP automation does not drive Firefox installed. Everything above was measured in
  the data path. The visual gate is still pending from manual pass.
- **Manual control (F4) is not implemented**, and there is a new lock and
  concrete: `twist_mux` ** is not in any image** and the module ** has no default
  route** (only the `<LAN_CIDR>`), so `apt` does not solve anything there. LAN
  `<LAN_GATEWAY>` gateway responds in 0,337 ms and the module already has
  corporate DNS — only `sudo ip route add default via <LAN_GATEWAY> dev
  ethernet0` is missing, which needs to be run by those who have permission.

* *Next gate, in order requested by the operator:** (1) default route in module and F4;
(2) real A/B campaign with `nav_campaign.py`, n

### 26/08 (night) — reset did not reset itself: robot collapsed or dragged

Full evidence: `docs/results/cockpit-reset-nao-destrutivo.md` §3.1.

Reported by the operator: after reset the robot made turn back to the previous
orientation. Investigated with the robot in motion — not stopped, the only case tested
in the afternoon session — and found DOIS defects, both silent:

- **Teleport without reanchoring the gait**: `StateTrotting` (C++ controller) capture
  his posture reference (`pcd_`, `yaw_cmd_`) once, behind a lock that only a clean
  walk command. With `/demo/cmd_vel*` zeroed out and the target canceled — to exclude
  Nav2 as a cause — the robot still dragged 0,87 m and turned 135° in 26 s
  without any published commands;
- **Teleport non-stop**: `SetEntityPose` preserves speed. With a flow
  from `/demo/cmd_vel` alive to 10 Hz during reset (the real case, with nav2 driving),
  the robot COLAPSA — `z` by 0,337 m for 0,162 m in 1 s — and is writhing 40 s.

Fixed with two new services in `twist_to_inputs` (the only `/control_input` writer):
`/demo/gait/hold` (trotting → fixed stand, immobile robot, `GAIT_STOP_S = 2,0 s`
waiting) called BEFORE the teleport, `/demo/gait/resume` (fixed stand → trotting,
`StateTrotting::enter()` reanchors `pcd_`/`yaw_cmd_` in the new pose) called AFTER. Best effort: in a
differential plant the two services do not exist and this is normal way — but the
absence enters the message itself of the `Trigger` reset, never stays silent.

Verified repeating the case that failed (living command to 10 Hz during reset: robot
never leaves 0,35-0,36 m in height, again obeys the same command after reset, a
second reset re-arm. **Checked also with the robot CAIDO** (tombed 180°, stuck in
`mode=RECOVER` with `tilt=131°` — this mode does not come out upside down alone): the
reset recovers it standing, `mode=HOLD`, `tilt=0,2°`, `yawSat=0%`, and it goes back to
walking normally.

New guards in `test_sim_reset.py` and `test_twist_to_inputs.py`: the order to
stop→teleport→retake has to be in this sequence at the source, the drop command (`2`)
leaves exactly once, the hold has no time limit, and every way out of the reset handler
— including those of error — has to resume the gait.

### 26/08 (early) — where to resume

Full evidence: `docs/results/ml35-f5-clock-fanout.md`.

* *The item 1 of the previous session (CPU of the module) is FECHADO.** Do not reopen it
by the old path: it is not the MPPI loop (refuted in `ml35-f5-mppi-amostragem.md`) and
it is not the `/clock` rate (refuted on 21/08 — throttling killed
navigation).

* *It was `/clock` subscription fan-out.**Profiling `/proc/<tid>/stat` by thread, the
`nav` container spent **367% of 800% with the robot STOPPED**, and 111% of this were
three republishers in Python — `odom_tf`, `cmd_vel_si_to_stick` and `nav_control_relay` — which **do
not call the clock once** and subscribed to `/clock` at ~870 Hz just because `use_sim_time:
true` makes rclpy create the signature.

Fixed with `use_sim_time: False` on all three. The three fell from **111% to 17,7%**,
and `ros2 topic info /clock -v` confirms that none of them sign anymore. Four
`demo_bringup/test/test_sim_time_scope.py` tests, verified by mutation, lock the
invariant in both directions — including the reverse, which is what matters: **No
a node without `use_sim_time` cannot call `get_clock()`**.

* * What it bought, measured: *

|  | before | later |
| --- | ---: | ---: |
| refusals `Ignoring the source` | 16 | **0** |
| `Robot to stop due to invalid source` | 4 | **0** |
| costmap discards | — | **0** |
| machine clearance under navigation | none | **307% of 800%** |

* * What it didn't buy: movement. * The confirmation race gave 0,0246 m/s, `vx` at zero
at **90,7%** of the samples, rotating in **90,3%**, and **0 of 2 goals of 8
m**. Inside the known noise range. Zero falls.

* *So the item 2 of the previous session — decision of the route — is now alone and
without confusion.** With CPU remaining and without a single sensor refusal, the robot
continues to rotate rather than relocate. This was no side effect of CPU.

**Two traps discovered in this session**

- `docker/.env` still loads `MODULE_IP=<MODULE_IP>`, old address of
  bench, and he beats the defaults**. `ssh` works like this because it uses the mDNS
  name, so the error only appears in `sync`/`build` (`não identifiquei a interface do
  módulo que carrega <MODULE_IP>`). Pass explicit `MODULE_IP=` and `HOST_IP=`, or fix
  the `.env`.
- One hypothesis was tested and ** discarded**: `inflation_radius` 0,55 in a corridor
  from 1,20 m would leave 10 cm of free track and would turn cheaper than advance.
  Measured in `local_costmap`: **64,1% of cells at cost 0**, and what was ahead was
  real wall. Don't spend tune on this without measuring again.

* Next gate, in order:

1. * * Fix protocol before tuning. ** n ≥ 3 by condition,
   interspersed, median and range. The dispersion of 2,4× in identical configuration
   remains valid and no single race decides — this session included.
2. ** Change the primary metric** for `cmd_vx` working ratio and fraction of
   `vx`  They gave 0,6% and 0,6% in two separate races, against 2,4× of medium
   velocity dispersion, and measure the symptom directly.

* *Instrument delivered on this resume:** `scripts/nav_trial.py` now prints and records
the test with both metrics, using fixed bands
`|cmd vx| <= 0,005 m/s` (quase zero) e `cmd vx > 0,05 m/s` (work for
front). `scripts/summarize_trials.py` summarizes replications by condition without
grouping samples, showing `n`, median and range observed. The summary reproduces
historical numbers (RAW 5,8% working; compressed 0,6%), so the new definition does
not change the baseline.

The native rebuild of the arm64 image was completed in Aquila and the containers were
recreated. `module.sh verify` returned to **3/3**, and the `route_server` log of the new
image contains only `AdjustSpeedLimit` (does not contain the old `ReroutingService`). The n≥3/A-B trial remains deliberately pending on budget: each leg lasts up to 420 s and the full campaign requires six legs.
3. **Only then MPPI** (`PathAlignCritic` 14,0 × `PathAngleCritic` 2,0), now in
   clean test, with costmap measured at each condition.

### 25/08 (night) — where to resume (read this before touching anything)

Full evidence: `docs/results/ml35-f5-ethernet0-repeticao.md`. Previous orientation of
this section (fix PHY, switch cable, measure later) ** It has been accomplished and is
unsuccessful** — do not repeat it.

* *The link is resolved and proven. `enp0s31f6` to 1000 Mb/s full, host `<HOST_IP>` ↔
Aquila `<MODULE_IP>` by `ethernet0`, RTT 0,400 ms, symmetric route in both
directions, `scripts/module.sh verify` returning **0** with the three steps.
The asymmetry disappeared structurally: Wi-Fi remained in metrics 600 versus cable 100,
so the entire `/24` prefers cable.

* * The 8 m gate continues REPROVADO, and the network is not the cause.** The
`ethernet1`/`ethernet0` hypothesis that was open here is **refuted by measurement**:
with correct route and clean link, the two goals of 8 m burst the same deadline.

* * The two causes measured in order of size:**

1. **CPU of the module.** Nav2 alone consumes **600–727% of 800%** on the AM69. The
   perception adds ~187% and passes capacity. Then the coolness of the sensor
   collapses: the `collision_monitor` refused the LiDAR cloud 16 times
   with 1,0–1,2 s lag. With the idle Nav2 this lag is 42 ms — that is, it is line by
   containment, **no** transport. Measured cost: **2,8×** at average speed (0,0429 →
   0,0155 m/s).
2. **Decision of route.** In the complete HIL the robot has `vx` at zero at **79% ** of
   samples and rotates in **93,9% ** of them: he passes the essay by spinning instead
   of transferring**. Without the camera the standard relieves but does not
   disappear, and the cost migrates to the route — 18,00 m of path for 5,20 m
   liquids, **28,9% of efficiency** against 57% in the host. It matches the already
   registered hypothesis of `PathAlignCritic` 14,0 against `PathAngleCritic`
   2,0, which **remains without a correction test**.

* *Getting the wire camera doesn't make the goal pass. It was measured: 0,0429 m/s and
yet 0 by 2 targets. It's two independent limits, and only one is CPU.

* *Stability: ** Zero falls in both races, but the peak tilt goes from 0,94° to
**15,66°** precisely in the race where the robot runs. The low value of the complete HIL
describes a robot almost stopped, not a stable robot. Casting off follows in **+6,5
cm**.

* * 2 and 3 of protocol n=3 were not executed** — 1 failed and the mechanism was
identified; repeat would spend bench without new information.

* *A closed method trap in this session:** `verify` disapproved by `/clock` absent from
a module that reads `/clock` at 616 Hz. Stage 2 collected with `grep /demo/` and then
required `/clock`, which is not under `/demo/`. The test that existed passed all the
time because it only checked if the string appeared in the file. Corrected, with
mutation failure test.

**PHASE 2 OF THE PLAN HAS ALREADY BEEN EXECUTED (25/08, night).** The
compressed camera is implemented, validated and measured: `docs/results/ml35-f5-camera-comprimida.md`. It delivers to
engineering (~82× less yarn, CPU module ~711% → ~600%) and **does not move the gate** –
speed has not improved reliably and the working reason has worsened (2,1–2,7% vs 5,8%).
The `collision_monitor` continues to refuse the cloud with ~1,0 s of lag and emitting
`Robot to stop due to invalid source`. The dominant variable is the **presence** of
perception, not the format of transport: with perception in the module the ratio is 2–6%
in any format; without it, 16,8%. Do not repeat phase 2 and do not discuss image format
again.

**PHASE 1 OF THE PLAN HAS ALSO ALREADY BEEN EXECUTED (25/08, night) AND FAILED.**
`time_steps` 96→64 with `model_dt` 0,10→0,15 (constant horizon, 33% less sampling) **
did not reduce CPU**: ~437% → ~439%. The cost of MPPI here is not dominated by
`batch_size × time_steps`. Evidence: `docs/results/ml35-f5-mppi-amostragem.md`. Not
adopted; YAML back to baseline with A/B out of default path.

**READ THIS BEFORE RUNNING ANY NEW TRIAL — the current method does not
decide.** Two races in the **identical** configuration gave **0,0109 and 0,0265
m/s**, dispersion of **2,4×**. The noise between races is greater than the desired
effects, so **A/B of n=1 on this bench is ininterpretable**. Before tuning anything: n ≥
3 per condition, interspersed, median and reported range.

* * Falls are no longer variance:** 2 in 3 races after camera compressed, against 0 in 2
before. No mechanism identified and no cause demonstrated, but it is research item, not
footnote — stability is prerequisite of any goal.

* * Order suggested by the data for the next session: ** Reduce Nav2 CPU in the module →
take the RAW image from the wire (compressed transport to perception) → only then touch
the MPPI → repeat 420 s / 200 s with n=3.
---

* *Take decision: target changed from A1 to Go2** (see "F2 — verification executed";
license justification given on F2 was incomplete and corrected on F3 — see "F3 — license
tracking"). **F3 rotated and the gate crashed**: Go2 standing, stable, walking by
`/demo/cmd_vel` with the packages and the project launch, no longer with the spike. HOLD
failure of F4 was corrected on 20/08 and the full perception contract was
revalidated on 24/08. Evidence of gait in `docs/results/ml35-postura-parada.md`
and closing below.

Current movement plan: **`docs/ml35/plano-movimentacao.md`** (19/08/2026). The
previous phase plan 1–3 has been removed because it is fully superseded; its
results remain in `docs/results/ml35-f4-parcial.md`.

Open parallel work — ** Unified cockpit**: plan approved at 24/08/2026 and **F1
completed on the same day**. Axle changed from "capture X11" windows (four failed
attempts) to "render from ROS 2 topics" on a web cockpit that then turns HMI from
the M3. Decisions, evidence and phases in **`docs/ml35/plano-cockpit-web.md`**;
evidence from F1 (screen captures, fees, reconnection) in
**`docs/results/cockpit-web-f1.md`**. The previous checkpoint
(`docs/results/cockpit-standalone-parcial.md`) is marked as overwritten; do not resume
his recommendation.

Cockpit status by phase: **F1 and F3b closed** (24/08/2026). F1 has risen the services
`cockpit` and `hmi` in `compose.host.yml`, the bundle in `hmi/`, the live
camera and automatic reconnection. **F3b** closed the blue panel (two static, alternable
scene cameras) and the green one (costmap, plane, laser, footprint, and click sending
goal), with the gate completed: a target clicked on canvas was accepted and executed by
Nav2. Evidence on **`docs/results/cockpit-web-f3b.md`**; how to run and what each panel
does in **`docs/guia-completo.md`** (Part II).

On 25/08/2026, three **UI adjustments** requested on the bench, outside of the
phase numbering and without opening new phase: double Toradex brand, scene cameras
following the robot in both views, and "restart nav" from the cockpit. Evidence in
**`docs/results/cockpit-web-ui-ajustes.md`**. A finding with its own weight came out:
`RESET`+`STARTUP` on Nav2's `lifecycle_manager` **drops the container** with `SIGSEGV` while
setting up `route_server`, played twice — so reset uses `PAUSE`/`RESUME`. Candidate for
upstream issue; see 8 trap of Part II of `guia-completo.md`. **Next to the cockpit is
F4** (manual control behind the `twist_mux`).

On the same date they entered, at the operator's request: control of simulation by
cockpit (play/pause/reset), camera control (turn, tilt, move, zoom, refocus), visual
identity Toradex (white background, `#00508c`, `#96c837`, `#ff5a00`, with the brands
Toradex and ROS in the bar) and the increase in the quality of the scene cameras. Three
points worth loading for the next session:

1. **"Start simulation on target" is not possible** and was not done. The Gazebo
   is OGRE 2; AM69 only has OpenGL ES 3.2/Vulkan 1.2 (rule 1). What exists is play/pause/reset **from the** cockpit, acting on the host's Gazebo.
2. * * The browser does not speak Gazebo types.** Call `ControlWorld` direct by
   rosbridge fails with `InvalidModuleException` — the cockpit container does not
   have `ros_gz_interfaces`, and the M3 runs in the module. The border is `std_srvs`,
   and the translation lives on the `sim_control_relay` node, next to the simulator.
3. * *Camera resolution costs RTF.** With the two scene cameras at 1600x1200,
   `update_rate 15` delivery 9,43 Hz with real-time factor **0,59**, and `update_rate
   10` delivers 9,77 Hz with **0,97** — asking for 15 does not yield an extra
   frame and costs 40% of the speed of the simulation. Adopted 10. See section
   5 of `cockpit-web-f3b.md`.

None of this was executed in arm64 or Aquila. **F2 and Cockpit F4 remain open** (kiosk
in module and manual control with `twist_mux`).

### 21/08/2026 — navigating quality in the maze11 (within F5)

Scenario S6 passed to **`maze11`**, starting in the lower right corner
(`docs/results/ml35-labirinto.md`). Next, Nav2's decision-making quality was measured
and corrected: **0,0399 → 0,0650 m/s (+63%)**, reverse **11–62% → 0%**, route
efficiency **13% → 57%**, and the **first goal accomplished** (8 m in 96 s).
Evidence and limits in **`docs/results/ml35-navegacao-maze11.md`**.

Three faults, all decision and no sensor:

1. `vx_min: -0.10` produced **deadlock**: the robot retreated, leaned against the wall and back
   It was still great. Measured in 100% samples with the robot stopped at 0,00 m. Now
   `vx_min: 0.0`, with `wz_max` 0,12 → 0,20 for the spin to be a real
   alternative.
2. * *No behavior tree of the Nav2 Jazzy calls `SmoothPath`**, then the
   `smoother_server` was active and idle and the MPPI was pursuing the raw ladder of
   the NavFn. Now there's `demo_navigation/behavior_trees/nav_to_pose_smoothed.xml`.
3. NavFn chose route by length. Costmap inflation **global** went to
   0,85 / 2,0 — diverge from the site on purpose in the safe direction.

Handle and odometry were verified on request and **are sound** (odom vs TF error with
0,0000 m; no lidar self-collision). New tools: `scripts/sensor_check.py`,
`scripts/costmap_probe.py`, `scripts/selfhit.py`.

* * Not closed: ** Carcase clearance follows in **+6,5 cm** and is the gate of any
future speed increase. It comes from `robot_radius: 0.38` model the trunk as a circle;
the correction is polygonal footprint with `consider_footprint: true`.

### 21/08/2026 — HIL standing on the Aquila AM69 (within F5)

* *The application runs in the module. ** Nav2 arm64 active in Aquila AM69, composed in
a single process, host simulator, DDS bidirectional link verified. Build arm64 **Native
in module**, 1 rule verified in the four images. Evidence and limits in
**`docs/results/ml35-hil-aquila.md`**.

* *The module is not the bottleneck. The bottleneck is the camera stream of **74,2
Mbit/s** (640×480 rgb8 to 10,1 Hz, measured on the wire) crossing the Wi-Fi:

| Condition | Module | Camera on wire | Medium vel. |
|---|---|---|---|
| host-only, DDS multicast default | stopped | no | 0,0720 m/s |
| host-only, DDS by HIL | stopped | no | **0,0725 m/s** |
| HIL, Nav2 + perception | active | Yeah. | 0,0197 m/s |
| HIL only Nav2 | active | no | **0,0427 m/s** |

The CycloneDDS configuration with explicit peer ** costs nothing** — hypothesis raised
and refuted. Nav2 in the module costs 1,7×; the camera costs other 2,2×.

**Composition of Nav2**: `nav_quadruped.launch.py` started creating `nav2_container`.
Memory of container `nav` **6,89 GiB → 307 MiB**, load **21,9 → 9,5**,
activation at **~10 s**. Total CPU has not changed.

**Strangulate `/clock` was tried, measured and reversed**: the 100 Hz CPU dropped from
470% to 324% and navigation died (0,0039 vs 0,0251 m/s). The node stays in the
package with A/B in the header, out of default path.

* * Not closed, located: ** The working ratio of `cmd_vx` is low in both machines —
normal peak (0,10–0,14), medium 0,006–0,008. The start of the Maze11 requires spinning
stop of ~85° and the MPPI commands `wz = 0,035` rad/s, 17% of
the ceiling. Control loop, TF, costmap and `collision_monitor` were discarded by
measurement. Unmeasured hypothesis: `PathAlignCritic` in 14,0 against `PathAngleCritic`
in 2,0.

**Operator decision on 24/08/2026:** preserve 640×480 at 10 Hz and migrate HIL
to Ethernet. F5 only closes after the real race in this link; do not infer the result
from the band measured in Wi-Fi.

### 24/08/2026 — HIL Ethernet executed, long gate still open

The link was actually executed: Host `enp0s31f6` and Aquila `ethernet1`, with
`<HOST_IP>` and `<MODULE_IP>` peer cycloneDDDS. The two Aquila ports in the same subnet
announce the same hostname mDNS; leaving `MODULE_IP` implied switched between the two
addresses. The local configuration now fixes a port before `module.sh sync`.

Two QoS defects only appeared with fragmented samples in HIL. RAW camera from 921600
bytes needed a `RELIABLE` reader; the LiDAR cloud needed a `SENSOR_DATA` producer for
Nav2's `BEST_EFFORT` readers. After the two fixes, camera, detections and detection cloud
flowed at ~10 Hz, and `collision_monitor` stopped rejecting commands by old
font.

A short goal closed `SUCCEEDED` in **28 s**, with Nav2 + perception in AM69,
Gazebo/RViz/camera in the host and zero fall. The final protocol, however, did not close
the gate: **419,9 s, 8,31 m of path, 0,0198 m/s, 0 goals of 8 m completed**
(two 200 s deadlines). The value repeats Wi-Fi with perception (0,0197 m/s),
refuting the hypothesis that changing only the physical medium would remove the
bottleneck. The remaining cost is on the camera processing/copying/fragmentation path
and on the low working ratio of MPPI.

Full evidence and CSVs in **`docs/results/ml35-hil-ethernet.md`**. F5 remains open until
a 8 m goal finishes `SUCCEEDED` in the 420/200 s protocol.

* *Watch out when you read that plan: ** the blocker he records — "the TF tree does not
close, there is no frame `odom`" — ** was solved in 20/08/2026**. The tree now has
22 edges, 9 static, root `map`, and the Nav2 plans and deflects
over the quadruped. See section "F5 — Nav2 on legs" below.

Phase A (parametrix gait + versioned test bench) completed at 19/08/2026.

Phase B (Defect 2, drop in `HOLD` prolonged) **Completed in 20/08/2026** with
`hold.settle_rate: 0.02`, which became default in `gait_go2.yaml`. The correction that seemed obvious —
lowering only the yaw input of `balance.weight_moment` from 450 to 100 — was tested and
**REJEITADA**: advanced the collapse of 161,7 s to 91,1 s and took `RECOVER` from 74 to
357 in the same window. `gait_go2.yaml` records this next to the parameter so no one can
retain it. Evidence on `docs/results/ml35-postura-parada.md`.

Note that F3** was not a kinematic delay**. The A1→Go2 exchange eliminated this work:
Go2 is the native robot of the upstream base. F3 became a careful vendor + integration.

---

## Inviolable rules of this task

In addition to the project's `CLAUDE.md`, they do not replace them.

1. Gazebo is OGRE 2. Run the x86 host, never in the module. No container with
   `ros-jazzy-ros-gz` goes to arm64.
2. `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in all containers, hosts and module,
   No exception.
3. Topic contract preserved byte a byte: `/demo/cmd_vel`, `/demo/odom`,
   `/demo/scan`, `/demo/camera/image_raw`. Nav2 and perception cannot know that the
   robot has legs.
4. Nothing here measures performance. QEMU builds arm64 image and nothing else.
   Latency, jitter and gait stability are only worth measured in hardware, and this
   is outside the scope of ML3.5.
5. Diff-drive remains selectable by arg launch, in the `use_meshes` pattern.
6. `demo_perception` is not played in any phase.

* *Behaviour:** surgical change. Each altered line tracks to a phase. Do not improve
adjacent code, do not refactor what is not broken. When a premise falls, **stop and
say** — a premise that falls silent in the middle of kinematics delay is the error class
that ML3.1 has already paid for.

---

## Gates

Every phase stops at the gate and waits. Do not amend phases.

- **F0** — `colcon build` clean, 39 tests, clean tree, ML3.1
  describing what's in the tree.
- **F1** — today's diff-drive demo full wheel in containers with the same
  goal result Nav2 `SUCCEEDED`. No change of behavior.
- **F2** — Go2 upstream, unmodified, standing and walking by `cmd_vel` inside the
  container `sim`. ** Failed here, C dies** and we return to B (visual quadruped on
  diff-drive), with F0 and F1 already committed and valid.
- **F3** — Go2 (not A1, see decision of F2) standing, stable, responds to `cmd_vel`
  without falling, with the packages and the project launch.
- **F4** — contract identical to today, verified by `ros2 topic list` and by
  message type, with `demo_perception` untouched.
- **F5** — goal Nav2 `SUCCEEDED` with the leg robot: first everything in the host,
  Then with Nav2 running on the module.
- **F6** — `robot_type:=quadruped|diffdrive` working in both directions,
  Extended tests coming through.

---

## F0 — completed (commit `3885f2e`)

29 files, +2870/−655. Gate met: `colcon build` clean (6 packages), `colcon
test` 39 tests 0 failures, clean tree.

* * Surrender:

- Changelog ML3.1 rewritten. The previous entry described the assembly part to
  part by mesh (with measurement via `pycollada` and a `_visuals.xacro`) which ** does
  not exist in the tree** — was attempted and abandoned. What exists is the wrapper on
  TurtleBot 4 upstream.
- Reconciliation of documentation for the host/module axis: `CLAUDE.md` and
  `.ai/CLAUDE.md` updated; `compose/{learn,emul,target}.yaml` (the three empty,
  checked before removing) deleted.
- The `emul` mode was** discarded** together. Arm64 images are still built under
  QEMU, but there is no more dedicated compose to run the emulated stack.

* *Operator pendant, inherited from ML3.1:** Visual confirmation on RViz2/Gazebo with
GUI (requires interactive graphical session). It does not block F1.

---

## F1 — completed 14/08/2026

Beaten gate: **good Nav2 `SUCCEEDED`** (`error_code: 0`) with full demo in containers,
sent from container `tools`. `colcon build` clean (6 packages), `colcon test` **46 tests 0
failures** (were 39; +7 from `wait_for_clock`). Evidence of execution in
`docs/results/ml35-f1-execucao.md`.

Measured rates, learn mode, host x86: `/clock` 334 Hz, `/demo/odom` 27,8 Hz,
`/demo/scan` 10,0 Hz, `/demo/camera/image_raw` 10,0 Hz, `/demo/perception/detections` 10,0 Hz. AMCL, bt
navigator, controller server and planner server all `active`. Topics contract preserved,
`demo_perception` untouched (rule 6).

** Created:** `docker/{base,sim,nav,perception,viz,tools}/Dockerfile`,
`docker/hw/README.md`, `docker/compose.{host,module}.yml`,
`docker/cyclonedds/{host,module}.xml`, `docker/entrypoint.sh`, `docker/.env.example`. The old
scaffold directories (`navigation`, `simulation`, `hmi`) were all empty and
untraceable by git — there was no rename, it was created.

** `demo_bringup/launch/{sim,nav,perception,viz}.launch.py` (new),
`demo_bringup/{setup.py,package.xml}`, `demo_bringup/demo_bringup/wait_for_clock.py`
(new) and its test. `demo_navigation/{setup.py,package.xml}` and `navigation.launch.py`
— see "winding" below.

### Risk number one was confirmed, and the decision was to change the timers.

`learn.launch.py` (20 s perception, 25 s nav) delays measure the time since the
rise of the **own** container, which has no fixed relationship with the moment when
Gazebo finished loading the world. `docker compose up` all goes up together.

Replaced by the **`wait_for_clock`** (`demo_bringup`), waiting for `/clock` to exist **
and advance** before releasing Nav2 and perception. It requires two samples with
strictly increasing timestamp: one sample would pass with paused Gazebo (`gz sim`
without `-r` starts paused), exchanging a silent failure for another. Timeout of 120 s,
exits != 0 — container waiting forever seems locking, does not fail.

This** extends the scope of F1** in relation to the "no behavior change" of the gate: it
is new code, not just packing. Operator's decision, taken with the alternative (carry
the timers as they were) on the table. The 12 s of spawn and 15 s of bridge**within** of
`simulation.launch.py` remain untouched — they are intra-container and the timer there
still measures what it should.

### Three traps found in the execution, all silent

None of these appear as a mistake to name the cause. They're registered because they
cost time and will reappear.

**1. `${HOST_IP}` in bind-mounted file never expands.** XML from the §5 guide uses
`<Peer address="${HOST_IP}"/>`. Docker does not replace variables within mounted file,
so CycloneDDS receives the literal string `${HOST_IP}` as address. Combined with
`AllowMulticast=false`, result: **no discovery mechanism left**, neither between
processes of the same container. Symptoms: `ros2 node list` empty, `ros2 topic list` only with
`/rosout`, and Gazebo spawner in `Waiting messages on topic [robot_description]` forever
— while `robot_state_publisher` logged `Robot initialized` into the same container.
Correction: `<Peer address="127.0.0.1"/>`, which is **load-bearing**, non redundant. The
IP of the module enters F5 (see `module.xml`, which today only speaks to itself on
purpose).

Do not confuse with forcing `<NetworkInterface name="lo"/>`: this has been tested and
is**wrong** — isolates the client from nodes that have already selected the actual
interface (here `wlp0s20f3`). Stays `autodetermine`; peer localhost only adds discovery
address.

**2. The guide's `eth0` does not exist on this machine.** `ip -br link` in the host gives
`lo`, `enp0s31f6` (DOWN), `wlp0s20f3` (UP, Wi-Fi), `tailscale0`, `docker0`.
Both XMLs use `autodetermine` instead of a fixed name, with the verification procedure
commented on in the file. As the host is in Wi-Fi, the case "multicast dies" is
expected, not the exceptional.

**3. `GZ_SIM_RESOURCE_PATH` empty in container.** `demo_description` references mesh as
`model://nav2_minimal_tb4_description/meshes/*.dae`. Natively solves by environment ROS
environment; in container not. Symptoms: correct collision and inertiaal spawna robot —
physics and navigation work — and **no visible body**. Invisible robot in the GUI
but present for planner. Fixed with `ENV GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share` in `sim/Dockerfile`; mesh errors from N to
0.

### Vendorization of four Nav2 files (rule 1)

`ros-jazzy-nav2-bringup` **hard-depends** by `nav2-minimal-tb3-sim`,
`nav2-minimal-tb4-sim`, `ros-gz-sim` and `navigation2`. Measured: puts `libogre-1.9`,
`gz-ogre-next-vendor`, `gz-rendering`, `gz-gui` and 30+ packages in the image `nav` — **3,7 GB
and OGRE 2 in an image that goes to AM69**, direct violation of the 1 rule.
`--no-install-recommends` does not help: are `Depends`.

`ros-jazzy-navigation2` (the metapackage) has the same problem one level below, via
`nav2-rviz-plugins` → `rviz-ogre-vendor`.

Solution: The four Launch files that `navigation.launch.py` need (`bringup`,
`localization`, `navigation`, `slam`) are vendored in `demo_navigation/launch/nav2_vendored/`, Apache-2.0,
copyright headers intact, **only the root paths of rerouted package**. Nav2 servers
enter the Dockerfile individually. Provenance and exact editions on
`nav2_vendored/README.md`.

Result: `nav` from 3,7 GB → **2,48 GB**, and **zero** OGRE/RViz/Gazebo packages.
Verified the 18 plugins declared in `nav2_params.yaml` — all resolve via pluginlib
in the image (missing plugin is not build error: lifecycle transition failure).

* *Cost accepted:** the list of servers on `nav/Dockerfile` and
`demo_navigation/package.xml` now docks with `nav2_params.yaml`. Unlisted new package
plugin requires growing both lists. It's commented on both places.

### Image sizes

| Image | Size | Are you going to the module? |
|---|---|---|
| `base` | 912 MB | is the basis of all |
| `perception` | 912 MB | ** Yes** (arm64) |
| `tools` | 952 MB | Yes (arm64) |
| `nav` | 2,48 GB | ** Yes** (arm64) |
| `sim` | 2,47 GB | No, x86 only |
| `viz` | 2,7 GB | No, x86 only |

`nav` at 2,48 GB remains fat for Torizon data partition. It is no violation of
any rule (there is nothing more graphic), it is weight. Additional diet, if necessary,
is the work of F5 — that's when `nav` actually promotes for arm64 and goes to the
module.

### Not validated at this stage

- * Nothing in arm64. No arm64 images were built in F1; the `platform:`
  are declared and `compose.module.yml` is written but not executed. Rules 5 and 7.
- **Unable module** in this session. `compose.module.yml` and
  `cyclonedds/module.xml` are code not executed.
- * * Visual confirmation on GUI.** Mapping `/dev/dri` was not enough: `renderD128` is from
  group `render` (gid 992 on this host) and container user `ubuntu` is in `video`.
  Symptoms: `libEGL warning: failed to open /dev/dri/renderD128: Permission denied`
  and silent fall to render into software — the demo runs, just slowly. Fixed with
  `group_add: ["${RENDER_GID:-992}"]` in `sim` and `viz`; verified that with gid
  the device is readable and without it the open fails. After correction: 0 libEGL
  errors, 0 mesh errors, goal `SUCCEEDED`.

  **`RENDER_GID` is host-specific** (`getent group render | cut -d: -f3`). The default
  992 no compose applies to this machine. **Open thinking:** `docker/.env.example`
  does not document the variable. The file is blocked by environmental permission rule
  (`.env*` is denied for reading and shell), confirmed in two sessions — it is not
  transient. **Correction is manual, operator:** add to `docker/.env.example`

  ```
  # gid do grupo `render` DESTE host: getent group render | cut -d: -f3
  # Sem isto, sim e viz caem para render em software sem erro que nomeie a causa.
  RENDER_GID=992
  ```

  The explanatory comment is already in the two services of `compose.host.yml`, which
  is where the variable is consumed.

  What remains **not verified by human eye**: if the robot appears correct in Gazebo
  and RViz2. Mesh errors have zeroed and render is accelerated, but no one has looked
  at the screen. Inherited from ML3.1 and still pending from the operator.

---

## F2 — verification executed 14/08/2026, stopped at a blocker

`legubiao/quadruped_ros2_control` in `/tmp/f2-spike`, HEAD `5434c58` ("x30 repaint").
The table "To be confirmed in F2" was traveled whole** before** writing any code, and it
better have been: two claims of the plan fell, and one of them blocks.

### Check table result

| Statement of the plan | Checked in Tree | verdict |
|---|---|---|
| Branch default is Jazzy | default branch is `main`; README line 10 says "developed under ROS2 Jazzy", Humble has own branch | ✅ in practice |
| Supports Harmonic | `gz_quadruped_hardware` depends on `gz_sim_vendor`/`gz_plugin_vendor`; `descriptions/README.md` §2 asks `ros-jazzy-ros-gz` + `ros-jazzy-gz-ros2-control` | ✅ |
| Apache-2.0 License | root is Apache-2.0, and **all code**(controllers, commands, libraries, hardware) declares Apache-2.0 | ** only for code** |
| No A1 config | **False. ** `descriptions/unitree/a1_description/` exists, complete |  |
| What comes from `chvmp/robots` | **nothing.** Descriptions do not come from `chvmp/robots`; A1 has maintainer `laikago@unitree.cc`, i.e. direct Unitree origin | ❌ premise fell |

### The blocker: `a1_description` declares `<license>TODO</license>`

This is the find that for the phase. The licence **per package**, measured in the
`package.xml` of each of the 24 packages:

| Package | Licence declared |
|---|---|
| all code (11 packages: controllers, commands, libraries, hardware) | `Apache-2.0` |
| `go2_description` | `BSD` |
| `b2_description`, `magicdog_description` | `BSD` |
| `anymal_c_description` | `BSD-3` |
| `lite3_description`, `x30_description` | `MIT` |
| **`a1_description`** | **`TODO`** |
| `go1_description`, `aliengo_description`, `cyberdog_description` | `TODO` |

The root being Apache-2.0 ** does not cover** the `a1_description`: Repo-father license
does not inherit by assumption — it is the rule that already killed Tugbot in ML3.1 and
the two repos Go2 in choosing the base. There is no copyright header in any
`a1_description` file (neither in `robot.xacro` nor in the self-generated `robot.urdf`). The only source
sign is the maintainer `laikago@unitree.cc`. `LICENSES/` in root covers only
`legged_control` and `unitree_guide` — no description of robot.

* * Practical consequence:** A1 is the target robot of the demo. F3 sells precisely this
description. Vendorizing file without license declared in a commercial demo of Toradex
is exactly the risk the project has already decided not to take twice.

### What that doesn't block

The F2 gate is **Go2**, and `go2_description` declares **BSD** — valid license, and is
the description that the spike would use. The lock is from F3 onward, not the spike
itself. But running F2 without solving this means spending the spike phase to prove a
base whose destination (A1) is legally indefinite.

I didn't write code because the decision changes the target of the job, not just his
order.

### Possible paths — **decision taken: path 1**

1. **[ESCOLHIDO] Switch the target robot from A1 to Go2.** `go2_description` is BSD,
   has `ocs2`, `legged_gym`, `himloco` and `robot_lab`, and is the most exercised
   robot in the repo — including with `gazebo_rl_control.launch.py` itself, which
   the A1 does not have. Eliminates the blocker and reduces the risk of F3,
   which is delayed. Accepted cost: the original request names A1; the demo is now
   called Go2 quadruped.
2. Track the actual license of A1 upstream (`unitree_ros`) and follow if it is
   BSD-3 — not followed, tracking cost did not compensate with Go2 available.
3. Accept the risk explicitly — not followed.
4. Back to option B (visual quadruped on diff-drive) — not followed.

If A1 is a hard demo name requirement, the 2 path becomes a prerequisite before F3 turns
out the description — but nothing in F2/F3 technically requires A1
specifically; the topic contract (F4) does not distinguish the two.

## F2 — spike executed 14/08/2026, slammed gate

Disposable image `demo-sim:spike-go2` (Dockerfile in `/tmp/f2-spike`, **not committed**
— is spike, does not enter the tree). `ros:jazzy-ros-base` + apt
`ros-gz-sim`/`ros-gz-bridge`/`ros2-control`/`ros2-controllers`/`gz-ros2-control` (only to
satisfy build headers; the plugin that actually runs is `gz_quadruped_hardware` **from the clone
itself**, not from the apt — see finding below), a shallow clone of `quadruped_ros2_control` with the packages that
the spike does not build removed before the `rosdep install` (only removal of what does not
build: nothing the spike uses has been touched). Build via `colcon build
--packages-up-to go2_description unitree_guide_controller keyboard_input
gz_quadruped_playground`. 7 packages, clean build.

**Spike Launch** (`/spike/spike_go2.launch.py`, also uncommitted) reflects
`unitree_guide_controller/launch/gazebo.launch.py` upstream without modifying it, with
two deliberate changes: RViz2 removed (rule 1 — the `viz` of the actual project is in
the host, outside the `sim` container) and Gazebo headless (`-s`, without
GUI). A **spike bridge** (`twist_to_inputs.py`, idem) translates `/demo/cmd_vel`
(`geometry_msgs/Twist`, the actual name of the contract) to `/control_input`
(`control_input_msgs/Inputs`), which is what the controller actually accepts — found
already registered below. The bridge also runs the state machine (`PASSIVE → FIXEDDOWN →
FIXEDSTAND → TROTTING`) with real waiting 5 s between each command — insufficient in the
first attempt (see "trap" below).

### Result, measured by `gz topic -e -t .../dynamic_pose/info`, not by log

| Moment | z (height) | Guidance | Interpretation |
|---|---|---|---|
| Before any spawn command | ~0.5 (spawn height) | — | — |
| After FIXEDSTAND before TROTTING | **0.353 m** | near identity | * Standing, steady * |
| In TROTTING stopped (`cmd_vel`=0) | 0.15 m | identity | gait in lower position, but did not fall |
| Walking, `linear.x=0.03` (low win), 8 s continuous | **0.343 m sustained** | near identity | * Standing, stable, without falling * |
| Walk, `linear.x=0.15–0.3` | drops to 0.07–0.24 m, tipping orientation | robot loses balance | **Symptom, no structural failure** |

**Gate beaten in low gain**: Go2 upstream, unmodified, standing and walking by `cmd_vel`
within a container shaped as `sim`. Zero errors in the entire execution log (`grep -c
"Err\]"` = 0).

* The fall in high gain does not block the gate. The guide already recorded the risk
before running: *"Studented gait parameter... produces robot that walks badly without
generating error... The F3 gate is robot standing and stable responding to `cmd_vel`,
not built clean."* The probable cause is the spike bridge being a naive linear mapping
of `Twist` for the `-1..1` normalised joystick of `Inputs`, without the speed limits
(`v_x_limit_`) that a real joystick UI would respect — **this is the
responsibility of F4** (the actual bridge of the contract), not of F2.

### A silent trap at this stage too

Test the state machine manually via `ros2 topic pub .../control_input` **while the spike
bridge of the launch still ran in parallel** produced two publishers competing for the
same topic and a state setback (`trotting → fixed stand → fixed down`) that seemed to be
controller instability and was not — it was two test processes competing for the same
`/control_input`. Diagnosed by reading `StateTrotting::checkChange()` directly (file
line 76-84 of the file): `command==2` forces a return to `FIXEDSTAND` even in stable trot.
Corrected by isolating a single publisher by test. Registered because it's the kind of
failure that "seems the robot falling" when it's actually the test harness.

### Other facts collected in the clone, for F3 on

- **`gz_quadruped_hardware` is from the repo**, version 2.0.6, license
  `Apache 2`, maintained by Alejandro Hernández / Bence Magyar (is a `gz_ros2_control`
  upstream fork). The plan was supposed to use apt's `gz_ros2_control` 1.2.19 —
  ** that's not what the base uses**. Confirm which of the two enters the image `sim`
  before F2 runs; install apt's and expect the base to use it is an unverified
  assumption.
- `unitree_guide_controller/launch/gazebo.launch.py` climbs **RViz2 within it
  Launch** (no `rviz_ocs2`). This is OGRE 2: In our architecture, RViz lives in the
  `viz` container, not the `sim`. The spike will have to unplug that node — it's rule
  1.
- The lunch accepts `pkg_description:=<pacote>` and `height:=<z inicial>`. The README of
  A1 uses `height:=0.43`; the parameter is the z of spawn, and there is because
  quadruped spawnado on the floor falls.
- The CycloneDDS × `unitree_sdk2` collision is confirmed on README** (lines
  37-40), recommending FastDDS. Still not blocking the ML3.5 — the SDK only enters
  with a physical A1, out of scope — and the `hw` container already
  exists for that.

---

## F3 — completed 17/08/2026 (commits `db4e6f3`, `ae3d9a1`)

Batted gate, measured by `gz topic -e -t .../dynamic_pose/info`, never by log:

| Moment | z (height) | x | Interpretation |
|---|---|---|---|
| Standing after FSM | **0,352 m** | 0,041 | standing, stable |
| Walk, `linear.x=0.03`, 12 s | **0,351 m sustained** | 0,041 → **0,216** | Walks real, without losing height |
| Final Guideline | `-7,2e-05` | — | practically level |

0 errors in Gazebo, 3 drivers `active`. **Better than F2**, who saw the height drop from
0,353 to 0,343 during the march — changing timers for a chain of events made the climb
cleaner.

### F3.0 — the spike of F2 was gone

`/tmp/f2-spike` was taken by cleaning `/tmp`. The three files have never been committed
(decision of F2: spike does not enter the tree). Recovered from `demo-sim:spike-go2`
image, which survived: the two sources by `docker cp`, and the reconstructed Dockerfile
layer by layer from `docker history --no-trunc`. Copy in `scratchpad/f2-recovered/`.

** Lesson:** knowledge that exists only in `/tmp` does not exist. If a future spike
matters, either commit, or if you agree to lose it.

### License tracking — the justification for F2 was incomplete

F2 exchanged A1 for Go2 registering that "`go2_description` declares **BSD** — valid
license". True, but insufficient, and measured again at the time of sale:

| Evidence | `a1_description` (rejected at F2) | `go2_description` (chosen) |
|---|---|---|
| `<license>` | `TODO` | `BSD` |
| `LICENSE` file | absent | ** Absent** |
| Copyright Header | absent | ** Absent** |
| Author/maintainer | `laikago@unitree.cc` | **`TODO` / `TODO@email.com`** |
| Covered by root `LICENSES/` | no | ** No |

Go2 was better than A1 in ** one** field, and worse in another (A1 at least pointed a
traceable maintainer). "BSD" alone does not identify the variant, and all require
playing a copyright notice that did not exist in the package.

**Resolved by tracing to the real origin:** `unitreerobotics/unitree_ros`, BSD 3-Clause
with full text and identified holder (HangZhou YuShu TECHNOLOGY CO.,LTD.,
2016-2022). The **7 meshes are bit-identical** to the upstream, proven by
hash git blob against GitHub's API. Complete table and playback commands in
`ros2_ws/src/go2_description/README.md`.

The xacro layer **does not** match the upstream (it is a ROS 1 → ROS 2 port
from `legubiao`). Adopted as derivative work covered by BSD-3, with the residual risk
explicitly registered in README instead of erased.

Same thing in the control layer: `package.xml` declare Apache-2.0, but the three
packages derived from `unitree_guide` are covered by
`LICENSES/unitree_guide/LICENSE.txt` of the upstream root, which is **BSD-3 of
Unitree** — same mesh holder. Corrected statements and copied text inside each package.
See `unitree_guide_controller/PROVENANCE.md`.

### The silent trap of this phase

I copied from the plant diff-drive to `TimerAction` from 12 s before spawn. Measure:

```
spawn em z=0.49999 → z=0.0677 em menos de 1 s → controladores ativam ~3 s depois
```

The robot passes the entire window in **free fall without controller** and collapses.
Final status: collapsed on the ground, **three controllers reporting `active`, zero log
errors**, and the FSM marching through `passive → trotting` on top of a fallen robot.

No sign of log reports that. Only the pose reads straight from `gz`. It's exactly what
the F3 gate exists to catch — *"Robot standing and stable responding to `cmd_vel`, not
build clean"* — and validates the decision to measure by pose.

* *Correction:** immediate span, chained by `OnProcessExit` (`spawn → broadcasters →
controlador de marcha`), no timer. That's what `gazebo.launch.py` upstream already does.
The long comment on `quadruped.launch.py` explains why there can be no `TimerAction`
there.

### Created / touched

**Vendorized** (5 packages, upstream names preserved for `$(find)` to resolve without
editing): `go2_description` (25 MB), `control_input_msgs`, `controller_common`,
`unitree_guide_controller`, `gz_quadruped_hardware`. Source: `go2_description/README.md` and
`unitree_guide_controller/PROVENANCE.md`.

** Ours:** `demo_simulation/launch/quadruped.launch.py`,
`demo_simulation/demo_simulation/twist_to_inputs.py` (spike promotion),
`demo_bringup/launch/sim.launch.py` (`robot_type` router → a launch per plant without
conditionals), `docker/sim/Dockerfile`.

**Deactivated style lint** on both sold C++ packages: `ament_lint_auto` ran on
third-party code and produced 98 code failures that policy commands not to edit.
Correcting would destroy byte-identical; leaving makes `colcon test` red forever. Our
packages keep your linters.

### Not validated at this stage

- * Nothing in arm64, nothing in the module. ** Rules 5 and 7.
- ** RViz2 remains pending.** The quantitative execution of the gate was headless.
  In 17/08/2026 the operator repeated the launch with `gui:=true` in the spike
  image and confirmed the Go2 model visible in Gazebo in `empty.sdf`. This closes the
  preview of the model in Gazebo, but does not validate the TF tree and the
  RobotModel in RViz2.
- **The quadruped in the `warehouse.sdf` world.** The gate ran on `empty.sdf`. The
  project world takes ~10 s to load and has 50+ meshes; the spawn is now immediate, which
  is safe (`create` does retry), but was not exercised there.
- ** March in high gain. ** Continues what F2 measured: above ~0,15 the robot
  You lose balance. It is mapping synchronous in `twist_to_inputs`, and is **F4**.
- **Nav2 on legs.**F5. Plant does not publish `odom → base_link`.

---

## F4 — completed 24/08/2026

> * *Continuity:** this section records the checkpoint of 18/08. The drop in HOLD
> it was corrected in 20/08 by `hold.settle_rate: 0.02`, with the three criteria of
> green gear; see `docs/results/ml35-postura-parada.md`. Revalidation
> joint contract and perception was executed in 24/08.

Detailed checkpoint on `docs/results/ml35-f4-parcial.md`. Real names, types and messages
of odom, scan and image crossed two containers by DDS. The first mapping SI → stick
overthrew Go2 and was replaced by clamp in the `0.03` envelope proven in F3, but this
latest edition has not yet been revalidated in runtime. Perception, official warehouse
and diff-drive regression were pending at that checkpoint.

**Closure on 24/08/2026:** cold start of profile `learn` with Go2, warehouse, Nav2 and
perception in different containers. The five topics were discovered with the contract
types: `geometry_msgs/msg/Twist`, `nav_msgs/msg/Odometry`, `sensor_msgs/msg/LaserScan`, `sensor_msgs/msg/Image` and
`vision_msgs/msg/Detection2DArray`. Real messages were received from odom, scan, image 640 px and synthetic
detection in the consumer. `/clock` advanced, TF closed and Nav2 reached `Managed nodes
are active`. This closes the F4 gate without making performance or hardware claims.

**18/08/2026 — the gait came into existence.** `StateTrotting` was split into `WALK`,
`HOLD` and `RECOVER`, and `twist_to_inputs` gained a command watchdog. Three flaws
were overlapping and one hid the other:

1. the upstream pass gate asked `|v| > 0.03 m/s` and the command path
   entire delivery at most `0.012 m/s` — no pass was requested, and the `contact=[1 1
   1 1]` registered before was that, not dynamic;
2. `pcd_` is integrated reference and was not recaptured when stopping, so the QP
   kept accelerating the body after the command zeroed;
3. `Inputs` has no timeout: a publisher who simply left the robot
   Walking with a command no one sent.

Measure: `mode=WALK` already in `Twist linear.x=0.01`, alternating diagonal pairs,
`HOLD` stable for more than 35 s with `posErrXY ≈ 0,005 m`.

**18/08/2026 — the robot walks.**30 s of continuous trot to `v_cmd = 0,1 m/s`,
3,00 m covered, no entry into `RECOVER`, maximum tilt 2,3°, average speed
measured 0,106 m/s. Two causes, both measured before any adjustment:

1. * *The command was outside the marching regime.** `_SAFE_STICK_LIMIT = 0.03`
   was documented as a "stable envelope of F3", but was measured while the gait never
   activated — described the push on a planted foot robot, not walking speed. The
   `v_cmd = 0,004 m/s` step requested is 4 mm under standing elevation of 8 cm: the
   robot marched in place. High to `0.5`.
2. * *The course is not controllable by QP on this robot.**Instrumenting `bd_` against
   `A_ * F_`, the time of turn request was locked in ±5,3 N·m = stop `d_wbd(2) ±10
   rad/s²` times `Izz`. With `kp_w_ = 780` this stop saturates with **0,73°** of yaw
   error, and above that the signal becomes chosen by the gyroscope wave, not by
   error. Yaw on a legged robot is controlled by where the foot lands: `k_yaw_`
   in `FeetEndCalc` was worth 0,005 against the 0,1125 that needs to cancel at the
   moment of landing, so the support pattern was rotated and the legs crossed to the
   center. `k_yaw_ = 0.15` solves.

Extend the stop (±25) and dismember the attitude gains per axle were tested and
**rejected by measurement** — evidence in `ml35-f4-parcial.md`.

This was the state at 18/08. Correction and replacement criteria are in the
20/08 report cited above; do not use this historical paragraph to choose the
next experiment.

---

## Preparation of target — 20/08/2026

* It's not a phase. It is infrastructure work for F5, done in parallel to F4 trials on
the host, because the module has become accessible. Full evidence on
`docs/results/ml35-target-preparacao.md`.

What changed state in the project:

1. * * The premise "the module is not accessible" fell.** Aquila AM69 inventoried:
   Torizon OS 7.7.0+build.40, 8 × Cortex-A72, 31 GiB RAM, 108 G free, Docker
   25.0.9 arm64, Compose 2.26.0, `torizon` in group `docker`.
   `ethernet0` in `<MODULE_IP>/24`; host x86 in `<HOST_IP>` in the same /24.
2. * *The DDS network was measured, not assumed.** UDP in both directions in three ports
   69 domain. `ufw` is active in the host and ** does not block**. No firewall
   changes are required.
3. * *The 1 rule was being violated by the current tree in silence. The layer of
   container is F1 (diff-drive); F3 brought in `gz_quadruped_hardware`, which
   declares `gz_sim_vendor` and `gz_plugin_vendor` as `<depend>`. A blind
   `colcon build` of `src` would put OGRE 2 on the arm64 images. Fixed by two
   build args (`SKIP_KEYS_EXTRA`, `COLCON_IGNORE_PACKAGES`), both default empty — the
   amd64/host side does not change.
4. **`autodetermine` no `module.xml` was a real trap**, not theoretical: a
   Toradex easy-pair bridge Docker (`br-*`, 192.0.2.9) is UP along with `ethernet0`.
   The interface is now fixed in rendering time, and the host peer is injected there
   too, so no address enters git.
5. **`scripts/module.sh`** became the interface for the module:
   'inventory | sync | build | up | down | status | verify | shell`.
6. * *Four images `arm64` exist in the module**, built natively there:
   `base` 1,24 GB, `perception` 1,28 GB, `tools` 1,32 GB,
   `nav` 2,44 GB. Rule 1 verified in the four by inspection of installed
   libraries.
7. * *The contract crosses the machine border in both directions, measured.**
   69, `/demo/system/heartbeat`: module→host `count=11` received at the host;
   host→module `count=14` received within the `tools` container, with
   `/demo/heartbeat_publisher` visible in `ros2 node list` of the module. **This is the
   prerequisite for F4/F5 infrastructure, not their gate.**
8. * *Discovered that setting only one side of DDS fails identical to firewall.**
   CycloneDDS default announces by multicast (which the module ignores) and does not
   fix deterministic port (then the module unicast has no target). Both sides need
   married config. `scripts/module.sh` renders both: `module.xml` for the module and
   `docker/cyclonedds/host.rendered.xml` on the host, both with injected address and
   gitignored/generated.
9. **`ROS_NAMESPACE` does not work on ROS 2 Jazzy.** Verified: the variable is
   in the process environment (`printenv` confirms) and ROS ignores it; only
   `--ros-args -r __ns:=` works. `scripts/env.sh` exports `ROS_NAMESPACE=/demo` as if
   it worked — **it has not been changed**, the file is in use by the F4 trials.
   Stays as we find it.

What has not changed, and must be clear:

- **F5 remains blocked for the same reason as before.** The target is ready not
  solves the TF tree that does not close nor the absence of the frame `odom`
  (`plano-movimentacao.md`). Nav2 over legs does not pass the F5 gate due to lack of
  `odom`, regardless of whether the module is standing.
- **No performance was measured** (rule 5 and 7). The images were built
  native to the module instead of under QEMU, which is not a measurement of anything.
- * *The module does not see host simulation topics.** No module defect:
  `scripts/run_quadruped_sim.sh` goes up to yes without `CYCLONEDDS_URI`, so it
  announces by multicast and the module (multistat off) can't find it. The mechanism
  is proven in both directions with test publishers; there is no passing the config
  rendered to the host producer. **Not changed in this session because this script is
  in use by F4 trials.**
- **`compose.host.yml` continues to mount `cyclonedds/host.xml`**, the template without
  the peer of the module. For the `hil` containerized it needs to point to
  `host.rendered.xml`.
- **`nav` was not raised in the module. ** Nav2 publishes `/demo/cmd_vel`, and simulation
  the host runs in the same domain 69: two publishers in the topic that commands the
  robot would corrupt the ongoing trial without anything in log explaining.
  `scripts/module.sh up` detects active simulation and default refusal.

---

## F5 — Nav2 on legs: ongoing 20/08/2026

Closed and functioning mesh: cloud 3D → costmap → planner → MPPI → unit conversion →
march → Gazebo → odometry → TF → costmap. Measured in `quadruped_objects.sdf`, the robot
traveled 8,36 m, displaced 3,51 m net, reached **3,8 cm** of the goal and
passed through the four obstacles with positive clearance, without falling.

Full evidence on **`docs/results/ml35-nav2-quadrupede.md`**; how to run, on
**`docs/guides/cenarios/s5-nav2-desvio.md`**.

### The three F5 blockers are closed

| blocker | How it was closed | consequence to remember |
| --- | --- | --- |
| TF tree does not close | `demo_bringup/odom_tf` publishes `odom → base` and `map → odom` | ** is not a state estimate** — is Gazebo ground truth turning TF; leaves when the leg estimator exists |
| Base frame name | Nav 2: `nav2_params_go2.yaml` uses `base` | `go2_description` is sold byte-a-byte and cannot be edited |
| deal with a ring | the bridge exposes `/scan/points` as `PointCloud2` in `/demo/scan_cloud` | `/demo/scan` continues to exist and remains useless for costmap |

The number closing the third: in the world of objects, `/demo/scan` gives **zero**
obstacles — identical to the empty world — and `/demo/scan_cloud` gives **249**.

### Six defects found by measurement, all corrected

None of them advertise in log. They're listed because each one would cost hours again.

1. **`use_composition` without container.** `navigation_launch.py` with composition
   loads the servers on `/nav2_container`, which only `bringup_launch.py` creates.
   Including only the first: nothing goes up, nothing goes wrong.
2. ** Competing goals.** Between `send_goal_async` and acceptance, the handle is
   `None`; 1 s supervisor reentered and sent another goal.
3. **`progress_checker` of TB4.** 0,5 m in 10 s, against 13 s of turning at 0,12 rad/s
   no advance: 22 abortions with **zero falls**. When the verifier fails and the
   robot doesn't fall, the suspect is the verifier.
4. **MPPI horizon measured in time.** 2,8 s cover 1,4 m on the TB4 and 0,42 m on the
   Go2 — below the reference of ~1 m of `PathAlignCritic`, which has the highest
   weight. Horizon measures in distance.
5. **`/demo/cmd_vel` is not in SI.** Loads manche; controller multiplies
   `linear.x` by 0,4 and `angular.z` by 0,5 (`StateTrotting.cpp:192` with
   `invNormalize`, and `twist_to_inputs.py:283` with unit gain). Nav2 is the first consumer who
   cannot live with it, because MPPI **integrates** `vx` as m/s. Fixed with
   `demo_bringup/cmd_vel_si_to_stick`, a border node — the plant and the existing
   commanders remained intact.
6. **Yaw of the goal as the way out. ** Requires 110–139° to rotate stopped on arrival,
   and turn stopped does not stand still: the robot derived 0,78 m in y and left the
   position tolerance that had already satisfied. The yaw has to be the course
   of**coming**.

### Two hypotheses refuted by measurement

Registered so no one can hold them:

- **"The robot is within an inflated region."**Measure with the robot stopped: cost
  **0** in his cell, **0** within 0,6 m, 42 lethal cells in the obstacles, zero
  unknown. The costmap is correct.
- * *"The sampling dispersion of MPPI limits the magnitude."** Only the correction of
  units took the command of 0,006 to 0,119 m/s**with the same deviations**. `vx_std`
  and `wz_std` were as they were.

### What F5 doesn't have yet

- **State estimate with leg.** `odom_tf` republic ground truth. While
  Yeah, nothing here validates location.
- * *Meta de 8 m at the gate HIL Ethernet.**Ethernet, camera RAW, perception and a
  short meta has already passed in Aquila. In the 420 s / 200 s protocol per goal, the
  two long goals expired. Isolate camera cost and working ratio of MPPI, without
  reducing 640×480 to 10 Hz, and repeat the same protocol.

## F6 — selectable fallback: completed 24/08/2026

`ROBOT_TYPE=quadruped|diffdrive` now selects together the host plant and the
corresponding Nav2 launch, both in the Host Compose and in the module. `quadruped` is
the default; unknown value fails before starting Nav2. Unit tests check the matching and
rejection of invalid values.

Gate run from clean `learn` profile climbs:

- `quadruped`: TF closed, active nav2 and short x
  `SUCCEEDED`, `error_code: 0`;
- `diffdrive` with `SIM_GUI=false`: available odometry and x=0 target for x=1
  with `SUCCEEDED`, `error_code: 0`.

Base images, simulation, navigation, perception, tools and visualization were
reconstructed. The `gz_quadruped_hardware` backend exists only in the simulation image;
`COLCON_IGNORE` keeps headless functions isolated also in builds and later incremental
tests.

## Decisions taken

### Moving base: `legubiao/quadruped_ros2_control`

Apache-2.0, native `ros2_control`, default branch on ROS 2 Jazzy, supports Harmonic.
**All these statements come from README and should be confirmed in the tree at
F2** (see "To be confirmed in F2").

Discarded:

- **`chvmp/champ`** (BSD-3): ROS 1 only (Kinetic/Melodic), latest update
  ~Jul/2024. Porting would be to rewrite middleware + build + control layer.
- **`khaledgabr77/unitree_go2_ros2`** and **`RobInLabUJI/unitree_go2_ros2_jazzy`**:
  Jazzy + Harmonic + CHAMP, but Nav2 marked "coming soon" ** and undeclared license**
  — commercial demo blocker, same criterion that eliminated Fuel Tugbot no ML3.1.
- **`arjun-sadananda/go2_nav2_ros2`** (registered at ML2): single CHAMP+Nav2
  demonstrated, but Humble + Gazebo **Classic**, and compensates odometry error by
  doubling the linear velocity in the state estimator. Outline, not calibration.

Verified in 14/08/2026: there is still **no** quadruped A1 ready in Jazzy + Harmonic + Nav2.
Integration with Nav2 (F5) is ours; no one delivers.

### Compose layout: machine shaft, not mode

`docker/compose.{host,module}.yml` instead of `compose/{learn,emul,target}.yaml`.
Operator's decision. Modes have seen Profiles of Compose + which file is invoked on
which machine. `emul` mode has been discarded.

### F1 inserted before spike

Addition of operator to original plan. Justification at the top of section F1.

### F2/F3 reverse a decision of ML2

The ML2 decided against** `gz_ros2_control`, in favor of the native plugin
`gz-sim-diff-drive-system`, precisely because the first would drag `ros2_control` +
`controller_manager`. F2/F3 reverse this, and with reason: quadruped has no equivalent
native plugin. **Register the reversal in the changelog when F2 close**, so it does not
appear that the decision of ML2 was forgotten.

---

## To be confirmed on F2 — 

The table below is what was intended to be verified. It was verified in
14/08/2026 and **two statements fell**. Kept as a record of what was asked; the
results are in the section of F2.



Nothing from the description of `quadruped_ros2_control` enters as fact:

| Statement | How to check |
|---|---|
| Branch default is Jazzy | `package.xml` / CI in the tree, not README |
| Supports Harmonic | dependence `gz-*` real and version (Harmonic is `gz-sim8`) |
| Apache-2.0 License | `LICENSE` file at root ** and** headers of vendorized fonts |
| No A1 config | `find`/`ls` by `a1` in description and config |
| What comes from `chvmp/robots` | A1 description license ** and** the original license of `unitree_ros` from where it is derived |

Repo-father license**is not inherited by assumption**. It was undeclared license that
killed Tugbot at ML3.1 and the two repos Go2 here.

Also confirm: if the base loads `controller_manager` within the `gz sim` process (this
is what justifies `sim` being a single container), and if `gz_ros2_control` 1.2.19
home with the version that the base awaits.

---

## Verified environment (14/08/2026, host x86)

| Item | Status |
|---|---|
| ROS 2 Jazzy | natively installed |
| Gazebo Yes | 8.14.0 (Harmonic) |
| `ros_gz`, `ros_gz_bridge`, `ros_gz_sim` | installed |
| `ros2_control` | ** not installed** — apt has 4.45.2 |
| `ros2_controllers` | ** not installed** — apt has 4.40.1 |
| `gz_ros2_control` | ** not installed** — apt has 1.2.19 |
| Module Aquila AM69 | ** not accessible in this session** |

The three of `ros2_control` are the prerequisite of F2 and enter the image `sim` in F1.

---

## Registered invariant collation

`quadruped_ros2_control` documents that **CycloneDDS conflicts with `unitree_sdk2`** and
recommends FastDDS. The project's inviolable rule 2 is `rmw_cyclonedds_cpp` always.

* * Does not block ML3.5:** SDK only enters with physical A1, which is out of scope. The
`hw` container has been empty since F1 for the problem to be visible in the right place
instead of appearing as a surprise in the hardware ring-up.

---

## Premises in force

- The spec is `guia-ml35-docker.md`. Where she and the original plan diverge,
  Guide wins**.
- ~~The module is not accessible~~ — **PREMISSA CAÍDA em 20/08/2026.** The Aquila
  AM69 responded and was inventoried; see `docs/results/ml35-target-preparacao.md` and
  the " Target Preparation" section above. `ssh torizon@` and `rsync` now run by
  `scripts/module.sh`. The 7 rule remains fully valid: nothing performance, latency,
  thermal or FPS has been measured or claimed.
- ~~`eth0` in DDS XMLs is placeholder~~ — **RESOLVIDO for the module.** As
  verified interfaces are `ethernet0` and `ethernet1`; on the 24/08 HIL run
  `ethernet1` was explicitly chosen, and it is **fixed at rendering time**, detected from
  `MODULE_IP`, not written by hand: `autodetermine` can choose the Docker
  easy-pairing bridge (`br-*`, 192.0.2.9), which is UP at the same time. `host.xml` is template;
  `module.sh sync` generates `host.rendered.xml` with `enp0s31f6` and the chosen peer.
  With both ports on the same subnet, `MODULE_IP` should be explicit because the same
  hostname mDNS can solve for any of them.
- `tools` appears on `docker compose exec tools` in guide §9 but is not
  declared in the composition §6. It will be declared with `profiles: ["tools"]` and a
  `command` that does not close.
