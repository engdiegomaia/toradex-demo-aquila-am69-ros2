# ML3.5 F5 implementation handoff — next session

This is the authoritative execution handoff for the next session. Read
`estado-fases.md` for history and the round reports under `docs/results/` for evidence.
Do not restart the investigation from older F5 guides.

## Current delivery envelope (30/08/2026, R16 close-out) — read this first

**This branch delivers a stable, supervised navigation-and-exploration demo driven
from the Docker/web cockpit. It does NOT deliver a complete autonomous maze-escape
demonstration.** `/demo/maze/escaped` has never become `true` in any round. Present it
as the first sentence, not the second:

> Demo estável e supervisionada de navegação e exploração pelo cockpit Docker/web.

Do **not** present it as:

> Demonstração autônoma completa de fuga do labirinto.

F5 remains formally **in progress** in `estado-fases.md` because: `escaped` never
reached `true`; blind homing carries a documented, unmitigated fall risk
([[homing-cego-derruba-quadrupede]], `docs/results/ml35-f5-homing-fall-analise.md`);
sweep-recovery (the zero-raw-frontier-cluster path) has never been exercised in HIL;
no three-cold-starts acceptance run has been attempted; and the southwest region
toward the exit was not reached in R16. **In any live demonstration, an operator must
watch `/demo/exploration/status` and cancel immediately if `state` reaches
`homing_exit` — do not let the run cross into homing unattended, let alone attempt an
autonomous exit crossing.**

R15a is **frozen**: an independent code review found four issues (movement watchdog
blind to legitimate rotation and armed too early, an overstated "confirms" claim in two
docs, a map fingerprint blind to geometry-only changes, an inverted "above/below"
threshold comment), all fixed in commit `26aba8f`, all re-verified by a full integrated
HIL round (R16, commit `bcb07fc`) run through the official cockpit path. R16 found no
defect in R15a worth another code change — see `docs/results/ml35-f5-exploration-r16.md`.
**No further Python/YAML/Nav2-parameter change is planned for this delivery.** The next
session should treat R15a as the baseline to build on, not re-litigate it, unless new HIL
evidence contradicts R16's finding.

**Identifiers for this delivery:**

| what | commit / tag |
| --- | --- |
| code actually running on the bench (R15a) | `26aba8f` |
| current consolidated evidence (R16 report + status log) | `bcb07fc` |
| delivered version (this documentation close-out, tagged) | `ml35-f5-r16-demo-stable` |

Bench state at handoff: R15a code, robot at spawn, exploration `idle`, `nav`/
`perception` freshly recreated and `module.sh verify` 4/4 passing. Do not start another
HIL round against this tag without a reason — see the open items above for what such a
round should target.

## 0. Session closed 30/08/2026 (R9–R12) — superseded by R13–R16 above, kept for history

Everything through R12, the `module.sh` spawn guard and its independent-review fixes,
and the positioned AprilTag validation is **committed** (tree clean, nothing uncommitted;
the branch is now pushed as part of the R16 close-out above — see that section for the
current ahead/behind state, not the number below).
Sections 1–9 below predate this work and are kept for the older blocker narrative
(footprint polygon, magenta range bias) — read `estado-fases.md`'s dated entries from
29–30/08 for what actually happened this session; do not treat §2's "Repository
checkpoint" numbers below as current, they describe an earlier point in the same branch.

**What changed since §2 was last accurate, newest first:**

- `docs/results/ml35-f5-apriltag-positioned.md` (commit `d18e399`) — positioned,
  non-acceptance-counted HIL validation of the fiducial detector: PNG renders, id 0
  detected reliably at ~2.5 m (marginal at ~3.6 m), TF resolves, adverse framing fails
  closed, the 29/08 target-latch fix holds under real detections. **Unplanned finding:**
  the quadruped tipped over near the exit opening during unattended blind homing — not
  investigated further, see [[homing-cego-derruba-quadrupede]] and the memory it links.
- `73f96d6`, `7f03df7` — `scripts/module.sh up` now refuses to recreate `nav`/
  `perception` when the robot is more than 1 m from spawn (prevents a `slam_toolbox`
  map-anchor corruption bug, most likely first seen in R10 and confirmed in R12).
  Bypass is `--force-spawn`, independent of the pre-existing `--force` (cmd_vel
  collision guard). Tested in `tests/test_module_spawn_guard.py`.
- `7bc120e` — `frontier_score`'s route penalty changed from linear to
  `-0.5*sqrt(route_m)`: fixed a real northward exploration bias (R9/R11), confirmed by
  R12 physically reaching `(-5.077, 1.469)`, the farthest west/toward-exit any round had
  gotten.
- `63bf264` — R10/R11 fixes: per-cluster multi-candidate retry, a `min_travel_m` floor
  on the recessed navigation endpoint (fixes a real Nav2-goal-tolerance false-arrival
  stall), map-generation-gated provisional recovery, new telemetry fields.
- R9–R12 exploration rounds (`docs/results/ml35-f5-exploration-r{9,10,11,12}.md`):
  Gate B (`escaped == true`) is **still not reached**. R12 is the healthiest round so
  far — ran out of its 600 s time budget while still actively finding frontiers
  (`barren_cycles = 0`), rather than barren-out like every prior round.

**Open question for the next session (R12's own "recommended next step", still
unanswered):** take a `/map` snapshot right after a run reaches `x < -4.5, y < 2.0` and
run the offline `extract_frontiers` diagnostic (same method used for R10/R11) to
determine whether the region south of `(-5.08, 1.47)` toward the exit is a
coverage/topology gap, a clearance-filtering gap, a scoring problem, or a
controller/execution problem. Do not change `frontier_score`, clearance, timeout, or
setback again before that diagnostic — R12 showed the current scoring already reaches
the west corridor and stays active for the full budget; the next fix, if any, should be
targeted from that diagnostic, not guessed.

Before starting a new HIL round: `/demo/sim/reset`, confirm `/demo/odom` near `(0,0)`,
**then** `scripts/module.sh up` (now enforced automatically; see above). The robot was
left upright at spawn at the end of this session.

## 1. Objective and acceptance bar

Finish the autonomous maze demonstration with one exploration start request, no
manual driving or goals, a fresh map and pose graph, escape within 600 s, zero falls,
and no recurrent TF/costmap/frame failures. Final acceptance is three successful cold
starts out of three.

A smoke run can expose a defect but cannot prove reliability. Rounds 2, 3, 4, 4a, the
29/08 observed round and 4b used byte-identical preconditions and travelled 21.93, 4.26,
0.82, 5.86, ~3.7 and ~0.6 m with six different endings.

## 2. Repository checkpoint

Branch: `feat/f5-percepcao-e-partidas-frias`, 14 commits ahead of its remote before
the current uncommitted changes. Do not reset, clean or overwrite the working tree.

Expected working tree: the seven files from the previous handoff, plus
`scripts/exploration_trial.py` (recorder columns), plus new evidence files under
`docs/results/`. Validation completed on 29/08:

```text
demo_navigation tests   65 passed   (61 + 4 new measurement tests)
root contract tests     241 passed
hmi tests               180 passed
ament flake8/pep257     passed  (scoped to the package — see the trap below)
git diff --check        passed
host + module Compose   passed
```

**Trap:** run the package linters from inside `ros2_ws/src/demo_navigation`. Run from
the repo root, `test_flake8`/`test_pep257` scan the whole tree and report ~94
pre-existing errors that are not yours.

No HIL run counted toward acceptance, no commit and no push have been performed.

## 3. Bench access

`aquila-am69.local` does **not** resolve — the module's real hostname is
`aquila-am69-12593525`, so mDNS fails even when the module is up, reachable and serving
SSH. That failure looks exactly like a dead bench; it is not. Find the module by ARP,
not by name (no address is recorded here, per the no-hard-coded-IP convention):

```bash
ip -brief addr show <wired-iface>          # the workstation's side of the link
ip neigh show dev <wired-iface>            # the lladdr with the Toradex OUI 00:14:2d
ssh torizon@<ip>                           # confirm before blaming DDS or the link
MODULE_HOST=<ip> scripts/module.sh sync    # bare IP: the script adds the torizon@ prefix
```

Check in this order before declaring the bench unavailable: NIC carrier, then the ARP
entry above, then SSH.

Host-side builds need `BUILDX_BUILDER=default` (the workstation is IPv6-only; the
`armbuilder` builder has no IPv4 route). `build.network: host` is already set on every
build in `compose.host.yml`.

## 4. Proven state

- TF availability 99.94%; global costmap, map update, gait and short stability pass.
- Aquila perception passes: 60/60 detections, finite pose, timestamped TF.
- Round 2 travelled 21.93 m autonomously, mapped 8915 cells, detected the exit marker
  and entered `homing_exit`.
- The provisional-suppression recovery is **field-validated twice** (observed round and
  R4b): increments, releases only the provisional lists, never touches the hard
  blacklist, does not livelock, terminates through `barren_cycles`. It passes its
  contract and it rescues neither run — see §5.

F5 remains open because `/demo/maze/escaped` has never become true.

## 5. The 253 blocker — addressed by the footprint polygon (not yet promoted)

**The defect:** NavFn refuses to plan from a start cell whose global-costmap cost is
`253` (`INSCRIBED_INFLATED_OBSTACLE`) or above, whatever the goal is. Measured at the R4b
terminal state with the robot at `(-0.52, 0.36)`, its own cell was exactly 253. The five
goals refused in that run were all passable (costs 131–195) and inside the robot's own
connected component; a BFS finds paths to them only because it **relocates the start**,
which NavFn does not.

Consequences, which is why four rounds were mis-diagnosed:

- **`_refused` records the wrong cause.** The planner rejects the *start*; the explorer
  books it against the *frontier*. One pose problem suppresses every cluster at once.
- The R4a deadlock, the observed round's frontier collapse and the R4b barren ending are
  **the same defect wearing three counters.**

**The remedy, measured: replace `robot_radius` with the real trunk polygon.**
`robot_radius: 0.38` is the *circumscribed* radius of a 0.70 × 0.31 m body — a 0.38 m
circle around a 0.31 m wide robot throws away 23 cm of real corridor per side. The
variant `nav2_params_go2_footprint.yaml` swapped both costmaps to
`[[0.37,0.18],[0.37,-0.18],[-0.37,-0.18],[-0.37,0.18]]` and changed nothing else (proved,
before promotion, by a contract test that reverted the polygon to `robot_radius` and
required the two YAMLs to compare equal).

Full evidence: **`docs/results/ml35-f5-footprint-ab.md`**. Headlines:

| | arm A (`robot_radius`) | arm B (polygon) |
| --- | --- | --- |
| own-cell cost, median / max | 168 / **243** | 135 / **165** |
| % of walking time at ≥ 243 | **7.1 %** | **0.0 %** |
| headroom from max to the fatal 253 | **10** | **88** |
| run outcome | R4b dead at 116 s, `refused=4`, `barren=10` | survived to the **600 s deadline**, `refused=2`, `barren=0` |

The collision-monitor coupling the old note warned about did **not** bite, because the
variant *removes* `robot_radius` rather than adding a polygon beside it: no SIGSEGV, no
"Inconsistent configuration in collision checking", footprint published on both costmap
topics, `collision_monitor` `active [3]`, static corridor planning 4/4.

Two honest caveats: neither arm ever recorded a 253 sample, so this shows **margin**, not
a prevented event; and arm B covered 50 distinct poses against arm A's 86, so the traces
are not pose-matched. `consider_footprint` stays `false` — flipping it would be a second
variable and `nav2_params_go2.yaml:537` records that `true` beside `robot_radius`
SIGSEGVs the whole container.

**Promoted in code, HIL final pending.** As of 29/08/2026 `nav2_params_go2.yaml` ships
the footprint polygon in both costmaps (`robot_radius` removed); `params-align8.yaml`
mirrors the same change. `nav2_params_go2_footprint.yaml` is now byte-equivalent to the
default (`test_footprint_variant_matches_the_promoted_default`) and kept only because
Compose, docs and prior campaign commands still reference
`NAV2_PARAMS=.../nav2_params_go2_footprint.yaml`. What promotion in code does NOT close:
a clean smoke from t = 0 under the new default plus a lateral-displacement replan check,
both still to run in HIL.

## 6. THE BLOCKER — marker range is unbiased now, but noisy with distance

**R6 found it and R7 half-fixed it.** `magenta_bbox` took the global min/max over every
magenta pixel in the frame, so any second magenta region merged into one box, inflated
`width_px`, and — since `range_m = fx * marker_width_m / width_px` — collapsed the range.
Replacing it with the largest **connected** region (8-connectivity on the sampled lattice,
dependency-free) moved the estimate/truth ratio against the SDF marker at `(-4.90, -2.60)`:

| | samples | ratio mean | range |
| --- | --- | --- | --- |
| R5+R6, global bbox | 12 | **0.478** | 0.402 – 0.539 |
| R7, connected components | 131 | **1.055** | 0.474 – 2.403 |

Calibration was excluded first: `horizontal_fov` 2.094 rad at 640 px gives fx 184.75
against the published 184.836.

**What remains is range-dependent error, and it is structural:**

| estimated range | n | ratio | mean abs error |
| --- | --- | --- | --- |
| 0 – 2 m | 19 | 0.579 | 1.28 m |
| 2 – 3 m | 39 | 0.876 | 0.60 m |
| **3 – 4 m** | **43** | **1.062** | **0.41 m** |
| 4 – 6 m | 14 | 1.316 | 1.14 m |
| > 6 m | 16 | 1.813 | **3.08 m** |

The overestimate at range is the signature of **partial visibility**: a panel glimpsed
through a maze opening presents less than its 0.80 m width, and a narrower blob reads as
farther. No width-based estimator fixes that.

**The structural answer is a fiducial** — AprilTag or ArUco give four corners and a PnP
pose, and fail closed instead of returning a confident wrong range
(`ros2-aruco-pose-estimation`, `apriltag_ros`). The cost is not the node: it is that
`quadruped_maze11.sdf` states the demo's premise — *"Navigation receives no coordinates:
perception must find this unique magenta panel"* — and swapping the panel re-derives the
marker pose, the opening geometry, `maze_escape_validator`'s constants, the scene cameras
and the contract tests. **That is a product decision, not a bug fix.** Take it if the gate
below is not enough.

Nav2's **Docking Server** (`SimpleChargingDock` consuming `detected_dock_pose`) is the
matching structural replacement for the approach logic, and pairs naturally with a fiducial.
Also deferred for the same reason.

## 6b. Fixed this session — deployed, tested, NOT committed

| change | evidence | status |
| --- | --- | --- |
| arm B costmap: trunk polygon instead of `robot_radius` | own-cell cost max 243 → 165; headroom to the fatal 253 10 → 88 | works, **not promoted** |
| homing survives occlusion (`homing_persistence_s` 90 s, full blind approach) | R5: one entry, `homing_abandons = 0`, closed 1.85 → 1.08 m blind after 11 straight field failures | works |
| arrival when the remaining step is below `xy_goal_tolerance` (`nav_goal_tolerance_m`) | R6 froze 94 s at a byte-identical pose, 0.75 m vs 0.70 m stop | works |
| `magenta_bbox` → largest connected component | ratio 0.478 → 1.055 (131 samples) | works |
| homing entry gate 4.0 m + 3 confirmations + `marker_far_ignored` | R7 committed at 7.35 m and homing held the run 520 s | **untested — R8 never saw the marker** |
| progress checker 0.20/40 → 0.30/25 | R5+R6 simulation over 45 goals: 6/9 expiries caught, 107 s returned, 0 good goals aborted | **unjudged for two rounds** |
| `goal_timeout_s` 90 → 45 | **returned nothing** (3 × 90 = 6 × 45 = 270 s) | kept; see §6d |

`demo_perception`'s Python is now bind-mounted on the module like the explorer's, so a
detector edit costs a sync + restart instead of a QEMU rebuild
(`tests/test_module_params_mount.py` locks it).

Tests: 78 `demo_navigation`, 36 `demo_perception`, 249 root.

## 6d. Two traps this session walked into — do not repeat

1. **`timed_out` is a suppression list, not an event tally.** Reading it as a count is what
   produced the false claim that `goal_timeout_s` 90 → 45 worked. Always count expiries in
   the per-goal CSV. The change in fact returned **zero** budget: R5 had 3 × 90 s, R6 had
   **6 × 45 s**, both 270 s. A fixed ceiling measures elapsed time, not progress.
2. **A persistence fix without an entry gate inverts the failure.** R5's
   `homing_persistence_s` correctly stopped homing abandoning on occlusion; shipped alone,
   it let R7's single 7.35 m observation own 520 of 600 s. The gate and the persistence
   belong in the same change.

## 6c. Ranked behind the blocker

**Homing entry gate.** Still only a partial measurement: `homing_entry_distance_m` of
**1.85 m** (R5) and **2.36 m** (R6), plus R2's 3.9 m glimpse. Record a distant marker
without cancelling exploration; add hysteresis and separate counters for "distant marker
ignored" and "homing entry accepted". `test_the_measurement_round_adds_no_homing_gate` is
now eligible for replacement. **Note this is much less urgent than it looked** — with the
persistence fix, a homing excursion no longer collapses the frontier set.

**Near-frontier stranding.** The near filter (`min_frontier_distance_m = 0.35`) runs
*before* the provisional filter, so a near-but-reachable frontier is invisible to the
recovery by construction. Do **not** lower it below `xy_goal_tolerance` 0.25 — 0.35 exists
to stop R4's instant-arrival loop. A bounded nudge/rotate is the right shape, only if it
reappears.

**Refuted, do not revisit:** frontier goal *clearance* is not the constraint (sweep 0.45 →
0.70 added no reachable goal); raising `goal_timeout_s` to 180 s (R3).

## 6e. Audit of 29/08 and what it changed

An independent review of the R7/R8 work found **two functional defects the tests did not
catch, because the tests encoded the same wrong semantics.** Both are fixed.

1. **"Three confirmations" were three timer cycles, not three observations.** The streak
   was incremented in `_tick` (1 Hz) while detections arrive in `_on_exit_pose`. A single
   pose stays fresh for `marker_stale_s` = 2 s, so **one bad frame satisfied all three
   confirmations** — the gate provided no hysteresis at all, and `marker_far_ignored`
   counted timer cycles rather than detections. Confirmation now advances only in
   `_on_exit_pose`, once per new message, deduplicated by `(frame_id, stamp_ns)` so
   transport duplication of one frame cannot confirm twice.
2. **The gate protected only the transition, not the target.** Every new observation
   overwrote `_exit_pose_map`, including during `homing_exit`, so a later partial view
   could move the target metres away mid-approach — R7's 1.27–7.94 m swing could still
   command homing after entry. Split into `_exit_candidate_pose_map` (latest raw
   observation, always published for the record) and `_exit_pose_map` (target accepted by
   the gate, **latched for the duration of the attempt** and cleared only when homing
   ends).

Also from the audit, all applied:

- **Tolerance edge**: `distance - stop < tolerance` missed the equality case, which Nav2
  also accepts without motion. Now `<= tolerance + 1e-9`, with a test on the boundary.
- **Overclaims corrected.** R7's 131 range pairs are consecutive 2 Hz samples on one
  trajectory and are **strongly autocorrelated** — not 131 degrees of freedom; the report
  now says so. R8's "same configuration as R5" was wrong (R8 also carries the
  connected-component detector, the 0.30/25 checker, the gate and the 45 s timeout); it now
  reads "same topology and footprint", and the 41.44 m vs 3.46 m mobility gap is labelled a
  difference between two runs, not an A/B.
- **Derived numbers are now reproducible.** `scripts/analyse_exploration.py` regenerates the
  range-ratio buckets and the progress-checker replay from the committed CSVs, with pure
  helpers unit-tested in `tests/test_analyse_exploration.py`:

  ```bash
  scripts/analyse_exploration.py ratio    docs/results/ml35-f5-exploration-r7.csv
  scripts/analyse_exploration.py progress docs/results/ml35-f5-exploration-r{5,6}.csv
  ```

  Verified to reproduce every quoted figure: 131 observations at ratio 1.055, the five
  buckets, and 2/9·29.1 s / 6/9·106.6 s / 7/9·188.3 s(1 false) / 9/9·426.8 s(9 false).
- **Instrumentation added.** The explorer publishes `marker_observations`,
  `marker_confirmations` and both candidate and accepted marker poses; the detector
  publishes `/demo/perception/maze_exit/diagnostics` (JSON on `String`, same shape as the
  explorer status) with region count, chosen bbox, sample count, **aspect ratio** and the
  widths of every region. Aspect ratio is the datum that separates "far" from "partially
  occluded" — the panel is square, so a chosen blob far from 1.0 is a partial view. All of
  it is now in `exploration_trial.py`'s columns.

**Still not done, and the audit is right that it gates R9:**

- **The 4 m gate does not resolve range ambiguity.** The 0–2 m band still carries 1.28 m of
  mean absolute error, so a genuinely distant marker can be underestimated *into* the gate.
  Confirmations do not remove a systematic error from persistent occlusion.
- **The "mobility collapse" was a recorder artefact, corrected 29/08.** The 0.004 vs
  0.050 m/s gap came from averaging `vx_mean_abs` over the whole recording, and R8's
  recorder kept sampling ~594 s past the explorer's 76.5 s failure. `scripts/
  exploration_trial.py` now reports `active_*` (first active sample to first terminal
  sample) alongside `recording_*` (full file); recomputed from the CSVs, R8's
  `active_vx_work_ratio` is 55.5 % (`active_vx_mean_abs` 0.036) against R5's 66.5 %
  (0.055) — a normal run-to-run gap, not an order-of-magnitude collapse. No diagnosis is
  owed here before R9; see `docs/results/ml35-f5-exploration-r8.md`.
- **The fiducial route is cheaper than previously argued.** An AprilTag placed **on the
  existing magenta panel at the same SDF pose** preserves the demo's premise, the opening
  geometry, `maze_escape_validator`'s constants and the scene cameras — the earlier
  objection assumed replacing the panel and moving it. Feed the fiducial pose into the
  current homing first; adopting Nav2's Docking Server is a separate, larger decision.
- Neither the gate nor the tolerance guard has been exercised in HIL. Unit-green is not
  field-validated.

## 7. Smoke protocol

Preconditions to verify and record every time:

1. `MODULE_HOST=<ip> scripts/module.sh sync`.
2. Restart the **sim first**, wait for `/clock`, then restart `nav` and `perception`.
   In that order — SLAM must start against the new clock or the map is stale.
3. Confirm fresh `/map` (small, ~88 × 86), spawn pose `(0.00, 0.04)`, lifecycle nodes
   `active [3]`, explorer `idle` with all counters zero.
4. Confirm the deployed `maze_explorer.py` is byte-identical host → module → container.
5. No arm64 rebuild for a smoke; the source mount exists for iteration.

Recording — passive, publishes nothing, writes the CSV **only at exit**, so give it a
path that exists *inside* the container and copy it out afterwards:

```bash
docker compose -f docker/compose.host.yml --profile tools up -d tools
docker compose -f docker/compose.host.yml exec -T tools \
    /usr/local/bin/entrypoint.sh python3 - /tmp/rN.csv --seconds 660 --hz 2 \
    --stop-on-escape < scripts/exploration_trial.py
docker cp docker-tools-1:/tmp/rN.csv docs/results/...
```

The `tools` container mounts only the DDS config — there is no writable repo mount, and
a bad path loses the whole run at the final write.

Then exactly one `/demo/exploration/start`, no manual goals, and only bounded filtered
log windows afterwards.

## 8. Official rebuild and final acceptance

After a smoke with `escaped = true`:

```bash
scripts/module.sh sync && scripts/module.sh build && scripts/module.sh up && scripts/module.sh verify
```

Require a clean tree, record the tested commit, confirm the rebuilt image and mounted
source are byte-identical to it. Then three cold starts: recreate simulation, SLAM, Nav2
and perception; load no saved map/graph; verify lifecycle and topics; one start; stop on
escape, fall, terminal failure or 600 s; preserve evidence before teardown. One failed
start keeps F5 open.

## 9. Closure

After 3/3, create `docs/results/ml35-f5-final-acceptance.md`, preserve all run and
per-goal CSVs, update `estado-fases.md` and `ml35-f5-busca-autonoma.md`, and separate
infrastructure, perception, exploration, homing and crossing evidence. State HIL-only
limitations and what does not validate a physical Go2.

Run root/package/HMI tests, both Compose validations and the sensitive-pattern scan.
Commit code with its measured evidence only after the result is known. Do not push or
rewrite history without explicit authorization.
