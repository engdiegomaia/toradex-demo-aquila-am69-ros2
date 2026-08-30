# ML3.5 F5 — round 11: multi-candidate + recessed-endpoint fix, real progress, no fall, exit not found

Date: 29-30/08/2026, real HIL (Gazebo/Go2 on the x86 host, `nav`/`demo_perception` on the
Aquila AM69). Follow-up to R9 (`docs/results/ml35-f5-exploration-r9.md`) and R10
(`docs/results/ml35-f5-exploration-r10.md`). Three variables against R10, all from the
same review of R9/R10's failure modes:

1. **Per-cluster multi-candidate retry** (`frontier.py`'s `Frontier.alternates`,
   `frontier_max_alternates=2`, `frontier_alternate_spacing_m=0.25`) — a cluster is no
   longer abandoned over a single unreachable point.
2. **Recessed navigation endpoint along the validated plan** (`_setback_point()`,
   `frontier_endpoint_setback_m=0.40`) — the commanded pose is walked back from the
   frontier's own endpoint along the path Nav2 already proved reachable, instead of the
   raw (sometimes too-close-to-a-wall) frontier point.
3. **Map-generation-gated provisional recovery** (`_last_provisional_map_seq`) — a
   suppressed goal can only be retried in a later map generation than the one that
   suppressed it, closing R9's goals-7/8 identical-retry pattern.

**This round required a mid-flight fix.** The first attempt (same day, before this one)
stalled completely: the robot sat motionless for minutes on a byte-identical map,
`ComputePathToPose` kept succeeding, but no `NavigateToPose` ever produced real motion.
Root cause: a setback point recessed 0.40 m from a short path landed only ~0.23 m from
the robot's own current pose — inside Nav2's `xy_goal_tolerance` (0.25 m,
`nav2_params_go2.yaml`), so `SimpleGoalChecker` called the goal reached without the robot
moving at all. This is the exact failure class R4 already fixed once for a frontier's own
endpoint (`min_frontier_distance_m`, see `test_a_frontier_inside_the_goal_tolerance_is_
never_dispatched`); the new setback point reintroduced it through a path that filter never
covers. Fixed by giving `_setback_point()` a `min_travel_m` floor
(`nav_goal_tolerance_m` + 0.10 m): a setback candidate this close to the path's own start
is discarded (`None`) and the caller falls back to the original, already-validated
endpoint. 6 new unit tests added (`test_setback_point_*` in `test_maze_explorer.py`,
88/88 passing). `nav`/`perception` rebuilt and redeployed a second time with this fix
before the round reported here was run. No CSV exists for the stalled attempt (it was
cancelled and the recorder killed before its single end-of-run write); the stall itself
was confirmed directly via `/demo/odom` (motionless to sub-millimeter precision across a
78 s window) and `/demo/exploration/status` (`candidate_point_x/y` frozen, `path_requests`
climbing without `candidates_checked` ever advancing).

> **Verdict: FAIL on Gate B (`escaped` never became `true`), but the healthiest round so
> far by a wide margin.** No fall at any instant (`tilt_deg` max 1.46°, `z` stayed in
> 0.334–0.367 m throughout). `path_m` **30.91 m** (R9: 34.78 m over the same 660 s budget,
> R10: 0.06 m). Map coverage **51.6 %** (15422/29882 cells; R9: 48.6 %, R10: 9.13 %). Of
> 19 goals attempted, **14 succeeded (74 %)** — R9's rate over 14 goals was 5/14 (36 %).
> Final state `failed` / "nenhuma fronteira segura alcancavel" after 10 barren cycles and
> exactly 1 provisional-recovery use. The exit marker was never seen (`marker_visible`
> never `true`, `marker_observations` 0 throughout) — final pose `(-1.31, 9.32)`, and the
> robot's full trajectory only ever reached `x` in `[-3.66, 0.28]` and `y` in
> `[-0.36, 9.99]`. R9's report places the actual opening at `x=-4.90, y=-0.90`
> (unverified in this round, carried over from that report) — **both R9 and R11 explored
> heavily northward (`y` up to 9.3–10.0) and never got close to that region.** That is a
> real, repeated pattern across two rounds now, not a one-off.

## Preconditions

- `nav`/`perception` images rebuilt natively on the module and redeployed
  (`scripts/module.sh sync && scripts/module.sh build && scripts/module.sh up`) with the
  `_setback_point` `min_travel_m` fix in place. Live parameters confirmed before
  triggering: `frontier_wall_clearance_m=0.38`, `frontier_max_alternates=2`,
  `frontier_alternate_spacing_m=0.25`, `frontier_endpoint_setback_m=0.40`.
- `/demo/sim/reset` called twice (once right after redeploy, once immediately before the
  trigger, for tight timing) — both returned `success=true`, robot at spawn.
- Recorder (`scripts/exploration_trial.py`) started before `/demo/exploration/start`, at
  660 s / 2 Hz / `--stop-on-escape`, writing into
  `docs/results/ml35-f5-exploration-r11.csv` (repo-root-relative path inside the `tools`
  container's `docs/results` bind mount).
- `/demo/exploration/start` called once (`success=true`, "busca iniciada").
- Real motion was independently confirmed twice during the run via direct `/demo/odom`
  sampling (not just `ComputePathToPose` success) — the exact check that would have
  caught the earlier stall sooner: ~1.37 m in 30 s early in the run, ~1.57 m in 20 s at
  elapsed_s ~420, both while `/demo/exploration/status` showed genuine `navigating`/
  `selecting` progression rather than a frozen candidate.

## Goal-by-goal record

19 goals attempted; **14 ok, 5 failed** (all 5 failures are the 45 s `goal_timeout_s`
expiring, message "meta de fronteira expirou" — a Nav2-execution timeout, not a planner
refusal; `refused_final=5` in the summary counts a *different* thing, clusters whose
`ComputePathToPose` never returned a usable plan and were retired):

| # | goal (x, y) | outcome | elapsed_s | note |
| --- | --- | --- | --- | --- |
| 0 | (-0.183, 0.705) | ok | 8.2 | |
| 1 | (-0.133, 2.396) | ok | 11.1 | |
| 2 | (-0.183, 0.696) | **timeout** | 45.0 | |
| 3 | (-0.833, 0.096) | **timeout** | 45.0 | |
| 4 | (-3.246, 0.346) | ok | 32.1 | westernmost point reached |
| 5 | (-3.397, 0.896) | **timeout** | 45.0 | |
| 6 | (-3.047, 1.746) | ok | 24.2 | |
| 7 | (-4.236, 1.796) | ok | 11.3 | |
| 8 | (-1.586, 2.246) | ok | 32.2 | |
| 9 | (-1.636, 3.096) | ok | 32.3 | |
| 10 | (-1.386, 5.246) | ok | 14.5 | |
| 11 | (-1.986, 3.546) | ok | 12.4 | |
| 12 | (-1.636, 5.246) | ok | 29.3 | |
| 13 | (-0.186, 5.596) | ok | 27.3 | |
| 14 | (-0.386, 6.646) | **timeout** | 45.0 | |
| 15 | (-0.636, 9.996) | ok | 30.3 | northernmost point reached |
| 16 | (-1.489, 9.946) | ok | 13.2 | |
| 17 | (-2.689, 9.896) | ok | 18.3 | |
| 18 | (-1.667, 9.896) | **timeout** | 45.0 | last goal before barren-out |

No back-to-back identical-coordinate retry appears anywhere in this list (R9's goals 7-8
pattern) — consistent with the map-generation-gated provisional recovery working as
intended, though 19 goals across 1 run is not a strong statistical claim on its own.

## Metrics

| metric | value |
| --- | --- |
| samples / sim span / wall span | 1320 / 630.1 s / 659.5 s |
| `escaped` | **false**, at every sample |
| final state / message | `failed` / "nenhuma fronteira segura alcancavel" |
| `path_m` (final) | **30.91 m** |
| `map_known_cells` first → last | 1707 → 15422 (map is 134×223 = 29882 cells) |
| `map_known_pct` (final) | **51.61 %** |
| goals dispatched | 19 (14 ok / 5 timeout) |
| terminal counters | `refused=5 timed_out=0 blacklisted=0 provisional_recoveries=1 barren_cycles=10` |
| `frontier_clusters` (final) / `frontier_clusters_raw` (final) | 6 / 11 |
| state time breakdown | `idle` 5.2 s, `selecting` 31.1 s, `navigating` 523.4 s, `failed` 70.4 s |
| tilt max / z range | 1.46° / 0.3344–0.367 m — no fall |
| trajectory bounding box | x ∈ [-3.658, 0.284], y ∈ [-0.361, 9.995] |
| marker | never seen (`marker_visible` false throughout, `marker_observations=0`) |

## Reading it

The three fixes did what they were built for. `navigating` occupied 523.4 s of the
554.5 s the robot was actively doing anything (idle + failed excluded) — a 94 % duty
cycle, vastly higher than R9's, where 64 % of goals ended in a 45 s dead timeout. The
`frontier_clusters_raw` vs `frontier_clusters` telemetry pair (added this round) shows
the filter is doing real work throughout — e.g. 11 raw clusters filtered to 6 near the
end — not silently discarding everything down to one, as R10's frozen snapshot showed.
`refused=5` (whole clusters retired, all points exhausted) and `timed_out=0` in the
final counters — combined with 5 goal-level timeouts in the goals table — says the
*planner* essentially never got stuck this round (candidates were found and validated
promptly, `frontier_extract_ms` p50 258.5 ms / p95 476.3 ms), and the remaining failures
are a *controller/execution* problem (MPPI or the physical approach taking longer than
45 s), a distinct layer from anything this round's fixes touched.

The run ended the same way R9 did — barren-out, not a timeout or a fall — after
exhausting reachable frontiers in the region it explored. **The open question from R9
repeats here nearly unchanged**: the frontier scoring/selection consistently walks the
robot north (toward `y≈10`) and never meaningfully toward the reported opening at
`(-4.90, -0.90)`, southwest of spawn. Two consecutive rounds with the same directional
bias is enough to treat this as a real pattern, not run-to-run noise — though nothing in
this diagnostic isolates *why* (candidate distribution from `extract_frontiers`, the
`frontier_score` distance term, or genuine maze topology funneling exploration that way)
without directly inspecting frontier candidates against the maze's connectivity, which
this round did not attempt.

## What this round does and does not close

- Does **not** close Gate B.
- **Confirms** all three of this round's fixes work as designed under real HIL load:
  alternates get generated and consumed (per-goal Nav2 status alone can't show this, but
  the healthy 74 % success rate and absence of R10's single-point-cluster-death pattern
  are consistent with it), the setback mechanism drives real, substantial motion once the
  `min_travel_m` floor was added (confirmed directly via `/demo/odom`, not inferred), and
  no identical-coordinate back-to-back retry occurred.
- **Adds one real regression, found and fixed within this same round**: the setback
  mechanism's initial version could stall the robot completely by landing inside Nav2's
  own goal tolerance. Caught by a live HIL run, not by the unit tests written earlier in
  the session (none of them exercised a short real Nav2 path) — six tests now cover this
  specifically (`test_setback_point_discarded_when_it_would_fall_inside_goal_tolerance`
  and neighbors).
- Does **not** explain the repeated northward exploration bias — open across two rounds
  now (R9, R11), and is the most likely reason Gate B keeps failing on maze coverage
  grounds rather than on any of the mechanisms this round touched.

## Recommended next step

Investigate the northward bias directly: dump `extract_frontiers`' candidate list (all
clusters, not just the selected one) at a few points mid-run and check whether the
south/southwest region is producing candidates at all (a real dead end / disconnected
region in the map at that point) or producing candidates that `frontier_score` simply
ranks lower every time (a scoring bias, fixable in `frontier_score` or in cluster
selection order). Do not touch `frontier_wall_clearance_m`,
`frontier_endpoint_setback_m`, or the multi-candidate mechanism again based on this round
— all three are working as intended and are not implicated in this pattern.

## Limitations

HIL only; nothing here validates a physical Go2, leg odometry, thermals, or isolated
module performance (`CLAUDE.md` rules 5 and 7). One clean round (plus one stalled,
uncounted attempt fixed mid-session) — not enough to rule out run-to-run variance on the
74 % goal success rate or the exact barren-out point.
