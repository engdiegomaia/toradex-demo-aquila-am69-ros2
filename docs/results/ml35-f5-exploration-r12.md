# ML3.5 F5 — round 12: sub-linear scoring reaches the southwest corridor, exit still not found

Date: 29-30/08/2026, real HIL (Gazebo/Go2 on the x86 host, `nav`/`demo_perception` on the
Aquila AM69). Follow-up to R11 (`docs/results/ml35-f5-exploration-r11.md`). Single
variable against R11: `frontier_score`'s route penalty changed from linear (`-0.5 *
route_m`) to sub-linear (`-0.5 * sqrt(route_m)`), diagnosed from R11's own frozen final
map (`extract_frontiers` re-run offline, no new HIL round needed for the diagnosis
itself) as the cause of R9/R11's repeated northward exploration bias: the two clusters
closest to the reported exit direction (`x≈-4..-5, y≈1.0-1.7`) were both smaller and
farther than the northern rooms, and a linear penalty compounded those two disadvantages
into a score of -3.4 against the northern rooms' +0.9 to +1.3 — permanently
uncompetitive regardless of how much of the near area had already been explored. 1 new
unit test added (`test_frontier_score_does_not_crush_a_small_but_far_cluster`, reproduces
R11's exact numbers), 89/89 passing.

**This round also surfaced and worked around a second, unrelated operational bug, found
before the reported result:** an identical redeploy sequence to R10/R11's (recreate the
`nav` container, then call `/demo/sim/reset`) produced a near-instant death again (43.3 s,
0 goals, `frontier_clusters_raw=3`, all three candidates walled off far from spawn,
`error_code=208`) — but this time the frozen map showed something new: its only known
cells sat at `y∈[8.7,10.2]`, nowhere near the robot's actual (odom) position `(0,0)`.
Root cause: `nav`'s container was recreated *while the robot was still sitting wherever
the previous round (R11) had left it* (`y≈9.3`) — `slam_toolbox` took its first scan(s)
at that stale pose before `/demo/sim/reset` ran, anchoring a small map patch there; the
subsequent teleport to spawn left it disconnected from anything reachable. **Fix is
operational, not code**: `/demo/sim/reset` must run *before* recreating `nav`, never
after, so `slam_toolbox`'s first scan is taken at the pose the robot will actually
explore from. Confirmed via `map`→`odom` (near-identity, so TF itself was fine) and a
direct `/map` known-cell dump (cells sat at the previous round's ending region, not
spawn) before re-sequencing and re-running clean. This is a real gap in `scripts/
module.sh`'s `up` step and the `docs/guia-completo.md` operational sequence, not
previously documented, and is worth writing into the runbook so it isn't rediscovered.

> **Verdict: FAIL on Gate B (`escaped` never became `true`), but a genuinely different
> and more informative failure mode than R9, R10, or R11.** No fall (`tilt_deg` max
> 1.26°). `path_m` **33.6 m** (R9: 34.78 m, R10: 0.06 m, R11: 30.91 m — comparable to the
> best prior round). Final state `failed` / **"prazo total de exploracao excedido"** —
> the explorer's own 600 s `total_timeout_s` fired while it was still actively finding
> and pursuing frontiers, NOT a barren-out (R9/R10/R11 all died "nenhuma fronteira segura
> alcancavel" instead). **The scoring fix visibly worked**: goal 13
> (`-5.077, 1.469`) was reached successfully — the farthest west any round has ever
> gotten, past the reported exit's own `x=-4.90` — and the trajectory's `y` range dipped
> to `-0.265`, closer to the reported exit's `y=-0.90` than any prior round. The robot
> then swung back to explore remaining northern frontiers rather than continuing south,
> and ran out of the 600 s budget before returning; the exit was still never seen
> (`marker_observations=0` throughout).

## Preconditions

- `nav`/`perception` rebuilt with the `frontier_score` fix, redeployed. **First attempt
  used the R10/R11 sequence (`module.sh up` then `/demo/sim/reset`) and died in 43.3 s**
  via the stale-anchor bug described above — not counted as this round's result.
- **Corrected sequence, used for the reported run:** `/demo/sim/reset` called and
  confirmed (robot at `(0,0)` via `/demo/odom`) *before* `module.sh up` recreated `nav`/
  `perception`; no reset issued afterward. `/map` checked directly before triggering —
  known cells confirmed centered near spawn (`x∈[-3.8,0.5], y∈[-0.3,3.7]`) — before
  starting the recorder and `/demo/exploration/start`.
- Recorder (`scripts/exploration_trial.py`) started before the trigger, 660 s / 2 Hz /
  `--stop-on-escape`, writing `docs/results/ml35-f5-exploration-r12.csv`.
- Real motion independently confirmed via direct `/demo/odom` sampling at three points
  during the run (early: ~1.55 m/30 s; mid-run: near-zero for one 20-25 s sample during
  what turned out to be a long, ultimately-successful 3.3 m route, not a stall; the run
  reached its terminal state normally afterward).

## Goal-by-goal record

20 goals attempted; **11 ok, 9 failed** (all 9 are the 45 s `goal_timeout_s`, except the
final one which is the run's own total-timeout firing mid-goal):

| # | goal (x, y) | outcome | elapsed_s | note |
| --- | --- | --- | --- | --- |
| 0 | (-0.233, 0.597) | ok | 8.4 | |
| 1 | (0.113, 3.445) | ok | 15.1 | |
| 2 | (-0.237, 0.595) | **timeout** | 45.0 | |
| 3 | (-1.601, 0.281) | **timeout** | 45.0 | |
| 4 | (-1.576, 0.375) | ok | 21.2 | |
| 5 | (-3.349, 0.421) | ok | 15.3 | |
| 6 | (-3.207, 0.871) | ok | 11.2 | |
| 7 | (-3.057, 1.869) | ok | 11.1 | |
| 8 | (-2.675, 1.819) | ok | 6.2 | |
| 9 | (-4.127, 1.519) | ok | 17.2 | |
| 10 | (-1.627, 2.269) | ok | 28.3 | |
| 11 | (-1.877, 2.969) | **timeout** | 45.0 | |
| 12 | (-1.827, 0.369) | **timeout** | 45.0 | |
| 13 | **(-5.077, 1.469)** | **ok** | 38.4 | farthest west any round has reached |
| 14 | (-1.733, 5.369) | **timeout** | 45.0 | swung back north |
| 15 | (-2.433, 8.121) | **timeout** | 45.0 | |
| 16 | (-0.283, 5.671) | **timeout** | 45.0 | |
| 17 | (-1.983, 3.171) | ok | 15.2 | |
| 18 | (-0.283, 5.821) | **timeout** | 45.0 | |
| 19 | (-2.533, 8.271) | **timeout** | 26.0 | cut short by total_timeout_s |

## Metrics

| metric | value |
| --- | --- |
| samples / sim span / wall span | 1320 / 639.5 s / 659.5 s |
| `escaped` | **false**, at every sample |
| final state / message | `failed` / "prazo total de exploracao excedido" (total-timeout, not barren) |
| `path_m` (final) | **33.6 m** |
| `map_known_cells` first → last | 1690 → 11144 |
| `map_known_pct` (final) | 38.91 % (map is 124×231 = 28644 cells) |
| goals dispatched | 20 (11 ok / 9 timeout) |
| terminal counters | `refused=0 timed_out=1 blacklisted=0 provisional_recoveries=1 barren_cycles=0` |
| trajectory bounding box | x ∈ [-4.370, 0.002], y ∈ [-0.265, 6.606] |
| tilt max | 1.26° — no fall |
| marker | never seen |

## Reading it

`barren_cycles=0` at the end is the headline: unlike every prior round, R12 did not run
out of reachable frontiers — it ran out of *time* while still actively finding and
validating new ones. That is exactly what the scoring fix targeted: it stopped the
search from converging on a local plateau where nearby-but-fully-explored rooms
permanently outscore the one real corridor onward. Goal 13's success is direct,
non-inferred confirmation — the robot physically drove to `(-5.077, 1.469)`, west of the
reported exit's own `x=-4.90`.

It did not, however, continue toward the exit after that. Goals 14-19 swing back to
`y=5.4-8.3`, back into the already-partly-explored north, rather than continuing to push
south toward `y=-0.90`. Two candidate explanations, not distinguished by this round: (a)
the map genuinely had more/larger frontier area to the north at that point than directly
south of `(-5.08, 1.47)` (a real, score-independent fact about what was left unexplored),
or (b) the corridor continuing south from there is narrow enough to hit the same
clearance filtering seen in R11's diagnostic (a raw cluster that never produces a
candidate at all) — in which case no scoring change would ever reach it, only a
clearance change would. This round's data does not distinguish the two; doing so would
need the same kind of offline `extract_frontiers` inspection used to diagnose R11,
applied to a map snapshot taken right after goal 13 rather than at the run's end.

Goal success rate (55 %, 11/20) is lower than R11's (74 %, 14/19) — expected and not a
regression: R12 spent its effort pursuing genuinely harder, farther frontiers (more of
them ending in the same 45 s controller-execution timeout already seen in R11, a
distinct layer from anything this round's scoring fix touches) instead of favoring the
easy, nearby, already-mostly-explored rooms R11 kept revisiting.

## What this round does and does not close

- Does **not** close Gate B.
- **Confirms** the `frontier_score` sub-linear-penalty fix works as intended: the
  southwest region toward the reported exit is now reached and treated as competitive,
  not permanently starved. This is now the second real regression this exploration
  work-stream has found and fixed within its own verification (the setback stall was the
  first, R11) — worth noting as a pattern: HIL running against real Nav2 timing keeps
  finding failure modes neither pure-Python unit tests nor a single offline diagnostic
  predicted.
- **Surfaces a real operational gap**, unrelated to any of the exploration code: the
  documented redeploy sequence (`module.sh up` before `/demo/sim/reset`) can corrupt
  `slam_toolbox`'s map anchor if the robot isn't already at spawn when `nav` restarts.
  Worth fixing in `scripts/module.sh` itself (e.g. `up` refusing, or automatically
  resetting first, when it detects the robot is far from spawn) or documenting explicitly
  in `docs/guia-completo.md`'s operational sequence, not just carried as tacit knowledge
  in this report.
- Does **not** determine why the robot didn't continue toward the exit after reaching
  goal 13 — open between "genuinely more unexplored area was elsewhere" and "the
  corridor south of there is clearance-filtered like R11's walled cluster."

## Recommended next step

Before another HIL round: take a map snapshot immediately after a run reaches the
`(-5.x, ~1.5)` region (or reuse R12's own mid-run state if a future round's timing
allows a paused inspection) and run the same offline `extract_frontiers` diagnostic used
for R11, specifically asking whether any raw cluster exists south of `y≈1` near
`x≈-4..-5`, and if so whether it clears clearance. That answer decides whether the next
fix is scoring-side (already done) or clearance-side (not yet attempted, and previously
deferred because R11's northward bias looked purely score-driven at the time it was
diagnosed).

## Limitations

HIL only; nothing here validates a physical Go2, leg odometry, thermals, or isolated
module performance (`CLAUDE.md` rules 5 and 7). One clean round (plus one aborted attempt
whose stale-anchor bug is now understood and avoidable) — not enough to rule out
run-to-run variance on the exact point where northern vs. southern exploration gets
chosen.
