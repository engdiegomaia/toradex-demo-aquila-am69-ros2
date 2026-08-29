# ML3.5 F5 — round 3: raising `goal_timeout_s` was measured and REJECTED

Date: 29/08/2026. Baseline for this round: round 2, `ml35-f5-exploration-r2.md`.
Topology: **real HIL** — Gazebo Harmonic on the x86 host, Nav2 + SLAM + perception on
the Aquila AM69, `ROS_DOMAIN_ID=69`, `rmw_cyclonedds_cpp`, `ROBOT_TYPE=quadruped`,
world `quadruped_maze11.sdf`.

Single variable against round 2: **`goal_timeout_s` 90 s → 180 s.**

Raw samples: `ml35-f5-exploration-r3.csv` (1320 rows at 2 Hz).
Per-goal records: `ml35-f5-exploration-r3-goals.csv`.

> **Verdict: REJECTED. The change made every measured quantity worse.** The value has
> been reverted to 90 s and a test now locks it there, so nobody repeats this.
> `test_the_goal_timeout_stays_at_the_value_that_was_measured_best` cites this file.

---

## 1. Round 3 against round 2

| metric | round 2 (90 s) | round 3 (180 s) | |
| --- | --- | --- | --- |
| distance travelled | 21.93 m | **4.26 m** | 5× worse |
| `map_known_cells` | 8915 | **3746** | |
| goals dispatched / reached | 13 / 7 | 6 / 5 | |
| `vx` work ratio | 41.71% | **8.60%** | |
| marker detected | yes, `sim_s` 402.9 | **no** | |
| `homing_exit` entered | yes | **no** | |
| `escaped` | false | false | |

Preconditions were met identically: fresh `/map` 88 × 86, robot at
`(-0.002, 0.042, 0.347)`, `escaped=false`, Nav2 `is_active` → `success=True`,
exploration started once at `sim_s` 54, no manual goal, RTF 0.973.

## 2. Why the reasoning was wrong

Round 2's three failures all stopped at **exactly 90.0 s**, and the slowest successful
goal took 65 s. That reads as "the ceiling is clipping slow but healthy traversals",
and the fix follows: raise the ceiling.

Round 3 shows the premise was false.

| # | goal | distance from robot | elapsed | outcome |
| --- | --- | --- | --- | --- |
| 4 | (-0.386, 0.258) | **~0.4 m** | **180.0 s** | `meta de fronteira expirou` |

A goal **0.4 m away** consumed the full 180 s. That is not a slow traversal — nothing
about 0.4 m takes three minutes at any speed this robot has shown. It is a stall, and
`goal_timeout_s` was the thing cutting stalls short. Doubling it simply made each stall
twice as expensive: one goal ate 28% of the entire 600 s budget, and the run never
recovered enough map to reach the far half of the maze.

Round 2's distant expiries and round 3's near expiry are the same phenomenon at
different distances. Distance was never the discriminator.

## 3. Criteria from §2.3, scored

| criterion | measured | verdict |
| --- | --- | --- |
| `frontier_extract_ms` p95 < 100 ms | 99.1 ms (p50 43.5) | pass — but only because the map stayed small |
| no infinite repetition of the same selection | 17 cycles, 18 path requests | pass |
| map and coverage growing | 1790 → 3746 cells | pass, weakly |
| no fall | max tilt 1.27°, `z` min 0.343 m | pass |
| no manual intervention | none after the single start call | pass |
| escape within 600 s | `escaped=false`, `failed` at `sim_s` ~410 | **FAIL** |

The extraction figure passing here is not an improvement — it passes because the robot
mapped 2.4× less. Do not read it as a win.

## 4. What was reverted

`goal_timeout_s` is back to **90.0**, the round 2 value, which remains the best measured
configuration. Round 2 is that value's evidence, so the revert needs no run of its own.

The other round 3 change — the invariant that three goals must fit inside
`total_timeout_s` — is kept. It is true regardless of the ceiling's value and it is
what would have flagged a jump to 300 s.

## 5. The variable that is actually indicated

Not the ceiling. **The permanence.** A goal that times out is written to the *hard*
blacklist and never returns, and in round 2 three such entries swallowed the four
frontier clusters that were still present at `sim_s` 570. A stall says something about
this attempt — the robot's pose, the current costmap, the plan it happened to get — not
something permanent about the frontier.

Two candidate shapes, one variable each, to be measured separately:

1. A timed-out frontier is suppressed for a bounded number of selection cycles rather
   than for the run, so it returns once the robot is somewhere else.
2. A timed-out frontier is retired only after it has timed out twice.

Ranked behind those, unchanged from round 2: the homing commit threshold — a 3.9 m
glimpse through the opening should probably not trigger `homing_exit`, or
`marker_stale_s = 2.0` should tolerate brief occlusion.
