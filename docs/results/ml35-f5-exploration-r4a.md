# ML3.5 F5 — round 4a: the instant-arrival loop is gone; provisional suppression deadlocks

Date: 29/08/2026. Baselines: round 4 (`ml35-f5-exploration-r4.md`) for the loop, round 2
(`ml35-f5-exploration-r2.md`) for exploration reach. Topology: **real HIL** — Gazebo
Harmonic on the x86 host, Nav2 + SLAM + perception on the Aquila AM69,
`ROS_DOMAIN_ID=69`, `rmw_cyclonedds_cpp`, `ROBOT_TYPE=quadruped`, world
`quadruped_maze11.sdf`.

Single variable against round 4: **`min_frontier_distance_m = 0.35`.** Candidates closer
to the robot than that are dropped during selection, after suppression and before the
nearest-first sort. The drop is relative to the current pose, recomputed every cycle,
and never written to `_blacklist`, `_refused` or `_timed_out`. Count published as
`near_frontiers_skipped`.

Raw samples: `ml35-f5-exploration-r4a.csv` (1320 rows at 2 Hz).
Per-goal records: `ml35-f5-exploration-r4a-goals.csv`.

> **Verdict: FAIL on escape. The round 4 loop did not recur — 565 selection cycles
> became 14 and 4508 path requests became 8 — but `near_frontiers_skipped` stayed 0 all
> run, so the filter itself never fired and this run does not attribute that improvement
> to it.**
>
> The run died at `sim_s` 249 on something now seen three times: **every frontier
> cluster suppressed while real frontier is still present.** This time the suppression
> was entirely provisional — `blacklisted` 0, `refused` 2, `timed_out` 1 — which exposes
> a **deadlock in the release rule**: provisional suppression is lifted only by reaching
> a frontier, and there was no unsuppressed frontier left to reach.

---

## 1. Against round 4 and round 2

| metric | round 2 | round 4 | **round 4a** |
| --- | --- | --- | --- |
| distance travelled | 21.93 m | 0.82 m | **5.86 m** |
| `map_known_cells` | 1710 → 8915 | frozen 2669 | 1790 → **5133** |
| `selection_cycle` | 10 | 565 | **14** |
| `path_requests` | 33 | 4508 | **8** |
| time in `selecting` | ~13 s | 610 s | **15 s** |
| time in `navigating` | most of run | 6.5 s | **184 s** |
| `vx` work ratio | 41.71% | 1.01% | 11.63% |
| `blacklisted` at the end | 3 | 0 | **0** |
| `near_frontiers_skipped` | — | — | **0** |
| `escaped` | false | false | false |

`blacklisted` 0 with a goal that did expire is **R4 behaving as designed**: the timeout
went to `_timed_out`, not to the hard list. That much of R4 is now confirmed in the
field, not only in unit tests.

## 2. The run, and where it stopped

| `sim_s` | state | pose | what happened |
| --- | --- | --- | --- |
| 54.2 | `selecting` | (0.00, 0.04) | 3 clusters, 669 cells |
| 55.2 – 64.5 | `navigating` | → (-0.10, 0.90) | goal 0 reached in 9.1 s |
| 65.5 – 147.3 | `navigating` | → (-2.68, 0.27) | goal 1 reached in 82.1 s, 2.6 m |
| 148.3 – 238.5 | `navigating` | → (-3.47, 0.43) | goal 2 **expired at 90.0 s** → `_timed_out` = 1 |
| 239.4 | `selecting` | (-3.47, 0.43) | `frontier_count` 1 of 3 clusters |
| 240.4 | `selecting` | (-3.47, 0.43) | `frontier_count` **0** of 3 clusters — 2 more refused |
| 249.3 | `failed` | (-3.47, 0.43) | 10 barren cycles |

Goal 1 covered 2.6 m in 82.1 s, which is real progress and well inside the 90 s ceiling
— further evidence that 90 s is the right value and round 3 was correctly rejected.

## 3. The deadlock, stated plainly

At `sim_s` 240 the state was:

- `frontier_clusters` = 3, `frontier_cells` = 182 — **real frontier exists**;
- `frontier_count` = 0 — **none permitted**;
- `blacklisted` = 0, `refused` = 2, `timed_out` = 1 — **all suppression is provisional**;
- `near_frontiers_skipped` = 0 — the new filter is not involved.

Both provisional lists are released by exactly one event: `_on_nav_result` returning
success for some other frontier. So:

> to clear the suppression the robot must reach a frontier,
> and to reach a frontier it needs one that is not suppressed.

Once the provisional lists cover every cluster, nothing can ever lift them. The design
is strictly better than the hard blacklist it replaced — it *can* recover, where the
hard list could not — but its release condition is unreachable in precisely the state
that matters.

This is the same shape of ending as round 1 and as round 2's `sim_s` 570, with a
different list doing the suppressing each time. Three runs, one structural cause.

## 4. Criteria from §2.3, scored

| criterion | measured | verdict |
| --- | --- | --- |
| no success–selection cycle on the same coordinate | 14 cycles, 8 path requests | **pass** |
| material displacement after the start | 5.86 m, robot reached x = -3.47 | **pass** |
| continuous map growth | 1790 → 5133 cells | **pass** |
| `selection_cycle` / `path_requests` far below round 4 | 14 / 8 against 565 / 4508 | **pass** |
| `selecting` does not dominate the run | 15 s of 650 s | **pass** |
| searching for new frontiers | 3 goals, 2 reached, then deadlocked | **partial** |
| new autonomous marker detection | none | **FAIL** |
| escape within 600 s | `escaped=false` | **FAIL** |
| `frontier_extract_ms` p95 < 100 ms | 146.9 ms | **FAIL** |

No fall: max tilt 0.67°, `z` min 0.347 m. No manual intervention after the single start
call. RTF 0.985. Preconditions identical to rounds 2–4.

## 5. What this does not validate

- **R4a's own filter.** `near_frontiers_skipped` was 0, so the filter never fired. The
  absence of round 4's loop is not attributable to it in this run. The filter removes a
  demonstrated failure mode and is covered by unit tests; keep it, but do not claim this
  run as its field evidence.
- **R4's timeout policy in full.** The timeout did fire and did stay out of the hard
  list, which is the half that matters most. The release-on-arrival half is what §3
  shows to be insufficient.
- Nothing about a physical Go2 — Gazebo on the x86 host (`CLAUDE.md` rule 7).
- Nothing about homing or crossing; the marker was never seen.

## 6. Run-to-run variance is now itself a finding

Rounds 2, 3, 4 and 4a started from byte-identical preconditions — same world, same
restart sequence, same fresh 88 × 86 map, same start pose to within 2 mm — and produced
four different failure modes and distances of 21.93, 4.26, 0.82 and 5.86 m. Single runs
can show that a specific defect is *present*; they cannot yet attribute an improvement,
because the spread between runs is larger than the effect being measured.

That has a direct consequence for the acceptance protocol: 3/3 cold starts is the right
bar, and no single smoke should be read as proof that a change worked.

## 7. The variable indicated next

**Break the deadlock at its only reachable point: the barren cycle.** When a selection
finds real clusters but permits none, and the suppression responsible is *entirely
provisional*, clear the provisional lists and retry that selection once before counting
the cycle as barren. The hard blacklist is untouched, so genuinely bad frontiers stay
out; a frontier that was merely unreachable-from-there gets one more chance from
wherever the robot now stands.

It is one variable, it is where all three deadlocked runs ended, and it cannot
resurrect the round 1 livelock, because a refusal that repeats is re-suppressed
immediately and the barren counter still runs.

Behind it, unchanged: R4b (measure real displacement before clearing `_barren_cycles`)
is still not triggered — no success-without-movement occurred in this run. The homing
entry threshold is still not reachable, since exploration has not reached the marker
again. `frontier_extract_ms` breached its criterion once more at 146.9 ms.
