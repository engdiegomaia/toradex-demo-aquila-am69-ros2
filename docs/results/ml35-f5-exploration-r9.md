# ML3.5 F5 — round 9: full 660 s budget spent, no fall, exit not found

Date: 29/08/2026, real HIL (Gazebo/Go2 on the x86 host, `nav`/`demo_perception` on the
Aquila AM69). Same session as the fiducial-marker addition, the footprint-polygon
default, and the homing-entry hysteresis fix — see `docs/ml35/estado-fases.md`
(29/08 entries) and the git history for that date.

**This file replaces an earlier, coarse version of the R9 report.** The first attempt at
this round used `scripts/exploration_trial.py` against the `tools` container before that
container had a bind mount for `docs/results/`; the recorder ran the whole 660 s in memory
and then crashed on its single, end-of-run `write_csv()` call, losing every sample. That
mistake is described in full, and left in git history, in the version this file replaces
(§0 of the prior draft). The root cause (a service-level `volumes:` key in
`compose.host.yml` silently replacing rather than merging the `*common` anchor's mount
list) is now fixed, `tools` has a real `docs/results` bind mount, and this run was
re-triggered from a fresh `/demo/sim/reset` + `nav` container restart with the recorder
launched correctly beforehand. `ml35-f5-exploration-r9.csv` (1320 samples) and
`ml35-f5-exploration-r9-goals.csv` (14 goals) are real, complete, and on disk.

> **Verdict: FAIL on Gate B (`escaped` never became `true`), but a clean, complete,
> non-destructive run.** No fall at any instant (`tilt_deg` max 1.53°, `z` stayed in
> 0.343–0.369 m throughout). The robot spent the full 660 s budget, reached 48.6 % map
> coverage, and ended in `failed` / "nenhuma fronteira segura alcancavel" after both
> `provisional_recoveries` slots were used. The exit marker was never seen
> (`marker_visible` never `true` in any of the 1320 samples) — the frontier search never
> brought the robot near the opening at `x=-4.90, y=-0.90`; final pose was `(-1.68, 7.49)`,
> well to the north.

## Preconditions

- `/demo/sim/reset` → robot at spawn; `nav` container restarted immediately after for a
  clean `slam_toolbox` map/pose-graph.
- `/demo/exploration/start` called once (`success=true`).
- Recorder (`scripts/exploration_trial.py`) started separately, before the trigger, at
  660 s / 2 Hz / `--stop-on-escape`, writing into the now-mounted
  `docs/results/ml35-f5-exploration-r9.csv`.

## Goal-by-goal record

14 goals attempted; **5 ok, 9 failed** (all 9 failures are the same message, "meta de
fronteira expirou" — the ~45 s per-goal timeout):

| # | goal (x, y) | sent `sim_s` | dur | outcome |
| --- | --- | --- | --- | --- |
| 0 | (-0.03, 1.19) | 2234.0 | 11.1 s | **ok** |
| 1 | (-3.33, 0.64) | 2246.0 | 45.0 s | failed — expirou |
| 2 | (-2.24, 0.30) | 2293.0 | 11.1 s | **ok** |
| 3 | (-3.35, 0.38) | 2305.0 | 8.2 s | **ok** |
| 4 | (-3.15, 1.37) | 2314.0 | 45.0 s | failed — expirou |
| 5 | (-1.79, 2.22) | 2361.0 | 29.2 s | **ok** |
| 6 | (-2.99, 0.37) | 2391.0 | 45.0 s | failed — expirou |
| 7 | (-2.15, 3.12) | 2438.0 | 45.0 s | failed — expirou |
| 8 | (-2.15, 3.12) | 2485.0 | 45.0 s | failed — expirou (same coords as #7, retried) |
| 9 | (-2.75, 7.97) | 2532.0 | 42.2 s | **ok** |
| 10 | (-0.75, 5.57) | 2575.0 | 45.0 s | failed — expirou |
| 11 | (-4.35, 7.37) | 2622.0 | 45.0 s | failed — expirou |
| 12 | (-0.70, 5.72) | 2669.0 | 45.0 s | failed — expirou |
| 13 | (-4.15, 7.82) | 2716.0 | 45.0 s | failed — expirou (last goal; run then went barren) |

Note goals 7 and 8: the identical `(-2.15, 3.12)` frontier was re-selected and timed out
twice in a row before the explorer moved on. 9 of 14 goals (64 %) ended in the 45 s
timeout rather than arrival — a markedly worse ratio than R8's 1-of-3.

## Metrics

| metric | value |
| --- | --- |
| samples / sim span / wall span / RTF | 1320 / 634.8 s / 659.5 s / 0.962 |
| `escaped` | **false**, at every sample |
| final `state` / `message` | `failed` / "nenhuma fronteira segura alcancavel" |
| `path_m` (final) | 34.78 m |
| `map_known_cells` / `map_known_pct` | 1711 → 11636 / 22.4 % → 48.6 % |
| `recording_vx_work_ratio` (full 660 s) | 54.6 % (`mean_abs` 0.038) |
| **`active_vx_work_ratio`** (558.5 s from first non-idle sample to the terminal sample) | **64.5 %** (`active_vx_mean_abs` 0.045) |
| goals total / ok / failed | 14 / 5 / 9 |
| terminal counters | `refused=0 timed_out=2 blacklisted=0 provisional_recoveries=2 barren_cycles=10` |
| `marker_visible` | never `true`; `marker_observations=0`, `homing_entries=0` |
| `frontier_extract_ms` | mean 150.3 ms, max 302.5 ms — both over the previously-flagged 100 ms budget |
| tilt max / z range | 1.53° / 0.3433–0.369 m — **no fall at any of the 1320 samples** |

## Reading it honestly

- **Gate B is not met.** `escaped` stayed `false` for the entire budget; the robot never
  got near the opening.
- **This run's `active_vx_work_ratio` (64.5 %) is in the same range as R5's and R8's** —
  mobility itself is not the story here. The story is goal outcome: 64 % of goals expired
  rather than arrived, roughly double R8's expiry rate (though R8 only had 3 goals total,
  so that comparison is weak evidence, not a controlled A/B).
- **The two repeated timeouts at the identical `(-2.15, 3.12)` frontier (goals 7–8)** and
  the generally high expiry rate are consistent with — but do not on their own prove — a
  live behavioural report from watching this exact run: the robot appears to peel off from
  a path and turn back well before it reaches a wall directly ahead of it, which would
  read in this data exactly as "goal sent, never arrives, times out at 45 s" for any
  frontier placed near a wall or corner. See "Follow-up" below; this run used the
  pre-existing frontier-goal placement and does not itself isolate that cause.
- **No fall, at any instant.** `tilt_deg` never exceeded 1.53° and `z` stayed within 3 cm
  of nominal standing height for the full 660 s — the mobility base of the demo continues
  to be solid even across many goal failures.
- **`frontier_extract_ms` (mean 150 ms, max 302 ms)** confirms R9's first (data-lost)
  attempt was not a one-off: this run's own extraction cost is consistently over the
  100 ms budget mentioned in earlier reports, now backed by a full distribution rather
  than one sample.

## Follow-up: a candidate cause, diagnosed but not yet tested

While this run was in progress, live observation reported the robot abandoning a
forward path too early on encountering a wall ahead, missing openings and side
corridors. Reading `frontier.py`'s `extract_frontiers`, a frontier candidate's goal point
must clear **0.45 m** from any occupied cell in every direction (`has_clearance`,
`clearance_m` default) before it is accepted at all — larger than the robot's own
footprint half-length (0.37 m, `nav2_params_go2.yaml`). Near a narrow opening or a
corner, that 0.45 m radius check can reject the closest usable candidate outright, or
push the accepted goal further from the wall than necessary, which would show up in this
kind of data exactly as elevated goal-timeout rates near dead ends and corners.

Made this session, **not yet exercised in any HIL round**: `frontier_wall_clearance_m`
is now a declared parameter on `maze_explorer.py` (default lowered from the previous
hardcoded 0.45 to 0.38 m, matched to the footprint half-length plus a small margin), threaded
into the `extract_frontiers(..., clearance_m=...)` call. This is a single, isolated
variable change against R9's configuration — the next round should run with it and
nothing else changed, per this project's one-variable-per-round discipline. It has not
been rebuilt into the `nav` container or deployed to the module; R9 above ran entirely on
the prior 0.45 m default.

## What this round does and does not close

- Does **not** close Gate B: `escaped=false`.
- Does **not** exercise the fiducial detector in motion (never in range of the marker).
- Does confirm the recorder fix: a full, complete CSV and goals CSV survive this time,
  with no data loss.
- Provides real, if inconclusive, evidence for the goal-timeout pattern that motivated the
  `frontier_wall_clearance_m` change above; that change is a hypothesis this run helps
  motivate, not one it tests.

## Limitations

HIL only; nothing here validates a physical Go2, leg odometry, thermals, or isolated
module performance (`CLAUDE.md` rules 5 and 7). One round.
