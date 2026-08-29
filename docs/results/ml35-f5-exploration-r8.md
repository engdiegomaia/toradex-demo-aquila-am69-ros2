# ML3.5 F5 — round 8: died in the barren mode before reaching anything under test

Date: 29/08/2026. Same HIL topology and arm B costmap as R7. Single variable against R7:
the **homing entry gate** (`homing_max_distance_m = 4.0`,
`homing_confirm_observations = 3`, new `marker_far_ignored` counter).

Preconditions verified: fresh `/map` 88 × 86, `maze_explorer.py` md5-identical host →
module → container (`eb255c9b…`), gate parameters read back live off the node
(4.0 / 3), explorer `idle` with every counter zero. One start accepted; a second Trigger
was issued by mistake and **rejected as a no-op** (`busca ja esta em andamento`), so
exactly one start is in force.

> **Verdict: FAIL, and INCONCLUSIVE for everything it was meant to test.** The run died at
> `elapsed` 76.5 s in the barren mode. The exit marker was never detected
> (`homing_entries = 0`, `marker_far_ignored = 0`), so neither the entry gate nor the
> connected-component range fix was exercised. The progress-checker change went unjudged
> for the second round running.

## What happened

Per-goal record — the authority, and it corrects a first reading of this round that counted
two expiries from one status message repeating across two samples:

| # | phase | sent `sim_s` | dur | outcome |
| --- | --- | --- | --- | --- |
| 0 | exploration | 70.0 | 7.3 s | **ok** — fronteira alcancada |
| 1 | exploration | 78.0 | 8.1 s | **ok** — fronteira alcancada |
| 2 | exploration | 87.0 | **45.0 s** | failed — meta de fronteira expirou |

Then, within roughly ten seconds of that single expiry: the provisional recovery released
what it could, the restored candidates were refused, `frontier_count` reached zero, and the
barren limit ended the run.

| metric | value |
| --- | --- |
| samples / sim span / RTF | 1400 / 670.0 s / 0.958 |
| `escaped` | **false** |
| `final_state` / `final_message` | `failed` / "nenhuma fronteira segura alcancavel" |
| **`path_m`** | **3.46 m** |
| `map_known_cells` | 1715 → 3665 |
| `recording_vx_work_ratio` (full 670 s window) | 6.3 % (`recording_vx_mean_abs` 0.004) |
| **`active_vx_work_ratio`** (76.6 s the explorer was actually active) | **55.5 %** (`active_vx_mean_abs` 0.036) |
| goals total / ok / failed / homing | 3 / 2 / 1 / **0** |
| terminal counters | `refused=2 timed_out=0 blacklisted=0 provisional_recoveries=1 barren_cycles=10` |
| tilt max / z min | 1.06° / 0.3462 — no fall |

**Correction (29/08, second pass): the "10x mobility collapse" first reported here was a
recorder artefact, not physical.** The explorer failed at `elapsed` 76.5 s and entered
`failed`, but the recorder (`scripts/exploration_trial.py`) kept sampling for the full
670 s window, so ~594 s of post-failure zeros diluted `vx_mean_abs` by roughly 8.8x. With
`scripts/exploration_trial.py` now separating `active_*` (samples from the first active
state to the first terminal sample, inclusive) from `recording_*` (the full file),
recomputed directly from `ml35-f5-exploration-r8.csv`:

| | R5 (`active_*`) | R8 (`active_*`) |
| --- | ---: | ---: |
| `active_vx_work_ratio` | 66.5 % | 55.5 % |
| `active_vx_mean_abs` | 0.0554 | 0.0357 |
| `active_duration_s` | 600.2 s | 76.6 s |

R8's active mobility is **lower** than R5's (55.5 % vs 66.5 % work ratio, 0.036 vs
0.055 m/s), but that is a normal-sized gap, not the order-of-magnitude collapse the
full-window numbers implied. R8 travelled 3.46 m at a 55.5 % active work ratio; R5
travelled 41.44 m over a full 600 s active window at 66.5 %. R5 and R8 share the HIL
topology and the arm B footprint, but they are **not the same configuration** — R8 also
carries the connected-component detector, the progress checker at 0.30/25, the homing
entry gate and `goal_timeout_s` 45. This is a difference between two runs, not a
controlled A/B. R8 was cut short by the barren-mode death at 76.5 s, not by a mobility
regression — its active mobility, while it ran, tracked R5's within the range this project
has already recorded as ordinary run-to-run variance.

## Reading it honestly

This is the R4b signature: refusals appearing right after the robot walks into a pose it
cannot plan out of. Arm B made that rare rather than impossible — R5, R6 and R7 all
finished with `refused` ≤ 2 and survived, because they still had frontiers left. R8 had
only **two** frontiers when the refusals landed, so refusing both emptied the set — and it
had only 76.6 s of active exploration before dying (3.46 m at a 55.5 % active work ratio,
in line with R5's), so it simply had not had time to build up more frontiers.

That points at a fragility rather than a new defect: the barren limit is absolute, but the
frontier set size at the moment of a refusal is luck. **One round cannot separate that from
ordinary variance** — this project has already recorded six different endings from
byte-identical preconditions (`implementation-handoff.md` §1).

Two things are worth noting without over-claiming:

- The 45 s ceiling fired exactly on schedule at 19.5 + 45 = 64.5 s, so the parameter is
  live and behaving.
- The progress checker (0.30 m / 25 s) did **not** fire on that goal, meaning the robot was
  covering ≥ 0.30 m per 25 s while failing to arrive. It was moving, not stuck. Its R5+R6
  sizing evidence stands; it still has not met a round that explores long enough to judge it.

## What this round does not say

It does not validate the entry gate, the connected-component range fix, or the progress
checker. It does not refute them either. The next round must simply be re-run under the
same configuration.

## Limitations

HIL only; nothing here validates a physical Go2, leg odometry, thermals or isolated module
performance (`CLAUDE.md` rules 5 and 7). One round, and an early death at that.
