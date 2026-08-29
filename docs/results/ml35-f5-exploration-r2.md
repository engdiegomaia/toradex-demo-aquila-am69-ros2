# ML3.5 F5 — round 2: the robot explores the maze and finds the exit marker by itself

Date: 29/08/2026. Commit under test: the provisional-refusal change on top of `a554a25`.
Baseline for this round: round 1, `ml35-f5-exploration-r1.md`. Topology: **real HIL** —
Gazebo Harmonic on the x86 host, Nav2 + SLAM + perception on the Aquila AM69,
`ROS_DOMAIN_ID=69`, `rmw_cyclonedds_cpp`, `ROBOT_TYPE=quadruped`, world
`quadruped_maze11.sdf`.

Single variable against round 1: **planner refusals are provisional.** They live in
their own list and are cleared whenever a goal is reached, because arriving is what
changes the map that caused the refusal. The hard blacklist — Nav2 refusing a goal, or
a goal timing out — stays permanent.

Raw samples: `ml35-f5-exploration-r2.csv` (1319 rows at 2 Hz).
Per-goal records: `ml35-f5-exploration-r2-goals.csv`.

> **Verdict: FAIL on escape, but the demonstration now essentially works.** The robot
> travelled **21.93 m**, mapped **8915 cells**, dispatched **13 goals with 7 reached**,
> and — for the first time in this project — **detected the exit marker autonomously**
> at `sim_s` 402.9 and entered `homing_exit`. Two things stopped it: homing lost the
> marker after 9 s, and three spurious 90 s goal timeouts poisoned the frontier set
> until nothing was left at `sim_s` 570. F5 stays open; round 3 has one variable.

---

## 1. The three runs side by side

| metric | baseline 28/08 | round 1 | **round 2** |
| --- | --- | --- | --- |
| distance travelled | 2.73 m | 2.97 m | **21.93 m** |
| `map_known_cells` | 3564, frozen | 3698 | **8915** |
| goals dispatched / reached | 5 / 4 | 4 / 3 | **13 / 7** |
| `vx` work ratio | — | 5.96% | **41.71%** |
| time in `selecting` | 459 s | 15 s | **~13 s** |
| marker detected autonomously | no | no | **yes, `sim_s` 402.9** |
| `homing_exit` entered | no | no | **yes, `sim_s` 403.0** |
| `escaped` | false | false | false |

The robot got from `x ≈ 0` — where rounds 0 and 1 both died — to `x ≈ -4.66`, which is
the far end of the maze, one cell from the opening at `OPENING_X = -4.90`.

### Preconditions, all met

Sim restarted (`/clock` from `sec: 0`), module stack restarted, fresh `/map` 88 × 85,
robot at `(-0.004, 0.075, 0.347)`, `escaped=false`, explorer `idle`, Nav2 `is_active` →
`success=True`. Exploration was started once, at `sim_s` 52, response
`success=True, message='busca iniciada'`. No manual goal was sent. RTF 0.971.

## 2. The marker was found without help

```
first_detection_sim_s : 402.863
first_exit_pose_sim_s : 402.864
exit_pose_frame       : front_camera
homing_started_sim_s  : 403.013
```

This is the piece that was missing from every previous run, and it closes the loop the
perception gate could only demonstrate with the robot placed by hand
(`ml35-f5-perception-aquila.md` §6). Exploration reached a pose with line of sight, the
detector confirmed, and the explorer switched to `homing_exit` on its own.

### And then lost it, 9 s later

| `sim_s` | state | pose | `marker_visible` |
| --- | --- | --- | --- |
| 403.0 | `homing_exit` | — | true |
| 411.5 | `homing_exit` | (-3.17, 1.01), yaw -91.2° | **false** |
| 412.0 | `selecting` | (-3.16, 1.03) | false — `marcador perdido; retomando fronteiras` |

The detection happened from `(-3.15, 1.05)`, with the marker at `(-4.90, -2.60)` — a
**3.9 m line of sight through the maze opening**, not a close approach. Both homing
goals then failed (0.99 s and 8.0 s), `marker_stale_s = 2.0` elapsed, and the explorer
correctly fell back to frontiers. The fallback behaved exactly as designed; the problem
is that homing was committed to from a glimpse that the next wall occluded.

## 3. What actually ended the run

At `sim_s` 570.5 the extraction reported `frontier_clusters=4` with `frontier_count=0`
— over-suppression again, but this time from the **hard** blacklist, which had 3 entries
and every one of them came from a goal timeout:

| # | goal | elapsed | outcome |
| --- | --- | --- | --- |
| 4 | (-3.295, 0.428) | **90.0 s** | `meta de fronteira expirou` |
| 5 | (-3.741, 1.828) | **90.0 s** | `meta de fronteira expirou` |
| 11 | (-1.659, 2.226) | **90.0 s** | `meta de fronteira expirou` |

All three stopped at **exactly 90.0 s** — the ceiling, not a stall. The successful goals
took 9.1, 19.1, 9.2, 34.1, 24.1, 35.0 and 27.2 s, so the slowest success was 65 s
counting goal 7's partial. With mean `|vx|` of 0.0252 m/s and a 41.7% work ratio, a goal
3 m away does not fit in 90 s. Each spurious expiry wrote a permanent blacklist point,
and three of them swallowed the four clusters that were still there.

Nav2 already runs its own `progress_checker` for a genuine stall. The explorer's
`goal_timeout_s` was duplicating that job and getting it wrong.

## 4. Criteria from §2.3, scored

| criterion | measured | verdict |
| --- | --- | --- |
| no infinite repetition of the same selection | 10 selection cycles, 33 path requests | **pass** |
| map and coverage growing | 1710 → 8915 cells | **pass** |
| goals actually dispatched | 13, seven reached | **pass** |
| no fall | max tilt 0.81°, `z` min 0.336 m | **pass** |
| no manual intervention | none after the single start call | **pass** |
| `frontier_extract_ms` p95 < 100 ms | **201.1 ms** (p50 139.8) | **FAIL — new** |
| escape within 600 s | `escaped=false` | **FAIL** |

`frontier_extract_ms` regressed because the map is now 2.4× bigger — the cost is a
consequence of the run finally working, not of a new defect. It is a real criterion
breach and is recorded as one; it is **not** round 3's variable, because at 10 selection
cycles the total extraction cost across the whole run is under 2 s.

## 5. What this does not validate

- Gazebo on the x86 host: nothing here says anything about a physical Go2
  (`CLAUDE.md` rule 7).
- No crossing. `/demo/maze/escaped` stayed `false`; the 0.05 m/s crossing criterion was
  never reached, so it remains untested.
- Homing was entered but never completed. `completed` and
  `marcador alcancado; aguardando confirmacao de cruzamento` are still unexercised.

## 6. Round 3, single variable

**`goal_timeout_s` 90 s → 180 s.** It is what ended the run, the evidence is
unambiguous (every failure at exactly the ceiling, slowest success at 65 s), and 180 s
still fits three times inside `total_timeout_s` of 600 s. Two tests lock the two
relationships: three goals must fit in the total budget, and the ceiling must clear the
slowest measured success with margin.

Ranked behind it, not applied: the homing commit threshold (a 3.9 m glimpse through the
opening should probably not trigger `homing_exit`, or `marker_stale_s = 2.0` should
tolerate brief occlusion), and the frontier extraction cost on the larger map.
