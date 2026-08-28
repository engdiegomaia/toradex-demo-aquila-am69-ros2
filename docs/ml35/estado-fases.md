# ML3.5 — phase status

Continuity document. Whoever takes this project in a new session reads **this file
first**, then `guia-ml35-docker.md` (a spec).

Update the table and phase section when closing each gate.

---

## Objective of ML3.5

Replace the diff-drive with a A1 quadruped with real leg locomotion** (ROS 2 Jazzy +
Gazebo Harmonic), with each part of the system in its own container and the host x86 /
Aquila module ZZX0003QXZZ explicit since the first phase.

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
| **F5** | Nav2 on legs + ZZXQ0000 modeQXZZ | ** In progress** 28/08/2026 (late) | **APROVADOS:**TF (99,94%), global costmap window, map update, gait and**short stability port** (3/3 targets in the three races, worst ZZXQ005QXZZ s goal of ZZXQ006QXZZ). ** NO EXECUTADOS:** validation of perception in Aquila, exploitation smoke, crossing performance gate and the three cold matches. See "Session ZZXQ008QXZZ/08 (late)", `docs/results/ml35-f5-ab-joint-states.md` and `ml35-f5-portao-tres-metas.md` |
| **F6** | Selectable Fallback and Tests | **Completed** 24/08/2026 | cold start + goal `SUCCEEDED` on both robots |

### 28/08 (late) — the TF gate is closed; speed is not

Evidence: **`docs/results/ml35-f5-ab-joint-states.md`** and
**`ml35-f5-portao-tres-metas.md`**, with the CSVs next door. Committees `a7dc097`
(decimation), `235ac1f` (probe), `d7efd30` (reversion of the map), ZZXQ005QXZZ and
ZZXQ006QXZZ (evidence).

* *The root cause was `/tf` a 1090 Hz.** The `controller_manager` runs the 1000 Hz
because physics runs at 1000 Hz, and a controller without ZZXQ005QXZZ itself inherits
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
also corrects the morning assignment (ZZXQ006QXZZ / 95,30% / ZZXQ008QXZZ), which
compared three separate executions of different durations: the qualitative conclusion
was right, the numbers were not comparable with each other.

* *One of my hypotheses was REPROVADA and is registered as such.** I had said that the
~68% of a `maze_explorer` ZZX0003QXZZ were his `TransformListener` deserializing
ZZXQ005QXZZ messages per second. With the lower ZZXQ006QXZZ flow it SUBIU, for
ZZXQ008QXZZ. The plausible explanation is strangulation — with the load dropping from
26,9 to 18,5, a previously disputed knot turns at ease — but this is hypothesis, not
measurement. Profile his threads (Method of `ml35-f5-clock-fanout.md`) is the next step
if the goal is CPU.

**`map_update_interval` returned to 1.0**, in an independent round. He had gone to 5.0
in this same session by economy of CPU; the economy was measured and did not exist
(31,4% → ZZXQ005QXZZ in ZZXQ006QXZZ, within noise). In return the cost is 1 pp,
symmetrical — small variation and operationally irrelevant, in both directions.
ZZXQ008QXZZ climbs from 0,2 to 1,000 Hz and `static_layer` stops staying until 5 s
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

**The CONTRATO DO ZZX0002QXZZ ZZX0003QXZZ, and that's what closes this session.** The
limit of 0,05 m/s came from a TRAVESSIA test and was being charged with targets
separated by ZZXQ006QXZZ m, where the average route includes acceptance, acceleration,
deceleration by the goal checker, reacquisition and the return route of the recycled
sequence. This measures stability and latency of goals, not crossing. The correction is
not to lower the limit until it passes — it is to separate:

- ** Short port of ESTABILIDADE**, charged from `maze11-short`: three goals
  `SUCCEEDED`, each within 45 s, zero Nav2 errors, zero extrapolations, `worldToMap`
  and `invalid source`, zero falls, zero major route changes. **APROVADO** — Worst
  target 27,9 s, and the durations per target (send) are ZZXQ006QXZZ/4,8/ZZXQ008QXZZ,
  16,5/15,8/5,8 and 27,9/21,3/5,3 s;
- **port of DESEMPENHO crossing**, with targets separated by at least the
  horizon of MPPI, or preferably the autonomous exit itself in 600 s. The limit of
  0,05 m/s still goes there, untouched. ** NO EXECUTADO. **

`maze11-short` (0,0383 / 0,0342 / 0,0447 m/s) are registered and ** are not a
criterion**. They are also not regression: the baseline of Maze11 in `gait_go2.yaml` is
ZZXQ005QXZZ and these give average 0,0391. What has improved is the working ratio in vx,
from 6,2% to ZZXQ008QXZZ–ZZX0009QXZZ — the robot spends two to three times more time
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
desired frequency ZZXQ005QXZZ correlated stops.

**The `maze_explorer` to 76% idle does not justify profiling now.** 76% of a core is
cost, not functional failure, and idle value is not the right measurement of Step 4 —
the extraction of borders only runs in the state `selecting`. The right measurement is
the smoke. Profile only if it shows extraction above 100 ms, controller losing cycles
continuously, exploration without selecting new borders, ZZXQ00006QXZZ regressing, load
preventing perception, or output time incompatible with ZZX0007QXZZ s.

* *Next real lock: perception in Aquila.** Confirm with limited capture:
`/demo/camera/camera_info` reaching the module, image effectively processed, detection
in at least ZZX0001QXZZ from ZZX0002QXZZ frames, exit pose in the correct frame,
detector CPU, TF remaining ≥ZZXQ005QXZZ and controller without material degradation. If
the detector CPU interferes, increase **only** `sample_stride` and repeat — do not touch
ZZXQ008QXZZ together.

* *Agreed sequence until closing: ** Contract/documentation → perception → exploitation
smoke → diagnosis only if smoking fails → three cold matches → final report and
cleaning. Do not increase `vx_max` before this: the carcass clearance of ~6,5 cm remains
small.

* *New armadilla, which cost a whole race of 180 s.** Shortly after recreating the `sim`
container, `/clock` appears in the graph but does not deliver a message to a subscriber
NOVO for a few minutes. Any script with `use_sim_time: True` that climbs into this
window reads clock stopped: ZZXQ005QXZZ ZZXQ006QXZZ, cloud age −240 s, ZZXQ008QXZZ
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
cockpit **ZZXQ005QXZZ**. None of them measure navigation.

* * The gate remains the lock, and it failed. ** Last race
(`artifacts/maze11-short-gate.csv`): 37,1 s, 0,69 m, **0,0185 m/s**, `cmd_vx` non-null
in 18,7% of the samples — **below the floor of ZZXQ006QXZZ m/s**, and without
3/ZZXQ008QXZZ targets. The previous race, before `restamp_tf: true`, had the
robot**frozen** (`cmd_vx` zero in 150 s). The parameter unlocked the command and **did
not close the gate**.

`restamp_tf` was verified as a real parameter of Jazzy `slam_toolbox`
(`slam_toolbox_common.hpp:177`, and `restamp_tf: false` in the five
`mapper_params_*.yaml` of ZZXQ005QXZZ) — it is not ZZXQ006QXZZ ignored silently.
`transform_timeout` is in ZZXQ008QXZZ as required.

**`nav_trial.py` started archiving the evidence by goal.** Before the outcome of each
action died in the stdout and the goal in flight at the end of the trial was never
recorded — a gate of 3 goals reported 2. Now comes out a CSV brother `<csv>-metas.csv`
with target, outcome, ZZXQ005QXZZ, ZZXQ006QXZZ/`error_msg` of Nav2 and route
changes**from that** goal, and each telemetry sample carries ZZXQ008QXZZ.

* * Open risk that precedes any conclusion about perception: ** Camera RAW no longer
crosses the wire since `ml35-f5-camera-comprimida.md`. `SetRemap` by
`demo_bringup/launch/perception.launch.py` automatically reconnects the detector image,
but `/demo/camera/camera_info` is not remapped**. Without it the detector publishes
detection and never publishes pose — silent failure. Check ZZXQ005QXZZ ** in the
module** before blaming the vision.

### 27/08 (Part 3) — mechanism found: the global plan alternates 1 Hz. `clearing: false` ZZX005QXZZ

Evidence: **`docs/results/ml35-f5-memoria-costmap.md`**.

* *Causes root, reading `/plan` every 5 s in a stuck target (0,0) → (0,8):**

| t | length | initial course |
| ---: | ---: | ---: |
| +5 s / +10 s | **11,49 m** | 173° — true route |
| +15 s / +20 s | **8,59 m** | 89° — cross wall not seen |
| +25 s / +30 s | 8,66 / 8,81 m | 35° / 18° |

The 11,5 m match the offline geodesic (12,23 m). The 8,6 m only exist because
`allow_unknown: true` makes the unknown cheap. **The MPPI receives a path that reverses
ZZXQ005QXZZ–180° every second** — then rotates without moving. Completes the finding of
part 2: there it was proven that the symptom disappears with good goal; here is the
mechanism by which the bad goal produces it.

* *A failed experiment — do not repeat.** `clearing: false` no ZZX0001QXZZ global
("memory map") improved margin (movement 0,19 → 1,04 m, working ratio 0,0% →
ZZXQ005QXZZ) and ** did not move the mechanism**: the plan continued alternating
ZZXQ006QXZZ ↔ 8,77 m, zero goals. Reading `costmap_raw`, with it connected **150 of 161
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
`ros2_ws/src/demo_navigation/config` over ZZX0002QXZZ (the final target of the
symlink**, not the path installed — mounting on the installed would be quiet). **
Parameter in the module came to cost `sync`, not `build`.**

* *Open Anomalia:** the global costmap brand first cell ≥ 253 to **0,55 m in +y**, where
`maze_fit.py` measures **3,47 m freeway**. Measure before running SLAM — a persistent
map would inherit the error definitively.

### 27/08 (Part 2) — PROVADO No HIL: 8 by ZZXQ006QXZZ met goals, working ratio 0,0% → ZZXQ008QXZZ

Evidence: **`docs/results/ml35-f5-rota-conectada.md`** + the two CSVs next door. A/B
with minutes apart, real **HIL** (Nav2 in Aquila ZZX0002QXZZ), same images, same
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
ZZXQ005QXZZ without touching anything but the goal. Efficiency ZZXQ006QXZZ reproduces
57% measured on the host in ZZXQ008QXZZ/08 — the module has always been able to do so.

* * The unidirectional spin of §10 is not a controller defect. With valid plan the
`cmd_wz` alternates 45,2% / 51,3% and the drift is +1,1° in four minutes. With meta
behind wall back to be unidirectional — **and with the inverted sign** in relation to
§ZZXQ005QXZZ, which kills the family "asymmetry of criticism" / "sign error in turn":
signal error does not change signal.

* *Consequence to the gate. ** "Goal Nav2 `SUCCEEDED` with the leg robot, Nav2 in the
module" was completed eight times in a race**. What I failed was the 8 m protocol on
patrol targets. The F5 gate has to be rewritten on connected route or on persisted map
before being charged again.

* ==References====External links== 37,5% and 0,0454 m/s are still below `vx_max` 0,15
m/s. Critical tune only makes sense from here.

* *New armadilla:**Every target of the Maze11 has negative `x`, and `--goals -1.50,...`
is read by the argparse as flag — the script prints `usage` and exits **0**. With
`2>/dev/null` turns silent race that does nothing. Always use ZZXQ005QXZZ. Documented in
`nav_trial.py` itself.

### 27/08 (Part 1) — the test targets are behind the wall; the global plan crosses the wall

Evidence and numbers: `docs/ml35/proximos-passos-navegacao.md` §11. Measure **offline**,
without bench, without ROS and without Gazebo — read only the STL of the Maze11, with
the new tool `scripts/maze_geodesic.py` (ZZXQ005QXZZ guards in ZZXQ006QXZZ, three
verified by mutation).

* *The four `MAZE11_GOALS` have a straight wall. The geodesic for navigable space is
1,53× the 4,22× the straight, and from spam the robot sees, with occlusion, **19,6%**
the free space within the 8 m ZZX005QXZZ.

With `global_costmap` escalator **without `static_layer` and without map** and the NavFn
in `allow_unknown: true`, the plan of these goals crosses unobserved wall — `SUCCEEDED`,
beautiful path in the RViz and in the cockpit, **zero error or log**. Corroborates with
data already in the repository: the path measured in §8 had ~ZZXQ005QXZZ m for a goal
whose real route is ZZXQ006QXZZ m and whose own straight is 8,00 m.

* ==References== ** §§7–10 measured MPPI with an invalid entry. This does not reopen the
five refuted hypotheses of §1, but no conclusion about critics survives — the critical
off test goes down from priority.

* * The ordination that ¢Ü9 searched for bearing and did not find** is by distance to
the first wall on the line: 0,96 m → 0,00 m displacement; ZZX0003QXZZ m → 0,07 m;
ZZXQ005QXZZ m → ZZXQ006QXZZ m. The cut falls on the horizon of MPPI (ZZXQ008QXZZ m).

* *Next step, cheap and decisive, in the host, without touching the image: ** Rotate the
connected route of `maze_route.py` (0 of ZZX0002QXZZ legs with wall on the straight,
100% visible in all, against 4 of ZZXQ005QXZZ blocked on patrol). Command ready at §11.

* *Persisting the map — operator's request — is not a new front: it is connecting what
is already in the tree.** Go2 `static_layer` is already set and inert with the procedure
next door, and the diff-drive path already navigates over `maps/warehouse.{pgm,yaml}`.
The two real locks: `slam_params.yaml` has `base_frame: base_link` (parameter,
non-architecture) and `slam_toolbox` consumes ZZXQ005QXZZ, while Go2's ZZXQ006QXZZ is
the degenerate ring that 3 delta of ZZXQ008QXZZ has already measured as **zero
obstacles**. The good data is `/demo/scan_cloud`, and flattening it is
`ros-jazzy-pointcloud-to-laserscan` (touch, 2.0.2 in Jazzy's apt, not yet in any image).
**No AMCL** in this topology: Gazebo's odometry is true of terrain and `map`→`odom` is
already the identity of `odom_tf`.

### 26/08 (late) — cockpit reset, target telemetry, campaign protocol

Full evidence: `docs/results/cockpit-reset-nao-destrutivo.md`.

* * Closed serious defect: the cockpit reset button erased the robot. `/demo/sim/reset`
used `ControlWorld.reset.all`, which returns the world to SDF of origin — and the robot
and the two scene cameras are INSERIDOS after loading (`ros_gz_sim create`), so they are
not on it. Measure: `/joint_states` 999 Hz → dead, `/demo/imu` ZZXQ008QXZZ Hz → dead,
`/demo/odom` 49,6 Hz → dead, `gz model -m demo_robot` → `No model named <demo_robot>`.

The failure mode was the worst of this project: clock followed 999 Hz and orphaned
sensors at 10 Hz, then **The cockpit became whole green pointing to a nonexistent
plant** without a log line. Recover required rebooting of `sim`.

Now the reset TELEPORTA the robot for the birth pose of the scenario, by the same
`/demo/sim/set_entity_pose` that the cameras already used. Verified: robot lap from
(2,0; −1,5) to (0,00003; −ZZXQ005QXZZ), ZZXQ00006QXZZ 1000 Hz, ZZXQ008QXZZ 974 Hz,
ZZXQ0010QZZ 49,9 Hz, and the five models follow in the world. The watch **does not**
return to zero, on purpose: a jump backwards would invalidate Nav2's TF buffer and
`controller_manager`.

> "Resent alone" was written here and was false — corrected in session
> Next, see below.

* *Telemetry of the target in the cockpit, measured in the actual AM69. `target_monitor`
publishes `/demo/target/status` (CPU, memory, temperature, load) and
`/demo/target/ops_log` (axis commanded in ZZXQ005QXZZ, manche and odom, in short text).
The log panel no longer depends on crude `/rosout` — ZZX0007QXZZ is signed only as
filtered reservation for warnings and errors. Temperature checked against sensor:
ZZXQ008QXZZ °C reported against `thermal_zone1/6` reading 34498 thousandths at the same
time; the seven zones between 32,1 and 34,5 °C. Cost of the node: **4,3% of a core** in
800% available (`use_sim_time: False` keeps it cheap — it does not sign `/clock`).

**The screens survive the two restarts.** Probe with the rosbridge client of the cockpit
itself, a panel subscriber: `sim` restore and restore of the application on the target
do not lose any panel, and WebSocket does not fall (in this topology `cockpit` and
ZZX0002QXZZ run on the host). The only zero is `/demo/cmd_vel_si` without active target,
which **is not a defect** — Nav2 went up `Managed nodes are active` and ZZXQ005QXZZ only
publishes after the first target; the new channel says so in text.

One hypothesis was tested and discarded**: reconnect the `<img>` of MJPEG after the
publisher returns. Measured with `curl` in the same response HTTP through a restort of
`sim`, bytes grow unbroken (ZZXQ005QXZZ ZZX006QXZZ → 4,61 ZZXQ008QXZZ).
`web_video_server` keeps registration and response open. Don't spend code on it.

* * Campaign protocol delivered: `scripts/nav_campaign.py`** It is the missing link
between `nav_trial.py` (a race) and `summarize_trials.py` (resume replicatas): decides
ORDEM and what happens between legs. Intercala `A B A B A B` instead of blocking, and
replace robot and costmap before each leg. **It is only possible because of the repair
of the reset above** — a campaign that called the old reset between legs would measure,
from the leg ZZXQ005QXZZ onward, a world without robot and with nothing accusing. Ten
guards in ZZXQ006QXZZ, including the reverse (the blocked order has to fail) and the
fact that `ros2 service call` leaves ZZXQ008QXZZ even with `success=False`.

45 s smoke leg to prove the loop: work by `cmd_vx` **0,0%**, `vx` It reproduces the
symptom; n=1 and ZZXQ008QXZZ s do not decide anything.

**Two things this session didn't do:**

- * *The cockpit was not opened in a browser.** There is no Chrome on this machine and the
  MCP automation does not drive Firefox installed. Everything above was measured in
  the data path. The visual gate is still pending from manual pass.
- **Manual control (F4) is not implemented**, and there is a new lock and
  concrete: `twist_mux` ** is not in any image** and the module ** has no default
  route** (only the `<LAN_CIDR>`), so `apt` does not solve anything there. LAN
  ZZX0004QXZZ gateway responds to ZZXQ005QXZZ ms and the module already has
  ZZXQ006QXZZ corporate — only `sudo ip route add default via <LAN_GATEWAY> dev
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
  Nav2 as a cause — the robot still dragged 0,87 m and turned 135° in ZZXQ005QXZZ s
  without any published commands;
- **Teleport non-stop**: `SetEntityPose` preserves speed. With a flow
  from `/demo/cmd_vel` alive to 10 Hz during reset (the real case, with nav2 driving),
  the robot COLAPSA — `z` by 0,337 m for 0,162 m in 1 s — and is writhing 40 s.

Fixed with two new services in the `twist_to_inputs` (only ZZX0001QXZZ writer):
`/demo/gait/hold` (trotting → fixed stand, immobile robot, `GAIT_STOP_S = 2,0 s`
waiting) called teleporte ANTES, ZZXQ005QXZZ (fixed stand → trotting, ZZXQ006QXZZ
reancora ZZX0007QXZZ/ZZXQ008QXZZ in new pose) called ZZXQ009QXZZ. Better effort: in a
differential plant the two services do not exist and this is normal way — but the
absence enters the message itself of the `Trigger` reset, never stays silent.

Verified repeating the case that failed (living command to 10 Hz during reset: robot
never leaves 0,35-ZZX0002QXZZ m in height, again obeys the same command after reset, a
second reset re-arm. **Checked also with the robot CAIDO** (tombed 180°, stuck in
`mode=RECOVER` with ZZXQ005QXZZ — this mode does not come out upside down alone): the
reset recovers it standing, ZZXQ006QXZZ, `tilt=0,2°`, ZZXQ008QXZZ, and it goes back to
walking normally.

New guards in `test_sim_reset.py` and `test_twist_to_inputs.py`: the order to
stop→teleport→retake has to be in this sequence at the source, the drop command (`2`)
leaves exactly once, the hold has no time limit, and every way out of the reset handler
— including those of error — has to resume the gait.

### 26/08 (early) — where to resume

Full evidence: `docs/results/ml35-f5-clock-fanout.md`.

* *The item 1 of the previous session (CPU of the module) is FECHADO.** Do not reopen it
by the old path: it is not the MPPI loop (refuted in `ml35-f5-mppi-amostragem.md`) and
it is not the ZZXQ005QXZZ rate (refuted in ZZXQ006QXZZ/08 — strangulation killed
navigation).

* *It was `/clock` subscription fan-out.**Profiling `/proc/<tid>/stat` by thread, the
`nav` container spent **367% by 800% with the robot ZZXQ005QXZZ**, and 111% of this were
three republishers in Python — `odom_tf`, ZZXQ008QXZZ and `nav_control_relay` — who did
not call the clock once** and signed ZZXQ0010QZZ to ~870 Hz just because `use_sim_time:
true` makes rclpy create the signature.

Fixed with `use_sim_time: False` on all three. The three fell from **111% to 17,7%**,
and `ros2 topic info /clock -v` confirms that none of them sign anymore. Four
`demo_bringup/test/test_sim_time_scope.py` tests, verified by mutation, lock the
invariant in both directions — including the reverse, which is what matters: **No
ZZXQ005QXZZ cannot call `get_clock()`**.

* * What it bought, measured: *

|  | before | later |
| --- | ---: | ---: |
| refusals `Ignoring the source` | 16 | **0** |
| `Robot to stop due to invalid source` | 4 | **0** |
| costmap discards | — | **0** |
| machine clearance under navigation | none | **307% of 800%** |

* * What it didn't buy: movement. * The confirmation race gave 0,0246 m/s, `vx` at zero
at **90,7%** of the samples, rotating in **90,3%**, and **0 at ZZXQ005QXZZ targets of 8
m**. Inside the known noise range. Zero falls.

* *So the item 2 of the previous session — decision of the route — is now alone and
without confusion.** With CPU remaining and without a single sensor refusal, the robot
continues to rotate rather than relocate. This was no side effect of CPU.

**Two traps discovered in this session**

- `docker/.env` still loads `MODULE_IP=<MODULE_IP>`, old address of
  bench, and he beats the defaults**. `ssh` works like this because it uses the mDNS
  name, so the error only appears in `sync`/`build` (`não identifiquei a interface do
  módulo que carrega <MODULE_IP>`). Pass explicit `MODULE_IP=` and ZZXQ005QXZZ, or fix
  ZZXQ006QXZZ.
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
historical numbers (RAW ZZX0003QXZZ working; 0,6% tablet), so the new definition does
not change the baseline.

The native rebuild of the arm64 image was completed in Aquila and the containers were
recreated. `module.sh verify` returned to **3/3**, and the `route_server` log of the new
image contains only `AdjustSpeedLimit` (does not contain the old ZZXQ005QXZZ). Test n
3. * *Only then MPPI** (`PathAlignCritic` ZZX0002QXZZ × `PathAngleCritic` 2,0), now in
   clean test, with costmap measured at each condition.

### 25/08 (night) — where to resume (read this before touching anything)

Full evidence: `docs/results/ml35-f5-ethernet0-repeticao.md`. Previous orientation of
this section (fix PHY, switch cable, measure later) ** It has been accomplished and is
unsuccessful** — do not repeat it.

* *The link is resolved and proven. `enp0s31f6` to 1000 Mb/s full, host `<HOST_IP>` ↔
Aquila `<MODULE_IP>` by `ethernet0`, ZZXQ005QXZZ ZZXQ006QXZZ ms, symmetric route in both
directions, `scripts/module.sh verify` returning **ZZXQ008QXZZ** with the three steps.
The asymmetry disappeared structurally: Wi-Fi remained in metrics 600 versus cable 100,
so the entire `/24` prefers cable.

* * The 8 m gate continues REPROVADO, and the network is not the cause.** The
ZZX0002QXZZ/`ethernet0` hypothesis that was open here is **refuted by measurement**:
with correct route and clean link, the two goals of 8 m burst the same deadline.

* * The two causes measured in order of size:**

1. **CPU of the module.** Nav2 alone consumes **600–ZZX0002QXZZ by ZZX0003QXZZ** no AM69. A
   perception adds ~187% and passes capacity. Then the coolness of the sensor
   collapses: the `collision_monitor` refused the cloud of LiDAR ZZX0002QXZZ times
   with 1,0–1,2 s lag. With the idle Nav2 this lag is 42 ms — that is, it is line by
   containment, **no** transport. Measured cost: **2,8×** at average speed (0,0429 →
   ZZXQ008QXZZ m/s).
2. **Decision of route.** In the complete HIL the robot has `vx` at zero at **79% ** of
   samples and rotates in **93,9% ** of them: he passes the essay by spinning instead
   of transferring**. Without the camera the standard relieves but does not
   disappear, and the cost migrates to the route — 18,00 m of way to ZZX0002QXZZ m
   liquids, **28,9% of efficiency** against 57% in the host. It matches the already
   registered hypothesis of `PathAlignCritic` ZZXQ00006QXZZ against ZZX0007QXZZ
   ZZXQ008QXZZ, which is followed without correction test**.

* *Getting the wire camera doesn't make the goal pass. It was measured: 0,0429 m/s and
yet 0 by 2 targets. It's two independent limits, and only one is CPU.

* *Stability: ** Zero falls in both races, but the peak tilt goes from 0,94° to
**15,66°** precisely in the race where the robot runs. The low value of the complete HIL
describes a robot almost stopped, not a stable robot. Casting off follows in **+6,5
cm**.

* * 2 and 3 of protocol n=3 were not executed** — 1 failed and the mechanism was
identified; repeat would spend bench without new information.

* *A closed method trap in this session:** `verify` disapproved by `/clock` absent from
a module that reads `/clock` to 616 Hz. The 2 stage collected with ZZXQ005QXZZ and then
required ZZXQ006QXZZ, which is not under `/demo/`. The test that existed passed all the
time because it only checked if the string appeared in the file. Corrected, with
mutation failure test.

**FASE 2 ZZX0002QXZZ ZZX0003QXZZ JÁ FOI ZZXQ005QXZZ (ZZXQ006QXZZ/08, night).** The
compressed camera is implemented, validated and measured: ZZXQ008QXZZ. It delivers to
engineering (~82× less yarn, CPU module ~711% → ~600%) and **does not move the gate** –
speed has not improved reliably and the working reason has worsened (2,1–2,7% vs 5,8%).
The `collision_monitor` continues to refuse the cloud with ~1,0 s of lag and emitting
`Robot to stop due to invalid source`. The dominant variable is the **presence** of
perception, not the format of transport: with perception in the module the ratio is 2–6%
in any format; without it, 16,8%. Do not repeat phase 2 and do not discuss image format
again.

**FASE 1 DO ZZX0003QXZZ ALSO FOI ZZXQ005QXZZ (ZZXQ006QXZZ/08, night) AND ZZXQ008QXZZ.**
`time_steps` 96→64 with `model_dt` 0,10→0,15 (constant horizon, 33% less sampling) **
did not reduce CPU**: ~437% → ~439%. The cost of MPPI here is not dominated by
`batch_size × time_steps`. Evidence: `docs/results/ml35-f5-mppi-amostragem.md`. Not
adopted; YAML back to baseline with A/B out of default path.

**LEIA ISTO ANTES ZZX0003QXZZ RODAR ZZXQ005QZZ ENSAIO NOVO — the current method does not
decide.** Two races in the **identical** configuration gave **ZZXQ008QXZZ and 0,0265
m/s**, dispersion of **2,4×**. The noise between races is greater than the desired
effects, so **A/B of n=1 on this bench is ininterpretable**. Before tuning anything: n ≥
3 per condition, interspersed, median and reported range.

* * Falls are no longer variance:** 2 in 3 races after camera compressed, against 0 in 2
before. No mechanism identified and no cause demonstrated, but it is research item, not
footnote — stability is prerequisite of any goal.

* * Order suggested by the data for the next session: ** Reduce Nav2 CPU in the module →
take the RAW image from the wire (compressed transport to perception) → only then touch
the MPPI → repeat 420 s / 200 s with n=ZZXQ005QXZZ.
---

* *Take decision: target changed from A1 to Go2** (see "F2 — verification executed";
license justification given on F2 was incomplete and corrected on F3 — see "F3 — license
tracking"). **F3 rotated and the gate crashed**: Go2 standing, stable, walking by
`/demo/cmd_vel` with the packages and the project launch, no longer with the spike. HOLD
failure of ZZXQ008QXZZ was corrected in 20/08 and the full perception contract was
revalidated in 24/ZZX0012QXZZ. Evidence of gait in `docs/results/ml35-postura-parada.md`
and closing below.

Current drive plan: **`docs/ml35/plano-movimentacao.md`** (19/ZZX0002QXZZ/2026). The
previous phase plan 1–ZZX0005QXZZ has been removed because it is fully overcome; its
results remain in `docs/results/ml35-f4-parcial.md`.

Open parallel work — ** Unified cockpit**: plan approved at 24/08/2026 and **F1
completed on the same day**. Axle changed from "capture X11" windows (four failed
attempts) to "render from ROS 2 topics" on a web cockpit that then turns HMI from
ZZXQ008QXZZ. Decisions, evidence and phases in **`docs/ml35/plano-cockpit-web.md`**;
evidence from F1 (screen captures, fees, reconnection) in
**`docs/results/cockpit-web-f1.md`**. The previous checkpoint
(`docs/results/cockpit-standalone-parcial.md`) is marked as overwritten; do not resume
his recommendation.

Cockpit status by phase: **F1 and F3b closed** (24/08/2026). F1 has risen the services
ZZXQ005QXZZ and ZZXQ006QXZZ in `compose.host.yml`, the bundle in ZZXQ008QXZZ, the live
camera and automatic reconnection. **F3b** closed the blue panel (two static, alternable
scene cameras) and the green one (costmap, plane, laser, footprint, and click sending
goal), with the gate completed: a target clicked on canvas was accepted and executed by
Nav2. Evidence on **`docs/results/cockpit-web-f3b.md`**; how to run and what each panel
does in **`docs/guia-completo.md`** (Part II).

In 25/08/ZZX0002QXZZ, three **adjustments of UI**orders on the bench, outside of the
phase numbering and without opening new phase: double Toradex brand, scene cameras
following the robot in both views, and "restart nav" from the cockpit. Evidence in
**`docs/results/cockpit-web-ui-ajustes.md`**. A finding with its own weight came out:
`RESET`+`STARTUP` on Nav2 ZZX0007QXZZ**drops the container** with `SIGSEGV` while
setting up `route_server`, played twice — so reset uses `PAUSE`/`RESUME`. Candidate for
upstream issue; see 8 trap of Part II of `guia-completo.md`. **Next to the cockpit is
F4** (manual control behind the `twist_mux`).

On the same date they entered, at the operator's request: control of simulation by
cockpit (play/pause/reset), camera control (turn, tilt, move, zoom, refocus), visual
identity Toradex (white background, `#00508c`, `#96c837`, ZZX0002QXZZ, with the brands
Toradex and ROS in the bar) and the increase in the quality of the scene cameras. Three
points worth loading for the next session:

1. **"Start simulation on target" is not possible** and was not done. The Gazebo
   is OGRE 2; AM69 only has OpenGL ES 3.2/Vulkan ZZXQ005QXZZ (rule ZZXQ006QXZZ). What
   exists is play/pause/reset ==References====External links==
2. * * The browser does not speak Gazebo types.** Call `ControlWorld` direct by
   rosbridge fails with `InvalidModuleException` — the cockpit container does not
   have `ros_gz_interfaces`, and the M3 runs in the module. The border is `std_srvs`,
   and the translation lives on the `sim_control_relay` node, next to the simulator.
3. * *Camera resolution costs RTF.** With the two scene cameras at 1600x1200,
   `update_rate 15` delivery 9,43 Hz with real-time factor **0,59**, and `update_rate
   10` delivers 9,77 Hz with **0,97** — ask ZZXQ00006QXZZ does not yield an extra
   frame and costs 40% the speed of the simulation. Adopted ZZXQ008QXZZ. See section
   5 of `cockpit-web-f3b.md`.

None of this was executed in arm64 or Aquila. **F2 and Cockpit F4 remain open** (kiosk
in module and manual control with `twist_mux`).

### 21/08/2026 — navigating quality in the maze11 (within F5)

Scenario S6 passed to **`maze11`**, starting in the lower right corner
(`docs/results/ml35-labirinto.md`). Next, Nav2's decision-making quality was measured
and corrected: **0,0399 → 0,0650 m/s (+63%)**, re **11–62% → ZZXQ008QXZZ**, route
efficiency **13% → 57%**, and the first meta accomplished** (8 m at ZZX0012QXZZ s).
Evidence and limits in **`docs/results/ml35-navegacao-maze11.md`**.

Three faults, all decision and no sensor:

1. `vx_min: -0.10` produced **deadlock**: the robot retreated, leaned against the wall and back
   It was still great. Measured in 100% samples with the robot stopped at 0,00 m. Now
   ZZX0002QXZZ, with `wz_max` ZZX0004QXZZ → ZZXQ005QXZZ for the spin to be a real
   alternative.
2. * *No behavior tree of the Nav2 Jazzy calls `SmoothPath`**, then the
   `smoother_server` was active and idle and the MPPI was pursuing the raw ladder of
   the NavFn. Now there's `demo_navigation/behavior_trees/nav_to_pose_smoothed.xml`.
3. NavFn chose route by length. Costmap inflation **global** went to
   0,85 / 2,0 — diverge from the site on purpose in the safe direction.

Handle and odometry were verified on request and **are sound** (odom vs TF error with
0,0000 m; no self-collision to handle). New Tools: ZZX0002QXZZ,
`scripts/costmap_probe.py`, `scripts/selfhit.py`.

* * Not closed: ** Carcase clearance follows in **+6,5 cm** and is the gate of any
future speed increase. It comes from `robot_radius: 0.38` model the trunk as a circle;
the correction is polygonal footprint with `consider_footprint: true`.

### 21/08/2026 — HIL standing in Aquila AM69 (within ZZXQ005QXZZ)

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
Container memory ZZX0002QXZZ **6,89 GiB → 307 MiB**, load **ZZXQ005QXZZ → 9,5**,
activation at **~10 s** ZZXQ008QXZZ total has not changed.

**Strangulate `/clock` was tried, measured and reversed**: the 100 Hz CPU dropped from
470% to 324% and navigation died (ZZXQ005QXZZ vs ZZXQ006QXZZ m/s). The node stays in the
package with A/B in the header, out of default path.

* * Not closed, located: ** The working ratio of `cmd_vx` is low in both machines —
normal peak (0,10–0,14), medium 0,006–0,008. The start of the Maze11 requires spinning
stop of ~ZZXQ005QXZZ and the ZZXQ006QXZZ commands `wz = 0,035` rad/s, ZZXQ008QXZZ from
the ceiling. Control loop, TF, costmap and `collision_monitor` were discarded by
measurement. Unmeasured hypothesis: `PathAlignCritic` in 14,0 against `PathAngleCritic`
in 2,0.

* * Operator decision at 24/08/2026:** preserve 640×480 a ZZXQ005QXZZ Hz and migrate HIL
to Ethernet. F5 only closes after the real race in this link; do not infer the result
from the band measured in Wi-Fi.

### 24/08/2026 — HIL Ethernet executed, long gate still open

The link was actually executed: Host `enp0s31f6` and Aquila `ethernet1`, with
`<HOST_IP>` and `<MODULE_IP>` peer cycloneDDDS. The two Aquila ports in the same subnet
announce the same hostname mDNS; leaving `MODULE_IP` implied switched between the two
addresses. The local configuration now fixes a port before `module.sh sync`.

Two QoS defects only appeared with fragmented samples in HIL. RAW camera from 921600
bytes needed a ZZX0003QXZZ reader; the LiDAR cloud needed a `SENSOR_DATA` producer for
Nav2 ZZXQ005QXZZ readers. After the two fixes, camera, detections and detection cloud
flowed to ~ZZXQ00006QXZZ Hz, and `collision_monitor` stopped rejecting commands by old
font.

A short goal closed `SUCCEEDED` in **28 s**, with Nav2 + perception in AM69,
Gazebo/RViz/camera in the host and zero fall. The final protocol, however, did not close
the gate: **419,9 s, 8,31 m by way, ZZXQ005QXZZ m/s, ZZXQ00006QXZZ 8 m goals completed**
(two ZZXQ008QXZZ s deadlines). The value repeats Wi-Fi with perception (0,0197 m/s),
refuting the hypothesis that changing only the physical medium would remove the
bottleneck. The remaining cost is on the camera processing/copying/fragmentation path
and on the low working ratio of MPPI.

Full evidence and CSVs in **`docs/results/ml35-hil-ethernet.md`**. F5 remains open until
a 8 m finish `SUCCEEDED` in 420/ZZXQ005QXZZ s protocol.

* *Watch out when you read that plan: ** the blocker he records — "the TF tree does not
close, there is no frame `odom`" — ** was solved in 20/08/2026**. The tree now has
ZZXQ005QXZZ edges, ZZXQ00006QXZZ static, root `map`, and the Nav2 plans and deflects
over the quadruped. See section "ZZXQ008QXZZ — Nav2 on legs" below.

Phase A (parametrix gait + versioned test bench) completed at 19/08/2026.

Phase B (Defect 2, drop in `HOLD` prolonged) **Completed in 20/08/2026** with
ZZXQ005QXZZ, which became default in ZZXQ006QXZZ. The correction that seemed obvious —
download only `balance.weight_moment` input from ZZXQ008QXZZ to 100 — was tested and
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

29 files, +2870/−655. Gate slammed: ZZX0003QXZZ clean (6 packages), ZZXQ005QXZZ
ZZXQ006QXZZ tests 0 failures, clean tree.

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

## F1 — completed 14/ZZX0002QXZZ/2026

Beaten gate: **good Nav2 `SUCCEEDED`** (`error_code: 0`) with full demo in containers,
sent from container `tools`. ZZX0003QXZZ clean (6 packages), ZZXQ005QXZZ **ZZXQ006QXZZ 0
faults** (was ZZXQ008QXZZ; +7) Evidence of execution in
`docs/results/ml35-f1-execucao.md`.

Measured rates, learn mode, host x86: `/clock` 334 Hz, `/demo/odom` ZZX0003QXZZ Hz,
`/demo/scan` ZZXQ005QXZZ Hz, ZZXQ006QXZZ 10,0 Hz, ZZXQ008QXZZ ZZXQ0009QZZ Hz. AMCL, bt
navigator, controller server and planner server all `active`. Topics contract preserved,
`demo_perception` untouched (rule 6).

** Created:** `docker/{base,sim,nav,perception,viz,tools}/Dockerfile`,
`docker/hw/README.md`, `docker/compose.{host,module}.yml`,
`docker/cyclonedds/{host,module}.xml`, `docker/entrypoint.sh`, ZZXQ005QXZZ. The old
scaffold directories (ZZXQ006QXZZ, `simulation`, ZZXQ008QXZZ) were all empty and
untraceable by git — there was no rename, it was created.

** `demo_bringup/launch/{sim,nav,perception,viz}.launch.py` (new),
`demo_bringup/{setup.py,package.xml}`, `demo_bringup/demo_bringup/wait_for_clock.py`
(new) and its test. `demo_navigation/{setup.py,package.xml}` and `navigation.launch.py`
— see "winding" below.

### Risk number one was confirmed, and the decision was to change the timers.

`learn.launch.py` (20 s perception, ZZX0002QXZZ s nav) delays measure the time since the
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
processes of the same container. Symptoms: `ros2 node list` empty, ZZXQ008QXZZ only with
`/rosout`, and Gazebo spawner in `Waiting messages on topic [robot_description]` forever
— while `robot_state_publisher` logged `Robot initialized` into the same container.
Correction: `<Peer address="127.0.0.1"/>`, which is **load-bearing**, non redundant. The
IP of the module enters F5 (see `module.xml`, which today only speaks to itself on
purpose).

Do not confuse with forcing `<NetworkInterface name="lo"/>`: this has been tested and
is**wrong** — isolates the client from nodes that have already selected the actual
interface (here `wlp0s20f3`). Stays `autodetermine`; peer localhost only adds discovery
address.

**2. Guide ZZX0001QXZZ does not exist on this machine.** `ip -br link` in the host gives
ZZX0003QXZZ, `enp0s31f6` (ZZXQ005QXZZ), `wlp0s20f3` (UP, Wi-Fi), ZZXQ008QXZZ, `docker0`.
Both XMLs use `autodetermine` instead of a fixed name, with the verification procedure
commented on in the file. As the host is in Wi-Fi, the case "multicast dies" is
expected, not the exceptional.

**3. `GZ_SIM_RESOURCE_PATH` empty in container.** `demo_description` references mesh as
`model://nav2_minimal_tb4_description/meshes/*.dae`. Natively solves by environment ROS
environment; in container not. Symptoms: correct collision and inertiaal spawna robot —
physics and navigation work — and **no visible body**. Invisible robot on ZZXQ005QXZZ
but present for planner. Fixed with ZZXQ006QXZZ no `sim/Dockerfile`; N mesh errors for
ZZXQ008QXZZ.

### Vendorization of four Nav2 files (rule 1)

`ros-jazzy-nav2-bringup` **hard-depends** by `nav2-minimal-tb3-sim`,
`nav2-minimal-tb4-sim`, `ros-gz-sim` and `navigation2`. Measure: put ZZXQ005QXZZ,
ZZXQ006QXZZ, `gz-rendering`, ZZXQ008QXZZ and 30+ packages in the image `nav` — **3,7 GB
and OGRE 2 in an image that goes to AM69**, direct violation of the 1 rule.
`--no-install-recommends` does not help: are `Depends`.

`ros-jazzy-navigation2` (the metapackage) has the same problem one level below, via
`nav2-rviz-plugins` → `rviz-ogre-vendor`.

Solution: The four Launch files that `navigation.launch.py` need (`bringup`,
`localization`, `navigation`, `slam`) are sold in ZZXQ005QXZZ, Apache-ZZXQ006QXZZ,
copyright headers intact, **only the root paths of rerouted package**. Nav2 servers
enter the Dockerfile individually. Provenance and exact editions on
`nav2_vendored/README.md`.

Result: `nav` by 3,7 ZZX0002QXZZ → **2,48 GB**, and **zero** OGRE/RViz/Gazebo packages.
Verified ZZXQ006QXZZ plugins declared in `nav2_params.yaml` — all resolve via pluginlib
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

`nav` a 2,48 ZZX0002QXZZ remains fat for Torizon data partition. It is no violation of
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
  `group_add: ["${RENDER_GID:-992}"]` in `sim` and ZZX0007QXZZ; verified that with gid
  the device is readable and without it the open fails. After correction: 0 libEGL
  errors, 0 mesh errors, goal `SUCCEEDED`.

  **`RENDER_GID` is host-specific** (`getent group render | cut -d: -f3`). The default
  992 no compose applies to this machine. **Open thinking:** `docker/.env.example`
  does not document the variable. The file is blocked by environmental permission rule
  (`.env*` is denied for reading and shell), confirmed in two sessions — it is not
  transient. **Correction is manual, operator:** add to ZZXQ005QXZZ

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

## F2 — verification executed 14/ZZX0002QXZZ/2026

`legubiao/quadruped_ros2_control` in `/tmp/f2-spike`, HEAD ZZX0003QXZZ ("x30 repaint").
The table "To be confirmed in F2" was traveled whole** before** writing any code, and it
better have been: two claims of the plan fell, and one of them blocks.

### Check table result

| Statement of the plan | Checked in Tree | verdict |
|---|---|---|
| Branch default is Jazzy | default branch is `main`; README line ZZX0002QXZZ says "developed under ROS2 Jazzy", Humble has own branch |  |
| Supports Harmonic | `gz_quadruped_hardware` depends on `gz_sim_vendor`/`gz_plugin_vendor`; `descriptions/README.md` §2 asks ZZXQ005QXZZ + ZZXQ006QXZZ | ✅ |
| Apache-2.0 License | root is Apache-2.0, and **all code**(controllers, commands, libraries, hardware) declares Apache-2.0 | ** only for code** |
| No A1 config | **False. ** `descriptions/unitree/a1_description/` exists, complete |  |
| What comes from `chvmp/robots` | Nothing. Descriptions do not come from `chvmp/robots`; A1 has maintainer ZZX0002QXZZ, i.e. direct Unitree origin |  |

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
`a1_description` file (nor ZZXQ005QXZZ or ZZXQ006QXZZ self-generated). The only source
sign is the maintainer `laikago@unitree.cc`. ZZXQ008QXZZ in root covers only
`legged_control` and `unitree_guide` — no description of robot.

* * Practical consequence:** A1 is the target robot of the demo. F3 sells precisely this
description. Vendorizing file without license declared in a commercial demo of Toradex
is exactly the risk the project has already decided not to take twice.

### What that doesn't block

The F2 gate is **Go2**, and `go2_description` declares **BSD** — valid license, and is
the description that the spike would use. The lock is from F3 onward, not the spike
itself. But running F2 without solving this means spending the spike phase to prove a
base whose destination (ZZXQ005QXZZ) is legally indefinite.

I didn't write code because the decision changes the target of the job, not just his
order.

### Possible paths — **decision taken: path 1**

1. **[ESCOLHIDO] Switch the target robot from A1 to Go2.** `go2_description` is BSD,
   has `ocs2`, `legged_gym`, `himloco` and `robot_lab`, and is the most exercised
   robot in the repo — including with `gazebo_rl_control.launch.py` itself, which
   ZZXQ005QXZZ does not have. Eliminates the blocker and reduces the risk of F3,
   which is delayed. Accepted cost: the original request names A1; the demo is now
   called Go2 quadruped.
2. Track the actual license of A1 upstream (`unitree_ros`) and follow if it is
   BSD-3 — not followed, tracking cost did not compensate with Go2 available.
3. Accept the risk explicitly — not followed.
4. Back to option B (visual quadruped on diff-drive) — not followed.

If A1 is a hard demo name requirement, the 2 path becomes a prerequisite before F3 turns
out the description — but nothing in F2/F3 technically requires ZZXQ005QXZZ
specifically; the topic contract (ZZXQ006QXZZ) does not distinguish the two.

## F2 — spike executed 14/08/2026, slammed gate

Disposable image `demo-sim:spike-go2` (Dockerfile in `/tmp/f2-spike`, **not committed**
—is spike, does not enter the tree). ZZX0002QXZZ + apt
`ros-gz-sim`/`ros-gz-bridge`/`ros2-control`/ZZXQ00006QXZZ/`gz-ros2-control` (only to
satisfy build headers; the plugin that actually runs is ZZXQ008QXZZ ** from the clone
itself**, not from the apt — see found below), ZZX0009QXZZ clone with the packages that
the non-builda spike removed before the ZZX0010QXZZ (only removal from what does not
build: nothing the spike uses has been touched). Build via `colcon build
--packages-up-to go2_description unitree_guide_controller keyboard_input
gz_quadruped_playground`. 7 packages, clean build.

**Spike Launch** (`/spike/spike_go2.launch.py`, also uncommitted) reflects
`unitree_guide_controller/launch/gazebo.launch.py` upstream without modifying it, with
two deliberate changes: RViz2 removed (rule 1 — the `viz` of the actual project is in
the host, outside the `sim` container) and Gazebo headless (ZZX0005QXZZ, without
ZZX006QXZZ). A ** spike bridge** (`twist_to_inputs.py`, idem) translates ZZXQ008QXZZ
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
| In TROTTING stopped (`cmd_vel`=ZZX0002QXZZ) | 0.15 m | identity | march in lower position, but did not fall |
| Walking, `linear.x=0.03` (low win), 8 s continuous | **0.343 m sustained** | near identity | * Standing, stable, without falling * |
| Walk, `linear.x=0.15–0.3` | drops to 0.07–0.24 m, tipping orientation | robot loses balance | **Symptom, no structural failure** |

**Gate beaten in low gain**: Go2 upstream, unmodified, standing and walking by `cmd_vel`
within a container shaped as `sim`. Zero errors in the entire execution log (`grep -c
"Err\]"` = 0).

* The fall in high gain does not block the gate. The guide already recorded the risk
before running: *"Studented gait parameter... produces robot that walks badly without
generating error... The F3 gate is robot standing and stable responding to `cmd_vel`,
not built clean."* The probable cause is the spike bridge being a naive linear mapping
of `Twist` for the ZZX0003QXZZ standard joystick of `Inputs`, without the speed limits
(ZZXQ005QXZZ) that ZZXQ006QXZZ of real joystick would respect — ** this is the
responsibility of F4** (the actual bridge of the contract), not ZZXQ008QXZZ.

### A silent trap at this stage too

Test the state machine manually via `ros2 topic pub .../control_input` **while the spike
bridge of the launch still ran in parallel** produced two publishers competing for the
same topic and a state setback (`trotting → fixed stand → fixed down`) that seemed to be
controller instability and was not — it was two test processes competing for the same
`/control_input`. Diagnosed by reading `StateTrotting::checkChange()` directly (file
line 76-ZZQ005QXZZ): ZZXQ006QXZZ force returns to `FIXEDSTAND` even in stable trot.
Corrected by isolating a single publisher by test. Registered because it's the kind of
failure that "seems the robot falling" when it's actually the test harness.

### Other facts collected in the clone, for F3 on

- **`gz_quadruped_hardware` is from the repo**, version 2.0.6, license
  `Apache 2`, maintained by Alejandro Hernández / Bence Magyar (is a `gz_ros2_control`
  upstream fork). The plan was supposed to use apt's `gz_ros2_control` ZZX0003QXZZ —
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
  with ZZXQ005QXZZ physical, out of scope — and the ZZXQ006QXZZ container already
  exists for that.

---

## F3 — completed 17/ZZX0002QXZZ/ZZX0003QXZZ (commits `db4e6f3`, ZZXQ005QXZZ)

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
layer by layer of ZZXQ005QXZZ. Copy to `scratchpad/f2-recovered/`.

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

* ==References====External links== `unitreerobotics/unitree_ros`, BSD ZZX0002QXZZ-Claude
with full text and identified holder (HangZhou YuShu TECHNOLOGY CO.,ZZXQ005QXZZ.,
ZZXQ006QXZZ-2022). **ZZXQ008QXZZ meshes are bit-identical** to the upstream, proven by
hash git blob against GitHub's API. Complete table and playback commands in
`ros2_ws/src/go2_description/README.md`.

The xacro layer**does not match the upstream (is port ROS ZZX0001QXZZ → ROS ZZX0003QXZZ
from `legubiao`). Adopted as derivative work covered by BSD-3, with the residual risk
explicitly registered in README instead of erased.

Same thing in the control layer: `package.xml` declare Apache-2.0, but the three
packages derived from `unitree_guide` are covered by
`LICENSES/unitree_guide/LICENSE.txt` of the upstream root, which is **BSD-ZZXQ005QXZZ of
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
editing): `go2_description` (25 MB), ZZXQ005QXZZ, ZZXQ006QXZZ,
`unitree_guide_controller`, ZZXQ008QXZZ. Source: `go2_description/README.md` and
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
  In 17/08/ZZX0002QXZZ the operator repeated the launch with `gui:=true` in the spike
  image and confirmed the Go2 model visible in Gazebo in `empty.sdf`. This closes the
  preview of the model in Gazebo, but does not validate the ZZXQ005QXZZ tree and the
  RobotModel in RViz2.
- * ==References====External links== The gate ran on `empty.sdf`. O
  world of design carries ~10 s and has 50+ meshes; the spawn is now immediate, which
  is safe (`create` does retry), but was not exercised there.
- ** March in high gain. ** Continues what F2 measured: above ~0,15 the robot
  You lose balance. It is mapping synchronous in `twist_to_inputs`, and is **F4**.
- **Nav2 on legs.**F5. Plant does not publish `odom → base_link`.

---

## F4 — completed 24/ZZX0002QXZZ/2026

> * *Continuity:** this section records the checkpoint of 18/08. The drop in HOLD
> it was corrected in 20/08 by `hold.settle_rate: 0.02`, with the three criteria of
> green gear; see `docs/results/ml35-postura-parada.md`. Revalidation
> joint contract and perception was executed in 24/08.

Detailed checkpoint on `docs/results/ml35-f4-parcial.md`. Real names, types and messages
of odom, scan and image crossed two containers by DDS. The first mapping SI → stick
overthrew Go2 and was replaced by clamp in the `0.03` envelope proven in F3, but this
latest edition has not yet been revalidated in runtime. Perception, official warehouse
and diff-drive regression were pending at that checkpoint.

* *Close 24/08/ZZX0002QXZZ:** cold start profile `learn` with Go2, warehouse, Nav2 and
perception in different containers. The five topics were discovered with the contract
types: `geometry_msgs/msg/Twist`, ZZXQ005QXZZ, ZZXQ006QXZZ, `sensor_msgs/msg/Image` and
ZZXQ008QXZZ. Real messages were received from odom, scan, image 640 px and synthetic
detection in the consumer. `/clock` advanced, TF closed and Nav2 reached `Managed nodes
are active`. This closes the F4 gate without making performance or hardware claims.

**18/08/2026 — the march has taken place.** ZZX0003QXZZ was separated into `WALK`,
ZZXQ005QXZZ and ZZXQ006QXZZ, and `twist_to_inputs` won watchdog command. Three flaws
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
ZZX005QXZZ m covered, no entry into ZZXQ006QXZZ, maximum tilt 2,3°, average speed
measured ZZXQ008QXZZ m/s. Two causes, both measures before any adjustment:

1. * *The command was outside the marching regime.** `_SAFE_STICK_LIMIT = 0.03`
   was documented as a "stable envelope of F3", but was measured while the gait never
   activated — described the push on a planted foot robot, not walking speed. The
   `v_cmd = 0,004 m/s` step requested is 4 mm under standing elevation of 8 cm: the
   robot marched in place. High to `0.5`.
2. * *The course is not controllable by QP on this robot.**Instrumenting `bd_` against
   `A_ * F_`, the time of turn request was locked in ±5,3 N·m = stop `d_wbd(2) ±10
   rad/s²` times `Izz`. With ZZX0004QXZZ this stop satura with **0,73°** swivel
   error, and above that the signal becomes chosen by the gyroscope wave, not by
   error. Towing in robot with legs controls with where the foot lands: ZZXQ00006QXZZ
   in `FeetEndCalc` was worth 0,005 against the 0,1125 that needs to cancel at the
   moment of landing, so the support pattern was rotated and the legs crossed to the
   center. `k_yaw_ = 0.15` solves.

Extend the stop (±25) and dismember the attitude gains per axle were tested and
**rejected by measurement** — evidence in `ml35-f4-parcial.md`.

This was the state at 18/08. Correction and replacement criteria are in the
20/ZZX0003QXZZ report cited above; do not use this historical paragraph to choose the
next experiment.

---

## Preparation of target — 20/08/2026

* It's not a phase. It is infrastructure work for F5, done in parallel to F4 trials on
the host, because the module has become accessible. Full evidence on
`docs/results/ml35-target-preparacao.md`.

What changed state in the project:

1. * * The premise "the module is not accessible" fell.** Aquila AM69 inventoried:
   Torizon OS 7.7.0+build.40, 8 × Cortex-A72, ZZXQ005QXZZ GiB ZZXQ006QXZZ, 108 Free
   G, Docker ZZXQ008QXZZ arm64, Compose 2.26.0, `torizon` in group `docker`.
   `ethernet0` in `<MODULE_IP>/24`; host x86 in `<HOST_IP>` in the same /24.
2. * *The DDS network was measured, not assumed.** UDP in both directions in three ports
   69 domain. `ufw` is active in the host and ** does not block**. No firewall
   changes are required.
3. * *The 1 rule was being violated by the current tree in silence. The layer of
   container is F1 (diff-drive); F3 brought in `gz_quadruped_hardware`, which
   declares `gz_sim_vendor` and `gz_plugin_vendor` as ZZXQ005QXZZ. A blind
   ZZXQ006QXZZ from `src` would put ZZXQ008QXZZ 2 on the arm64 images. Fixed by two
   build args (`SKIP_KEYS_EXTRA`, `COLCON_IGNORE_PACKAGES`), both default empty — the
   amd64/host side does not change.
4. **`autodetermine` no `module.xml` was a real trap**, not theoretical: a
   Toradex easy-pair bridge Docker (`br-*`, 192.0.2.9) is UP along with `ethernet0`.
   The interface is now fixed in rendering time, and the host peer is injected there
   too, so no address enters git.
5. **`scripts/module.sh`** became the interface for the module:
   'inventory | sync | build | up | down | status | verify | shell`.
6. * *Four images `arm64` exist in the module**, built natively there:
   `base` 1,24 GB, `perception` 1,28 ZZXQ005QXZZ, ZZXQ006QXZZ 1,32 ZZXQ008QXZZ,
   ZZXQ0009QZZ 2,44 GB. 1 rule verified in the four by inspection of installed
   libraries.
7. * *The contract crosses the machine border in both directions, measured.**
   69, `/demo/system/heartbeat`: module→host `count=11` received at the host;
   host→module `count=14` received within the `tools` container, with ZZXQ005QXZZ
   visible in ZZXQ006QXZZ of the module. **This is the prerequisite for
   F4/ZZX0008QXZZ infrastructure, not their gate.**
8. * *Discovered that setting only one side of DDS fails identical to firewall.**
   CycloneDDS default announces by multicast (which the module ignores) and does not
   fix deterministic port (then the module unicast has no target). Both sides need
   married config. `scripts/module.sh` renders both: `module.xml` for the module and
   `docker/cyclonedds/host.rendered.xml` on the host, both with injected address and
   gitignored/generated.
9. **`ROS_NAMESPACE` does not work on ROS ZZX0002QXZZ Jazzy.** Verified: variable is
   in the process environment (`printenv` confirms) and ROS ignores it; only
   `--ros-args -r __ns:=` works. `scripts/env.sh` exports `ROS_NAMESPACE=/demo` as if
   it worked — ** has not been changed**, the file is in use by ZZXQ005QXZZ trials.
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

## F5 — Nav2 on legs: ongoing 20/08/ZZX0003QXZZ

Closed and functioning mesh: cloud 3D → costmap → planner → MPPI → unit conversion →
march → Gazebo → odometry → TF → costmap. Measured in `quadruped_objects.sdf`, the robot
traveled 8,36 m, displaced ZZXQ005QXZZ m liquids, reached **3,8 cm** of the goal and
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
3. **`progress_checker` of TB4.**0,5 m in 10 s, against 13 s in ZZXQ005QXZZ rad/s
   no advance: 22 abortions with **zero falls**. When the verifier fails and the
   robot doesn't fall, the suspect is the verifier.
4. * ==References====External links== ZZX0001QXZZ s cover 1,4 m no TB4 e 0,42 m no
   Go2 — below the reference of ~1 m of `PathAlignCritic`, which has the highest
   weight. Horizon measures in distance.
5. **`/demo/cmd_vel` is not in SI.** Loads manche; controller multiplies
   `linear.x` by 0,4 and `angular.z` by 0,5 (`StateTrotting.cpp:192` with
   ZZXQ005QXZZ, and ZZXQ006QXZZ with unit gain). Nav2 is the first consumer who
   cannot live with it, because MPPI**integrate** ZZXQ008QXZZ as m/s. Fixed with
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
  reducing 640×480 to ZZXQ005QXZZ Hz, and repeat the same protocol.

## F6 — selectable fallback: completed 24/ZZX0002QXZZ/2026

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
ZZXQ005QXZZ** (see "To be confirmed in F2").

Discarded:

- **`chvmp/champ`** (BSD-3): ZZX0003QXZZ 1 only (Kinetic/Melodic), latest update
  ~Jul/2024. Porting would be to rewrite middleware + build + control layer.
- **`khaledgabr77/unitree_go2_ros2`** and **`RobInLabUJI/unitree_go2_ros2_jazzy`**:
  Jazzy + Harmonic + CHAMP, but Nav2 marked "coming soon" ** and undeclared license**
  — commercial demo blocker, same criterion that eliminated Fuel Tugbot no ML3.1.
- **`arjun-sadananda/go2_nav2_ros2`** (registered at ML2): single CHAMP+Nav2
  demonstrated, but Humble + Gazebo **Classic**, and compensates odometry error by
  doubling the linear velocity in the state estimator. Outline, not calibration.

Verified in 14/08/ZZX0002QXZZ: still no** quadruped A1 ready in Jazzy + Harmonic + Nav2.
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
appear that the decision of ZZXQ008QXZZ was forgotten.

---

## To be confirmed on F2 — 

The table below is what was intended to be verified. It was verified in
14/08/ZZX0002QXZZ and **two statements fell**. Kept as a record of what was asked; the
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
is what justifies `sim` being a single container), and if `gz_ros2_control` ZZX0004QXZZ
home with the version that the base awaits.

---

## Verified environment (14/08/ZZX0002QXZZ, host x86)

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
recommends FastDDS. The project's inviolable rule ZZX0002QXZZ is ZZX0003QXZZ always.

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
  verified interfaces are `ethernet0` and `ethernet1`; HIL ZZX0003QXZZ/08 was
  explicitly chosen ZZXQ005QXZZ, and it is fixed in rendering time**, detected from
  ZZXQ006QXZZ, not written by hand: `autodetermine` can choose the bridge easy-pairing
  docker (`br-*`, ZZX0009QXZZ), which is UP at the same time. `host.xml` is template;
  `module.sh sync` generates `host.rendered.xml` with `enp0s31f6` and the chosen peer.
  With both ports on the same subnet, `MODULE_IP` should be explicit because the same
  hostname mDNS can solve for any of them.
- `tools` appears on `docker compose exec tools` in the §ZZX0002QXZZ tab but is not
  declared in the composition §6. It will be declared with `profiles: ["tools"]` and a
  `command` that does not close.
