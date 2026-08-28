# Go2 — command guide for testing the application

Operating manual for the quadruped spike in simulation. Everything here runs on
the **x86 workstation**, nothing on the Aquila module — Gazebo is OGRE 2 and
cannot come up on the AM69 (see `CLAUDE.md`, rule 1).

State of the application on 20/08/2026: the robot **walks, stops and stays
stopped repeatably**, and runs an **exposure routine in a loop** indefinitely
(§7.3) — 300 s of continuous motion without a single fall. What was the blocker
of 19/08, the fall in `HOLD` between ~18 s and ~46 s (Defect 2, §7.5), is
closed.

On 20/08/2026 **straight-line walking** was fixed: `foot_placement.k_yaw` 0,15 →
0,25 dropped the lateral deviation from 1,14–1,56 m to 0,08–0,11 m and the heading
drift from 35–39° to less than 1°, with zero falls and commanded-yaw tracking
rising from 85% to 99% (`../results/ml35-caminhada-reta.md`).

Also on 20/08/2026 the **long stop was closed**: `hold.settle_rate = 0.02`
re-anchors the body reference on the support centroid during `HOLD`, and the
robot went from collapsing at 161,7 s to 180 s standing with zero `RECOVER`,
while keeping the three F4 criteria (`../results/ml35-postura-parada.md`).
Lowering the yaw-moment weight during the stop was measured and **rejected** — it
makes things worse.

Two visible behaviours remain **open**, both measured and neither caused by the
fixes above:

- **Tremor while stopped**: ~2,6° of yaw peak to peak, peaks of 14 °/s. It
  predates the fixes; `k_yaw` already cut it from 6,1° to 2,3°.
- **Crabbing while walking**: the heading closes within 0,2°, but the robot
  slides sideways by ~2% of the forward distance. The cause is **Defect 1**: the
  estimator believes it walked straight (`estPos` in y ≈ 0) while the real
  `/demo/odom` shows 0,10–0,17 m of deviation. No gait gain fixes this — `k_y` was
  swept and refuted. Closing that loop is navigation's job, by contract.

Evidence in `docs/results/ml35-f4-parcial.md`; plan and experiment table in
`../ml35/plano-movimentacao.md`; implementation context in
`go2-proximos-passos.md`.

---

## 1. Prerequisites

Once per machine:

```bash
# the spike image must exist
docker images | grep demo-sim
# expected: demo-sim   spike-go2   ...
```

The script already takes care of `xhost` and `/dev/dri`. If Gazebo opens black,
the problem is host graphics acceleration, not the application.

---

## 2. Bring up the simulation

```bash
cd ~/toradex/demo/aquila-am69-ros2
./scripts/run_quadruped_sim.sh
```

Keep the terminal open — it is what holds the `aquila-go2` container. The script
copies `ros2_ws/src` inside, runs `colcon build` and launches
`demo_simulation quadruped.launch.py`. **The first startup takes ~2 min** because
of the build.

For a different world:

```bash
./scripts/run_quadruped_sim.sh /caminho/para/mundo.sdf
```

### Wait for the state machine

Do not send a command before this appears:

```bash
docker logs -f aquila-go2 2>&1 | grep --line-buffered "gait FSM"
```

Expected sequence: `passive` → `fixed down` → `fixed stand` → the line
`gait FSM: fixed stand. Waiting`. Only from there on does the robot accept
velocity.

To block until it is ready, in a script:

```bash
until docker logs aquila-go2 2>&1 | grep -q "gait FSM: fixed stand. Waiting"; do sleep 5; done
```

---

## 3. Open a shell with the ROS environment

Every `ros2` command in the following sections assumes this environment:

```bash
docker exec -it aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh && . /test/install/setup.sh && exec bash'
```

From the **host** it works too, as long as the environment matches —
`ROS_DOMAIN_ID=69` and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. Swapping the RMW
breaks discovery between containers (`CLAUDE.md`, rule 2):

```bash
export ROS_DOMAIN_ID=69 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## 4. Check the topic contract

```bash
ros2 topic list | grep demo
ros2 topic hz /demo/odom              # expected ~100 Hz
ros2 topic hz /demo/scan
ros2 topic hz /demo/camera/image_raw
ros2 node list
```

Types, to confirm the bridge is up:

```bash
ros2 topic info /demo/cmd_vel         # geometry_msgs/msg/Twist
ros2 topic info /control_input        # control_input_msgs/msg/Inputs
```

Do not pipe these commands into `| head`: `ros2 topic info` ends with a
`BrokenPipeError` and pollutes the output for no reason.

`/demo/cmd_vel` is the public interface. `/control_input` is internal to the
controller, emulating a gamepad — do not publish to it directly.

---

## 5. Make the robot walk

```bash
ros2 topic pub -r 20 /demo/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25}}"
```

`Ctrl-C` to stop. **Publish at 20 Hz or faster**: the bridge has a 0,3 s
watchdog and silence means stop.

### Command conversion

The `Twist` goes through a bridge that emulates a gamepad, so the number you
publish is not the final velocity:

| `Twist` | limit | resulting velocity | maximum |
|---|---|---|---|
| `linear.x` | stick ±0,5 | `0,4 × x` m/s | 0,20 m/s |
| `linear.y` | stick ±0,5 | `0,3 × y` m/s | 0,15 m/s |
| `angular.z` | stick ±0,5 | `0,5 × z` rad/s | 0,25 rad/s |

So `linear.x = 0.25` → **0,1 m/s**, which is the validated operating point.

> Do not command very low velocities thinking that is safer. The trot has a
> design point: below ~0,05 m/s the step length becomes millimetric under an 8 cm
> foot lift, and the robot marches in place and destabilizes. This is measured in
> `ml35-f4-parcial.md`.

Walking backwards, sideways and turning:

```bash
ros2 topic pub -r 20 /demo/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.25}}"
ros2 topic pub -r 20 /demo/cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.25}}"
ros2 topic pub -r 20 /demo/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.3}}"
```

Commanded yaw **works, with measured margin**: over 15 s at `w_cmd = 0,10`
rad/s (`angular.z = 0.2`) the robot delivers 72,7° of the 86° commanded — 85%.
But the yaw axis operates pinned against the clamp limit (`yawSat = 100%`, §6):
it is on/off, not a regulator. And walking **while turning** is the worst case
for Defect 2 — it is precisely the trigger used to reproduce it (§11).

---

## 6. Read the diagnostic line

The trot supervisor prints at 4 Hz. It is the main instrument:

```bash
docker logs -f aquila-go2 2>&1 | grep --line-buffered "trot supervisor"
```

```
trot supervisor: mode=WALK cmd=(0.1000,0.0000,0.0000) tilt=0.4deg
  posErrXY=0.0071 velErrXY=0.0564 contact=[1 0 0 1]
  estPos=(0.041,0.072,0.334) estVel=(0.071,-0.029) estYaw=-0.0deg
  Mz=5.3/5.2Nm Fz=142/142N footErr=[0.010 0.008 0.008 0.010]
  yawErr=.../pk...deg dWzPk=... dWzMed=... yawSat=...%
```

The last line is the yaw-axis instrumentation, elided here because there is no
measured reading of it during straight-line walking — the signature that does
exist is from `HOLD` and appears further down.

| field | what it means | healthy value |
|---|---|---|
| `mode` | `HOLD` stopped, `WALK` walking, `RECOVER` losing attitude | — |
| `cmd` | velocity that reached the controller (m/s, m/s, rad/s) | matches the table in §5 |
| `tilt` | body angle against the vertical | < 2° |
| `posErrXY` | error against the body position reference | < 0,015 m |
| `contact` | feet on the ground, order FR FL RR RL | alternates `[1 0 0 1]` ↔ `[0 1 1 0]` |
| `estPos` / `estYaw` | the estimator's belief | see the note below |
| `Mz` | yaw moment **requested/delivered** | the two close together |
| `Fz` | vertical force **requested/delivered** | ~142/142 N |
| `footErr` | distance from each foot to its target | < 0,02 m and **symmetric** |
| `yawErr` | yaw error **current/window peak** | oscillating around zero |
| `dWzPk` / `dWzMed` | yaw acceleration demand **before the clamp**, window peak and mean | ≪ `yaw_clamp` (10 rad/s²) |
| `yawSat` | % of the window's ticks with the axis at the limit | 0 in steady state |

In the yaw fields, peak and mean are **accumulated over the window and reset at
every print**. That is not a detail: sampling a 500 Hz relay at 4 Hz without
accumulating is aliasing, and that is what kept the defect invisible. Signature
measured in `HOLD`, moments before a fall:

```
yawErr=4.86/pk5.31deg  dWzPk=366.2  dWzMed=287.6  yawSat=100%
```

Read that as three facts: the error is **pinned to one side** (3,5–5,3°, never
crossing zero) because `kp_w = 780` against a clamp of 10 rad/s² gives a
proportional band of 0,73°; the demand is 33× the clamp; and the axis is at the
limit on 100% of the ticks. That is not a healthy reading — it is Defect 2
forming.

Reading notes that were expensive to discover:

- `estPos(2)` sits ~24 mm **below** the real value. It is a known, constant bias
  (`foot_radius = 0.02` against `feet_h_ = 0` in the estimator), not drift.
- `Mz` locked at ±5,3 with alternating sign is the **limit**, not control. The
  ceiling of 5,3 N·m is the QP's on this robot and is not a failure in itself —
  but sitting there **continuously**, without alternating, is: see `yawSat` above
  and §7.5. Raising the clamp was measured and is worse (13,2 requested, 5,8
  delivered, fall at 4 s).
- `footErr` that is **asymmetric and constant** is the signature of the
  stale-target bug already fixed. If it comes back, it is a regression.

---

## 7. Test scripts

### 7.1 Continuous walking

```bash
ros2 topic pub -r 20 /demo/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25}}" &
sleep 30 && kill %1
```

Acceptance: no `mode=RECOVER` line, `tilt` always < 2°, ~3 m covered.

### 7.2 Repeated walking and stopping — the test that catches regressions

```bash
docker exec aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh && . /test/install/setup.sh
  for c in 1 2 3 4 5; do
    echo "ciclo $c antes:"; timeout 12s ros2 topic echo /demo/odom --field pose.pose --once
    timeout 8s ros2 topic pub -r 20 /demo/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25}}" >/dev/null 2>&1 || true
    sleep 8
    echo "ciclo $c depois:"; timeout 12s ros2 topic echo /demo/odom --field pose.pose --once
  done'
```

Acceptance measured on 18/08/2026: **zero falls**, maximum `tilt` 1,0° walking
and 0,9° stopped, `footErr` from 0,4 to 1,3 cm and symmetric, 3,85 m over 5 cycles.

Heading **became a criterion** on 20/08/2026. With `k_yaw = 0,25` the drift
measured over 5 cycles is **−0,3°**; the earlier wording of this section ("random
walk by design, 12° to 43°") described `k_yaw = 0,15` and no longer holds — the
random walk was a positive feedback loop in foot placement, not a design choice
(`../results/ml35-caminhada-reta.md`). Acceptance: drift < 5° over 5 cycles.

What remains Nav2's job is the **absolute** heading between movements:
`captureBodyReference()` re-anchors the reference at every stop, so an already
accumulated deviation is not recovered.

### 7.3 Continuous exposure routine

The `demo_routine` node drives the simulation on its own, in a loop, with a
posture settle between each movement. It runs on the host or on the module — it
speaks only the public contract `/demo/cmd_vel` and reads `/demo/odom` only to
know whether the robot is standing.

```bash
ros2 launch demo_bringup routine.launch.py
```

**Never run it together with `nav.launch.py`**: both publish to `/demo/cmd_vel`
and the commands interleave.

One pass lasts 233 s and is closed by geometry: a four-sided 0,8 m box with a
90° turn in place, a circle of radius 1,0 m in four 90° arcs, and the
left/right lateral pair. Measured: 0,183 m of closure error per pass, +23,2° of
precession, maximum radius of 2,18 m around the start point, zero falls in
300 s (`../results/ml35-postura-parada.md`).

The posture settle between movements is **silence**, not a `Twist` of zeros:
publishing zeros keeps the command fresh and the robot never reaches `HOLD`. The
0,3 s watchdog (§7.4 below) is what converts silence into a stop.

Useful parameters:

```bash
ros2 launch demo_bringup routine.launch.py settle_s:=8.0 move_s:=6.0 loop:=false
```

If the figure translates instead of precessing, someone replaced the
choreography with arc pairs with an inverted `wz` sign — that does not close,
and the measured cost was 1,59 m of drift per pass.

---

### 7.4 Command watchdog

```bash
timeout 5s ros2 topic pub -r 20 /demo/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25}}"
```

Acceptance: within 0,3 s after the `timeout`, the supervisor shows `mode=HOLD`
with `cmd=(0.0000,0.0000,0.0000)`. The robot **must not** keep walking.

### 7.5 Prolonged stationary posture — known failure (Defect 2)

**Fixed on 20/08/2026 by `hold.settle_rate = 0.02`.** The text below describes
the failure because it comes back as soon as that value leaves the YAML, and
because the obvious fix is the wrong one.

`captureBodyReference()` freezes `pcd_` at the body position at the instant of
the stop; the feet keep sliding after that. The frozen reference ends up
displaced from the support centroid by the residual the stop was carrying, the
QP receives an impossible force-distribution request, the yaw-moment residual
saturates at 100% and the robot buckles. `settleHoldPosture()` walks `pcd_` back
to the centroid at 0,02 m/s, rate-limited so it does not request a force step.

| condition | `Syaw` in `HOLD` | `settle_rate` | 180 s stop | `RECOVER` | `yawSat` |
|---|---|---|---|---|---|
| old default | 450 | 0 | collapse at 161,7 s | 74 | 100% |
| lower the weight only | 100 | 0 | **collapse at 91,1 s** | **357** | 100% |
| **promoted** | 450 | 0,02 | **180 s standing** | **0** | 10–66% |
| both | 100 | 0,02 | 180 s standing | 0 | 54–90% |

Lowering `balance.weight_moment[2]` during the stop looks like the fix and **is
not**: it brought the collapse forward by 70 s and multiplied `RECOVER` by five.
Softening the residual does not remove the moment request, it removes the reason
for the QP to distribute force against it. Do not repeat this on its own.

Without `settle_rate`, the time to fall scales with the residual the gait
carried into the stop — read `velErrXY` on the first `HOLD` line:

| `velErrXY` on entering `HOLD` | how it got there | time to fall |
|---|---|---|
| 0,010 | straight, clean start | did not fall in 90 s |
| 0,020 | straight, after 5 cycles | 46,1 s |
| 0,034 | after a turn | 17,8 s |

Repeatable, not deterministic — which is why a condition only counts with the
fixed trigger from §11, and never with n = 1.

That also explains why the green F4 criterion uses **8 s** stops: in 8 s you
only see the beginning of the transient. Inspecting a short stop, which is
today's criterion:

```bash
docker logs aquila-go2 2>&1 | grep "mode=HOLD" \
  | sed -E 's/.*tilt=([0-9.]+)deg.*footErr=\[(.*)\]/tilt=\1 err=[\2]/' | tail -20
```

Acceptance on an 8 s stop: `tilt` < 1°, `footErr` symmetric and < 2 cm, no
`RECOVER`.

To exercise the long stop on purpose, use `--final-hold` (§11): the harness
records the fall instead of aborting the trial.

---

## 8. Measure pose and drift

Single pose, readable:

```bash
ros2 topic echo /demo/odom --field pose.pose --once
```

Pose with yaw in degrees:

```bash
ros2 topic echo /demo/odom --field pose.pose --once | python3 -c '
import sys, math, re
t = sys.stdin.read()
g = lambda k: float(re.search(k + r": (-?[0-9.e-]+)", t).group(1))
x, y, z = g("x"), g("y"), g("z")
o = re.search(r"orientation:(.*)", t, re.S).group(1)
q = lambda k: float(re.search(k + r": (-?[0-9.e-]+)", o).group(1))
yaw = math.degrees(math.atan2(2 * (q("w") * q("z") + q("x") * q("y")),
                              1 - 2 * (q("y") ** 2 + q("z") ** 2)))
print(f"pos=({x:.3f},{y:.3f},{z:.3f}) yaw={yaw:.2f}deg")'
```

The names `x`, `y`, `z` are extracted before the f-string on purpose: escaped
quotes inside an f-string do not survive the shell's quote nesting.

`/demo/odom` is Gazebo ground truth. The controller's internal estimator shows
up as `estPos`/`estYaw` on the diagnostic line — comparing the two is what
separates "the robot fell" from "the controller thinks it fell".

Quick summaries of a run:

```bash
docker logs aquila-go2 2>&1 | grep -c "mode=RECOVER"                     # falls
docker logs aquila-go2 2>&1 | grep "mode=WALK" \
  | sed -E 's/.*tilt=([0-9.]+)deg.*/\1/' | sort -g | tail -1             # max tilt
```

---

## 9. Unit tests

```bash
docker exec aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh && cd /test
  colcon test --packages-select demo_simulation
  colcon test-result --verbose'
```

Expected: **20 tests, 0 failures**. They cover the `Twist` → `Inputs` mapping,
the watchdog and the trot start latch.

---

## 10. Failure signatures

Every line below was observed and diagnosed in this application.

| Symptom | Likely cause | Where to look |
|---|---|---|
| `state=fixed stand` forever, with the bridge publishing | start handshake lost — the controller reads a struct, not a stream | `_StartLatch` in `twist_to_inputs.py` |
| marches in place, does not translate | command below the gait's design point | table in §5; use `linear.x ≥ 0.15` |
| legs cross towards the body centre | support pattern rotating with the yaw rate | `k_yaw_` in `FeetEndCalc.cpp` |
| turns in place, drags the feet, topples over while standing | stale stance target on one diagonal pair | asymmetric `footErr`; `GaitGenerator::generate` |
| walks backwards with a positive command | QP chasing a moment it cannot deliver | `Mz` requested ≫ delivered |
| falls on the first step, `footErr` > 30 cm | joint PD dominating the force controller | `calcGain`, must be 3,0/2,0 |
| `tilt` > 12° | the supervisor enters `RECOVER` and cancels the command | expected; investigate what came before |
| topples while stopped, 18–46 s after the gait stops | **Defect 2**: yaw axis at the limit pumping residual, and the QP shifting force distribution to chase an unreachable `Mz` | `yawSat` and `velErrXY` at the entry into `HOLD`; §7.5 |
| `yawErr` pinned to one side, never crossing zero | proportional band of 0,73° — the axis is a relay, not a regulator | `trot.kp_w` and `ang_acc_limit_yaw` in `gait_go2.yaml` |
| `dWzMed` ≈ `dWzPk`, in the hundreds | derivative term reading trunk vibration, ~50× the body's real rotation | `trot.kd_w`; this is experiment B1c |
| **walks in a curve**, `yawErr` grows monotonically and `Mz` stays pinned at the ceiling | positive feedback in foot placement: the neutral term follows the measured rotation and `k_yaw` does not cancel it outside touchdown | `foot_placement.k_yaw` (≥ 0,25); `../results/ml35-caminhada-reta.md` |
| toppled with reduced yaw `kd_w` | the derivative term at 70 is damping that walking needs, not just noise | negative result already measured; do not repeat B1c in isolation |
| topples while stopped after ~90–160 s, `yawSat` = 100% over the whole window | body reference frozen away from the support centroid by the stop's residual | `hold.settle_rate`; §7.5 |
| topples while stopped **earlier** after lowering the yaw-moment weight | softening the residual removes the reason for the QP to distribute force, not the moment request | negative result already measured; do not repeat in isolation |
| **trembles while stopped**, ~2,6° of yaw peak to peak | yaw-axis residual; predates the settle, already reduced 2,7× by `k_yaw` | open; amplitude per stop in `../results/ml35-postura-parada.md` |
| **drifts off the line while walking with a stable heading** (crabbing, ~2% of the distance) | **Defect 1**: the robot faithfully tracks an estimator that drifts; `k_y` was swept and refuted | compare the supervisor's `estPos` with the real `/demo/odom`; §8 |

---

## 11. Gait tuning and instrumented trial

Trot tuning is no longer compiled into literals. It lives in
`ros2_ws/src/demo_simulation/config/gait_go2.yaml` and is injected by the
spawner:

```bash
# use another file without rebuilding
ros2 launch demo_simulation quadruped.launch.py \
  gait_params:=/test/src/demo_simulation/config/minha_varredura.yaml
```

The path is the one **inside** the container, and the file has to exist in
`ros2_ws/src/demo_simulation/config/` before the sim comes up — `/proj/src` is
mounted read-only and copied at startup. How to do this in practice is at the
end of this section.

The file does **not** go in `go2_description/config/gazebo.yaml`: that package
is vendored and its README guarantees the configs are intact byte for byte —
that guarantee is what supports the licensing argument. The `<parameters>` of
the hardware plugin is also in a vendored xacro, so the injection point that is
ours is the spawner.

### Prove which values are in effect

Two lines, and both matter. The first proves that the file was applied:

```bash
docker logs aquila-go2 2>&1 | grep "node arguments"
# ... --params-file .../go2_description/config/gazebo.yaml
#     --params-file .../demo_simulation/config/gait_go2.yaml
```

Our file has to appear **last** — that is what makes it override. The second
shows what the controller actually loaded:

```bash
docker logs aquila-go2 2>&1 | grep "gait params:"
# gait params: period=0.450 st_ratio=0.500 height=0.080 k=(0.0050 0.0050 0.1500)
#              kp_w=780.0 yaw_clamp=10.0 band=0.0100 S_moment=(450 450 450) mu=0.40
```

Without the first line, values equal to the compiled defaults are
indistinguishable from "the file was not read". Read both before believing a
sweep.

### Instrumented trial

`scripts/gait_trial.sh` drives the trial and records the evidence, replacing the
`for` loop of `ros2 topic pub` from §7.2:

```bash
./scripts/gait_trial.sh /tmp/run.csv --v-cmd 0.10 --cycles 5 --walk 8 --hold 8
```

What it does that the manual version did not:

| | Why |
|---|---|
| aborts if `z <= --min-z` (0,30 m), before the trial and before each cycle | an entire sweep was already lost measuring a toppled robot being dragged, with plausible-looking numbers |
| paginates the phases by **simulation time**, with both time bases on the same CSV line | 8 s of wall clock are only 8 s of simulation at RTF 1 |
| stops publishing instead of publishing zeros | the bridge watchdog is the stopping mechanism; publishing zero tests a path the robot never takes |
| `--v-cmd`/`--w-cmd` in SI, with the stick conversion applied and excess refused | `v_cmd = 0,4 × linear.x`; asking beyond the envelope would be clamped silently |
| `--final-hold N` stays stopped for N s after the last cycle and **records** the fall instead of aborting | in that stretch, falling is the measurement, not a precondition violation |

Summary on stderr (path length, net displacement, heading drift, peak tilt per
phase, RTF); CSV in the file.

The summary's header lines cover **only the walk/stop cycles**. The
`--final-hold` is a different experiment — there the robot may fall — and a body
sliding on its back adds metres of "path" and drags down the average speed:
with both windows summed, one run reported 0,0222 m/s where the correct value
was 0,0629. The final stop appears on its own line, in one of two forms:

```
final hold     survived 89.9 s standing
final hold     COLLAPSED at 39.4 s (z <= 0.30 m)
```

### Count the trial's `RECOVER`, not the session's

`grep -c mode=RECOVER` over the whole log counts everything, including what
happened before and after the trial — and what happens **after** is Defect 2:
left in HOLD with no command, the robot topples between 18 s and 46 s. One
Phase A run returned 23 `RECOVER` with a peak tilt of 2,58° within the trial
window; the 23 were from 31 s after the last cycle.

Cut by the wall-clock window that the CSV itself records:

```bash
ini=$(awk -F, 'NR==2{print int($4)}' /tmp/run.csv)
fim=$(awk -F, 'END{print int($4)+1}' /tmp/run.csv)
docker logs aquila-go2 2>&1 | grep mode=RECOVER \
  | awk -v a="$ini" -v b="$fim" -F'[][]' '{t=int($6)} t>=a && t<=b' | wc -l
```

### Defect 2's trigger — fix it before comparing any condition

The first baseline **did not reproduce the defect**: 3 cycles at
`v_cmd = 0,10` followed by 90 s of `HOLD` stayed standing, with a peak tilt of
0,23°. Consistent with the mechanism (time to fall scales with the residual) and
fatal for the comparison — with n = 1 per condition and a stochastic defect, any
"improvement" against that baseline would be indistinguishable from luck.

The trigger came from the documented worst case, walking **while turning**:

```bash
./scripts/gait_trial.sh /tmp/b0.csv --v-cmd 0.10 --w-cmd 0.10 \
  --cycles 1 --walk 15 --hold 0 --final-hold 90
```

With it the baseline falls at **39,4 s**, within the recorded range, with
`yawSat = 100%` constant on the last `HOLD` lines. Touching `--walk`,
`--w-cmd` or `--final-hold` changes the probability of a fall and invalidates
the comparison: use exactly this trigger across conditions.

And it does **not** replace §7.2. The trigger measures the long stop; the F4
criterion that the four rejected experiments broke is the walk/stop script. A
condition only becomes the default after passing both.

### Sweeping a single YAML line

The shortest path, which works with `run_quadruped_sim.sh` unchanged: edit the
value in `ros2_ws/src/demo_simulation/config/gait_go2.yaml` **on the host,
before bringing up the sim** — the script copies `ros2_ws/src` into the container
at startup. Confirm with the two log lines above, run the trigger, and undo with
`git checkout` at the end.

> `run_quadruped_sim.sh` accepts **only a world path** as `$1`; it does not
> forward `gait_params`. `./scripts/run_quadruped_sim.sh gait_params:=/tmp/x.yaml`
> becomes `world:=gait_params:=/tmp/x.yaml` and Gazebo fails to load the world.
> To use an alternative file by name, use the `ros2 launch ... gait_params:=`
> above, executed inside the container.

Example with experiment B1a, lowering only the yaw entry of the QP's moment
weight (`balance.weight_moment: [450, 450, 100]`) — one line, no rebuild.
Measured result, **n = 1**: it survived 89,9 s where the baseline fell at 39,4 s,
zero `RECOVER`, `yawSat` of 54–84% instead of pinned at 100%, and walking within
3% in path, tilt and `z`. The cost landed on yaw, as predicted: tracking of 76%
against 85%, and +13,5° of heading drift over 90 s stopped.

That is **signal, not a conclusion** — n = 1, and the §7.2 script was not run
under B1a. That is why `gait_go2.yaml` stays at 450. The Phase B experiment
table (B1a, B1b, B1c, B2, B3) is in `../ml35/plano-movimentacao.md`.

---

## 12. Shut down

```bash
docker stop aquila-go2
```

The container is `--rm`; nothing persists. To rebuild after touching
`ros2_ws/src`, just bring it up again — the script always runs `colcon build`.
