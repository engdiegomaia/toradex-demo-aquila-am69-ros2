# Go2 — next-steps implementation guide

This document is the operational roadmap for turning the Go2 spike into a
reproducible simulation, improving the gait incrementally and integrating the
warehouse scenario without losing the diff-drive regression.

## Current state

Already validated:

- the six quadruped packages build in the `demo-sim:spike-go2` image;
- Gazebo opens with the Go2 visible and the sensors publish;
- the robot goes through `PASSIVE -> FIXEDDOWN -> FIXEDSTAND`;
- `/demo/cmd_vel` reaches the bridge and is converted to `/control_input`;
- the FSM switches to `TROTTING`;
- the `WaveGenerator` alternates the contact pairs;
- the diff-drive was not replaced by the quadruped by default;
- the conversion of foot position/velocity to the global frame was fixed;
- the `demo_simulation` suite passed with 10 tests.

Validated on 18/08/2026, after the WALK/HOLD/RECOVER split (full evidence in
`docs/results/ml35-f4-parcial.md`):

- the gait activates at the slowest command in the plan, `Twist linear.x=0.01`;
- the legs do lift for real, in diagonal pairs (`[1 0 0 1]` ↔ `[0 1 1 0]`);
- ceasing to publish zeroes the command within 0,3 s through the bridge watchdog;
- `HOLD` keeps the body free of residual motion (`posErrXY ≈ 0,005 m` for >35 s);
- the attitude supervisor enters `RECOVER` at 12° of tilt.

Validated on 18/08/2026, after the heading correction through foot placement
(`k_yaw_` from 0,005 → 0,15 in `FeetEndCalc`):

- **the robot walks**: 30 s of continuous trot at `v_cmd = 0,1 m/s`, 3,00 m
  covered, no entry into `RECOVER`, maximum tilt 2,3°;
- measured average speed of 0,106 m/s against 0,1 m/s commanded;
- yaw went from divergent (−36° and a fall) to bounded, self-correcting
  oscillation (±23,7°), with a zero yaw command;
- the watchdog stop leads to a standing `HOLD`, with no `RECOVER`, tilt ≤ 0,7°;
- the `demo_simulation` suite passed with 20 tests.

Validated on 18/08/2026, after latching the stance target at foot touchdown
(`GaitGenerator::generate`):

- **repeated walking and stopping is stable**: 5 cycles of (walk 8 s, stop 8 s),
  3,85 m covered, **zero falls**, maximum tilt 1,0° walking and 0,9° stopped;
- the per-foot position error while stopped dropped from a fixed 8 cm on one
  diagonal pair to a symmetric 0,4–1,3 cm;
- stopping now **corrects** the heading in 4 of the 5 cycles, instead of adding
  up to +44° per stop.

Still open:

- **open-loop heading is not held**, by design: with no yaw command the trot
  performs a random walk (final heading of 43°, 29° and 12° across three runs of
  5 cycles). The QP's ceiling of ~5,3 N·m of yaw moment does not allow correcting
  the heading while stopped — trying knocks the robot over (trial 7). Closing the
  loop is Nav2's job, via `angular.z` on `/demo/cmd_vel`; still to be tested;
- the estimated `z` sits 24 mm below the real value, constantly, because of
  `foot_radius = 0.02` against `feet_h_ = 0` in the estimator: every swing-foot
  target aims 2 cm below the ground;
- the `linear.x = 0.01` and `0.03` gates were written under the void premise of
  `_SAFE_STICK_LIMIT = 0.03` and are 25× below the gait's design point —
  they need to be rewritten in terms of `v_cmd` before they can serve as acceptance;
- the warehouse with the Go2, RViz2 and a complete TF tree still need validation;
- F4 must not be marked complete before those gates.

## 1. Prepare the host

Run this on the x86 workstation with Docker, X11 and GPU access:

```bash
cd ~/toradex/demo/aquila-am69-ros2
xhost +local:docker
```

Do not run `ros2 launch` directly on the host if ROS 2 is not installed. Every
ROS command in this guide, except the startup script, runs inside the
container.

## 2. Run the spike application

The script builds the six packages inside the container, uses DDS domain 69 and
keeps the name `aquila-go2`:

```bash
./scripts/run_quadruped_sim.sh
```

The default scenario is the fixture:

```text
quadruped_empty.sdf
```

That world is preferable to Gazebo's `empty.sdf` because it contains the
Sensors system required for the camera and the lidar.

Keep the first terminal open. The container is removed by `--rm` when that
terminal is closed.

### Verify startup

In another terminal:

```bash
docker ps --filter name=aquila-go2 \
  --format 'table {{.Names}}\t{{.Status}}'

docker logs -f aquila-go2 2>&1 | grep -E \
  'gait FSM|gait diagnostics|trot supervisor|watchdog|Switched|controller'
```

Wait for:

```text
gait FSM: fixed stand. Waiting for a non-zero /demo/cmd_vel
```

At that point the robot should be standing, at roughly `z=0.35 m`, with no
motion command.

## 3. Minimum diagnostics before changing code

Two lines per second, from different sources. The controller's line shows what
arrived over the topic:

```text
gait diagnostics: state=... command=... sticks=(...) contact=[...]
```

The trot supervisor's line shows what the controller decided from it:

```text
trot supervisor: mode=WALK cmd=(0.0040,-0.0000,0.0000) tilt=2.0deg \
  posErrXY=0.0147 velErrXY=0.0337 contact=[0 1 1 0]
```

`cmd` is already in SI units, after the stick conversion. The conversion costs
a factor of 0,4: `Twist 0.01 -> ly 0.01 -> 0.004 m/s`. Do not compare `cmd`
with the `Twist` value without keeping that in mind.

Interpretation:

| Observation | Conclusion |
|---|---|
| `ly` does not change after publishing a Twist | problem in DDS or in the bridge |
| `state` does not change to `trotting` | the command arrived before the FSM was ready |
| `mode=HOLD` with a non-zero `Twist` in progress | command below `V_START`, or the watchdog treating the stream as dead |
| `mode=WALK` and `contact` never alternates | problem in the `WaveGenerator` |
| `contact` alternates and the robot falls | dynamics, gains, estimator or QP |
| `mode=HOLD` and `posErrXY` grows | the body is being pushed; it is not a command |
| `mode=RECOVER` followed by `tilt > 90°` | the robot fell; the supervisor merely recorded it |
| `tilt` grows but `cmd` is already zero | posture, not command |

Never conclude success from the `controller ... active` log alone; that does not
prove the joints are holding the body up. And `mode=WALK` on its own does not
prove gait either: only alternating `contact` proves it.

## 4. Posture test without walking

This test enters trot with the slowest command and then **stops publishing**.
Do not publish zeros: the whole point is to exercise the watchdog, which is what
guarantees that silence means stop.

```bash
docker exec -it aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh
  . /test/install/setup.sh

  timeout 3s ros2 topic pub -r 20 /demo/cmd_vel \
    geometry_msgs/msg/Twist \
    "{linear: {x: 0.01}}" || true

  timeout 20s ros2 topic echo /demo/odom --field pose.pose >/dev/null || true
'
```

Criterion, in the supervisor log: `mode=WALK` while publishing, `cmd_vel
watchdog: stale` within 0,3 s after it ends, and then `mode=HOLD`,
`contact=[1 1 1 1]`, `posErrXY` below 0,01 m and `tilt` below 2° for 20 s.

Run on 18/08/2026: passed, with `posErrXY` between 0,0018 and 0,0077 m, stable
for more than 35 s. If it fails, do not test forward motion yet.

## 5. Improve the motion in small steps

Change one variable at a time and keep a record of every trial.

### Step 5.1 — confirm joint tracking

During the test, watch:

```bash
docker exec -it aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh
  . /test/install/setup.sh
  ros2 topic hz /joint_states
'
```

Confirm that there are 12 joints and that the rate does not drop. For a single
sample:

```bash
docker exec -it aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh && . /test/install/setup.sh
  ros2 topic echo /joint_states --once
'
```

If the joints publish but do not follow the references, investigate the PD in
the Gazebo hardware first (`joint_effort + kp*(q_cmd-q) + kd*(qd_cmd-qd)`).

### Step 5.2 — stance gains: **next isolated experiment**

The current value, `Kp=3.0` / `Kd=2.0`, the same as swing, was chosen against a
log in which **no step was ever requested** (`contact=[1 1 1 1]` the whole
time). That evidence no longer holds: with the gait active, the stance leg
becomes force-controlled by the QP, and a stiff joint PD chasing a frozen foot
target fights against it. Upstream uses `Kp=0.8` / `Kd=0.8` in stance.

Trial: go back to the upstream value, rebuild, repeat **exactly** the same 30 s
window at `linear.x=0.01`, and compare time-to-fall and the `tilt` envelope
against the baseline already on record (fall at ~8 s, `tilt` oscillating up to
9°).

```bash
./scripts/run_quadruped_sim.sh
```

If it does not improve, the next suspect is `BalanceCtrl::Ib_`, which has the
A1's inertia hard-coded — see `docs/results/ml35-f4-parcial.md`. Never change
gains, `gait_height`, inertia and period at the same time.

### Step 5.3 — test very slow forward motion

Only after passing the posture test:

```bash
docker exec -it aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh && . /test/install/setup.sh
  timeout 10s ros2 topic pub -r 10 /demo/cmd_vel \
    geometry_msgs/msg/Twist \
    "{linear: {x: 0.01}}"
'
```

Criteria:

- the body stays level;
- `contact` alternates in pairs;
- `/demo/odom` grows monotonically in x;
- there is no fall or abrupt yaw jump.

Only then test `0.02` and, last of all, `0.03`. The value `0.03` is the current
empirical limit, not a guarantee of stability.

### Step 5.4 — test rotation

Do not combine yaw and forward motion in the first trial:

```bash
docker exec -it aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh && . /test/install/setup.sh
  timeout 8s ros2 topic pub -r 10 /demo/cmd_vel \
    geometry_msgs/msg/Twist \
    "{angular: {z: 0.02}}"
'
```

If it rotates in the wrong direction, review the sign of `rx` in
`twist_to_inputs.py` and in `StateTrotting::getUserCmd()`. If it topples,
reduce yaw before touching translation.

### Step 5.5 — safe stop: **implemented on 18/08/2026**

The bridge has a watchdog: `twist_to_inputs` publishes at 20 Hz and zeroes the
sticks when the last `Twist` is more than 0,3 s old. Silence now means stop, and
the controller enters `HOLD` by itself.

What was **not** implemented, on purpose: going back to `FIXEDSTAND` through
`command=2` after N seconds stopped. `HOLD` is trot posture with four feet on
the ground, measured stable for more than 35 s; switching FSM state would hide
any eventual failure of the posture controller behind the `FIXEDSTAND`
controller, which is precisely what we do not want to measure right now.

## 6. Measure odometry and pose

During each trial, save a sample:

```bash
docker exec -it aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh && . /test/install/setup.sh
  ros2 topic echo /demo/odom --once
'
```

For a repeated measurement, record the simulator log:

```bash
docker logs -f aquila-go2 > /tmp/aquila-go2.log
```

The stability criterion is a level pose, `z` close to `0.35 m`, with no
continuous growth of roll/pitch and no displacement that is merely a fall or a
slide.

## 7. Integrate the warehouse scenario

The spike does not contain `nav2_minimal_tb4_sim`; that is why the official
world must not be referenced as an implicit substitute in the launch file.
There are two options.

### Option A — pass an existing SDF

When a valid `warehouse.sdf` exists on the host:

```bash
./scripts/run_quadruped_sim.sh \
  /caminho/absoluto/warehouse.sdf
```

The SDF must include the Physics, SceneBroadcaster and Sensors systems. After
startup, confirm the world name in the log and test:

```bash
docker exec -it aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh && . /test/install/setup.sh
  ros2 topic hz /demo/scan
  ros2 topic hz /demo/camera/image_raw
'
```

### Option B — install the official package

With the network available, in the environment that builds the image:

```bash
sudo apt install -y ros-jazzy-nav2-minimal-tb4-sim
ros2 pkg prefix nav2_minimal_tb4_sim
```

Locate the installed SDF and pass its absolute path to the script or to the
launch file. Do not copy an archived Gazebo Classic world; use an SDF
compatible with Gazebo Harmonic.

The scenario must first be validated without Nav2: robot visible, sensors
active, `/demo/odom` publishing and no dangling link in the TF tree.

## 8. RViz2 and TF

RViz2 must run outside the Gazebo container, following the project's
separation. Once the official image is available:

```bash
cd ~/toradex/demo/aquila-am69-ros2/docker
docker compose -f compose.host.yml up viz
```

In RViz2, confirm that:

- `RobotModel` shows the body and 12 joints;
- `joint_states` animates the legs;
- camera, lidar and IMU have active topics;
- there are no dangling links;
- `odom -> base_link` has a single owner.

In the spike, Gazebo odometry is a temporary F4 source. Replacing it with
legged odometry and the definitive ownership of `odom -> base_link` belong to
F5.

## 9. Official compose

When the network comes back:

```bash
cd ~/toradex/demo/aquila-am69-ros2/docker
docker compose -f compose.host.yml build base sim viz
docker compose -f compose.host.yml run --rm sim \
  ros2 launch demo_bringup sim.launch.py robot_type:=quadruped
docker compose -f compose.host.yml up viz
```

Since F6 the default is `quadruped`. Use `ROBOT_TYPE=diffdrive` to exercise the
fallback; the same value also selects the corresponding Nav2 launch file.

## 10. Diff-drive regression

After any vendoring or launch-file change:

```bash
cd ~/toradex/demo/aquila-am69-ros2/docker
docker compose -f compose.host.yml --profile learn up --build
```

The gate is a Nav2 goal ending in `SUCCEEDED`. If it fails, keep the diff-drive
regression separate from the Go2 investigation; do not fix both paths in the
same commit.

## 11. Criteria for closing F4

Mark F4 only when all of them are true. State on 18/08/2026:

| Criterion | State |
|---|---|
| Go2 appears with correct meshes and proportions | ✅ |
| stable in `FIXEDSTAND` for at least 60 s | ✅ |
| there is physical leg swing in diagonal pairs | ✅ `[1 0 0 1]` ↔ `[0 1 1 0]` |
| a zeroed command leads to `HOLD` with no residual motion | ✅ >35 s |
| stable in stationary trot for at least 20 s | ✅ |
| walks at `v_cmd = 0,05 m/s` without falling | ✅ 97% tracking |
| walks at `v_cmd = 0,10 m/s` without falling | ✅ 105% |
| walks at `v_cmd = 0,20 m/s` without falling | ✅ 111%, bridge ceiling |
| bounded yaw with a zero yaw command | ✅ ±23,7°, self-correcting |
| does not drift in yaw while stopped in `HOLD` | ✅ fixed by the touchdown latch |
| commanded yaw does not produce explosive rotation | ✅ zero `RECOVER`, tilt ≤ 1,3° |
| **commanded yaw is trackable** | ⚠️ saturates at ~0,13 rad/s, see below |
| warehouse loads with camera and lidar active | ✅ headless, F4 contract |
| RViz2 shows the 12 joints and a consistent TF tree | 🟡 visual confirmation pending; TF closes |
| diff-drive keeps the Nav2 goal at `SUCCEEDED` | ✅ 24/08/2026 |

**The robot's real envelope, measured on 18/08: 0,20 m/s linear and 0,13 rad/s
angular.** Yaw saturates at ~0,13 rad/s because the QP moment tops out at
5,3 N·m — above that the command has no effect. `nav2_params.yaml` needs
`max_vel_theta ≈ 0,12`, not the 0,25 the bridge accepts.

**The TF tree now closes at `odom → base`**, but `odom_tf` derives that edge from
Gazebo's ground truth. That enables Nav2 in simulation; it does not replace the
legged state estimation required for localization on hardware.

The two earlier `linear.x` criteria were written when `_SAFE_STICK_LIMIT = 0.03` was
taken to be the "stable F3 envelope". That envelope was measured with the gait
never activating, so it described the maximum push on a robot with planted feet,
not walking speed. `linear.x = 0.01` gives `v_cmd = 0,004 m/s`, 25× below the
trot's design point: a 4 mm step under an 8 cm foot lift. Rewrite it in terms of
`v_cmd` before using it as acceptance.

Order matters: do not test yaw or the warehouse while straight-line forward
motion still falls, or the two trials will measure the same fall under different
names.

Each adjustment must be a separate commit, with command, duration, initial pose
and final pose recorded in this document or in
`docs/results/ml35-f4-parcial.md`.
