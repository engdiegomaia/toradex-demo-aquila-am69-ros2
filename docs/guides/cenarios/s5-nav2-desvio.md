# S5 — Nav2 avoiding obstacles

World: `quadruped_objects.sdf` · x86 workstation · ~10 min

## Purpose

This is the first scenario in which the robot decides where to go. In scenarios
S0–S4, `gait_trial.sh` or `demo_routine` issued commands by publishing velocity
directly. Here the command is a **goal**, Nav2 plans, and avoidance is a
consequence of the costmap—not a hand-written route.

This exercises the full chain end to end: lidar cloud → costmap → planner → MPPI
→ `/demo/cmd_vel` → `twist_to_inputs` → gait controller → Gazebo → odometry →
TF → costmap. A closed loop. Almost every defect in this stack appears here.

## Where each component runs

| component | machine | reason |
| --- | --- | --- |
| Gazebo + robot + bridge | `aquila-go2` container, x86 host | OGRE 2, rule 1 in `CLAUDE.md` |
| Nav2 + `odom_tf` + patrol | x86 host in learn; Aquila AM69 in HIL | same multi-arch image and same launch |

HIL was run on the Aquila on 21/08/2026 with composed Nav2; the evidence is in
`docs/results/ml35-hil-aquila.md`. Gazebo remains on the host in both modes.

## Geometry and goal rationale

World obstacles (from the SDF itself):

| object | position (x, y) | planar half-extent |
| --- | --- | --- |
| red box | 1,5 · 0,0 | 0,21 (half-diagonal) |
| green cylinder | 3,0 · +0,45 | 0,18 |
| blue box | 3,0 · −0,55 | 0,28 (half-diagonal) |
| yellow cylinder | 4,5 · 0,0 | 0,12 |

Go2 trunk circumscribed radius: **0,383 m**.

The default `patrol_commander` route is a triangle of three goals chosen so
**each goal is reachable** and **the straight line along each leg is blocked**:

Clearance is calculated using **point-to-segment distance**, not vertical
distance at a chosen `x`; the latter overestimates clearance and was the error
in the first version of this table.

| leg | straight-line clearance | consequence |
| --- | --- | --- |
| (0,0) → (4, 1,5) | **−0,068 m** from the red box | line blocked |
| (4, 1,5) → (4, −1,5) | **−0,003 m** from the yellow cylinder | line blocked |
| (4, −1,5) → (0,0) | **−0,127 m** from the blue box | line blocked |

All three lines are blocked, so avoidance is mandatory. All three **goals** also
have ample clearance—+0,887, +0,713, and +0,905 m—because a tight goal makes
Nav2 fail due to an impossible arrival, which can be mistaken for an avoidance
failure. If the robot walks in a straight line, either the costmap is empty or
it passed through the obstacle; both are failures.

### The seemingly obvious 3 m square does not work

The (3,0) goal falls in the gap between the green cylinder and blue box. That
gap has 0,45 − 0,18 = 0,27 on one side and −0,55 + 0,28 = −0,27 on the other:
**0,54 m of free width**, versus the **0,77 m** the robot needs. The goal is
impossible.

Nav2 **accepts** this goal. It rejects it only after exhausting recovery
behaviors, and the log shows a navigation abort—which reads as "avoidance does
not work" but is actually a goal that never had a solution. Check the arithmetic
before blaming the planner.

## Run

Terminal 1—simulator:

```bash
./scripts/run_quadruped_sim.sh quadruped_objects.sdf
```

Wait for `state=fixed stand` in the log. **Do not continue before that.**

Terminal 2—Nav2 and TF:

```bash
cd ros2_ws && source install/setup.bash
ros2 launch demo_bringup nav_quadruped.launch.py
```

The readiness gate is the `Managed nodes are active` line from
`lifecycle_manager_navigation`. Wait for it.

Terminal 3—patrol:

```bash
source ros2_ws/install/setup.bash
ros2 run demo_bringup patrol_commander --ros-args -p use_sim_time:=true
```

## Pitfalls, all measured

### `ros2 action list` hangs forever

Do not use `ros2 action list` as a scripted readiness gate. With an incomplete
graph, it **blocks indefinitely**—it does not return an empty result or time out.
One runner remained stuck there for 10 min. Use the `lifecycle_manager` log.

### Nothing starts and there is no error

Symptom: the launch log ends at `odom_tf` and nothing else;
`navigate_to_pose` never appears.

Cause: `use_composition` is enabled while including only `navigation_launch.py`.
That file uses `LoadComposableNodes` to load servers into `/nav2_container`, but
it **does not create** that container—`bringup_launch.py` creates it, and that is
the file not being included. The nodes silently target a nonexistent container.

For this reason, `nav_quadruped.launch.py` fixes `use_composition: 'False'`.

### The robot moves in spasms

`demo_routine` is running at the same time. Both publish `/demo/cmd_vel`—Nav2
through `collision_monitor`, the routine directly. `twist_to_inputs` follows the
latest message received and alternates between the two at 20 Hz. No log mentions
this.

```bash
ros2 topic info /demo/cmd_vel --verbose | grep -c "Node name"   # must be 1
```

A `patrol_commander` **from a previous run** produces the same symptom and is
easy to leave behind: it does not exit when the simulator stops; it simply waits
for the action to return. This happened four times in a measurement batch, and
the orphan processes kept publishing on domain 69. Before starting, check:

```bash
pgrep -af "patrol_commander|demo_routine"   # must be empty
```

### Two TF publishers

`odom_tf` publishes `map -> odom` as identity. AMCL and SLAM also publish this
edge. Running both produces no error: the consumer alternates between two
beliefs about the robot's location. When starting AMCL or SLAM, pass
`publish_map_identity:=false`.

### `Failed to make progress` every 10 s

The progress checker uses TurtleBot 4 values: 0,5 m in 10 s. The Go2 turns at
0,12 rad/s, so reorienting by 90° takes 13 s with almost no forward movement—an
abort is guaranteed before the robot starts walking. `nav2_params_go2.yaml`
uses 0,20 m in 40 s.

Symptom in the first measurement: **22 aborts in 5 min, zero falls.** When the
checker fails and the robot does not fall, suspect the checker.

### The robot moves at 40% of the request with no warning

`/demo/cmd_vel` **is not in SI units.** It is a `Twist` carrying normalized
joystick position, and the controller multiplies `linear.x` by 0,4 and
`angular.z` by 0,5 upon receipt (`StateTrotting.cpp:192`,
`twist_to_inputs.py:283`).

Nav2 cannot publish there directly because MPPI **integrates** `vx` as m/s to
predict position. This is why `cmd_vel_si_to_stick` exists: Nav2 publishes SI
units to `/demo/cmd_vel_si`, and the node converts them. If you reconnect Nav2
to `/demo/cmd_vel`, the robot moves at 40% of the request with no log error.

```bash
ros2 topic info /demo/cmd_vel_si --verbose | grep -c "Node name"   # 1: Nav2
ros2 topic info /demo/cmd_vel    --verbose | grep -c "Node name"   # 1: the converter
```

### The robot reaches the goal but the goal fails

Final orientation. If the goal `yaw` is not the **arrival** heading, Nav2
requests an in-place turn on arrival—110 to 139° on the default route, or 16 to
20 s at the yaw-rate limit. An in-place turn does not remain in place: in
measurements, the robot came within 3,8 cm of the goal and drifted 0,78 m in y
while turning, leaving the position tolerance.

For this reason, `patrol_commander` uses the arrival heading, and
`yaw_goal_tolerance` is 0,5 rad.

## Measured results (20/08/2026)

Route (4 · 1,5) → (4 · −1,5) → (0 · 0), repeated, over 300 s of simulation
time. Details and evidence in
[`../../results/ml35-nav2-quadrupede.md`](../../results/ml35-nav2-quadrupede.md).

| run | change | path | displacement | maximum SI `cmd_vx` | aborts | falls |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | TB4 parameters | 4,16 m | 0,73 m (**backward**) | 0,016 m/s | 22 | 0 |
| 2 | 1,44 m horizon, 10 Hz loop | 5,21 m | 0,74 m | 0,023 m/s | 3 | 0 |
| 3 | unit boundary | **8,36 m** | **3,51 m** | **0,119 m/s** | 2 | 0 |
| 4 | goals with arrival heading, 420 s | 10,87 m | 0,60 m | 0,119 m/s | 4 | 0 |

In run 3, the robot came within 3,8 cm of the first goal, passing all four
obstacles with positive clearance—a minimum of 0,014 m at the green cylinder.

In run 4, **the first goal completed** and the cycle continued, with continuous
movement for 420 s, zero collisions (minimum clearance 0,159 m), and zero falls.

**Be careful when comparing 3 with 4.** MPPI is a stochastic sampler
(`regenerate_noises: true`), and there is **n = 1 per condition**: the worse
displacement cannot be attributed to the change. Attribution requires repeated
runs for each condition.

The costmap was measured separately with the robot stationary: cost **0** in the
robot cell, **0** within 0,6 m, 42 lethal cells at obstacles, and zero unknown
cells. Of the 2097 cloud points, 1848 are ground and are discarded by the
0,12 m cutoff.

## What this scenario does NOT validate

- **Localization.** `/demo/odom` is Gazebo ground truth, and `odom_tf` merely
  republishes it as TF. The robot knows its exact location because the simulator
  reported it. Leg-based state estimation is F5.
- **Perception.** `demo_perception` remains a synthetic stub. The lidar performs
  avoidance here through `obstacle_layer`; `perception_layer` is in the costmap
  by contract and is fed by a stub.
- **Performance.** CPU, loop-rate, and FPS figures here are from the x86
  workstation. None transfer to the Aquila AM69, and arm64 emulation does not
  measure performance (rule 5 in `CLAUDE.md`).

## Acceptance

- [ ] `Managed nodes are active` appears in the `lifecycle_manager` log
- [ ] `ros2 topic info /demo/cmd_vel --verbose` shows **one** publisher
- [ ] the TF tree is rooted at `map` (`ros2 run tf2_tools view_frames`)
- [ ] `/demo/scan_cloud` publishes and the local costmap shows occupied cells
- [ ] the recorded trajectory has positive clearance from all four obstacles
- [ ] zero `mode=RECOVER` in the simulator log (the robot did not fall)
