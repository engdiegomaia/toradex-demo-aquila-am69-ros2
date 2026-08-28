# Simulation scenarios

Six worlds, each exercising a different part of the stack. One guide per
scenario describes what it measures, how to run it, and what result to accept.

**All run on the x86 workstation.** Gazebo Harmonic uses OGRE 2 and does not run
on the Aquila AM69—rule 1 in `CLAUDE.md`. No figure measured here constitutes
hardware validation.

Results from the 20/08/2026 batch, all at RTF 1,00:

| # | result | `RECOVER` | peak tilt | crabbing | estimator y error |
| --- | --- | --- | --- | --- | --- |
| S0 plane | passes | 0 | 1,08° | 3,20% | −0,145 m |
| S1 6° ramp | passes | 0 | **0,96°** | 2,38% | −0,111 m |
| S2 rough | passes | 0 | 2,81° | **6,31%** | **−0,244 m** |
| S3 corridor | passes | 0 | 1,38° | 0,33% | −0,141 m |
| S4 objects | passes (camera) | 0 | — | — | — |

Crabbing tracks estimator error across all scenarios, not gait difficulty: it
is **greater on rough terrain** and **lower on the ramp** than on the plane. This
confirms that lateral drift is Defect 1, not gait tuning. S2 is the reference
scenario for estimator work.

| # | Scenario | World | Exercises | Guide |
| --- | --- | --- | --- | --- |
| S0 | Empty plane | `quadruped_empty.sdf` | gait, reference for everything | [s0-plano-vazio.md](s0-plano-vazio.md) |
| S1 | 6° ramp | `quadruped_ramp.sdf` | incline balance, tilt limit | [s1-rampa.md](s1-rampa.md) |
| S2 | Rough terrain | `quadruped_rough.sdf` | foot placement, off-plane estimator | [s2-terreno-irregular.md](s2-terreno-irregular.md) |
| S3 | Corridor | `quadruped_corridor.sdf` | `/demo/scan`, costmap path (F5) | [s3-corredor.md](s3-corredor.md) |
| S4 | Objects | `quadruped_objects.sdf` | `/demo/camera/*`, perception contract | [s4-objetos.md](s4-objetos.md) |
| S5 | Nav2 avoidance | `quadruped_objects.sdf` | closed loop: cloud → costmap → MPPI → gait | [s5-nav2-desvio.md](s5-nav2-desvio.md) |
| S6 | Interactive maze | `quadruped_maze11.sdf` | click-to-set goals, TF, lidar and camera in RViz | [s6-labirinto.md](s6-labirinto.md) |

S6 changed from `maze10` to **`maze11`** on 21/08/2026: the same corridor width
(1,20 m) and wall height (0,60 m), with **35,4 m² navigable versus 18,4 m²** in
a single connected component. `quadruped_maze.sdf` (maze10) remains in the tree
as the reference that produced the earlier evidence. Geometry measured by
`scripts/maze_fit.py`, with figures in
[`../../results/ml35-labirinto.md`](../../results/ml35-labirinto.md).

## How to run any scenario

```bash
# on the x86 workstation, terminal 1 -- start the world
./scripts/run_quadruped_sim.sh quadruped_ramp.sdf

# terminal 2 -- verify the topic contract
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=69
python3 scripts/scenario_check.py --seconds 20
```

An argument to `run_quadruped_sim.sh` **without a slash** is resolved in the
`demo_simulation` `share`. With a slash, it is treated as an absolute path.
Passing a nonexistent name starts Gazebo with an empty world and no visible
error—if the robot appears on an infinite plane when you requested a ramp, this
is what happened.

## The checker

`scripts/scenario_check.py` is read-only: it does not publish to
`/demo/cmd_vel`, so it can run alongside any movement routine. It measures each
topic's rate against a floor; that floor is not a design target, but the value
below which a downstream consumer visibly breaks. A topic at 0 Hz whose name
appears in `ros2 topic list` is this simulation's quietest failure mode and the
reason the script exists.

## What is known and expected in ALL scenarios

These are not defects in a particular world; they are the application's current
state.

- **The TF tree connects when Go2 Nav2 is active.** `odom_tf` publishes
  `odom → base` from `/demo/odom` and `map → odom` as identity. This odometry
  is Gazebo ground truth: it serves the HIL demo but does not validate leg-based
  localization. Without the navigation launch, those two edges are expected to
  be absent.
- **Avoidance exists only in Nav2-commanded scenarios.** `gait_trial.sh` and
  `demo_routine` remain open-loop by design. S5/S6 use `/demo/scan_cloud`, all
  16 lidar rings, because the single-ring `/demo/scan` does not reliably see
  isolated objects.
- **`/demo/perception/detections` exists only while `demo_perception` is
  running**, and when present it is a **deterministic synthetic** stub: it does
  not inspect the image. No scenario here validates detection.
- **Crabbing of ~2% of forward distance.** The estimator believes it traveled
  straight while the robot slips sideways. The cause was measured in
  `../../results/ml35-postura-parada.md`; it is Defect 1, and no gait gain fixes
  it.
- **~2,6° peak-to-peak yaw jitter while stationary.**

## Pitfall: "the simulation froze; the robot did not stand up"

Symptom: Gazebo opens, the robot appears lying down or standing rigidly, and
nothing happens. The log repeats:

```
[controller_manager]: No clock received, using time argument instead!
```

**The robot is not the problem.** `/clock` is not reaching the ROS side. Without
it, `controller_manager` does not advance, the FSM never leaves its initial
state, and the robot never stands. Measured on 20/08/2026: Gazebo was running
normally—sim_time 524,9 s, RTF 0,9997, 524.932 iterations—and `ros_gz_bridge`
was alive and subscribed to the correct Gazebo topic. It simply did not relay
the messages.

### How to identify this in 30 seconds

```bash
# 1. Is Gazebo advancing? (inside the container)
docker exec aquila-go2 bash -c '. /opt/ros/jazzy/setup.sh; \
  gz topic -e -t /world/<nome_do_mundo>/stats -n 1'
# -> if real_time_factor ~1.0 and iterations increases, physics is OK

# 2. Does gz /clock publish?
docker exec aquila-go2 bash -c '. /opt/ros/jazzy/setup.sh; \
  gz topic -e -t /clock -n 2'

# 3. Does ROS /clock receive messages?
docker exec aquila-go2 bash -c '. /opt/ros/jazzy/setup.sh; \
  ros2 topic hz /clock'
```

Gazebo advancing + `gz /clock` publishing + `ros2 topic hz /clock` **with no
output** = this failure. Note that `ros2 topic info /clock` still shows correct
publisher and subscriber counts—counts do not prove delivery, which is why
`scenario_check.py` counts messages over a window instead of listing topics.

### Cause and response

The failure appeared in the fourth consecutive container in a batch, with rapid
turnover of containers using `--network=host` and the same `ROS_DOMAIN_ID`.
Running the same scenario **alone, from scratch** starts normally: 0 occurrences
of `No clock`. Therefore, neither the world nor the scenario is at fault; stale
DDS discovery state is crossing between containers.

Response:

1. **Never run two simulations at the same time.** They compete for the
   `aquila-go2` name and DDS domain. This is how the failure was triggered.
2. Stop the container, wait for it to **disappear from `docker ps -a`** (with
   `--rm`, removal is asynchronous and the name remains reserved), then start
   it again.
3. Between scenarios in a batch, allow a few seconds of idle time instead of
   restarting immediately.

## Navigation-path status (measured on 20/08/2026)

The three blockers this section previously listed as open **are closed**. Nav2
starts, activates, and plans for the quadruped. The history is preserved below
because the way each blocker was closed has consequences.

| component | status | evidence |
| --- | --- | --- |
| lidar as a real sensor | **yes** | `L1_lidar`, `gpu_lidar`, 640 × 16, 0,05–10 m, 10 Hz |
| `/demo/scan` (LaserScan) | yes, 10 Hz | **2D: one of 16 rings.** Useless for the costmap |
| `/demo/scan_cloud` (PointCloud2) | **yes** | all 16 rings. 2097 points, 249 above ground at 1,31–4,43 m |
| `/demo/odom` odometry | yes, 50 Hz | **Gazebo ground truth**, not leg-based estimation |
| `odom` → `base` in TF | **yes** | `demo_bringup/odom_tf` |
| `map` → `odom` in TF | **yes** | identity, same node. Not localization |
| complete TF tree | **yes** | 22 edges, 9 static, root `map` |
| base frame | resolved | it is `base`; Nav2 changed, not the URDF |
| Nav2 configured for Go2 | **yes** | `nav2_params_go2.yaml`, 13 deltas from TB4 |
| Nav2 activating and planning | **yes** | `Managed nodes are active`; see [s5-nav2-desvio.md](s5-nav2-desvio.md) |

### How each blocker was closed and the tradeoff

1. **Frames.** `demo_bringup/odom_tf` republishes `/demo/odom` as `odom → base`
   and publishes `map → odom` as identity. This **is not state estimation**;
   it turns simulator ground truth into TF. It exercises perception and
   planning, but is useless for localization validation and will be removed
   when F5 delivers the leg-based estimator.

   Pitfall: two publishers on the same TF edge **produce no error**. The
   consumer receives both and uses the last one received. When starting SLAM or
   AMCL, pass `publish_map_identity:=false`.

2. **Frame name.** The parameter file changed, not the URDF:
   `nav2_params_go2.yaml` uses `base`. `go2_description` is vendored with a
   byte-for-byte guarantee supporting the licensing argument, so the frame name
   is fixed.

3. **Lidar representation.** Neither reorient the sensor nor edit the URDF: the
   bridge now exposes `/scan/points` as `PointCloud2` on `/demo/scan_cloud`, and
   the costmap consumes it. The bridge is the project's available injection
   point.

   The supporting figure: in the `quadruped_objects.sdf` world, with four
   objects at 1,5–4,5 m, `/demo/scan` produces **zero** obstacles—numerically
   identical to the empty world—while `/demo/scan_cloud` produces **249**
   obstacle points. In the corridor, it produces 2422 starting at 1,06 m.
