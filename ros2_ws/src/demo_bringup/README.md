# demo_bringup

Top-level composition: one explicit launch file per execution mode, no
cross-mode conditionals. ML3 delivers `learn.launch.py`; `emul` and `target`
arrive with ML4 when the same services move into containers.

**Runs on:** `learn.launch.py` is entirely x86 host — it starts Gazebo and RViz,
neither of which may touch the module (`CLAUDE.md` rule 1).

## learn mode

```bash
ros2 launch demo_bringup learn.launch.py
```

Everything native on one workstation, no containers. Phases L1–L3 work this way
on purpose: containerizing over an unfamiliar system makes it impossible to
separate a ROS failure from a Docker failure.

Requires a map — see `demo_navigation/README.md` for the one-time SLAM run.

Arguments: `world`, `map`, `rviz`, `navigation`.

To teleoperate without Nav2 fighting for `/demo/cmd_vel`:

```bash
ros2 launch demo_bringup learn.launch.py navigation:=false
ros2 launch demo_simulation teleop.launch.py   # separate terminal
```

## Startup ordering

Composition order is enforced with timers, not luck:

| t | Component | Depends on |
| --- | --- | --- |
| 0 s | `demo_simulation` — Gazebo starts loading | — |
| 12 s | robot spawn (inside `simulation.launch.py`) | Gazebo's world service is up |
| 15 s | `ros_gz_bridge` (inside `simulation.launch.py`) | the robot exists |
| 20 s | `demo_perception` | `/demo/camera/image_raw` is bridged |
| 25 s | `demo_navigation`, RViz | `/clock`, `/demo/scan`, TF are bridged |

The 12 s and 15 s marks are not padding — see `demo_simulation/README.md`. A
spawner started before Gazebo is serving retries forever without ever
completing its handshake, and the plugins never initialize.

Nav2 started too early comes up before `/clock` publishes, and every lifecycle
node stalls waiting for a transform whose timestamps do not yet exist. The
delays are generous deliberately — this is a demo, not a boot-time benchmark.

RViz uses this package's `rviz/demo_view.rviz` — a copy of `nav2_bringup`'s
`nav2_default_view.rviz` (costmap, plan and goal-tool displays already wired) with
exactly two changes:

| Display | Upstream | Here |
| --- | --- | --- |
| `RobotModel` | `Enabled: false` | `Enabled: true` |
| `TF` | `Enabled: true`, Show Axes + Names | `Enabled: false` |

**Do not point this back at the upstream file.** With Nav2's defaults RViz draws
**no robot body** and **33 labelled axis triads** (four tower standoffs, four
weight blocks, six bumper zones, six OAK-D frames, IMU, wheels, caster) floating
where the robot should be. That looks exactly like a robot whose parts have come
apart — and it is not; it is a cloud of TF markers with the body switched off.

This symptom survives changing the robot model, re-measuring mesh offsets and
welding fixed joints, because none of those touch the RViz config. Tick `TF` in
the sidebar when you need to debug frames, then untick it.

`demo_description`'s config is for viewing the model alone and has no `map` frame,
so it is not a substitute here.

## Acceptance (ML3) — passed 2026-08-10

| Criterion | How to check | Result |
| --- | --- | --- |
| Robot can be teleoperated | `teleop.launch.py`, robot moves in Gazebo | PASS — (0,0) → (0.82, 2.31) via `/demo/cmd_vel` |
| Odom, scan, TF, commands exchange messages | `ros2 topic hz` on each; `ros2 run tf2_tools view_frames` | PASS — odom 27.6 Hz, scan 9.97 Hz, TF chain complete |
| Nav2 reaches active state | `ros2 lifecycle get /bt_navigator` → `active` | PASS — all 7 servers active |
| A goal completes | RViz **2D Goal Pose**, robot arrives | PASS — `SUCCEEDED`, (0,0) → (2.43, 0.20) |

Also verified: `/demo/perception/detection_cloud` at 15.15 Hz with
`Subscription count: 2` — both costmaps consuming detections, so the
perception → costmap seam required by `CLAUDE.md` is live and not just wired.

Reproduce the goal test headlessly:

```bash
ros2 launch demo_bringup learn.launch.py rviz:=false
# wait ~45 s for all lifecycle nodes to reach active, then:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.5}, orientation: {w: 1.0}}}}"
```
