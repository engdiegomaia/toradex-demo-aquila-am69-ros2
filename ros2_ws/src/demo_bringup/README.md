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
| 0 s | `demo_simulation` | — |
| 5 s | `demo_perception` | `/demo/camera/image_raw` exists |
| 8 s | `demo_navigation`, RViz | `/clock`, `/demo/scan`, TF exist |

Nav2 started too early comes up before `/clock` publishes, and every lifecycle
node stalls waiting for a transform whose timestamps do not yet exist. The
delays are generous deliberately — this is a demo, not a boot-time benchmark.

RViz uses `nav2_bringup`'s `nav2_default_view.rviz`, which already has costmap,
plan, and goal-tool displays wired. `demo_description`'s config is for viewing
the model alone and has no `map` frame.

## Acceptance (ML3)

| Criterion | How to check |
| --- | --- |
| Robot can be teleoperated | `teleop.launch.py`, robot moves in Gazebo |
| Odom, scan, TF, commands exchange messages | `ros2 topic hz` on each; `ros2 run tf2_tools view_frames` |
| Nav2 reaches active state | `ros2 lifecycle get /bt_navigator` → `active` |
| A goal completes | RViz **2D Goal Pose**, robot arrives |
