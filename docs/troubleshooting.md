# Troubleshooting

Most failures in a distributed ROS 2 system are silent: a topic exists but
carries no data, or two nodes both think they are in charge. Each entry below
lists the symptom, the cause and the fix.

## Discovery and networking

| Symptom | Cause | Fix |
| --- | --- | --- |
| `ros2 topic list` shows only `/rosout` and `/parameter_events`; spawners hang | Multicast is disabled and the `127.0.0.1` peer is missing, so no same-machine discovery | Keep the `127.0.0.1` peer in both CycloneDDS templates. Render them again. |
| Host and module do not see each other, with no error anywhere | `ROS_DOMAIN_ID` differs between the machines | Use the same value on both sides, then `scripts/module.sh sync`. `verify` stages 2–3 catch this. |
| Discovery broke after switching between Wi-Fi and Ethernet | The rendered DDS configuration pins the old interface | Run `scripts/module.sh sync` and then `scripts/module.sh up` |
| Editing `docker/cyclonedds/*.xml` has no effect | Containers mount the *rendered* copy | Render again (`sync` or `render-local`) and recreate the containers |
| A native tool on the host sees nothing | The process runs with default CycloneDDS settings (multicast, no peers) | `source scripts/env.sh` first, so every process uses the rendered configuration |
| In `learn` mode the robot trots in place with no translation | DDS picked a Docker or VPN interface | Keep `lo` pinned, as in the committed template. Do not rely on automatic interface selection. |
| The link is down after the host resumed from suspend | The host dropped its network interfaces | Check `ip -br link` for carrier before debugging DDS |
| The first service call from a fresh process fails | DDS discovery is still in progress | Let the process spin for a few seconds, or retry |
| mDNS cannot resolve `aquila-am69.local` | The module announces its full hostname, which includes the serial number | Use `aquila-am69-<serial>.local` or the address in `MODULE_HOST` |

## Simulation

| Symptom | Cause | Fix |
| --- | --- | --- |
| Gazebo runs at single-digit FPS | Software rendering: the container is not in the host `render` group | Set `RENDER_GID` in `docker/.env` |
| Gazebo crashes under load with no clear message | Default 64 MB `/dev/shm` | Already handled by `shm_size` and `ipc: host`. Keep those settings. |
| The maze launch aborts and names `MAZE_MODELS` | The external maze mesh is not mounted | Set `MAZE_MODELS` to `ros_maze_worlds/models` |
| Nodes freeze right after a `sim` restart; a probe reports RTF 0 | `/clock` was not yet flowing when the probe started | Wait for `/clock` before starting probes or recorders |
| After a reset the robot drifts or falls | The gait was not held during the teleport | Use `/demo/sim/reset`, which holds and resumes the gait. Do not teleport by hand. |

## Navigation

| Symptom | Cause | Fix |
| --- | --- | --- |
| Exploration fails with planner errors after a restart | `nav` was recreated while the robot was away from the spawn pose, so SLAM anchored an orphaned map | Reset the simulation first, then run `scripts/module.sh up` |
| The robot jerks; commands alternate | Two writers on `/demo/cmd_vel` (for example a gait trial next to Nav2) | Stop the other writer. `module.sh up` refuses while a host writer is running. |
| A goal "succeeds" without the robot moving | The goal lies within `xy_goal_tolerance` of the robot | Keep derived goals at least the goal tolerance away |
| `nav2_container` crashes (SIGSEGV) after a lifecycle reset | `route_server` crashes when it is reconfigured | Never cycle the lifecycle of individual managed nodes. Recreate the `nav` container. See [ADR 0014](decisions/0014-route-server-rerouting-disabled.md). |
| The robot moves much slower in HIL than in `learn` mode | Module CPU saturation makes sensor data stale, and the collision monitor slows the robot | Check module CPU in the cockpit. Run perception only when needed. |

## Containers and builds

| Symptom | Cause | Fix |
| --- | --- | --- |
| `pull access denied for local/demo-aquila-base` | The base image has not been built, or a `docker-container` buildx builder cannot see the local store | `BUILDX_BUILDER=default docker compose -f docker/compose.host.yml build base` |
| `ros2: command not found` in `docker compose exec` | `exec` bypasses the image entry point | Run the command through `/usr/local/bin/entrypoint.sh` |
| The module build fails the rendering-library check | A dependency pulled OGRE, Gazebo or RViz into an arm64 image | Add the key to the rosdep skip list or ignore the package. Never place rendering software on the module. |
| colcon reports `demo_bringup` as "not processed" | `gz_quadruped_hardware` was skipped instead of ignored | Use `--packages-ignore gz_quadruped_hardware` and keep its `COLCON_IGNORE` marker on the module images |
| Source changes do not reach the module | The module runs synced sources, and some are baked into images | `scripts/module.sh sync`, then `up`. Rebuild when you change a Dockerfile or a dependency. |
