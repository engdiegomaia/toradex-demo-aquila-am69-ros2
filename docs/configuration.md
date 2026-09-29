# Configuration

## Environment file

Copy `docker/.env.example` to `docker/.env`. The `.env` file is ignored by git:
addresses, hostnames and local paths never belong in a commit. Compose reads it
on the host, and `scripts/module.sh` reads it for module deployment.

| Variable | Default | Used by | Description |
| --- | --- | --- | --- |
| `REGISTRY` | `local` | build, both machines | Image name prefix: `${REGISTRY}/demo-aquila-<service>:${TAG}` |
| `TAG` | `dev` | build, both machines | Image tag |
| `ROS_DOMAIN_ID` | `69` | both machines | DDS domain. **Must match on host and module**; a mismatch fails silently. |
| `HOST_IP` | derived | `module.sh` | Host address as seen by the module |
| `MODULE_IP` | resolved from `MODULE_HOST` | `module.sh` | Module address on the host link |
| `MODULE_HOST` | `aquila-am69.local` | `module.sh` | SSH target for the module |
| `MODULE_USER` | `torizon` | `module.sh` | SSH user on the module |
| `REMOTE_DIR` | `/home/<user>/demo` | `module.sh` | Deployment directory on the module |
| `DISPLAY` | `:0` | `sim`, `viz` | X display on the host |
| `RENDER_GID` | `992` | `sim`, `viz` | Group id of the host's `render` group (`getent group render \| cut -d: -f3`). Without it rendering falls back to software. |
| `ROBOT_TYPE` | `quadruped` | `sim`, `nav` | `quadruped` (Go2, maze) or `diffdrive` (TurtleBot 4 fallback, warehouse) |
| `SIM_GUI` | `true` | `sim` | Open the Gazebo GUI |
| `SIM_ARGS` | empty | `sim` | Extra `sim.launch.py` arguments, for example `world:=<absolute path>`. Empty means the robot's official scenario. |
| `MAZE_MODELS` | `docker/models-extra` (empty) | `sim` | Absolute path to `ros_maze_worlds/models`. Required by the maze world. |
| `NAV2_PARAMS` | robot default | `nav` | Absolute path, inside the `nav` image, of an alternative Nav2 parameter file |
| `COCKPIT_HMI_PORT` | `8081` | `hmi` | HTTP port of the cockpit page |
| `COCKPIT_ROSBRIDGE_PORT` | `9090` | `cockpit`, `hmi` | rosbridge WebSocket port |
| `COCKPIT_VIDEO_PORT` | `8080` | `cockpit`, `hmi` | `web_video_server` MJPEG port |
| `COCKPIT_BACKEND_HOST` | empty | `hmi` | Host the browser uses to reach rosbridge and video. Empty means the host that served the page. |
| `COCKPIT_BUILD` | `TAG` | `hmi` | Build label shown in the cockpit |

## ROS middleware

Baked into the base image and mirrored by [`scripts/env.sh`](../scripts/env.sh)
for native tools on the host:

| Variable | Value |
| --- | --- |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` (never overridden at runtime) |
| `ROS_DOMAIN_ID` | `69` |
| `ROS_LOCALHOST_ONLY` | `0` |
| `CYCLONEDDS_URI` | `file:///cfg/cyclonedds.xml` (the rendered configuration, mounted read-only) |

## CycloneDDS

The committed templates [`docker/cyclonedds/host.xml`](../docker/cyclonedds/host.xml)
and [`module.xml`](../docker/cyclonedds/module.xml) disable multicast and declare
`127.0.0.1` as a peer. Same-machine discovery depends on that entry, so do not
remove it.

The containers mount a **rendered** copy:

| Command | Produces | Content |
| --- | --- | --- |
| `scripts/module.sh render-local` | `docker/cyclonedds/host.rendered.xml` | Template as is (`learn` mode) |
| `scripts/module.sh sync` | `host.rendered.xml` on the host, `cyclonedds/module.xml` on the module | Interface that routes to the peer pinned, peer unicast address added |

Rendered files are ignored by git. Editing a template has no effect until you
render again and recreate the containers.

## Navigation parameters

Nav2 parameters live in YAML under
[`demo_navigation/config/`](../ros2_ws/src/demo_navigation/config/), never in
code:

| File | Use |
| --- | --- |
| `nav2_params_go2.yaml` | Default for the quadruped (MPPI, rectangular footprint, SLAM map) |
| `nav2_params_go2_footprint.yaml` | Kept equal to the default for older campaign commands (a test enforces this) |
| `params-align8.yaml` | A/B variant (`PathAlignCritic.cost_weight = 8.0`), used by the evaluation campaigns |
| `nav2_params.yaml` | Differential-drive fallback (static map, AMCL) |
| `slam_params.yaml` | `slam_toolbox` for the live map |

On the module these files are bind-mounted from the synced sources, so a
parameter change needs `scripts/module.sh sync` and `scripts/module.sh up`, not
an image rebuild.

The gait parameters of the Go2 controller are in
[`demo_simulation/config/gait_go2.yaml`](../ros2_ws/src/demo_simulation/config/gait_go2.yaml).
