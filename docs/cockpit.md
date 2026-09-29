# Web Cockpit

The cockpit is the operator console of the demo. It is a static web page that
reads everything from ROS topics through `rosbridge`: map, costmap, plan, robot
pose, camera streams, detections and module telemetry. It commands the robot
only through Nav2 goals and a few `std_srvs/Trigger` services.

![Cockpit](images/cockpit-connected.png)

## Components

| Service | Runs on | Content |
| --- | --- | --- |
| `cockpit` | host | `rosbridge_websocket` (port 9090), `rosapi`, `web_video_server` (MJPEG, port 8080) |
| `hmi` | host | nginx serving [`hmi/`](../hmi/README.md) on port 8081, plus `/config.json` generated from the environment |

The bundle is plain HTML, CSS and ES modules. It has no build step and no npm
dependencies ([ADR 0006](decisions/0006-web-cockpit.md)), so the same bundle
can later be served to a kiosk browser on the module.

## Starting

```bash
docker compose -f docker/compose.host.yml up -d cockpit hmi
```

Open <http://localhost:8081>, or `http://<host>:8081` from another machine.
Both `learn` and `hil` mode use the same cockpit, because it always runs on the
host.

To point the page at a different backend, use query parameters:
`http://localhost:8081/?host=<backend>&rosbridgePort=9090&videoPort=8080`.

## Layout

The interface labels are currently in Portuguese (for example *Navegação*,
*Câmera*, *Cena*, *iniciar busca*). The table gives the English meaning.

The layout reference is [images/cockpit-layout.png](images/cockpit-layout.png).

| Panel | Shows | Commands |
| --- | --- | --- |
| Control bar | Link state, build label, simulation play/pause/reset | `/demo/sim/{play,pause,reset}` |
| Navigation | Floor plan: SLAM map, global costmap, footprint, plan, lidar, robot pose, exploration state, escape flag | Click to send a goal (`/navigate_to_pose`), exploration start/cancel, navigation reset |
| Robot camera | `/demo/camera/image_raw` with the detection overlay | — |
| Scene view | Isometric and top scene cameras | Orbit, move, zoom, follow the robot |
| Log | Movement log and module telemetry (CPU, temperature, memory) | — |

## Operating rules

- **One velocity writer.** Nav2 is the only writer of `/demo/cmd_vel`. The
  manual-control arrows in the bar are intentionally inert until a
  `twist_mux`-based manual control is added.
- **One goal source at a time.** A goal clicked while exploration runs replaces
  the explorer's goal. Cancel the exploration first.
- **Reset order.** *Reset* in the control bar teleports the robot back to the
  spawn pose and keeps the world and the clock running
  ([ADR 0008](decisions/0008-non-destructive-sim-reset.md)). *Navigation reset*
  cancels the goal and clears the costmaps. To restart SLAM, reset the
  simulation first and then recreate `nav`.
- **No Gazebo interfaces in the browser.** Simulation control goes through the
  `std_srvs` façade in `sim_control_relay`
  ([ADR 0007](decisions/0007-std-srvs-simulation-facade.md)).

## Staleness

Every panel tracks the age of its data and marks itself stale once its
threshold passes. A stale panel after a restart usually means that DDS
discovery has not finished, or that the source container is down. See
[troubleshooting.md](troubleshooting.md).

## Development

```bash
python3 -m http.server 8081 --directory hmi     # serve the bundle without nginx
cd hmi && node --test "test/**/*.test.js"       # unit tests, Node only
```
