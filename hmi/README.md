# Cockpit web — bundle

Operator interface for the demo. A single bundle serves two destinations:
the x86 workstation cockpit today, and (eventually) the Chromium kiosk on
the Aquila AM69 module.

Background and decisions: [ADR 0006](../docs/decisions/0006-web-cockpit.md)
(why a web cockpit instead of X11 window embedding) and
[ADR 0007](../docs/decisions/0007-std-srvs-simulation-facade.md) (why the
browser never calls Gazebo interfaces directly). Layout reference:
[`docs/images/cockpit-layout.png`](../docs/images/cockpit-layout.png).
Screenshot of a connected session:
[`docs/images/cockpit-connected.png`](../docs/images/cockpit-connected.png).

## Architecture

Two containers, defined in `docker/compose.host.yml`:

| Service | Role | Process | Port (env override) |
|---|---|---|---|
| `cockpit` | ROS-side bridge | `ros2 launch demo_bringup cockpit.launch.py`, which starts `rosbridge_websocket` + `rosapi_node` + `web_video_server` | 9090 WebSocket (`COCKPIT_ROSBRIDGE_PORT`), 8080 MJPEG (`COCKPIT_VIDEO_PORT`) |
| `hmi` | static bundle server | nginx serving this directory as-is, with `/config.json` generated from environment variables and a `/healthz` endpoint | 8081 (`COCKPIT_HMI_PORT`) |

Other env vars read by the `hmi` container: `COCKPIT_BACKEND_HOST` (host the
served `/config.json` points the browser at) and `COCKPIT_BUILD` (build label
shown in the control bar).

Server-side ROS façades the cockpit talks to: `sim_control_relay` and
`scene_view_controller` (both in `demo_simulation`), and `nav_control_relay`
(in `demo_navigation`).

## No build step

There is no `npm install`, bundler, transpiler or `node_modules`. This is
plain HTML/CSS/ES-module source served as-is by `nginx:alpine`. `package.json`
exists for two reasons, neither of which is a dependency manifest:
declaring `"type": "module"` so Node treats the `.js` files as ESM when
running the tests, and giving `npm test` a meaning.

`roslibjs` is **not** vendored. The rosbridge v2 protocol is plain JSON, and
the minimal client with reconnection logic lives in
`js/ros/rosbridge-client.js`.

## How to run

Through Compose (the normal path, from the repository root):

```bash
docker compose -f docker/compose.host.yml up -d cockpit hmi
```

Then open `http://localhost:8081`.

Directly from this directory, without a container, during UI development —
the defaults (`ws://localhost:9090`, `http://localhost:8080`) match exactly
what the `cockpit` service publishes:

```bash
python3 -m http.server 8081 --directory hmi
```

Override host and ports without editing a file:

```
http://localhost:8081/?host=aquila.local&rosbridgePort=9090&videoPort=8080
```

## Testing

```bash
cd hmi && node --test "test/**/*.test.js"    # equivalent to: npm test
```

Runs on Node's built-in test runner, no dependencies to install.

## Structure

```text
hmi/
├── index.html                    the five regions, semantic HTML
├── css/
│   ├── tokens.css                palette, type scale, motion
│   ├── layout.css                the five-region grid
│   └── panels.css                panel chrome, freshness dots, control bar
├── js/
│   ├── config.js                 configuration layers and URL building
│   ├── main.js                   wiring only: builds the client, mounts panels
│   ├── ros/
│   │   ├── rosbridge-client.js   protocol v2, reconnection, replay
│   │   ├── freshness.js          never / live / stale tracking
│   │   ├── png-decompress.js     shared canvas decoder for compressed topics
│   │   └── tf-tree.js            cached TF tree from /tf and /tf_static
│   └── panels/
│       ├── nav-panel.js          2D floor plan, click-to-goal, exploration
│       ├── map-view.js           pure world<->screen transform
│       ├── exploration.js        pure exploration-status decision logic
│       ├── stream-panel.js       MJPEG panel for robot camera and scene views
│       ├── detection-overlay.js  draws detection boxes over a stream
│       ├── view-controls.js      scene-camera orbit/move/zoom/follow
│       ├── sim-controls.js       play/pause/reset
│       ├── log-panel.js          movement log + telemetry strip
│       ├── control-bar.js        link badge, build label, manual-control arrows
│       └── palette.js            canvas colours read from CSS
└── test/                         node --test, one file per module above
```

## Panels and the ROS interfaces they use

| Panel | Role | ROS interfaces |
|---|---|---|
| `nav-panel.js` | Floor plan and the only command surface: click-to-goal, exploration start/cancel, nav reset | action `/navigate_to_pose`; services `/demo/nav/reset`, `/demo/exploration/start`, `/demo/exploration/cancel`; topics `/global_costmap/costmap`, `/global_costmap/published_footprint`, `/plan`, `/tf`, `/tf_static`, `/demo/scan`, `/demo/odom`, `/demo/exploration/status`, `/demo/maze/escaped` |
| `stream-panel.js` | MJPEG panel for the robot camera and the two scene (iso/top) views | `web_video_server` HTTP streams; `camera_info` topics for liveness |
| `detection-overlay.js` | Draws perception boxes over a stream | `/demo/perception/detections` |
| `view-controls.js` | Scene-camera orbit, move, zoom, follow | publishes `/demo/cockpit/scene/cmd_view`; services `/demo/cockpit/scene/reset_view`, `/demo/cockpit/scene/follow`; subscribes `/demo/cockpit/scene/following` |
| `sim-controls.js` | Simulation play/pause/reset | services `/demo/sim/{play,pause,reset}` (`std_srvs/Trigger`); subscribes `/clock` for state |
| `log-panel.js` | Movement log and telemetry strip | `/demo/target/ops_log`, `/demo/target/status`, `/demo/cmd_vel`, `/demo/cmd_vel_si`, `/rosout` (filtered, deliberately secondary) |
| `control-bar.js` | Link state, build id, manual-control arrows | none active — see invariants below |

`map-view.js`, `exploration.js` and `palette.js` are pure helpers with no
direct ROS interface, kept separate from their panels so the logic can be
unit tested without a DOM or a socket.

## Configuration layering

Values are resolved in `js/config.js` from three layers, most specific
first:

1. **Query string** — `?host=&rosbridgePort=&videoPort=&build=`
2. **`/config.json`** — served by nginx, rendered from the `hmi` container's
   `COCKPIT_*` environment variables
3. **`DEFAULTS`** — `host: ''` (falls back to the page's own hostname, then
   `localhost`), `rosbridgePort: 9090`, `videoPort: 8080`, `build: 'dev'`

`TOPICS` is a frozen object holding every topic name the bundle reads, kept
in one place so the topic contract (see the repository root `CLAUDE.md`) is
visible and a rename is a single edit.

## Invariants

- **The browser never calls `ros_gz_interfaces`.** Simulation control goes
  through the `std_srvs/Trigger` façades `/demo/sim/{play,pause,reset}`,
  served by `sim_control_relay`. Calling `ros_gz_interfaces/srv/ControlWorld`
  directly from rosbridge fails at runtime because the `cockpit` container
  does not ship that package — and on a real robot there is no Gazebo to
  call at all. See [ADR 0007](../docs/decisions/0007-std-srvs-simulation-facade.md).
- **Manual-control arrows are intentionally inert.** `control-bar.js` renders
  them but wires no publisher: Nav2 is the only writer of `/demo/cmd_vel`
  today, and publishing from the browser alongside it with no arbiter is the
  debt a future `twist_mux`-based manual control phase closes.

## Two transport paths, and why

- **Pixels** travel over HTTP (`web_video_server`, MJPEG inside an `<img>`).
- **Everything else** travels over the rosbridge WebSocket.

A 640x480 image as base64 JSON over the WebSocket is what makes a
data-driven cockpit feel slow. In exchange, an `<img>` element is a poor
liveness detector: no browser guarantees an event per frame, and a stalled
stream leaves the last frame painted with no signal that it stopped. That is
why camera freshness is derived from the matching `camera_info` topic —
a few hundred bytes at the same rate as the image.
