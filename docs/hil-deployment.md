# Hardware-in-the-Loop Deployment on the Aquila AM69

In `hil` mode the simulated robot stays on the x86 host, and navigation plus
perception run in arm64 containers on a Toradex Aquila AM69. This is the
reference configuration of the demo.

Complete [getting-started.md](getting-started.md) on the host first.

## Module requirements

| Item | Requirement |
| --- | --- |
| Module | Toradex Aquila AM69, hardware revision V1.0A |
| OS | Torizon OS **7.7.0**, installed with **Toradex Easy Installer** |
| Access | SSH as `torizon` with key-based authentication. `module.sh` uses `BatchMode=yes`, so password prompts are not supported. |
| Storage | About 6 GB on the data partition for the four images (they share base layers) |
| Link | Wired Ethernet to the host on the same subnet. Multicast is not required. |

> **Warning: install Torizon OS with Toradex Easy Installer only.** Do not use
> an OTA update to move an older image to this baseline. A V1.0 module with the
> old bootloader loads the V1.1 device tree and no longer boots. Recovery then
> needs an Easy Installer reflash and a new Torizon Cloud provisioning.

Nothing else is installed on the module. Docker ships with Torizon OS, and ROS 2
arrives inside the container images.

## 1. Configure the link

In `docker/.env` on the host, set:

| Variable | Meaning |
| --- | --- |
| `MODULE_HOST` | Module hostname or address used for SSH (default `aquila-am69.local`) |
| `MODULE_IP` | Module address on the host link. Resolved from `MODULE_HOST` if unset. |
| `HOST_IP` | Host address as seen by the module. Derived from the route to `MODULE_IP` if unset. |
| `ROS_DOMAIN_ID` | Must be identical on both machines (default `69`). A mismatch fails silently. |

mDNS resolves the module under its full hostname, which includes the serial
number (for example `aquila-am69-<serial>.local`), not under `aquila-am69.local`.
Set `MODULE_HOST` accordingly, or use the address.

Check SSH access:

```bash
ssh torizon@<module> true
scripts/module.sh inventory      # read-only: OS, CPU, memory, disk, links, images
```

## 2. Sync and build on the module

```bash
scripts/module.sh sync     # copy sources and Dockerfiles, render both DDS configs
scripts/module.sh build    # build base, nav, perception, tools natively on the module
```

`sync` renders the CycloneDDS configuration for both sides from the templates
in `docker/cyclonedds/`. On each machine it pins the interface that routes to
the peer and adds the peer's unicast address. The rendered files contain real
addresses and are not committed.

`build` runs on the module itself. A native build takes minutes, while QEMU on
the host takes hours ([ADR 0009](decisions/0009-native-arm64-builds.md)). After
building, it checks every image for desktop rendering libraries and fails if
it finds one.

Run `sync` again whenever you change the link (Wi-Fi or Ethernet, a new
address), `docker/.env` or the sources under `ros2_ws/src`. Run `build` again
after changing a Dockerfile, a package dependency or the base image.
Parameter files and Python sources of `demo_navigation` and `demo_perception`
are bind-mounted on the module. For those, `sync` plus `up` is enough.

## 3. Start the host side

Start the simulation first, so that the robot stands at its spawn pose before
SLAM receives its first scan:

```bash
docker compose -f docker/compose.host.yml up -d sim cockpit hmi   # add viz for RViz2
```

## 4. Start the module side

```bash
scripts/module.sh up
scripts/module.sh verify
```

`up` recreates the module containers so that they pick up the freshly rendered
DDS configuration. It refuses to start in two cases:

| Guard | Why | Override |
| --- | --- | --- |
| A host process already publishes `/demo/cmd_vel` | Two velocity writers make the robot jerk without any error message | `--force` |
| The robot is far from its spawn pose | SLAM would anchor its map to a stale pose | `--force-spawn` (prefer `/demo/sim/reset` first) |

`verify` checks the link in four stages:

1. UDP reachability of the RTPS discovery port.
2. The module sees the host's topics, including `/clock`.
3. A heartbeat published on the module is received on the host.
4. Nav2 is actually active (`bt_navigator` lifecycle state), not merely running.

Open the cockpit at <http://localhost:8081>. From another machine, use
`http://<host>:8081`.

## Module commands

| Command | Action |
| --- | --- |
| `scripts/module.sh inventory` | Read-only system report of the module |
| `scripts/module.sh render-local` | Render the host DDS configuration for `learn` mode (no module) |
| `scripts/module.sh sync` | Copy sources and Dockerfiles, render and push the DDS configurations, write the module `.env` |
| `scripts/module.sh build` | Build the module images natively, then run the rendering-library check |
| `scripts/module.sh up [--force] [--force-spawn]` | Recreate and start `nav` and `perception` |
| `scripts/module.sh verify` | Four-stage DDS and Nav2 check |
| `scripts/module.sh status` | Container status and the configured DDS peers |
| `scripts/module.sh shell` | Interactive shell in the module `tools` container, with ROS sourced |
| `scripts/module.sh down` | Stop all module containers |

`MODULE_HOST`, `MODULE_USER` (default `torizon`) and `REMOTE_DIR` (default
`/home/<user>/demo`) can be overridden. Precedence is: environment, then
`docker/.env`, then script defaults.

## Restarting a run

Reset the simulation **before** recreating navigation, never after:

```bash
source scripts/env.sh                      # or use a tools container
ros2 service call /demo/sim/reset std_srvs/srv/Trigger '{}'
ros2 topic echo /demo/odom --once          # confirm the robot is near (0, 0)
scripts/module.sh up
scripts/module.sh verify
```

When `nav` is recreated while the robot is away from its spawn pose, SLAM
anchors a map that no longer matches the world. Exploration then fails later
with planner errors. A reset afterwards does not repair that map.

## Stopping

```bash
scripts/module.sh down
docker compose -f docker/compose.host.yml down
```

## Performance notes

- Every CPU, latency and rate figure for this demo was measured on the module.
  None comes from emulation. See [validation.md](validation.md).
- Nav2 alone can take 5–7 of the 8 Cortex-A72 cores. Perception adds about one
  core. When the module is saturated, sensor data goes stale, the collision
  monitor slows the robot down, and nothing reports an error. Watch the
  telemetry strip in the cockpit.
- Thermals have not been characterized under sustained load.
