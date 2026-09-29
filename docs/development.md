# Development Guide

## Native workspace (optional)

Containers are the supported way to run the demo. A native ROS 2 Jazzy
installation on Ubuntu 24.04 is useful for fast iteration and for running
`ros2` CLI tools against the containers.

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-rmw-cyclonedds-cpp python3-colcon-common-extensions
source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
source ../scripts/env.sh     # same domain, RMW and DDS configuration as the containers
```

Always build from `ros2_ws/`. Running colcon from another directory drops
`build/`, `install/` and `log/` in the wrong place.

## Tests

| Suite | Location | Needs | Command |
| --- | --- | --- | --- |
| Repository contracts | `tests/` | Python 3, `pytest`, `PyYAML` | `python3 -m pytest tests -q` |
| Cockpit | `hmi/test/` | Node.js ≥ 18 | `cd hmi && node --test "test/**/*.test.js"` |
| ROS packages | `ros2_ws/src/*/test/` | Sourced Jazzy, built workspace | `cd ros2_ws && colcon test && colcon test-result --verbose` |

The repository contract tests read Compose files, launch files, parameter
files, behavior trees and scripts **statically**. They guard against the
failure modes described in [troubleshooting.md](troubleshooting.md): two
Compose files drifting apart, a rendering library reaching the module, a Nav2
variant diverging from the default, and so on. They need neither ROS nor
Docker. On a fresh clone a few tests skip because they need the external maze
mesh or a rendered DDS configuration.

The package tests include `flake8` and `pep257` linters and the unit tests of
the nodes: frontier selection, the maze explorer, the startup chain, the
velocity adapters, the simulation reset and the exit detector.

## Conventions

### Architecture

- **State the machine.** Every change runs either on the host or on the
  module. Rendering software never goes on the module.
- **One launch file per container role** in `demo_bringup` (`sim`, `nav`,
  `perception`, `viz`, `cockpit`). Do not write one launch file full of
  conditionals. `learn.launch.py` is the native, non-containerized composition.
- **One responsibility per container.** Perception stays separate because it
  is the unit of over-the-air updates.
- **Modes must not need different code.** If a change needs code that depends
  on the mode, the design is wrong.
- **Keep the topic contract.** See [architecture.md](architecture.md#topic-contract).
  Prefer standard messages (`Twist`, `Odometry`, `Image`, `LaserScan`,
  `Detection2DArray`) over custom equivalents.

### Code

- ROS packages use the `demo_` prefix and `ament_python`. Use C++ only where a
  measurement on the hardware justifies it.
- Nav2 and other tuning parameters live in YAML, never in code.
- Topic names are absolute under `/demo/`.
- Images: `${REGISTRY:-local}/demo-aquila-<service>:${TAG:-dev}`, with
  `platform:` declared explicitly in Compose.
- No IP addresses, hostnames or credentials in committed files. Use
  `docker/.env` (see [configuration.md](configuration.md)).

### Vendored packages

`go2_description`, `unitree_guide_controller`, `controller_common`,
`control_input_msgs` and `gz_quadruped_hardware` are third-party code. Keep
upstream names. Record every local change in the package's `PROVENANCE.md` or
`README.md`. Keep the diff against upstream as small as possible.

### Documentation

- A decision with lasting consequences gets an [ADR](decisions/README.md).
- Performance claims name the machine they were measured on. Hardware results
  that were not measured are marked **PENDING EXECUTION**.
- Comments in the code cite older evidence by paths such as
  `docs/results/<report>.md`. Those paths resolve in the
  [engineering log](engineering-log.md) archive tag.

## Building images

| Target | Command |
| --- | --- |
| Host images (amd64) | `BUILDX_BUILDER=default docker compose -f docker/compose.host.yml build base` then `... --profile learn build` |
| Module images (arm64, native) | `scripts/module.sh sync && scripts/module.sh build` |
| Multi-arch images for a registry | `docker buildx build --platform linux/amd64,linux/arm64 -f docker/<service>/Dockerfile -t <registry>/demo-aquila-<service>:<tag> --push .` |

For multi-arch builds on the host, enable arm64 emulation once:

```bash
docker run --privileged --rm tonistiigi/binfmt --install arm64
docker buildx create --use --name multiarch
```

Build contexts are the repository root, not `docker/`.

## Useful diagnostics

```bash
ros2 topic list && ros2 topic hz /demo/odom && ros2 node list
ros2 run tf2_tools view_frames
scripts/module.sh status
ssh torizon@<module> 'cat /etc/os-release; ostree admin status; tdx-info'
```
