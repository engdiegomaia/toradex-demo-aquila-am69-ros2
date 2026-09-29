# Architecture Decision Records

Each record captures one significant decision: the context that forced it,
what was decided, and what it costs. Records are immutable once accepted; a
change of direction is a new record that supersedes the old one.

| ADR | Decision | Status |
| --- | --- | --- |
| [0001](0001-cyclonedds-rmw.md) | CycloneDDS is the only RMW | Accepted |
| [0002](0002-simulator-stays-on-host.md) | Gazebo and RViz2 never run on the module | Accepted |
| [0003](0003-compose-split-by-machine.md) | One Compose file per machine, modes via profiles | Accepted |
| [0004](0004-quadruped-control-stack.md) | `quadruped_ros2_control` + Unitree Go2 model | Accepted |
| [0005](0005-perception-topic-contract.md) | Perception talks only through a fixed topic contract | Accepted |
| [0006](0006-web-cockpit.md) | Web cockpit instead of X11 window embedding | Accepted |
| [0007](0007-std-srvs-simulation-facade.md) | The browser never calls Gazebo interfaces | Accepted |
| [0008](0008-non-destructive-sim-reset.md) | Simulation reset teleports the robot | Accepted |
| [0009](0009-native-arm64-builds.md) | arm64 images are built natively on the module | Accepted |
| [0010](0010-compressed-camera-transport.md) | Compressed camera transport across the link | Accepted |
| [0011](0011-rectangular-footprint.md) | Rectangular footprint instead of `robot_radius` | Accepted |
| [0012](0012-fiducial-exit-marker.md) | AprilTag fiducial as primary exit detector | Accepted |
| [0013](0013-explorer-watchdog-over-mppi-tuning.md) | Explorer watchdog instead of further MPPI tuning | Accepted |
| [0014](0014-route-server-rerouting-disabled.md) | `route_server` rerouting disabled | Accepted |

Supporting measurements are cited by their path in the engineering log
(tag `archive/ml35-engineering-log`); read them with
`git show archive/ml35-engineering-log:<path>`.
