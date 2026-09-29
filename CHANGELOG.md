# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). Each milestone is
preserved in detail in the [engineering log](docs/engineering-log.md).

## [Unreleased]

### Changed

- Repository prepared for publication: the documentation was rewritten in
  English, and the engineering log moved to the tag `archive/ml35-engineering-log`.
- Evaluation, diagnostic and maze-analysis tools moved from `scripts/` to `tools/`.
- `scripts/run_quadruped_sim.sh` uses the Compose `sim` image by default
  (overridable with `SIM_IMAGE`).
- The host `tools` container writes evaluation output to `artifacts/`.

### Removed

- Legacy X11 window-embedding cockpit (`scripts/cockpit.py`,
  `scripts/cockpit_teleop.py`, `scripts/run_cockpit.sh`), superseded by the web
  cockpit.
- One-off probes `scripts/costmap_probe.py` and `scripts/selfhit.py`.
- The superseded maze world `quadruped_maze.sdf`, replaced by `quadruped_maze11.sdf`.

### Added

- `LICENSE` (Apache-2.0), `NOTICE`, `CONTRIBUTING.md`, architecture decision records.

## ML3.5: quadruped, containerization and HIL (August 2026)

### Added

- Unitree Go2 quadruped on `quadruped_ros2_control`, with Nav2 (MPPI) and a
  rectangular footprint.
- Per-role containers, one Compose file per machine, and native arm64 builds
  on the module through `scripts/module.sh`.
- HIL over Ethernet with Nav2 and perception on the Aquila AM69.
- SLAM-based frontier exploration of the maze, with breadcrumb backtracking,
  a movement watchdog and AprilTag exit detection.
- Web cockpit: map, camera and scene panels, click-to-goal, simulation control,
  module telemetry.
- `ROBOT_TYPE=quadruped|diffdrive` selecting the plant and navigation stack
  together.

### Status

- Supervised exploration demo. The autonomous escape acceptance run is pending
  (see [docs/validation.md](docs/validation.md)).

## ML1–ML3.1: foundations

- ROS 2 Jazzy workspace, tutorials, differential-drive robot description.
- Gazebo Harmonic simulation, Nav2 with a static map and AMCL, perception stub
  and topic contract.
