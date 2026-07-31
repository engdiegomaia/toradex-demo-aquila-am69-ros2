# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Read first

The authoritative guides live in `.ai/`. Read them before editing:

- `.ai/CLAUDE.md` — short operational contract in Portuguese. The invariants below come from it.
- `.ai/AGENTS.md` (mirrored as `.ai/codex.md`) — full implementation contract: milestones (M0, ML1-ML4, M0-HW, M1-M5, MX-TIDL), Definition of Done, naming, launch/compose conventions, HMI/perception specs, ADR format, completion-report format.
- `.ai/demo-ros2-aquila-am69.md` — project rationale, hardware/software premises, phases, risks.

The repo is currently a scaffold: `docker/{base,navigation,perception,hmi,simulation}/`, `ros2_ws/src/demo_{description,bringup,navigation,perception,simulation}/`, and `compose/{learn,emul,target}.yaml` exist as empty directories/files. M0 (repository foundation) has not been executed yet. Implement one milestone per session and stop.

## Project shape

Robotics demo on Torizon OS running on the Aquila AM69 (arm64) with Gazebo Harmonic replacing the physical robot. Same source tree and application images support three modes; the only thing that changes between them is `platform:` and which machine each service starts on:

| Mode | Compose file | Simulator | Robotics stack | Purpose |
| --- | --- | --- | --- | --- |
| `learn` | `compose/learn.yaml` | amd64 native, host | amd64, host | Learning / development |
| `emul` | `compose/emul.yaml` | amd64 native, host | arm64 emulated (QEMU), host | Validate build & DDS graph pre-hardware |
| `target` | `compose/target.yaml` | amd64 on host | arm64 on Aquila AM69 | Real demo |

If a change would require different code per mode, the design is wrong.

## Non-negotiable rules

Violating any of these costs days. They are not preferences.

1. **No desktop-OpenGL software runs on the target.** The AM69 GPU exposes only OpenGL ES 3.2 and Vulkan 1.2. Gazebo (OGRE 2), RViz2, and anything else on OGRE 2 stay on the x86 host. Never generate a launch file, compose service, or Dockerfile that places these tools on the module.
2. **RMW is always `rmw_cyclonedds_cpp`.** Bake it into the base image via `ENV`, never set at runtime, never swap for Fast DDS. There is a known inter-container discovery failure with the default RMW.
3. **Torizon OS is installed only via Toradex Easy Installer.** Never document, script, or suggest a remote OTA upgrade path to reach 7.4.0 on this module. A V1.0 module with the old bootloader loads the V1.1 device tree and stops booting. Recovery = Tezi reflash + Torizon Cloud reprovision.
4. **`/opt` is not modifiable by TorizonCore Builder.** Any proposal that depends on writing to `/opt` on the OS is wrong — it goes inside a container or into a Yocto build.
5. **arm64 emulation does not measure performance.** Never draw conclusions about CPU, latency, thermals, or FPS from QEMU. Those numbers only count on the real hardware.
6. **Perception speaks through the contract, never directly.** See the topic contract below.
7. **Never claim hardware validation that was not performed on the real Aquila AM69.** Templates for results files must be marked `PENDING EXECUTION` until real evidence is captured under `docs/results/`.

## Topic contract (project invariant)

Exists so that the NPU/TIDL work can arrive later without refactoring.

| Topic | Type | Producer | Consumers |
| --- | --- | --- | --- |
| `/demo/camera/image_raw` | `sensor_msgs/Image` | Gazebo, USB camera, or rosbag | `demo_perception` |
| `/demo/perception/detections` | `vision_msgs/Detection2DArray` | `demo_perception` | Nav2 costmap layer, HMI |
| `/demo/cmd_vel` | `geometry_msgs/Twist` | Nav2 | Gazebo or real driver |

`demo_perception` is a deterministic synthetic-detection stub today. It must never know the origin of the image, and no consumer must know whether detections came from the stub or from real inference. Swapping the stub for TIDL must be a container swap, not an interface change. Detections feed a Nav2 costmap layer, not just the HMI screen — keep that wiring even while it is a stub.

Namespace/topic conventions (from `.ai/AGENTS.md` §5.2): `/demo/cmd_vel`, `/demo/odom`, `/demo/scan`, `/demo/camera/image_raw`, `/demo/perception/detections`, `/demo/navigation/status`, `/demo/system/heartbeat`.

## Where things run

| Component | Runs on | Arch |
| --- | --- | --- |
| Gazebo Harmonic + `ros_gz_bridge` | x86 workstation | amd64 |
| RViz2 | x86 workstation only | amd64 |
| Nav2, bringup, perception stub, `rosbridge_server` | Aquila or arm64 emulation | arm64 |
| Chromium kiosk HMI | Aquila | arm64 with GPU acceleration |
| Future TIDL inference | separate Aquila container | arm64, out of initial scope |

When proposing a change, always state which of the two machines the code runs on. Half the possible errors in this project come from putting something on the wrong side.

## Conventions

- ROS package prefix: `demo_`. Python with `ament_python` by default; C++ only where hardware-measured performance justifies it.
- One explicit launch file per mode in `demo_bringup` (`learn.launch.py`, `emul.launch.py`, `target.launch.py`). No single launch file full of conditionals.
- Nav2 parameters in YAML under `demo_navigation`, never embedded in code.
- Each container has a single responsibility. `demo_perception` is separated from day one, stub or not, because it defines the OTA update granularity.
- Multi-arch images. `platform:` is declared explicitly in compose, never inferred.
- Image name convention: `${REGISTRY:-local}/demo-aquila-<service>:${TAG:-dev}`.
- No hard-coded IPs in committed files; use `.env.example` / documented overrides.
- Prefer standard ROS messages; do not redefine equivalents of `Twist`, `Odometry`, `Image`, `LaserScan`, `Detection2DArray`.

## Commands

```bash
# Workspace
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
colcon test && colcon test-result --verbose

# Enable arm64 emulation (once per host)
docker run --privileged --rm tonistiigi/binfmt --install arm64
docker buildx create --use --name multiarch

# Multi-arch images
docker buildx build --platform linux/amd64,linux/arm64 -t <reg>/<img>:<tag> --push docker/<dir>

# Run a mode
docker compose -f compose/learn.yaml up
docker compose -f compose/emul.yaml up
docker compose -f compose/target.yaml up

# ROS 2 diagnostics
ros2 topic list && ros2 topic hz <topic> && ros2 node list
ros2 run tf2_tools view_frames

# Module diagnostics (on the Aquila)
cat /etc/os-release && ostree admin status && tdx-info
```

`.ai/AGENTS.md` §6 defines the stable Makefile UX (`make help`, `make validate-compose`, `make build-multiarch`, `make up-learn`, `make verify-dds`, etc.) — implement it under M0.

## Current phase

L1 — ROS 2 fundamentals on the x86 host. Phases L1–L3 are done with ROS 2 **installed natively on the host, without containers**, on purpose: containerizing over an unfamiliar system makes it impossible to separate ROS failures from Docker failures. Containerization enters at L4. Order: L1 → L2 (URDF/TF2/RViz2) → L3 (Gazebo + Nav2) → L4 (containers + arm64 emulation) → hardware phases 0–5.

When finishing a phase, update `.ai/CLAUDE.md` "Onde estamos" and `.ai/changelog.md`.

## When proposing solutions

- Always say which of the two machines the code runs on — host or module.
- If the proposal involves GPU, device access, performance, or thermals, explicitly flag that it can only be validated on real hardware.
- Distinguish verified facts from hypotheses. Do not invent device-tree bindings, overlay names, kernel `CONFIG_` options, or BSP versions.
- Prefer the smallest change that resolves the issue. Do not refactor existing structure without being asked.
- End substantial tasks with the completion report format from `.ai/AGENTS.md` §15.
