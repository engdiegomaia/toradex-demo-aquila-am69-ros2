# Codex implementation guide — Demo ROS 2 on Torizon OS / Aquila AM69

Project owner: Diego Maia, FAE Toradex Brasil  
Project type: internal technical demonstration  
Target platform: Aquila AM69 V1.0A + Torizon OS 7.4.0  
Development host: Ubuntu 24.04 x86_64  
Primary stack: ROS 2 Jazzy, Gazebo Harmonic, Nav2, CycloneDDS, Docker Compose

> This file is the implementation contract for Codex CLI. Work incrementally, preserve the architecture below, and never claim hardware validation that was not performed on the real Aquila AM69.
>
> Codex CLI automatically discovers `AGENTS.md`. This repository keeps the requested `codex.md` as the human-readable source and should also contain an identical `AGENTS.md`, or a symbolic link from `AGENTS.md` to `codex.md`.

---

## 1. Mission

Build a reproducible robotics demonstration in which:

1. Gazebo Harmonic runs on an Ubuntu 24.04 x86_64 workstation.
2. Navigation, localization, planning, control, and perception-stub nodes run in containers intended for the Aquila AM69.
3. The x86 workstation and the Aquila communicate through ROS 2 DDS over Ethernet.
4. An embedded HMI runs on the Aquila through GPU-accelerated Chromium in kiosk mode.
5. The same source tree and application images support three execution modes:
   - native learning on x86;
   - arm64 emulation on x86;
   - distributed execution with Gazebo on x86 and the robotics stack on the Aquila.
6. Application containers can later be updated through Torizon Cloud.
7. The architecture includes stable extension seams for future TIDL/NPU perception without implementing NPU inference in the initial scope.

The demonstration must make it visibly clear that navigation is computed on the Aquila, not on the workstation.

---

## 2. How Codex must work

### 2.1 General behavior

For every task:

1. Read this file and inspect the repository before editing.
2. Check `git status` and do not overwrite unrelated user changes.
3. State the selected milestone and its acceptance criteria.
4. Implement only the requested milestone or the next incomplete milestone.
5. Keep changes small, reviewable, and reversible.
6. Run all validations available in the current environment.
7. Record commands, results, limitations, and unresolved hardware checks.
8. Do not silently replace project requirements with generic ROS 2 examples.
9. Do not fabricate performance, thermal, GPU, DDS, Torizon Cloud, or hardware results.
10. Do not commit, push, provision devices, publish images, or deploy OTA updates unless explicitly requested.

When a required dependency or hardware resource is unavailable, implement the source, tests, scripts, and runbook necessary for later execution, then clearly mark the validation as pending.

### 2.2 Definition of done for every change

A change is complete only when all applicable items are true:

- source files are present in the correct package;
- package metadata is valid;
- build succeeds, or the exact blocking dependency is documented;
- unit or static tests pass;
- launch or Compose configuration is syntactically valid;
- documentation includes the command to reproduce the result;
- generated artifacts are not committed unless intentionally required;
- no secret, token, certificate, device credential, or private registry password is added;
- architecture constraints in this document remain satisfied.

### 2.3 Preferred implementation order

Do not attempt the complete project in one run. Use the milestone order in section 9. A Codex session should normally implement one milestone, validate it, summarize the diff, and stop.

---

## 3. Non-negotiable architecture constraints

### 3.1 Runtime placement

| Component | Runtime location | Architecture |
| --- | --- | --- |
| Gazebo Harmonic server/client | workstation | amd64 |
| `ros_gz_bridge` | workstation | amd64 |
| RViz2 for development | workstation only | amd64 |
| Nav2 and robot bringup | Aquila target, or arm64 emulation | arm64 |
| Perception stub | Aquila target, or arm64 emulation | arm64 |
| `rosbridge_server` | Aquila target, or arm64 emulation | arm64 |
| Chromium kiosk HMI | Aquila target | arm64 with GPU acceleration |
| Future TIDL inference | separate Aquila container | arm64; out of initial scope |

Do not move Gazebo or RViz2 to the Aquila as the default architecture.

A separate experiment may test `gz sim -s` with a physics-only world and no rendering sensors. Keep that experiment isolated and do not make the main demo depend on it.

### 3.2 Networking and DDS

- Use Eclipse CycloneDDS.
- Bake `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` into the common ROS image.
- Use host networking for ROS 2 services in all Compose modes where supported.
- Use the same `ROS_DOMAIN_ID` on both machines.
- Default `ROS_LOCALHOST_ONLY=0`.
- Keep a project-owned CycloneDDS XML configuration that can be overridden by environment variable.
- Do not rely only on `ros2 topic list`; acceptance requires actual message exchange.
- The target environment assumes both machines are on the same Layer-2 Ethernet network.
- Avoid hard-coded IP addresses in committed files. Provide `.env.example` or documented overrides.
- Use sensor-data QoS for simulated image and scan streams where appropriate.
- Document multicast/firewall assumptions.

### 3.3 Torizon OS and Aquila safety rule

The target module is Aquila AM69 V1.0A and the required image is Torizon OS 7.4.0 installed from scratch with Toradex Easy Installer.

Never create instructions that update this V1.0 module to 7.4.0 through OTA from an older base image. The runbook must prominently state:

- install Torizon OS 7.4.0 from scratch through Tezi;
- do not use a remote base-OS upgrade from an older image on this module revision;
- recovery from an invalid boot state requires Tezi reflash and Torizon Cloud reprovisioning.

Application-container updates are in scope. Unsafe base-OS update automation is not.

### 3.4 GPU and display

- The target display path is direct DisplayPort.
- Do not assume an HDMI adapter or converter.
- The embedded HMI must use an Aquila-compatible GPU-accelerated Chromium/Weston container for Torizon OS 7.4.0.
- Software rendering is not acceptable as the final Phase 3 result.
- GPU acceleration must be validated on hardware and documented with evidence.
- Development fallback may run the web HMI in a normal desktop browser on x86, but that is not target acceptance.

### 3.5 Future perception contract

Implement the extension seams from the start:

- input: `sensor_msgs/msg/Image`;
- output: `vision_msgs/msg/Detection2DArray`;
- the initial perception node is a deterministic synthetic stub;
- perception lives in a separate package and container;
- consumers depend on the topic contract, not on the stub implementation;
- reserve a Nav2 costmap integration seam;
- reserve an HMI panel for perception output;
- do not add TIDL runtime or TI model assets to the initial implementation.

---

## 4. Repository layout

Create and preserve this structure:

```text
demo-aquila-ros2/
├── AGENTS.md
├── codex.md
├── README.md
├── .env.example
├── .gitignore
├── Makefile
├── docker/
│   ├── base/
│   │   └── Dockerfile
│   ├── navigation/
│   │   └── Dockerfile
│   ├── perception/
│   │   └── Dockerfile
│   ├── hmi/
│   │   └── Dockerfile
│   └── simulation/
│       └── Dockerfile
├── compose/
│   ├── learn.yaml
│   ├── emul.yaml
│   └── target.yaml
├── config/
│   └── cyclonedds.xml
├── ros2_ws/
│   └── src/
│       ├── demo_interfaces/
│       ├── demo_description/
│       ├── demo_bringup/
│       ├── demo_navigation/
│       ├── demo_perception/
│       └── demo_simulation/
├── hmi/
│   ├── package.json
│   ├── src/
│   └── public/
├── scripts/
│   ├── bootstrap-host.sh
│   ├── setup-binfmt.sh
│   ├── build-images.sh
│   ├── verify-dds.sh
│   ├── collect-baseline.sh
│   ├── measure-ros-latency.py
│   ├── measure-node-cpu.sh
│   └── soak-test.sh
├── tests/
│   ├── compose/
│   └── integration/
└── docs/
    ├── architecture.md
    ├── development.md
    ├── network.md
    ├── hardware-bringup.md
    ├── validation-matrix.md
    ├── performance-results.md
    ├── torizon-cloud.md
    ├── runbook.md
    ├── decisions/
    └── results/
```

Add files only when they have a defined purpose. Do not create empty placeholder directories without a `.gitkeep` or explanatory README.

---

## 5. Implementation standards

### 5.1 ROS 2 packages

Use ROS 2 Jazzy conventions.

Preferred package split:

- `demo_interfaces`: project-specific messages or services only when standard messages are insufficient;
- `demo_description`: URDF/xacro, meshes, RViz development configuration, robot model tests;
- `demo_bringup`: explicit launch files for each mode;
- `demo_navigation`: Nav2 parameters, maps, behavior trees, costmap plugins/configuration;
- `demo_perception`: synthetic detection publisher and future interface seam;
- `demo_simulation`: SDF worlds, Gazebo spawn configuration, bridge mappings.

Prefer Python for small custom nodes and launch files unless C++ is justified by measurable runtime requirements.

Each Python ROS package must include:

- `package.xml`;
- `setup.py`;
- `setup.cfg`;
- resource-index marker;
- importable module;
- console entry points;
- tests;
- license declaration;
- executable launch files installed through package data.

Use standard ROS messages whenever possible. Do not define custom equivalents of `Twist`, `Odometry`, `Image`, `LaserScan`, or `Detection2DArray`.

### 5.2 Naming

- ROS package names: `snake_case`.
- ROS node names: `snake_case`.
- Topic names: lowercase, stable, and namespaced.
- Frame IDs: `map`, `odom`, `base_link`, sensor frames beneath `base_link`.
- Container services: short lowercase names such as `simulation`, `navigation`, `perception`, `rosbridge`, `hmi`.
- Environment variables: uppercase.
- Docker image names: `${REGISTRY:-local}/demo-aquila-<service>:${TAG:-dev}`.

Suggested namespace and topics:

```text
/demo/cmd_vel
/demo/odom
/demo/scan
/demo/camera/image_raw
/demo/perception/detections
/demo/navigation/status
/demo/system/heartbeat
```

Remap third-party nodes to this namespace rather than forking them.

### 5.3 TF and robot model

The minimum transform tree is:

```text
map -> odom -> base_link
                 ├── base_footprint, if required by Nav2
                 ├── laser_frame
                 └── camera_link
```

Rules:

- no duplicate publishers for the same transform;
- use `robot_state_publisher`;
- model fixed joints explicitly;
- make wheel geometry and sensor offsets xacro parameters;
- include a test that parses the URDF and checks required frames;
- include a development RViz configuration for x86 only;
- document which node owns `map -> odom` and `odom -> base_link`.

### 5.4 Launch files

Do not build one launch file with many nested conditionals.

Create explicit launch files such as:

```text
learn.launch.py
emul.launch.py
target.launch.py
navigation.launch.py
perception.launch.py
hmi_bridge.launch.py
```

Shared launch helpers are acceptable, but each mode must remain understandable from its top-level launch file.

### 5.5 Docker images

General requirements:

- use official ROS 2 Jazzy images where suitable;
- explicitly set `SHELL ["/bin/bash", "-o", "pipefail", "-c"]` when Bash behavior is needed;
- run `apt-get update` and install in the same layer;
- use `--no-install-recommends`;
- remove apt lists;
- use `rosdep` where practical;
- build the workspace with `colcon`;
- source ROS and the workspace from a controlled entrypoint;
- set CycloneDDS variables in the base image;
- use OCI labels for source revision and version;
- support both `linux/amd64` and `linux/arm64` for application images;
- keep the simulation image `linux/amd64` only;
- never copy secrets into images;
- do not use floating `latest` tags in release documentation.

The base image must expose a reusable ROS entrypoint, but application images must declare their own final command.

### 5.6 Compose modes

#### `compose/learn.yaml`

Purpose: native amd64 containerized development after the native ROS learning exercises.

- all services run on x86;
- application platform is `linux/amd64`;
- Gazebo and bridge are native amd64;
- may mount source for development where documented;
- must use host networking for ROS communication.

#### `compose/emul.yaml`

Purpose: prove arm64 image compatibility before hardware.

- simulation and bridge: `linux/amd64`;
- navigation, perception, and bridge/HMI back end: `linux/arm64`;
- QEMU/binfmt is a host prerequisite;
- do not interpret emulated performance as target performance;
- HMI front end may run in a desktop browser during this mode.

#### `compose/target.yaml`

Purpose: distributed execution.

- target services are `linux/arm64`;
- simulation services are `linux/amd64`;
- use Compose profiles or documented service selection so the same file can be invoked separately on workstation and Aquila;
- do not assume Docker Compose remotely schedules services between hosts;
- workstation example: start only `simulation` and `gz_bridge`;
- Aquila example: start only `navigation`, `perception`, `rosbridge`, and `hmi`;
- both invocations must use consistent image tag, domain ID, namespace, and configuration.

### 5.7 HMI

Use a small web application with a stable, minimal dependency set.

Required initial views:

- connection status;
- robot pose;
- map;
- planned path;
- current goal;
- navigation state;
- basic CPU/temperature telemetry when available;
- reserved perception panel;
- build/version identifier.

Data path:

```text
ROS 2 nodes -> rosbridge_server -> WebSocket -> Chromium kiosk HMI
```

Requirements:

- handle WebSocket reconnects;
- display stale-data state;
- avoid hard-coded hostnames;
- provide development configuration for localhost;
- use responsive layout suitable for the target DisplayPort screen;
- keep business logic outside visual components;
- add tests for message parsing and connection-state behavior;
- do not claim GPU acceleration based only on Chromium starting.

### 5.8 Documentation and evidence

Store human-readable documentation in `docs/`.

Store execution evidence in timestamped files under `docs/results/`, for example:

```text
docs/results/2026-08-15-dds-latency-am69.md
docs/results/2026-08-20-nav2-cpu-am69.csv
docs/results/2026-08-28-soak-test-am69.md
```

Never generate fake result files. Templates must be clearly marked `PENDING EXECUTION`.

Record:

- hardware revision;
- Torizon OS version;
- container image digest or tag;
- Git revision;
- test date;
- network topology;
- command used;
- raw result location;
- summary and conclusion.

---

## 6. Build and quality commands

Create a root `Makefile` with discoverable targets. The exact implementation may evolve, but keep these stable user-facing commands:

```text
make help
make bootstrap
make rosdep
make build-native
make test-native
make lint
make build-amd64
make build-arm64
make build-multiarch
make validate-compose
make up-learn
make down-learn
make up-emul
make down-emul
make verify-dds
make verify-topics
make test-integration
make docs-check
```

Target-specific commands may be added:

```text
make up-target-workstation
make up-target-aquila
make down-target-workstation
make down-target-aquila
make collect-baseline
make measure-latency
make measure-cpu
make soak-test
```

Do not make a generic quality target silently modify files. Separate check and fix operations.

Recommended checks:

- `colcon build --symlink-install`;
- `colcon test`;
- `colcon test-result --verbose`;
- `ament_flake8`;
- `ament_pep257`;
- `pytest`;
- `ruff check` only if explicitly added as a project dependency;
- `yamllint`;
- `shellcheck`;
- `hadolint`, when installed;
- `docker compose config`;
- xacro/URDF parsing;
- JSON/schema checks for HMI configuration.

If a tool is optional, detect it and report `SKIPPED` rather than failing misleadingly.

---

## 7. Configuration contract

Create `.env.example` with non-secret defaults:

```dotenv
COMPOSE_PROJECT_NAME=demo-aquila-ros2
ROS_DOMAIN_ID=69
ROS_NAMESPACE=/demo
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///config/cyclonedds.xml
REGISTRY=local
TAG=dev
WORKSTATION_HOSTNAME=demo-workstation
TARGET_HOSTNAME=aquila-am69
ROSBRIDGE_PORT=9090
HMI_HTTP_PORT=8080
DISPLAY=:0
```

Do not commit a populated `.env` if it contains environment-specific values.

Centralize runtime parameters. Avoid repeating domain IDs, topic names, ports, and image tags across multiple files.

---

## 8. Required test strategy

### 8.1 Unit and static tests

At minimum:

- perception stub message-generation tests;
- robot description parsing test;
- launch-file import/syntax tests where feasible;
- HMI topic parsing and stale-state tests;
- shell script lint;
- Compose rendering test;
- Dockerfile lint when available.

### 8.2 Integration tests on x86

Automate tests that can run without hardware:

1. publisher/subscriber communication;
2. required topic discovery;
3. actual message receipt;
4. TF tree required frames;
5. perception stub publishes `Detection2DArray`;
6. Gazebo bridge mapping configuration is valid;
7. Nav2 lifecycle nodes reach active state when the simulation is available;
8. HMI bridge accepts a WebSocket connection;
9. arm64 images build under buildx;
10. arm64 application containers start under QEMU.

### 8.3 Hardware tests

Hardware acceptance must remain pending until run on the Aquila:

- `tdx-info` baseline capture;
- one-hour DDS stability;
- latency and jitter across the two machines;
- image-topic bandwidth and loss;
- Nav2 CPU consumption per node;
- total robotics CPU budget;
- GPU acceleration for Chromium/Weston;
- combined thermal load;
- 48-hour soak test;
- restart-policy recovery;
- Torizon Cloud application update.

Do not substitute QEMU measurements for these tests.

---

## 9. Milestone plan

### M0 — Repository foundation

Deliver:

- repository structure;
- root README;
- `.gitignore`;
- `.env.example`;
- root Makefile;
- Compose skeletons that render;
- basic CI-ready validation scripts;
- architecture and validation-matrix documents.

Acceptance:

```bash
make help
make validate-compose
make docs-check
```

All must pass without hardware.

### ML1 — ROS 2 fundamentals

Deliver:

- a small `ament_python` package, preferably within `demo_bringup` or a dedicated tutorial submodule;
- heartbeat publisher;
- heartbeat subscriber;
- parameters;
- service example;
- explicit launch file;
- tests.

Acceptance:

- workspace builds natively;
- launch starts both nodes;
- subscriber receives messages;
- tests pass.

Keep educational code clearly separated from production demo nodes if it will not be reused.

### ML2 — Robot model and TF

Deliver:

- parameterized xacro robot;
- required links/joints;
- `robot_state_publisher`;
- TF ownership documentation;
- RViz development configuration;
- URDF tests.

Acceptance:

- xacro expands without error;
- required frames exist;
- no duplicate TF publisher in the default launch;
- RViz works on x86.

### ML3 — Native simulation and navigation

Deliver:

- SDF world;
- robot spawn;
- `ros_gz_bridge` mappings;
- keyboard teleoperation;
- static map;
- Nav2 configuration;
- native launch and run instructions.

Acceptance:

- robot can be teleoperated;
- odometry, scan, TF, and command topics exchange messages;
- Nav2 reaches active state;
- a navigation goal can complete in the simulated environment.

### ML4 — Containerization and arm64 emulation

Deliver:

- base, navigation, perception, and simulation images;
- `learn.yaml`;
- `emul.yaml`;
- buildx/binfmt scripts;
- multi-architecture build documentation.

Acceptance:

- application images build for amd64 and arm64;
- simulation image builds for amd64;
- arm64 navigation/perception containers start under QEMU;
- actual DDS messages cross between amd64 and emulated arm64 containers.

### M0-HW — Hardware bench preparation

Deliver software/runbook support for:

- clean Tezi installation checklist;
- thermal-solution checklist;
- network setup;
- `tdx-info` capture;
- baseline collection;
- CPU/NPU thermal test procedure;
- results templates.

Acceptance is manual and hardware-dependent. Do not mark complete without captured evidence.

### M1 — Cross-machine ROS 2 backbone

Deliver:

- hardened CycloneDDS configuration;
- cross-machine publisher/subscriber;
- latency and jitter measurement tool;
- one-hour stability script;
- network troubleshooting guide.

Acceptance on hardware:

- continuous message exchange for one hour;
- measured latency/jitter recorded;
- no false positive based only on topic discovery.

### M2 — Distributed simulation and Nav2 on Aquila

Deliver:

- `target.yaml` service/profile split;
- workstation simulation command;
- Aquila navigation/perception command;
- CPU measurement script;
- build/version telemetry.

Acceptance on hardware:

- Gazebo runs on x86;
- planner/controller/localization run on Aquila;
- robot navigates between points;
- CPU usage per relevant node is recorded;
- total robotics load is evaluated against a provisional 60% budget across the eight A72 cores.

### M3 — Embedded HMI

Deliver:

- web HMI;
- rosbridge launch/configuration;
- target Chromium kiosk service;
- GPU validation procedure;
- reconnect/stale-data handling.

Acceptance on hardware:

- HMI appears through direct DisplayPort;
- map, pose, path, state, and version update in real time;
- GPU acceleration is evidenced;
- software rendering is not the final path.

### M4 — Torizon Cloud application update

Deliver:

- application release/versioning convention;
- container image manifest;
- deployment documentation;
- rollback plan;
- update demo script or checklist;
- explicit separation between application-container update and unsafe base-OS migration.

Acceptance on hardware/cloud:

- a new application-container version is applied remotely;
- no physical access to the target is required;
- post-update health is verified;
- rollback or recovery path is documented.

### M5 — Hardening and runbook

Deliver:

- restart policies;
- health checks;
- service dependency handling;
- 48-hour soak-test script;
- operator runbook;
- failure/recovery matrix;
- event setup checklist;
- log collection bundle.

Acceptance on hardware:

- 48-hour run completes or failures are analyzed;
- containers recover from intentional process termination;
- cold boot returns to the demo automatically;
- operator can start, validate, demonstrate, and recover the system using the runbook.

### MX-TIDL — Time-boxed future feasibility study

This is not part of the initial product implementation.

Deliver only a two-day investigation plan/report covering:

- TIDL runtime availability for Torizon OS 7.4.0;
- required C7x device nodes;
- runtime/model filesystem requirements;
- licensing and redistribution constraints;
- containerization feasibility;
- alternative custom-Yocto path;
- rough implementation estimate;
- identified blockers.

Do not import large TI runtimes or models into the main repository during this milestone.

---

## 10. Perception stub specification

Implement a deterministic ROS 2 node named `synthetic_detector`.

Inputs:

```text
/demo/camera/image_raw    sensor_msgs/msg/Image
```

Outputs:

```text
/demo/perception/detections    vision_msgs/msg/Detection2DArray
/demo/perception/status        diagnostic_msgs/msg/DiagnosticArray
```

Behavior:

- consume image headers when images are available;
- publish configurable synthetic detections;
- use parameters for class ID, score, box center, box size, and publish rate;
- preserve input timestamp/frame when applicable;
- optionally publish without an input image for early integration testing;
- never infer or imply real AI execution;
- include deterministic tests;
- support disabling detections to test empty scenes.

The Nav2 integration seam may initially be an adapter interface and documented costmap design rather than a complete custom plugin, unless the requested milestone explicitly includes plugin implementation.

---

## 11. Observability

Every long-running service must provide at least one of:

- ROS diagnostics;
- a heartbeat topic;
- Docker health check;
- structured log message with component and version.

At startup, application services should log:

- Git revision when available;
- image/application version;
- ROS domain ID;
- RMW implementation;
- namespace;
- selected mode;
- relevant configuration paths.

Do not log credentials or full tokens.

The HMI should expose an aggregated demo health state:

```text
simulation link
DDS link
navigation state
perception stub state
rosbridge connection
target telemetry freshness
application version
```

---

## 12. Security and operational rules

- no default passwords in committed files;
- no private keys, API tokens, registry credentials, or Torizon Cloud credentials;
- use environment variables or documented secret injection;
- bind development web services only as broadly as necessary;
- document exposed ports;
- keep approval-required operations explicit;
- never run destructive host commands without user approval;
- never use `--privileged` unless a documented target requirement proves it necessary;
- prefer precise device mounts and capability grants;
- treat device provisioning and OTA deployment as explicit human-approved operations;
- preserve logs needed for failure analysis.

This is a demo, not a safety-certified product. Do not add claims of functional safety, production qualification, or long-term product support.

---

## 13. Decision records

Create an ADR in `docs/decisions/` for material architecture choices. Initial ADRs should cover:

1. Gazebo on x86 rather than Aquila;
2. CycloneDDS and host networking;
3. web HMI with rosbridge;
4. separate perception container and final topic contract;
5. explicit launch files per mode;
6. Torizon OS 7.4.0 clean-install rule;
7. distributed use of one Compose file through profiles/service selection rather than remote scheduling.

ADR format:

```markdown
# ADR-NNN: Title

- Status:
- Date:
- Context:
- Decision:
- Consequences:
- Alternatives considered:
- Validation:
```

---

## 14. README requirements

The root README must answer, near the top:

- what the demo proves;
- where each component runs;
- what can run without hardware;
- what requires the Aquila;
- why Gazebo and RViz2 stay on x86;
- how to start `learn`, `emul`, and `target` modes;
- how to run validation;
- the critical Torizon OS installation warning;
- where measured results are stored;
- which features remain future work.

Keep detailed procedures in `docs/` and link to them from the README.

---

## 15. Completion report format for Codex

At the end of a Codex task, report:

```text
Milestone:
Implemented:
Files changed:
Validation executed:
Validation result:
Hardware checks still pending:
Known limitations:
Suggested next milestone:
```

Include exact failing commands and concise error excerpts when validation fails.

---

## 16. Commands for using this guide with Codex CLI

Interactive use from the repository root:

```bash
codex --sandbox workspace-write --ask-for-approval on-request \
  "Read codex.md. Inspect the repository and implement only milestone M0. Run every validation available locally and stop after reporting the results."
```

Implement the next incomplete milestone:

```bash
codex --sandbox workspace-write --ask-for-approval on-request \
  "Read codex.md and docs/validation-matrix.md. Determine the next incomplete milestone, implement only that milestone, validate it, and report pending hardware checks."
```

Non-interactive use:

```bash
codex exec --sandbox workspace-write --ask-for-approval on-request -C . \
  "Read codex.md. Implement only milestone ML1, run its acceptance checks, and output the completion report."
```

To make these instructions load automatically, keep the included `AGENTS.md` synchronized with `codex.md`.

---

## 17. First task

When this is a new or nearly empty repository, the first Codex task is:

```text
Implement milestone M0 only.

Create the repository foundation, explicit documentation skeleton, environment contract, Makefile, Compose files that render without launching services, validation scripts, and initial ADRs. Do not implement Nav2, Gazebo worlds, the HMI, Torizon Cloud deployment, or hardware-specific results yet.

Run:
- make help
- make validate-compose
- make docs-check

Report all generated files and any host prerequisites that remain missing.
```
