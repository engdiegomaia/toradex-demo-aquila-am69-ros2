# ML3.5, containerized architecture and execution guide

Implementation spec. It goes on the original plan where there is divergence.

Target: ROS 2 Jazzy, Gazebo Harmonic, quadrupede A1, host x86 simulation more module
Aquila AM69 running Torizon OS.

---

## 1. Modularity axis

Decomp is not by package ROS. It is by answering a question: what needs to be changed
when the simulation becomes real hardware?

The answer is one thing, the plant. Everything above it consumes the same topic contract
and does not know if you are talking to Gazebo or a physical A1.

```
        planta (trocável)                consumidores (fixos)
  ┌──────────────────────────┐     ┌─────────────────────────────┐
  │ sim   (Gazebo + control) │     │ nav        (Nav2)           │
  │   ou                     │ ==> │ perception (demo_perception)│
  │ hw    (A1 real, futuro)  │     │ viz        (RViz2)          │
  └──────────────────────────┘     └─────────────────────────────┘
            contrato: /demo/cmd_vel  /demo/odom  /demo/scan  /demo/camera/image_raw
```

Why `sim` is a container only and not two: `gz_ros2_control` is Gazebo plugin and loads
`controller_manager` within the `gz sim` process. Separate Gazebo from controllers in
different containers is not possible without rewriting integration. So the simulated
plant is a unit: Gazebo, `gz_ros2_control`, march controllers, ZZXQ005QXZZ and the
ZZXQ006QXZZ bridge.

---

## 2. Containers

| Container | Architecture | Where it spins | Content |
|---|---|---|---|
| `sim` | x86 64 only | host | Gazebo Harmonic, `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`, quadruped controllers, robot description, world |
| `nav` | x86 64 and arm64 | host or module | Nav2, map, params, costmaps |
| `perception` | x86 64 and arm64 | host or module | `demo_perception` without change |
| `viz` | x86 64 only | host only | RViz2, rqt |
| `tools` | x86 64 and arm64 | any | teleop, CLI ROS ZZX0002QXZZ, colcon, test run |
| `hw` | arm64 only | module | placeholder, A1 physical, outside the scope of ML3.5 |

`sim` and `viz` never go to arm64. 1 rule of the project, Gazebo is OGRE ZZX0004QXZZ and
RViz2 either ZZXQ005QXZZ desktop. The multi-arch build is selective, not uniform.

`hw` exists in the repo since F1, empty, with a README of a line. This is where the
collision of invariants registered in the plan will hit: the basis of
`quadruped_ros2_control` documents conflict between CyclonedDS and `unitree_sdk2` and
recommends FastDDS, while the project's 2 rule is CyclonedDS always. As long as the
ZZXQ006QXZZ is simulated, the SDK does not enter and there is no collision. The empty
container serves to make the problem visible in the right place instead of appearing as
a surprise.

---

## 3. Layout in repository

```
docker/
  base/Dockerfile           # ros:jazzy-ros-base, cyclonedds, usuário não root, entrypoint
  sim/Dockerfile            # FROM base, ros-jazzy-ros-gz, gz_ros2_control, controladores
  nav/Dockerfile            # FROM base, nav2
  perception/Dockerfile     # FROM base, deps de demo_perception
  viz/Dockerfile            # FROM base, rviz2, rqt
  tools/Dockerfile          # FROM base, teleop, colcon, pytest
  hw/README.md              # placeholder, A1 físico
  compose.host.yml
  compose.module.yml
  cyclonedds/host.xml
  cyclonedds/module.xml
  entrypoint.sh
  .env.example
docs/ml35/
  guia-ml35-docker.md       # este arquivo
```

`base` centralizes RMW configuration and `source` overlay configuration. A place to
change, not six.

---

## 4. Base Image

`ros:jazzy-ros-base` has an arm64 tag and serves host and module. Torizon OS is a Docker
host, so the official image runs in the module without userspace adaptation.

Two restrictions that apply to the arm64 images:

- Torizon container storage is on the data partition. Fat layer costs real space. Use `--no-install-recommends`, clean `/var/lib/apt/lists` in the same layer and keep ZZX0002QXZZ and `perception` without any graphics.
- Access to the AM69 accelerator from the container requires the device nodes and runtime of TI, which the generic ROS image does not bring. If `demo_perception` is for accelerated inference at any time, confirm the nodes and stack in the Toradex and TI documentation before assuming anything. List ZZXQ005QXZZ on host first. As long as the perception is CPU and OpenCV, none of this is necessary.

Build arm64 from host x86:

```bash
docker run --privileged --rm tonistiigi/binfmt --install arm64
docker buildx create --use --name demo || docker buildx use demo
docker buildx build --platform linux/arm64 \
  -f docker/nav/Dockerfile -t ${REGISTRY}/demo-nav:${TAG} --push .
```

QEMU here builds image. It doesn't measure anything. 5 rule.

---

## 5. DDS between containers and between machines

All containers use `network_mode: host`. This solves discovery between containers of the
same machine without additional configuration and avoids the problem class of DDS behind
bridge NAT.

Between host and module, multicast usually dies in Wi-Fi and managed switch. Don't
depend on him. Use explicit pears.

`docker/cyclonedds/host.xml`:

```xml
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="eth0" priority="default"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
        <Peer address="${HOST_IP}"/>
        <Peer address="${MODULE_IP}"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

`module.xml` is the same file with the module interface. Adjust `NetworkInterface name`
to what is on the machine, check with `ip -br link`.

`.env.example`:

```
REGISTRY=registry.local/demo
TAG=ml35
ROS_DOMAIN_ID=42
HOST_IP=192.168.1.10
MODULE_IP=192.168.1.20
DISPLAY=:0
```

`ROS_DOMAIN_ID` the same on both machines. Different domain is the most common cause of
"the topics do not appear" and does not cause any error.

---

## 6. Host Compose

`docker/compose.host.yml`, excerpt:

```yaml
x-common: &common
  network_mode: host
  environment:
    - ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
    - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    - CYCLONEDDS_URI=file:///cfg/cyclonedds.xml
  volumes:
    - ./cyclonedds/host.xml:/cfg/cyclonedds.xml:ro

services:
  sim:
    <<: *common
    image: ${REGISTRY}/demo-sim:${TAG}
    build:
      context: ..
      dockerfile: docker/sim/Dockerfile
    ipc: host
    environment:
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      - CYCLONEDDS_URI=file:///cfg/cyclonedds.xml
      - DISPLAY=${DISPLAY}
    volumes:
      - ./cyclonedds/host.xml:/cfg/cyclonedds.xml:ro
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    devices:
      - /dev/dri:/dev/dri
    command: >
      ros2 launch demo_bringup sim.launch.py
      robot_type:=${ROBOT_TYPE:-quadruped}

  nav:
    <<: *common
    image: ${REGISTRY}/demo-nav:${TAG}
    profiles: ["learn"]
    command: >
      ros2 launch demo_bringup nav_select.launch.py
      robot_type:=${ROBOT_TYPE:-quadruped}
      use_sim_time:=true

  perception:
    <<: *common
    image: ${REGISTRY}/demo-perception:${TAG}
    profiles: ["learn"]
    command: ros2 launch demo_bringup perception.launch.py use_sim_time:=true

  viz:
    <<: *common
    image: ${REGISTRY}/demo-viz:${TAG}
    ipc: host
    command: ros2 launch demo_bringup viz.launch.py
```

`nav` and `perception` are under the `learn` profile. In HIL mode they just don't go on
the host, without editing the file.

For NVIDIA on the host, change the mapping of `/dev/dri` by `gpus: all` with the
nvidia-container-toolkit installed. For Intel and AMD, `/dev/dri` is enough.

`use_sim_time` is `true` in everything that consumes the simulation, including in the
module in HIL mode, and the bridge needs to publish `/clock`. Wrong clock in module
produces TF extrapolating and Nav2 refusing goal without obvious message.

---

## 7. Module Compose

`docker/compose.module.yml`:

```yaml
x-common: &common
  network_mode: host
  restart: unless-stopped
  environment:
    - ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
    - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    - CYCLONEDDS_URI=file:///cfg/cyclonedds.xml
  volumes:
    - ./cyclonedds/module.xml:/cfg/cyclonedds.xml:ro

services:
  nav:
    <<: *common
    image: ${REGISTRY}/demo-nav:${TAG}
    command: >
      ros2 launch demo_bringup nav_select.launch.py
      robot_type:=${ROBOT_TYPE:-quadruped}
      use_sim_time:=true

  perception:
    <<: *common
    image: ${REGISTRY}/demo-perception:${TAG}
    command: ros2 launch demo_bringup perception.launch.py use_sim_time:=true
```

No `sim`, no `viz`, no X11, no `/dev/dri`. Nothing graphic gets to the module. The same
`ROBOT_TYPE` selects the plant in the host and the corresponding Nav2 in the two
Compose. In the quadruped, `odom_tf` still derives the ZZXQ00006QXZZ from Gazebo's
ground truth; leg estimation remains outside the closure of ML3.ZZXQ008QXZZ.

When there is a real camera in deploy mode, `perception` wins the device mapping:

```yaml
    devices:
      - "/dev/video0:/dev/video0"
```

Before debugging container permission, confirm that the node exists in the host. If
Device Tree didn't instigate the sensor, there's no mapping it can solve.

---

## 8. Execution modes

| Mode | Host x86 | Module Aquila AM69 | What is it for? |
|---|---|---|---|
| `learn` | `sim`, `nav`, `perception`, `viz` | off | development and gate of F2 to F5 |
| `hil` | `sim`, `viz` | `nav`, `perception` | proves that the division works, is the deliverable of ML3.5 |
| `deploy` | Nothing | `nav`, `perception`, `hw` | A1 physical, out of scope |

`learn` is what exists today, containerized. `hil` is the way to demonstrate: the
simulation wheel where it has GPU, navigation and perception run where they will run in
production.

---

## 9. How to rotate

### Learn mode, all in the host

```bash
cd docker
cp .env.example .env          # ajuste IPs e DISPLAY
xhost +local:docker
docker compose -f compose.host.yml --profile learn up --build
```

Verification from another terminal:

```bash
docker compose -f compose.host.yml exec tools bash
ros2 topic list | grep /demo/
ros2 topic hz /demo/scan
ros2 topic echo /demo/odom --once
ros2 control list_controllers
```

Navigation goal, same criterion as ML3 has already hit:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}}}}"
```

Close:

```bash
docker compose -f compose.host.yml --profile learn down
xhost -local:docker
```

### Hil mode, host simulation and module navigation

Host, only the plant and the display:

```bash
cd docker
docker compose -f compose.host.yml up sim viz
```

Post the arm64 images and take the compose to the module:

```bash
docker buildx build --platform linux/arm64 -f nav/Dockerfile \
  -t ${REGISTRY}/demo-nav:${TAG} --push ..
docker buildx build --platform linux/arm64 -f perception/Dockerfile \
  -t ${REGISTRY}/demo-perception:${TAG} --push ..

scp compose.module.yml .env cyclonedds/module.xml torizon@${MODULE_IP}:~/demo/
```

No register accessible, download the direct image:

```bash
docker save ${REGISTRY}/demo-nav:${TAG} | ssh torizon@${MODULE_IP} docker load
```

Module:

```bash
ssh torizon@${MODULE_IP}
cd ~/demo && docker compose -f compose.module.yml up -d
docker compose -f compose.module.yml logs -f nav
```

Check that the two machines see each other from the module:

```bash
ros2 daemon stop && ros2 daemon start
ros2 topic list | grep /demo/
ros2 topic hz /demo/scan
```

If the list is empty, the check order is: `ROS_DOMAIN_ID` equal on both sides, `ip -br
link` hitting with the `NetworkInterface name` ZZX0003QXZZ, `Peers` IPs correct, host
firewall releasing ZZXQ005QXZZ ZZXQ006QXZZ and adjacent.

### Deploy Mode

Outside the scope of ML3.5. `hw` container is empty until it exists A1 physical, and
that's where the CycloneDDS decision against FastDDS will need to be made.

---

## 10. Where each phase touches the infrastructure

| Phase | Docker | ROS |
|---|---|---|
| F0 | Nothing | ML3.1, 1047 line commit |
| F1 | creates `docker/` whole, `base`, `sim`, `nav`, `perception`, `viz`, `tools`, the two compounds, XML by ZZXQ008QXZZ | no behavior change, just packing the current diff-drive |
| F2 | Disposable tag `demo-sim:spike-go2` | clone from upstream base, Go2 without modification |
| F3 | same image `sim`, changes content | description of A1, kinematics, masses, joint limits, knitted or crocheted |
| F4 | Nothing | remaps for `/demo/*`, `demo_perception` untouched |
| F5 | promotes `nav` and `perception` for arm64, valid `hil` | `robot_radius`, footprint, tolerances |
| F6 | profile and variable `ROBOT_TYPE` no compose | # Robot type #|diffdrive`, extended tests |

F1 before F2 is deliberate. If you make it break after the quadruped enters, you don't
know if it was Docker, DDS or march. Containering what already works, F2 fails for one
reason only.

---

## 11. Points to confirm, not to assume

The original plan states things about `quadruped_ros2_control` that need to be checked
at the time of the clone in F2:

- Real support for Jazzy and Harmonic at the default branch.
- License before selling any description derived from `unitree_ros`.
- Absence of A1 config and what exactly comes from `chvmp/robots`.

About module:

- Device nodes and runtime required for the AM69 accelerator inside container. Confirm in the Toradex and TI documentation, list `/dev` in the host before.
- Free space on Torizon data partition before uploading images.

Silent risk that continues to be worth the original plane: tuned gait parameter for Go2
running on a A1 produces robot that walks poorly without generating error. Same class as
the scaling trap ML3.1 has already paid for. The F3 gate is robot standing and stable
responding to `cmd_vel`, not built clean.
