# demo_simulation

Gazebo Harmonic world, robot spawn, `ros_gz_bridge` mappings, and keyboard
teleop. Delivers the simulation half of ML3.

**Runs on:** x86 workstation only, in **all three modes**. Gazebo is an OGRE 2 /
desktop-OpenGL application and the AM69 GPU exposes only OpenGL ES 3.2 and
Vulkan 1.2 — see `CLAUDE.md` rule 1. In `target` mode this package still runs on
the host; only navigation and perception move to the module.

## Setup — the world is not vendored

The warehouse world comes from `nav2_minimal_tb4_sim`, maintained by the Nav2
organization and native Harmonic SDF. It is installed as a system package rather
than copied into this repo:

```bash
sudo apt install -y ros-jazzy-nav2-minimal-tb4-sim
```

Then either copy the world into `worlds/`, or point the launch at the installed
copy:

```bash
ros2 launch demo_simulation simulation.launch.py \
    world:=$(ros2 pkg prefix nav2_minimal_tb4_sim)/share/nav2_minimal_tb4_sim/worlds/depot.sdf
```

The AWS RoboMaker warehouse world was archived in July 2026 and is Gazebo
Classic — do not use it.

## Topic bridge

The plant selector uses one boundary file per robot:

- `config/bridge_warehouse.yaml` for the diff-drive;
- `config/bridge_quadruped.yaml` for the Go2.

Every ROS-side name lives under `/demo`, per the topic contract. The Go2's
`/demo/cmd_vel` does not cross into Gazebo: `twist_to_inputs` consumes it as a
ROS topic in the `sim` container and translates SI velocities to the private
normalized `/control_input` message used by the gait controller.

| Gazebo | ROS 2 | Direction |
| --- | --- | --- |
| `/clock` | `/clock` | gz → ros |
| `/cmd_vel` | `/demo/cmd_vel` | ros → gz |
| `/odom` | `/demo/odom` | gz → ros |
| `/tf` | `/tf` | gz → ros |
| `/joint_states` | `/joint_states` | gz → ros |
| `/scan` | `/demo/scan` | gz → ros |
| `/camera/image_raw` | `/demo/camera/image_raw` | gz → ros |
| `/camera/camera_info` | `/demo/camera/camera_info` | gz → ros |

For the Go2, `go2_sim.urdf.xacro` attaches a camera and lidar to links already
present in the vendored model. It also publishes `/demo/odom` from exact Gazebo
model pose as an explicit **temporary F4 source**. Its ground-truth TF topic is
not bridged. F5 replaces this odometry producer with legged-state estimation
and owns `odom → base_link`; publishing both would create a duplicate TF edge.

`worlds/quadruped_empty.sdf` is the deterministic, asset-free integration
fixture. Do not substitute Gazebo's built-in `empty.sdf`: it lacks the Sensors
system, so the camera and lidar appear in the model but never publish. The
warehouse world also has the Sensors system and remains the demo default.

**Every gz-side name here is unscoped, and that is not an oversight.**

The `<topic>`, `<odom_topic>` and `<tf_topic>` elements in the DiffDrive and
JointStatePublisher plugin blocks are taken *literally*. The plugins subscribe
to `/cmd_vel` and publish on `/odom`, `/tf` and `/joint_states` — no model
prefix, no world prefix.

The trap: Gazebo **also** advertises `/model/demo_robot/{cmd_vel,odom,tf}` and
`/world/warehouse/model/demo_robot/joint_state` as its default names. Those show
up in `gz topic -l`, look more "correct", and have nothing attached to them.
Bridging to them produces a robot that never moves and odometry that never
publishes, with no error printed anywhere.

`gz topic -l` alone will not tell you which is which — it lists both. Use
`gz topic -i -t <name>` and look for a real publisher/subscriber:

```bash
gz topic -i -t /cmd_vel                    # Subscribers: [address]  <- the live one
gz topic -i -t /model/demo_robot/cmd_vel   # No subscribers on topic  <- the decoy
```

Sensor topics are short for a different reason: the `<topic>` values in
`demo_description/urdf/_sensors.xacro` deliberately override Gazebo's
auto-generated
`/world/warehouse/model/demo_robot/link/laser_frame/sensor/lidar/scan`, which
embeds the world name and would break the moment the world changes.

## TF ownership

This package publishes no transforms itself. It starts
`robot_state_publisher` (fixed edges, from the URDF) and bridges
`odom → base_footprint` out of the `DiffDrive` plugin. `map → odom` belongs to
AMCL in `demo_navigation`. Full table in `demo_description/README.md`.

## Startup timing

`simulation.launch.py` delays the spawner to t=12 s and the bridge to t=15 s.
Do not remove those timers. `ros_gz_sim create` first calls Gazebo's
"list of world names" service; started at t=0 that service does not exist yet
and the client retries every 5 s **forever** instead of failing. The warehouse
world needs ~10 s to load its meshes.

The failure mode is deceptive: the robot still appears in the world (the async
create service accepts it) and the sensors publish, so `gz model --list` shows
`demo_robot` and everything looks healthy — but the DiffDrive and
JointStatePublisher plugins never initialize, and odometry and joint states stay
silent forever.

On a slower machine, raise both timers (and the matching ones in
`demo_bringup/launch/learn.launch.py`).

## Running

### Go2 no spike

Na imagem `demo-sim:spike-go2`, execute o script na raiz do repositório. O
padrão é o fixture `quadruped_empty.sdf` (com Sensors); um caminho absoluto
para outro cenário pode ser passado:

```bash
./scripts/run_quadruped_sim.sh
# quando disponível:
./scripts/run_quadruped_sim.sh /caminho/absoluto/warehouse.sdf
```

O script compila os seis pacotes no container e usa o nome fixo
`aquila-go2`. Em outro terminal, depois do log `fixed stand. Waiting for a
non-zero /demo/cmd_vel`, envie uma velocidade baixa:

```bash
docker exec -it aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh && . /test/install/setup.sh
  ros2 topic pub -r 10 /demo/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.03}}"
'
```

O robô entra no trote, mas a passada dinâmica ainda está instável: no ensaio
de 18/08/2026 ele avançou sem alternância de pernas confiável e caiu. Portanto
F4 continua aberto; a estabilidade em `FIXEDSTAND` é o critério atualmente
validado. Pare o publisher com `Ctrl-C` e o simulador no primeiro terminal.

```bash
# terminal 1 — simulator, robot, bridge
ros2 launch demo_simulation simulation.launch.py

# terminal 2 — drive it
ros2 launch demo_simulation teleop.launch.py
```

Headless, for CI:

```bash
ros2 launch demo_simulation simulation.launch.py gui:=false
```

Arguments: `world`, `robot_name`, `x`, `y`, `yaw`, `gui`, `model`.

### Teleop needs a TTY

`teleop_twist_keyboard` reads keys from the focused terminal, and a
launch-started node has no attached TTY — hence `prefix='xterm -e'`. Without
xterm installed, run the node directly instead:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/demo/cmd_vel
```

Do not run teleop and Nav2 simultaneously unless you want to watch them fight
over `/demo/cmd_vel`.

## Verifying the graph

```bash
ros2 topic list | grep demo
ros2 topic hz /demo/scan          # ~10 Hz
ros2 topic hz /demo/camera/image_raw   # ~15 Hz
ros2 topic echo /demo/odom --once
ros2 run tf2_tools view_frames
```
