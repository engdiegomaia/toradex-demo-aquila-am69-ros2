# Getting Started

This guide brings the demo up on a single x86 workstation (`learn` mode):
simulation, navigation, perception and the web cockpit, all in containers.
Continue with [hil-deployment.md](hil-deployment.md) to move navigation and
perception to the Toradex Aquila AM69.

## Requirements

### Host workstation

| Item | Requirement |
| --- | --- |
| OS | Ubuntu 24.04 LTS, x86_64 |
| Container runtime | Docker Engine with the Compose v2 plugin and `buildx` |
| GPU | A working desktop OpenGL driver (`glxinfo -B` must report a hardware renderer, not `llvmpipe`). Intel and AMD work through `/dev/dri`. NVIDIA needs `nvidia-container-toolkit` and `gpus: all` in place of the `devices:` mapping. |
| Display | An X11 session. Gazebo and RViz2 open windows on the host display. |
| Network (HIL only) | Wired Ethernet to the module. Wi-Fi works but is not recommended. |

A native ROS 2 Jazzy installation is optional. You only need it to run `ros2`
commands outside the containers (see [development.md](development.md)).

### External assets

The official scenario is a maze whose mesh comes from
[`cafemesa/ros_maze_worlds`](https://github.com/cafemesa/ros_maze_worlds). It is
not vendored here because that repository does not declare a license. Clone it
next to this repository:

```bash
git clone https://github.com/cafemesa/ros_maze_worlds.git ~/ros_maze_worlds
```

## 1. Clone and configure

```bash
git clone <this-repository-url> aquila-am69-ros2
cd aquila-am69-ros2
cp docker/.env.example docker/.env
```

Edit `docker/.env`. For `learn` mode, the relevant values are:

| Variable | Set to |
| --- | --- |
| `MAZE_MODELS` | Absolute path to `ros_maze_worlds/models`, for example `/home/<user>/ros_maze_worlds/models` |
| `RENDER_GID` | Output of `getent group render \| cut -d: -f3`. Without it Gazebo silently falls back to software rendering. |
| `DISPLAY` | Your X display, usually `:0` |

Every variable is documented in [configuration.md](configuration.md).

Render the local DDS configuration. The containers mount the rendered file,
not the template:

```bash
scripts/module.sh render-local
```

Allow containers to open windows on your display:

```bash
xhost +local:docker
```

## 2. Build the images

Build the base image first, then the role images:

```bash
BUILDX_BUILDER=default docker compose -f docker/compose.host.yml build base
BUILDX_BUILDER=default docker compose -f docker/compose.host.yml --profile learn build
```

`BUILDX_BUILDER=default` matters when a `docker-container` buildx builder is
active (for example one created for multi-arch builds). Such a builder cannot
see images in the local store, so every `FROM local/demo-aquila-base:dev` fails
with `pull access denied`.

## 3. Start the stack

```bash
docker compose -f docker/compose.host.yml --profile learn up -d sim nav perception cockpit hmi
```

Add `viz` to also open RViz2. Gazebo opens on the host display; set
`SIM_GUI=false` in `docker/.env` to run it headless.

Open the cockpit at <http://localhost:8081>.

The **exploration** button in the navigation panel starts the autonomous
maze exploration. The whole run is described in
[running-the-demo.md](running-the-demo.md).

## 4. Check that it works

From a `tools` container (no native ROS needed):

```bash
docker compose -f docker/compose.host.yml --profile tools up -d tools
docker compose -f docker/compose.host.yml exec tools /usr/local/bin/entrypoint.sh bash
# inside the container:
ros2 topic list | grep /demo
ros2 topic hz /demo/odom
ros2 action list        # /navigate_to_pose should be present
```

`docker compose exec` does not run the image entry point, so call it
explicitly as shown. Otherwise `ros2` is not on the `PATH`.

## 5. Stop

```bash
docker compose -f docker/compose.host.yml --profile learn --profile tools down
```

## Next steps

- [Hardware-in-the-loop on the Aquila AM69](hil-deployment.md)
- [Simulation scenarios](scenarios.md)
- [Web cockpit](cockpit.md)
- [Troubleshooting](troubleshooting.md)
