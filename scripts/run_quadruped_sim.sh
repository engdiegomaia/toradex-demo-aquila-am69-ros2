#!/usr/bin/env bash
set -euo pipefail

# Usage: ./scripts/run_quadruped_sim.sh [world.sdf] [launch_arg:=value...]
#
# Developer shortcut: runs only the Go2 plant in a standalone container named
# aquila-go2, rebuilding the simulation packages from the working tree so that
# local edits apply without an image rebuild. The default world is
# quadruped_empty.sdf. Keep this terminal open. The Compose `sim` service is the
# supported path for the demo itself.
#
# SIM_IMAGE overrides the image (default: the Compose sim image,
# ${REGISTRY:-local}/demo-aquila-sim:${TAG:-dev}; build it first).
#
# MAZE_MODELS=<dir> mounts a model directory OUTSIDE the repository at
# /maze/models and points GZ_SIM_RESOURCE_PATH at it. It is what the
# quadruped_maze*.sdf worlds need: the maze mesh comes from
# github.com/cafemesa/ros_maze_worlds, which is not versioned here because its
# package.xml declares <license>TODO</license>. Without the variable the world
# loads and the maze does not appear -- a mesh that does not resolve is a
# warning line, not a fatal error.
#
# Point MAZE_MODELS at a STABLE path, not /tmp: a reboot in the middle of a
# demonstration wipes the clone and the maze stops loading again, with the same
# discreet warning as always.
#
# GO2_SPAWN_YAW=<rad> rotates the robot at spawn. Normally it does NOT need to
# be passed: the world declares its own yaw in a `<!-- go2_spawn_yaw: N -->`
# line and this script reads it. The yaw belongs to the world because it is
# the world's geometry -- in a maze corner the default +x is a wall, and
# getting out requires turning in place, which is what this robot does worst.
# The environment variable overrides the file.

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
world_path="${1:-quadruped_empty.sdf}"
if [ "$#" -gt 0 ]; then
  shift
fi
launch_args=("$@")
container_name="aquila-go2"

maze_mount=()
maze_env=()
if [ -n "${MAZE_MODELS:-}" ]; then
  if [ ! -d "${MAZE_MODELS}" ]; then
    echo "MAZE_MODELS=${MAZE_MODELS} is not a directory." >&2
    exit 1
  fi
  maze_mount=(-v "$(cd "${MAZE_MODELS}" && pwd):/maze/models:ro")
  maze_env=(-e GZ_SIM_RESOURCE_PATH=/maze/models)
  echo "External models: ${MAZE_MODELS} -> /maze/models"
else
  # Matches ANY maze, not just the original file's maze10. A guard tied to a
  # literal name lets every new world reintroduce the same silent failure:
  # Gazebo starts, the maze does not appear, and the run looks like a perfect
  # detour in open field.
  case "$(basename "${world_path}")" in
    quadruped_maze*.sdf)
      echo "$(basename "${world_path}") requires MAZE_MODELS pointing at the models" >&2
      echo "of ros_maze_worlds; without it the maze does not load and the robot walks in" >&2
      echo "open field with no error. See docs/guides/cenarios/s6-labirinto.md" >&2
      exit 1
      ;;
  esac
fi

# Spawn yaw: the world declares its own; the environment variable overrides it.
# A name without a slash is resolved in the repository tree, which is where the
# file is before the container installs the share.
spawn_yaw="${GO2_SPAWN_YAW:-}"
if [ -z "${spawn_yaw}" ]; then
  case "${world_path}" in
    */*) world_file="${world_path}" ;;
    *) world_file="${repo_dir}/ros2_ws/src/demo_simulation/worlds/${world_path}" ;;
  esac
  if [ -f "${world_file}" ]; then
    spawn_yaw="$(sed -n 's/.*go2_spawn_yaw:[[:space:]]*\([-0-9.]\+\).*/\1/p' \
      "${world_file}" | head -n1)"
  fi
fi
spawn_yaw="${spawn_yaw:-0.0}"
echo "Spawn yaw: ${spawn_yaw} rad"

# ==================== DDS configuration, for hil mode ====================
#
# Without this the simulator container uses the CycloneDDS DEFAULT: multicast
# ON and no explicit peer. The Aquila module runs with AllowMulticast=false and
# explicit peers, and the two halves never find each other -- host->module
# because the module does not listen on multicast, module->host because a
# default participant does not pin a deterministic port and there is no port to
# aim at. The symptom is identical to a firewall blocking, and nothing in the
# log names DDS (guia-operacao.md section 9.11).
#
# The rendered file is NOT versioned: `scripts/module.sh sync` generates it
# with the module's address and the host's interface. If it does not exist, the
# simulator comes up with the default -- which is right for pure learn mode,
# everything on one machine -- and warns, instead of failing.
dds_env=()
dds_mount=()
host_dds="${repo_dir}/docker/cyclonedds/host.rendered.xml"
if [ -f "${host_dds}" ]; then
  dds_mount=(-v "${host_dds}:/cfg/cyclonedds.xml:ro")
  dds_env=(-e CYCLONEDDS_URI=file:///cfg/cyclonedds.xml)
  echo "DDS: host.rendered.xml (peers: $(grep -c '<Peer ' "${host_dds}"))"

  # The interface pinned in the file must be the one that ROUTES to the module
  # today. Diverging is a silent, recurring failure: the Wi-Fi cable is swapped
  # for Ethernet, the kernel starts routing through the new interface, and
  # host.rendered.xml keeps pinning the old one. CycloneDDS transmits from an
  # address the module cannot reply to, discovery fails, and nothing in the log
  # names any interface -- it looks like a firewall.
  #
  # The two patterns match the ELEMENT, not the word: host.xml's comment block
  # discusses `<NetworkInterface name="eth0"/>` and `name="lo"` in prose, and a
  # loose sed pins "eth0" and warns wrongly -- which is what happened in the
  # first version of this guard. So the whole line is anchored, accepting only
  # an XML comment after the closing tag.
  pinned_iface="$(sed -n \
    's|^[[:space:]]*<NetworkInterface [^>]*name="\([^"]*\)"[^>]*/>[[:space:]]*\(<!--.*\)\?$|\1|p' \
    "${host_dds}" | head -n1)"
  module_peer="$(sed -n \
    's|^[[:space:]]*<Peer address="\([^"]*\)"[^>]*/>[[:space:]]*\(<!--.*\)\?$|\1|p' \
    "${host_dds}" | grep -v '^127\.0\.0\.1$' | head -n1)"
  if [ -n "${pinned_iface}" ] && [ -n "${module_peer}" ]; then
    route_iface="$(ip route get "${module_peer}" 2>/dev/null \
      | sed -n 's/.* dev \([^ ]*\).*/\1/p' | head -n1)"
    if [ -n "${route_iface}" ] && [ "${route_iface}" != "${pinned_iface}" ]; then
      echo "     WARNING: the file pins ${pinned_iface}, but the route to" >&2
      echo "     ${module_peer} goes out via ${route_iface}. Run: scripts/module.sh sync" >&2
    else
      echo "     interface ${pinned_iface}, which is the one that routes to ${module_peer}"
    fi
  fi
  # Half a DDS configuration fails just like a firewall: if the simulator uses
  # explicit peers and this host's native Nav2 uses the multicast default, the
  # two cannot find each other on the same machine. Anyone running something
  # native needs the SAME config, which is why the line is printed instead of
  # left implied.
  echo "     NATIVE nodes on this host need the same config. In the other terminal:"
  echo "     export CYCLONEDDS_URI=file://${host_dds}"
else
  echo "DDS: CycloneDDS default (multicast). Fine for learn mode on a single"
  echo "     machine. For hil with the Aquila, run first: scripts/module.sh sync"
fi

if docker ps --format '{{.Names}}' | grep -qx "${container_name}"; then
  echo "Container ${container_name} is already running." >&2
  exit 1
fi

cd "${repo_dir}"
xhost +local:docker >/dev/null

docker run --rm --name "${container_name}" --network=host \
  -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-69}" \
  -e GO2_WORLD="${world_path}" -e GO2_SPAWN_YAW="${spawn_yaw}" \
  -e LAUNCH_ARGS="${launch_args[*]}" \
  "${maze_env[@]}" "${maze_mount[@]}" \
  "${dds_env[@]}" "${dds_mount[@]}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri:/dev/dri --group-add "${RENDER_GID:-992}" \
  -v "${repo_dir}/ros2_ws/src:/proj/src:ro" \
  --entrypoint bash "${SIM_IMAGE:-${REGISTRY:-local}/demo-aquila-sim:${TAG:-dev}}" -c '
    set -e
    . /opt/ros/jazzy/setup.sh
    mkdir -p /test/src
    cd /test
    cp -r /proj/src/go2_description /proj/src/control_input_msgs \
          /proj/src/controller_common /proj/src/unitree_guide_controller \
          /proj/src/gz_quadruped_hardware /proj/src/demo_simulation src/
    colcon build --symlink-install
    . install/setup.sh
    world="${GO2_WORLD}"
    # A name without a slash is resolved in the demo_simulation share. Passing
    # only the file name is the common case, and Gazebo does not find the world
    # there on its own -- it does not inherit the colcon install path. An argument
    # WITH a slash is treated as an explicit path and used as given.
    case "${world}" in
      */*) : ;;
      *) world=/test/install/demo_simulation/share/demo_simulation/worlds/"${world}" ;;
    esac
    ros2 launch demo_simulation quadruped.launch.py gui:=true \
        world:="${world}" yaw:="${GO2_SPAWN_YAW}" ${LAUNCH_ARGS}
  '
