#!/usr/bin/env bash
set -euo pipefail

# Usage: ./scripts/run_quadruped_sim.sh [world.sdf]
# The default is the image's built-in empty.sdf. Keep this terminal open.
#
# MAZE_MODELS=<dir> monta um diretorio de modelos EXTERNO ao repositorio em
# /maze/models e aponta GZ_SIM_RESOURCE_PATH para ele. E o que quadruped_maze.sdf
# precisa: a malha do labirinto vem de github.com/cafemesa/ros_maze_worlds, que
# nao esta versionado aqui porque o package.xml de la declara
# <license>TODO</license>. Sem a variavel o mundo carrega e o labirinto nao
# aparece -- malha que nao resolve e uma linha de aviso, nao um erro fatal.

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
world_path="${1:-quadruped_empty.sdf}"
container_name="aquila-go2"

maze_mount=()
maze_env=()
if [ -n "${MAZE_MODELS:-}" ]; then
  if [ ! -d "${MAZE_MODELS}" ]; then
    echo "MAZE_MODELS=${MAZE_MODELS} nao e um diretorio." >&2
    exit 1
  fi
  maze_mount=(-v "$(cd "${MAZE_MODELS}" && pwd):/maze/models:ro")
  maze_env=(-e GZ_SIM_RESOURCE_PATH=/maze/models)
  echo "Modelos externos: ${MAZE_MODELS} -> /maze/models"
elif [ "${world_path}" = "quadruped_maze.sdf" ]; then
  echo "quadruped_maze.sdf exige MAZE_MODELS apontando para os modelos do" >&2
  echo "ros_maze_worlds; sem isso o labirinto nao carrega e o robo anda em" >&2
  echo "campo aberto sem nenhum erro. Veja docs/guides/cenarios/s6-labirinto.md" >&2
  exit 1
fi

if docker ps --format '{{.Names}}' | grep -qx "${container_name}"; then
  echo "Container ${container_name} já está em execução." >&2
  exit 1
fi

cd "${repo_dir}"
xhost +local:docker >/dev/null

docker run --rm --name "${container_name}" --network=host \
  -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e ROS_DOMAIN_ID=69 \
  -e GO2_WORLD="${world_path}" \
  "${maze_env[@]}" "${maze_mount[@]}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri:/dev/dri --group-add 992 \
  -v "${repo_dir}/ros2_ws/src:/proj/src:ro" \
  --entrypoint bash demo-sim:spike-go2 -c '
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
    # Um nome sem barra e resolvido no share de demo_simulation. Passar so o
    # nome do arquivo e o caso comum, e o Gazebo nao encontra o mundo la sozinho
    # -- ele nao herda o caminho de instalacao do colcon. Um argumento COM barra
    # e tratado como caminho explicito e usado como veio.
    case "${world}" in
      */*) : ;;
      *) world=/test/install/demo_simulation/share/demo_simulation/worlds/"${world}" ;;
    esac
    ros2 launch demo_simulation quadruped.launch.py gui:=true world:="${world}"
  '
