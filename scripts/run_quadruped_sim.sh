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
# MAZE_MODELS=<dir> monta um diretorio de modelos EXTERNO ao repositorio em
# /maze/models e aponta GZ_SIM_RESOURCE_PATH para ele. E o que os mundos
# quadruped_maze*.sdf precisam: a malha do labirinto vem de
# github.com/cafemesa/ros_maze_worlds, que nao esta versionado aqui porque o
# package.xml de la declara <license>TODO</license>. Sem a variavel o mundo
# carrega e o labirinto nao aparece -- malha que nao resolve e uma linha de
# aviso, nao um erro fatal.
#
# Aponte MAZE_MODELS para um caminho ESTAVEL, nao para /tmp: um reboot no meio
# de uma demonstracao apaga o clone e o labirinto volta a nao carregar, com o
# mesmo aviso discreto de sempre.
#
# GO2_SPAWN_YAW=<rad> gira o robo no nascimento. Normalmente NAO precisa ser
# passado: o mundo declara o proprio yaw numa linha `<!-- go2_spawn_yaw: N -->`
# e este script a le. O yaw pertence ao mundo porque e geometria dele -- num
# canto de labirinto o +x default e parede, e sair de la exige girar parado,
# que e o que este robo faz pior. A variavel de ambiente sobrepoe o arquivo.

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
    echo "MAZE_MODELS=${MAZE_MODELS} nao e um diretorio." >&2
    exit 1
  fi
  maze_mount=(-v "$(cd "${MAZE_MODELS}" && pwd):/maze/models:ro")
  maze_env=(-e GZ_SIM_RESOURCE_PATH=/maze/models)
  echo "Modelos externos: ${MAZE_MODELS} -> /maze/models"
else
  # Casa QUALQUER labirinto, nao so o maze10 do arquivo original. Um guard
  # preso a um nome literal deixa cada mundo novo reintroduzir a mesma falha
  # silenciosa: o Gazebo sobe, o labirinto nao aparece, e a corrida parece um
  # desvio perfeito em campo aberto.
  case "$(basename "${world_path}")" in
    quadruped_maze*.sdf)
      echo "$(basename "${world_path}") exige MAZE_MODELS apontando para os modelos" >&2
      echo "do ros_maze_worlds; sem isso o labirinto nao carrega e o robo anda em" >&2
      echo "campo aberto sem nenhum erro. Veja docs/guides/cenarios/s6-labirinto.md" >&2
      exit 1
      ;;
  esac
fi

# Yaw de nascimento: o mundo declara o dele; a variavel de ambiente sobrepoe.
# Um nome sem barra e resolvido na arvore do repositorio, que e onde o arquivo
# esta antes de o container instalar o share.
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
echo "Yaw de nascimento: ${spawn_yaw} rad"

# ==================== configuracao de DDS, para o modo hil ====================
#
# Sem isto o container do simulador usa o DEFAULT do CycloneDDS: multicast
# LIGADO e nenhum peer explicito. O modulo Aquila roda com AllowMulticast=false
# e peers explicitos, e as duas metades nunca se acham -- host->modulo porque o
# modulo nao escuta multicast, modulo->host porque um participante default nao
# fixa porta deterministica e nao ha porta para mirar. O sintoma e identico a
# firewall bloqueando, e nada em log nomeia DDS (guia-operacao.md secao 9.11).
#
# O arquivo renderizado NAO e versionado: `scripts/module.sh sync` o gera com o
# endereco do modulo e a interface do host. Se ele nao existir, o simulador sobe
# com o default -- que e o certo para modo learn puro, tudo numa maquina -- e
# avisa, em vez de falhar.
dds_env=()
dds_mount=()
host_dds="${repo_dir}/docker/cyclonedds/host.rendered.xml"
if [ -f "${host_dds}" ]; then
  dds_mount=(-v "${host_dds}:/cfg/cyclonedds.xml:ro")
  dds_env=(-e CYCLONEDDS_URI=file:///cfg/cyclonedds.xml)
  echo "DDS: host.rendered.xml (peers: $(grep -c '<Peer ' "${host_dds}"))"

  # A interface fixada no arquivo tem de ser a que ROTEIA ate o modulo hoje.
  # Divergir e falha silenciosa e recorrente: troca-se o cabo de Wi-Fi para
  # Ethernet, o kernel passa a rotear pela nova interface, e o
  # host.rendered.xml continua fixando a antiga. O CycloneDDS transmite num
  # endereco que o modulo nao consegue responder, a descoberta falha, e nada em
  # log nomeia interface nenhuma -- parece firewall.
  #
  # Os dois padroes casam o ELEMENTO, nao a palavra: o bloco de comentario do
  # host.xml discute `<NetworkInterface name="eth0"/>` e `name="lo"` em prosa, e
  # um sed frouxo fixa "eth0" e avisa errado -- foi o que aconteceu na primeira
  # versao desta guarda. Por isso a linha inteira e ancorada, aceitando so um
  # comentario XML depois do fecho.
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
      echo "     AVISO: o arquivo fixa ${pinned_iface}, mas a rota ate" >&2
      echo "     ${module_peer} sai por ${route_iface}. Rode: scripts/module.sh sync" >&2
    else
      echo "     interface ${pinned_iface}, que e a que roteia ate ${module_peer}"
    fi
  fi
  # Configuracao de DDS pela metade falha igual a firewall: se o simulador usa
  # peers explicitos e o Nav2 nativo deste host usa o default com multicast, os
  # dois nao se acham na mesma maquina. Quem roda algo nativo precisa da MESMA
  # config, e por isso a linha e impressa em vez de subentendida.
  echo "     Nos NATIVOS neste host precisam da mesma config. No outro terminal:"
  echo "     export CYCLONEDDS_URI=file://${host_dds}"
else
  echo "DDS: default do CycloneDDS (multicast). Serve para modo learn numa"
  echo "     maquina so. Para hil com o Aquila, rode antes: scripts/module.sh sync"
fi

if docker ps --format '{{.Names}}' | grep -qx "${container_name}"; then
  echo "Container ${container_name} já está em execução." >&2
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
    # Um nome sem barra e resolvido no share de demo_simulation. Passar so o
    # nome do arquivo e o caso comum, e o Gazebo nao encontra o mundo la sozinho
    # -- ele nao herda o caminho de instalacao do colcon. Um argumento COM barra
    # e tratado como caminho explicito e usado como veio.
    case "${world}" in
      */*) : ;;
      *) world=/test/install/demo_simulation/share/demo_simulation/worlds/"${world}" ;;
    esac
    ros2 launch demo_simulation quadruped.launch.py gui:=true \
        world:="${world}" yaw:="${GO2_SPAWN_YAW}" ${LAUNCH_ARGS}
  '
