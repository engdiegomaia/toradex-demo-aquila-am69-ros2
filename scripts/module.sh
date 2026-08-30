#!/usr/bin/env bash
set -euo pipefail

# Bring the Aquila AM69 module side of the demo up, from this workstation.
#
# Usage:
#   scripts/module.sh inventory   # what the module is: OS, docker, disk, links
#   scripts/module.sh sync        # push sources + rendered config to ~/demo
#   scripts/module.sh build       # build the arm64 images ON the module
#   scripts/module.sh up          # docker compose up -d  (nav + perception)
#     --force        bypass the /demo/cmd_vel collision guard (two publishers)
#     --force-spawn  bypass the spawn-distance guard (SLAM re-anchor risk) --
#                    means "the anchor doesn't matter here", never "I'll
#                    reset after"; the two flags are independent and
#                    combinable
#   scripts/module.sh down
#   scripts/module.sh status
#   scripts/module.sh verify      # DDS reachability + topic contract from the module
#   scripts/module.sh shell       # interactive shell in the tools container
#
# Configuration, in precedence order: environment, then docker/.env, then the
# defaults below. Nothing here hard-codes an address (CLAUDE.md conventions).
#
#   MODULE_HOST   ssh target                    (default aquila-am69.local)
#   MODULE_IP     module address on the LAN     (resolved from MODULE_HOST if unset)
#   HOST_IP       this workstation, as the module sees it
#   ROS_DOMAIN_ID DDS domain                    (default 69)
#   TAG/REGISTRY  image naming                  (default dev / local)
#   ROBOT_TYPE    coupled plant/Nav2 selection  (default quadruped)
#
# --- WHY THE IMAGES ARE BUILT ON THE MODULE, NOT WITH QEMU ------------------
#
# CLAUDE.md documents `docker buildx --platform linux/arm64` from the host. That
# path works and this script deliberately does not use it, for two measured
# reasons:
#
#   1. The AM69 is 8x Cortex-A72 with 31 GiB of RAM and >100 GiB free. A native
#      build there is minutes; the same build under QEMU on this workstation is
#      hours, because ros2_control and unitree_guide_controller are C++.
#   2. A QEMU build saturates the workstation CPU, and the workstation is where
#      the Gazebo gait experiments run. Those experiments measure WHEN the robot
#      falls. Stealing CPU from them corrupts the measurement rather than merely
#      slowing it down.
#
# What this trades away: buildx produces a multi-arch manifest, a native module
# build produces an arm64-only local image. That is correct for bring-up and
# wrong for distribution — when a registry enters the picture, the multi-arch
# path in CLAUDE.md is the one to use. Neither path measures performance
# (CLAUDE.md rule 5); only the real hardware does, and only at runtime.

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

# docker/.env is optional. The documented precedence is: environment, then
# docker/.env, then the defaults below.
#
# `set -a; source "${env_file}"` CANNOT implement that, and the comment that
# used to sit here claimed it did. `source` assigns unconditionally, so a value
# in .env silently replaces one given on the command line; by the time the
# ${VAR:-} guards below run, the variable is set either way and they cannot tell
# the two apart. The guards defend against UNSET, never against .env winning.
#
# This bit for real: a stale `HOST_IP=` left in .env by an earlier wired session
# overrode an explicit `HOST_IP=` on the command line, and both CycloneDDS
# configs were rendered for an interface with no carrier. That fails as silent
# non-discovery — the exact failure mode this script exists to prevent.
env_file="${MODULE_ENV_FILE:-${repo_dir}/docker/.env}"
if [[ -f "${env_file}" ]]; then
  while IFS= read -r line || [[ -n "${line}" ]]; do
    line="${line#"${line%%[![:space:]]*}"}"          # strip leading blanks
    if [[ -z "${line}" || "${line}" == '#'* ]]; then
      continue
    fi
    line="${line#export }"
    if [[ "${line}" != *=* ]]; then
      continue
    fi
    env_key="${line%%=*}"
    env_val="${line#*=}"
    if [[ ! "${env_key}" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]]; then
      continue
    fi
    # Presence, not non-empty content, defines an explicit override. HOST_IP=
    # is useful: it deliberately asks resolve_addresses() to derive the current
    # route instead of accepting a stale value from docker/.env. Testing `-n`
    # here made that empty override indistinguishable from an unset variable.
    if [[ -v ${env_key} ]]; then
      continue
    fi
    # Strip one layer of matching quotes, as `source` would have.
    if [[ "${env_val}" == \"*\" || "${env_val}" == \'*\' ]]; then
      env_val="${env_val:1:${#env_val}-2}"
    fi
    export "${env_key}=${env_val}"
  done < "${env_file}"
  unset line env_key env_val
fi

MODULE_HOST="${MODULE_HOST:-aquila-am69.local}"
MODULE_USER="${MODULE_USER:-torizon}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-69}"
TAG="${TAG:-dev}"
REGISTRY="${REGISTRY:-local}"
ROBOT_TYPE="${ROBOT_TYPE:-quadruped}"
remote_dir="${REMOTE_DIR:-/home/${MODULE_USER}/demo}"
ssh_target="${MODULE_USER}@${MODULE_HOST}"

# Applies to every ssh/rsync call. BatchMode makes a missing key fail loudly
# instead of hanging on a password prompt inside a script.
SSH_OPTS=(-o BatchMode=yes -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10)

die() { printf '\n[module.sh] ERRO: %s\n' "$*" >&2; exit 1; }
say() { printf '\n[module.sh] %s\n' "$*"; }

remote() { ssh "${SSH_OPTS[@]}" "${ssh_target}" "$@"; }

# Path to the rendered host-side CycloneDDS config; set by render_host_config.
host_cfg=""

# --- Address discovery -----------------------------------------------------
# Both addresses are needed for the CycloneDDS peer lists and both are resolved,
# never guessed. A wrong address here is the failure mode this whole milestone
# keeps paying for: discovery silently finds nothing and no log names the cause.
resolve_addresses() {
  if [[ -z "${MODULE_IP:-}" ]]; then
    MODULE_IP="$(getent hosts "${MODULE_HOST}" | awk '{print $1}' | grep -E '^[0-9.]+$' | head -1 || true)"
    [[ -n "${MODULE_IP}" ]] || die "nao resolvi MODULE_IP para ${MODULE_HOST}. Defina MODULE_IP=..."
  fi

  # The route to the module decides which of this machine's addresses the module
  # can actually reply to. Taking the first address of the first UP interface
  # instead would pick docker0 or tailscale0 here and produce a peer entry that
  # is reachable from nothing.
  if [[ -z "${HOST_IP:-}" ]]; then
    HOST_IP="$(ip route get "${MODULE_IP}" 2>/dev/null | grep -oP 'src \K[0-9.]+' | head -1 || true)"
    [[ -n "${HOST_IP}" ]] || die "nao determinei HOST_IP pela rota ate ${MODULE_IP}. Defina HOST_IP=..."
  fi
  say "modulo ${MODULE_IP} | host ${HOST_IP} | dominio ${ROS_DOMAIN_ID} | tag ${TAG}"
}

# --- host-side CycloneDDS config -------------------------------------------
# The module half of the link is not enough, and this was proved the hard way.
#
# With the module correctly configured (multicast off, peers 127.0.0.1 and the
# host) and a heartbeat publisher demonstrably running on it, the host saw
# NOTHING. Neither direction discovered the other, for two different reasons at
# once:
#
#   host -> module: the host default config announces over MULTICAST, and the
#     module has AllowMulticast=false, so it never listens for those.
#   module -> host: the module sends unicast SPDP to the host's RTPS discovery
#     ports, but a default CycloneDDS participant does not bind a deterministic
#     one — it takes an ephemeral port and relies on multicast to be found. There
#     is no port for the module to aim at.
#
# So the host side needs a config that MATCHES: multicast off, ParticipantIndex
# auto (deterministic ports), and the module listed as a peer. With that in
# place the host immediately saw /demo/odom, /demo/scan, /clock and the module's
# own topics.
#
# Rendered, not committed, for the same reason as module.xml: no addresses in
# git. Written next to the template so `hil` mode can point compose at it later.
render_host_config() {
  local iface
  iface="$(ip -o -4 addr show | awk -v ip="${HOST_IP}" '$4 ~ "^"ip"/" {print $2}' | head -1)"
  [[ -n "${iface}" ]] || die "nao identifiquei a interface do host que carrega ${HOST_IP}"

  host_cfg="${repo_dir}/docker/cyclonedds/host.rendered.xml"

  say "renderizando cyclonedds/host.rendered.xml (iface ${iface}, peer do modulo ${MODULE_IP})"
  awk -v ip="${MODULE_IP}" -v iface="${iface}" '
    /^[[:space:]]*<NetworkInterface name="lo"[^>]*\/>[[:space:]]*$/ {
      # In LEARN, loopback is intentionally the preferred interface.  In HIL
      # that preference makes Cyclone bind external unicast writes to lo and
      # host -> Aquila fails with ddsi_udp_conn_write retcode -3.  Keep lo for
      # sibling containers, but prefer the routed interface below.
      printf "        <NetworkInterface name=\"lo\" priority=\"default\" multicast=\"true\"/>  <!-- HIL: secundaria -->\n"
      next
    }
    /^[[:space:]]*<NetworkInterface autodetermine="true"[^>]*\/>[[:space:]]*$/ {
      printf "        <NetworkInterface name=\"%s\" priority=\"10\"/>  <!-- HIL: preferida, fixada por scripts/module.sh -->\n", iface
      next
    }
    /^[[:space:]]*<\/Peers>[[:space:]]*$/ && !done {
      printf "        <Peer address=\"%s\"/>  <!-- modulo Aquila, injetado por scripts/module.sh -->\n", ip
      done = 1
    }
    { print }
  ' "${repo_dir}/docker/cyclonedds/host.xml" > "${host_cfg}"

  grep -q "<Peer address=\"${MODULE_IP}\"/>" "${host_cfg}" \
    || die "renderizacao do host.xml nao inseriu o peer do modulo"
  grep -q '<Peer address="127.0.0.1"/>' "${host_cfg}" \
    || die "renderizacao do host.xml perdeu o peer localhost, que e load-bearing"
  grep -q '<NetworkInterface name="lo" priority="default" multicast="true"/>' "${host_cfg}" \
    || die "host.rendered.xml nao rebaixou loopback no modo HIL"
  grep -q "<NetworkInterface name=\"${iface}\" priority=\"10\"/>" "${host_cfg}" \
    || die "host.rendered.xml nao priorizou a interface roteada no modo HIL"
  ! grep -q '<NetworkInterface autodetermine' "${host_cfg}" \
    || die "host.rendered.xml ainda usa autodetermine no elemento NetworkInterface"
  python3 -c "import xml.dom.minidom; xml.dom.minidom.parse('${host_cfg}')" \
    || die "host.rendered.xml nao e XML valido"
}

cmd_inventory() {
  remote 'bash -s' <<'EOS'
echo "--- OS ---";        grep PRETTY_NAME /etc/os-release
echo "--- ostree ---";    ostree admin status | head -3
echo "--- CPU/MEM ---";   printf 'nproc=%s\n' "$(nproc)"; free -h | sed -n 2p
echo "--- disco ---";     df -h /var/lib/docker 2>/dev/null || df -h /var
echo "--- links UP ---";  ip -br addr | grep -v ' DOWN'
echo "--- docker ---";    docker version --format '{{.Server.Version}} {{.Server.Arch}}'; docker compose version --short
echo "--- imagens ---";   docker images --format '{{.Repository}}:{{.Tag}} {{.Size}}' | grep demo-aquila || echo "(nenhuma imagem demo-aquila)"
echo "--- servicos ---";  docker ps --format '{{.Names}} {{.Status}}'
EOS
}

# --- sync ------------------------------------------------------------------
# Only what the arm64 Dockerfiles actually consume is pushed. The build context
# on the module mirrors the repo root layout, because base/Dockerfile does
# `COPY ros2_ws/src` and `COPY docker/entrypoint.sh` — paths relative to the
# repo root, not to docker/.
cmd_sync() {
  resolve_addresses

  remote "mkdir -p ${remote_dir}/ros2_ws ${remote_dir}/docker ${remote_dir}/cyclonedds"

  say "enviando ros2_ws/src"
  rsync -az --delete -e "ssh ${SSH_OPTS[*]}" \
    --exclude '__pycache__' --exclude '*.pyc' \
    ros2_ws/src "${ssh_target}:${remote_dir}/ros2_ws/"

  say "enviando Dockerfiles e entrypoint"
  # No sim/ and no viz/: those are OGRE 2 and never leave the x86 host
  # (CLAUDE.md rule 1). Their absence on the module is the guard.
  rsync -az -e "ssh ${SSH_OPTS[*]}" --delete \
    --include 'base/' --include 'base/**' \
    --include 'nav/' --include 'nav/**' \
    --include 'perception/' --include 'perception/**' \
    --include 'tools/' --include 'tools/**' \
    --include 'entrypoint.sh' \
    --exclude '*' \
    docker/ "${ssh_target}:${remote_dir}/docker/"

  say "enviando compose.module.yml"
  rsync -az -e "ssh ${SSH_OPTS[*]}" \
    docker/compose.module.yml "${ssh_target}:${remote_dir}/compose.module.yml"

  # --- rendered CycloneDDS config ---------------------------------------
  # The committed module.xml lists 127.0.0.1 only. The host peer is inserted
  # here rather than committed, so no address lives in git. See the long comment
  # in docker/cyclonedds/module.xml.
  # The interface is pinned, not autodetermined. Measured on this module,
  # `ip -br addr` reports ethernet0 AND a docker bridge (br-*, 172.18.0.0/16)
  # UP at the same time, because Torizon's own easy-pairing stack runs in
  # compose. autodetermine ranks interfaces and can pick the bridge, at which
  # point CycloneDDS transmits on an address the host cannot route to and
  # discovery fails with nothing in any log naming an interface.
  #
  # Detected on the module from MODULE_IP rather than hard-coded, so a different
  # board or a move to wlan0 does not silently keep an ethernet0 that is gone.
  local iface
  iface="$(remote "ip -o -4 addr show | awk '\$4 ~ /^${MODULE_IP}\// {print \$2}'" | head -1)"
  [[ -n "${iface}" ]] || die "nao identifiquei a interface do modulo que carrega ${MODULE_IP}"

  say "renderizando cyclonedds/module.xml (iface ${iface}, peer do host ${HOST_IP})"
  local rendered
  rendered="$(mktemp)"
  # shellcheck disable=SC2064
  trap "rm -f '${rendered}'" RETURN
  # Both patterns are anchored to a WHOLE line, and the peer injection fires
  # once. An unanchored /<\/Peers>/ also matched a mention of the closing tag
  # inside the file's own comment block, which injected the host peer into the
  # middle of a comment and produced malformed XML. The XML validation below is
  # what caught it, which is why it is not optional.
  awk -v ip="${HOST_IP}" -v iface="${iface}" '
    /^[[:space:]]*<NetworkInterface autodetermine="true"[^>]*\/>[[:space:]]*$/ {
      printf "        <NetworkInterface name=\"%s\" priority=\"default\"/>  <!-- fixado por scripts/module.sh -->\n", iface
      next
    }
    /^[[:space:]]*<\/Peers>[[:space:]]*$/ && !done {
      printf "        <Peer address=\"%s\"/>  <!-- host x86, injetado por scripts/module.sh -->\n", ip
      done = 1
    }
    { print }
  ' docker/cyclonedds/module.xml > "${rendered}"

  grep -q "<Peer address=\"${HOST_IP}\"/>" "${rendered}" \
    || die "renderizacao do module.xml nao inseriu o peer do host"
  grep -q '<Peer address="127.0.0.1"/>' "${rendered}" \
    || die "renderizacao do module.xml perdeu o peer localhost, que e load-bearing"
  grep -q "<NetworkInterface name=\"${iface}\"" "${rendered}" \
    || die "renderizacao do module.xml nao fixou a interface ${iface}"
  # Matches the ELEMENT, not the word: module.xml's comment block discusses
  # autodetermine at length, so a bare `grep autodetermine` always trips.
  ! grep -q '<NetworkInterface autodetermine' "${rendered}" \
    || die "module.xml renderizado ainda usa autodetermine no elemento NetworkInterface"
  python3 -c "import xml.dom.minidom,sys; xml.dom.minidom.parse('${rendered}')" \
    || die "module.xml renderizado nao e XML valido"

  rsync -az -e "ssh ${SSH_OPTS[*]}" "${rendered}" "${ssh_target}:${remote_dir}/cyclonedds/module.xml"

  render_host_config

  # --- .env on the module ------------------------------------------------
  # Written here, not rsync'd, because docker/.env is workstation-local and the
  # module needs its own values. compose.module.yml reads all four.
  say "escrevendo ${remote_dir}/.env"
  remote "cat > ${remote_dir}/.env" <<EOF
# Gerado por scripts/module.sh sync. Nao editar a mao: o proximo sync sobrescreve.
ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
HOST_IP=${HOST_IP}
MODULE_IP=${MODULE_IP}
REGISTRY=${REGISTRY}
TAG=${TAG}
ROBOT_TYPE=${ROBOT_TYPE}
EOF

  say "sync concluido em ${ssh_target}:${remote_dir}"
}

# --- build -----------------------------------------------------------------
# base first, then the three role images that inherit it. Sequential on purpose:
# the role builds FROM base, so there is nothing to parallelise, and a serial log
# is readable when one of them fails.
cmd_build() {
  resolve_addresses
  remote "test -f ${remote_dir}/docker/base/Dockerfile" \
    || die "fontes ausentes no modulo. Rode: scripts/module.sh sync"

  # Passed to every build. See docker/base/Dockerfile for why both are required
  # and why neither alone is enough.
  local skip_keys="gz_sim_vendor gz_plugin_vendor"
  local ignore_pkgs="gz_quadruped_hardware"

  say "build arm64 NATIVO no modulo (base -> nav, perception, tools)"
  remote "bash -s" <<EOS
set -euo pipefail
cd ${remote_dir}

echo "=== base ==="
docker build -f docker/base/Dockerfile \
  --build-arg SKIP_KEYS_EXTRA="${skip_keys}" \
  --build-arg COLCON_IGNORE_PACKAGES="${ignore_pkgs}" \
  -t ${REGISTRY}/demo-aquila-base:${TAG} .

for role in nav perception tools; do
  echo "=== \${role} ==="
  docker build -f docker/\${role}/Dockerfile \
    --build-arg REGISTRY=${REGISTRY} \
    --build-arg TAG=${TAG} \
    --build-arg COLCON_IGNORE_PACKAGES="${ignore_pkgs}" \
    -t ${REGISTRY}/demo-aquila-\${role}:${TAG} .
done

echo "=== resultado ==="
docker images --format '{{.Repository}}:{{.Tag}}\t{{.Size}}' | grep demo-aquila
EOS

  # Rule 1 is checked, not trusted. A Gazebo library inside an image that ships
  # to the AM69 is invisible until runtime, so the build is not done until this
  # passes.
  # Rule 1 says no desktop-OpenGL software on the target. What that forbids is
  # the RENDERING stack: OGRE, gz-rendering, gz-sim, gz-gui, RViz.
  #
  # It does NOT forbid every package whose name starts with gz-. Measured on
  # this build, rosdep pulls gz-cmake-vendor, gz-math-vendor, gz-tools-vendor
  # and gz-utils-vendor through sdformat-vendor: build tooling and math headers,
  # 42 MB total, no GPU contact. Grepping for a bare "gz" would flag those and
  # send the next person hunting a violation that is not there — while a grep
  # for "gazebo" alone would MISS gz-sim, because Gazebo Harmonic dropped the
  # name.
  #
  # The pattern is narrow for a reason found the hard way. A case-insensitive
  # /ogre/ reported demo-aquila-nav as violating rule 1, on:
  #
  #     libnav2_progress_checker_selector_bt_node.so
  #     libpose_progress_checker.so
  #     libsimple_progress_checker.so
  #
  # "pr-OGRE-ss". Three Nav2 progress checkers, no OpenGL anywhere near them.
  # Real OGRE ships as libOgreMain / libOgreOverlay (capital O) or libogre-next-*,
  # so the OGRE half is CASE-SENSITIVE and the ogre-next half is hyphen-anchored.
  # gz-sim and gz-gui take a trailing digit (libgz-sim8.so) so they cannot match
  # gz-math or gz-utils.
  #
  # Do not "simplify" this to grep -i ogre.
  #
  # This runs because rule 1 cannot be checked by reading the Dockerfile: the
  # build succeeds either way and the violation only surfaces at runtime on the
  # module, as a library that wants an OpenGL the AM69 does not have.
  say "verificando regra 1: nenhuma stack de renderizacao nas imagens do modulo"
  remote "bash -s" <<EOS
set -uo pipefail
fail=0
for role in base nav perception tools; do
  found=\$(docker run --rm --entrypoint sh ${REGISTRY}/demo-aquila-\${role}:${TAG} -c \
    'ls /opt/ros/jazzy/lib /usr/lib/aarch64-linux-gnu 2>/dev/null \
      | grep -E "libOgre|ogre-next|gz-rendering|gz-sim[0-9]|gz-gui[0-9]|rviz" | head -5' || true)
  if [ -n "\$found" ]; then
    echo "REGRA 1 VIOLADA: demo-aquila-\${role} contem:"
    echo "\$found" | sed 's/^/    /'
    fail=1
  else
    echo "ok: demo-aquila-\${role} sem stack de renderizacao"
  fi
  # Informativo, nao e falha: matematica e build tooling do Gazebo entram via
  # sdformat e sao CPU puro.
  docker run --rm --entrypoint sh ${REGISTRY}/demo-aquila-\${role}:${TAG} -c \
    'ls /opt/ros/jazzy/lib 2>/dev/null | grep -ioE "gz-(cmake|math|tools|utils)" | sort -u | tr "\n" " "' 2>/dev/null \
    | sed 's/^/    (gz vendor sem GPU: /; s/ *\$/)/' || true
  echo
done
exit \$fail
EOS
}

# --- up --------------------------------------------------------------------
# Starts nav and perception on the module. nav means Nav2, and Nav2 PUBLISHES
# /demo/cmd_vel.
#
# If a Gazebo simulation is already running on this workstation on the same
# ROS_DOMAIN_ID, that makes two publishers on the topic that drives the robot.
# The simulated robot then moves on commands nobody sent, and nothing in any log
# on either machine says why — the topic is valid, both publishers are healthy,
# and DDS is doing exactly what it was told. Any gait or timing experiment
# running on the host is silently corrupted rather than interrupted.
#
# So this is checked, and refused by default. --force is for when the collision
# is understood and intended.
cmd_up() {
  resolve_addresses

  # Dois guards independentes, dois bypasses independentes. Cada um so
  # significa "este risco especifico nao se aplica agora" -- nunca "vou
  # corrigir depois". Em particular --force-spawn NUNCA deve ser lido como
  # "vou resetar o sim em seguida": resetar DEPOIS de recriar 'nav' e
  # exatamente a sequencia que ancora o slam_toolbox na pose errada (ver
  # check_robot_near_spawn_before_nav_restart abaixo). Um --force generico que
  # desativasse os dois guards de uma vez esconderia essa distincao.
  local has_force=0
  local has_force_spawn=0
  local arg
  for arg in "$@"; do
    case "${arg}" in
      --force) has_force=1 ;;
      --force-spawn) has_force_spawn=1 ;;
    esac
  done

  # A GUARDA CERTA E "quem PUBLICA /demo/cmd_vel neste host", nao "a simulacao
  # esta rodando".
  #
  # A versao anterior recusava quando havia container de simulacao no host, e
  # isso torna o modo hil impossivel de subir sem --force: em hil o simulador
  # TEM de estar no host. O simulador nao e publicador de /demo/cmd_vel -- ele
  # ASSINA. Quem publica no lado do host e:
  #
  #   cmd_vel_si_to_stick  do nav_quadruped.launch.py NATIVO (o conflito real,
  #                        porque e o mesmo no que sobe no modulo)
  #   demo_routine         a coreografia de malha aberta
  #
  # `pgrep -x` casa o NOME do processo, entao nao casa com a linha de comando
  # deste script. Os nomes vem truncados em 15 caracteres, limite de `comm` no
  # Linux -- dai `cmd_vel_si_to_s`.
  local host_pubs=''
  local proc
  for proc in cmd_vel_si_to_s demo_routine; do
    if pgrep -x "${proc}" >/dev/null 2>&1; then
      host_pubs+="  ${proc} (pid $(pgrep -x "${proc}" | tr '\n' ' '))"$'\n'
    fi
  done

  if [[ -n "${host_pubs}" && "${has_force}" -eq 0 ]]; then
    printf '\n[module.sh] RECUSADO: publicador de /demo/cmd_vel ativo neste host:\n%s' "${host_pubs}" >&2
    cat >&2 <<EOF
Subir 'nav' no modulo agora coloca um SEGUNDO publisher em /demo/cmd_vel no
dominio ${ROS_DOMAIN_ID}. Dois publicadores no mesmo topico nao geram erro: o
twist_to_inputs obedece a ultima mensagem que chegou e o robo anda em espasmos,
alternando entre as duas origens a 20 Hz. Nenhum log identifica a causa.

Escolha uma saida:
  1. Derrube o Nav2 nativo do host e repita. Matar o 'ros2 launch' nao basta --
     ele orfana os filhos (guia-operacao.md secao 9.12):
       pkill -9 -x cmd_vel_si_to_s; pkill -9 -x odom_tf
       for n in controller_serv bt_navigator planner_server behavior_server \\
                route_server smoother_server waypoint_follow opennav_docking \\
                collision_monit velocity_smooth lifecycle_manag; do
         pkill -9 -x "\$n"; done
  2. Suba somente perception, que nao publica cmd_vel:
       ssh ${ssh_target} 'cd ${remote_dir} && docker compose -f compose.module.yml up -d perception'
  3. Use um dominio separado para o modulo:
       ROS_DOMAIN_ID=70 scripts/module.sh sync && ROS_DOMAIN_ID=70 scripts/module.sh up
     (perde a ligacao com o host, serve para bring-up isolado)
  4. Force, sabendo do conflito:
       scripts/module.sh up --force
EOF
    exit 1
  fi

  # Em hil a simulacao no host e ESPERADA, e sem ela o Nav2 do modulo nao tem
  # /clock nem sensores: sobe, fica em espera, e parece travado. Avisar e util;
  # recusar seria errado.
  if ! docker ps --format '{{.Names}} {{.Image}}' 2>/dev/null \
      | grep -qiE 'aquila-go2|demo-aquila-sim|demo-sim'; then
    say "AVISO: nenhuma simulacao neste host. Em hil o modulo depende do /clock"
    say "e dos sensores que o Gazebo publica; sem isso o Nav2 fica esperando."
  fi

  check_robot_near_spawn_before_nav_restart "${has_force_spawn}"

  # The rendered CycloneDDS file is a bind mount whose PATH does not change
  # between syncs. Compose therefore considers an old container up to date even
  # when the peer/interface inside that file changed. CycloneDDS reads the XML
  # only at process start, so a normal `up -d` leaves the stale interface alive.
  remote "cd ${remote_dir} && docker compose -f compose.module.yml up -d --force-recreate"
  cmd_status
}

# Recreating 'nav' restarts slam_toolbox, which anchors its first scan(s) at
# whatever pose the robot has RIGHT NOW. If that pose is far from spawn --
# e.g. wherever a previous exploration round left the robot -- and
# /demo/sim/reset is only called AFTER this restart, slam_toolbox anchors a
# map patch at the stale pose before the reset teleport happens, leaving that
# patch permanently disconnected from spawn. Symptom: the next exploration
# round dies in well under a minute with ComputePathToPose returning
# error_code=208 (NO_VALID_PATH) on every candidate, and /map's known cells
# sit far from (0,0) even though `map`->`odom` reports identity -- the TF is
# fine, the map itself was just built in the wrong place. Reproduced twice
# (ML3.5 F5 R10 and R12's first attempt, 29-30/08/2026); see
# docs/results/ml35-f5-exploration-r12.md.
#
# So: refuse by default when the robot is not near spawn, same shape as the
# cmd_vel guard above. --force-spawn is ONLY for "the SLAM anchor does not
# matter for this operation" (e.g. this 'up' is not going to precede an
# exploration round at all) -- it is NEVER "I will reset the sim afterward".
# Resetting AFTER this restart is precisely the sequence that produces the
# bug: slam_toolbox has already anchored on its first scan by the time the
# reset's teleport happens, and the teleport does not undo an existing
# anchor. The only sequence that is actually safe for exploration is reset ->
# confirm odom at spawn -> up.
# Reads /demo/odom's x/y position from the host, once. Prints "x y" (space
# separated) on success, or nothing at all if unavailable (ROS not installed
# on this host, sim not up yet, or the read timed out) -- callers must treat
# "nothing" as "let it through", never as a refusal: this guard must never be
# the reason 'up' fails for a caller it has nothing useful to say to. Split
# out from check_robot_near_spawn_before_nav_restart so tests can stub this
# one function instead of needing a live /demo/odom.
_read_current_odom_xy() {
  if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    return 0
  fi

  [[ -f "${host_cfg:-}" ]] || render_host_config

  # `ros2 topic echo --once --field ...` prints the value on its own line,
  # THEN a bare `---` YAML end-of-document marker on the next -- `head -1`
  # keeps only the value. Without this, the raw two-line string fed straight
  # into a python expression below is a syntax error, and under `set -e` a
  # failing command substitution aborts the whole script silently (found by
  # actually running this against the live module, not by inspection).
  local pos_x pos_y
  pos_x="$(CYCLONEDDS_URI="file://${host_cfg}" bash -c '
    source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID='"${ROS_DOMAIN_ID}"'
    timeout 5 ros2 topic echo --once --field pose.pose.position.x /demo/odom 2>/dev/null
  ' | head -1)"
  pos_y="$(CYCLONEDDS_URI="file://${host_cfg}" bash -c '
    source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID='"${ROS_DOMAIN_ID}"'
    timeout 5 ros2 topic echo --once --field pose.pose.position.y /demo/odom 2>/dev/null
  ' | head -1)"

  [[ -z "${pos_x}" || -z "${pos_y}" ]] && return 0
  printf '%s %s\n' "${pos_x}" "${pos_y}"
}

# Pure decision given a position: "far <distance>" or "near <distance>", or
# nothing if the inputs cannot be parsed as numbers (a malformed odom read) --
# same "nothing means let it through" contract as _read_current_odom_xy.
# 1.0 m: comfortably above spawn noise (a few cm in every round observed this
# session), comfortably below "a previous round actually explored". `|| true`
# keeps a malformed value (or python itself missing) from aborting the whole
# script under `set -e`.
_spawn_distance_verdict() {
  local pos_x="$1" pos_y="$2"
  python3 -c "
import math
distance = math.hypot(${pos_x}, ${pos_y})
print('far' if distance > 1.0 else 'near', f'{distance:.2f}')
" 2>/dev/null || true
}

check_robot_near_spawn_before_nav_restart() {
  local force_spawn="$1"

  local odom
  odom="$(_read_current_odom_xy)"
  # No /demo/odom at all (sim not up yet, first-ever bring-up): nothing to
  # compare against, and nothing this guard can usefully say. Let it through.
  [[ -z "${odom}" ]] && return 0
  local pos_x pos_y
  read -r pos_x pos_y <<<"${odom}"

  local verdict
  verdict="$(_spawn_distance_verdict "${pos_x}" "${pos_y}")"
  if [[ -z "${verdict}" ]]; then
    return 0
  fi
  local far_or_near distance
  read -r far_or_near distance <<<"${verdict}"

  if [[ "${far_or_near}" == "far" ]] && [[ "${force_spawn}" -eq 0 ]]; then
    printf '\n[module.sh] RECUSADO: robo a %s m do spawn (x=%s y=%s).\n' \
      "${distance}" "${pos_x}" "${pos_y}" >&2
    cat >&2 <<EOF
Recriar 'nav' agora ancora o slam_toolbox nessa pose, nao no spawn. Se um
/demo/sim/reset rodar so DEPOIS deste restart, o mapa nasce com um pedaco
orfao longe de (0,0) e a proxima exploracao morre em menos de um minuto
com ComputePathToPose recusando tudo (error_code=208). Reproduzido em
ML3.5 F5 R10 e R12 -- docs/results/ml35-f5-exploration-r12.md.

--force-spawn NAO significa "vou resetar o sim depois" -- resetar DEPOIS
deste restart e exatamente o bug acima (o slam_toolbox ja ancorou na
primeira varredura antes do teleporte acontecer). So use --force-spawn
quando a ancora do SLAM for irrelevante para esta operacao, tipicamente
porque este 'up' nao antecede uma rodada de exploracao.

Escolha uma saida:
  1. (recomendado para exploracao) Resete o sim primeiro, confirme o robo
     perto do spawn, so entao suba:
       ros2 service call /demo/sim/reset std_srvs/srv/Trigger '{}'
       scripts/module.sh up
  2. Force, sabendo que a ancora do SLAM nao importa agora:
       scripts/module.sh up --force-spawn
EOF
    exit 1
  fi
}

cmd_down() {
  remote "cd ${remote_dir} && docker compose -f compose.module.yml --profile tools down"
}

cmd_status() {
  remote "cd ${remote_dir} && docker compose -f compose.module.yml ps; echo '--- peers ---'; grep -o '<Peer[^/]*/>' cyclonedds/module.xml"
}

# --- verify ----------------------------------------------------------------
# Three separate questions, in this order, because they fail differently:
#   1. can the two machines exchange UDP on the DDS discovery ports at all
#   2. does the module SEE what the host publishes
#   3. does the host RECEIVE what the module publishes
# Skipping (1) and debugging (2) is how a firewall gets mistaken for a DDS bug.
# Stopping at (2) is how a one-way link gets reported as working.
cmd_verify() {
  resolve_addresses
  local verify_failed=0

  # RTPS unicast discovery ports for domain N: 7400 + 250*N + 10 + 2*index.
  #
  # Index 0 is the canonical one and is frequently ALREADY BOUND, by a live
  # CycloneDDS participant on this host. Measured: with the Go2 simulation
  # running on domain 69, binding 24660 fails with EADDRINUSE and the original
  # probe died on a traceback before testing anything.
  #
  # That is not a condition to paper over. A busy port means DDS is UP locally on
  # this domain, which is worth saying out loud, and the firewall question is
  # answered just as well by the next free index in the same range.
  local base_port=$((7400 + 250 * ROS_DOMAIN_ID + 10))
  local port busy=""
  port="$(python3 - "${base_port}" <<'PYPORT'
import socket, sys
base = int(sys.argv[1])
for idx in range(10):
    cand = base + 2 * idx
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("0.0.0.0", cand))
    except OSError:
        s.close()
        continue
    s.close()
    print(cand)
    break
else:
    print("")
PYPORT
)" || true

  if [[ -z "${port}" ]]; then
    printf '    nenhuma porta livre em %s..%s: alcance UDP nao verificado.\n' \
      "${base_port}" "$((base_port + 18))"
    verify_failed=1
  else
    [[ "${port}" == "${base_port}" ]] || busy=" (${base_port} ocupada: DDS vivo neste host)"
    say "1/4 alcance UDP no dominio ${ROS_DOMAIN_ID}, porta ${port}${busy}"

    python3 - "${port}" <<'PYLISTEN' &
import socket, sys, time
p = int(sys.argv[1])
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    s.bind(("0.0.0.0", p))
except OSError as exc:
    print(f"    modulo -> host: nao consegui escutar em {p}: {exc}")
    raise SystemExit(1)
# The payload IS checked. These are live RTPS discovery ports: with a simulation
# running on the same domain, the first datagram to arrive is often real SPDP
# traffic from another participant, and accepting it would report "OK" for a
# port the module never reached. Measured once as "de <HOST_IP>" — the
# host's own address — on a probe that was supposed to prove the module could
# reach us.
deadline = time.monotonic() + 15
while True:
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        print("    modulo -> host: so chegou trafego RTPS de terceiros, probe nao confirmado.")
        raise SystemExit(1)
    s.settimeout(remaining)
    try:
        data, addr = s.recvfrom(2048)
    except socket.timeout:
        print("    modulo -> host: TIMEOUT. Verifique firewall do host nesta porta UDP.")
        raise SystemExit(1)
    if data == b"dds-probe":
        print(f"    modulo -> host: OK (de {addr[0]})")
        raise SystemExit(0)
PYLISTEN
    local listener=$!
    sleep 2
    remote "python3 -c \"import socket;s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);[s.sendto(b'dds-probe',('${HOST_IP}',${port})) for _ in range(3)]\"" || true
    if ! wait "${listener}"; then
      verify_failed=1
    fi
  fi

  say "2/4 contrato de topicos visto de dentro do modulo"
  [[ -f "${host_cfg:-}" ]] || render_host_config
  remote "cd ${remote_dir} && docker compose -f compose.module.yml --profile tools up -d tools" >/dev/null
  # Discovery over unicast peers is not instant; a list taken immediately after
  # the container starts is empty for reasons that have nothing to do with the
  # configuration.
  sleep 8
  # /usr/local/bin/entrypoint.sh IS REQUIRED HERE. `docker compose exec` does not
  # run the image ENTRYPOINT, so ROS is not sourced and `ros2` is not even on
  # PATH — measured: `which ros2` returns nothing, PATH is the bare system one.
  #
  # This failed silently and convincingly. `ros2 topic list 2>/dev/null | grep
  # /demo/ || echo "(nenhum topico visivel)"` prints the same "no topics" line
  # whether DDS found nothing or the ros2 binary was never found, because 2>/dev/null
  # swallowed "command not found". Two verification steps reported a discovery
  # failure that did not exist.
  #
  # So: no 2>/dev/null on the ros2 calls, and the entrypoint sources the underlay
  # and the /ws/install overlay before anything runs.
  local seen seen_status
  set +e
  seen="$(remote "cd ${remote_dir} && docker compose -f compose.module.yml exec -T tools \
    /usr/local/bin/entrypoint.sh bash -c '
      ros2 node list
      # /clock IS NOT UNDER /demo/, AND A BARE grep /demo/ DROPS IT.
      #
      # The required-topic loop below asks for /clock. Collecting with the
      # /demo/ filter alone removed it before that loop ever ran, so verify
      # printed AUSENTE: /clock and returned failure for a link that was
      # healthy -- measured 616 Hz on /clock from inside the module at the same
      # moment the check called it missing. That is the false negative this
      # script exists to prevent, pointed at its own reader.
      #
      # Keep the anchor: every topic named in that loop must be matched here.
      #
      # Two -e patterns and not an -E alternation on purpose. This whole block
      # is nested inside remote "...", a double-quoted string, so a quoted
      # regex or a parenthesised group would be eaten by the outer shell before
      # ssh ever sees it. Plain -e arguments need no quoting at all.
      ros2 topic list | grep -e /clock -e /demo/
    '" | grep -v 'ROS_LOCALHOST_ONLY\|automatic_discovery_range')"
  seen_status=$?
  set -e

  if [[ ${seen_status} -eq 0 && -n "${seen}" ]]; then
    printf '%s\n' "${seen}" | sed 's/^/    /'
    local required_topic
    for required_topic in /clock /demo/odom /demo/scan /demo/camera/image_raw; do
      if ! grep -qx "${required_topic}" <<<"${seen}"; then
        printf '    AUSENTE: %s\n' "${required_topic}"
        verify_failed=1
      fi
    done
  else
    printf '    o modulo nao ve nada publicado no dominio %s.\n' "${ROS_DOMAIN_ID}"
    verify_failed=1
    # An empty list here is EXPECTED while the host side runs without the
    # rendered config, and saying only "no topics" sends the reader hunting the
    # module. The producer is what is misconfigured, not the consumer.
    local host_sim
    host_sim="$(docker ps --format '{{.Names}}' 2>/dev/null | grep -iE 'aquila-go2|demo-sim' || true)"
    if [[ -n "${host_sim}" ]]; then
      cat <<EOF
    Ha simulacao rodando neste host (${host_sim}), e ela NAO usa o config
    renderizado: sem CYCLONEDDS_URI o CycloneDDS default anuncia por multicast,
    que o modulo ignora (AllowMulticast=false). O modulo nao tem como descobri-la.

    Para o modulo ver a simulacao, o produtor do host precisa subir com:
      -e CYCLONEDDS_URI=file:///cfg/cyclonedds.xml
      -v ${host_cfg:-docker/cyclonedds/host.rendered.xml}:/cfg/cyclonedds.xml:ro
    ou, nativo no host, exportando CYCLONEDDS_URI para o mesmo arquivo.

    A etapa 3 abaixo mede a outra direcao e nao depende disso.
EOF
    else
      printf '    Nenhuma simulacao ativa neste host: nao ha o que descobrir.\n'
    fi
  fi

  # Listing /clock proves discovery metadata only.  It passed while every
  # host-side Cyclone participant logged ddsi_udp_conn_write retcode -3 and no
  # clock sample reached the module; wait_for_clock then held the entire Nav2
  # launch before SLAM and lifecycle_manager_navigation.  Require one payload
  # in the same direction as the simulated sensors.
  say "2b/4 host publica, modulo recebe uma amostra real de /clock"
  if remote "cd ${remote_dir} && docker compose -f compose.module.yml exec -T tools \
    /usr/local/bin/entrypoint.sh timeout 12 ros2 topic echo --once /clock rosgraph_msgs/msg/Clock" \
      2>&1 | grep -q '^clock:'; then
    printf '    /clock: OK (mensagem recebida no modulo)\n'
  else
    printf '    /clock: SEM MENSAGEM. A lista de topicos pode estar stale; verifique\n'
    printf '    prioridade da interface no host.rendered.xml e erros retcode -3.\n'
    verify_failed=1
  fi

  # --- 3/4: module -> host, the direction the demo actually needs -----------
  # Step 2 only proves the module can SEE the host. This proves the module can
  # PUBLISH and the host receives it, which is the data path the demo is for.
  #
  # heartbeat_publisher (demo_tutorials, ML1) is used on purpose rather than nav
  # or perception: it publishes only /demo/system/heartbeat, which no other node
  # in this project consumes. Starting nav here instead would put a second
  # publisher on /demo/cmd_vel and drive whatever simulation is running on the
  # host — a destructive side effect on someone else's experiment, with no error
  # anywhere to explain the robot moving on its own.
  #
  # The namespace comes from `--ros-args -r __ns:=/demo`, NOT from ROS_NAMESPACE.
  #
  # ROS_NAMESPACE was tried first and DOES NOT WORK on Jazzy. It is not a
  # passthrough problem: `printenv ROS_NAMESPACE` inside the container returned
  # /demo, and the node still published /system/heartbeat. ROS 2 simply ignores
  # the variable. Note that scripts/env.sh exports it as if it worked.
  #
  # The namespace is needed because the node publishes the RELATIVE topic
  # `system/heartbeat`, while base/Dockerfile deliberately sets no namespace
  # image-wide (it would double-prefix the absolute /demo/* names everywhere
  # else). Without the remap the topic is /system/heartbeat and a subscriber on
  # /demo/system/heartbeat waits forever on a name nobody publishes.
  say "3/4 modulo publica, host recebe (/demo/system/heartbeat)"

  if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    printf '    ROS nativo ausente no host, etapa 3 nao executada\n'
    return 1
  fi

  # Start the NEW participant on the host first. Under unicast discovery the
  # participant that joins last is precisely the one least likely to have been
  # announced to its peer. The old order started a remote publisher, slept an
  # arbitrary six seconds, and only then created the subscriber; identical
  # links alternated between PASS and false-negative depending on the SPDP
  # period. A subscriber already waiting cannot miss the first useful sample.
  [[ -f "${host_cfg:-}" ]] || render_host_config
  local heartbeat_output subscriber_pid
  heartbeat_output="$(mktemp)"
  (
    CYCLONEDDS_URI="file://${host_cfg}" bash -c '
      source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      export ROS_DOMAIN_ID='"${ROS_DOMAIN_ID}"'
      timeout 75 ros2 topic echo --once /demo/system/heartbeat std_msgs/msg/String
    '
  ) >"${heartbeat_output}" 2>&1 &
  subscriber_pid=$!

  # `timeout 70` and not a later pkill: procps is not guaranteed in ros-base, so
  # a cleanup that depends on pkill can silently fail and leave a publisher
  # injecting into the domain after this script exits. The node bounds itself.
  if ! remote "cd ${remote_dir} && docker compose -f compose.module.yml exec -d tools \
      /usr/local/bin/entrypoint.sh timeout 70 ros2 run demo_tutorials heartbeat_publisher \
      --ros-args -r __ns:=/demo" >/dev/null; then
    printf '    falha ao iniciar o publisher no modulo.\n'
    kill "${subscriber_pid}" >/dev/null 2>&1 || true
    wait "${subscriber_pid}" >/dev/null 2>&1 || true
    rm -f "${heartbeat_output}"
    return 1
  fi

  # exec -d hides every error, including "command not found". Confirm the node is
  # actually up before blaming DDS for the silence on the host side. Polling is
  # bounded, but does not confuse a chosen fixed sleep with a readiness signal.
  local publisher_ready=0 attempt
  for attempt in {1..15}; do
    if remote "cd ${remote_dir} && docker compose -f compose.module.yml exec -T tools \
          /usr/local/bin/entrypoint.sh ros2 node list" 2>/dev/null | grep -q heartbeat_publisher; then
      publisher_ready=1
      break
    fi
    sleep 1
  done
  if [[ ${publisher_ready} -eq 0 ]]; then
    printf '    o publisher NAO subiu no modulo. Nada a concluir sobre o link DDS.\n'
    verify_failed=1
    kill "${subscriber_pid}" >/dev/null 2>&1 || true
    wait "${subscriber_pid}" >/dev/null 2>&1 || true
    rm -f "${heartbeat_output}"
    return 1
  fi
  printf '    publisher ativo no modulo\n'

  set +e
  wait "${subscriber_pid}"
  local subscriber_status=$?
  set -e
  local received
  received="$(<"${heartbeat_output}")"
  rm -f "${heartbeat_output}"

  # Belt and braces on top of the timeout above; harmless when pkill is absent.
  remote "cd ${remote_dir} && docker compose -f compose.module.yml exec -T tools pkill -f heartbeat_publisher" >/dev/null 2>&1 || true

  if [[ ${subscriber_status} -eq 0 ]] && printf '%s' "${received}" | grep -q 'count='; then
    printf '    host recebeu do modulo: %s\n' "$(printf '%s' "${received}" | grep -m1 'count=')"
  else
    printf '    host NAO recebeu. Saida do echo:\n%s\n' "${received}"
    verify_failed=1
  fi

  # --- 4/4: o Nav2 esta ATIVO, e nao apenas de pe --------------------------
  #
  # AS TRES ETAPAS ACIMA PASSAM COM O NAV2 MORTO. Medido em 26/08/2026: depois
  # de um `up`, a aresta odom -> base demorou mais de 60 s para atravessar a
  # fronteira, o local_costmap nao ativou, e o gerenciador de ciclo de vida
  # ABORTOU o bringup em definitivo -- sem nova tentativa. O container ficou de
  # pe, todos os topicos apareceram, `verify` retornou 0, e toda meta era
  # recusada com "Action server is inactive. Rejecting the goal."
  #
  # Topico existir nao e servico funcionar. Esta etapa pergunta o estado de
  # ciclo de vida, que e a unica coisa que separa os dois casos.
  #
  # `ros2 lifecycle get` e nao `service call`: a forma com service call precisa
  # de "{}" como argumento, e este bloco esta dentro de remote "...", uma string
  # entre aspas duplas que o shell de fora expande antes de o ssh ver. Chaves e
  # aspas ali dentro ja quebraram este arquivo uma vez.
  say "4/4 Nav2 ativo no modulo (bt_navigator)"
  local nav_state
  nav_state="$(remote "cd ${remote_dir} && docker compose -f compose.module.yml exec -T tools \
    /usr/local/bin/entrypoint.sh bash -c 'ros2 lifecycle get /bt_navigator'" 2>/dev/null \
    | tr -d '\r' | tail -1 || true)"

  if printf '%s' "${nav_state}" | grep -Eq '^active([[:space:]]|$)'; then
    printf '    bt_navigator: %s\n' "${nav_state}"
  else
    printf '    bt_navigator NAO esta ativo: %s\n' "${nav_state:-<sem resposta>}"
    printf '    Toda meta sera recusada. Procure no log do nav:\n'
    printf '      "Failed to bring up all requested nodes"  -> a subida abortou\n'
    printf '      "did not become available before timeout" -> foi a TF odom -> base\n'
    printf '    Destrave com STARTUP no gerenciador; conserte com o portao\n'
    printf '    wait_for_tf em nav_quadruped.launch.py.\n'
    verify_failed=1
  fi

  return "${verify_failed}"
}

cmd_shell() {
  remote "cd ${remote_dir} && docker compose -f compose.module.yml --profile tools up -d tools" >/dev/null
  # Through the entrypoint, otherwise the shell opens without ROS sourced and
  # every ros2 command answers "command not found".
  ssh "${SSH_OPTS[@]}" -t "${ssh_target}" \
    "cd ${remote_dir} && docker compose -f compose.module.yml exec tools /usr/local/bin/entrypoint.sh bash"
}

cmd_render_local() {
  # Renderiza a config de DDS para o modo LEARN, sem modulo nenhum.
  #
  # `sync` exige HOST_IP e MODULE_IP porque injeta o peer do Aquila e fixa a
  # interface. Em learn nao existe modulo, e ainda assim o compose monta
  # `host.rendered.xml` -- se o arquivo nao existir o Compose cria um DIRETORIO
  # com esse nome e o CycloneDDS falha ao ler a config.
  #
  # Antes disto o unico caminho documentado era "rode module.sh sync antes do
  # primeiro up", que em learn nao roda. O resultado era copiar o template a mao,
  # e uma copia manual e a que fica velha: a correcao de loopback de 25/08/2026
  # ficou commitada no template enquanto os containers rodavam a copia antiga.
  local src dst
  src="${repo_dir}/docker/cyclonedds/host.xml"
  dst="${repo_dir}/docker/cyclonedds/host.rendered.xml"
  cp "${src}" "${dst}"
  python3 -c "import xml.dom.minidom; xml.dom.minidom.parse('${dst}')" \
    || die "host.rendered.xml nao e XML valido"
  say "host.rendered.xml renderizado para LEARN (sem peer de modulo)."
  say "Recrie os containers para que a config nova seja lida:"
  say "  docker compose -f docker/compose.host.yml up -d --force-recreate"
}

# Sourcing is a test seam for the configuration loader. It deliberately stops
# before command dispatch, so precedence tests never need SSH and never touch
# the operator's real module.
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  return 0
fi

case "${1:-}" in
  inventory)    cmd_inventory ;;
  render-local) cmd_render_local ;;
  sync)      cmd_sync ;;
  build)     cmd_build ;;
  up)        cmd_up "$@" ;;
  down)      cmd_down ;;
  status)    cmd_status ;;
  verify)    cmd_verify ;;
  shell)     cmd_shell ;;
  *)         sed -n '5,17p' "${BASH_SOURCE[0]}"; exit 1 ;;
esac
