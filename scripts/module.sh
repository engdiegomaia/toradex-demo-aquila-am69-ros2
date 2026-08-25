#!/usr/bin/env bash
set -euo pipefail

# Bring the Aquila AM69 module side of the demo up, from this workstation.
#
# Usage:
#   scripts/module.sh inventory   # what the module is: OS, docker, disk, links
#   scripts/module.sh sync        # push sources + rendered config to ~/demo
#   scripts/module.sh build       # build the arm64 images ON the module
#   scripts/module.sh up          # docker compose up -d  (nav + perception)
#   scripts/module.sh down
#   scripts/module.sh status
#   scripts/module.sh verify      # DDS reachability + topic contract from the module
#   scripts/module.sh shell       # interactive shell in the tools container
#
# Configuration, in precedence order: environment, then docker/.env, then the
# defaults below. Nothing here hard-codes an address (CLAUDE.md conventions).
#
#   MODULE_HOST   ssh target                    (default aquila-am69-12593525.local)
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

# docker/.env is optional and, when present, must not override an explicit
# environment variable — hence the ${VAR:-} guards after sourcing.
env_file="${repo_dir}/docker/.env"
if [[ -f "${env_file}" ]]; then
  # shellcheck disable=SC1090
  set -a; source "${env_file}"; set +a
fi

MODULE_HOST="${MODULE_HOST:-aquila-am69-12593525.local}"
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
    /^[[:space:]]*<NetworkInterface autodetermine="true"[^>]*\/>[[:space:]]*$/ {
      printf "        <NetworkInterface name=\"%s\" priority=\"default\"/>  <!-- fixado por scripts/module.sh -->\n", iface
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

  if [[ -n "${host_pubs}" && "${2:-}" != "--force" ]]; then
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

  remote "cd ${remote_dir} && docker compose -f compose.module.yml up -d"
  cmd_status
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
    printf '    nenhuma porta livre em %s..%s: DDS ocupando a faixa toda. Siga para a etapa 2.\n' \
      "${base_port}" "$((base_port + 18))"
  else
    [[ "${port}" == "${base_port}" ]] || busy=" (${base_port} ocupada: DDS vivo neste host)"
    say "1/3 alcance UDP no dominio ${ROS_DOMAIN_ID}, porta ${port}${busy}"

    python3 - "${port}" <<'PYLISTEN' &
import socket, sys
p = int(sys.argv[1])
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    s.bind(("0.0.0.0", p))
except OSError as exc:
    print(f"    modulo -> host: nao consegui escutar em {p}: {exc}")
    raise SystemExit(0)
s.settimeout(15)
# The payload IS checked. These are live RTPS discovery ports: with a simulation
# running on the same domain, the first datagram to arrive is often real SPDP
# traffic from another participant, and accepting it would report "OK" for a
# port the module never reached. Measured once as "de 192.168.15.98" — the
# host's own address — on a probe that was supposed to prove the module could
# reach us.
deadline = 15
while deadline > 0:
    s.settimeout(deadline)
    start = None
    try:
        data, addr = s.recvfrom(2048)
    except socket.timeout:
        print("    modulo -> host: TIMEOUT. Verifique firewall do host nesta porta UDP.")
        break
    if data == b"dds-probe":
        print(f"    modulo -> host: OK (de {addr[0]})")
        break
    deadline -= 1
else:
    print("    modulo -> host: so chegou trafego RTPS de terceiros, probe nao confirmado.")
PYLISTEN
    local listener=$!
    sleep 2
    remote "python3 -c \"import socket;s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);[s.sendto(b'dds-probe',('${HOST_IP}',${port})) for _ in range(3)]\"" || true
    wait "${listener}"
  fi

  say "2/3 contrato de topicos visto de dentro do modulo"
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
  local seen
  seen="$(remote "cd ${remote_dir} && docker compose -f compose.module.yml exec -T tools \
    /usr/local/bin/entrypoint.sh bash -c '
      ros2 node list
      ros2 topic list | grep /demo/
    '" 2>/dev/null | grep -v 'ROS_LOCALHOST_ONLY\|automatic_discovery_range' || true)"

  if [[ -n "${seen}" ]]; then
    printf '%s\n' "${seen}" | sed 's/^/    /'
  else
    printf '    o modulo nao ve nada publicado no dominio %s.\n' "${ROS_DOMAIN_ID}"
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

  # --- 3/3: module -> host, the direction the demo actually needs -----------
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
  say "3/3 modulo publica, host recebe (/demo/system/heartbeat)"

  [[ -f /opt/ros/jazzy/setup.bash ]] || {
    printf '    ROS nativo ausente no host, etapa 3 nao executada\n'; return 0
  }

  # `timeout 60` and not a later pkill: procps is not guaranteed in ros-base, so
  # a cleanup that depends on pkill can silently fail and leave a publisher
  # injecting into the domain after this script exits. The node bounds itself.
  remote "cd ${remote_dir} && docker compose -f compose.module.yml exec -d tools \
    /usr/local/bin/entrypoint.sh timeout 60 ros2 run demo_tutorials heartbeat_publisher \
    --ros-args -r __ns:=/demo" >/dev/null

  # exec -d hides every error, including "command not found". Confirm the node is
  # actually up before blaming DDS for the silence on the host side.
  sleep 4
  if ! remote "cd ${remote_dir} && docker compose -f compose.module.yml exec -T tools \
        /usr/local/bin/entrypoint.sh ros2 node list" 2>/dev/null | grep -q heartbeat_publisher; then
    printf '    o publisher NAO subiu no modulo. Nada a concluir sobre o link DDS.\n'
    return 1
  fi
  printf '    publisher ativo no modulo\n'

  # Unicast discovery in both directions takes a few seconds. Echoing
  # immediately reports "no messages" for a reason unrelated to configuration.
  sleep 6
  local received
  set +e
  # CYCLONEDDS_URI is the whole point: without the rendered host config this
  # subscriber uses CycloneDDS defaults and never discovers the module. See
  # render_host_config.
  [[ -f "${host_cfg:-}" ]] || render_host_config
  received="$(
    CYCLONEDDS_URI="file://${host_cfg}" bash -c '
      source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      export ROS_DOMAIN_ID='"${ROS_DOMAIN_ID}"'
      timeout 25 ros2 topic echo --once /demo/system/heartbeat std_msgs/msg/String 2>&1
    '
  )"
  set -e

  # Belt and braces on top of the timeout above; harmless when pkill is absent.
  remote "cd ${remote_dir} && docker compose -f compose.module.yml exec -T tools pkill -f heartbeat_publisher" >/dev/null 2>&1 || true

  if printf '%s' "${received}" | grep -q 'count='; then
    printf '    host recebeu do modulo: %s\n' "$(printf '%s' "${received}" | grep -m1 'count=')"
  else
    printf '    host NAO recebeu. Saida do echo:\n%s\n' "${received}"
    return 1
  fi
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
