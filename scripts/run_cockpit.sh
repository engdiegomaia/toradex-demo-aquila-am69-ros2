#!/usr/bin/env bash
set -euo pipefail

# One-command operator UI for the x86 host.
#
# Gazebo, RViz and rqt_image_view keep rendering in their normal host
# containers. cockpit.py adopts their X11 client windows into one standalone Qt
# application and adds manual robot controls. It never replaces or reconfigures
# the desktop session. In HIL mode only Nav2 and perception run on the Aquila
# AM69. No graphical package reaches arm64.

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
compose=(docker compose -f "${repo_dir}/docker/compose.host.yml")
state_dir="${repo_dir}/log/cockpit"
sim_pid_file="${state_dir}/sim.pid"

usage() {
  cat <<'EOF'
Uso:
  scripts/run_cockpit.sh start [opções]
  scripts/run_cockpit.sh stop [--mode hil|learn]
  scripts/run_cockpit.sh status [--mode hil|learn]

Opções de start:
  --mode hil|learn       HIL no Aquila (padrão) ou stack inteira no host
  --world ARQUIVO        mundo SDF (padrão: quadruped_maze11.sdf)
  --screen TELA          monitor Qt/XRandR (padrão: primary; ex.: DP-1)
  --maze-models DIRETÓRIO  modelos externos do ros_maze_worlds
  --keep-running         ao fechar a UI, mantenha os serviços ativos

Exemplo do bench:
  scripts/run_cockpit.sh start --mode hil \
    --maze-models /home/$USER/ros_maze_worlds/models --screen DP-1
EOF
}

mode=hil
world=quadruped_maze11.sdf
screen=primary
maze_models="${MAZE_MODELS:-}"
keep_running=false
command_name="${1:-start}"
if [[ "${command_name}" == -h || "${command_name}" == --help ]]; then
  usage
  exit 0
fi
[[ $# -eq 0 ]] || shift

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) mode="${2:?--mode exige hil ou learn}"; shift 2 ;;
    --world) world="${2:?--world exige um arquivo}"; shift 2 ;;
    --screen) screen="${2:?--screen exige um nome}"; shift 2 ;;
    --maze-models) maze_models="${2:?--maze-models exige um diretório}"; shift 2 ;;
    --keep-running) keep_running=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Opção desconhecida: %s\n\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

# docker/.env is the existing machine-local configuration surface.  Read only
# this cockpit value; sourcing the whole file here could replace the DISPLAY of
# the active X11 session with an old value kept for another login.
if [[ -z "${maze_models}" && -f "${repo_dir}/docker/.env" ]]; then
  maze_models="$(sed -n 's/^MAZE_MODELS=//p' "${repo_dir}/docker/.env" | tail -n1)"
  maze_models="${maze_models%\"}"
  maze_models="${maze_models#\"}"
fi

[[ "${mode}" == hil || "${mode}" == learn ]] \
  || { echo "--mode deve ser hil ou learn" >&2; exit 2; }

say() { printf '\n[cockpit] %s\n' "$*"; }
die() { printf '\n[cockpit] ERRO: %s\n' "$*" >&2; exit 1; }

module_action() {
  "${repo_dir}/scripts/module.sh" "$@"
}

camera_is_running() {
  "${compose[@]}" exec -T viz pgrep -f '[r]qt_image_view' >/dev/null 2>&1
}

start_camera() {
  if camera_is_running; then
    say "camera já está aberta"
    return
  fi
  say "abrindo /demo/camera/image_raw no host"
  "${compose[@]}" exec -d viz bash -lc \
    '. /opt/ros/jazzy/setup.bash; . /ws/install/setup.bash; exec ros2 run rqt_image_view rqt_image_view /demo/camera/image_raw'
}

stop_host() {
  say "encerrando telas e simulação do host"
  "${compose[@]}" stop viz >/dev/null 2>&1 || true
  if [[ "${mode}" == learn ]]; then
    "${compose[@]}" --profile learn stop nav perception >/dev/null 2>&1 || true
  fi
  if docker ps -a --format '{{.Names}}' | grep -qx aquila-go2; then
    docker stop -t 10 aquila-go2 >/dev/null 2>&1 || true
  fi
  if [[ -f "${sim_pid_file}" ]]; then
    sim_pid="$(<"${sim_pid_file}")"
    if [[ "${sim_pid}" =~ ^[0-9]+$ ]]; then
      kill "${sim_pid}" >/dev/null 2>&1 || true
    fi
    rm -f "${sim_pid_file}"
  fi
  xhost -local:docker >/dev/null 2>&1 || true
}

stop_all() {
  stop_host
  if [[ "${mode}" == hil ]]; then
    say "encerrando Nav2 e percepção no Aquila"
    module_action down || true
  fi
}

show_status() {
  echo "--- host ---"
  docker ps --format '{{.Names}}\t{{.Status}}' \
    | grep -E 'aquila-go2|demo-aquila-(viz|nav|perception)' || echo '(parado)'
  if [[ "${mode}" == hil ]]; then
    echo "--- Aquila ---"
    module_action status
  fi
}

case "${command_name}" in
  stop) stop_all; exit 0 ;;
  status) show_status; exit 0 ;;
  start) ;;
  *) usage >&2; exit 2 ;;
esac

[[ -n "${DISPLAY:-}" ]] || die 'DISPLAY não está definido; abra numa sessão gráfica X11.'
command -v xwininfo >/dev/null || die 'xwininfo ausente; instale o pacote x11-utils no host.'
command -v xprop >/dev/null || die 'xprop ausente; instale o pacote x11-utils no host.'
python3 -c 'import PyQt5' >/dev/null 2>&1 \
  || die 'PyQt5 ausente; instale python3-pyqt5 no host.'
docker info >/dev/null 2>&1 || die 'Docker não está acessível para este usuário.'

if [[ "${XDG_SESSION_TYPE:-x11}" != x11 ]]; then
  die 'o encaixe de janelas requer sessão X11; encerre a sessão Wayland e entre em Ubuntu on Xorg.'
fi

# The cockpit is deliberately a normal client of the existing window manager.
# Requiring a live EWMH root here makes that boundary explicit: startup fails
# locally if the desktop session is unavailable instead of attempting to alter
# or replace any part of it.
wm_name="$(xprop -root _NET_SUPPORTING_WM_CHECK 2>/dev/null || true)"
[[ "${wm_name}" != *'not found'* && "${wm_name}" == *'_NET_SUPPORTING_WM_CHECK'* ]] \
  || die 'gerenciador de janelas EWMH indisponível; o cockpit não altera a sessão gráfica.'

if [[ "$(basename "${world}")" == quadruped_maze*.sdf ]]; then
  [[ -n "${maze_models}" ]] \
    || die 'mundo de labirinto exige --maze-models DIRETÓRIO (clone ros_maze_worlds/models).'
  [[ -d "${maze_models}" ]] || die "diretório de modelos inexistente: ${maze_models}"
fi

mkdir -p "${state_dir}"

# A UI owns the lifecycle by default.  Closing it leaves the workstation and
# module clean; --keep-running is explicit for debugging after closing a panel.
cleanup_needed=true
cleanup() {
  status=$?
  if [[ "${cleanup_needed}" == true && "${keep_running}" == false ]]; then
    stop_all
  fi
  exit "${status}"
}
trap cleanup EXIT INT TERM

if [[ "${mode}" == hil ]]; then
  [[ -f "${repo_dir}/docker/cyclonedds/host.rendered.xml" ]] \
    || die 'DDS do host não foi renderizado; rode scripts/module.sh sync primeiro.'
  say "subindo Nav2 e percepção no Aquila AM69"
  module_action up
else
  say "subindo Nav2 e percepção no host (modo learn)"
  "${compose[@]}" --profile learn up -d --no-build nav perception
fi

if docker ps --format '{{.Names}}' | grep -qx aquila-go2; then
  die 'aquila-go2 já está ativo; use stop antes de iniciar outro cockpit.'
fi

say "subindo Gazebo no host: ${world}"
MAZE_MODELS="${maze_models}" "${repo_dir}/scripts/run_quadruped_sim.sh" "${world}" \
  >"${state_dir}/sim.log" 2>&1 &
sim_pid=$!
printf '%s\n' "${sim_pid}" > "${sim_pid_file}"

for _ in $(seq 1 45); do
  docker ps --format '{{.Names}}' | grep -qx aquila-go2 && break
  kill -0 "${sim_pid}" 2>/dev/null \
    || die "simulação encerrou durante a subida; veja ${state_dir}/sim.log"
  sleep 1
done
docker ps --format '{{.Names}}' | grep -qx aquila-go2 \
  || die "simulação não ficou pronta em 45 s; veja ${state_dir}/sim.log"

say "subindo RViz no host"
"${compose[@]}" up -d --no-build viz
for _ in $(seq 1 30); do
  [[ "$("${compose[@]}" ps --status running -q viz | wc -l)" -eq 1 ]] && break
  sleep 1
done
[[ "$("${compose[@]}" ps --status running -q viz | wc -l)" -eq 1 ]] \
  || die 'container viz não ficou ativo; consulte docker compose logs viz.'

start_camera

say "abrindo cockpit standalone no host (${screen}); feche a janela para encerrar a aplicação"
python3 "${repo_dir}/scripts/cockpit.py" \
  --mode "${mode}" \
  --screen "${screen}" \
  --compose-file "${repo_dir}/docker/compose.host.yml" \
  --teleop-log "${state_dir}/teleop.log"

if [[ "${keep_running}" == true ]]; then
  cleanup_needed=false
  say "UI fechada; serviços preservados por --keep-running"
fi
