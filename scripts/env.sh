# Source this file after /opt/ros/jazzy/setup.bash to apply the project's ROS 2
# environment contract. These same variables are baked into the base container
# image as ENV lines at L4 — the container is a repackaging of what runs here.
#
# Usage:
#   source /opt/ros/jazzy/setup.bash
#   source scripts/env.sh
#
# Origin of each value: .ai/AGENTS.md §7 (configuration contract).

# DDS domain shared by every process in the demo (host + module later).
# Chosen to avoid the default 0, which floods with unrelated traffic on shared LANs.
export ROS_DOMAIN_ID=69

# All demo topics/nodes live under /demo/* (see .ai/AGENTS.md §5.2).
# NOTE: unset this when running turtlesim or other third-party nodes that
# publish under their own namespace (e.g. /turtle1/*), otherwise the remap
# hides them.
export ROS_NAMESPACE=/demo

# CycloneDDS is the project RMW. The default RMW has a known inter-process
# discovery failure where `ros2 topic list` shows a topic but no messages
# arrive. See CLAUDE.md rule 2 and ADR-002 (to be written under M0).
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 0 = allow DDS discovery across the LAN (needed for the two-machine setup
# in target mode). Set to 1 only for isolated single-host debugging.
export ROS_LOCALHOST_ONLY=0

# A MESMA configuracao de DDS que os containers usam.
#
# Antes desta linha havia duas configuracoes vivas ao mesmo tempo: os containers
# montavam docker/cyclonedds/host.xml (multicast OFF, peer 127.0.0.1 explicito) e
# qualquer ferramenta rodada nativamente no host usava o DEFAULT do CycloneDDS
# (multicast ON, interface automatica). Duas politicas de descoberta diferentes
# no mesmo dominio funcionam enquanto a interface escolhida por acidente coincide,
# e param de funcionar sem avisar quando ela deixa de coincidir.
#
# O sintoma medido em 25/08/2026: `nav_trial.py` no host abortava com
# "navigate_to_pose nao apareceu" enquanto `docker compose logs nav` mostrava
# "Managed nodes are active" e /demo/odom chegava a 49 Hz. Metade do grafo
# visivel, metade nao, e nada nomeando DDS.
#
# Prefere o RENDERIZADO, que e o que os containers montam, e cai no template
# quando ele nao existe. Os dois casos sao reais: em hil o renderizado tem o peer
# do modulo e o template nao (endereco nao entra em git), e num clone novo em
# learn o renderizado ainda nao foi gerado.
#
# Ancorado no diretorio deste script para nao depender do cwd de quem o carrega:
# os scripts sao chamados tanto da raiz do repo quanto de dentro de scripts/.
_ecc_env_dir="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
_ecc_dds="${_ecc_env_dir}/../docker/cyclonedds/host.rendered.xml"
[ -f "${_ecc_dds}" ] || _ecc_dds="${_ecc_env_dir}/../docker/cyclonedds/host.xml"
export CYCLONEDDS_URI="file://${_ecc_dds}"
unset _ecc_env_dir _ecc_dds
