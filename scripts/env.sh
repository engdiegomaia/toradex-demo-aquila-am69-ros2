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

# The SAME DDS configuration the containers use.
#
# There used to be two live configurations at once: the containers mounted
# docker/cyclonedds/host.xml (multicast OFF, explicit peer 127.0.0.1), while any
# tool run natively on the host used the CycloneDDS DEFAULT (multicast ON,
# automatic interface). Two different discovery policies on the same domain work
# as long as the interface picked by accident happens to match, and stop working
# without warning when it stops matching.
#
# Symptom measured on 25/08/2026: `nav_trial.py` on the host aborted with
# "navigate_to_pose did not appear" while `docker compose logs nav` showed
# "Managed nodes are active" and /demo/odom arrived at 49 Hz. Half the graph
# visible, half not, and nothing naming DDS.
#
# Prefers the RENDERED file, which is what the containers mount, and falls back
# to the template when it does not exist. Both cases are real: in hil the
# rendered file has the module's peer and the template does not (the address
# does not go into git), and in a fresh learn clone the rendered file has not
# been generated yet.
#
# Anchored to this script's directory so it does not depend on the cwd of
# whoever sources it: the scripts are called both from the repo root and from
# inside scripts/.
_ecc_env_dir="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
_ecc_dds="${_ecc_env_dir}/../docker/cyclonedds/host.rendered.xml"
[ -f "${_ecc_dds}" ] || _ecc_dds="${_ecc_env_dir}/../docker/cyclonedds/host.xml"
export CYCLONEDDS_URI="file://${_ecc_dds}"
unset _ecc_env_dir _ecc_dds
