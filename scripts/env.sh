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
