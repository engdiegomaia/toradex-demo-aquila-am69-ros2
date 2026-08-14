#!/usr/bin/env bash
#
# Shared entrypoint for every demo container.
#
# Sources the ROS underlay and this workspace's overlay, then execs the command.
# The overlay source is the load-bearing part: without it `ros2 launch
# demo_bringup sim.launch.py` fails with "package 'demo_bringup' not found"
# even though the package is built and sitting in /ws/install.
#
# set -e matters here. Without it, a failed `source` leaves a container that
# starts, finds no packages, and exits 0 — which compose reports as a clean
# shutdown rather than a failure.
set -e

# shellcheck disable=SC1090,SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# The overlay is absent in the rare case of a base image built without the
# workspace; treat that as a hard error rather than continuing into a confusing
# "package not found" from ros2 launch.
if [ ! -f /ws/install/setup.bash ]; then
    echo "entrypoint: /ws/install/setup.bash missing — the workspace did not build." >&2
    exit 1
fi

# shellcheck disable=SC1091
source /ws/install/setup.bash

# CYCLONEDDS_URI points at a file mounted by compose. A missing file is not
# fatal to CycloneDDS — it silently falls back to defaults, which means
# multicast discovery and a host<->module link that appears to work on a flat
# LAN and fails on Wi-Fi or a managed switch. Warn loudly instead.
if [ -n "${CYCLONEDDS_URI}" ]; then
    cfg="${CYCLONEDDS_URI#file://}"
    if [ ! -f "${cfg}" ]; then
        echo "entrypoint: WARNING CYCLONEDDS_URI=${CYCLONEDDS_URI} but ${cfg} does not exist." >&2
        echo "entrypoint: CycloneDDS will fall back to multicast defaults." >&2
    fi
fi

exec "$@"
