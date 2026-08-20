# Base image for every demo container — host and module alike.
#
# Builds for: linux/amd64 and linux/arm64. ros:jazzy-ros-base publishes both, and
# Torizon OS is an ordinary Docker host, so the official image runs on the module
# with no userspace adaptation.
#
# This image exists to hold the ROS 2 environment contract in ONE place. Its job
# is the RMW setting, the domain, and the overlay source line; the role images
# (sim, nav, perception, viz, tools) add packages and nothing else. Six copies of
# an ENV line is six chances for one of them to drift.
#
# Build (from the repo root, not from docker/):
#   docker build -f docker/base/Dockerfile -t local/demo-aquila-base:dev .
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

# --- ROS 2 environment contract -------------------------------------------
# Mirrors scripts/env.sh, which is what runs natively on the host. The comment
# in that file states these values get baked in as ENV at the container stage;
# this is that stage. Keep the two in sync — a container on a different domain
# than the host tooling produces "the topics do not appear" with no error.
#
# RMW: CycloneDDS always (CLAUDE.md rule 2). Set here as ENV and never at
# runtime, never swapped for Fast DDS. The default RMW has a known
# inter-container discovery failure where `ros2 topic list` shows the topic and
# no messages ever arrive.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Domain 69 comes from scripts/env.sh, not from the 42 in the ML3.5 guide's
# .env.example. The repo's existing value wins so host-native tooling and
# containers land on the same domain by default; compose can still override it.
ENV ROS_DOMAIN_ID=69

# 0 = allow DDS discovery across the LAN, required for the host<->module split.
ENV ROS_LOCALHOST_ONLY=0

# Unbuffered Python so node logs reach `docker compose logs` as they happen
# rather than in blocks when the pipe buffer fills. Without this a crashing node
# can appear to have produced no output at all.
ENV PYTHONUNBUFFERED=1

# NOTE: ROS_NAMESPACE is deliberately NOT set here, unlike scripts/env.sh.
# Topic names in this project are already absolute (/demo/*) in the launch files
# and bridge config, so a namespace would double-prefix them to
# /demo/demo/cmd_vel and silently break the topic contract.

# --- CycloneDDS ------------------------------------------------------------
# The RMW implementation package is not in ros-base; without it every node
# fails at startup with a library-load error naming rmw_cyclonedds_cpp.
#
# --no-install-recommends and the same-layer apt-list cleanup are not cosmetic:
# container storage on Torizon lives on the data partition, so a fat layer costs
# real space on the module (guia-ml35-docker.md §4).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# --- Non-root user ---------------------------------------------------------
# ros:jazzy-ros-base runs as root and already ships an `ubuntu` user (uid 1000).
# Reuse it rather than creating another uid-1000 user, which would fail.
#
# uid 1000 also matches the default `torizon` user on the module, so files a
# container writes to a bind mount stay accessible from the host shell.
ARG USERNAME=ubuntu
RUN mkdir -p /cfg /ws \
    && chown -R ${USERNAME}:${USERNAME} /cfg /ws

# --- Workspace -------------------------------------------------------------
# The whole ros2_ws source tree is copied and built into an overlay at
# /ws/install. Every role image inherits the same built workspace; they differ
# only in which system packages are present to satisfy it at runtime.
#
# Context is the REPO ROOT, so the path is ros2_ws/src (see the build command in
# the header). Building from docker/ would put this path out of context and fail
# with a confusing "not found".
WORKDIR /ws
COPY ros2_ws/src /ws/src

# --- Per-arch build variation ----------------------------------------------
# Both default to empty, so an amd64/host build behaves exactly as before.
#
# They exist because F3 added quadruped packages to a tree whose container layer
# was written in F1, when the tree was diff-drive only. `gz_quadruped_hardware`
# carries <depend>gz_sim_vendor</depend> and <depend>gz_plugin_vendor</depend> —
# build dependencies, not exec — so rosdep resolves the ENTIRE Gazebo stack, and
# a blanket `colcon build` of src therefore puts OGRE 2 on the arm64 images that
# ship to the AM69. That is CLAUDE.md rule 1, and nothing in the build output
# names it: the build succeeds and the violation is invisible until the module
# tries to run a desktop-OpenGL library that does not exist there.
#
# The module build passes:
#   --build-arg SKIP_KEYS_EXTRA="gz_sim_vendor gz_plugin_vendor"
#   --build-arg COLCON_IGNORE_PACKAGES="gz_quadruped_hardware"
#
# Both are needed and neither is sufficient alone: skip-keys stops rosdep from
# INSTALLING Gazebo, --packages-ignore stops colcon from putting the package in
# the workspace graph at all. Skipping only the first fails the build; skipping
# only the second still ships Gazebo in the image.
#
# IT MUST BE --packages-ignore, NOT --packages-skip. This was tried the other way
# and failed, and the reason is not obvious:
#
#   ERROR:colcon.colcon_ros.task.ament_python.build:Failed to find the following
#   files: /ws/install/gz_quadruped_hardware/share/gz_quadruped_hardware/package.sh
#   Check that the following packages have been built: gz_quadruped_hardware
#
# demo_simulation declares gz_quadruped_hardware as exec_depend. colcon's
# dependency graph does not distinguish exec from build, so --packages-skip
# leaves the package IN the graph while not building it, and the ament_python
# environment-hook step for demo_simulation then requires an install/ artifact
# that was never produced. demo_simulation fails, demo_bringup is "not
# processed", and demo_bringup holds nav.launch.py and perception.launch.py —
# the entrypoints of both module services. --packages-ignore removes the package
# from the workspace entirely, so nothing depends on it.
#
# `gz_quadruped_hardware` is the Gazebo-side hardware interface for
# ros2_control. It has no purpose on the module in hil mode, where the simulated
# robot lives on the host. `unitree_guide_controller` IS built here — it needs
# controller_interface and kdl_parser only, no Gazebo — because deploy mode runs
# the leg controller on the module.
ARG SKIP_KEYS_EXTRA=""
ARG COLCON_IGNORE_PACKAGES=""

# rosdep needs the apt lists present, so update/install/clean has to be one
# layer with the build. Splitting it leaves either a stale list or a fat image.
#
# --skip-keys: the packages a role image is expected to provide (or deliberately
# omit) must not block the base build. Gazebo and RViz2 are the important ones —
# resolving them here would pull OGRE 2 into arm64 images, violating rule 1.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths src --ignore-src -y \
        --skip-keys "ros_gz_sim ros_gz_bridge rviz2 nav2_minimal_tb4_sim \
nav2_minimal_tb4_description navigation2 nav2_bringup slam_toolbox \
joint_state_publisher_gui teleop_twist_keyboard turtlesim \
nav2_amcl nav2_behavior_tree nav2_behaviors nav2_bt_navigator \
nav2_collision_monitor nav2_controller nav2_costmap_2d nav2_lifecycle_manager \
nav2_map_server nav2_mppi_controller nav2_navfn_planner nav2_planner nav2_route \
nav2_smoother nav2_velocity_smoother nav2_waypoint_follower opennav_docking \
${SKIP_KEYS_EXTRA}" \
    && colcon build --symlink-install --event-handlers console_direct+ \
        ${COLCON_IGNORE_PACKAGES:+--packages-ignore ${COLCON_IGNORE_PACKAGES}} \
    && rm -rf /var/lib/apt/lists/* /ws/log

# --- Entrypoint ------------------------------------------------------------
# Sources the ROS underlay AND this workspace's overlay before exec'ing the
# command. Without the overlay source, `ros2 launch demo_bringup ...` fails with
# "package not found" even though the package is built and present.
COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

USER ${USERNAME}

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
