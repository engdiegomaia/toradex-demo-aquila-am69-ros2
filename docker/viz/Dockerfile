# Visualization container — RViz2.
#
# Builds for: linux/amd64 ONLY.
#
# DO NOT ADD linux/arm64. RViz2 is an OGRE 2 application and wants desktop
# OpenGL; the AM69 has OpenGL ES 3.2 only (CLAUDE.md rule 1). This is also why
# compose.module.yml has no viz service at all — the guard is in both places on
# purpose, because a compose file is easy to copy and edit by hand.
#
# Build (from the repo root):
#   docker build -f docker/viz/Dockerfile -t local/demo-aquila-viz:dev .
ARG REGISTRY=local
ARG TAG=dev
FROM ${REGISTRY}/demo-aquila-base:${TAG}

ARG ROS_DISTRO=jazzy
USER root

# rviz2 for the map/costmap/robot view; rqt_* for topic and TF inspection during
# development. nav2_bringup is needed here too: demo_view.rviz is derived from
# Nav2's default view and the Nav2 RViz plugins (the goal tool, the costmap
# displays) live in nav2_rviz_plugins.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-nav2-rviz-plugins \
        ros-${ROS_DISTRO}-rqt-common-plugins \
        ros-${ROS_DISTRO}-nav2-minimal-tb4-description \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
# RViz consumes robot descriptions and Nav2 data but does not load the Gazebo
# ros2_control backend. Keep the simulator-only package out of this role.
ARG COLCON_IGNORE_PACKAGES="gz_quadruped_hardware"
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install --event-handlers console_direct+ \
        ${COLCON_IGNORE_PACKAGES:+--packages-ignore ${COLCON_IGNORE_PACKAGES}} \
    && rm -rf /ws/log \
    && chown -R ubuntu:ubuntu /ws /home/ubuntu

USER ubuntu

CMD ["ros2", "launch", "demo_bringup", "viz.launch.py"]
