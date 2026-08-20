# Perception container — detection stub + costmap adapter.
#
# Builds for: linux/amd64 AND linux/arm64. The second image that migrates to the
# module. CPU and OpenCV only, nothing graphical.
#
# This container is separated from day one precisely because it defines the OTA
# update granularity: swapping the synthetic stub for real TIDL inference must be
# a container swap, not an interface change (CLAUDE.md rule 6). demo_perception
# itself is not modified by any ML3.5 phase.
#
# When perception eventually moves to accelerated inference it needs the AM69
# device nodes and the TI runtime, which this generic ROS image does not carry.
# Confirm the nodes and stack in the Toradex/TI documentation before assuming
# anything, and list /dev on the module first (guia-ml35-docker.md §4).
#
# Build (from the repo root):
#   docker build -f docker/perception/Dockerfile -t local/demo-aquila-perception:dev .
ARG REGISTRY=local
ARG TAG=dev
FROM ${REGISTRY}/demo-aquila-base:${TAG}

ARG ROS_DISTRO=jazzy
USER root

# vision_msgs carries Detection2DArray, the published half of the topic
# contract. sensor_msgs (Image) and the geometry/PointCloud2 types come with
# ros-base.
#
# The base image resolved demo_perception's dependencies already, so this is
# thin by design. It stays a separate image rather than reusing base directly so
# the TIDL swap has an obvious place to land.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# Must carry the same value as the base build that produced the parent image.
# This RUN rebuilds the WHOLE workspace, so without it colcon reaches
# gz_quadruped_hardware and fails against a Gazebo that the module image
# deliberately does not have (see docker/base/Dockerfile). Empty default keeps
# amd64/host builds unchanged.
ARG COLCON_IGNORE_PACKAGES=""

WORKDIR /ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install --event-handlers console_direct+ \
        ${COLCON_IGNORE_PACKAGES:+--packages-ignore ${COLCON_IGNORE_PACKAGES}} \
    && rm -rf /ws/log \
    && chown -R ubuntu:ubuntu /ws

USER ubuntu

CMD ["ros2", "launch", "demo_bringup", "perception.launch.py", "use_sim_time:=true"]
