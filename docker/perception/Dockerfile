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
#
# image-transport + compressed-image-transport chegaram em 25/08/2026 para o
# camera_decompressor de demo_bringup/launch/perception.launch.py. Esta imagem
# nao tinha NENHUM dos dois: `ls /opt/ros/jazzy/lib/image_transport/` vinha
# vazio. Sem o plugin o `republish` sobe e nao publica nada, sem erro legivel.
#
# Isto puxa OpenCV por cv_bridge, o que engorda a imagem. Nao viola a regra 1:
# OpenCV e CPU, nao e stack de renderizacao -- mas a verificacao de regra 1 do
# `module.sh build` continua sendo o que decide isso, e nao este comentario.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-vision-msgs \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-compressed-image-transport \
    && rm -rf /var/lib/apt/lists/*

# Must carry the same value as the base build that produced the parent image.
# This RUN rebuilds the WHOLE workspace, so without it colcon reaches
# gz_quadruped_hardware and fails against a Gazebo that this headless role does
# not have (see docker/base/Dockerfile). The simulation role is the only image
# that needs to build that package.
ARG COLCON_IGNORE_PACKAGES="gz_quadruped_hardware"

WORKDIR /ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install --event-handlers console_direct+ \
        ${COLCON_IGNORE_PACKAGES:+--packages-ignore ${COLCON_IGNORE_PACKAGES}} \
    && rm -rf /ws/log \
    && chown -R ubuntu:ubuntu /ws

USER ubuntu

CMD ["ros2", "launch", "demo_bringup", "perception.launch.py", "use_sim_time:=true"]
