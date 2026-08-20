# Tools container — teleop, ROS 2 CLI, colcon, pytest.
#
# Builds for: linux/amd64 AND linux/arm64. This is the shell used to inspect a
# running stack from either side of the host/module split:
#
#   docker compose -f compose.host.yml exec tools bash
#   ros2 topic list | grep /demo/
#   ros2 topic hz /demo/scan
#
# Nothing graphical, so it is safe on the module. teleop_twist_keyboard is
# terminal-only.
#
# Build (from the repo root):
#   docker build -f docker/tools/Dockerfile -t local/demo-aquila-tools:dev .
ARG REGISTRY=local
ARG TAG=dev
FROM ${REGISTRY}/demo-aquila-base:${TAG}

ARG ROS_DISTRO=jazzy
USER root

# teleop_twist_keyboard: drive /demo/cmd_vel by hand, which is how the contract
#   gets checked without Nav2 in the loop.
# demo_nav2_* / nav2_msgs: `ros2 action send_goal /navigate_to_pose` needs the
#   action definitions present to send the goal at all.
# ros2cli common extensions + colcon + pytest: run the test suite inside the
#   image the demo actually ships with, not just on the host.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
        ros-${ROS_DISTRO}-nav2-msgs \
        ros-${ROS_DISTRO}-tf2-tools \
        ros-${ROS_DISTRO}-tf2-ros \
        python3-colcon-common-extensions \
        python3-pytest \
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

# Must not exit: this service exists to be exec'd into. A container whose command
# returns immediately shows up as Exited and `compose exec` then fails with a
# message about the container not running, which reads like a compose problem
# rather than a finished command.
CMD ["sleep", "infinity"]
