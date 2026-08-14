# Simulation container — Gazebo Harmonic, the robot, and the ROS<->gz bridge.
#
# Builds for: linux/amd64 ONLY.
#
# DO NOT ADD linux/arm64 TO THIS IMAGE. Gazebo is OGRE 2 and needs desktop
# OpenGL; the AM69 exposes only OpenGL ES 3.2 and Vulkan 1.2 (CLAUDE.md rule 1).
# An arm64 build of this image would succeed under QEMU and then fail on the
# module at runtime, which is the expensive way to discover the rule.
#
# Why sim is one container and not two: gz_ros2_control is a Gazebo plugin and
# loads the controller_manager inside the gz sim process. Splitting Gazebo from
# the controllers across containers is not possible without rewriting the
# integration (guia-ml35-docker.md §1). The plant is one unit.
#
# Build (from the repo root):
#   docker build -f docker/sim/Dockerfile -t local/demo-aquila-sim:dev .
ARG REGISTRY=local
ARG TAG=dev
FROM ${REGISTRY}/demo-aquila-base:${TAG}

ARG ROS_DISTRO=jazzy
USER root

# ros_gz_sim / ros_gz_bridge: the simulator and the bridge.
# nav2_minimal_tb4_sim: the warehouse world. NOT vendored into this repo — it is
#   Apache-2.0 and ~40 MB of meshes, and simulation.launch.py resolves its
#   default world through this package's share path.
# nav2_minimal_tb4_description: the meshes demo_description wraps for use_meshes.
# robot_state_publisher / xacro: expand the URDF and publish the fixed TF edges.
#
# ros2_control, ros2_controllers and gz_ros2_control are NOT installed here.
# They are the F2 prerequisite (docs/ml35/estado-fases.md) and F1 is diff-drive
# only, which uses Gazebo's native gz-sim-diff-drive-system plugin and needs no
# controller_manager. Adding them now would put untested weight in the image and
# blur what F2 is actually testing.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-gz-sim \
        ros-${ROS_DISTRO}-ros-gz-bridge \
        ros-${ROS_DISTRO}-nav2-minimal-tb4-sim \
        ros-${ROS_DISTRO}-nav2-minimal-tb4-description \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

# Rebuild the overlay now that the simulation dependencies resolve. The base
# image skipped them (--skip-keys), so demo_simulation built without them
# present; this makes the installed overlay consistent with the packages here.
WORKDIR /ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install --event-handlers console_direct+ \
    && rm -rf /ws/log

# Gazebo writes ~/.gz and ~/.ignition at startup. With a read-only or
# root-owned home the GUI fails in ways that do not name the home directory.
RUN chown -R ubuntu:ubuntu /ws /home/ubuntu

USER ubuntu

# Gazebo picks its render backend from the environment. Left unset inside a
# container it can select a software path and drop to single-digit FPS while
# appearing to work — slow, not broken, which is harder to diagnose.
# ogre2 is the default and is stated explicitly so a change is a visible edit.
ENV GZ_SIM_RENDER_ENGINE=ogre2

# Where Gazebo resolves model:// URIs. REQUIRED IN THE CONTAINER.
#
# demo_description references the TurtleBot 4 meshes as
# model://nav2_minimal_tb4_description/meshes/*.dae. Natively this resolves
# because the ambient ROS environment contributes the share path; inside the
# container GZ_SIM_RESOURCE_PATH is empty and every mesh fails to load with
#   [Err] [SceneManager.cc:426] Failed to load geometry for visual: ...
#   [Wrn] [Util.cc:859] Failed to load mesh from [model://...]
# The robot still spawns and still has correct collision and inertia, so physics
# and navigation behave — it is the visual bodies that go missing. A robot that
# is invisible in the GUI but present to the planner is a confusing way to spend
# an afternoon.
ENV GZ_SIM_RESOURCE_PATH=/opt/ros/${ROS_DISTRO}/share

CMD ["ros2", "launch", "demo_bringup", "sim.launch.py"]
