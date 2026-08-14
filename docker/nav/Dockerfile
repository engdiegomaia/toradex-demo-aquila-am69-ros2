# Navigation container — Nav2 over a static map.
#
# Builds for: linux/amd64 AND linux/arm64. This is one of the two images that
# actually migrates to the Aquila AM69, so it carries nothing graphical: no
# Gazebo, no RViz2, no OGRE 2, no X11 dependency (CLAUDE.md rule 1).
#
# Keeping it lean is a module constraint, not a preference — container storage on
# Torizon lives on the data partition (guia-ml35-docker.md §4).
#
# Build (from the repo root):
#   docker build -f docker/nav/Dockerfile -t local/demo-aquila-nav:dev .
#
# arm64 build under QEMU. It produces a working image and measures NOTHING:
# no CPU, no latency, no thermals (CLAUDE.md rule 5).
#   docker buildx build --platform linux/arm64 \
#     -f docker/nav/Dockerfile -t ${REGISTRY}/demo-aquila-nav:${TAG} --push .
ARG REGISTRY=local
ARG TAG=dev
FROM ${REGISTRY}/demo-aquila-base:${TAG}

ARG ROS_DISTRO=jazzy
USER root

# ros-jazzy-nav2-bringup IS DELIBERATELY NOT INSTALLED HERE. Do not add it back.
#
# It is a demo/bringup convenience package and hard-depends on
# nav2-minimal-tb3-sim, nav2-minimal-tb4-sim, ros-gz-sim and ros-gz-bridge,
# which pull the entire Gazebo stack: libogre-1.9, gz-ogre-next-vendor,
# gz-rendering, gz-gui and 30+ more. Measured: it took this image to 3.7 GB and
# put OGRE 2 on an image that ships to the AM69, where desktop OpenGL does not
# exist (CLAUDE.md rule 1). --no-install-recommends does not help; these are
# hard Depends.
#
# The four launch files navigation.launch.py actually needs are vendored into
# demo_navigation/launch/nav2_vendored/ instead — same upstream code, Apache-2.0,
# only the package-root paths re-rooted. See that directory's README.md.
#
# ros-jazzy-navigation2 (the metapackage) IS ALSO NOT INSTALLED, for the same
# reason one level down: it depends on nav2-rviz-plugins, which pulls
# rviz-ogre-vendor — another OGRE build (17 MB plus the rviz stack) on a headless
# module image. An RViz *plugin* library has no purpose in the nav container;
# RViz itself lives in the viz container, x86 only.
#
# So the servers are listed explicitly. The list is not arbitrary — it is
# exactly what gets loaded at runtime, from two sources:
#
#   1. Servers spawned by the vendored launch files. Derived mechanically:
#      grep -hoE "package='[a-z0-9_]+'" launch/nav2_vendored/*.py | sort -u
#
#   2. Plugin libraries named in demo_navigation/config/nav2_params.yaml. These
#      are pluginlib loads, so a missing one is NOT a build error — the server
#      starts, fails to load the plugin, and the lifecycle transition fails with
#      a message about the plugin name rather than the missing package:
#        nav2_mppi_controller    (controller_server FollowPath)
#        nav2_navfn_planner      (planner_server GridBased)
#        nav2_costmap_2d         (static/obstacle/inflation layers)
#        nav2_behavior_tree      (the BT nodes bt_navigator ticks)
#
# When nav2_params.yaml gains a plugin from a package not listed here, this list
# must grow with it. That coupling is the cost of not installing the metapackage.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-nav2-amcl \
        ros-${ROS_DISTRO}-nav2-behavior-tree \
        ros-${ROS_DISTRO}-nav2-behaviors \
        ros-${ROS_DISTRO}-nav2-bt-navigator \
        ros-${ROS_DISTRO}-nav2-collision-monitor \
        ros-${ROS_DISTRO}-nav2-controller \
        ros-${ROS_DISTRO}-nav2-costmap-2d \
        ros-${ROS_DISTRO}-nav2-lifecycle-manager \
        ros-${ROS_DISTRO}-nav2-map-server \
        ros-${ROS_DISTRO}-nav2-mppi-controller \
        ros-${ROS_DISTRO}-nav2-navfn-planner \
        ros-${ROS_DISTRO}-nav2-planner \
        ros-${ROS_DISTRO}-nav2-route \
        ros-${ROS_DISTRO}-nav2-smoother \
        ros-${ROS_DISTRO}-nav2-velocity-smoother \
        ros-${ROS_DISTRO}-nav2-waypoint-follower \
        ros-${ROS_DISTRO}-opennav-docking \
        ros-${ROS_DISTRO}-rclcpp-components \
    && rm -rf /var/lib/apt/lists/*

# Rebuild the overlay so demo_navigation and demo_bringup resolve against the
# Nav2 packages the base image skipped.
WORKDIR /ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install --event-handlers console_direct+ \
    && rm -rf /ws/log \
    && chown -R ubuntu:ubuntu /ws

USER ubuntu

# use_sim_time:=true even on the module in hil mode — the module follows the
# host's /clock over DDS. A module on wall time while the host runs sim time
# produces TF extrapolation errors and Nav2 silently refusing goals.
CMD ["ros2", "launch", "demo_bringup", "nav.launch.py", "use_sim_time:=true"]
