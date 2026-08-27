# Navigation container — robot-selected Nav2 stack (Go2 or diff-drive).
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

# Ubuntu empacota o runtime do slam_toolbox e seu plugin RViz no mesmo .deb.
# Instalar o pacote normalmente arrasta rviz_common, rviz_rendering, OGRE e Qt
# para o AM69, embora async_slam_toolbox_node nao ligue nenhuma dessas libs.
# Extraimos o .deb e removemos apenas o plugin grafico; as dependencias CPU do
# runtime sao declaradas explicitamente no estagio final abaixo.
FROM ${REGISTRY}/demo-aquila-base:${TAG} AS slam-toolbox-runtime

ARG ROS_DISTRO=jazzy
USER root
RUN apt-get update \
    && cd /tmp \
    && apt-get download ros-${ROS_DISTRO}-slam-toolbox \
    && mkdir -p /slam-runtime \
    && dpkg-deb -x ros-${ROS_DISTRO}-slam-toolbox_*.deb /slam-runtime \
    && rm -f \
        /slam-runtime/opt/ros/${ROS_DISTRO}/lib/libSlamToolboxPlugin.so \
        /slam-runtime/opt/ros/${ROS_DISTRO}/share/rviz_plugins.xml \
        /slam-runtime/opt/ros/${ROS_DISTRO}/share/slam_toolbox/rviz_plugins.xml \
        /slam-runtime/opt/ros/${ROS_DISTRO}/share/slam_toolbox/config/slam_toolbox_default.rviz \
        /slam-runtime/opt/ros/${ROS_DISTRO}/share/ament_index/resource_index/rviz_common__pluginlib__plugin/slam_toolbox \
    && rm -rf /var/lib/apt/lists/* /tmp/ros-${ROS_DISTRO}-slam-toolbox_*.deb

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
        libboost-serialization1.83.0 \
        libboost-thread1.83.0 \
        libceres4t64 \
        libconsole-bridge1.0 \
        libgoogle-glog0v6t64 \
        liborocos-kdl1.5 \
        libtbb12 \
        libtinyxml2-10 \
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
        ros-${ROS_DISTRO}-bondcpp \
        ros-${ROS_DISTRO}-interactive-markers \
        ros-${ROS_DISTRO}-message-filters \
        ros-${ROS_DISTRO}-pointcloud-to-laserscan \
        ros-${ROS_DISTRO}-rclcpp-components \
        ros-${ROS_DISTRO}-tf2-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY --from=slam-toolbox-runtime /slam-runtime/ /

# Falha no build, nao na bancada: o executavel copiado deve resolver todas as
# libs e o payload headless nao pode reintroduzir o plugin RViz removido acima.
RUN ! ldd /opt/ros/${ROS_DISTRO}/lib/slam_toolbox/async_slam_toolbox_node \
        | grep -q 'not found' \
    && test ! -e /opt/ros/${ROS_DISTRO}/lib/libSlamToolboxPlugin.so

# Rebuild the overlay so demo_navigation and demo_bringup resolve against the
# Nav2 packages the base image skipped.
# This RUN rebuilds the WHOLE workspace, so without the default below colcon reaches
# gz_quadruped_hardware and fails against a Gazebo that the module image
# deliberately does not have (see docker/base/Dockerfile). The simulation role
# is the only image that needs to build that package.
ARG COLCON_IGNORE_PACKAGES="gz_quadruped_hardware"

WORKDIR /ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install --event-handlers console_direct+ \
        ${COLCON_IGNORE_PACKAGES:+--packages-ignore ${COLCON_IGNORE_PACKAGES}} \
    && rm -rf /ws/log \
    && chown -R ubuntu:ubuntu /ws

USER ubuntu

# use_sim_time:=true even on the module in hil mode — the module follows the
# host's /clock over DDS. A module on wall time while the host runs sim time
# produces TF extrapolation errors and Nav2 silently refusing goals.
# Standalone default matches Compose: Go2 is the ML3.5 default and the selector
# keeps the diff-drive launch available through robot_type:=diffdrive.
CMD ["ros2", "launch", "demo_bringup", "nav_select.launch.py", "robot_type:=quadruped", "use_sim_time:=true"]
