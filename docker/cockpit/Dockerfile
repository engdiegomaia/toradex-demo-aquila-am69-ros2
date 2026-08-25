# Cockpit transport container — rosbridge_server + web_video_server.
#
# Builds for: linux/amd64 AND linux/arm64.
#
# This is the one graphical-adjacent role that IS allowed on the module. Nothing
# here renders: rosbridge is a WebSocket relay and web_video_server is an HTTP
# MJPEG encoder. Neither links OGRE 2 nor desktop OpenGL, so CLAUDE.md rule 1
# does not apply to this image the way it applies to viz and sim. That is the
# reason the cockpit backend is a separate role instead of extra packages in the
# viz image, which is amd64-only forever.
#
# Build (from the repo root, not from docker/):
#   docker build -f docker/cockpit/Dockerfile -t local/demo-aquila-cockpit:dev .
ARG REGISTRY=local
ARG TAG=dev
FROM ${REGISTRY}/demo-aquila-base:${TAG}

ARG ROS_DISTRO=jazzy
USER root

# rosbridge-suite is the metapackage: rosbridge_server (the WebSocket node),
# rosbridge_library (the JSON<->message converter) and rosapi (topic/type
# introspection). Installing only rosbridge-server leaves /rosapi/* unanswered,
# and the panels use it to distinguish "topic does not exist" from "topic exists
# and is silent".
#
# web-video-server carries the MJPEG endpoint. It pulls image_transport, which
# is a small dependency set and no renderer.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rosbridge-suite \
        ros-${ROS_DISTRO}-web-video-server \
    && rm -rf /var/lib/apt/lists/*

USER ubuntu

# The workspace is NOT rebuilt here. Unlike viz and sim, this role adds no
# package that the base build had to skip-key: it only needs demo_bringup's
# launch file, which the base image already installed.
CMD ["ros2", "launch", "demo_bringup", "cockpit.launch.py"]
