#!/usr/bin/env bash
set -euo pipefail

# Usage: ./scripts/run_quadruped_sim.sh [world.sdf]
# The default is the image's built-in empty.sdf. Keep this terminal open.

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
world_path="${1:-quadruped_empty.sdf}"
container_name="aquila-go2"

if docker ps --format '{{.Names}}' | grep -qx "${container_name}"; then
  echo "Container ${container_name} já está em execução." >&2
  exit 1
fi

cd "${repo_dir}"
xhost +local:docker >/dev/null

docker run --rm --name "${container_name}" --network=host \
  -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e ROS_DOMAIN_ID=69 \
  -e GO2_WORLD="${world_path}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri:/dev/dri --group-add 992 \
  -v "${repo_dir}/ros2_ws/src:/proj/src:ro" \
  --entrypoint bash demo-sim:spike-go2 -c '
    set -e
    . /opt/ros/jazzy/setup.sh
    mkdir -p /test/src
    cd /test
    cp -r /proj/src/go2_description /proj/src/control_input_msgs \
          /proj/src/controller_common /proj/src/unitree_guide_controller \
          /proj/src/gz_quadruped_hardware /proj/src/demo_simulation src/
    colcon build --symlink-install
    . install/setup.sh
    world="${GO2_WORLD}"
    if [ "${world}" = quadruped_empty.sdf ]; then
      world=/test/install/demo_simulation/share/demo_simulation/worlds/quadruped_empty.sdf
    fi
    ros2 launch demo_simulation quadruped.launch.py gui:=true world="${world}"
  '
