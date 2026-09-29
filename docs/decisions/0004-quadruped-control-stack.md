# ADR 0004: `quadruped_ros2_control` and the Unitree Go2 model

- **Status:** Accepted

## Context

A legged platform was needed on ROS 2 Jazzy + Gazebo Harmonic, with a path to
Nav2. The candidates were CHAMP (ROS 1 only), two Go2/Nav2 forks (one without
Nav2, one without a declared license), an older Humble/Gazebo Classic fork
(which "corrected" odometry by doubling velocity in the estimator), and
`legubiao/quadruped_ros2_control`.

The Unitree A1 model was the first choice, but its description package had no
license (`<license>TODO</license>`, no LICENSE file).

## Decision

- Vendor the control layer from `legubiao/quadruped_ros2_control`, pinned to
  commit `5434c58`: `control_input_msgs`, `controller_common`,
  `unitree_guide_controller` and `gz_quadruped_hardware`. Upstream names are
  kept so the diff against upstream stays minimal.
- Use the Unitree Go2 model. Its meshes were traced by git blob hash to
  `unitreerobotics/unitree_ros` (BSD-3-Clause).

## Consequences

- There was no off-the-shelf Nav2 integration for this legged stack on
  Jazzy/Harmonic, so the project built one (`nav_quadruped.launch.py`,
  `cmd_vel_si_to_stick`, `odom_tf`).
- Provenance and license findings for each vendored package are recorded in
  its `PROVENANCE.md` / `README.md`. See also the repository [NOTICE](../../NOTICE).
