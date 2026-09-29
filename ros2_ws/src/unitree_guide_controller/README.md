# Unitree Guide Controller

This is a `ros2_control` controller based on Unitree Guide. The original
Unitree Guide project can be found [here](https://github.com/unitreerobotics/unitree_guide).
Kinematics and dynamics are computed with KDL, so controller behavior differs
from the original (occasionally quite unstable).

This package is vendored from `legubiao/quadruped_ros2_control` and has been
edited from ML3.5 phase F4 onward to fit this project's Go2 gait. See
[`PROVENANCE.md`](PROVENANCE.md) for origin, license and the exact list of
local edits.

Tested environment:

* Ubuntu 24.04
  * ROS 2 Jazzy
* Ubuntu 22.04
  * ROS 2 Humble

[![](http://i1.hdslb.com/bfs/archive/310e6208920985ac43015b2da31c01ec15e2c5f9.jpg)](https://www.bilibili.com/video/BV1aJbAeZEuo/)

## 1. Interfaces

Required hardware interfaces:

* command:
  * joint position
  * joint velocity
  * joint effort
  * KP
  * KD
* state:
  * joint effort
  * joint position
  * joint velocity
  * IMU sensor
    * linear acceleration
    * angular velocity
    * orientation

## 2. Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to unitree_guide_controller
```

## 3. Launch

The launch files below are the upstream standalone spikes kept for
zero-diff vendoring; this project's own entry points live under
`demo_bringup` and `demo_simulation` (see the repository root `CLAUDE.md`).

### 3.1 Mujoco simulation

> **Note:** launch [Unitree Mujoco C++ Simulation](https://github.com/legubiao/unitree_mujoco)
> before launching the controller.

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch unitree_guide_controller mujoco.launch.py pkg_description:=go2_description
```

### 3.2 Gazebo Harmonic

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch unitree_guide_controller gazebo.launch.py pkg_description:=go2_description
```
