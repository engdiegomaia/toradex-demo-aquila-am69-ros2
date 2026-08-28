# S6 — Interactive maze with click-to-set goals

World: `quadruped_maze11.sdf` · x86 workstation · Nav2 + RViz2

This workflow drives the Go2 using goals without manually publishing velocity.
Gazebo runs in the container and Nav2/RViz on the x86 host. RViz's `GoalTool`
turns each click-and-drag into a `NavigateToPose` action; the robot plans,
avoids obstacles using the costmap, and starts trotting on its own.

## 1. Prepare the maze models (once)

The maze STL is an external dependency and is not versioned in the project—the
upstream `package.xml` declares `<license>TODO</license>` and there is no LICENSE
file. Download it outside the repository, **to a stable path**:

```bash
git clone --depth 1 https://github.com/cafemesa/ros_maze_worlds.git \
  ~/ros_maze_worlds
```

If the directory already exists, update it with `git -C ~/ros_maze_worlds pull
--ff-only`. The script must receive the `models` directory, not the clone root.

**Do not use `/tmp`.** A reboot during a demo deletes the clone, and the symptom
is not an error: Gazebo starts, but the maze simply does not appear.

Confirm that the maze fits this robot before running:

```bash
python3 scripts/maze_fit.py --models ~/ros_maze_worlds/models maze11
```

Wait for `VEREDITO: SERVE`. The script also prints the recommended `<pose>`—the
source of the value in the SDF. Method and figures are in
[`docs/results/ml35-labirinto.md`](../../results/ml35-labirinto.md).

## 2. Start Gazebo

In the first terminal, at the repository root:

```bash
export MAZE_MODELS=~/ros_maze_worlds/models
./scripts/run_quadruped_sim.sh quadruped_maze11.sdf
```

The script prints two confirmation lines before starting:

```
Modelos externos: /home/você/ros_maze_worlds/models -> /maze/models
Yaw de nascimento: 1.5708 rad
```

Wait for the log to reach `state=fixed stand` and for the world to appear as
`World [quadruped_maze11] initialized`.

**The starting point is the maze's lower-right corner**, so the demo starts at
one end and crosses the entire maze. At that corner, `+x` is a wall 16 cm away,
and the robot spawns facing `+x`; the world therefore declares its own spawn yaw
in a `<!-- go2_spawn_yaw: 1.5708 -->` line, which the script reads. **There is
nothing to pass on the command line**; `GO2_SPAWN_YAW=<rad>` overrides it if you
want to try another heading. Confirm in `scenario_check` that the pose reports
`yaw=90.0 deg`.

Without `MAZE_MODELS`, `run_quadruped_sim.sh` **rejects** any
`quadruped_maze*.sdf` world and explains why. This guard exists because the
failure it prevents is silent: an unresolved mesh produces one warning line in
the middle of the log, not a fatal error, and the robot then walks in an open
field—a run that looks like perfect obstacle avoidance.

## 3. Open Nav2 and interactive RViz

In the second terminal, on the x86 host:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select demo_navigation demo_bringup
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=69
export ROS_LOG_DIR=/tmp
ros2 launch demo_bringup maze_nav_rviz.launch.py
```

The readiness gate is `Managed nodes are active`. Do not start `demo_routine`,
`patrol_commander`, or any other `/demo/cmd_vel` publisher alongside this
launch.

## 4. Send the robot to a location

In RViz:

1. Use the top view (`TopDownOrtho`) and leave `Fixed Frame` set to `map`.
2. Select the **Nav2 Goal** tool (target/arrow icon).
3. Click the destination and drag toward the final heading; release to send.
4. Click and drag another goal to cancel/replace the current one.

Goals must remain inside corridors and within lidar range. The costmap is
rolling and there is no static map: clicking a wall or an area not yet observed
may cause the planner to reject the goal.

### Where goals fit in `maze11`

The navigable component in robot coordinates (the robot spawns at 0,0 in the
lower-right corner, so the entire maze lies in `−x` and `+y`):

```
x[-9,87 ... +0,16]     y[-0,94 ... +9,87]
```

Goals measured at corridor centers, within the 8 m accepted by
`patrol_commander`; these are also the defaults in `nav_trial.py`:

```
(0.00, 8.00)   (-8.00, 0.00)   (-1.60, 1.60)   (-5.83, 4.91)
```

Two restrictions that are not announced:

- **A 1,20 m corridor with 21,7 cm clearance on each side.** With a
  `robot_radius` of 0,38 m and `inflation_radius` of 0,55 m, almost the entire
  corridor carries cost; the cost-free strip is the centerline. The planner
  works under these conditions, but it rejects goals placed near the wall.
- **`patrol_commander` rejects goals beyond 8 m** (`MAX_GOAL_RADIUS_M`). The
  `y = −10,38` end lies outside that radius. RViz does not impose this limit,
  but the global costmap is a rolling window: a distant goal is *accepted* and
  then fails near the edge.

### Change the maze

The 11 upstream mazes are not interchangeable: each has its own pose, and
**reusing another maze's pose places the robot inside a wall** without a Gazebo
error.

| maze | scale | `<pose>` | yaw | navigable area |
| --- | --- | --- | --- | --- |
| `maze10` (`quadruped_maze.sdf`) | 0,002 | `-2.670 3.061 0 0 0 0` | 0 | 18,4 m² |
| **`maze11`** (`quadruped_maze11.sdf`) | 0,002 | `-11.672 11.649 0 0 0 0` | 1,5708 | 35,4 m² |

For a new maze, measure rather than estimate:

```bash
python3 scripts/maze_fit.py --models ~/ros_maze_worlds/models maze7 \
  --start se --goals 4
```

The script prints the `<pose>`, the recommended `yaw:=` (by measuring free space
in all four directions from the starting cell), and goals at corridor centers.
`--start` accepts `run` (the longest clear path in `+x`, for a straight-walking
trial) or a corner: `se`, `ne`, `nw`, `sw`.

STLs 1 through 7 use **Y-up** and would require `roll 1.5708`; 8 through 11 are
already Z-up and use `rpy 0 0 0`. This is why the worlds in this project
reference the mesh directly—an `<include>model://mazeN` lays the maze on its
side because the upstream `model.sdf` declares roll for all of them.

## 5. Displays already open

The automatically loaded `demo_view.rviz` file enables:

- **TF:** `map`, `odom`, `base`, `trunk`, `lidar`, `front_camera`, and `imu_link`;
- **LaserScan:** `/demo/scan`;
- **Lidar PointCloud:** `/demo/scan_cloud` (the 3D cloud used by the costmap);
- **Go2Camera:** `/demo/camera/image_raw`.

To check from the terminal, measure the contract without publishing commands:

```bash
cd ..
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=69 ROS_LOG_DIR=/tmp
python3 scripts/scenario_check.py --seconds 10
```

The expected result is `PASSOU: 0 problema(s)`, camera at approximately 10 Hz,
lidar at 10 Hz, and TF rooted at `map`.

## 6. Shut down

Close RViz/Nav2 with `Ctrl-C`, then the Gazebo terminal with `Ctrl-C`. Do not
start another simulation until the `aquila-go2` container disappears from
`docker ps`.

This run uses Gazebo's `/demo/odom` as ground truth to close `odom → base`; it
validates planning and perception in simulation, not localization or the real
hardware's state estimator.
