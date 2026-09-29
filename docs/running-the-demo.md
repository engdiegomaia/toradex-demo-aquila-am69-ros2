# Running the Maze Demo

This is the operator procedure for the reference demo: a simulated Unitree Go2
maps an unknown maze and searches for its exit. Nav2, SLAM and perception run
on the Toradex Aquila AM69, and the operator uses the web cockpit.

It assumes the one-time setup in [getting-started.md](getting-started.md) and
[hil-deployment.md](hil-deployment.md) is done. For a single workstation,
replace steps 2–3 with the `learn` command in the getting-started guide.

> **Scope.** This release is a *supervised* exploration demo. The robot
> explores autonomously and usually gets far into the maze, but a complete
> autonomous escape has not yet been demonstrated. See [validation.md](validation.md).

## 1. Pre-flight

```bash
docker ps -a --format '{{.Names}}\t{{.Status}}'   # no stale stacks from a previous session
ip -br link                                        # the Ethernet link to the module is UP
```

If the host is a laptop, resuming from suspend can drop both network
interfaces. Check the link carrier before suspecting DDS.

## 2. Host side

```bash
xhost +local:docker
docker compose -f docker/compose.host.yml up -d sim cockpit hmi
```

Wait until Gazebo shows the robot standing at the spawn pose.

## 3. Module side

```bash
scripts/module.sh up
scripts/module.sh verify        # all four stages must pass
```

## 4. Open the cockpit

Open <http://localhost:8081>. Before starting, check that:

- the link indicator is connected and no panel is stale;
- the navigation panel shows the robot at the origin with a growing SLAM map;
- the telemetry strip shows module CPU and temperature.

## 5. Explore

Press **Start search** in the navigation panel. The equivalent CLI command is:

```bash
ros2 service call /demo/exploration/start std_srvs/srv/Trigger '{}'
```

Follow the explorer state in the navigation panel. The robot:

1. picks frontier goals on the live SLAM map and walks to them;
2. backtracks along breadcrumbs when a region is exhausted;
3. switches to homing when the exit AprilTag is confirmed by perception;
4. shows **EXIT CONFIRMED** once the escape validator sees
   it leave the maze.

To stop at any time:

```bash
ros2 service call /demo/exploration/cancel std_srvs/srv/Trigger '{}'
```

## 6. Run again

1. Cancel the exploration.
2. Press the simulation reset button (&#8634;) in the control bar, or call
   `/demo/sim/reset`.
3. Confirm that the robot is back near `(0, 0)`.
4. Run `scripts/module.sh up` and `scripts/module.sh verify` to recreate
   navigation with a fresh map.

Always reset the simulation *before* recreating navigation
([why](hil-deployment.md#restarting-a-run)).

## 7. Shut down

```bash
scripts/module.sh down
docker compose -f docker/compose.host.yml down
```

## Known limitations during a run

- The robot can zigzag around straight plans. This is MPPI oscillation: the
  explorer's watchdog cancels goals that genuinely stall.
- Long goals can time out when the frontier lies behind unmapped walls.
- **Homing towards the exit without a fresh view of the marker has made the
  robot fall once near the exit opening.** Supervise that phase.
