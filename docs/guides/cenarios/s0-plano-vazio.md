# S0 — Empty plane

World: `quadruped_empty.sdf` · x86 workstation · ~4 min

**This is the reference for everything.** No other scenario means anything
without a run of this one on the same day: gait figures are comparable only
between runs at the same RTF, and this is where the day's RTF and baseline are
established.

## Purpose

Infinite flat ground, with nothing at lidar height. It isolates the gait:
anything that appears in other scenarios but not here comes from the world, not
the controller.

## Run

```bash
# terminal 1
./scripts/run_quadruped_sim.sh quadruped_empty.sdf

# terminal 2
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=69
python3 scripts/scenario_check.py --seconds 20

# terminal 2 -- instrumented walk
./scripts/gait_trial.sh /tmp/s0.csv --v-cmd 0.10 --w-cmd 0.0 \
  --cycles 5 --walk 8 --hold 8
```

Wait for `state=fixed stand` in terminal 1 before issuing any command. Sending a
command before that produces a robot that never starts trotting, with no error
to explain why.

## Acceptance

Measured on 20/08/2026, RTF 1,00:

| measurement | value | acceptance |
| --- | --- | --- |
| `/clock` | 1000 Hz | > 50 |
| `/demo/imu` | 1000 Hz | > 50 |
| `/demo/odom` | 50 Hz | > 10 |
| `/demo/scan` | 10 Hz | > 5 |
| `/demo/camera/image_raw` | 10 Hz, 640×480 rgb8 | > 5, geometry = `camera_info` |
| `RECOVER` | **0** | 0 |
| peak tilt while walking | 1,08° | < 3° |
| `z` | 0,343–0,359 m | range < 3 cm |
| heading drift over 5 cycles | −0,7° | < 5° |
| average speed | 0,1115 m/s | 0,10 commanded, 97–115% |

The full report outputs `PASSOU: 0 problema(s)`.

## Scenario-specific pitfall

**The lidar returns 7% valid beams, and that is correct.** Measured: 45–46 of
640 beams, between 4,66 and 9,78 m. The sensor is not faulty, nor is "almost
nothing working": the lower beams are hitting the **ground plane** several
meters away. The other 93% point toward the horizon and return infinity because
there is nothing at scan-plane height.

If you use this world to test the costmap, it fills with a ring of ground at
5–10 m and no obstacles. Use S3 for actual obstacles.

**The TF tree is complete within the robot and open at the top.** Measured: 20
edges, 8 static, root `base`; `base` → `trunk` → `lidar`, `imu_link`,
`front_camera`, and the four legs down to the feet. Only `odom → base` and
`map → odom` are missing.

Be careful when measuring: `/tf_static` uses `TRANSIENT_LOCAL` durability. The
fixed joints are published **once** when `robot_state_publisher` starts and are
retained for later subscribers. A subscriber with the default (`VOLATILE`) QoS
receives nothing and concludes that the tree lacks its fixed joints. The first
version of `scenario_check.py` made exactly this mistake and reported 12 edges
rooted at `trunk`, which was incorrect.
