#!/usr/bin/env python3
"""
Drive and record one gait trial on the Go2, from inside the sim container.

Runs on: x86 workstation ONLY, inside `aquila-go2`. Needs /demo/odom and
/demo/cmd_vel, so it is a sim-side tool and never ships to the module.

    docker exec -i aquila-go2 bash -lc \
      '. /opt/ros/jazzy/setup.sh; . /test/install/setup.sh; python3 - "$@"' \
      _ --v-cmd 0.10 --cycles 5 < scripts/gait_trial.py > run.csv

or, with the wrapper that does the same plumbing: scripts/gait_trial.sh.

WHY THIS EXISTS

Every gait measurement recorded in docs/results/ml35-f4-parcial.md was taken
with an ad-hoc recorder retyped per run. Two costs followed. First, one whole
sweep was lost measuring a robot that had already tipped over and was being
dragged: the numbers looked plausible because nothing checked the body height
before believing them. Second, sampling was paced by the wall clock while the
quantity of interest is paced by sim time, and the two are the same number only
at RTF 1.

So this script owns three things the retyped version kept getting wrong:

1. `z > --min-z` is a precondition, checked before the trial and again before
   every cycle. A fallen robot aborts the run instead of producing data.
2. Sim time and wall clock go on the same CSV line, and the phase schedule is
   paced by SIM time. A trial asking for 8 s of walking gets 8 s of simulated
   walking whatever the RTF is doing.
3. Stopping means not publishing. The bridge's watchdog is the stop mechanism
   (twist_to_inputs._CMD_TIMEOUT_S), so publishing zeros would test a path the
   robot never takes when Nav2 finishes a goal or a link dies.

COMMAND UNITS

`--v-cmd` and `--w-cmd` are the body command in SI, as StateTrotting sees it.
The conversion to Twist is not identity and is not documented anywhere the
operator can see it, so it is applied here:

    Twist.linear.x  = v_cmd / 0.4    (ly stick, unit gain, clamped to 0.5)
    Twist.angular.z = w_cmd / 0.5    (rx stick, negated internally, same clamp)

which caps the reachable command at v_cmd = 0.2 m/s and w_cmd = 0.25 rad/s.
Asking for more is refused up front rather than silently clamped downstream.
"""

import argparse
import math
import sys

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

# twist_to_inputs: unit gain on the stick, clamped, then StateTrotting's
# invNormalize against its own limits. Kept as named constants because a wrong
# factor here does not fail, it just measures the wrong speed.
_STICK_CLAMP = 0.5
_V_PER_STICK = 0.4  # m/s per unit of ly
_W_PER_STICK = 0.5  # rad/s per unit of rx


def _yaw_and_tilt(q) -> tuple[float, float]:
    """Yaw and body-z tilt away from gravity, both in degrees.

    Tilt is acos(R22) off the quaternion, the same definition
    StateTrotting::updateMotionMode uses, so the two numbers are comparable.
    """
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    r22 = max(-1.0, min(1.0, 1.0 - 2.0 * (q.x * q.x + q.y * q.y)))
    return math.degrees(yaw), math.degrees(math.acos(r22))


class GaitTrial(Node):
    def __init__(self, args) -> None:
        super().__init__('gait_trial')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.args = args
        self.pose = None
        self.rows: list[tuple] = []

        self.create_subscription(
            Odometry, '/demo/odom', self._on_odom,
            QoSPresetProfiles.SENSOR_DATA.value)
        self.cmd_pub = self.create_publisher(Twist, '/demo/cmd_vel', 10)

    def _on_odom(self, msg: Odometry) -> None:
        self.pose = msg.pose.pose

    # -- clocks ------------------------------------------------------------

    def sim_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def wall_s() -> float:
        return rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.SYSTEM_TIME) \
            .now().nanoseconds / 1e9

    # -- trial -------------------------------------------------------------

    def wait_for_stand(self) -> None:
        """Block until the robot is up, or give up loudly.

        Not a sleep: twist_to_inputs walks the FSM up with 5 s holds per
        transition, and how long that takes depends on the RTF. Sleeping a fixed
        amount and then measuring is how a trial ends up recording a robot that
        is still on its belly.
        """
        deadline = self.wall_s() + self.args.wait_stand
        standing_since = None

        while self.wall_s() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is None:
                continue
            if self.pose.position.z > self.args.min_z:
                standing_since = standing_since or self.sim_s()
                if self.sim_s() - standing_since >= 1.0:
                    return
            else:
                standing_since = None

        if self.pose is None:
            raise SystemExit('no /demo/odom in %.0f s: is the sim up, and is '
                             'ROS_DOMAIN_ID 69?' % self.args.wait_stand)
        raise SystemExit('robot never reached z > %.2f m (last z = %.3f m) in '
                         '%.0f s: it is not standing, so there is nothing to '
                         'measure' % (self.args.min_z, self.pose.position.z,
                                      self.args.wait_stand))

    def require_upright(self, when: str) -> None:
        z = self.pose.position.z
        if z <= self.args.min_z:
            self._dump()
            raise SystemExit('ABORT %s: z = %.3f m is at or below --min-z '
                             '%.2f m. The robot is down; anything measured '
                             'past this point describes a body being dragged.'
                             % (when, z, self.args.min_z))

    def phase(self, cycle: int, name: str, duration_s: float,
              twist: Twist | None) -> None:
        """Run one phase for `duration_s` of SIM time, sampling as it goes."""
        start = self.sim_s()
        next_pub = start
        next_sample = start

        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            now = self.sim_s()
            if now - start >= duration_s:
                return

            if twist is not None and now >= next_pub:
                self.cmd_pub.publish(twist)
                next_pub = now + 1.0 / self.args.cmd_rate

            if now >= next_sample:
                self._sample(cycle, name)
                next_sample = now + 1.0 / self.args.sample_rate

    def _sample(self, cycle: int, phase: str) -> None:
        if self.pose is None:
            return
        p = self.pose.position
        yaw, tilt = _yaw_and_tilt(self.pose.orientation)
        self.rows.append((cycle, phase, round(self.sim_s(), 3),
                          round(self.wall_s(), 3), round(p.x, 4), round(p.y, 4),
                          round(p.z, 4), round(yaw, 2), round(tilt, 2)))

    def run(self) -> None:
        self.wait_for_stand()
        self.require_upright('before the trial')

        twist = Twist()
        twist.linear.x = self.args.v_cmd / _V_PER_STICK
        twist.angular.z = self.args.w_cmd / _W_PER_STICK

        print('# v_cmd=%.3f m/s w_cmd=%.3f rad/s -> Twist(linear.x=%.4f, '
              'angular.z=%.4f), %d cycles of %.1f s walk / %.1f s hold'
              % (self.args.v_cmd, self.args.w_cmd, twist.linear.x,
                 twist.angular.z, self.args.cycles, self.args.walk,
                 self.args.hold), file=sys.stderr)

        for cycle in range(1, self.args.cycles + 1):
            self.require_upright('before cycle %d' % cycle)
            self.phase(cycle, 'walk', self.args.walk, twist)
            # No zero Twist here on purpose: silence is the stop command.
            self.phase(cycle, 'hold', self.args.hold, None)

        self._dump()
        self._summarize()

    # -- output ------------------------------------------------------------

    def _dump(self) -> None:
        out = sys.stdout
        print('cycle,phase,t_sim,t_wall,x,y,z,yaw_deg,tilt_deg', file=out)
        for row in self.rows:
            print(','.join(str(v) for v in row), file=out)

    def _summarize(self) -> None:
        if len(self.rows) < 2:
            print('no samples: nothing to summarize', file=sys.stderr)
            return

        path = sum(
            math.hypot(b[4] - a[4], b[5] - a[5])
            for a, b in zip(self.rows, self.rows[1:]))
        first, last = self.rows[0], self.rows[-1]
        net = math.hypot(last[4] - first[4], last[5] - first[5])

        walk = [r for r in self.rows if r[1] == 'walk']
        hold = [r for r in self.rows if r[1] == 'hold']
        sim_span = last[2] - first[2]
        wall_span = last[3] - first[3]

        def peak_tilt(rows):
            return max((r[8] for r in rows), default=float('nan'))

        print('\n'.join([
            '',
            'path length      %.3f m' % path,
            'net displacement %.3f m' % net,
            'mean speed       %.4f m/s (path / sim time)'
            % (path / sim_span if sim_span else float('nan')),
            'yaw             %.1f -> %.1f deg (drift %.1f)'
            % (first[7], last[7], last[7] - first[7]),
            'peak tilt        walk %.2f deg, hold %.2f deg'
            % (peak_tilt(walk), peak_tilt(hold)),
            'z                min %.3f m, max %.3f m'
            % (min(r[6] for r in self.rows), max(r[6] for r in self.rows)),
            'sim %.1f s in %.1f s wall (RTF %.2f) -- gait numbers are only '
            'comparable between runs at the same RTF'
            % (sim_span, wall_span, sim_span / wall_span if wall_span else 0.0),
            '%d samples' % len(self.rows),
        ]), file=sys.stderr)


def _parse_args(argv) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--v-cmd', type=float, default=0.10,
                   help='forward body command in m/s (max %.2f)'
                        % (_STICK_CLAMP * _V_PER_STICK))
    p.add_argument('--w-cmd', type=float, default=0.0,
                   help='yaw rate command in rad/s (max %.2f)'
                        % (_STICK_CLAMP * _W_PER_STICK))
    p.add_argument('--cycles', type=int, default=5)
    p.add_argument('--walk', type=float, default=8.0,
                   help='seconds of SIM time walking per cycle')
    p.add_argument('--hold', type=float, default=8.0,
                   help='seconds of SIM time standing per cycle')
    p.add_argument('--min-z', type=float, default=0.30,
                   help='body height below which the robot counts as down')
    p.add_argument('--wait-stand', type=float, default=60.0,
                   help='wall seconds to wait for the robot to stand up')
    p.add_argument('--cmd-rate', type=float, default=20.0)
    p.add_argument('--sample-rate', type=float, default=10.0)

    args = p.parse_args([a for a in argv if a != '_'])

    v_max = _STICK_CLAMP * _V_PER_STICK
    w_max = _STICK_CLAMP * _W_PER_STICK
    if abs(args.v_cmd) > v_max or abs(args.w_cmd) > w_max:
        p.error('command outside the bridge envelope (|v| <= %.2f m/s, '
                '|w| <= %.2f rad/s). twist_to_inputs would clamp it and the '
                'trial would record a speed nobody asked for.' % (v_max, w_max))
    return args


def main(argv=None) -> None:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init()
    node = GaitTrial(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
