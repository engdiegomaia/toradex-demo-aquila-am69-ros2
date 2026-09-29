"""
Converts the Twist in SI units that Nav2 produces into the contract's stick units.

Runs on the same machine as Nav2 (x86 workstation today, module in hil mode).

    ros2 run demo_bringup cmd_vel_si_to_stick

## Why this node has to exist

`/demo/cmd_vel` **is not in SI units**, despite being a `geometry_msgs/Twist`. It
carries normalized stick position, and the gait controller applies its own
gain when it receives it. The proof is arithmetic, in the vendored source:

    StateTrotting.cpp:192  v_cmd = invNormalize(ly, -0.4, +0.4)
    mathTools.h:10         invNormalize(v, min, max) = 0.4 * v   (minLim=-1, maxLim=1)
    twist_to_inputs.py:283 ly = linear.x                          (UNIT gain)

So `linear.x` reaches the robot multiplied by **0.4**. The same applies to yaw,
with gain **0.5**. The project has always known this and works around it:
`demo_routine.to_twist` divides by the gain before publishing, and
`docs/results/ml35-f4-parcial.md` records "command `linear.x = 0.25` (→
`v_cmd = 0.1 m/s`)".

Nav2 **cannot** work like this. It is not just a velocity publisher: MPPI
integrates `vx` as meters per second to predict where the robot will be. If the
published number is 0.4× what it models, every rollout misses the distance by
2.5×, and a horizon calibrated at 1.44 m becomes 0.58 m on the real plant.

Measured on 20/08/2026, with `vx_max: 0.15` interpreted as stick: over 300 s the
robot commanded at most 0.058 (mean 0.014), i.e. **0.006 m/s real** — below
the ~0.05 m/s minimum at which the gait stays stable, and 17× less than the
validated point of 0.10 m/s. No goal fit in the time budget.

## Where this node sits, and what it deliberately does NOT do

    Nav2 (collision_monitor)  --/demo/cmd_vel_si-->  THIS NODE  --/demo/cmd_vel-->  plant
                                   SI                              stick

It **does not change the contract** of `/demo/cmd_vel` and **does not touch the
plant**. `demo_routine` and `gait_trial.sh` keep publishing stick values
directly on `/demo/cmd_vel`, unchanged, and every number already recorded in
the results keeps meaning what it meant.

The alternative — making `twist_to_inputs` accept SI — would make
`/demo/cmd_vel` honest, and was rejected on scope grounds: it would change the
plant, both existing commanders, and the meaning of every `--v-cmd` already
recorded in `docs/results/`.

## The trap this node creates

There are now two `Twist` topics with different units. `_si` in the name is
the only defense, and it is weak. If someone wires Nav2 directly into
`/demo/cmd_vel`, the robot moves at 40% of what was requested and nothing
flags it. The symptom is exactly what was measured above: a slow robot that
never arrives, with no error in any log.

`/demo/cmd_vel` still has **one** publisher at a time. This node is one of them.
"""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

# Controller gains, from `trot.v_x_limit` / `v_y_limit` / `w_yaw_limit` in
# `demo_simulation/config/gait_go2.yaml`, which are [-0.4, 0.4], [-0.3, 0.3] and
# [-0.5, 0.5]. `invNormalize` with these limits reduces to multiplying by the
# upper limit. If you change the YAML, change it here too -- the node has no
# way to discover this on its own, because the parameter belongs to the
# controller, not to this node.
VX_PER_STICK = 0.4
VY_PER_STICK = 0.3
WZ_PER_STICK = 0.5

# Stick envelope proven stable (`_SAFE_STICK_LIMIT` in `twist_to_inputs.py`).
# Clamp here instead of letting the plant clamp silently: a request above the
# envelope becomes visible in this node.
STICK_CLAMP = 0.5


def to_stick(value: float, gain: float, clamp: float = STICK_CLAMP) -> float:
    """Divide an SI velocity by the controller gain and limit it to the envelope."""
    return max(-clamp, min(clamp, value / gain))


def convert(si: Twist) -> Twist:
    """Build the stick-equivalent Twist from an SI Twist."""
    out = Twist()
    out.linear.x = to_stick(si.linear.x, VX_PER_STICK)
    out.linear.y = to_stick(si.linear.y, VY_PER_STICK)
    # No sign flip here: `twist_to_inputs` already negates lx and rx, because
    # the upstream controller negates those axes internally. Negating again
    # would make the robot turn the wrong way, and Nav2 would correct by
    # increasing the error.
    out.angular.z = to_stick(si.angular.z, WZ_PER_STICK)
    return out


class CmdVelSiToStick(Node):
    """Republish `/demo/cmd_vel_si` as `/demo/cmd_vel` in stick units."""

    def __init__(self) -> None:
        """Open the SI subscription and the stick publication."""
        super().__init__('cmd_vel_si_to_stick')
        self._publisher = self.create_publisher(Twist, '/demo/cmd_vel', 10)
        self.create_subscription(Twist, '/demo/cmd_vel_si', self._on_si, 10)
        self._forwarded = 0
        self._saturated = 0
        self.get_logger().info(
            'converting /demo/cmd_vel_si (SI) to /demo/cmd_vel (stick) with '
            'gains vx=%.2f vy=%.2f wz=%.2f and clamp %.2f. DO NOT wire Nav2 '
            'directly into /demo/cmd_vel: the robot would move at %.0f%% of '
            'what was requested with no error logged.'
            % (VX_PER_STICK, VY_PER_STICK, WZ_PER_STICK, STICK_CLAMP,
               100.0 * VX_PER_STICK))

    def _on_si(self, message: Twist) -> None:
        """Republish a message in stick units, warning when it saturates."""
        out = convert(message)
        self._publisher.publish(out)

        self._forwarded += 1
        if abs(out.linear.x) >= STICK_CLAMP or abs(out.angular.z) >= STICK_CLAMP:
            self._saturated += 1
            # Saturating means Nav2 is requesting above the gait envelope.
            # It's not an error in this node, it's a sign that MPPI's limits
            # are too loose.
            self.get_logger().warning(
                'stick saturated: requested vx=%.3f wz=%.3f SI exceeds the '
                'envelope. Tighten vx_max/wz_max in nav2_params_go2.yaml (%d '
                'of %d messages)' % (message.linear.x, message.angular.z,
                                     self._saturated, self._forwarded),
                throttle_duration_sec=10.0)


def main(args=None) -> None:
    """Run the converter until interrupted."""
    rclpy.init(args=args)
    node = CmdVelSiToStick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
