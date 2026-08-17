"""
Translate /demo/cmd_vel into what the gait controller actually accepts.

Runs on: x86 workstation ONLY, in the `sim` container, next to Gazebo.

WHY THIS NODE EXISTS

unitree_guide_controller does not subscribe to geometry_msgs/Twist. It
subscribes to control_input_msgs/Inputs on /control_input, which models a
gamepad: a `command` button code plus four normalized stick axes in [-1, 1].
Nothing in the stack speaks that message, and nothing should have to — the
topic contract (CLAUDE.md) says the plant is driven by /demo/cmd_vel, and Nav2
must not learn that the robot has legs.

So this node owns two jobs that the diff-drive plant never needed:

1. Walk the gait state machine up to TROTTING. A quadruped does not accept
   velocity commands from a cold start; it has to stand up first.
2. Convert Twist to normalized stick axes once trotting.

F3 SCOPE, AND WHAT IS DELIBERATELY NOT HERE

This is the F3 version: enough to prove the plant stands and walks. The mapping
below is linear and unclamped by any real velocity limit, which is exactly the
behaviour F2 measured as unstable above ~0.15 m/s (the robot loses balance and
the trunk drops from 0.34 m to 0.07-0.24 m). That is a tuning problem in this
mapping, not a defect in the controller, and fixing it properly means reading
the controller's own velocity limits and scaling against them — F4 work, where
the real contract bridge is built. F3's gate is "stands and walks at low gain".

Do not raise MAX_* here to make the robot look faster. It will fall over, and
it will do so without a single error in the log.
"""

from control_input_msgs.msg import Inputs
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

# Gait FSM button codes, read from the controller's own sources (not guessed):
# StatePassive::checkChange, StateFixedDown::checkChange and
# StateFixedStand::checkChange in unitree_guide_controller/src/FSM/.
#   PASSIVE --(2)--> FIXEDDOWN --(2)--> FIXEDSTAND --(4)--> TROTTING
_CMD_NONE = 0
_CMD_STAND_STEP = 2
_CMD_START_TROT = 4

# Wall-clock seconds to hold between FSM transitions.
#
# LOAD-BEARING, and 5 s is not padding. Each FSM state gates its own exit on
# percent_ >= 1.5 (roughly 1.2 s of sim time), and the robot also has to
# physically settle from the spawn drop. The F2 spike first fired all three
# transitions inside ~2 s of wall time; the robot ended up collapsed at
# z ~= 0.07 m, which is indistinguishable from "the robot never stood" unless
# you check the pose directly. Nothing in the log says anything is wrong.
_TRANSITION_HOLD_S = 5.0
_TICK_PERIOD_S = 1.0
_TICKS_PER_TRANSITION = int(_TRANSITION_HOLD_S / _TICK_PERIOD_S)


class TwistToInputs(Node):
    """Bridge /demo/cmd_vel -> /control_input, driving the gait FSM first."""

    def __init__(self) -> None:
        super().__init__('twist_to_inputs')

        self._publisher = self.create_publisher(Inputs, '/control_input', 10)
        self._subscription = self.create_subscription(
            Twist, '/demo/cmd_vel', self._on_twist, 10,
        )

        self._stage = 'settling'
        self._ticks = 0
        self._timer = self.create_timer(_TICK_PERIOD_S, self._advance_gait_fsm)

        self.get_logger().info(
            'twist_to_inputs up: /demo/cmd_vel -> /control_input. '
            'Standing the robot up before accepting velocity commands.'
        )

    def _advance_gait_fsm(self) -> None:
        """
        Walk PASSIVE -> FIXEDDOWN -> FIXEDSTAND -> TROTTING, then stop.

        Wall-clock paced on purpose — see _TRANSITION_HOLD_S.
        """
        self._ticks += 1
        if self._ticks < _TICKS_PER_TRANSITION:
            return
        self._ticks = 0

        if self._stage == 'settling':
            self._send_command(_CMD_STAND_STEP)
            self._stage = 'fixed_down'
            self.get_logger().info('gait FSM: passive -> fixed down')
        elif self._stage == 'fixed_down':
            self._send_command(_CMD_STAND_STEP)
            self._stage = 'fixed_stand'
            self.get_logger().info('gait FSM: fixed down -> fixed stand')
        elif self._stage == 'fixed_stand':
            self._send_command(_CMD_START_TROT)
            self._stage = 'trotting'
            self.get_logger().info(
                'gait FSM: fixed stand -> trotting. Now driven by /demo/cmd_vel.'
            )
            # Nothing left to sequence. Leaving the timer running would keep
            # publishing command codes and fight the velocity commands: a
            # stray command=2 forces the FSM back to FIXEDSTAND even from
            # stable trotting (StateTrotting::checkChange), which reads as the
            # robot randomly stopping.
            self._timer.cancel()

    def _send_command(self, command: int) -> None:
        message = Inputs()
        message.command = command
        self._publisher.publish(message)

    def _on_twist(self, twist: Twist) -> None:
        """Map Twist onto the controller's normalized stick axes."""
        # Velocity commands before TROTTING are not queued, they are dropped.
        # Forwarding them would inject axis values while the robot is still
        # standing up and knock it over mid-transition.
        if self._stage != 'trotting':
            return

        message = Inputs()
        message.command = _CMD_NONE
        # ly is forward/back and lx is strafe on the left stick; rx yaws.
        message.ly = _clamp(twist.linear.x)
        message.lx = _clamp(twist.linear.y)
        message.rx = _clamp(twist.angular.z)
        message.ry = 0.0
        self._publisher.publish(message)


def _clamp(value: float) -> float:
    """Clamp to the [-1, 1] range the Inputs message models."""
    return max(-1.0, min(1.0, float(value)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwistToInputs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
