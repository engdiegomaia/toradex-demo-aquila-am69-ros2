"""
std_srvs facade for simulator control.

Runs on: x86 workstation ONLY, in the `sim` container, next to
`sim_control_bridge`.

    /demo/sim/play             std_srvs/Trigger   (incoming)
    /demo/sim/pause            std_srvs/Trigger   (incoming)
    /demo/sim/reset            std_srvs/Trigger   (incoming)
    /demo/sim/control          ros_gz_interfaces/srv/ControlWorld   (outgoing)
    /demo/sim/set_entity_pose  ros_gz_interfaces/srv/SetEntityPose  (outgoing)

WHY RESET NO LONGER USES `reset.all`

`reset.all` DELETES THE ROBOT. Measured on 2026-08-26, world `quadruped_maze11`,
robot standing, with a single call to /demo/sim/reset:

    topic                      before     after
    /joint_states             999 Hz      dead
    /demo/imu                 996 Hz      dead
    /demo/odom               49.6 Hz      dead
    /demo/scan                 10 Hz      10 Hz
    /demo/camera/image_raw     10 Hz      10 Hz
    /clock                    999 Hz      997 Hz

    $ gz model -m demo_robot
    No model named <demo_robot> was found

The robot is INSERTED into the world after it loads, by `ros_gz_sim create`
(quadruped.launch.py). `reset.all` returns the world to its source SDF, and the
source SDF does not contain the robot -- nor the two scene cameras, which are
also inserted. What remains is a world without a plant.

And the failure mode is the worst possible: the clock keeps running at 999 Hz,
the lidar and camera keep publishing at 10 Hz (Gazebo leaves orphaned sensors
publishing), so the cockpit stays ENTIRELY green -- video, scene, map, clock --
pointing at a robot that no longer exists. Nothing in any log says the plant was
deleted. Recovering requires restarting the `sim` container.

WHAT RESET DOES NOW

It teleports the robot to the scenario spawn pose, through the same
`set_entity_pose` the scene cameras already use. Measured the same day:

    after the teleport:  /joint_states 999 Hz, /demo/imu 982 Hz, /demo/odom 50 Hz
    pose read from /demo/odom: x=1.000 (commanded x=1.0)

The plant survives intact. The clock does NOT go back to zero, and that is
deliberate: a backward time jump invalidates the Nav2 TF buffer and the
`controller_manager`, and nothing the operator wants from a reset ("put the
robot at the start") calls for it.

WHY THE TELEPORT ALONE IS NOT ENOUGH (quadruped)

"the robot resettles to walking height on its own" used to be written here and
it is FALSIFIED. Measured on 2026-08-26 in `quadruped_maze11`, after a reset:

    ground truth /demo/odom   z=0.131 m   versus 0.353 m walking height
    yaw                       141.7 deg   versus the 90 deg commanded
    position                  (-0.015, -0.220)  commanded (0.000, 0.000)
    Mz on the rail            100% of ticks, constant yaw residual

The robot does not resettle: it COLLAPSES and drags itself. The cause is not the
teleport but StateTrotting -- the HOLD reference (pcd_ and yaw_cmd_) is captured
only once, behind a latch that only a clean walking command releases, so after
the teleport the controller chases the PREVIOUS pose on a yaw axis that
saturates at ~5.3 N.m and "only drags the feet trying" (the comment is from
captureBodyReference itself). The estimator makes the picture worse without
being the cause: it is a KF of IMU + feet, with no absolute position, and so it
does NOT see the teleport in xy -- 1.8 m of divergence measured against ground
truth.

And there is a SECOND cause, which only shows up with Nav2 driving:
`SetEntityPose` repositions the body and PRESERVES VELOCITY. Measured the same
day, reset during a live /demo/cmd_vel stream at 10 Hz -- z went from 0.337 m to
0.162 m in one second, sliding 0.27 m, and stayed that way for the next 40 s. A
robot walking at ~0.2 m/s with its legs mid-swing is dropped from 0.15 m still
travelling.

So the reset STOPS THE ROBOT BEFORE teleporting, and does not re-anchor after:

    /demo/gait/hold     -> FIXEDSTAND, axes centred; the robot plants and stops
    wait GAIT_STOP_S
    set_entity_pose     -> teleport, with the robot at rest
    /demo/gait/resume   -> settles and returns to TROTTING, whose enter()
                           re-anchors pcd_ and yaw_cmd_ at the new pose

Both are served by twist_to_inputs, the only writer of /control_input.

The accumulated costmap is NOT cleared here. That is done by /demo/nav/reset,
which the cockpit exposes on its own navigation-reset button -- the separate
granularity is what allows repositioning the robot without taking Nav2 down.

WHY NOT CALL ControlWorld DIRECTLY FROM THE BROWSER

It was the first attempt and it fails, with a message worth recording:

    call_service InvalidModuleException: Unable to import ros_gz_interfaces.srv
    from package ros_gz_interfaces

rosbridge builds the request by importing the interfaces package INSIDE its own
container, and the cockpit container has no `ros_gz_interfaces` -- nor should
it. In M3 the cockpit is served by the Aquila, and in `deploy` mode there is no
Gazebo at all: installing the simulator interfaces there would carry to the
module the definition of something that, in that mode, does not exist.

So the browser boundary speaks std_srvs, which is ROS core and present in every
container, and the translation to the Gazebo vocabulary happens here -- on the
side that already has Gazebo. It is the same choice `scene_view_controller`
makes for the cameras: the browser sends intent, the simulator owns the type.

The `success=False` in the response is this facade's useful path. Without it, a
stopped simulator returned an import exception to the UI, and the operator saw
"failed to pause" with the wrong cause.
"""

import math
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.srv import ControlWorld, SetEntityPose
from std_srvs.srv import Trigger

CONTROL_SERVICE = '/demo/sim/control'
SET_POSE_SERVICE = '/demo/sim/set_entity_pose'
# Served by twist_to_inputs, which exists only on the legged plant. See the
# WHY THE TELEPORT ALONE IS NOT ENOUGH block in the header.
HOLD_SERVICE = '/demo/gait/hold'
RESUME_SERVICE = '/demo/gait/resume'

# How long to wait for the bridge to answer. Gazebo answers ControlWorld in a
# few milliseconds; this limit exists for the case where the service does not
# exist on the other side, where what matters is failing fast enough that the
# button does not look stuck.
CALL_TIMEOUT_S = 3.0

# Wait for the gait services. Short and deliberately separate from
# CALL_TIMEOUT_S: on the differential plant these services DO NOT EXIST (there
# is no gait), and their absence is a normal path, not a failure. The reset must
# not stall for 3 s because of it.
GAIT_TIMEOUT_S = 1.0

# How long to wait for the robot to STOP before teleporting.
#
# LOAD-BEARING. `SetEntityPose` repositions the body and preserves velocity:
# teleporting a walking quadruped drops it from 0.15 m still travelling at
# ~0.2 m/s with its legs mid-swing, and it falls. Measured on 2026-08-26 with a
# live /demo/cmd_vel stream at 10 Hz -- z went from 0.337 m to 0.162 m in one
# second, and stayed that way for the next 40 s.
#
# 2 s covers FIXEDSTAND's own gate (percent_ >= 1.5, ~1.2 s) plus braking. WALL
# clock, for the reason documented in _wait: this code may be running with the
# simulation paused, and the simulated clock is precisely what will not
# advance.
GAIT_STOP_S = 2.0


def _request(action: str) -> ControlWorld.Request:
    """
    Translate the intent into the WorldControl vocabulary.

    Only play and pause: `reset` no longer goes through here. See the WHY RESET
    NO LONGER USES `reset.all` block in the header -- the `reset.all` variant
    deletes the robot, and `time_only` jumps the clock backwards without moving
    the robot, which is worse than doing nothing.
    """
    request = ControlWorld.Request()
    if action == 'play':
        request.world_control.pause = False
    elif action == 'pause':
        request.world_control.pause = True
    else:
        raise ValueError(f'unknown action in WorldControl: {action}')
    return request


class SimControlRelay(Node):

    def __init__(self) -> None:
        super().__init__('sim_control_relay')

        # Reentrant group: the Trigger callbacks BLOCK waiting for the
        # ControlWorld response. In the default mutually exclusive group that
        # wait would keep the executor from processing the response itself, and
        # every click would time out.
        self._group = ReentrantCallbackGroup()

        self._client = self.create_client(
            ControlWorld, CONTROL_SERVICE, callback_group=self._group,
        )
        self._pose_client = self.create_client(
            SetEntityPose, SET_POSE_SERVICE, callback_group=self._group,
        )
        self._hold_client = self.create_client(
            Trigger, HOLD_SERVICE, callback_group=self._group,
        )
        self._resume_client = self.create_client(
            Trigger, RESUME_SERVICE, callback_group=self._group,
        )

        # Spawn pose and model name. They come from a PARAMETER, filled in by
        # sim_control.launch.py from `scenarios.spawn_pose`: the same place the
        # `-x/-y/-Y` of the `create` that spawned the robot come from.
        # Hard-coding (0, 0) here would send the robot back to the origin in any
        # world whose usable area is not at the origin -- the maze is exactly
        # that case, and the error would be silent (the robot reappears inside
        # a wall).
        self.declare_parameter('robot_name', 'demo_robot')
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        # Not the walking height: it is the SPAWN height, the same as the
        # create's `-z`. A quadruped placed at ground level interpenetrates the
        # floor and falls before the controller stabilises.
        self.declare_parameter('spawn_z', 0.5)
        self.declare_parameter('spawn_yaw', 0.0)

        for action in ('play', 'pause'):
            self.create_service(
                Trigger,
                f'/demo/sim/{action}',
                self._handler(action),
                callback_group=self._group,
            )
        self.create_service(
            Trigger, '/demo/sim/reset', self._handle_reset,
            callback_group=self._group,
        )

        self.get_logger().info(
            f'simulation facade ready: play/pause -> {CONTROL_SERVICE}, '
            f'reset -> {SET_POSE_SERVICE} '
            f'(teleports {self._robot_name()} to the spawn pose)'
        )

    def _robot_name(self) -> str:
        return str(self.get_parameter('robot_name').value)

    def _spawn(self) -> tuple:
        """Spawn (x, y, z, yaw), read at the instant of the click."""
        return (
            float(self.get_parameter('spawn_x').value),
            float(self.get_parameter('spawn_y').value),
            float(self.get_parameter('spawn_z').value),
            float(self.get_parameter('spawn_yaw').value),
        )

    def _handle_reset(self, request, response):
        """
        Return the robot to the spawn pose WITHOUT deleting it.

        Does not zero the clock and does not touch the world. The accumulated
        costmap is /demo/nav/reset's business, which has its own cockpit button.
        """
        del request
        if not self._pose_client.wait_for_service(timeout_sec=CALL_TIMEOUT_S):
            response.success = False
            response.message = (
                f'{SET_POSE_SERVICE} did not respond; is the ros_gz simulation '
                'control bridge up?'
            )
            return response

        x, y, z, yaw = self._spawn()
        name = self._robot_name()

        # STOP BEFORE TELEPORTING, never after. See GAIT_STOP_S: the teleport
        # preserves velocity, and a walking quadruped that is teleported falls.
        gait = self._hold_gait()
        if gait is not None:
            time.sleep(GAIT_STOP_S)

        pose_request = SetEntityPose.Request()
        pose_request.entity.name = name
        pose_request.pose.position.x = x
        pose_request.pose.position.y = y
        pose_request.pose.position.z = z
        # Pure yaw: a quadruped repositioned with roll or pitch falls.
        pose_request.pose.orientation.z = math.sin(yaw / 2.0)
        pose_request.pose.orientation.w = math.cos(yaw / 2.0)

        future = self._pose_client.call_async(pose_request)
        if not _wait(future, CALL_TIMEOUT_S):
            response.success = False
            response.message = f'{SET_POSE_SERVICE} timed out while repositioning {name}'
            # Returning here without resuming would leave the robot stuck in
            # FIXEDSTAND, accepting no command and with nothing in the log
            # saying why.
            self._resume_gait(gait)
            return response

        result = future.result()
        # Gazebo returns success=False when NO entity with that name EXISTS.
        # That is this response's useful path: it tells the operator the model
        # is not in the world, instead of leaving the button silent.
        response.success = bool(result and result.success)
        if not response.success:
            response.message = (
                f'Gazebo refused to reposition {name}: is there a model with that name?'
            )
            self.get_logger().warning(response.message)
            self._resume_gait(gait)
            return response

        pose = f'{name} repositioned at x={x:.3f} y={y:.3f} yaw={yaw:.4f}'
        response.message = f'{pose}; {self._resume_gait(gait)}'
        return response

    def _hold_gait(self):
        """
        Tell the gait to stop and hold still. Returns None if there is no gait.

        Best effort, by design: the differential plant has no gait to hold, and
        that absence is a normal path. But it must not be silent either -- on a
        quadruped, teleporting without stopping leaves the robot collapsed, and
        that is how the defect got this far. So the result goes into the reset's
        own message, which the cockpit shows.
        """
        if not self._hold_client.wait_for_service(timeout_sec=GAIT_TIMEOUT_S):
            return None
        return self._call_gait(self._hold_client, HOLD_SERVICE)

    def _resume_gait(self, held) -> str:
        """Return the gait to TROTTING, re-anchored. `held` comes from _hold_gait."""
        if held is None:
            return f'no {HOLD_SERVICE} (plant without gait): nothing to stop'
        return f'{held}; {self._call_gait(self._resume_client, RESUME_SERVICE)}'

    def _call_gait(self, client, name: str) -> str:
        future = client.call_async(Trigger.Request())
        if not _wait(future, GAIT_TIMEOUT_S):
            message = f'{name} timed out: the gait was NOT re-anchored'
            self.get_logger().warning(message)
            return message

        result = future.result()
        if not (result and result.success):
            message = f'{name} refused: the gait was NOT re-anchored'
            self.get_logger().warning(message)
            return message
        return str(result.message)

    def _handler(self, action: str):
        def handle(request, response):
            del request
            if not self._client.wait_for_service(timeout_sec=CALL_TIMEOUT_S):
                response.success = False
                response.message = (
                    f'{CONTROL_SERVICE} did not respond; is the sim container up?'
                )
                return response

            future = self._client.call_async(_request(action))
            if not _wait(future, CALL_TIMEOUT_S):
                response.success = False
                response.message = f'{CONTROL_SERVICE} timed out on {action}'
                return response

            result = future.result()
            response.success = bool(result and result.success)
            response.message = (
                f'{action} applied' if response.success
                else f'Gazebo refused {action}'
            )
            return response

        return handle


def _wait(future, timeout_s: float) -> bool:
    """
    Wait for the future without spinning the executor.

    `spin_until_future_complete` and `spin_once` do NOT work here: we are
    already inside an executor callback, and spinning it again over the same
    node is reentrancy on the wrong object. Whoever completes this future is
    another MultiThreadedExecutor thread -- the reentrant group exists exactly
    to allow that -- so all that is left for this thread is to sleep and look.

    The clock is the wall clock, on purpose: this code may be waiting for the
    result of PAUSING the simulation, and the simulated clock is the one thing
    that certainly will not advance afterwards.
    """
    deadline = time.monotonic() + timeout_s
    while not future.done():
        if time.monotonic() > deadline:
            return False
        time.sleep(0.01)
    return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimControlRelay()
    # Multithreaded so the reentrant group above counts: with a single-thread
    # executor, the blocked callback would be the only available thread.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
