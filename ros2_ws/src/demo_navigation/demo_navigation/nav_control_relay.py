"""
std_srvs facade for restarting navigation -- the cockpit's "reset goal".

Runs on: Aquila AM69 (arm64) in hil mode, x86 workstation in learn mode. It
lives on the Nav2 side, in the SAME container, which is why restarting
navigation "on the Aquila" is a service call and not SSH access to the module.

    /demo/nav/reset   std_srvs/Trigger   (in)
    /demo/nav/cancel  std_srvs/Trigger   (in)
    /navigate_to_pose/_action/cancel_goal              action_msgs/CancelGoal
    /{global,local}_costmap/clear_entirely_*_costmap   nav2_msgs/ClearEntireCostmap
    /lifecycle_manager_navigation/manage_nodes         nav2_msgs/ManageLifecycleNodes

WHAT "RESET" DOES, AND WHY IT IS NOT JUST CANCELLING THE GOAL

Cancelling the goal stops the robot and leaves everything else in place: the
accumulated costmap, the servers in whatever state they were. That is enough
when the operator merely changed their mind -- and for that case the green
panel has `cancel goal`, which talks directly to the action.

It is not enough in the case that motivated this node: a stuck Nav2. In the
24/08 HIL run both 8 m goals timed out in the 420/200 s protocol, with the robot
spending its time in recoveries over a dirty costmap. There what is wanted is a
clean stack, and what this node does, in order, is:

    1. CANCEL   every active navigate_to_pose goal;
    2. CLEAR    both costmaps entirely, global and local;
    3. PAUSE    deactivates all managed nodes, in reverse order;
    4. RESUME   reactivates all of them, in the right order.

After that the robot is stopped, the costmap carries no phantom obstacles, the
bt_navigator has lost any stuck internal state and MPPI has been reinitialized.

Measured on 24/08/2026 on the workstation, with an active goal: 6.6 s end to
end (PAUSE 2.4 s, RESUME 2.4 s, the rest in cancelling and clearing). The goal
in progress ended with status CANCELED and a new goal was accepted immediately
afterwards. On the AM69's Cortex-A72 this is slower and was NOT measured (rule 5
of CLAUDE.md).

WHY IT IS NOT RESET + STARTUP, WHICH WOULD BE THE OBVIOUS CHOICE

Because that BREAKS the stack, reproducibly. Measured on 24/08/2026, in learn
mode, quadruped path:

    ManageLifecycleNodes RESET (3) followed by STARTUP (0) kills the
    `component_container_isolated` process with SIGSEGV (exit code -11), always
    at the same point of the second CONFIGURE:

        [route_server]: Configuring Rerouting service operation.
        [ERROR] process has died [pid 40, exit code -11, ...]

Two attempts, two identical deaths -- with an active goal and without. After
that there is NO navigation at all: the nodes are not inactive, the process that
contained them is gone, and only `docker compose restart nav` brings it back. A
"fix navigation" button that kills navigation is worse than no button.

The cause is in `nav2_route`: the `route_server` is reconfigured from scratch by
STARTUP (RESET does CLEANUP, which logically destroys the node) and the
`ReroutingService` operation does not survive the cycle. The `route_server` is in
the `lifecycle_nodes` list of the vendored `navigation_launch.py` -- which is an
upstream copy and must stay identical (see launch/nav2_vendored/README.md) --
and this project uses NO routes: the behavior tree is NavigateToPose with
NavfnPlanner and MPPI, and `route_server` does not even appear in
nav2_params_go2.yaml. It comes up, configures, activates and is never called.

PAUSE + RESUME does not go through CONFIGURE, so it does not touch that path.
What is lost relative to RESET is what CONFIGURE would redo: the behavior tree
is not re-read from the XML and the plugins are not re-instantiated. Neither
changes during a demo. What is really lost is the destruction of the costmap --
and that is why step 2 exists: `clear_entirely_*_costmap` empties the same state
without going through the lifecycle.

If the `route_server` ever leaves the managed list, RESET + STARTUP becomes the
stronger option again. Until then, this.

WHY LOCALIZATION IS LEFT OUT

There is a second manager, `lifecycle_manager_localization`, on the static-map
path (nav.launch.py, with AMCL). It is deliberately NOT touched here: pausing
AMCL in the middle of a demo turns "navigation got stuck" into "the robot no
longer knows where it is". The quadruped path does not even have that manager --
what publishes map -> odom there is `odom_tf`, as an identity, and it is not a
lifecycle node.

Practical consequence: after a reset the robot is still localized and accepts
goals again. If localization is what is wrong, this button does not fix it.

WHY THE BROWSER DOES NOT CALL manage_nodes DIRECTLY

This is NOT the reason for trap 2 of docs/guia-completo.md (Part II). There the
browser CANNOT call the Gazebo service, because rosbridge builds the request by
importing the interfaces package inside the `cockpit` container and
`ros_gz_interfaces` does not exist there. Here it exists: `demo_bringup` declares
`<depend>nav2_msgs</depend>` and the key is not in the --skip-keys list of
docker/base/Dockerfile, so `ros-jazzy-nav2-msgs` is installed in the base image
and, by inheritance, in the cockpit one. Verified on 24/08/2026:

    docker run --rm --entrypoint bash local/demo-aquila-cockpit:dev -lc \
      'source /opt/ros/jazzy/setup.bash; python3 -c "import nav2_msgs.srv"'

It is the same reason click-to-goal works: NavigateToPose belongs to
`nav2_msgs`. If it were absent, not even the goal would go out.

The real reason is different, and stronger:

1. IT IS NOT ONE CALL, IT IS FOUR STEPS WITH AN INVALID INTERMEDIATE STATE.
   Between PAUSE and RESUME the stack is inactive: no goal is accepted and
   nothing brings it back on its own. If the sequence lived in the browser, an
   F5, a closed tab or a dropped WebSocket in the middle of it would leave Nav2
   deactivated with nobody to finish the job -- and the symptom would be
   "navigation died after I pressed the fix-navigation button". Here the whole
   sequence runs in a process that does not depend on the page.

2. WHICH COMMAND TO USE IS A DECISION OF THE STACK, NOT OF THE SCREEN. The
   section above is a finding measured on this Nav2 in this configuration.
   Written in JavaScript, it would live far from the file that brings up the
   managed nodes and would diverge on the first change to the stack topology.

3. It is the same choice `scene_view_controller` makes on the simulator side:
   the cockpit sends INTENT, whoever holds the state owns the machine.
"""

import time

from action_msgs.srv import CancelGoal
from nav2_msgs.srv import ClearEntireCostmap, ManageLifecycleNodes
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

# Default name of the NAVIGATION STACK manager (not the localization one). It
# comes from `name='lifecycle_manager_navigation'` in
# demo_navigation/launch/nav2_vendored/navigation_launch.py, in both branches --
# composed and non-composed. Parameterized because a namespaced deploy would
# prefix it, and a nonexistent service here is a dead button.
DEFAULT_MANAGER = '/lifecycle_manager_navigation/manage_nodes'

# The action the cockpit's click-to-goal uses. Cancelling through here, with
# `goal_info` zeroed, cancels ALL active goals -- which is what "reset the
# target" means, and what the handle of a reloaded cockpit can no longer do
# because it lost the uuid.
CANCEL_SERVICE = '/navigate_to_pose/_action/cancel_goal'

# The two costmaps. The global one accumulates the obstacles that dirty a long
# demo; the local one is what MPPI sees. Clearing one and not the other leaves
# the robot dodging a phantom that only half of the stack knows about.
COSTMAP_SERVICES = (
    '/global_costmap/clear_entirely_global_costmap',
    '/local_costmap/clear_entirely_local_costmap',
)

# Measured: PAUSE and RESUME take ~3 s each on the workstation. The limit is
# generous because it is slower on the AM69's Cortex-A72, and because timing out
# in the middle of a RESUME leaves the stack inactive -- worse than waiting.
TRANSITION_TIMEOUT_S = 60.0

# Cancelling and clearing are cheap and must not hold up the cycle: if the action
# does not even exist (stack already inactive), the following PAUSE/RESUME
# resolves it anyway.
SHORT_TIMEOUT_S = 5.0


class NavControlRelay(Node):
    """Translate Trigger calls into Nav2 lifecycle transitions."""

    def __init__(self) -> None:
        super().__init__('nav_control_relay')

        self.declare_parameter('manager_service', DEFAULT_MANAGER)
        self.declare_parameter('cancel_service', CANCEL_SERVICE)
        self.declare_parameter('costmap_services', list(COSTMAP_SERVICES))
        self._manager_name = self.get_parameter('manager_service').value
        self._cancel_name = self.get_parameter('cancel_service').value
        costmap_names = self.get_parameter('costmap_services').value

        # Reentrant group, exactly as in sim_control_relay: the Trigger
        # callbacks BLOCK waiting for the manage_nodes response. In the default
        # mutually exclusive group that wait prevents the executor from
        # processing the response itself, and every call times out.
        self._group = ReentrantCallbackGroup()

        self._manager = self.create_client(
            ManageLifecycleNodes, self._manager_name,
            callback_group=self._group,
        )
        self._cancel = self.create_client(
            CancelGoal, self._cancel_name, callback_group=self._group,
        )
        self._cancel_exploration = self.create_client(
            Trigger, '/demo/exploration/cancel', callback_group=self._group,
        )
        self._costmaps = {
            name: self.create_client(
                ClearEntireCostmap, name, callback_group=self._group)
            for name in costmap_names
        }

        self.create_service(
            Trigger, '/demo/nav/reset', self._on_reset,
            callback_group=self._group,
        )
        self.create_service(
            Trigger, '/demo/nav/cancel', self._on_cancel,
            callback_group=self._group,
        )

        self.get_logger().info(
            'navigation facade ready: /demo/nav/{reset,cancel} -> '
            f'{self._manager_name}'
        )

    # --- inbound -----------------------------------------------------------

    def _on_reset(self, request, response):
        del request

        if not self._manager.wait_for_service(timeout_sec=SHORT_TIMEOUT_S):
            response.success = False
            response.message = (
                f'{self._manager_name} did not respond; is the nav container up?'
            )
            self.get_logger().error(response.message)
            return response

        # Cancel BEFORE deactivating. It is not redundant: without it the
        # bt_navigator is deactivated with a goal in progress, which aborts it
        # without the client receiving a clean result, and the cockpit is left
        # with the HUD saying "navigating" over a stack that is no longer
        # running.
        self._stop_exploration()
        self._cancel_all()

        # Clear with the costmaps STILL ACTIVE: `clear_entirely_*` is a service
        # of the costmap nodes themselves, and an inactive node does not serve.
        cleared = self._clear_costmaps()

        paused, pause_message = self._transition(
            ManageLifecycleNodes.Request.PAUSE, 'PAUSE')
        if not paused:
            # We do NOT stop here. A PAUSE that failed midway leaves part of the
            # stack inactive, and returning the error without trying to bring it
            # back up would give the operator dead navigation and a button that
            # "did not work".
            self.get_logger().warning(
                f'PAUSE failed ({pause_message}); trying RESUME')

        resumed, resume_message = self._transition(
            ManageLifecycleNodes.Request.RESUME, 'RESUME')

        response.success = bool(resumed)
        if resumed and paused and cleared:
            response.message = (
                'navigation restarted: goal discarded, costmaps cleared, '
                'servers reactivated'
            )
        elif resumed:
            details = ', '.join(
                part for part in (
                    None if paused else pause_message,
                    None if cleared else 'the costmaps were not cleared',
                ) if part
            )
            response.message = f'navigation active, with caveats: {details}'
        else:
            response.message = (
                f'navigation did NOT come back up: {resume_message}. '
                'Restart the nav container.'
            )
        self.get_logger().info(response.message)
        return response

    def _on_cancel(self, request, response):
        del request
        self._stop_exploration()
        cancelled = self._cancel_all()
        response.success = cancelled
        response.message = (
            'goals cancelled' if cancelled
            else f'{self._cancel_name} did not respond; is there an active goal?'
        )
        return response

    # --- outbound ----------------------------------------------------------

    def _stop_exploration(self):
        """Best-effort: prevent the executive from resending after a reset."""
        if not self._cancel_exploration.wait_for_service(timeout_sec=0.5):
            return False
        future = self._cancel_exploration.call_async(Trigger.Request())
        return _wait(future, SHORT_TIMEOUT_S) and future.result() is not None

    def _transition(self, command, label):
        """Run one manager transition. Return (ok, reason)."""
        future = self._manager.call_async(
            ManageLifecycleNodes.Request(command=command))
        if not _wait(future, TRANSITION_TIMEOUT_S):
            return False, f'{label} timed out after {TRANSITION_TIMEOUT_S:.0f} s'
        result = future.result()
        if result is None:
            return False, f'{label} returned no response'
        if not result.success:
            return False, f'the manager refused {label}'
        return True, f'{label} applied'

    def _cancel_all(self):
        """
        Cancel every active navigate_to_pose goal.

        `goal_info` is left zeroed on purpose: in the ROS 2 action protocol, an
        empty id and zero stamp mean "all". Filling in the uuid would require
        knowing the goal, which is exactly what nobody knows after reloading the
        page.
        """
        if not self._cancel.wait_for_service(timeout_sec=SHORT_TIMEOUT_S):
            self.get_logger().warning(
                f'{self._cancel_name} does not exist yet; nothing to cancel')
            return False
        future = self._cancel.call_async(CancelGoal.Request())
        if not _wait(future, SHORT_TIMEOUT_S):
            self.get_logger().warning('the cancellation timed out')
            return False
        return future.result() is not None

    def _clear_costmaps(self):
        """Clear both costmaps. Return True only if both responded."""
        ok = True
        for name, client in self._costmaps.items():
            if not client.wait_for_service(timeout_sec=SHORT_TIMEOUT_S):
                # Not fatal: on the static-map path the global one has a
                # static_layer and comes back by itself, and the reset is still
                # worth it for the rest.
                self.get_logger().warning(f'{name} did not respond')
                ok = False
                continue
            future = client.call_async(ClearEntireCostmap.Request())
            if not _wait(future, SHORT_TIMEOUT_S):
                self.get_logger().warning(f'{name} timed out')
                ok = False
        return ok


def _wait(future, timeout_s: float) -> bool:
    """
    Wait for the future without spinning the executor.

    `spin_until_future_complete` does NOT work here, for the same reason
    recorded in demo_simulation/sim_control_relay.py: we are already inside an
    executor callback, and spinning it again over the same node is re-entrancy
    on the wrong object. Whoever completes this future is another thread of the
    MultiThreadedExecutor -- and the reentrant group exists to allow that.

    The clock is WALL time, and here that matters more than in the simulator
    relay: this node runs with use_sim_time, and a Nav2 deactivated in the
    middle of a PAUSE can perfectly well coexist with a stopped /clock.
    Measuring the timeout in simulated time would turn "timed out" into "waits
    forever".
    """
    deadline = time.monotonic() + timeout_s
    while not future.done():
        if time.monotonic() > deadline:
            return False
        time.sleep(0.02)
    return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavControlRelay()
    # Multithreaded so the reentrant group counts: with a single thread, the
    # blocked callback would be the only one available and the response would
    # never arrive.
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
