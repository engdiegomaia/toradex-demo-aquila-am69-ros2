/**
 * Simulation: play, pause and reset, from the cockpit.
 *
 * WHAT THIS IS AND WHAT IT IS NOT
 *
 * Gazebo runs on the x86 workstation and will keep running there. The AM69
 * exposes only OpenGL ES 3.2 and Vulkan 1.2, Gazebo's OGRE 2 needs desktop
 * OpenGL, and rule 1 of CLAUDE.md exists precisely to stop anyone from trying.
 * These buttons control the simulation THROUGH the cockpit — in M3 the cockpit
 * is served by the Aquila, so the click leaves the module — but whoever runs
 * the simulator is still the host. What crosses over is a service call on the
 * ROS graph, just like the Nav2 goal today.
 *
 * WHY THE STATE COMES FROM /clock AND NOT FROM THE BUTTON
 *
 * The obvious path would be to paint "paused" when the operator clicks pause.
 * That lies in every case that matters: the `sim` container went down, the call
 * timed out, someone paused from the Gazebo GUI, the world was reset by someone
 * else. The simulated clock is the only witness that knows whether the
 * simulator is actually running, so it is the one that writes the label — even
 * when it disagrees with the last click.
 *
 * /clock publishes at ~1 kHz and none of that resolution is needed: the
 * subscription is throttled on the server (throttle_rate), not filtered on the
 * client, so the traffic never even reaches the browser.
 */

/**
 * std_srvs/Trigger, one per action, served by `sim_control_relay`.
 *
 * It is NOT `/demo/sim/control` (ros_gz_interfaces/srv/ControlWorld), and
 * trying to call it directly from here is what motivated the facade. rosbridge
 * builds the request by importing the interface package inside its own
 * container:
 *
 *     call_service InvalidModuleException: Unable to import
 *     ros_gz_interfaces.srv from package ros_gz_interfaces
 *
 * The cockpit container does not have that package — and in `deploy` mode,
 * where there is no Gazebo at all, it would make no sense to have it. The
 * browser boundary speaks std_srvs, which is core ROS; the translation happens
 * on the simulator side.
 */
export const SIM_SERVICES = Object.freeze({
  play: '/demo/sim/play',
  pause: '/demo/sim/pause',
  reset: '/demo/sim/reset',
});

export const CLOCK_TOPIC = '/clock';
export const CLOCK_TYPE = 'rosgraph_msgs/msg/Clock';

/** ~2.5 Hz: enough for "moved or did not move", negligible on the wire. */
export const CLOCK_THROTTLE_MS = 400;

/**
 * With no advance of the simulated clock for this long, the simulation is
 * paused.
 *
 * It must be comfortably larger than CLOCK_THROTTLE_MS: with samples every
 * 400 ms, a tight threshold would flash "paused" on every network jitter.
 */
export const STALL_AFTER_MS = 1100;

/** With NO sample at all for this long, there is no simulator on the other end. */
export const OFFLINE_AFTER_MS = 3000;

/** Milliseconds the reset button stays armed waiting for confirmation. */
export const RESET_ARM_MS = 4000;

export const SimState = Object.freeze({
  RUNNING: 'running',
  PAUSED: 'paused',
  OFFLINE: 'no simulator',
});

/**
 * Pure core: receives samples of the simulated clock and answers which state
 * the simulation is in. Kept apart from the DOM because it is the only part
 * with real logic and the only one that can be tested without a browser.
 */
export function createClockWatch() {
  let lastSeconds = null;
  let lastSampleAt = null;
  let lastChangeAt = null;

  return {
    /** @param {number} seconds simulated time @param {number} atMs wall clock */
    sample(seconds, atMs) {
      // Only ADVANCE counts as a sign of life. A repeated sample proves the
      // bridge is alive, not that the world is running — and that is exactly
      // the difference between "paused" and "no simulator".
      if (lastSeconds === null || seconds !== lastSeconds) {
        lastChangeAt = atMs;
      }
      lastSeconds = seconds;
      lastSampleAt = atMs;
    },

    /** Back to unknown: used when the WebSocket drops. */
    reset() {
      lastSeconds = null;
      lastSampleAt = null;
      lastChangeAt = null;
    },

    state(nowMs) {
      if (lastSampleAt === null) return SimState.OFFLINE;
      if (nowMs - lastSampleAt > OFFLINE_AFTER_MS) return SimState.OFFLINE;
      if (nowMs - lastChangeAt > STALL_AFTER_MS) return SimState.PAUSED;
      return SimState.RUNNING;
    },
  };
}

/** Seconds of a rosgraph_msgs/Clock, tolerating missing fields. */
export function clockSeconds(message) {
  const stamp = message?.clock ?? {};
  return (stamp.sec ?? 0) + (stamp.nanosec ?? 0) / 1e9;
}

export function createSimControls({ root, client, onNotice }) {
  const buttons = [...root.querySelectorAll('[data-role="sim"]')];
  const label = root.querySelector('[data-role="sim-state"]');
  const resetButton = buttons.find((button) => button.dataset.command === 'reset');
  const resetLabel = resetButton?.textContent ?? '';

  const watch = createClockWatch();
  let armedUntil = 0;

  const unsubscribe = client.subscribe(
    CLOCK_TOPIC,
    CLOCK_TYPE,
    (message) => watch.sample(clockSeconds(message), Date.now()),
    { throttleRate: CLOCK_THROTTLE_MS },
  );

  const disarm = () => {
    armedUntil = 0;
    if (!resetButton) return;
    resetButton.dataset.armed = 'false';
    resetButton.textContent = resetLabel;
  };

  async function send(command) {
    try {
      const result = await client.callService(SIM_SERVICES[command], {});
      // Trigger answers success=false when Gazebo refused or the bridge did
      // not respond. Treating that as success would leave the button silent in
      // the only situation in which it has something to say.
      if (result?.success === false) {
        onNotice?.(`the simulator refused ${command}: ${result.message ?? ''}`);
      }
    } catch (error) {
      onNotice?.(`failed to ${command} the simulation: ${error.message}`);
    }
  }

  for (const button of buttons) {
    button.addEventListener('click', () => {
      const command = button.dataset.command;

      // Reset lands in the middle of the demo and teleports the robot to the
      // spawn point — Nav2 loses the goal in progress and sees the robot jump.
      // It is not destructive (the world and the clock stay), but it is not
      // something you want from an accidental click either. Two clicks.
      //
      // On a quadruped the call takes a few seconds, and that is on purpose:
      // the robot is STOPPED before the teleport and resettled afterwards.
      // Teleporting a moving robot knocks it over — see GAIT_STOP_S in
      // sim_control_relay.py.
      if (command === 'reset' && Date.now() > armedUntil) {
        armedUntil = Date.now() + RESET_ARM_MS;
        button.dataset.armed = 'true';
        button.textContent = 'confirm';
        window.setTimeout(disarm, RESET_ARM_MS);
        return;
      }
      if (command === 'reset') disarm();

      send(command);
    });
  }

  return {
    tick() {
      const state = watch.state(Date.now());
      label.textContent = state;
      label.dataset.state = state;
    },

    onLinkDown() {
      watch.reset();
      disarm();
    },

    destroy() {
      unsubscribe();
    },
  };
}
