/**
 * Rotate, move and zoom the scene camera.
 *
 * THE BROWSER DOES NOT KNOW WHERE THE CAMERA IS, AND THAT IS THE DESIGN
 *
 * Each button publishes a DELTA on /demo/cockpit/scene/cmd_view. The one that
 * holds the orbit (azimuth, elevation, distance, target), saturates the limits
 * and writes the pose into Gazebo is the `scene_view_controller`, on the
 * simulator side.
 *
 * The alternative — the client computing the pose and sending it ready-made —
 * breaks in three ways that would already be here: two open cockpits fight
 * over the pose, an F5 resets the framing, and the orbit math (the zero-roll
 * quaternion in particular) would exist in two languages that have to agree.
 * On this side there is only "turn a little to the left".
 *
 * `header.frame_id` picks the camera. It follows the iso/top button in the
 * panel header: commanding the camera that is not on screen is the kind of
 * error that looks like "the buttons do not work".
 *
 * FOLLOW THE ROBOT
 *
 * The `follow robot` button is a SetBool, and applies to both views at once —
 * the orbit target is the robot pose, and there is no version of that which
 * makes sense for a single camera. It is not the browser that follows either:
 * the one that reads /demo/odom and rewrites the pose is the same simulator
 * node, for the usual reason (the cockpit knows no geometry).
 *
 * The button state comes from /demo/cockpit/scene/following, published by the
 * node, and NOT from the click itself. Same reasoning as the simulation label
 * in sim-controls.js: reloading the page, opening the cockpit on a second
 * screen, or opening it after someone turned following off from the command
 * line are three cases in which the local click does not know the answer. The
 * topic is latched, so a new tab receives the value without waiting for the
 * next change.
 */

import { ConnectionState } from '../ros/rosbridge-client.js';

export const CMD_VIEW_TOPIC = '/demo/cockpit/scene/cmd_view';
export const CMD_VIEW_TYPE = 'geometry_msgs/msg/TwistStamped';

/** std_srvs/Trigger, served by the scene_view_controller. */
export const RESET_VIEW_SERVICE = '/demo/cockpit/scene/reset_view';

/** std_srvs/SetBool, same node. Turns following of both views on and off. */
export const FOLLOW_VIEW_SERVICE = '/demo/cockpit/scene/follow';

/** Follow state, published by the node. Latched — see the header. */
export const FOLLOWING_TOPIC = '/demo/cockpit/scene/following';
export const FOLLOWING_TYPE = 'std_msgs/msg/Bool';

/**
 * Until the topic arrives, the button assumes the node default (parameter
 * `follow`, true in scene_cameras.launch.py). Assuming `false` here would paint
 * an off button over a camera that is already following, and the first click
 * would send the value that already holds — with no visible effect.
 */
export const FOLLOW_DEFAULT = true;

/**
 * Step of each command, in the unit the controller integrates.
 *
 * Measured on the scene, not chosen: with an orbit step of 0.30 rad a full
 * turn takes ~21 clicks, which lets you frame without counting clicks and
 * without overshooting. The dolly is negative to ZOOM IN because `linear.x` is
 * the change in distance to the target.
 */
export const VIEW_STEPS = Object.freeze({
  'orbit-left': { angular: { z: 0.3 } },
  'orbit-right': { angular: { z: -0.3 } },
  'pitch-up': { angular: { y: 0.18 } },
  'pitch-down': { angular: { y: -0.18 } },
  'zoom-in': { linear: { x: -1.2 } },
  'zoom-out': { linear: { x: 1.2 } },
  'pan-left': { linear: { y: 0.8 } },
  'pan-right': { linear: { y: -0.8 } },
  'pan-forward': { linear: { z: 0.8 } },
  'pan-back': { linear: { z: -0.8 } },
});

/** Holding the button repeats: framing from far away would cost dozens of clicks. */
export const HOLD_DELAY_MS = 340;
export const HOLD_INTERVAL_MS = 130;

/** Builds the complete TwistStamped — rosbridge does not fill nested fields. */
export function viewCommand(command, camera) {
  const step = VIEW_STEPS[command];
  if (!step) return null;
  return {
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: camera },
    twist: {
      linear: { x: 0, y: 0, z: 0, ...step.linear },
      angular: { x: 0, y: 0, z: 0, ...step.angular },
    },
  };
}

export function createViewControls({ root, client, camera = 'scene_iso', onNotice }) {
  const pad = root.querySelector('[data-role="view-pad"]');
  const resetButton = root.querySelector('[data-role="view-reset"]');
  const followButton = root.querySelector('[data-role="view-follow"]');
  let active = camera;
  let following = FOLLOW_DEFAULT;

  client.advertise(CMD_VIEW_TOPIC, CMD_VIEW_TYPE);

  const publish = (command) => {
    const message = viewCommand(command, active);
    if (message) client.publish(CMD_VIEW_TOPIC, message);
  };

  // --- hold to repeat -------------------------------------------------------
  // A single timer, shared by all buttons: two buttons pressed at the same time
  // would be an ambiguous command, not two commands.
  let holdTimer = null;
  let holdRepeat = null;

  const stopHold = () => {
    window.clearTimeout(holdTimer);
    window.clearInterval(holdRepeat);
    holdTimer = null;
    holdRepeat = null;
  };

  for (const button of pad.querySelectorAll('[data-role="view"]')) {
    const command = button.dataset.command;

    button.addEventListener('pointerdown', (event) => {
      // Without this the button keeps repeating after the pointer leaves it, and
      // the camera spins on its own until someone clicks elsewhere.
      button.setPointerCapture?.(event.pointerId);
      publish(command);
      stopHold();
      holdTimer = window.setTimeout(() => {
        holdRepeat = window.setInterval(() => publish(command), HOLD_INTERVAL_MS);
      }, HOLD_DELAY_MS);
    });
    button.addEventListener('pointerup', stopHold);
    button.addEventListener('pointercancel', stopHold);
    button.addEventListener('lostpointercapture', stopHold);

    // The keyboard does not generate pointerdown. Without this the pad is
    // unreachable by keyboard, which is how the mouseless M3 kiosk will be
    // operated.
    button.addEventListener('keydown', (event) => {
      if (event.key === 'Enter' || event.key === ' ') publish(command);
    });
  }

  resetButton?.addEventListener('click', async () => {
    try {
      await client.callService(RESET_VIEW_SERVICE, {});
    } catch (error) {
      onNotice?.(`failed to recentre the camera: ${error.message}`);
    }
  });

  const paintFollow = () => {
    if (followButton) followButton.setAttribute('aria-pressed', String(following));
  };

  // The one that writes `following` is the node, through this topic. The click below only asks.
  const offFollowing = client.subscribe(
    FOLLOWING_TOPIC,
    FOLLOWING_TYPE,
    (message) => {
      following = Boolean(message?.data);
      paintFollow();
    },
  );

  followButton?.addEventListener('click', async () => {
    const wanted = !following;
    try {
      const result = await client.callService(FOLLOW_VIEW_SERVICE, { data: wanted });
      // success=false is the useful path here: the node answers that way when
      // it could not write the pose. Without this branch the button stays
      // silent exactly when it has something to say.
      if (result?.success === false) {
        onNotice?.(`the simulator refused follow=${wanted}: ${result.message ?? ''}`);
      }
    } catch (error) {
      onNotice?.(`failed to toggle camera following: ${error.message}`);
    }
    // No `following = wanted` here: the painted value is whatever the node publishes.
  });

  paintFollow();

  // The pad goes dim with the link. A button that publishes into the void must
  // not look alive — and this is the only panel whose effect appears nowhere on
  // screen when it fails, because its result IS the image that stays the same.
  const unregister = client.onStateChange((state) => {
    const up = state === ConnectionState.CONNECTED;
    pad.dataset.enabled = String(up);
    if (!up) stopHold();
  });

  return {
    /** Called by the iso/top button: commands follow the visible image. */
    setCamera(name) {
      active = name;
    },

    /** Exposed for tests: the painted button must reflect the node, not the click. */
    isFollowing() {
      return following;
    },

    destroy() {
      stopHold();
      unregister();
      offFollowing();
    },
  };
}
