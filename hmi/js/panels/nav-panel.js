/**
 * Navigation panel — the 2D floor plan, and the only place the operator can
 * command the robot in F3b.
 *
 * Draws, in this order (back to front):
 *
 *   global costmap  /global_costmap/costmap            PNG-compressed, 0.5 Hz
 *   1 m grid        (derived)
 *   global plan     /plan                              map frame
 *   laser           /demo/scan                         lidar frame, via TF
 *   footprint       /global_costmap/published_footprint  map frame
 *   goal            (local, from the last click)
 *
 * The quadruped now feeds the live slam_toolbox map into the global costmap.
 * The base raster here remains that costmap (rather than subscribing to /map a
 * second time), because it is the exact combination of static, obstacle,
 * perception and inflation layers the planner uses.
 *
 * The costmap window IS rolling — `rolling_window: true`, restored on
 * 27/08/2026 because `false` let the StaticLayer shrink the master grid to the
 * first SLAM rectangle and the planner walked off the edge with
 * `worldToMap failed`. So the raster already follows the robot. The camera
 * still centres on map->base EXPLICITLY anyway, because zoom needs a focus
 * point that does not move when the raster's bounds do.
 *
 * The panel has TWO destructive controls and they are not the same thing:
 *
 *   cancel goal      talks to the navigate_to_pose action. Stops the robot and
 *                    leaves everything else in place. It is the "I changed my
 *                    mind" button, and only appears while a goal is active.
 *   restart nav      calls /demo/nav/reset, and the navigation-side facade
 *                    cancels the goal, clears both costmaps and does
 *                    PAUSE + RESUME on the Nav2 servers — which run on the
 *                    Aquila in hil mode. It is the "Nav2 hung" button, and so
 *                    it is ALWAYS visible: the case it serves is exactly the
 *                    one where the action has stopped responding.
 *                    It is NOT RESET + STARTUP; that pair kills the container,
 *                    and the measurement is in the header of
 *                    nav_control_relay.py.
 *
 * Detections are deliberately NOT drawn here. They reach this panel already,
 * through the costmap's `perception_layer` — which is the wiring CLAUDE.md
 * requires ("Detections feed a Nav2 costmap layer, not just the HMI screen").
 * Painting them a second time from /demo/perception/detections would need the
 * camera intrinsics and a depth projection, and would show the same obstacle
 * twice with two different provenances. The image-space boxes belong on the
 * camera panel, and that is where they are.
 */

import { createExplorationStore } from './exploration.js';
import { readMapPalette } from './palette.js';
import { TOPICS } from '../config.js';
import { applyTransform } from '../ros/tf-tree.js';
import {
  buildCostLut,
  DEFAULT_MAP_ZOOM,
  createView,
  defaultExtent,
  extentOfGrid,
  stepMapZoom,
} from './map-view.js';

const NAVIGATE_ACTION = '/navigate_to_pose';
const NAVIGATE_TYPE = 'nav2_msgs/action/NavigateToPose';

/**
 * std_srvs/Trigger served by `nav_control_relay`, on the navigation side.
 *
 * It is NOT `/lifecycle_manager_navigation/manage_nodes`. And the reason here
 * is NOT the Gazebo trap: `nav2_msgs` exists in the `cockpit` container (it is
 * the package of the NavigateToPose used by the goal above), so a direct call
 * would work.
 *
 * The reason is that restarting Nav2 is FOUR steps with an invalid state in
 * the middle — between PAUSE and RESUME the stack is inactive, and nothing
 * brings it back on its own. A sequence like that driven by the browser dies
 * with an F5 and leaves navigation disabled with nobody to finish it. On the
 * ROS side it runs to completion, in a process that does not depend on this
 * page.
 *
 * And which pair of transitions to use is a measured finding, not an interface
 * choice: RESET + STARTUP kills the container. The header of
 * demo_navigation/nav_control_relay.py has the measurement.
 */
export const NAV_RESET_SERVICE = '/demo/nav/reset';
export const EXPLORATION_START_SERVICE = '/demo/exploration/start';
export const EXPLORATION_CANCEL_SERVICE = '/demo/exploration/cancel';

/** Milliseconds the restart button stays armed waiting for confirmation. */
export const RESET_ARM_MS = 4000;

/** Metres between grid lines. */
const GRID_STEP_M = 1;

/** A click closer than this to the robot is a mis-click, not a goal. */
const MIN_GOAL_DISTANCE_M = 0.25;

export function createNavPanel({ root, client, tracker }) {
  // A single read, at mount time. See the header of palette.js.
  const palette = readMapPalette(root);

  const canvas = root.querySelector('[data-role="nav-canvas"]');
  const hud = root.querySelector('[data-role="nav-hud"]');
  const cancelButton = root.querySelector('[data-role="nav-cancel"]');
  const resetButton = root.querySelector('[data-role="nav-reset"]');
  const zoomInButton = root.querySelector('[data-role="nav-zoom-in"]');
  const zoomOutButton = root.querySelector('[data-role="nav-zoom-out"]');
  const explorationStartButton = root.querySelector('[data-role="exploration-start"]');
  const explorationCancelButton = root.querySelector('[data-role="exploration-cancel"]');
  const context = canvas.getContext('2d');

  const lut = buildCostLut();
  const raster = document.createElement('canvas');
  const rasterContext = raster.getContext('2d', { willReadFrequently: true });

  const state = {
    gridInfo: null,
    footprint: null,
    plan: null,
    scan: null,
    goal: null,
    /**
     * 'idle' | 'sent' | 'running' | 'ok' | 'fail' | 'lost' | 'cancelled'
     * | 'resetting' | 'reset' | 'reset-failed'
     */
    goalState: 'idle',
    goalDetail: '',
    feedback: null,
    handle: null,
    tf: null,
    zoom: DEFAULT_MAP_ZOOM,
  };

  // Zoom lives in `state`; the search state lives HERE. Kept apart on
  // purpose: no exploration path should be able to touch the framing the
  // operator chose.
  const exploration = createExplorationStore();

  const unsubscribes = [];
  let cssWidth = 0;
  let cssHeight = 0;

  // --- sizing -------------------------------------------------------------
  // The canvas backing store is sized in DEVICE pixels and the context scaled,
  // otherwise everything is soft on a HiDPI screen and the kiosk looks cheap.
  const resize = () => {
    const rect = canvas.getBoundingClientRect();
    const ratio = window.devicePixelRatio || 1;
    cssWidth = Math.max(1, Math.round(rect.width));
    cssHeight = Math.max(1, Math.round(rect.height));
    canvas.width = Math.round(cssWidth * ratio);
    canvas.height = Math.round(cssHeight * ratio);
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
  };
  const observer = new ResizeObserver(resize);
  observer.observe(canvas);
  resize();

  const viewNow = () => {
    const extent = state.gridInfo ? extentOfGrid(state.gridInfo) : defaultExtent();
    const pose = robotPose();
    const center = pose ? { x: pose.x, y: pose.y } : null;
    return createView(extent, cssWidth, cssHeight, {
      zoom: state.zoom,
      center,
    });
  };

  // --- subscriptions ------------------------------------------------------

  unsubscribes.push(
    client.subscribe(
      '/global_costmap/costmap',
      'nav_msgs/msg/OccupancyGrid',
      (message) => {
        tracker.mark('costmap');
        rasterizeCostmap(message);
      },
      // Measured 24/08/2026: 456.8 KiB per frame as JSON, 13.8 KiB as PNG.
      // Without this the panel alone would spend ~230 KiB/s on localhost and
      // would not survive the bench Ethernet link at all.
      { compression: 'png' },
    ),
  );

  unsubscribes.push(
    client.subscribe(
      TOPICS.explorationStatus,
      'std_msgs/msg/String',
      (message) => exploration.apply(message),
    ),
  );

  unsubscribes.push(
    client.subscribe(
      TOPICS.mazeEscaped,
      'std_msgs/msg/Bool',
      (message) => exploration.setEscaped(message?.data),
    ),
  );

  unsubscribes.push(
    client.subscribe(
      '/global_costmap/published_footprint',
      'geometry_msgs/msg/PolygonStamped',
      (message) => {
        tracker.mark('costmap');
        state.footprint = message;
      },
    ),
  );

  unsubscribes.push(
    client.subscribe('/plan', 'nav_msgs/msg/Path', (message) => {
      state.plan = message;
    }),
  );

  unsubscribes.push(
    client.subscribe(
      TOPICS.scan,
      'sensor_msgs/msg/LaserScan',
      (message) => {
        tracker.mark('scan');
        state.scan = message;
      },
      { throttleRate: 200 },
    ),
  );

  // /tf runs at ~380 Hz on this robot (twelve leg joints plus odometry).
  // Relaying that raw would cost more than the costmap. 100 ms is five times
  // faster than the repaint tick, so nothing visible is lost.
  unsubscribes.push(
    client.subscribe(
      '/tf',
      'tf2_msgs/msg/TFMessage',
      (message) => state.tf?.update(message),
      { throttleRate: 100 },
    ),
  );

  // --- /tf_static, and why it needs a retry loop ---------------------------
  //
  // /tf_static has TWO independent latched publishers here:
  // robot_state_publisher, which owns the eight fixed robot edges, and odom_tf,
  // which owns map->odom. Each publishes ONE message, once, at startup. TfTree
  // accumulates rather than replaces precisely so the two can coexist.
  //
  // Measured against rosbridge 2.7.0 on 24/08/2026: a fresh subscription to
  // /tf_static receives exactly ONE of those two latched messages, and WHICH
  // one varies between runs. Three consecutive probes returned, in order: the
  // eight robot edges; map->odom; map->odom. Raising queue_length to 16 changed
  // nothing, so the sample is lost above the client queue.
  //
  // The visible symptom is a cockpit that works on some page loads and not
  // others: with map->odom missing the HUD reports "no TF map→base (21
  // edges)", the laser is not drawn, and goals still go out but land without
  // a heading. Nothing in any log names the cause.
  //
  // Unsubscribing and re-subscribing makes rosbridge hand over a latched
  // message again, and the union converges: measured 2 and 4 rounds in two
  // runs. So the panel retries until the chain to `map` closes, then stops for
  // good. Bounded, self-limiting, and contained in the cockpit — the
  // alternative would be changing how odom_tf publishes, which is on the
  // navigation path that F5/F6 validated on hardware.
  const STATIC_TF_RETRY_MS = 1500;
  const STATIC_TF_MAX_ATTEMPTS = 8;
  const staticTf = { off: null, attempts: 0, lastAt: 0, done: false };

  function listenStaticTf() {
    staticTf.off?.();
    // NOT throttled: these arrive once and a throttle window could swallow the
    // only copy.
    staticTf.off = client.subscribe(
      '/tf_static',
      'tf2_msgs/msg/TFMessage',
      (message) => state.tf?.update(message),
      { queueLength: 16 },
    );
  }

  function refetchStaticTf(now) {
    if (staticTf.done) return;
    if (state.tf?.lookup('map', 'base')) {
      staticTf.done = true;
      return;
    }
    if (staticTf.attempts >= STATIC_TF_MAX_ATTEMPTS) return;
    if (now - staticTf.lastAt < STATIC_TF_RETRY_MS) return;
    staticTf.lastAt = now;
    staticTf.attempts += 1;
    listenStaticTf();
  }

  listenStaticTf();

  // --- costmap raster -----------------------------------------------------

  function rasterizeCostmap(message) {
    const info = message.info;
    const data = message.data;
    if (!info || !data) return;

    state.gridInfo = info;
    raster.width = info.width;
    raster.height = info.height;
    const image = rasterContext.createImageData(info.width, info.height);
    const out = image.data;

    for (let row = 0; row < info.height; row += 1) {
      // OccupancyGrid row 0 sits at the grid ORIGIN, which is the bottom-left
      // in map coordinates; ImageData row 0 is the top. Without this flip the
      // map draws mirrored about the horizontal axis and still looks like a
      // plausible map — the classic silent bug of this kind of viewer.
      const source = row * info.width;
      const target = (info.height - 1 - row) * info.width;
      for (let col = 0; col < info.width; col += 1) {
        const value = data[source + col] & 0xff;
        const from = value * 4;
        const to = (target + col) * 4;
        out[to] = lut[from];
        out[to + 1] = lut[from + 1];
        out[to + 2] = lut[from + 2];
        out[to + 3] = lut[from + 3];
      }
    }
    rasterContext.putImageData(image, 0, 0);
  }

  // --- goals --------------------------------------------------------------

  function robotPose() {
    const transform = state.tf?.lookup('map', 'base');
    return transform ?? null;
  }

  function sendGoal(world) {
    if (explorationBusy()) return;
    const pose = robotPose();
    if (pose) {
      const distance = Math.hypot(world.x - pose.x, world.y - pose.y);
      if (distance < MIN_GOAL_DISTANCE_M) return;
    }
    // Point the goal along the direction of travel. Nav2 honours goal yaw, and
    // leaving it at identity makes the robot spin in place on arrival — which
    // is the manoeuvre this quadruped does worst (nav_quadruped.launch.py).
    const yaw = pose
      ? Math.atan2(world.y - pose.y, world.x - pose.x)
      : 0;

    cancelActive();

    state.goal = { x: world.x, y: world.y, yaw };
    state.goalState = 'sent';
    state.goalDetail = '';
    state.feedback = null;

    const handle = client.sendActionGoal(
      NAVIGATE_ACTION,
      NAVIGATE_TYPE,
      {
        pose: {
          header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
          pose: {
            position: { x: world.x, y: world.y, z: 0 },
            orientation: {
              x: 0,
              y: 0,
              z: Math.sin(yaw / 2),
              w: Math.cos(yaw / 2),
            },
          },
        },
      },
      {
        onFeedback: (values) => {
          // ~100 Hz, measured. Store only; the repaint happens on the shared
          // 4 Hz tick.
          state.feedback = values;
          if (state.goalState === 'sent') state.goalState = 'running';
        },
      },
    );
    state.handle = handle;

    handle.result.then((outcome) => {
      if (state.handle !== handle) return;
      state.handle = null;
      state.feedback = null;
      if (outcome.lost) {
        state.goalState = 'lost';
        state.goalDetail = 'link dropped — the goal may still be running on the robot';
      } else if (outcome.notSent) {
        state.goalState = 'fail';
        state.goalDetail = 'no connection to rosbridge';
      } else if (outcome.succeeded) {
        state.goalState = 'ok';
        state.goalDetail = '';
      } else {
        state.goalState = 'fail';
        state.goalDetail = `status ${outcome.status ?? '?'}`;
      }
    });
  }

  function cancelActive() {
    if (!state.handle) return;
    state.handle.cancel();
    state.handle = null;
    state.goalState = 'cancelled';
    state.goalDetail = '';
    state.feedback = null;
  }

  canvas.addEventListener('click', (event) => {
    if (explorationBusy()) return;
    const rect = canvas.getBoundingClientRect();
    sendGoal(viewNow().toWorld(event.clientX - rect.left, event.clientY - rect.top));
  });

  cancelButton?.addEventListener('click', cancelActive);

  function explorationActive() {
    return exploration.isActive();
  }

  /**
   * Includes the in-flight command, not just the state published by the Aquila.
   *
   * Between the click on "start search" and the first status there is a window
   * in which the explorer has already accepted the search and the cockpit does
   * not know yet. Closing the manual-goal gates with `isActive()` alone leaves
   * that window open.
   */
  function explorationBusy() {
    return exploration.isBusy();
  }

  async function explorationCommand(service, button) {
    if (button) button.dataset.busy = 'true';
    const starting = service === EXPLORATION_START_SERVICE;
    try {
      const result = await client.callService(service, {});
      if (result?.success === false) {
        const text = result.message ?? 'search command refused';
        // A refused start must undo the local `starting`, or the panel stays
        // stuck in a state that only the cockpit invented.
        if (starting) exploration.refuseStart(text);
        else exploration.merge({ message: text });
      }
    } catch (error) {
      if (starting) exploration.refuseStart(error.message);
      else exploration.merge({ state: 'failed', message: error.message });
    } finally {
      exploration.endCommand();
      if (button) button.dataset.busy = 'false';
    }
  }

  explorationStartButton?.addEventListener('click', () => {
    // A double click must become ONE call. The guard comes before any side
    // effect, including the cancellation of the manual goal.
    if (explorationBusy()) return;
    cancelActive();
    exploration.beginStart();
    // Paint the lock NOW, without waiting for the next frame: between the click
    // and the first status from the Aquila the map must look locked, not just
    // be locked.
    updateHud();
    explorationCommand(EXPLORATION_START_SERVICE, explorationStartButton);
  });
  explorationCancelButton?.addEventListener('click', () => {
    exploration.beginCancel();
    explorationCommand(EXPLORATION_CANCEL_SERVICE, explorationCancelButton);
  });

  function changeZoom(direction) {
    state.zoom = stepMapZoom(state.zoom, direction);
  }

  zoomInButton?.addEventListener('click', (event) => {
    event.stopPropagation();
    changeZoom('in');
  });
  zoomOutButton?.addEventListener('click', (event) => {
    event.stopPropagation();
    changeZoom('out');
  });

  // --- restart navigation -------------------------------------------------
  //
  // Two clicks, and the button label says which of the two we are at. Same
  // pattern as the simulation reset in sim-controls.js, and for the same
  // reason: a click by mistake costs tens of seconds in the middle of a demo.
  const resetLabel = resetButton?.textContent ?? '';
  let armedUntil = 0;

  function disarmReset() {
    armedUntil = 0;
    if (!resetButton) return;
    resetButton.dataset.armed = 'false';
    resetButton.textContent = resetLabel;
  }

  async function resetNavigation() {
    // The local handle dies with the stack; dropping it here keeps the result's
    // `.then`, which arrives aborted, from overwriting the restart state with a
    // 'fail' that describes the consequence and not the cause.
    state.handle = null;
    state.feedback = null;
    state.goal = null;
    state.goalState = 'resetting';
    state.goalDetail = '';
    if (resetButton) {
      resetButton.dataset.busy = 'true';
      resetButton.textContent = 'restarting…';
    }

    try {
      // No browser-side timeout, on purpose. Measured at 6.6 s on the
      // workstation, and slower on the AM69 arm64 — a local limit would paint
      // "failed" over a stack that was coming back. The limits belong to the
      // facade, which knows what it is waiting for.
      const result = await client.callService(NAV_RESET_SERVICE, {});
      if (result?.success === false) {
        state.goalState = 'reset-failed';
        state.goalDetail = result.message ?? '';
      } else {
        state.goalState = 'reset';
        state.goalDetail = result?.message ?? '';
      }
    } catch (error) {
      state.goalState = 'reset-failed';
      state.goalDetail = error.message;
    } finally {
      if (resetButton) {
        resetButton.dataset.busy = 'false';
        resetButton.textContent = resetLabel;
      }
      disarmReset();
    }
  }

  resetButton?.addEventListener('click', () => {
    if (Date.now() > armedUntil) {
      armedUntil = Date.now() + RESET_ARM_MS;
      resetButton.dataset.armed = 'true';
      resetButton.textContent = 'confirm';
      window.setTimeout(disarmReset, RESET_ARM_MS);
      return;
    }
    resetNavigation();
  });

  // --- drawing ------------------------------------------------------------

  function draw() {
    refetchStaticTf(Date.now());
    const view = viewNow();
    context.clearRect(0, 0, cssWidth, cssHeight);

    if (state.gridInfo && raster.width > 0) {
      const rect = view.rasterRect();
      // The costmap is a discrete grid; smoothing it invents cells that are
      // not there and blurs the lethal boundary the operator is reading.
      context.imageSmoothingEnabled = false;
      context.drawImage(raster, rect.x, rect.y, rect.width, rect.height);
    }

    drawGrid(view);
    drawPlan(view);
    drawScan(view);
    drawFootprint(view);
    drawGoal(view);
    updateHud();
  }

  function drawGrid(view) {
    const { extent } = view;
    context.save();
    context.strokeStyle = palette.grid;
    context.lineWidth = 1;
    context.beginPath();
    const firstX = Math.ceil(extent.originX / GRID_STEP_M) * GRID_STEP_M;
    for (let x = firstX; x <= extent.originX + extent.widthM; x += GRID_STEP_M) {
      const a = view.toScreen(x, extent.originY);
      const b = view.toScreen(x, extent.originY + extent.heightM);
      context.moveTo(a.x, a.y);
      context.lineTo(b.x, b.y);
    }
    const firstY = Math.ceil(extent.originY / GRID_STEP_M) * GRID_STEP_M;
    for (let y = firstY; y <= extent.originY + extent.heightM; y += GRID_STEP_M) {
      const a = view.toScreen(extent.originX, y);
      const b = view.toScreen(extent.originX + extent.widthM, y);
      context.moveTo(a.x, a.y);
      context.lineTo(b.x, b.y);
    }
    context.stroke();
    context.restore();
  }

  function drawPlan(view) {
    const poses = state.plan?.poses;
    if (!poses?.length) return;
    context.save();
    context.strokeStyle = palette.plan;
    context.lineWidth = 2;
    context.lineJoin = 'round';
    context.beginPath();
    poses.forEach((entry, index) => {
      const p = entry.pose.position;
      const s = view.toScreen(p.x, p.y);
      if (index === 0) context.moveTo(s.x, s.y);
      else context.lineTo(s.x, s.y);
    });
    context.stroke();
    context.restore();
  }

  function drawScan(view) {
    const scan = state.scan;
    if (!scan?.ranges?.length) return;
    const transform = state.tf?.lookup('map', scan.header?.frame_id ?? 'lidar');
    // No chain yet: draw nothing rather than draw at the origin. Beams piled on
    // the map origin look exactly like a lidar failure.
    if (!transform) return;

    context.save();
    context.fillStyle = palette.scan;
    // Alpha on the context and not embedded in the colour: the token is a solid
    // value, and dissolving it into rgba() here would recreate the duplication
    // that palette.js removes.
    context.globalAlpha = 0.85;
    const size = Math.max(1.5, view.scale * 0.05);
    for (let i = 0; i < scan.ranges.length; i += 1) {
      const range = scan.ranges[i];
      if (!Number.isFinite(range)) continue;
      if (range < scan.range_min || range > scan.range_max) continue;
      const angle = scan.angle_min + i * scan.angle_increment;
      const world = applyTransform(
        transform,
        range * Math.cos(angle),
        range * Math.sin(angle),
      );
      const s = view.toScreen(world.x, world.y);
      context.fillRect(s.x - size / 2, s.y - size / 2, size, size);
    }
    context.restore();
  }

  function drawFootprint(view) {
    const points = state.footprint?.polygon?.points;
    if (!points?.length) return;
    context.save();
    context.fillStyle = palette.robot;
    context.strokeStyle = palette.robot;
    context.lineWidth = 2;
    context.beginPath();
    points.forEach((point, index) => {
      const s = view.toScreen(point.x, point.y);
      if (index === 0) context.moveTo(s.x, s.y);
      else context.lineTo(s.x, s.y);
    });
    context.closePath();
    context.globalAlpha = 0.3;
    context.fill();
    context.globalAlpha = 1;
    context.stroke();

    // Heading whisker. A symmetric footprint says nothing about which way the
    // robot faces, and that is the first thing you need when reading a plan.
    const pose = robotPose();
    if (pose) {
      const nose = applyTransform(pose, 0.45, 0);
      const a = view.toScreen(pose.x, pose.y);
      const b = view.toScreen(nose.x, nose.y);
      context.beginPath();
      context.moveTo(a.x, a.y);
      context.lineTo(b.x, b.y);
      context.stroke();
    }
    context.restore();
  }

  function drawGoal(view) {
    if (!state.goal) return;
    const s = view.toScreen(state.goal.x, state.goal.y);
    const active = state.goalState === 'sent' || state.goalState === 'running';
    context.save();
    context.strokeStyle = palette.goal;
    // A goal already reached or cancelled stays drawn, dimmed: it leaves the
    // foreground without leaving the screen, which is how "it was heading
    // there" is read.
    context.globalAlpha = active ? 1 : 0.45;
    context.lineWidth = 2;
    context.beginPath();
    context.arc(s.x, s.y, 7, 0, Math.PI * 2);
    context.moveTo(s.x - 11, s.y);
    context.lineTo(s.x + 11, s.y);
    context.moveTo(s.x, s.y - 11);
    context.lineTo(s.x, s.y + 11);
    context.stroke();
    context.restore();
  }

  const GOAL_LABELS = {
    idle: 'click on the map to send a goal',
    sent: 'goal sent',
    running: 'navigating',
    ok: 'goal reached',
    fail: 'goal failed',
    lost: 'goal lost',
    cancelled: 'goal cancelled',
    resetting: 'restarting navigation — the costmap comes back empty',
    reset: 'navigation restarted · click on the map to send a goal',
    'reset-failed': 'navigation restart FAILED',
  };

  function updateHud() {
    const parts = exploration.ownsHud()
      ? [exploration.label()]
      : [GOAL_LABELS[state.goalState] ?? state.goalState];
    parts.push(...exploration.hudParts());
    const remaining = state.feedback?.distance_remaining;
    if (Number.isFinite(remaining)) parts.push(`${remaining.toFixed(2)} m remaining`);
    const recoveries = state.feedback?.number_of_recoveries;
    if (recoveries > 0) parts.push(`${recoveries} recovery(ies)`);
    if (state.goalDetail) parts.push(state.goalDetail);
    // Name WHY the pose is missing. "no TF" alone sends the operator to the
    // wrong place half the time: no edges at all means the /tf relay is not
    // arriving, while a populated tree with no chain to `base` means a
    // publisher is missing (map->odom comes from odom_tf, odom->base from the
    // plant) — different containers, different fixes.
    if (!robotPose()) {
      const known = state.tf?.frames().length ?? 0;
      parts.push(known ? `no TF map→base (${known} edges)` : 'waiting for TF');
    }

    const text = parts.join(' · ');
    if (hud.textContent !== text) hud.textContent = text;
    hud.dataset.state = state.goalState;

    const active = state.goalState === 'sent' || state.goalState === 'running';
    if (cancelButton) cancelButton.hidden = !active;
    const exploring = explorationBusy();
    if (explorationStartButton) explorationStartButton.hidden = exploring;
    if (explorationCancelButton) explorationCancelButton.hidden = !exploring;
    canvas.classList.toggle('canvas--disabled', exploring);
  }

  return {
    /** Injected by main.js so the TF cache is shared with any future panel. */
    useTf(tree) {
      state.tf = tree;
      return this;
    },
    tick: draw,
    onLinkDown() {
      tracker.clear('costmap');
      tracker.clear('scan');
      // The subscription replay after a reconnect re-runs the same latched-
      // message lottery, so the retry loop has to be armed again. The tree
      // itself is kept: static edges do not change and dynamic ones are
      // overwritten by the next sample.
      staticTf.done = false;
      staticTf.attempts = 0;
      staticTf.lastAt = 0;
      // Geometry from a dead link is not "old", it is unknown: the robot has
      // been moving while we were blind, so the footprint and beams on screen
      // are worse than nothing.
      state.scan = null;
      state.plan = null;
      state.footprint = null;
      state.feedback = null;
      state.tf?.clear();
      // A restart in progress lost its response along with the link, and a
      // button stuck on "restarting…" would be the wrong reading: the call may
      // have been applied. Go back to the label and let the HUD say what it
      // knows.
      disarmReset();
      if (resetButton) {
        resetButton.dataset.busy = 'false';
        resetButton.textContent = resetLabel;
      }
      if (state.goalState === 'resetting') {
        state.goalState = 'lost';
        state.goalDetail = 'link dropped during the navigation restart';
      }
    },
    destroy() {
      observer.disconnect();
      // /tf_static is not in `unsubscribes`: its unsubscribe handle is swapped
      // every retry, so only the current one may be called.
      staticTf.off?.();
      for (const off of unsubscribes) off();
    },
  };
}
