/**
 * Navigation panel — the 2D floor plan, and the only place the operator can
 * command the robot in F3b.
 *
 * Draws, in this order (back to front):
 *
 *   global costmap  /global_costmap/costmap            PNG-compressed, 0,5 Hz
 *   1 m grid        (derived)
 *   global plan     /plan                              map frame
 *   laser           /demo/scan                         lidar frame, via TF
 *   footprint       /global_costmap/published_footprint  map frame
 *   goal            (local, from the last click)
 *
 * There is NO /map on the quadruped path: nav_quadruped.launch.py runs Nav2
 * without map_server and without AMCL, on a ROLLING global costmap. So the
 * base raster here is the costmap, not a static map — and that is also why the
 * view follows the robot for free: the costmap window does.
 *
 * Detections are deliberately NOT drawn here. They reach this panel already,
 * through the costmap's `perception_layer` — which is the wiring CLAUDE.md
 * requires ("Detections feed a Nav2 costmap layer, not just the HMI screen").
 * Painting them a second time from /demo/perception/detections would need the
 * camera intrinsics and a depth projection, and would show the same obstacle
 * twice with two different provenances. The image-space boxes belong on the
 * camera panel, and that is where they are.
 */

import { readMapPalette } from './palette.js';
import { TOPICS } from '../config.js';
import { applyTransform } from '../ros/tf-tree.js';
import {
  buildCostLut,
  createView,
  defaultExtent,
  extentOfGrid,
} from './map-view.js';

const NAVIGATE_ACTION = '/navigate_to_pose';
const NAVIGATE_TYPE = 'nav2_msgs/action/NavigateToPose';

/** Metres between grid lines. */
const GRID_STEP_M = 1;

/** A click closer than this to the robot is a mis-click, not a goal. */
const MIN_GOAL_DISTANCE_M = 0.25;

export function createNavPanel({ root, client, tracker }) {
  // Uma leitura, na montagem. Ver o cabeçalho de palette.js.
  const palette = readMapPalette(root);

  const canvas = root.querySelector('[data-role="nav-canvas"]');
  const hud = root.querySelector('[data-role="nav-hud"]');
  const cancelButton = root.querySelector('[data-role="nav-cancel"]');
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
    /** 'idle' | 'sent' | 'running' | 'ok' | 'fail' | 'lost' | 'cancelled' */
    goalState: 'idle',
    goalDetail: '',
    feedback: null,
    handle: null,
    tf: null,
  };

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

  const viewNow = () =>
    createView(
      state.gridInfo ? extentOfGrid(state.gridInfo) : defaultExtent(),
      cssWidth,
      cssHeight,
    );

  // --- subscriptions ------------------------------------------------------

  unsubscribes.push(
    client.subscribe(
      '/global_costmap/costmap',
      'nav_msgs/msg/OccupancyGrid',
      (message) => {
        tracker.mark('costmap');
        rasterizeCostmap(message);
      },
      // Measured 24/08/2026: 456,8 KiB per frame as JSON, 13,8 KiB as PNG.
      // Without this the panel alone would spend ~230 KiB/s on localhost and
      // would not survive the bench Ethernet link at all.
      { compression: 'png' },
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
  // others: with map->odom missing the HUD reports "sem TF map→base (21
  // arestas)", the laser is not drawn, and goals still go out but land without
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
        state.goalDetail = 'link caiu — a meta pode continuar no robô';
      } else if (outcome.notSent) {
        state.goalState = 'fail';
        state.goalDetail = 'sem conexão com o rosbridge';
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
    const rect = canvas.getBoundingClientRect();
    sendGoal(viewNow().toWorld(event.clientX - rect.left, event.clientY - rect.top));
  });

  cancelButton?.addEventListener('click', cancelActive);

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
    // Alfa no contexto e não embutido na cor: o token é um valor sólido, e
    // dissolvê-lo em rgba() aqui recriaria a duplicação que palette.js remove.
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
    // Meta já cumprida ou cancelada continua desenhada, apagada: some do
    // primeiro plano sem sumir da tela, que é como se lê "estava indo ali".
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
    idle: 'clique no mapa para mandar uma meta',
    sent: 'meta enviada',
    running: 'navegando',
    ok: 'meta cumprida',
    fail: 'meta falhou',
    lost: 'meta perdida',
    cancelled: 'meta cancelada',
  };

  function updateHud() {
    const parts = [GOAL_LABELS[state.goalState] ?? state.goalState];
    const remaining = state.feedback?.distance_remaining;
    if (Number.isFinite(remaining)) parts.push(`${remaining.toFixed(2)} m restantes`);
    const recoveries = state.feedback?.number_of_recoveries;
    if (recoveries > 0) parts.push(`${recoveries} recuperação(ões)`);
    if (state.goalDetail) parts.push(state.goalDetail);
    // Name WHY the pose is missing. "sem TF" alone sends the operator to the
    // wrong place half the time: no edges at all means the /tf relay is not
    // arriving, while a populated tree with no chain to `base` means a
    // publisher is missing (map->odom comes from odom_tf, odom->base from the
    // plant) — different containers, different fixes.
    if (!robotPose()) {
      const known = state.tf?.frames().length ?? 0;
      parts.push(known ? `sem TF map→base (${known} arestas)` : 'esperando TF');
    }

    const text = parts.join(' · ');
    if (hud.textContent !== text) hud.textContent = text;
    hud.dataset.state = state.goalState;

    const active = state.goalState === 'sent' || state.goalState === 'running';
    if (cancelButton) cancelButton.hidden = !active;
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
