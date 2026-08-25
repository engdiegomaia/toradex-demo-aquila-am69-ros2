/**
 * Cockpit entry point — wiring only.
 *
 * Business logic lives in js/ros/* (transport, staleness) and js/panels/*
 * (rendering). AGENTS.md §5.7: "keep business logic outside visual components".
 * This file resolves configuration, builds the client, mounts the panels, and
 * runs the one shared repaint tick.
 *
 * ML3.5 cockpit-web F1 + F3b. Five panels, one WebSocket, one timer.
 *
 * Two objects are built here and injected rather than created inside the panels
 * that use them, because both are shared and both are expensive to duplicate:
 *
 *   the PNG decoder  one offscreen canvas, reused by every compressed topic;
 *   the TF tree      one cache fed by /tf and /tf_static. A second subscriber
 *                    to a ~380 Hz topic would double the relay cost for a tree
 *                    that is identical either way.
 */

import {
  endpointsFor,
  loadServedConfig,
  overridesFromQuery,
  resolveConfig,
  TOPICS,
} from './config.js';
import { ConnectionState, RosbridgeClient } from './ros/rosbridge-client.js';
import { FreshnessTracker } from './ros/freshness.js';
import { createPngDecoder } from './ros/png-decompress.js';
import { TfTree } from './ros/tf-tree.js';
import { createStreamPanel } from './panels/stream-panel.js';
import { createNavPanel } from './panels/nav-panel.js';
import { createLogPanel } from './panels/log-panel.js';
import { createControlBar } from './panels/control-bar.js';
import { createSimControls } from './panels/sim-controls.js';
import { createViewControls } from './panels/view-controls.js';

/** One repaint tick for every freshness dot. 4 Hz reads as instant and costs nothing. */
const TICK_MS = 250;

/**
 * Per-source staleness windows.
 *
 * These are transport budgets, not publication rates: a value must be generous
 * enough that normal jitter never flickers the dot amber, because a dot that
 * cries wolf is a dot nobody reads.
 */
const SOURCES = Object.freeze({
  camera: { staleAfterMs: 2000 },
  // The scene cameras run at 5 Hz by design (scene_cameras.launch.py), so the
  // budget is looser than the robot camera's.
  scene: { staleAfterMs: 3000 },
  rosout: { staleAfterMs: 15000 },
  cmdVel: { staleAfterMs: 2000 },
  odom: { staleAfterMs: 2000 },
  // Nav2 publishes the global costmap at 0,5 Hz and republishes the footprint
  // with it. Anything under ~4 s here would sit amber during normal operation.
  costmap: { staleAfterMs: 6000 },
  scan: { staleAfterMs: 2000 },
});

/**
 * The two scene cameras, keyed by the value of the toggle's data-source.
 *
 * Each carries its own heartbeat: the camera_info bridged next to the image.
 * See stream-panel.js for why an <img> cannot report its own liveness.
 */
const SCENE_VIEWS = Object.freeze({
  iso: {
    // O nome que o scene_view_controller espera em header.frame_id. Não é o
    // mesmo do botão ('iso') nem o do modelo no Gazebo ('cockpit_scene_iso'),
    // e escrevê-lo aqui é o que evita adivinhar no meio do handler.
    frame: 'scene_iso',
    topic: TOPICS.sceneIso,
    heartbeat: {
      topic: '/demo/cockpit/scene_iso/camera_info',
      type: 'sensor_msgs/msg/CameraInfo',
    },
  },
  top: {
    frame: 'scene_top',
    topic: TOPICS.sceneTop,
    heartbeat: {
      topic: '/demo/cockpit/scene_top/camera_info',
      type: 'sensor_msgs/msg/CameraInfo',
    },
  },
});

async function start() {
  const served = await loadServedConfig();
  const config = resolveConfig({
    served,
    query: overridesFromQuery(window.location.search),
  });
  const endpoints = endpointsFor(config, window.location);

  const shell = document.querySelector('[data-role="cockpit"]');
  const tracker = new FreshnessTracker();
  for (const [key, options] of Object.entries(SOURCES)) {
    tracker.register(key, options);
  }

  const client = new RosbridgeClient({
    url: endpoints.rosbridge,
    // Console only. A toast for every reconnect attempt would cover the panels
    // during exactly the outage the operator is trying to watch.
    onError: (message, error) => console.warn('[cockpit]', message, error ?? ''),
    // Without this the client cannot honour `compression: 'png'` and every
    // subscriber that asks for it would silently receive nothing. The costmap
    // is 456,8 KiB per frame as JSON and 13,8 KiB as PNG (measured).
    decodePng: createPngDecoder(),
  });

  // One cache, fed once. The nav panel is its only reader today; the injection
  // point exists so that a second consumer costs a getter and not a second
  // subscription to a 380 Hz topic.
  const tf = new TfTree();

  const scenePanel = createStreamPanel({
    root: document.querySelector('[data-panel="scene"]'),
    client,
    tracker,
    videoBase: endpoints.video,
    topic: SCENE_VIEWS.iso.topic,
    heartbeat: SCENE_VIEWS.iso.heartbeat,
    freshnessKey: 'scene',
  });

  const panels = [
    scenePanel,
    createNavPanel({
      root: document.querySelector('[data-panel="nav"]'),
      client,
      tracker,
    }).useTf(tf),
    createStreamPanel({
      root: document.querySelector('[data-panel="camera"]'),
      client,
      tracker,
      videoBase: endpoints.video,
      topic: TOPICS.camera,
      // Liveness proxy — see the header of stream-panel.js for why the <img>
      // cannot answer this itself. camera_info is bridged alongside the image
      // in demo_simulation/config/bridge_quadruped.yaml.
      heartbeat: {
        topic: '/demo/camera/camera_info',
        type: 'sensor_msgs/msg/CameraInfo',
      },
      freshnessKey: 'camera',
    }),
    createLogPanel({
      root: document.querySelector('[data-panel="log"]'),
      client,
      tracker,
    }),
    createControlBar({
      root: document.querySelector('[data-role="bar"]'),
      client,
      endpoints,
      build: config.build,
      shell,
    }),
    // Play/pause/reset do Gazebo. O simulador continua na workstation x86
    // (regra 1 do CLAUDE.md); o que sai daqui é uma chamada de serviço.
    createSimControls({
      root: document.querySelector('[data-role="bar"]'),
      client,
      onNotice: (message) => console.warn('[cockpit]', message),
    }),
  ];

  // Não entra em `panels`: não tem tick nem estado de frescor, e o único ciclo
  // de vida que lhe interessa é o do link, que ele mesmo observa.
  const viewControls = createViewControls({
    root: document.querySelector('[data-panel="scene"]'),
    client,
    camera: SCENE_VIEWS.iso.frame,
    onNotice: (message) => console.warn('[cockpit]', message),
  });

  client.onStateChange((state) => {
    if (state === ConnectionState.CONNECTED) return;
    // Everything the WebSocket fed is now unknown, not merely old. Without this
    // the dots stay green for the length of their window after the link dies.
    for (const panel of panels) panel.onLinkDown?.();
  });

  // --- scene view toggle ---------------------------------------------------
  // aria-pressed is both the accessible state and the CSS hook, so it is set
  // here and nowhere else. The source label follows it: a panel that says one
  // topic while showing another is worse than a panel with no label at all.
  const sceneButtons = [...document.querySelectorAll('[data-role="scene-source"]')];
  const sceneLabel = document.querySelector('[data-role="scene-source-label"]');
  for (const button of sceneButtons) {
    button.addEventListener('click', () => {
      const view = SCENE_VIEWS[button.dataset.source];
      if (!view) return;
      scenePanel.setSource(view);
      // Os botões de câmera passam a comandar a imagem que está na tela.
      // Comandar a outra é o erro que se lê como "os botões não funcionam".
      viewControls.setCamera(view.frame);
      for (const other of sceneButtons) {
        other.setAttribute('aria-pressed', String(other === button));
      }
      sceneLabel.textContent = view.topic;
    });
  }

  // One timer for the whole cockpit. Panels expose tick() instead of owning
  // intervals, so the repaint cost is bounded no matter how many panels are
  // mounted, and so a paused tab resumes everything in step.
  const dots = [...document.querySelectorAll('[data-freshness]')];
  window.setInterval(() => {
    for (const dot of dots) {
      dot.dataset.state = tracker.stateOf(dot.dataset.freshness);
    }
    for (const panel of panels) panel.tick?.();
  }, TICK_MS);

  client.connect();

  // Closing the tab without this leaves rosbridge holding the subscriptions
  // until its own timeout; on a bench where the page is reloaded dozens of
  // times that accumulates into real publisher-side work.
  window.addEventListener('beforeunload', () => client.close());
}

start().catch((error) => {
  console.error('[cockpit] falha ao iniciar', error);
});
