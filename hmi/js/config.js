/**
 * Runtime configuration.
 *
 * AGENTS.md §5.7: "avoid hard-coded hostnames" and "provide development
 * configuration for localhost". Three layers, most specific first:
 *
 *   1. URL query string   — ?host=aquila.local&rosbridgePort=9090
 *   2. /config.json       — rendered by nginx from the compose environment
 *   3. DEFAULTS below     — opening index.html straight from a checkout works
 *
 * `host: ''` means "whatever hostname served this page", which is the value
 * that is correct in every deployment we have: workstation today, module kiosk
 * in M3, and a laptop pointed at the bench in between. A literal "localhost"
 * default would work on the developer machine and fail on the bench — the
 * project's characteristic failure mode.
 */

export const DEFAULTS = Object.freeze({
  host: '',
  rosbridgePort: 9090,
  videoPort: 8080,
  build: 'dev',
});

/**
 * Topics the cockpit reads. Kept here rather than inside the panels so the
 * topic contract (CLAUDE.md) is visible in one place, and so a rename is one
 * edit. These are contract names — changing one here does not change ROS.
 */
export const TOPICS = Object.freeze({
  camera: '/demo/camera/image_raw',
  // The two external "operator" views, from cameras spawned into whatever world
  // is loaded (demo_simulation/launch/scene_cameras.launch.py). They are the
  // only panel that answers "did the robot move?" without trusting any node in
  // the stack, which is why there are two of them and why they carry their own
  // camera_info-free heartbeat below.
  sceneIso: '/demo/cockpit/scene_iso/image_raw',
  sceneTop: '/demo/cockpit/scene_top/image_raw',
  scan: '/demo/scan',
  odom: '/demo/odom',
  cmdVel: '/demo/cmd_vel',
  detections: '/demo/perception/detections',
  navStatus: '/demo/navigation/status',
  rosout: '/rosout',
});

const NUMERIC_KEYS = ['rosbridgePort', 'videoPort'];

function coerce(raw) {
  const config = { ...DEFAULTS, ...raw };
  for (const key of NUMERIC_KEYS) {
    const value = Number(config[key]);
    // A non-numeric port from a hand-edited query string must not silently
    // become NaN and produce ws://host:NaN/, which fails with a DOM exception
    // that names nothing.
    config[key] = Number.isFinite(value) && value > 0 ? value : DEFAULTS[key];
  }
  config.host = String(config.host ?? '');
  config.build = String(config.build ?? DEFAULTS.build);
  return Object.freeze(config);
}

/** Pure: parse overrides out of a query string. Exported for tests. */
export function overridesFromQuery(search) {
  const params = new URLSearchParams(search ?? '');
  const overrides = {};
  for (const key of ['host', 'build', ...NUMERIC_KEYS]) {
    if (params.has(key)) overrides[key] = params.get(key);
  }
  return overrides;
}

/** Pure: merge the three layers. Exported for tests. */
export function resolveConfig({ served = {}, query = {} } = {}) {
  return coerce({ ...served, ...query });
}

/** Pure: build the endpoint URLs a config implies. Exported for tests. */
export function endpointsFor(config, pageLocation) {
  const host = config.host || pageLocation.hostname || 'localhost';
  const wsScheme = pageLocation.protocol === 'https:' ? 'wss:' : 'ws:';
  const httpScheme = pageLocation.protocol === 'https:' ? 'https:' : 'http:';
  return Object.freeze({
    rosbridge: `${wsScheme}//${host}:${config.rosbridgePort}`,
    video: `${httpScheme}//${host}:${config.videoPort}`,
  });
}

/**
 * Percent-encode a ROS topic for a web_video_server query, KEEPING the slashes.
 *
 * web_video_server does not percent-decode `topic`. Measured on 24/08/2026
 * against ros-jazzy-web-video-server 3.1.0:
 *
 *   topic=%2Fdemo%2Fcamera%2Fimage_raw  -> HTTP 200, 22 bytes, no stream
 *   topic=/demo/camera/image_raw        -> HTTP 200, MJPEG flows
 *
 * Both answer 200, which is why this is worth a named function: URLSearchParams
 * escapes `/` by default, and the resulting panel is blank with a healthy
 * status code and nothing in any log.
 */
export function encodeTopicParam(topic) {
  return encodeURIComponent(topic).replace(/%2F/gi, '/');
}

/**
 * Qualidade JPEG dos streams, 1-100.
 *
 * 95 e nao 70: o painel de cena mostra um render 3D com superficies grandes e
 * de cor quase uniforme, e e exatamente nesse conteudo que o ringing do JPEG
 * aparece — as paredes do labirinto ficavam sujas nas bordas. O custo e
 * largura de banda, que em `learn` e localhost.
 *
 * No modo hil o stream atravessa a Ethernet ate o Aquila. Se a banda apertar,
 * o lugar de baixar isto e aqui, e nao a resolucao da camera: reduzir a
 * qualidade degrada suavemente, reduzir a resolucao muda o enquadramento em
 * pixels e desalinha as caixas desenhadas por cima.
 */
export const STREAM_QUALITY = 95;

/**
 * Build the web_video_server MJPEG URL for a topic.
 *
 * The cache-buster is not superstition: a reconnecting <img> pointed at an
 * identical URL is served from the browser's cache as a dead stream in Chromium,
 * so the panel shows a still frame and reports itself healthy.
 */
export function streamUrl(videoBase, topic, { quality = STREAM_QUALITY, nonce = 0 } = {}) {
  const query = [
    `topic=${encodeTopicParam(topic)}`,
    'type=mjpeg',
    `quality=${encodeURIComponent(quality)}`,
  ];
  if (nonce) query.push(`_=${encodeURIComponent(nonce)}`);
  return `${videoBase}/stream?${query.join('&')}`;
}

/** Fetch /config.json, tolerating its absence (file:// or bare http server). */
export async function loadServedConfig(fetchFn = fetch) {
  try {
    const response = await fetchFn('config.json', { cache: 'no-store' });
    if (!response.ok) return {};
    return await response.json();
  } catch {
    // Not an error: opening the bundle without nginx is a supported
    // development path, and DEFAULTS cover it.
    return {};
  }
}
