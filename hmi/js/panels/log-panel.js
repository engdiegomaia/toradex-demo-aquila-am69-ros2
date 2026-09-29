/**
 * Movement log panel — target operations plus a live telemetry strip.
 *
 * `/rosout` is intentionally not the primary feed. In HIL it mixes Gazebo,
 * rosbridge, Nav2, the host and the module, which is useful for debugging and
 * noisy for operating. The cockpit shows `/demo/target/ops_log` first: short
 * lines generated on the target from `/demo/cmd_vel_si`, `/demo/cmd_vel` and
 * `/demo/odom`. `/rosout` remains subscribed only as a filtered fallback for
 * warnings/errors and selected navigation events.
 */

import { TOPICS } from '../config.js';

/** Ring size. Enough to cover a Nav2 startup burst, small enough to stay smooth. */
const MAX_ROWS = 400;

/** rcl_interfaces/msg/Log severity bytes. */
const LEVEL_NAMES = Object.freeze({
  10: 'debug',
  20: 'info',
  30: 'warn',
  40: 'error',
  50: 'fatal',
});

const ROSOUT_IMPORTANT_NODES = new Set([
  'bt_navigator',
  'controller_server',
  'planner_server',
  'collision_monitor',
  'lifecycle_manager_navigation',
  'nav_control_relay',
  'cmd_vel_si_to_stick',
  'target_monitor',
]);

/** Pure: severity byte -> lowercase name. Exported for tests. */
export function levelName(level) {
  return LEVEL_NAMES[level] ?? 'info';
}

/** Pure: builtin_interfaces/Time -> hh:mm:ss. Exported for tests. */
export function formatStamp(stamp) {
  if (!stamp || typeof stamp.sec !== 'number') return '--:--:--';
  const date = new Date(stamp.sec * 1000);
  return date.toISOString().slice(11, 19);
}

/** Pure: UNIX seconds -> hh:mm:ss. Exported for tests. */
export function formatWallStamp(seconds) {
  if (!Number.isFinite(seconds)) return '--:--:--';
  return new Date(seconds * 1000).toISOString().slice(11, 19);
}

/** Pure: quaternion -> yaw in radians. Exported for tests. */
export function yawFromQuaternion(q) {
  if (!q) return 0;
  const { x = 0, y = 0, z = 0, w = 1 } = q;
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

const fixed = (value, digits = 2) =>
  Number.isFinite(value) ? value.toFixed(digits) : '—';

const percent = (value, digits = 0) =>
  Number.isFinite(value) ? `${value.toFixed(digits)}%` : '—';

const celsius = (value) =>
  Number.isFinite(value) ? `${value.toFixed(1)}°C` : '—';

const megabytes = (value) =>
  Number.isFinite(value) ? `${value.toFixed(0)} MB` : '—';

/** Pure: parse std_msgs/String JSON, returning null for old/plain publishers. */
export function parseJsonString(message) {
  const raw = message?.data;
  if (typeof raw !== 'string' || raw.trim() === '') return null;
  try {
    return JSON.parse(raw);
  } catch {
    return null;
  }
}

/** Pure: resource payload -> compact memory label. Exported for tests. */
export function formatMemory(status) {
  const used = status?.mem_used_mb;
  const total = status?.mem_total_mb;
  const pct = status?.mem_percent;
  if (Number.isFinite(used) && Number.isFinite(total)) {
    return `${megabytes(used)} / ${megabytes(total)}`;
  }
  return percent(pct);
}

/** Pure: keep only /rosout lines that help an operator during HIL. */
export function shouldAppendRosout(message) {
  if (!message) return false;
  if (message.level >= 30) return true;
  const name = String(message.name ?? '').replace(/^\//, '');
  if (!ROSOUT_IMPORTANT_NODES.has(name)) return false;
  const text = String(message.msg ?? '').toLowerCase();
  return /(goal|reset|restart|clear|costmap|saturat|active|inactive|fail|abort)/.test(text);
}

export function createLogPanel({ root, client, tracker }) {
  const list = root.querySelector('[data-role="log-list"]');
  const values = {
    lin: root.querySelector('[data-role="tele-lin"]'),
    ang: root.querySelector('[data-role="tele-ang"]'),
    pose: root.querySelector('[data-role="tele-pose"]'),
    yaw: root.querySelector('[data-role="tele-yaw"]'),
    cpu: root.querySelector('[data-role="tele-cpu"]'),
    mem: root.querySelector('[data-role="tele-mem"]'),
    temp: root.querySelector('[data-role="tele-temp"]'),
  };

  const unsubscribes = [];

  const appendRow = ({ stampText, level = 'info', node = '', msg = '' }) => {
    // Auto-scroll only while the operator is already at the bottom. Yanking the
    // view down while someone is reading an error from ten seconds ago is the
    // fastest way to make a log panel useless during a demo.
    const pinned =
      list.scrollTop + list.clientHeight >= list.scrollHeight - 24;

    const row = document.createElement('li');
    row.className = 'log__row';
    row.dataset.level = level;

    const time = document.createElement('span');
    time.className = 'log__time';
    time.textContent = stampText ?? '--:--:--';

    const levelCell = document.createElement('span');
    levelCell.className = 'log__level';
    levelCell.textContent = level.toUpperCase().slice(0, 4);

    const nodeCell = document.createElement('span');
    nodeCell.className = 'log__node';
    nodeCell.textContent = node;
    nodeCell.title = node;

    const text = document.createElement('span');
    text.className = 'log__msg';
    // textContent, never innerHTML: target logs are generated by us, but the
    // filtered /rosout fallback can still carry text from another machine.
    text.textContent = msg;
    text.title = msg;

    row.append(time, levelCell, nodeCell, text);
    list.append(row);

    while (list.childElementCount > MAX_ROWS) list.firstElementChild.remove();
    if (pinned) list.scrollTop = list.scrollHeight;
  };

  unsubscribes.push(
    client.subscribe(
      TOPICS.targetOpsLog,
      'std_msgs/msg/String',
      (message) => {
        tracker.mark('targetOps');
        const payload = parseJsonString(message);
        appendRow({
          stampText: formatWallStamp(payload?.stamp),
          level: payload?.level ?? 'info',
          node: payload?.node ?? 'target',
          msg: payload?.msg ?? message?.data ?? '',
        });
      },
      { queueLength: 100 },
    ),
  );

  unsubscribes.push(
    client.subscribe(
      TOPICS.rosout,
      'rcl_interfaces/msg/Log',
      (message) => {
        tracker.mark('rosout');
        if (!shouldAppendRosout(message)) return;
        appendRow({
          stampText: formatStamp(message.stamp),
          level: levelName(message.level),
          node: message.name ?? '',
          msg: message.msg ?? '',
        });
      },
      { queueLength: 100 },
    ),
  );

  unsubscribes.push(
    client.subscribe(
      TOPICS.cmdVelSi,
      'geometry_msgs/msg/Twist',
      (message) => {
        tracker.mark('cmdVelSi');
        values.lin.textContent = fixed(message?.linear?.x);
        values.ang.textContent = fixed(message?.angular?.z);
      },
      { throttleRate: 100 },
    ),
  );

  unsubscribes.push(
    client.subscribe(
      TOPICS.targetStatus,
      'std_msgs/msg/String',
      (message) => {
        tracker.mark('targetStatus');
        const status = parseJsonString(message);
        values.cpu.textContent = percent(status?.cpu_percent);
        values.mem.textContent = formatMemory(status);
        values.temp.textContent = celsius(status?.temp_c);
      },
      { throttleRate: 1000 },
    ),
  );

  unsubscribes.push(
    client.subscribe(
      TOPICS.odom,
      'nav_msgs/msg/Odometry',
      (message) => {
        tracker.mark('odom');
        const position = message?.pose?.pose?.position;
        const orientation = message?.pose?.pose?.orientation;
        values.pose.textContent =
          position ? `${fixed(position.x)}, ${fixed(position.y)}` : '—';
        values.yaw.textContent = `${fixed(
          (yawFromQuaternion(orientation) * 180) / Math.PI,
          1,
        )}°`;
      },
      { throttleRate: 200 },
    ),
  );

  return {
    onLinkDown() {
      tracker.clear('targetOps');
      tracker.clear('targetStatus');
      tracker.clear('rosout');
      tracker.clear('cmdVelSi');
      tracker.clear('odom');
      for (const value of Object.values(values)) value.textContent = '—';
    },
    destroy() {
      for (const off of unsubscribes) off();
    },
  };
}
