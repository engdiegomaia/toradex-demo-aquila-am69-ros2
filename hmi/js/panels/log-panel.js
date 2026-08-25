/**
 * Movement log panel — /rosout plus a live telemetry strip.
 *
 * The panel answers two different operator questions with one region:
 *
 *   "what is the stack saying?"  -> the /rosout list;
 *   "what is the robot doing?"   -> the cmd_vel / odom strip above it.
 *
 * In HIL mode /rosout carries lines from BOTH machines over DDS, so the node
 * column is not decoration: it is how you tell a message from Nav2 on the
 * Aquila apart from one from the bridge on the workstation.
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

/** Pure: quaternion -> yaw in radians. Exported for tests. */
export function yawFromQuaternion(q) {
  if (!q) return 0;
  const { x = 0, y = 0, z = 0, w = 1 } = q;
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

const fixed = (value, digits = 2) =>
  Number.isFinite(value) ? value.toFixed(digits) : '—';

export function createLogPanel({ root, client, tracker }) {
  const list = root.querySelector('[data-role="log-list"]');
  const values = {
    lin: root.querySelector('[data-role="tele-lin"]'),
    ang: root.querySelector('[data-role="tele-ang"]'),
    pose: root.querySelector('[data-role="tele-pose"]'),
    yaw: root.querySelector('[data-role="tele-yaw"]'),
  };

  const unsubscribes = [];

  const appendRow = (message) => {
    // Auto-scroll only while the operator is already at the bottom. Yanking the
    // view down while someone is reading an error from ten seconds ago is the
    // fastest way to make a log panel useless during a demo.
    const pinned =
      list.scrollTop + list.clientHeight >= list.scrollHeight - 24;

    const row = document.createElement('li');
    row.className = 'log__row';
    row.dataset.level = levelName(message.level);

    const time = document.createElement('span');
    time.className = 'log__time';
    time.textContent = formatStamp(message.stamp);

    const level = document.createElement('span');
    level.className = 'log__level';
    level.textContent = levelName(message.level).toUpperCase().slice(0, 4);

    const node = document.createElement('span');
    node.className = 'log__node';
    node.textContent = message.name ?? '';
    node.title = message.name ?? '';

    const text = document.createElement('span');
    text.className = 'log__msg';
    // textContent, never innerHTML: /rosout carries text from nodes we do not
    // control, and in HIL mode from another machine entirely.
    text.textContent = message.msg ?? '';
    text.title = message.msg ?? '';

    row.append(time, level, node, text);
    list.append(row);

    while (list.childElementCount > MAX_ROWS) list.firstElementChild.remove();
    if (pinned) list.scrollTop = list.scrollHeight;
  };

  unsubscribes.push(
    client.subscribe(
      TOPICS.rosout,
      'rcl_interfaces/msg/Log',
      (message) => {
        tracker.mark('rosout');
        appendRow(message);
      },
      { queueLength: 100 },
    ),
  );

  unsubscribes.push(
    client.subscribe(
      TOPICS.cmdVel,
      'geometry_msgs/msg/Twist',
      (message) => {
        tracker.mark('cmdVel');
        values.lin.textContent = fixed(message?.linear?.x);
        values.ang.textContent = fixed(message?.angular?.z);
      },
      { throttleRate: 100 },
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
      tracker.clear('rosout');
      tracker.clear('cmdVel');
      tracker.clear('odom');
      for (const value of Object.values(values)) value.textContent = '—';
    },
    destroy() {
      for (const off of unsubscribes) off();
    },
  };
}
