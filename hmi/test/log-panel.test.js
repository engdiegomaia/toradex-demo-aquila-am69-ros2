/** Message parsing for the log panel (AGENTS.md §5.7). */

import assert from 'node:assert/strict';
import { describe, it } from 'node:test';

import {
  formatMemory,
  formatStamp,
  formatWallStamp,
  levelName,
  parseJsonString,
  shouldAppendRosout,
  yawFromQuaternion,
} from '../js/panels/log-panel.js';

describe('levelName', () => {
  it('maps the rcl_interfaces severity bytes', () => {
    assert.equal(levelName(10), 'debug');
    assert.equal(levelName(20), 'info');
    assert.equal(levelName(30), 'warn');
    assert.equal(levelName(40), 'error');
    assert.equal(levelName(50), 'fatal');
  });

  it('degrades to info for an unknown byte instead of rendering undefined', () => {
    assert.equal(levelName(7), 'info');
    assert.equal(levelName(undefined), 'info');
  });
});

describe('formatStamp', () => {
  it('renders hh:mm:ss UTC from a builtin_interfaces/Time', () => {
    assert.equal(formatStamp({ sec: 3661, nanosec: 0 }), '01:01:01');
  });

  it('renders a placeholder for a missing stamp', () => {
    assert.equal(formatStamp(undefined), '--:--:--');
    assert.equal(formatStamp({}), '--:--:--');
  });
});

describe('formatWallStamp', () => {
  it('renders wall-clock unix seconds as hh:mm:ss UTC', () => {
    assert.equal(formatWallStamp(3661), '01:01:01');
  });

  it('renders a placeholder for invalid wall time', () => {
    assert.equal(formatWallStamp(undefined), '--:--:--');
    assert.equal(formatWallStamp(Number.NaN), '--:--:--');
  });
});

describe('parseJsonString', () => {
  it('parses JSON carried by std_msgs/String', () => {
    assert.deepEqual(parseJsonString({ data: '{"cpu_percent":12.5}' }), {
      cpu_percent: 12.5,
    });
  });

  it('returns null for plain text logs', () => {
    assert.equal(parseJsonString({ data: 'plain log' }), null);
    assert.equal(parseJsonString({}), null);
  });
});

describe('formatMemory', () => {
  it('prefers absolute used/total memory when available', () => {
    assert.equal(
      formatMemory({ mem_used_mb: 512.2, mem_total_mb: 4096.8, mem_percent: 12.5 }),
      '512 MB / 4097 MB',
    );
  });

  it('falls back to percent when absolute values are unavailable', () => {
    assert.equal(formatMemory({ mem_percent: 12.5 }), '13%');
  });
});

describe('shouldAppendRosout', () => {
  it('keeps warnings and errors from any node', () => {
    assert.equal(shouldAppendRosout({ level: 30, name: 'rosbridge', msg: 'warn' }), true);
    assert.equal(shouldAppendRosout({ level: 40, name: 'gazebo', msg: 'error' }), true);
  });

  it('keeps selected navigation info that changes operator state', () => {
    assert.equal(
      shouldAppendRosout({
        level: 20,
        name: 'nav_control_relay',
        msg: 'reset concluido',
      }),
      true,
    );
  });

  it('drops generic info noise', () => {
    assert.equal(
      shouldAppendRosout({ level: 20, name: 'rosbridge_websocket', msg: 'client connected' }),
      false,
    );
  });
});

describe('yawFromQuaternion', () => {
  const near = (actual, expected) =>
    assert.ok(
      Math.abs(actual - expected) < 1e-9,
      `${actual} is not close to ${expected}`,
    );

  it('is zero for the identity orientation', () => {
    near(yawFromQuaternion({ x: 0, y: 0, z: 0, w: 1 }), 0);
  });

  it('is +90 degrees for a quarter turn about z', () => {
    const half = Math.SQRT1_2;
    near(yawFromQuaternion({ x: 0, y: 0, z: half, w: half }), Math.PI / 2);
  });

  it('is -90 degrees for the opposite quarter turn', () => {
    const half = Math.SQRT1_2;
    near(yawFromQuaternion({ x: 0, y: 0, z: -half, w: half }), -Math.PI / 2);
  });

  it('treats a missing orientation as zero rather than NaN', () => {
    assert.equal(yawFromQuaternion(undefined), 0);
    near(yawFromQuaternion({}), 0);
  });
});
