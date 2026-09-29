/**
 * The camera commands that leave the cockpit.
 *
 * The simulator node saturates and integrates; only a delta leaves from here.
 * What can go wrong on this side is the SHAPE of the message — a missing field
 * in a TwistStamped is not an error anywhere in the chain, just a camera that
 * does not move.
 */

import assert from 'node:assert/strict';
import { describe, it } from 'node:test';

import { VIEW_STEPS, viewCommand } from '../js/panels/view-controls.js';

describe('viewCommand', () => {
  it('fills all six twist fields, not just the one that changes', () => {
    // rosbridge does not complete missing nested fields: a linear without `z`
    // reaches the node as an incomplete message, with no error anywhere.
    const message = viewCommand('orbit-left', 'scene_iso');
    assert.deepEqual(Object.keys(message.twist.linear).sort(), ['x', 'y', 'z']);
    assert.deepEqual(Object.keys(message.twist.angular).sort(), ['x', 'y', 'z']);
    for (const axis of ['x', 'y', 'z']) {
      assert.equal(typeof message.twist.linear[axis], 'number');
      assert.equal(typeof message.twist.angular[axis], 'number');
    }
  });

  it('frame_id picks the camera', () => {
    assert.equal(viewCommand('zoom-in', 'scene_top').header.frame_id, 'scene_top');
  });

  it('carries a stamp, because the type is TwistStamped', () => {
    const { stamp } = viewCommand('zoom-in', 'scene_iso').header;
    assert.deepEqual(stamp, { sec: 0, nanosec: 0 });
  });

  it('zooming in reduces the distance to the target', () => {
    // linear.x is the change in distance: an inverted sign here would move the
    // camera away on the "+" button, which is the kind of error nobody spots
    // when reading the code.
    assert.ok(viewCommand('zoom-in', 'scene_iso').twist.linear.x < 0);
    assert.ok(viewCommand('zoom-out', 'scene_iso').twist.linear.x > 0);
  });

  it('an unknown command returns null instead of an empty message', () => {
    // A message of zeroed deltas would be accepted by the node and do nothing.
    assert.equal(viewCommand('does-not-exist', 'scene_iso'), null);
  });

  it('each button pair is symmetric', () => {
    const pairs = [
      ['orbit-left', 'orbit-right'],
      ['pitch-up', 'pitch-down'],
      ['zoom-in', 'zoom-out'],
      ['pan-left', 'pan-right'],
      ['pan-forward', 'pan-back'],
    ];
    for (const [a, b] of pairs) {
      const ma = viewCommand(a, 'scene_iso').twist;
      const mb = viewCommand(b, 'scene_iso').twist;
      for (const kind of ['linear', 'angular']) {
        for (const axis of ['x', 'y', 'z']) {
          // `+ 0` normalises -0: assert.equal tells 0 from -0, and the unused
          // axes of a symmetric pair fall exactly in that case.
          assert.equal(
            ma[kind][axis] + 0,
            -mb[kind][axis] + 0,
            `${a}/${b} diverge at ${kind}.${axis}`,
          );
        }
      }
    }
  });

  it('every pad button has a defined step', () => {
    // The pad is built from the HTML; a data-command with no entry here would
    // be a button that does nothing and does not complain.
    assert.equal(Object.keys(VIEW_STEPS).length, 10);
  });
});
