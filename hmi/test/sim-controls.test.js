/**
 * The simulation state label.
 *
 * What is under test is the only real decision of this panel: telling
 * "paused" apart from "no simulator". The two look the same from the outside —
 * nothing happens on screen — and confusing them makes the operator click play
 * on a dead container.
 */

import assert from 'node:assert/strict';
import { describe, it } from 'node:test';

import {
  clockSeconds,
  createClockWatch,
  OFFLINE_AFTER_MS,
  SimState,
  STALL_AFTER_MS,
} from '../js/panels/sim-controls.js';

describe('clockSeconds', () => {
  it('joins sec and nanosec into seconds', () => {
    assert.equal(clockSeconds({ clock: { sec: 12, nanosec: 500_000_000 } }), 12.5);
  });

  it('treats an empty message as zero instead of NaN', () => {
    // `NaN !== NaN` is always true, so a
    // malformed message would freeze the clock at "changed just now" forever
    // and the simulation would look eternally alive.
    assert.equal(clockSeconds({}), 0);
    assert.equal(clockSeconds(undefined), 0);
  });
});

describe('createClockWatch', () => {
  it('with no sample at all, there is no simulator', () => {
    assert.equal(createClockWatch().state(1000), SimState.OFFLINE);
  });

  it('an advancing clock means the simulation is running', () => {
    const watch = createClockWatch();
    watch.sample(1.0, 1000);
    watch.sample(1.4, 1400);
    assert.equal(watch.state(1500), SimState.RUNNING);
  });

  it('samples arriving with the SAME simulated time mean paused', () => {
    const watch = createClockWatch();
    watch.sample(1.0, 1000);
    watch.sample(1.0, 1400);
    watch.sample(1.0, 1800);
    // The bridge is alive (samples arrive) but the world is not moving.
    assert.equal(watch.state(1000 + STALL_AFTER_MS + 1), SimState.PAUSED);
  });

  it('no longer receiving samples means no simulator, not paused', () => {
    const watch = createClockWatch();
    watch.sample(1.0, 1000);
    watch.sample(1.4, 1400);
    assert.equal(watch.state(1400 + OFFLINE_AFTER_MS + 1), SimState.OFFLINE);
  });

  it('absence beats pause when both conditions hold', () => {
    // A dead container satisfies both: time stopped AND samples stopped. The
    // label has to be the one that sends the operator to look at the
    // container.
    const watch = createClockWatch();
    watch.sample(1.0, 1000);
    assert.equal(watch.state(1000 + OFFLINE_AFTER_MS + 1), SimState.OFFLINE);
  });

  it('reset goes back to unknown', () => {
    const watch = createClockWatch();
    watch.sample(1.0, 1000);
    watch.reset();
    assert.equal(watch.state(1050), SimState.OFFLINE);
  });

  it('the first sample after a reset does not count as advance', () => {
    // Without this, reconnecting with the simulation paused would show
    // "running" for STALL_AFTER_MS, which is exactly the moment someone decides
    // to click.
    const watch = createClockWatch();
    watch.sample(7.0, 1000);
    watch.reset();
    watch.sample(7.0, 5000);
    watch.sample(7.0, 5400);
    assert.equal(watch.state(5000 + STALL_AFTER_MS + 1), SimState.PAUSED);
  });

  it('the pause threshold is larger than the interval between samples', () => {
    // Regression guard on the constant, not on the code: with samples every
    // 400 ms a tight threshold would flash "paused" on every network jitter.
    assert.ok(STALL_AFTER_MS > 400 * 2);
    assert.ok(OFFLINE_AFTER_MS > STALL_AFTER_MS);
  });
});

