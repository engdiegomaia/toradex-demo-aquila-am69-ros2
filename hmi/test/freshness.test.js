/** Stale-data state machine (AGENTS.md §5.7). */

import assert from 'node:assert/strict';
import { describe, it } from 'node:test';

import { Freshness, FreshnessTracker } from '../js/ros/freshness.js';

function trackerAt(clock) {
  return new FreshnessTracker({ now: () => clock.t });
}

describe('FreshnessTracker', () => {
  it('reports never before anything arrives', () => {
    const clock = { t: 0 };
    const tracker = trackerAt(clock).register('camera', { staleAfterMs: 1000 });
    assert.equal(tracker.stateOf('camera'), Freshness.NEVER);
    assert.equal(tracker.ageOf('camera'), null);
  });

  it('reports live inside the window and stale outside it', () => {
    const clock = { t: 1000 };
    const tracker = trackerAt(clock).register('camera', { staleAfterMs: 500 });

    tracker.mark('camera');
    assert.equal(tracker.stateOf('camera'), Freshness.LIVE);

    clock.t += 500;
    assert.equal(tracker.stateOf('camera'), Freshness.LIVE, 'boundary is inclusive');

    clock.t += 1;
    assert.equal(tracker.stateOf('camera'), Freshness.STALE);
  });

  it('keeps stale distinct from never', () => {
    const clock = { t: 0 };
    const tracker = trackerAt(clock).register('camera', { staleAfterMs: 100 });
    tracker.mark('camera');
    clock.t += 5000;

    // A frozen picture must not look like an empty panel: one means the source
    // died mid-run, the other means it never started.
    assert.equal(tracker.stateOf('camera'), Freshness.STALE);
    assert.notEqual(tracker.stateOf('camera'), Freshness.NEVER);
  });

  it('clear() puts a source back to never', () => {
    const clock = { t: 0 };
    const tracker = trackerAt(clock).register('camera', { staleAfterMs: 1000 });
    tracker.mark('camera');
    tracker.clear('camera');
    assert.equal(tracker.stateOf('camera'), Freshness.NEVER);
  });

  it('clearAll() resets every source, which is what a link drop means', () => {
    const clock = { t: 0 };
    const tracker = trackerAt(clock)
      .register('camera', { staleAfterMs: 1000 })
      .register('odom', { staleAfterMs: 1000 });
    tracker.mark('camera');
    tracker.mark('odom');

    tracker.clearAll();

    assert.deepEqual(tracker.snapshot(), {
      camera: Freshness.NEVER,
      odom: Freshness.NEVER,
    });
  });

  it('auto-registers an unknown key on first mark', () => {
    const clock = { t: 0 };
    const tracker = trackerAt(clock);
    tracker.mark('surprise');
    assert.equal(tracker.stateOf('surprise'), Freshness.LIVE);
  });

  it('uses per-source windows independently', () => {
    const clock = { t: 0 };
    const tracker = trackerAt(clock)
      .register('camera', { staleAfterMs: 1000 })
      .register('rosout', { staleAfterMs: 15000 });
    tracker.mark('camera');
    tracker.mark('rosout');

    clock.t += 2000;

    assert.equal(tracker.stateOf('camera'), Freshness.STALE);
    assert.equal(tracker.stateOf('rosout'), Freshness.LIVE);
  });
});
