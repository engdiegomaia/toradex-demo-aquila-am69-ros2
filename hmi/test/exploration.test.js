/**
 * The autonomous search as seen from the cockpit.
 *
 * The regressions this file exists to catch are all of the same family: the
 * cockpit believing one thing while the Aquila does another.
 *
 *   - an unreadable status treated as "there is no search", and the click on
 *     the map sends a manual goal again on top of an explorer that keeps
 *     running;
 *   - `state: 'completed'` painted as a confirmed exit, when the one that
 *     confirms it is the ground-truth validator and only it;
 *   - the HUD lost after a rosbridge drop, even though the topic is
 *     TRANSIENT_LOCAL and redelivers the last message by itself.
 */

import assert from 'node:assert/strict';
import { describe, it } from 'node:test';

import {
  BUSY_STATES,
  createExplorationStore,
  explorationHudParts,
  isExplorationActive,
  parseExplorationStatus,
  TERMINAL_STATES,
} from '../js/panels/exploration.js';

/** The same payload that `maze_explorer._publish_status()` builds. */
function status(fields = {}) {
  return {
    data: JSON.stringify({
      state: 'navigating',
      elapsed_s: 12.4,
      frontier_count: 3,
      goal: null,
      blacklisted: 0,
      marker_visible: false,
      message: '',
      ...fields,
    }),
  };
}

describe('parseExplorationStatus', () => {
  it('reads the JSON the explorer publishes', () => {
    const parsed = parseExplorationStatus(status({ frontier_count: 7 }));
    assert.equal(parsed.state, 'navigating');
    assert.equal(parsed.frontier_count, 7);
  });

  it('broken JSON becomes a VISIBLE failure, not the absence of a search', () => {
    // A `null` here would hand the manual click back to the operator in the
    // middle of a search that keeps running on the module: two goal sources on
    // the same Nav2.
    const parsed = parseExplorationStatus({ data: '{not json' });
    assert.equal(parsed.state, 'failed');
    assert.ok(parsed.message);
  });

  it('a message without a `data` field also becomes a failure', () => {
    assert.equal(parseExplorationStatus(undefined).state, 'failed');
    assert.equal(parseExplorationStatus({}).state, 'failed');
  });

  it('valid JSON that is not an object does not pass as a status', () => {
    // `JSON.parse('4')` and `JSON.parse('[]')` do not throw. Accepting them
    // would turn `state` into undefined and make the search vanish from the
    // screen with no error at all.
    assert.equal(parseExplorationStatus({ data: '4' }).state, 'failed');
    assert.equal(parseExplorationStatus({ data: '[]' }).state, 'failed');
    assert.equal(parseExplorationStatus({ data: 'null' }).state, 'failed');
  });
});

describe('isExplorationActive', () => {
  it('every busy state of the node blocks the manual goal', () => {
    for (const state of BUSY_STATES) {
      assert.equal(isExplorationActive({ state }), true, state);
    }
  });

  it('a terminal state releases control back to the operator', () => {
    for (const state of TERMINAL_STATES) {
      assert.equal(isExplorationActive({ state }), false, state);
    }
  });

  it('with no status at all, the cockpit belongs to the operator', () => {
    assert.equal(isExplorationActive(null), false);
    assert.equal(isExplorationActive({ state: 'idle' }), false);
  });

  it('the busy states of the ROS node are all here', () => {
    // Diverging from this list is the silent wiring failure: a new state in
    // maze_explorer that the cockpit does not recognise hands the manual click
    // back in the middle of the search. The list lives in two languages and
    // has to be compared.
    //
    // `starting` is left OUT of the comparison on purpose: it is the only
    // state in this list that the ROS node does not know. It covers the window
    // between the click and the first status, which exists only on the cockpit
    // side. A test in tests/test_maze_exploration_contract.py locks the other
    // half: `starting` must not appear in the maze_explorer vocabulary.
    const fromRos = [...BUSY_STATES].filter((name) => name !== 'starting');
    assert.deepEqual(fromRos.sort(),
      ['homing_exit', 'navigating', 'selecting', 'waiting_map']);
    assert.ok(BUSY_STATES.includes('starting'));
  });
});

describe('explorationHudParts', () => {
  it('shows time, frontiers and message in reading order', () => {
    const parsed = parseExplorationStatus(
      status({ elapsed_s: 41.6, frontier_count: 2, message: 'navigating' }));
    assert.deepEqual(explorationHudParts(parsed, false),
      ['42 s', '2 frontier(s)', 'navigating']);
  });

  it('announces the marker only when perception sees it', () => {
    const seen = parseExplorationStatus(status({ marker_visible: true }));
    assert.ok(explorationHudParts(seen, false).includes('exit detected'));
    const unseen = parseExplorationStatus(status({ marker_visible: false }));
    assert.ok(!explorationHudParts(unseen, false).includes('exit detected'));
  });

  it('`completed` is NOT a confirmed exit', () => {
    // `completed` says the explorer got close to the magenta panel. The one
    // that confirms the crossing of the opening is /demo/maze/escaped, from the
    // ground-truth validator, and it is the only one that may write that
    // label.
    const parsed = parseExplorationStatus(status({ state: 'completed' }));
    assert.ok(!explorationHudParts(parsed, false).includes('EXIT CONFIRMED'));
    assert.ok(explorationHudParts(parsed, true).includes('EXIT CONFIRMED'));
  });

  it('the confirmed exit survives without a search status', () => {
    assert.deepEqual(explorationHudParts(null, true), ['EXIT CONFIRMED']);
  });

  it('missing fields do not become NaN on screen', () => {
    const parsed = parseExplorationStatus({ data: '{"state":"selecting"}' });
    assert.deepEqual(explorationHudParts(parsed, false), []);
  });
});

describe('createExplorationStore', () => {
  it('starts idle: the map belongs to the operator', () => {
    const store = createExplorationStore();
    assert.equal(store.isActive(), false);
    assert.equal(store.ownsHud(), false);
    assert.equal(store.escaped(), false);
  });

  it('the HUD belongs to the search while it runs and after it ends', () => {
    const store = createExplorationStore();
    store.apply(status({ state: 'navigating' }));
    assert.equal(store.ownsHud(), true);
    assert.equal(store.label(), 'search: navigating');
    store.apply(status({ state: 'failed', message: 'total deadline exceeded' }));
    assert.equal(store.isActive(), false);
    assert.equal(store.ownsHud(), true);
    assert.ok(store.hudParts().includes('total deadline exceeded'));
  });

  it('a rosbridge reconnection rebuilds the screen from the last message', () => {
    // The topic is TRANSIENT_LOCAL: after the drop, the new subscriber receives
    // the last status. The store cannot have a memory of its own to reconcile
    // -- applying the redelivery has to be enough, and the search on the
    // Aquila is not touched.
    const before = createExplorationStore();
    before.apply(status({ state: 'homing_exit', elapsed_s: 88, message: 'x' }));

    const afterReconnect = createExplorationStore();
    afterReconnect.apply(status({ state: 'homing_exit', elapsed_s: 88, message: 'x' }));

    assert.deepEqual(afterReconnect.snapshot(), before.snapshot());
    assert.deepEqual(afterReconnect.hudParts(), before.hudParts());
    assert.equal(afterReconnect.isActive(), true);
  });

  it('a service refusal shows up without erasing the current state', () => {
    const store = createExplorationStore();
    store.apply(status({ state: 'navigating', frontier_count: 5 }));
    store.merge({ message: 'search already in progress' });
    assert.equal(store.snapshot().state, 'navigating');
    assert.equal(store.snapshot().frontier_count, 5);
    assert.ok(store.hudParts().includes('search already in progress'));
  });

  it('only `true` turns on the confirmed exit', () => {
    const store = createExplorationStore();
    for (const value of [undefined, null, 0, '', 'true']) {
      store.setEscaped(value);
      assert.equal(store.escaped(), false, String(value));
    }
    store.setEscaped(true);
    assert.equal(store.escaped(), true);
  });

  it('the search has no concept of zoom', () => {
    // The framing belongs to the operator. This test guards against someone
    // deciding to "centre on the robot when starting the search": zoom lives in
    // `state` in the panel, and nothing here may reach it.
    const store = createExplorationStore();
    store.apply(status({ state: 'navigating' }));
    const surface = Object.keys(store).join(' ');
    assert.ok(!/zoom|center|centr|view/i.test(surface));
    assert.ok(!/zoom/i.test(JSON.stringify(store.snapshot())));
  });
});

describe('fail-safe between the click and the first status', () => {
  it('the in-flight service promise already blocks the manual goal', () => {
    // Between the click and the service response, the Aquila may already have
    // accepted the search. A click on the map in that window would send a
    // manual goal on top of it.
    const store = createExplorationStore();
    assert.equal(store.isBusy(), false);

    store.beginStart();

    assert.equal(store.isBusy(), true);
    assert.equal(store.snapshot().state, 'starting');
    assert.ok(store.hudParts().includes('starting search'));
  });

  it('`starting` hides the start and shows the cancel', () => {
    // These are the same two questions the panel asks to decide the buttons.
    const store = createExplorationStore();
    store.beginStart();

    assert.equal(store.isBusy(), true);
    assert.equal(store.ownsHud(), true);
  });

  it('a double click does not become two calls', () => {
    // The panel tests `isBusy()` BEFORE any side effect. Simulating the second
    // click is asking exactly that.
    const store = createExplorationStore();
    let calls = 0;
    const click = () => {
      if (store.isBusy()) return;
      store.beginStart();
      calls += 1;
    };

    click();
    click();
    click();

    assert.equal(calls, 1);
  });

  it('the first real status replaces the local `starting`', () => {
    const store = createExplorationStore();
    store.beginStart();

    store.apply(status({ state: 'selecting' }));

    assert.equal(store.snapshot().state, 'selecting');
    assert.equal(store.isBusy(), true);
  });

  it('a refused start gives the cockpit back instead of hanging on `starting`', () => {
    // `starting` is a state only the cockpit invented. If the service refuses
    // and nobody undoes it, the map stays locked with no search on the other
    // side.
    const store = createExplorationStore();
    store.beginStart();

    store.refuseStart('search already in progress');

    assert.equal(store.isBusy(), false);
    assert.ok(store.hudParts().includes('search already in progress'));
  });
});

describe('an unreadable status does not give the map back to the operator', () => {
  it('`navigating` followed by broken JSON stays locked', () => {
    // Converting to `failed` would release the manual goal on top of a search
    // that keeps running on the Aquila. `failed` is terminal, and terminal
    // releases.
    const store = createExplorationStore();
    store.apply(status({ state: 'navigating' }));

    store.apply({ data: '{not json' });

    assert.equal(store.isBusy(), true);
    assert.equal(store.snapshot().state, 'navigating');
  });

  it('`selecting` followed by a disconnection stays locked', () => {
    const store = createExplorationStore();
    store.apply(status({ state: 'selecting' }));

    store.apply(undefined);

    assert.equal(store.isBusy(), true);
    assert.equal(store.snapshot().state, 'selecting');
  });

  it('the communication failure appears next to the last valid state', () => {
    // The operator needs to see BOTH things: what the robot was doing, and that
    // the cockpit stopped knowing.
    const store = createExplorationStore();
    store.apply(status({ state: 'navigating', message: 'navigating to frontier' }));

    store.apply({ data: '[]' });

    const parts = store.hudParts();
    assert.ok(parts.includes('navigating to frontier'));
    assert.ok(parts.includes('search status invalid'));
    assert.equal(store.linkError(), 'search status invalid');
  });

  it('a valid status afterwards clears the communication error', () => {
    const store = createExplorationStore();
    store.apply(status({ state: 'navigating' }));
    store.apply({ data: 'null' });

    store.apply(status({ state: 'selecting' }));

    assert.equal(store.linkError(), null);
    assert.ok(!store.hudParts().includes('search status invalid'));
  });

  it('with no valid state yet, the error is what there is to show', () => {
    // There is nothing to preserve here, and a silent screen is worse than a visible error.
    const store = createExplorationStore();

    store.apply({ data: '{not json' });

    assert.equal(store.snapshot().state, 'failed');
    assert.equal(store.isBusy(), false);
  });

  it('an explicit cancel gives control back to the operator', () => {
    const store = createExplorationStore();
    store.apply(status({ state: 'navigating' }));
    store.beginCancel();
    assert.equal(store.isBusy(), true);

    store.endCommand();
    store.apply(status({ state: 'cancelled', message: 'search cancelled by the operator' }));

    assert.equal(store.isBusy(), false);
    assert.ok(store.hudParts().includes('search cancelled by the operator'));
  });
});
