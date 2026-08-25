/**
 * Connection-state, message-parsing, action and PNG tests for the client.
 *
 * AGENTS.md §5.7 asks for the first two by name: "add tests for message parsing
 * and connection-state behavior". Run with:
 *
 *   npm test --prefix hmi          # or: node --test "hmi/test/**\/*.test.js"
 *
 * The quoted glob is not decoration: node 24 treats a bare directory as a glob
 * pattern and resolves `hmi/test/` to nothing, which exits 0 having run no
 * tests at all.
 *
 * No test runner is installed and none is needed — node's built-in runner keeps
 * the "no npm, no build step" rule (plano-cockpit-web.md Decisão 5) intact.
 */

import assert from 'node:assert/strict';
import { describe, it } from 'node:test';

import {
  ConnectionState,
  RETRY_DELAYS_MS,
  RosbridgeClient,
} from '../js/ros/rosbridge-client.js';
import { manualTimers, trackingFactory } from './helpers/fake-socket.js';

function makeClient(overrides = {}) {
  const sockets = trackingFactory();
  const timers = manualTimers();
  const errors = [];
  const client = new RosbridgeClient({
    url: 'ws://test:9090',
    socketFactory: sockets.factory,
    setTimeoutFn: timers.setTimeoutFn,
    clearTimeoutFn: timers.clearTimeoutFn,
    onError: (message) => errors.push(message),
    ...overrides,
  });
  return { client, sockets, timers, errors };
}

describe('connection state', () => {
  it('starts disconnected and reports connecting before the socket opens', () => {
    const { client, sockets } = makeClient();
    const seen = [];
    client.onStateChange((state) => seen.push(state));

    assert.deepEqual(seen, [ConnectionState.DISCONNECTED]);

    client.connect();
    assert.equal(client.state, ConnectionState.CONNECTING);

    sockets.latest.open();
    assert.equal(client.state, ConnectionState.CONNECTED);
    assert.deepEqual(seen, [
      ConnectionState.DISCONNECTED,
      ConnectionState.CONNECTING,
      ConnectionState.CONNECTED,
    ]);
  });

  it('reconnects after an unexpected drop, with backoff', () => {
    const { client, sockets, timers } = makeClient();
    client.connect();
    sockets.latest.open();

    sockets.latest.drop();
    assert.equal(client.state, ConnectionState.DISCONNECTED);
    assert.deepEqual(timers.delays(), [RETRY_DELAYS_MS[0]]);

    timers.runAll();
    assert.equal(sockets.sockets.length, 2);
    assert.equal(client.state, ConnectionState.CONNECTING);
  });

  it('backs off further on each consecutive failure, then caps', () => {
    const { client, sockets, timers } = makeClient();
    client.connect();

    const observed = [];
    for (let attempt = 0; attempt < RETRY_DELAYS_MS.length + 2; attempt += 1) {
      sockets.latest.drop();
      observed.push(timers.delays()[0]);
      timers.runAll();
    }

    assert.deepEqual(observed.slice(0, RETRY_DELAYS_MS.length), [
      ...RETRY_DELAYS_MS,
    ]);
    const last = RETRY_DELAYS_MS.at(-1);
    assert.deepEqual(observed.slice(RETRY_DELAYS_MS.length), [last, last]);
  });

  it('resets the backoff once a connection succeeds', () => {
    const { client, sockets, timers } = makeClient();
    client.connect();

    sockets.latest.drop();
    timers.runAll();
    sockets.latest.drop();
    timers.runAll();
    sockets.latest.open();

    sockets.latest.drop();
    assert.deepEqual(timers.delays(), [RETRY_DELAYS_MS[0]]);
  });

  it('does not reconnect after a deliberate close', () => {
    const { client, sockets, timers } = makeClient();
    client.connect();
    sockets.latest.open();

    client.close();

    assert.equal(client.state, ConnectionState.DISCONNECTED);
    assert.equal(timers.size, 0);
    assert.equal(sockets.sockets.length, 1);
  });
});

describe('subscription lifecycle', () => {
  it('replays subscriptions and advertisements on reconnect', () => {
    const { client, sockets, timers } = makeClient();
    client.connect();
    sockets.latest.open();

    client.subscribe('/demo/odom', 'nav_msgs/msg/Odometry', () => {});
    client.advertise('/demo/cmd_vel', 'geometry_msgs/msg/Twist');
    assert.equal(sockets.latest.opsSent('subscribe').length, 1);

    sockets.latest.drop();
    timers.runAll();
    sockets.latest.open();

    // The whole point: rosbridge keeps no state across connections, so a
    // reconnect with no replay looks healthy and delivers nothing.
    assert.deepEqual(
      sockets.latest.opsSent('subscribe').map((frame) => frame.topic),
      ['/demo/odom'],
    );
    assert.deepEqual(
      sockets.latest.opsSent('advertise').map((frame) => frame.topic),
      ['/demo/cmd_vel'],
    );
  });

  it('queues nothing while down: a subscribe made offline is sent on connect', () => {
    const { client, sockets } = makeClient();
    client.subscribe('/demo/scan', 'sensor_msgs/msg/LaserScan', () => {});
    client.connect();
    sockets.latest.open();

    assert.deepEqual(
      sockets.latest.opsSent('subscribe').map((frame) => frame.topic),
      ['/demo/scan'],
    );
  });

  it('uses depth 1 by default and honours a queue length override', () => {
    const { client, sockets } = makeClient();
    client.connect();
    sockets.latest.open();

    client.subscribe('/demo/odom', 'nav_msgs/msg/Odometry', () => {});
    client.subscribe('/rosout', 'rcl_interfaces/msg/Log', () => {}, {
      queueLength: 100,
    });

    const byTopic = Object.fromEntries(
      sockets.latest.opsSent('subscribe').map((f) => [f.topic, f.queue_length]),
    );
    assert.equal(byTopic['/demo/odom'], 1);
    assert.equal(byTopic['/rosout'], 100);
  });

  it('unsubscribes only when the last handler goes away', () => {
    const { client, sockets } = makeClient();
    client.connect();
    sockets.latest.open();

    const offA = client.subscribe('/demo/odom', 'nav_msgs/msg/Odometry', () => {});
    const offB = client.subscribe('/demo/odom', 'nav_msgs/msg/Odometry', () => {});

    offA();
    assert.equal(sockets.latest.opsSent('unsubscribe').length, 0);
    offB();
    assert.equal(sockets.latest.opsSent('unsubscribe').length, 1);
  });
});

describe('message parsing', () => {
  it('routes a publish frame to the handlers of its topic only', () => {
    const { client, sockets } = makeClient();
    client.connect();
    sockets.latest.open();

    const odom = [];
    const scan = [];
    client.subscribe('/demo/odom', 'nav_msgs/msg/Odometry', (m) => odom.push(m));
    client.subscribe('/demo/scan', 'sensor_msgs/msg/LaserScan', (m) => scan.push(m));

    sockets.latest.deliver({
      op: 'publish',
      topic: '/demo/odom',
      msg: { pose: { pose: { position: { x: 1.5 } } } },
    });

    assert.equal(odom.length, 1);
    assert.equal(odom[0].pose.pose.position.x, 1.5);
    assert.equal(scan.length, 0);
  });

  it('ignores a publish for a topic nobody subscribed to', () => {
    const { client, sockets, errors } = makeClient();
    client.connect();
    sockets.latest.open();

    sockets.latest.deliver({ op: 'publish', topic: '/unknown', msg: {} });
    assert.deepEqual(errors, []);
  });

  it('survives a malformed frame and keeps delivering afterwards', () => {
    const { client, sockets, errors } = makeClient();
    client.connect();
    sockets.latest.open();

    const received = [];
    client.subscribe('/demo/odom', 'nav_msgs/msg/Odometry', (m) => received.push(m));

    sockets.latest.deliverRaw('{not json');
    sockets.latest.deliver({ op: 'publish', topic: '/demo/odom', msg: { ok: true } });

    assert.equal(errors.length, 1);
    assert.match(errors[0], /malformed/);
    assert.deepEqual(received, [{ ok: true }]);
  });

  it('a throwing handler does not stop the other panels', () => {
    const { client, sockets, errors } = makeClient();
    client.connect();
    sockets.latest.open();

    const good = [];
    client.subscribe('/demo/odom', 'nav_msgs/msg/Odometry', () => {
      throw new Error('panel bug');
    });
    client.subscribe('/demo/odom', 'nav_msgs/msg/Odometry', (m) => good.push(m));

    sockets.latest.deliver({ op: 'publish', topic: '/demo/odom', msg: { n: 1 } });

    assert.deepEqual(good, [{ n: 1 }]);
    assert.equal(errors.length, 1);
  });

  it('reports rosbridge error status frames', () => {
    const { client, sockets, errors } = makeClient();
    client.connect();
    sockets.latest.open();

    sockets.latest.deliver({ op: 'status', level: 'error', msg: 'no such topic' });
    assert.match(errors[0], /no such topic/);
  });
});

describe('publish and service calls', () => {
  it('refuses to publish while disconnected instead of buffering', () => {
    const { client, sockets } = makeClient();
    client.connect();
    sockets.latest.open();
    client.advertise('/demo/cmd_vel', 'geometry_msgs/msg/Twist');

    sockets.latest.drop();
    const accepted = client.publish('/demo/cmd_vel', { linear: { x: 0.25 } });

    // A buffered velocity command replayed on reconnect drives a robot the
    // operator believes is stopped. Dropping it is the safe failure.
    assert.equal(accepted, false);
  });

  it('refuses to publish to a topic that was never advertised', () => {
    const { client, sockets, errors } = makeClient();
    client.connect();
    sockets.latest.open();

    assert.equal(client.publish('/demo/cmd_vel', {}), false);
    assert.match(errors[0], /un-advertised/);
  });

  it('resolves a service call with its response values', async () => {
    const { client, sockets } = makeClient();
    client.connect();
    sockets.latest.open();

    const pending = client.callService('/rosapi/topics');
    const request = sockets.latest.opsSent('call_service')[0];
    sockets.latest.deliver({
      op: 'service_response',
      id: request.id,
      result: true,
      values: { topics: ['/demo/odom'] },
    });

    assert.deepEqual(await pending, { topics: ['/demo/odom'] });
  });

  it('rejects in-flight service calls when the link drops', async () => {
    const { client, sockets } = makeClient();
    client.connect();
    sockets.latest.open();

    const pending = client.callService('/rosapi/topics');
    sockets.latest.drop();

    await assert.rejects(pending, /closed/);
  });
});

describe('action goals', () => {
  const GOAL = { pose: { header: { frame_id: 'map' } } };

  const sendGoal = (client, options) =>
    client.sendActionGoal('/navigate_to_pose', 'nav2_msgs/action/NavigateToPose', GOAL, options);

  it('sends the goal with the action type and the feedback flag', () => {
    const { client, sockets } = makeClient();
    client.connect();
    sockets.latest.open();

    sendGoal(client, { onFeedback: () => {} });

    const [frame] = sockets.latest.opsSent('send_action_goal');
    assert.equal(frame.action, '/navigate_to_pose');
    assert.equal(frame.action_type, 'nav2_msgs/action/NavigateToPose');
    assert.deepEqual(frame.args, GOAL);
    // Asking for feedback we do not consume costs ~100 frames per second of
    // full poses, measured against Nav2.
    assert.equal(frame.feedback, true);
  });

  it('does not ask for feedback when no handler was given', () => {
    const { client, sockets } = makeClient();
    client.connect();
    sockets.latest.open();

    sendGoal(client);
    assert.equal(sockets.latest.opsSent('send_action_goal')[0].feedback, false);
  });

  it('resolves with succeeded when the result arrives', async () => {
    const { client, sockets } = makeClient();
    client.connect();
    sockets.latest.open();

    const handle = sendGoal(client);
    sockets.latest.deliver({
      op: 'action_result',
      id: handle.id,
      result: true,
      status: 4,
      values: { result: {} },
    });

    const outcome = await handle.result;
    assert.equal(outcome.succeeded, true);
    assert.equal(outcome.status, 4);
  });

  it('treats a refused goal as a normal outcome, not an error', async () => {
    const { client, sockets } = makeClient();
    client.connect();
    sockets.latest.open();

    const handle = sendGoal(client);
    sockets.latest.deliver({ op: 'action_result', id: handle.id, result: false, status: 6 });

    // A rejected or aborted goal has to reach the screen. Throwing it away
    // leaves the HUD saying "navegando" over a robot that stopped.
    const outcome = await handle.result;
    assert.equal(outcome.succeeded, false);
    assert.equal(outcome.status, 6);
    assert.ok(!outcome.lost, 'aborto não é perda de link');
  });

  it('routes feedback to the goal that asked for it', () => {
    const { client, sockets } = makeClient();
    client.connect();
    sockets.latest.open();

    const seen = [];
    const handle = sendGoal(client, { onFeedback: (values) => seen.push(values) });
    sockets.latest.deliver({
      op: 'action_feedback',
      id: handle.id,
      values: { distance_remaining: 3.5 },
    });
    sockets.latest.deliver({ op: 'action_feedback', id: 'other', values: { x: 1 } });

    assert.deepEqual(seen, [{ distance_remaining: 3.5 }]);
  });

  it('survives a feedback handler that throws', () => {
    const { client, sockets, errors } = makeClient();
    client.connect();
    sockets.latest.open();

    const handle = sendGoal(client, {
      onFeedback: () => {
        throw new Error('boom');
      },
    });
    sockets.latest.deliver({ op: 'action_feedback', id: handle.id, values: {} });

    assert.equal(errors.length, 1);
    assert.equal(client.activeGoalIds().length, 1, 'a meta continua viva');
  });

  it('cancel sends cancel_action_goal naming the action', () => {
    const { client, sockets } = makeClient();
    client.connect();
    sockets.latest.open();

    const handle = sendGoal(client);
    handle.cancel();

    const [frame] = sockets.latest.opsSent('cancel_action_goal');
    assert.equal(frame.id, handle.id);
    assert.equal(frame.action, '/navigate_to_pose');
  });

  it('forgets a goal once its result has arrived', async () => {
    const { client, sockets } = makeClient();
    client.connect();
    sockets.latest.open();

    const handle = sendGoal(client);
    sockets.latest.deliver({ op: 'action_result', id: handle.id, result: true });
    await handle.result;

    assert.deepEqual(client.activeGoalIds(), []);
    // Cancelling a finished goal must not put a frame on the wire; rosbridge
    // has no handle for it any more.
    assert.equal(client.cancelActionGoal(handle.id), false);
  });

  it('reports notSent when the link is down', async () => {
    const { client } = makeClient();

    const handle = sendGoal(client);
    const outcome = await handle.result;

    assert.equal(outcome.notSent, true);
    assert.equal(outcome.succeeded, false);
    assert.deepEqual(client.activeGoalIds(), []);
  });

  it('reports a goal as lost, not failed, when the socket dies', async () => {
    const { client, sockets } = makeClient();
    client.connect();
    sockets.latest.open();

    const handle = sendGoal(client);
    sockets.latest.drop();

    // The distinction matters operationally: the goal is very probably still
    // executing on the robot and can no longer be cancelled from here.
    const outcome = await handle.result;
    assert.equal(outcome.lost, true);
    assert.deepEqual(client.activeGoalIds(), []);
  });

  it('does not replay goals on reconnect', async () => {
    const { client, sockets, timers } = makeClient();
    client.connect();
    sockets.latest.open();

    const handle = sendGoal(client);
    sockets.latest.drop();
    await handle.result;
    timers.runAll();
    sockets.latest.open();

    // Subscriptions are replayed; goals are NOT. Re-sending a navigation goal
    // the operator has not re-issued would drive the robot on its own.
    assert.equal(sockets.latest.opsSent('send_action_goal').length, 0);
  });
});

describe('png compression', () => {
  const pngFrame = (message) => ({ op: 'png', data: JSON.stringify(message) });

  /** Decoder double: the data field carries the JSON directly. */
  const decoders = () => {
    const pending = [];
    const decodePng = (data) =>
      new Promise((resolve) => pending.push(() => resolve(JSON.parse(data))));
    return { decodePng, pending };
  };

  it('dispatches the message a png frame carries', async () => {
    const { client, sockets } = makeClient({
      decodePng: async (data) => JSON.parse(data),
    });
    client.connect();
    sockets.latest.open();

    const seen = [];
    client.subscribe('/costmap', 'nav_msgs/msg/OccupancyGrid', (msg) => seen.push(msg));
    sockets.latest.deliver(pngFrame({ op: 'publish', topic: '/costmap', msg: { data: [1] } }));

    await new Promise((resolve) => setImmediate(resolve));
    assert.deepEqual(seen, [{ data: [1] }]);
  });

  it('drops a frame that decoded out of order', async () => {
    const { decodePng, pending } = decoders();
    const { client, sockets } = makeClient({ decodePng });
    client.connect();
    sockets.latest.open();

    const seen = [];
    client.subscribe('/costmap', 'nav_msgs/msg/OccupancyGrid', (msg) => seen.push(msg.n));
    sockets.latest.deliver(pngFrame({ op: 'publish', topic: '/costmap', msg: { n: 1 } }));
    sockets.latest.deliver(pngFrame({ op: 'publish', topic: '/costmap', msg: { n: 2 } }));

    // Decoding is asynchronous, so two frames can finish in the wrong order.
    // The newer map must not be replaced by the older one — a costmap that
    // goes backwards looks exactly like Nav2 forgetting an obstacle.
    pending[1]();
    await new Promise((resolve) => setImmediate(resolve));
    pending[0]();
    await new Promise((resolve) => setImmediate(resolve));

    assert.deepEqual(seen, [2]);
  });

  it('orders frames per topic, not globally', async () => {
    const { decodePng, pending } = decoders();
    const { client, sockets } = makeClient({ decodePng });
    client.connect();
    sockets.latest.open();

    const seen = [];
    client.subscribe('/a', 'std_msgs/msg/String', (msg) => seen.push(`a${msg.n}`));
    client.subscribe('/b', 'std_msgs/msg/String', (msg) => seen.push(`b${msg.n}`));
    sockets.latest.deliver(pngFrame({ op: 'publish', topic: '/a', msg: { n: 1 } }));
    sockets.latest.deliver(pngFrame({ op: 'publish', topic: '/b', msg: { n: 1 } }));

    pending[1]();
    await new Promise((resolve) => setImmediate(resolve));
    pending[0]();
    await new Promise((resolve) => setImmediate(resolve));

    // A global sequence would discard /a here purely because /b decoded first.
    assert.deepEqual(seen.sort(), ['a1', 'b1']);
  });

  it('reports a png frame that arrives with no decoder configured', () => {
    const { client, sockets, errors } = makeClient();
    client.connect();
    sockets.latest.open();

    sockets.latest.deliver(pngFrame({ op: 'publish', topic: '/costmap', msg: {} }));

    // Silence here is the worst outcome: the panel would sit blank while the
    // subscription looks healthy on both ends.
    assert.equal(errors.length, 1);
    assert.match(errors[0], /png/);
  });

  it('reports a decode failure instead of dropping it', async () => {
    const { client, sockets, errors } = makeClient({
      decodePng: async () => {
        throw new Error('corrupt');
      },
    });
    client.connect();
    sockets.latest.open();

    sockets.latest.deliver(pngFrame({ op: 'publish', topic: '/costmap', msg: {} }));
    await new Promise((resolve) => setImmediate(resolve));

    assert.equal(errors.length, 1);
  });
});
