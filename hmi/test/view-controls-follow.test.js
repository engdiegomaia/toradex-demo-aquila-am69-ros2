/**
 * The `follow robot` button: who writes its state.
 *
 * The regression this file exists to catch is a one-liner: painting
 * `aria-pressed` from the click itself instead of from the topic the node
 * publishes. It passes any "the button calls the service" test and lies
 * exactly in the cases that matter — reloaded page, two open cockpits, or
 * someone who turned following off from the command line.
 */

import assert from 'node:assert/strict';
import { describe, it } from 'node:test';

import {
  FOLLOW_DEFAULT,
  FOLLOW_VIEW_SERVICE,
  FOLLOWING_TOPIC,
  createViewControls,
} from '../js/panels/view-controls.js';
import { fakeClient, fakeRoot, installWindowTimers } from './helpers/fake-dom.js';

installWindowTimers();

const PAD = '[data-role="view-pad"]';
const FOLLOW = '[data-role="view-follow"]';

function mount(options = {}) {
  const root = fakeRoot([PAD, FOLLOW, '[data-role="view-reset"]']);
  const client = fakeClient(options);
  const controls = createViewControls({ root, client, onNotice: () => {} });
  return { root, client, controls, button: root.get(FOLLOW) };
}

describe('createViewControls: follow the robot', () => {
  it('starts pressed, same as the node default', () => {
    // Diverging from this makes the first click send the value that already
    // holds: nothing happens on screen and the button looks broken.
    const { button, controls } = mount();
    assert.equal(button.getAttribute('aria-pressed'), String(FOLLOW_DEFAULT));
    assert.equal(controls.isFollowing(), FOLLOW_DEFAULT);
  });

  it('subscribes to the state topic published by the node', () => {
    const { client } = mount();
    const topics = client.subscriptions.map((entry) => entry.topic);
    assert.ok(topics.includes(FOLLOWING_TOPIC));
  });

  it('the click asks through the service, with the inverted value', async () => {
    const { client, button } = mount();
    await Promise.all(button.emit('click'));

    assert.deepEqual(client.calls, [
      { service: FOLLOW_VIEW_SERVICE, args: { data: !FOLLOW_DEFAULT } },
    ]);
  });

  it('the click does NOT paint the button — only the topic paints', async () => {
    const { client, button, controls } = mount();

    await Promise.all(button.emit('click'));
    assert.equal(
      button.getAttribute('aria-pressed'),
      String(FOLLOW_DEFAULT),
      'the button changed before the node confirmed',
    );

    client.deliver(FOLLOWING_TOPIC, { data: false });
    assert.equal(button.getAttribute('aria-pressed'), 'false');
    assert.equal(controls.isFollowing(), false);
  });

  it('follows the node even when nobody clicked on this screen', () => {
    // The case of the second open cockpit, and of `ros2 service call` on the bench.
    const { client, button } = mount();
    client.deliver(FOLLOWING_TOPIC, { data: false });
    assert.equal(button.getAttribute('aria-pressed'), 'false');
  });

  it('a reply without `data` reads as off, not as undefined', () => {
    // std_msgs/Bool with false comes from rosbridge as `{data: false}`, but a
    // missing field must not become the string "undefined" in an ARIA
    // attribute.
    const { client, button } = mount();
    client.deliver(FOLLOWING_TOPIC, {});
    assert.equal(button.getAttribute('aria-pressed'), 'false');
  });

  it('destroy cuts the state subscription', () => {
    const { client, controls } = mount();
    controls.destroy();
    const following = client.subscriptions.find(
      (entry) => entry.topic === FOLLOWING_TOPIC,
    );
    assert.equal(following.active, false);
  });
});
