/**
 * TF composition, in 2D.
 *
 * These tests exist because every bug this file can have is invisible: a wrong
 * composition order, a missed rotation or a dropped edge all still draw a map
 * with a robot on it. The tests pick cases where a sign error moves geometry to
 * a place that is obviously wrong on paper.
 */

import test from 'node:test';
import assert from 'node:assert/strict';

import { applyTransform, compose, TfTree, yawOf } from '../js/ros/tf-tree.js';

const QUARTER = Math.PI / 2;

/** Quaternion for a yaw-only rotation, the only kind this cockpit sees. */
const quatZ = (theta) => ({ x: 0, y: 0, z: Math.sin(theta / 2), w: Math.cos(theta / 2) });

const near = (actual, expected, message) =>
  assert.ok(
    Math.abs(actual - expected) < 1e-9,
    `${message}: expected ${expected}, received ${actual}`,
  );

test('yawOf recovers the angle from a yaw-only quaternion', () => {
  near(yawOf(quatZ(0)), 0, 'zero');
  near(yawOf(quatZ(QUARTER)), QUARTER, 'quarter turn');
  near(yawOf(quatZ(-QUARTER)), -QUARTER, 'negative quarter turn');
});

test('yawOf treats a missing rotation as identity', () => {
  assert.equal(yawOf(null), 0);
  assert.equal(yawOf(undefined), 0);
  // A partial quaternion is what a hand-built test fixture looks like; it must
  // not become NaN and poison every downstream coordinate.
  assert.equal(yawOf({}), 0);
});

test('applyTransform rotates before translating', () => {
  // A point 1 m ahead of a frame that is rotated a quarter turn and sits at
  // (2, 0) lands at (2, 1). Translating first would put it at (3, 0).
  const point = applyTransform({ x: 2, y: 0, theta: QUARTER }, 1, 0);
  near(point.x, 2, 'x');
  near(point.y, 1, 'y');
});

test('compose applies the second argument first', () => {
  // b: quarter turn about the origin. a: 1 m along its own x.
  // b∘a means "do a, then b" -> the metre ends up along +y.
  const result = compose({ x: 0, y: 0, theta: QUARTER }, { x: 1, y: 0, theta: 0 });
  near(result.x, 0, 'x');
  near(result.y, 1, 'y');
  near(result.theta, QUARTER, 'theta');
});

/** A tf2_msgs/TFMessage as rosbridge delivers it. */
const tfMessage = (...edges) => ({
  transforms: edges.map(([parent, child, x, y, theta = 0]) => ({
    header: { frame_id: parent },
    child_frame_id: child,
    transform: { translation: { x, y, z: 0 }, rotation: quatZ(theta) },
  })),
});

test('lookup walks the chain from source up to target', () => {
  const tree = new TfTree();
  tree.update(tfMessage(['map', 'odom', 1, 0], ['odom', 'base', 0, 2]));

  const transform = tree.lookup('map', 'base');
  near(transform.x, 1, 'x');
  near(transform.y, 2, 'y');
});

test('lookup composes rotations along the chain', () => {
  const tree = new TfTree();
  // odom is a quarter turn from map; base sits 1 m along odom's x, which is
  // map's +y.
  tree.update(tfMessage(['map', 'odom', 0, 0, QUARTER], ['odom', 'base', 1, 0]));

  const transform = tree.lookup('map', 'base');
  near(transform.x, 0, 'x');
  near(transform.y, 1, 'y');
  near(transform.theta, QUARTER, 'theta');
});

test('lookup returns null when the chain is incomplete', () => {
  const tree = new TfTree();
  tree.update(tfMessage(['odom', 'base', 0, 0]));

  // No map->odom edge yet. Returning identity here would draw the whole robot
  // at the map origin, which reads as a robot that never moved.
  assert.equal(tree.lookup('map', 'base'), null);
});

test('lookup of a frame onto itself is identity', () => {
  const tree = new TfTree();
  assert.deepEqual(tree.lookup('map', 'map'), { x: 0, y: 0, theta: 0 });
});

test('update accumulates instead of replacing', () => {
  const tree = new TfTree();
  // The real failure this guards: /tf_static has two latched publishers here,
  // so each message carries only part of the tree. Replacing on every message
  // drops whichever half arrived first.
  tree.update(tfMessage(['map', 'odom', 1, 0]));
  tree.update(tfMessage(['base', 'lidar', 0, 0.5]));
  tree.update(tfMessage(['odom', 'base', 0, 2]));

  const transform = tree.lookup('map', 'lidar');
  near(transform.x, 1, 'x');
  near(transform.y, 2.5, 'y');
});

test('update overwrites an edge with the newer sample', () => {
  const tree = new TfTree();
  tree.update(tfMessage(['odom', 'base', 0, 0]));
  tree.update(tfMessage(['odom', 'base', 3, 0]));

  near(tree.lookup('odom', 'base').x, 3, 'x');
});

test('leading slashes do not split a frame in two', () => {
  const tree = new TfTree();
  tree.update(tfMessage(['/map', '/odom', 1, 0], ['odom', 'base', 0, 1]));

  const transform = tree.lookup('map', 'base');
  near(transform.x, 1, 'x');
  near(transform.y, 1, 'y');
});

test('a cyclic tree terminates instead of hanging', () => {
  const tree = new TfTree();
  tree.update(tfMessage(['b', 'a', 1, 0], ['a', 'b', 1, 0]));

  // Nothing in a browser recovers from an infinite loop in a repaint tick.
  assert.equal(tree.lookup('map', 'a'), null);
});

test('an edge with no parent is ignored rather than stored broken', () => {
  const tree = new TfTree();
  tree.update({ transforms: [{ child_frame_id: 'base', transform: {} }] });
  assert.equal(tree.has('base'), false);
});

test('update tolerates a message with no transforms', () => {
  const tree = new TfTree();
  tree.update(null);
  tree.update({});
  assert.deepEqual(tree.frames(), []);
});
