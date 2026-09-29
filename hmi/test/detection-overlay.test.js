/**
 * Detection boxes over a letterboxed MJPEG stream.
 *
 * The geometry is the only part worth testing and it is the only part that can
 * be wrong invisibly: boxes computed against the ELEMENT instead of against the
 * displayed picture land almost right, which reads as a perception error rather
 * than as a layout bug.
 */

import test from 'node:test';
import assert from 'node:assert/strict';

import {
  boxOf,
  containRect,
  labelOf,
} from '../js/panels/detection-overlay.js';

const near = (actual, expected, message) =>
  assert.ok(
    Math.abs(actual - expected) < 1e-9,
    `${message}: expected ${expected}, received ${actual}`,
  );

test('a wider element letterboxes with bars on the sides', () => {
  // 640x480 inside 800x480: scale 1, 160 px of slack split evenly.
  const fit = containRect(800, 480, 640, 480);
  near(fit.scale, 1, 'escala');
  near(fit.x, 80, 'barra esquerda');
  near(fit.y, 0, 'no bar on top');
  near(fit.width, 640, 'rendered width');
});

test('a taller element letterboxes with bars above and below', () => {
  const fit = containRect(640, 600, 640, 480);
  near(fit.scale, 1, 'escala');
  near(fit.x, 0, 'no side bar');
  near(fit.y, 60, 'barra superior');
});

test('the scale is the smaller of the two ratios', () => {
  // contain never crops: the limiting axis wins.
  const fit = containRect(320, 480, 640, 480);
  near(fit.scale, 0.5, 'escala limitada por x');
  near(fit.height, 240, 'rendered height');
});

test('an unknown image size falls back to the element without scaling', () => {
  // camera_info has not arrived and naturalWidth is still 0. Scaling by zero
  // would collapse every box onto one pixel in the corner.
  const fit = containRect(800, 600, 0, 0);
  assert.deepEqual(fit, { x: 0, y: 0, width: 800, height: 600, scale: 1 });
});

test('boxOf centres the rect on the detection centre', () => {
  // The measured shape of the stub on 24/08/2026.
  const box = boxOf({
    bbox: { center: { position: { x: 167.2, y: 240 } }, size_x: 120, size_y: 160 },
  });
  near(box.x, 107.2, 'x');
  near(box.y, 160, 'y');
  near(box.width, 120, 'width');
  near(box.height, 160, 'height');
});

test('boxOf accepts a centre without the position wrapper', () => {
  // vision_msgs changed shape between distros; both spellings show up in
  // recorded bags.
  const box = boxOf({ bbox: { center: { x: 10, y: 20 }, size_x: 4, size_y: 6 } });
  near(box.x, 8, 'x');
  near(box.y, 17, 'y');
});

test('boxOf returns null for a detection with no usable centre', () => {
  // Drawing NaN silently paints nothing; returning null lets the caller skip.
  assert.equal(boxOf({}), null);
  assert.equal(boxOf({ bbox: {} }), null);
  assert.equal(boxOf({ bbox: { center: { position: { x: 'x', y: 1 } } } }), null);
});

test('labelOf shows the class and the score as a percentage', () => {
  assert.equal(
    labelOf({ results: [{ hypothesis: { class_id: 'box', score: 0.87 } }] }),
    'box 87%',
  );
});

test('labelOf falls back to the detection id with no hypothesis', () => {
  assert.equal(labelOf({ id: 'det-3' }), 'det-3');
  assert.equal(labelOf({}), '?');
});

test('labelOf omits a missing score instead of printing NaN%', () => {
  assert.equal(labelOf({ results: [{ hypothesis: { class_id: 'box' } }] }), 'box');
});
