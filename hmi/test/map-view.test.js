/**
 * World <-> screen mapping.
 *
 * The single y flip is the whole point of these tests. A map drawn with an
 * extra flip still looks like a map, and the mirroring only becomes visible
 * when a click sends the robot to the wrong side of the room.
 */

import test from 'node:test';
import assert from 'node:assert/strict';

import {
  buildCostLut,
  DEFAULT_MAP_ZOOM,
  MAX_MAP_ZOOM,
  MIN_MAP_ZOOM,
  clampMapZoom,
  createView,
  defaultExtent,
  extentOfGrid,
  stepMapZoom,
} from '../js/panels/map-view.js';

const EXTENT = { originX: -5, originY: -5, widthM: 10, heightM: 10 };

const near = (actual, expected, message) =>
  assert.ok(
    Math.abs(actual - expected) < 1e-9,
    `${message}: esperado ${expected}, veio ${actual}`,
  );

test('map north is at the top of the canvas', () => {
  const view = createView(EXTENT, 216, 216, { padding: 8 });
  const top = view.toScreen(0, 5);
  const bottom = view.toScreen(0, -5);
  assert.ok(top.y < bottom.y, 'y maior no mundo deve dar y menor na tela');
});

test('toWorld undoes toScreen exactly', () => {
  const view = createView(EXTENT, 300, 200);
  for (const [x, y] of [[0, 0], [-4.25, 3.5], [4.9, -4.9]]) {
    const screen = view.toScreen(x, y);
    const back = view.toWorld(screen.x, screen.y);
    near(back.x, x, `round-trip x de (${x}, ${y})`);
    near(back.y, y, `round-trip y de (${x}, ${y})`);
  }
});

test('both axes share one scale', () => {
  // A costmap drawn with different x and y scales is subtly wrong in a way that
  // only shows up when the robot turns.
  const view = createView(EXTENT, 400, 200);
  const oneMetreX = view.toScreen(1, 0).x - view.toScreen(0, 0).x;
  const oneMetreY = view.toScreen(0, 0).y - view.toScreen(0, 1).y;
  near(oneMetreX, oneMetreY, 'metro em x vs em y');
  near(oneMetreX, view.scale, 'escala publicada');
});

test('the drawn map is centred in the leftover space', () => {
  // 400x200 for a square extent: the raster is 184 wide (200 - 2*8 padding) and
  // the 200 px of slack are split evenly, not dumped on one side.
  const view = createView(EXTENT, 400, 200, { padding: 8 });
  const rect = view.rasterRect();
  near(rect.width, rect.height, 'raster quadrado para extensão quadrada');
  near(rect.x, (400 - rect.width) / 2, 'sobra horizontal dividida');
  near(rect.y, (200 - rect.height) / 2, 'sobra vertical dividida');
});

test('the navigation default zoom draws the map at half the fitted scale', () => {
  const fitted = createView(EXTENT, 300, 200);
  const overview = createView(EXTENT, 300, 200, { zoom: DEFAULT_MAP_ZOOM });
  near(overview.scale, fitted.scale * 0.5, 'zoom inicial');
  near(overview.rasterRect().width, fitted.rasterRect().width * 0.5, 'largura');
});

test('a custom focus remains at the exact canvas centre at every zoom', () => {
  const robot = { x: -4.2, y: 3.1 };
  for (const zoom of [MIN_MAP_ZOOM, DEFAULT_MAP_ZOOM, 1, MAX_MAP_ZOOM]) {
    const view = createView(EXTENT, 320, 180, { center: robot, zoom });
    const screen = view.toScreen(robot.x, robot.y);
    near(screen.x, 160, `centro x em ${zoom}`);
    near(screen.y, 90, `centro y em ${zoom}`);
    const back = view.toWorld(screen.x, screen.y);
    near(back.x, robot.x, `round-trip x em ${zoom}`);
    near(back.y, robot.y, `round-trip y em ${zoom}`);
  }
});

test('zoom buttons use a 1.25 factor and clamp both ends', () => {
  near(stepMapZoom(DEFAULT_MAP_ZOOM, 'in'), 0.625, 'zoom in');
  near(stepMapZoom(DEFAULT_MAP_ZOOM, 'out'), 0.4, 'zoom out');
  assert.equal(stepMapZoom(MAX_MAP_ZOOM, 'in'), MAX_MAP_ZOOM);
  assert.equal(stepMapZoom(MIN_MAP_ZOOM, 'out'), MIN_MAP_ZOOM);
  assert.equal(clampMapZoom(Number.NaN), DEFAULT_MAP_ZOOM);
});

test('the raster rect and toScreen agree on the corners', () => {
  const view = createView(EXTENT, 320, 240);
  const rect = view.rasterRect();
  const topLeft = view.toScreen(EXTENT.originX, EXTENT.originY + EXTENT.heightM);
  near(topLeft.x, rect.x, 'canto x');
  near(topLeft.y, rect.y, 'canto y');
});

test('a degenerate extent does not produce NaN', () => {
  // An OccupancyGrid with width 0 arrives before Nav2 has sized its costmap.
  const view = createView({ originX: 0, originY: 0, widthM: 0, heightM: 0 }, 100, 100);
  const screen = view.toScreen(0, 0);
  assert.ok(Number.isFinite(screen.x) && Number.isFinite(screen.y));
  assert.ok(Number.isFinite(view.scale) && view.scale > 0);
});

test('extentOfGrid converts cells to metres', () => {
  const extent = extentOfGrid({
    width: 200,
    height: 100,
    resolution: 0.05,
    origin: { position: { x: -3, y: -2 } },
  });
  assert.deepEqual(extent, { originX: -3, originY: -2, widthM: 10, heightM: 5 });
});

test('defaultExtent is centred on the origin', () => {
  assert.deepEqual(defaultExtent(4), {
    originX: -4,
    originY: -4,
    widthM: 8,
    heightM: 8,
  });
});

test('the cost LUT separates unknown from free', () => {
  const lut = buildCostLut();
  const rgba = (value) => [...lut.slice(value * 4, value * 4 + 4)];

  // -1 arrives as 255 through an unsigned view. It MUST NOT look like free
  // space: allow_unknown is true, so the planner routes through it and the
  // operator has to be able to see where it did that.
  assert.notDeepEqual(rgba(255), rgba(0), 'desconhecido vs livre');
  assert.equal(rgba(255)[3], 255, 'desconhecido é opaco');
});

test('the cost LUT is fully opaque across the valid range', () => {
  const lut = buildCostLut();
  // A transparent cell reads as "no data" and there is no such value in an
  // OccupancyGrid.
  for (const value of [0, 1, 50, 98, 99, 100, 255]) {
    assert.ok(lut[value * 4 + 3] > 200, `alpha em ${value}`);
  }
});

test('lethal, inscribed and inflation are three distinct colours', () => {
  const lut = buildCostLut();
  const rgba = (value) => lut.slice(value * 4, value * 4 + 4).join(',');
  const distinct = new Set([rgba(50), rgba(99), rgba(100)]);
  assert.equal(distinct.size, 3);
});

test('out-of-range values are painted as obviously wrong', () => {
  const lut = buildCostLut();
  // 101..254 cannot occur in a well-formed OccupancyGrid. Magenta is a tell
  // that the producer is broken, not a colour anyone would pick on purpose.
  assert.deepEqual([...lut.slice(150 * 4, 150 * 4 + 4)], [255, 0, 255, 255]);
});

test('the LUT covers every byte value', () => {
  assert.equal(buildCostLut().length, 256 * 4);
});
