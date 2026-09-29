/**
 * World <-> screen mapping for the navigation panel.
 *
 * Pure on purpose: this is the part that is wrong in every hand-rolled map
 * viewer, it is wrong silently (everything draws, in the wrong place), and it
 * is the only part that can be tested without a canvas.
 *
 * Two conventions meet here and they disagree about which way is up:
 *
 *   ROS/map — x right, y UP, origin at the grid's `origin` corner, metres;
 *   canvas  — x right, y DOWN, origin top-left, pixels.
 *
 * So the y axis is flipped exactly once, in toScreen, and undone exactly once,
 * in toWorld. Anything that flips it a second time somewhere else produces a
 * map that looks plausible and mirrors every click.
 */

/**
 * Build a view that fits `extent` (metres, in map coordinates) into a canvas of
 * `width` x `height` CSS pixels, preserving aspect ratio and centring the
 * leftover.
 *
 * @param {{originX:number, originY:number, widthM:number, heightM:number}} extent
 */
export const DEFAULT_MAP_ZOOM = 0.5;
export const MIN_MAP_ZOOM = 0.125;
export const MAX_MAP_ZOOM = 4;
export const MAP_ZOOM_STEP = 1.25;

export function clampMapZoom(value) {
  if (!Number.isFinite(value)) return DEFAULT_MAP_ZOOM;
  return Math.min(MAX_MAP_ZOOM, Math.max(MIN_MAP_ZOOM, value));
}

export function stepMapZoom(value, direction) {
  const factor = direction === 'in' ? MAP_ZOOM_STEP : 1 / MAP_ZOOM_STEP;
  return clampMapZoom(value * factor);
}

export function createView(
  extent,
  width,
  height,
  { padding = 8, zoom = 1, center = null } = {},
) {
  const usableW = Math.max(1, width - padding * 2);
  const usableH = Math.max(1, height - padding * 2);
  const spanX = extent.widthM > 0 ? extent.widthM : 1;
  const spanY = extent.heightM > 0 ? extent.heightM : 1;

  // One scale for both axes: a costmap drawn with different x and y scales is
  // subtly wrong in a way that only shows up when the robot turns.
  const scale = Math.min(usableW / spanX, usableH / spanY)
    * clampMapZoom(zoom);

  const drawnW = spanX * scale;
  const drawnH = spanY * scale;
  const focus = center ?? {
    x: extent.originX + spanX / 2,
    y: extent.originY + spanY / 2,
  };
  // Keep `focus` at the exact canvas centre. The raster is allowed to extend
  // beyond or occupy only part of the canvas: it is world data, not the camera.
  const offsetX = width / 2 - (focus.x - extent.originX) * scale;
  const offsetY = height / 2 - drawnH + (focus.y - extent.originY) * scale;

  return Object.freeze({
    scale,
    extent,
    width,
    height,

    /** metres in map frame -> CSS pixels on the canvas */
    toScreen(x, y) {
      return {
        x: offsetX + (x - extent.originX) * scale,
        // The single y flip. drawnH - (...) puts map-north at the top.
        y: offsetY + drawnH - (y - extent.originY) * scale,
      };
    },

    /** CSS pixels on the canvas -> metres in map frame */
    toWorld(px, py) {
      return {
        x: extent.originX + (px - offsetX) / scale,
        y: extent.originY + (drawnH - (py - offsetY)) / scale,
      };
    },

    /** Pixel rect the raster occupies, for drawImage. */
    rasterRect() {
      return { x: offsetX, y: offsetY, width: drawnW, height: drawnH };
    },
  });
}

/** Pure: the metric extent of a nav_msgs/OccupancyGrid. */
export function extentOfGrid(info) {
  return {
    originX: info.origin.position.x,
    originY: info.origin.position.y,
    widthM: info.width * info.resolution,
    heightM: info.height * info.resolution,
  };
}

/**
 * Fallback extent used before the first costmap arrives.
 *
 * Centred on the origin rather than on the robot: the robot pose also comes
 * over TF and may not be there yet either, and a view that jumps once when the
 * costmap lands is better than one that jumps twice.
 */
export function defaultExtent(halfSpanM = 10) {
  return {
    originX: -halfSpanM,
    originY: -halfSpanM,
    widthM: halfSpanM * 2,
    heightM: halfSpanM * 2,
  };
}

/**
 * Colour lookup for Nav2 costmap cell values, as 4-byte RGBA rows.
 *
 * The Nav2 scale is not linear in meaning, so it is not drawn linearly:
 *
 *   -1        unknown          — must be visibly different from free, because
 *                               "allow_unknown: true" means the planner will
 *                               route through it (nav_quadruped.launch.py)
 *    0        free
 *    1..98    inflation        — cost gradient around obstacles
 *    99       inscribed        — the robot's centre cannot go here
 *    100      lethal           — obstacle
 *
 * Returns a Uint8ClampedArray of 256*4 indexed by (value & 0xff), so the
 * caller can map an int8 array without branching per pixel.
 */
export function buildCostLut() {
  const lut = new Uint8ClampedArray(256 * 4);
  const put = (index, r, g, b, a) => {
    lut[index * 4] = r;
    lut[index * 4 + 1] = g;
    lut[index * 4 + 2] = b;
    lut[index * 4 + 3] = a;
  };

  // Light palette, for the white background of the Toradex identity. The
  // reading order is the same as in the dark version and is still the one that
  // matters: free must be the LIGHTEST surface, unknown must be visibly
  // different from free, and the inflation gradient must darken monotonically
  // towards the obstacle — so the map reads like relief even in greyscale.
  //
  // int8 -1 arrives as 255 when read through an unsigned view.
  put(255, 214, 221, 228, 255); // unknown
  put(0, 250, 252, 253, 255); // free

  for (let value = 1; value <= 98; value += 1) {
    const t = value / 98;
    // Light blue -> brand orange. It darkens and warms at the same time, so it
    // survives a badly calibrated monitor and a black-and-white photo.
    put(value, 176 + 79 * t, 202 - 68 * t, 226 - 194 * t, 240);
  }
  put(99, 255, 90, 0, 255); // inscribed — orange #ff5a00
  put(100, 176, 34, 26, 255); // lethal

  // Values above 100 are not valid in an OccupancyGrid, but a malformed
  // producer should paint something obviously wrong rather than transparent.
  for (let value = 101; value <= 254; value += 1) put(value, 255, 0, 255, 255);

  return lut;
}
