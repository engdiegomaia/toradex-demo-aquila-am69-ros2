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
export function createView(extent, width, height, { padding = 8 } = {}) {
  const usableW = Math.max(1, width - padding * 2);
  const usableH = Math.max(1, height - padding * 2);
  const spanX = extent.widthM > 0 ? extent.widthM : 1;
  const spanY = extent.heightM > 0 ? extent.heightM : 1;

  // One scale for both axes: a costmap drawn with different x and y scales is
  // subtly wrong in a way that only shows up when the robot turns.
  const scale = Math.min(usableW / spanX, usableH / spanY);

  const drawnW = spanX * scale;
  const drawnH = spanY * scale;
  const offsetX = padding + (usableW - drawnW) / 2;
  const offsetY = padding + (usableH - drawnH) / 2;

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

  // Paleta clara, para o fundo branco da identidade Toradex. A ordem de leitura
  // é a mesma da versão escura e continua sendo a que importa: livre tem de ser
  // a superfície MAIS clara, desconhecido tem de ser visivelmente diferente de
  // livre, e o gradiente de inflação tem de escurecer monotonicamente até o
  // obstáculo — assim o mapa se lê como um relevo mesmo em tons de cinza.
  //
  // int8 -1 arrives as 255 when read through an unsigned view.
  put(255, 214, 221, 228, 255); // unknown
  put(0, 250, 252, 253, 255); // free

  for (let value = 1; value <= 98; value += 1) {
    const t = value / 98;
    // Azul claro -> laranja da marca. Escurece e esquenta ao mesmo tempo, então
    // sobrevive a um monitor mal calibrado e a uma foto em preto e branco.
    put(value, 176 + 79 * t, 202 - 68 * t, 226 - 194 * t, 240);
  }
  put(99, 255, 90, 0, 255); // inscribed — laranja #ff5a00
  put(100, 176, 34, 26, 255); // lethal

  // Values above 100 are not valid in an OccupancyGrid, but a malformed
  // producer should paint something obviously wrong rather than transparent.
  for (let value = 101; value <= 254; value += 1) put(value, 255, 0, 255, 255);

  return lut;
}
