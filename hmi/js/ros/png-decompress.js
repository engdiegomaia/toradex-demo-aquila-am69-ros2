/**
 * rosbridge `compression: "png"` — client side.
 *
 * Why this is worth 80 lines, measured on 24/08/2026 against the running
 * `/global_costmap/costmap` (400x400 cells, 0.5 Hz):
 *
 *   compression   bytes per frame
 *   none               456.8 KiB
 *   cbor               156.5 KiB
 *   png                 13.8 KiB     <- 33x smaller than JSON
 *
 * That is the difference between a costmap panel that works over the bench
 * Ethernet link and one that does not. It answers open point 2 of
 * docs/ml35/plano-cockpit-web.md.
 *
 * The wire format is NOT a picture of the map. rosbridge takes the outgoing
 * JSON, encodes it as UTF-8, packs those bytes three-per-pixel into an RGB PNG
 * padded with '\n', and base64s the result — see
 * rosbridge_library/internal/pngcompression.py, which this mirrors exactly.
 * The frame arrives as {"op":"png","data":"<base64>"} and the DECODED payload
 * is an ordinary {"op":"publish",...} message.
 *
 * Two details this implementation does not inherit from roslibjs:
 *
 *   - the bytes are decoded with TextDecoder, not String.fromCharCode.
 *     The encoder calls .encode("utf-8"), so any non-ASCII field would come
 *     back as mojibake through the charcode route.
 *   - the '\n' padding is left in place. It is whitespace to JSON.parse, and
 *     stripping every newline (what the Python decoder does) would also strip
 *     newlines that legitimately appear... nowhere, today — but the safe
 *     version costs nothing.
 *
 * The logic is split so that everything except six lines of DOM is pure and
 * testable under `node --test`.
 */

/**
 * Pure: drop the alpha channel from canvas RGBA data.
 *
 * The PNG is RGB, so the canvas fills alpha with 255 and the colour channels
 * round-trip exactly. Every fourth byte is that constant alpha and is not part
 * of the payload.
 */
export function unpackRgb(rgba) {
  const pixels = Math.floor(rgba.length / 4);
  const out = new Uint8Array(pixels * 3);
  for (let p = 0, o = 0; p < pixels; p += 1) {
    const i = p * 4;
    out[o] = rgba[i];
    out[o + 1] = rgba[i + 1];
    out[o + 2] = rgba[i + 2];
    o += 3;
  }
  return out;
}

/** Pure: UTF-8 bytes (with '\n' padding) -> the original message object. */
export function bytesToMessage(bytes, decoder = new TextDecoder('utf-8')) {
  // Trailing '\n' is whitespace as far as JSON.parse is concerned, so the
  // padding needs no special handling.
  return JSON.parse(decoder.decode(bytes));
}

/**
 * Browser glue. Returns an async (base64) => message.
 *
 * `willReadFrequently` matters here: without it Chromium keeps the canvas on
 * the GPU and every getImageData is a readback stall, which on a 0.5 Hz costmap
 * is invisible but on a 10 Hz topic is not.
 */
export function createPngDecoder({
  createImage = () => new Image(),
  createCanvas = () => document.createElement('canvas'),
} = {}) {
  const canvas = createCanvas();
  const context = canvas.getContext('2d', { willReadFrequently: true });

  return async function decodePng(base64) {
    const image = createImage();
    image.src = `data:image/png;base64,${base64}`;
    await image.decode();

    canvas.width = image.width;
    canvas.height = image.height;
    context.drawImage(image, 0, 0);
    const { data } = context.getImageData(0, 0, image.width, image.height);

    return bytesToMessage(unpackRgb(data));
  };
}
