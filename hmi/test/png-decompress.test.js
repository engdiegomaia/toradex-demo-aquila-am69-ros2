/**
 * rosbridge `compression: "png"`, client side.
 *
 * The encoder is reproduced here in a few lines so the tests exercise the real
 * round trip rather than a hand-picked byte string: pack UTF-8 JSON three bytes
 * per RGB pixel, pad with '\n'. It mirrors
 * rosbridge_library/internal/pngcompression.py.
 */

import test from 'node:test';
import assert from 'node:assert/strict';

import {
  bytesToMessage,
  createPngDecoder,
  unpackRgb,
} from '../js/ros/png-decompress.js';

/** The encoder side: message -> the RGBA bytes a canvas would hand back. */
function encodeToRgba(message) {
  const bytes = new TextEncoder().encode(JSON.stringify(message));
  // Pad to a whole number of pixels exactly as the server does.
  const pixels = Math.ceil(bytes.length / 3);
  const padded = new Uint8Array(pixels * 3).fill(0x0a);
  padded.set(bytes);

  const rgba = new Uint8ClampedArray(pixels * 4);
  for (let p = 0; p < pixels; p += 1) {
    rgba[p * 4] = padded[p * 3];
    rgba[p * 4 + 1] = padded[p * 3 + 1];
    rgba[p * 4 + 2] = padded[p * 3 + 2];
    rgba[p * 4 + 3] = 255;
  }
  return rgba;
}

test('unpackRgb drops the alpha channel', () => {
  const rgba = new Uint8ClampedArray([1, 2, 3, 255, 4, 5, 6, 255]);
  assert.deepEqual([...unpackRgb(rgba)], [1, 2, 3, 4, 5, 6]);
});

test('unpackRgb ignores a truncated trailing pixel', () => {
  const rgba = new Uint8ClampedArray([1, 2, 3, 255, 9, 9]);
  assert.deepEqual([...unpackRgb(rgba)], [1, 2, 3]);
});

test('bytesToMessage parses through the newline padding', () => {
  const message = { op: 'publish', topic: '/global_costmap/costmap' };
  assert.deepEqual(bytesToMessage(unpackRgb(encodeToRgba(message))), message);
});

test('non-ASCII survives the round trip', () => {
  // This is the reason the decoder uses TextDecoder rather than
  // String.fromCharCode: the server encodes UTF-8, and a rosout line with an
  // accent is routine in this project.
  const message = { op: 'publish', msg: { name: 'navigation · attention · naïve café' } };
  assert.deepEqual(bytesToMessage(unpackRgb(encodeToRgba(message))), message);
});

test('an int8 payload survives byte for byte', () => {
  // The costmap is the real cargo: 0..100 plus -1, serialised as JSON numbers.
  const data = [-1, 0, 1, 99, 100, -1, 50];
  const message = { op: 'publish', msg: { data } };
  const decoded = bytesToMessage(unpackRgb(encodeToRgba(message)));
  assert.deepEqual(decoded.msg.data, data);
});

test('a payload whose length is an exact multiple of three needs no padding', () => {
  // Off-by-one in the padding maths only shows on this boundary.
  const message = { abc: 1 };
  const json = JSON.stringify(message);
  assert.equal(json.length % 3, 0, 'fixture must land on the boundary');
  assert.deepEqual(bytesToMessage(unpackRgb(encodeToRgba(message))), message);
});

test('createPngDecoder drives the canvas and returns the message', async () => {
  const message = { op: 'publish', topic: '/x', msg: { data: [1, 2, 3] } };
  const rgba = encodeToRgba(message);
  const width = rgba.length / 4;

  const seen = { src: null, drawn: 0, readFrequently: null };
  const decode = createPngDecoder({
    createImage: () => ({
      width,
      height: 1,
      set src(value) {
        seen.src = value;
      },
      decode: async () => {},
    }),
    createCanvas: () => ({
      width: 0,
      height: 0,
      getContext(_type, options) {
        seen.readFrequently = options?.willReadFrequently ?? false;
        return {
          drawImage: () => {
            seen.drawn += 1;
          },
          getImageData: () => ({ data: rgba }),
        };
      },
    }),
  });

  assert.deepEqual(await decode('QUJD'), message);
  assert.equal(seen.src, 'data:image/png;base64,QUJD', 'data URL montada');
  assert.equal(seen.drawn, 1);
  // Without willReadFrequently every getImageData is a GPU readback stall.
  assert.equal(seen.readFrequently, true);
});

test('the decoder reuses one canvas across frames', async () => {
  let canvases = 0;
  const rgba = encodeToRgba({ n: 1 });
  const decode = createPngDecoder({
    createImage: () => ({
      width: rgba.length / 4,
      height: 1,
      set src(_value) {},
      decode: async () => {},
    }),
    createCanvas: () => {
      canvases += 1;
      return {
        width: 0,
        height: 0,
        getContext: () => ({
          drawImage: () => {},
          getImageData: () => ({ data: rgba }),
        }),
      };
    },
  });

  await decode('AA');
  await decode('AA');
  // A canvas per frame at 0.5 Hz is survivable; at 10 Hz it is not, and the
  // difference is invisible until it is not.
  assert.equal(canvases, 1);
});

test('a corrupt payload rejects rather than resolving with junk', async () => {
  const decode = createPngDecoder({
    createImage: () => ({
      width: 1,
      height: 1,
      set src(_value) {},
      decode: async () => {},
    }),
    createCanvas: () => ({
      width: 0,
      height: 0,
      getContext: () => ({
        drawImage: () => {},
        getImageData: () => ({ data: new Uint8ClampedArray([120, 121, 122, 255]) }),
      }),
    }),
  });

  await assert.rejects(() => decode('AA'), SyntaxError);
});
