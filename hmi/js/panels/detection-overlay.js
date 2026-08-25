/**
 * Detection boxes drawn over an MJPEG stream.
 *
 * `/demo/perception/detections` is a vision_msgs/Detection2DArray in IMAGE
 * space — measured shape on 24/08/2026: centre (167,2, 240,0) px, size
 * 120x160 px, frame `front_camera`, class `box`, score 0,87. That is where it
 * belongs on screen: over the picture, not on the floor plan. The map already
 * sees these obstacles through the costmap's perception_layer.
 *
 * The whole difficulty here is that the <img> uses `object-fit: contain`, so
 * the picture is letterboxed inside its element and image pixels are NOT
 * element pixels. Getting that wrong draws boxes that are almost right, which
 * is worse than obviously wrong: it reads as a perception error.
 */

import { readMapPalette } from './palette.js';
import { TOPICS } from '../config.js';

/**
 * Pure: where an `object-fit: contain` image actually lands inside its box.
 *
 * @returns {{x:number, y:number, width:number, height:number, scale:number}}
 */
export function containRect(elementW, elementH, imageW, imageH) {
  if (!(imageW > 0) || !(imageH > 0)) {
    return { x: 0, y: 0, width: elementW, height: elementH, scale: 1 };
  }
  const scale = Math.min(elementW / imageW, elementH / imageH);
  const width = imageW * scale;
  const height = imageH * scale;
  return {
    x: (elementW - width) / 2,
    y: (elementH - height) / 2,
    width,
    height,
    scale,
  };
}

/** Pure: a Detection2D -> a rect in image pixels. */
export function boxOf(detection) {
  const bbox = detection?.bbox;
  if (!bbox) return null;
  const centre = bbox.center?.position ?? bbox.center;
  if (!centre || !Number.isFinite(centre.x) || !Number.isFinite(centre.y)) return null;
  const sizeX = bbox.size_x ?? 0;
  const sizeY = bbox.size_y ?? 0;
  return { x: centre.x - sizeX / 2, y: centre.y - sizeY / 2, width: sizeX, height: sizeY };
}

/** Pure: the best hypothesis of a detection, as a short label. */
export function labelOf(detection) {
  const best = detection?.results?.[0]?.hypothesis;
  if (!best) return detection?.id ?? '?';
  const score = Number.isFinite(best.score) ? ` ${(best.score * 100).toFixed(0)}%` : '';
  return `${best.class_id ?? '?'}${score}`;
}

export function createDetectionOverlay({ root, client, tracker, freshnessKey = 'detections' }) {
  // Mesma fonte de cor do mapa: a meta e as detecções são as duas coisas que
  // o operador aponta com o dedo, e devem ser a mesma cor.
  const palette = readMapPalette(root);

  const canvas = root.querySelector('[data-role="detections"]');
  const image = root.querySelector('[data-role="stream"]');
  const context = canvas.getContext('2d');

  const state = { detections: [], imageW: 0, imageH: 0 };
  const unsubscribes = [];
  let cssWidth = 0;
  let cssHeight = 0;

  const resize = () => {
    const rect = canvas.getBoundingClientRect();
    const ratio = window.devicePixelRatio || 1;
    cssWidth = Math.max(1, Math.round(rect.width));
    cssHeight = Math.max(1, Math.round(rect.height));
    canvas.width = Math.round(cssWidth * ratio);
    canvas.height = Math.round(cssHeight * ratio);
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
  };
  const observer = new ResizeObserver(resize);
  observer.observe(canvas);
  resize();

  unsubscribes.push(
    client.subscribe(
      TOPICS.detections,
      'vision_msgs/msg/Detection2DArray',
      (message) => {
        tracker.mark(freshnessKey);
        state.detections = message?.detections ?? [];
      },
      { throttleRate: 100 },
    ),
  );

  // The same subscription the stream panel uses — the client shares one ROS
  // subscription per topic and fans out to handlers, so this costs nothing on
  // the wire. camera_info is authoritative for the image size; naturalWidth is
  // the fallback for a stream whose CameraInfo is not published.
  unsubscribes.push(
    client.subscribe(
      '/demo/camera/camera_info',
      'sensor_msgs/msg/CameraInfo',
      (message) => {
        state.imageW = message?.width ?? 0;
        state.imageH = message?.height ?? 0;
      },
      { throttleRate: 1000 },
    ),
  );

  function draw() {
    context.clearRect(0, 0, cssWidth, cssHeight);
    if (!state.detections.length) return;

    const imageW = state.imageW || image?.naturalWidth || 0;
    const imageH = state.imageH || image?.naturalHeight || 0;
    if (!imageW || !imageH) return;

    const fit = containRect(cssWidth, cssHeight, imageW, imageH);

    context.save();
    context.lineWidth = 2;
    context.strokeStyle = palette.goal;
    context.fillStyle = palette.goal;
    context.font = '11px ui-monospace, monospace';
    context.textBaseline = 'bottom';

    for (const detection of state.detections) {
      const box = boxOf(detection);
      if (!box) continue;
      const x = fit.x + box.x * fit.scale;
      const y = fit.y + box.y * fit.scale;
      const width = box.width * fit.scale;
      const height = box.height * fit.scale;
      context.strokeRect(x, y, width, height);
      const label = labelOf(detection);
      const textWidth = context.measureText(label).width;
      context.globalAlpha = 0.85;
      context.fillRect(x, y - 14, textWidth + 8, 14);
      context.globalAlpha = 1;
      context.save();
      context.fillStyle = palette.label;
      context.fillText(label, x + 4, y - 2);
      context.restore();
    }
    context.restore();
  }

  return {
    tick: draw,
    onLinkDown() {
      tracker.clear(freshnessKey);
      state.detections = [];
    },
    destroy() {
      observer.disconnect();
      for (const off of unsubscribes) off();
    },
  };
}
