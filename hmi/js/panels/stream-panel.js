/**
 * MJPEG stream panel — drives both the robot camera and the scene view.
 *
 * Split of concerns, and it is not arbitrary:
 *
 *   pixels    arrive over HTTP from web_video_server, inside an <img>;
 *   liveness  arrives over the rosbridge WebSocket, from a TINY companion
 *             topic (CameraInfo) published alongside the image.
 *
 * The <img> is a poor liveness oracle and this was measured, not assumed: in
 * Firefox an <img> bound to multipart/x-mixed-replace never fires `load` and
 * reports `complete === false` for the entire life of a healthy stream. A panel
 * that trusted `load` would declare "sem sinal" over live video. Conversely a
 * stream that stops leaves the last frame painted with no event at all — the
 * failure AGENTS.md §5.7 asks the stale-data state to catch. CameraInfo is a
 * few hundred bytes at the image rate and answers both questions honestly.
 *
 * naturalWidth is still consulted, for one specific reason: it separates "ROS
 * is silent" from "ROS is fine and the HTTP side is broken". Those have
 * different fixes and used to look identical.
 *
 * The source is swappable at runtime (setSource) because the scene panel offers
 * two cameras, isometric and top, over a single element. Swapping resets the
 * grace period and clears the freshness key: the new topic has genuinely never
 * been seen, and inheriting the old topic's "live" state would paint a green
 * dot over a camera that may not exist.
 */

import { STREAM_QUALITY, streamUrl } from '../config.js';

/** Give the encoder time to produce a first frame before calling it dead. */
const FIRST_FRAME_GRACE_MS = 5000;

/** Backoff after the <img> reports an outright transport error. */
const RETRY_AFTER_ERROR_MS = 2000;

export function createStreamPanel({
  root,
  client,
  tracker,
  videoBase,
  topic,
  heartbeat = null,
  freshnessKey,
  quality = STREAM_QUALITY,
  setTimeoutFn = setTimeout,
  now = () => Date.now(),
}) {
  const img = root.querySelector('[data-role="stream"]');
  const notice = root.querySelector('[data-role="notice"]');
  const noticeDetail = root.querySelector('[data-role="notice-detail"]');
  const noticeHeadline = root.querySelector('[data-role="notice-headline"]');

  let nonce = 0;
  let unsubscribe = null;
  let transportError = false;
  let currentTopic = topic;
  let currentHeartbeat = heartbeat;
  let mountedAt = now();

  const showNotice = (headline, detail) => {
    if (noticeHeadline.textContent !== headline) {
      noticeHeadline.textContent = headline;
    }
    if (noticeDetail.textContent !== detail) noticeDetail.textContent = detail;
    notice.hidden = false;
  };

  const load = () => {
    transportError = false;
    nonce += 1;
    img.src = streamUrl(videoBase, currentTopic, { quality, nonce });
  };

  // An <img> whose stream endpoint is refused fires `error` once and then stays
  // blank forever. Without an explicit retry the panel never recovers from a
  // web_video_server restart while every other panel does, which teaches the
  // operator to trust the wrong thing.
  img.addEventListener('error', () => {
    transportError = true;
    setTimeoutFn(load, RETRY_AFTER_ERROR_MS);
  });

  const listen = () => {
    if (unsubscribe) unsubscribe();
    unsubscribe = null;
    if (!currentHeartbeat) return;
    unsubscribe = client.subscribe(
      currentHeartbeat.topic,
      currentHeartbeat.type,
      () => tracker.mark(freshnessKey),
      // 5 Hz is plenty to drive a dot and keeps a 30 Hz camera_info from
      // waking the main thread on every frame for no visible benefit.
      { throttleRate: 200 },
    );
  };

  listen();
  load();

  return {
    /**
     * Repaint the overlay. Called from the shared tick in main.js so the panel
     * has no timer of its own.
     *
     * The four states are deliberately distinct — each has a different fix:
     */
    tick() {
      const fresh = tracker.stateOf(freshnessKey);
      const decoded = img.naturalWidth > 0;
      const settling = now() - mountedAt < FIRST_FRAME_GRACE_MS;

      // With nothing to show, the <img> is hidden rather than covered: an alt
      // string bleeding through a translucent overlay reads as a broken page.
      // For `stale` it stays visible on purpose — the frozen frame is the
      // evidence, and the overlay only labels it.
      const blank = transportError || fresh === 'never';
      if (img.hidden !== blank) img.hidden = blank;

      if (transportError) {
        showNotice('sem sinal', `${currentTopic}\nweb_video_server inacessível — nova tentativa`);
        return;
      }
      if (fresh === 'never') {
        showNotice(
          settling ? 'conectando' : 'sem sinal',
          settling ? `${currentTopic}\naguardando o primeiro quadro` : `${currentTopic}\nnada publicado neste tópico`,
        );
        return;
      }
      if (fresh === 'stale') {
        showNotice('sinal congelado', `${currentTopic}\npublicação parou — o quadro na tela é antigo`);
        return;
      }
      if (!decoded) {
        // ROS is publishing but the browser has no pixels: the HTTP half is at
        // fault, not the robot. Naming web_video_server here is the difference
        // between a two-minute fix and an afternoon.
        showNotice(
          'sem vídeo',
          `${currentTopic}\ntópico ativo, mas o MJPEG não decodifica — verifique web_video_server`,
        );
        return;
      }
      notice.hidden = true;
    },

    /** Called by the main loop when the transport state changes. */
    onLinkDown() {
      tracker.clear(freshnessKey);
    },

    /** Force a fresh stream — used after the backend comes back. */
    reload: load,

    /** Which topic the element is currently pointed at. */
    source: () => currentTopic,

    /**
     * Point the element at a different camera (the scene panel's iso/top
     * toggle). No-op when already there, so a click on the active button does
     * not tear down a working stream.
     */
    setSource({ topic: next, heartbeat: nextHeartbeat = null }) {
      if (next === currentTopic) return;
      currentTopic = next;
      currentHeartbeat = nextHeartbeat;
      transportError = false;
      mountedAt = now();
      tracker.clear(freshnessKey);
      listen();
      load();
    },

    destroy() {
      if (unsubscribe) unsubscribe();
      img.removeAttribute('src');
    },
  };
}
