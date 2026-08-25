/**
 * Per-source staleness tracking.
 *
 * AGENTS.md §5.7 requires the HMI to "display stale-data state". The three
 * states are kept distinct on purpose:
 *
 *   never — nothing has ever arrived on this source since the page loaded;
 *   live  — a sample arrived within this source's window;
 *   stale — samples arrived and then stopped.
 *
 * Collapsing "stale" into "never" would hide the single most common bench
 * failure: a simulator or a bridge that dies mid-demo while the browser keeps
 * displaying the last frame it received. The panel still shows a picture, and
 * without this distinction nothing on screen says the picture is frozen.
 *
 * Measured against the BROWSER's wall clock, not ROS time. The cockpit must
 * report "this stopped updating" even when the thing that stopped is /clock
 * itself.
 */

export const Freshness = Object.freeze({
  NEVER: 'never',
  LIVE: 'live',
  STALE: 'stale',
});

/** Generous enough for a 1 Hz map republish, tight enough to catch a stall. */
export const DEFAULT_STALE_AFTER_MS = 2000;

export class FreshnessTracker {
  /** @param {{now?: () => number}} [options] */
  constructor({ now = () => Date.now() } = {}) {
    this._now = now;
    /** key -> {staleAfterMs, lastSeen: number|null} */
    this._sources = new Map();
  }

  register(key, { staleAfterMs = DEFAULT_STALE_AFTER_MS } = {}) {
    this._sources.set(key, { staleAfterMs, lastSeen: null });
    return this;
  }

  /** Record a sample. Registers the source with defaults if it is unknown. */
  mark(key) {
    let source = this._sources.get(key);
    if (!source) {
      source = { staleAfterMs: DEFAULT_STALE_AFTER_MS, lastSeen: null };
      this._sources.set(key, source);
    }
    source.lastSeen = this._now();
  }

  /**
   * Reset a source back to "never".
   *
   * Used when the transport drops: a stream that was live 200 ms before a
   * disconnect should not keep reporting "live" for the rest of its window
   * while the socket is down.
   */
  clear(key) {
    const source = this._sources.get(key);
    if (source) source.lastSeen = null;
  }

  clearAll() {
    for (const source of this._sources.values()) source.lastSeen = null;
  }

  stateOf(key) {
    const source = this._sources.get(key);
    if (!source || source.lastSeen === null) return Freshness.NEVER;
    const age = this._now() - source.lastSeen;
    return age <= source.staleAfterMs ? Freshness.LIVE : Freshness.STALE;
  }

  /** Age in ms since the last sample, or null if there has never been one. */
  ageOf(key) {
    const source = this._sources.get(key);
    if (!source || source.lastSeen === null) return null;
    return this._now() - source.lastSeen;
  }

  /** @returns {Record<string, string>} key -> Freshness */
  snapshot() {
    const out = {};
    for (const key of this._sources.keys()) out[key] = this.stateOf(key);
    return out;
  }
}
