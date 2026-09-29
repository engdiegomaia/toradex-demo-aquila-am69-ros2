/**
 * The navigation panel's exploration decision, with no DOM and no canvas.
 *
 * It lives outside `nav-panel.js` for a testing reason, not an organizational
 * one: the panel only exists after a `canvas.getContext('2d')`, and the
 * bundle has no jsdom by project decision (plano-cockpit-web.md, Decision 5).
 * Mounting the whole panel to ask "should this click become a goal?" would
 * require faking an entire 2D context, and the test would end up measuring
 * the fake.
 *
 * What is here are the three questions the operator asks, where a wrong
 * answer is expensive:
 *
 *   is the search running?   -> if so, a map click does NOT become a manual goal
 *   what should the HUD show? -> state, time, frontiers, marker, failure
 *   was the exit confirmed?  -> and that comes from the validator, never the explorer
 *
 * The search state arrives on a TRANSIENT_LOCAL topic: after a rosbridge
 * drop, the last message is redelivered and the screen rebuilds itself. That
 * is why this store has no "forget": it is a function of the last status
 * seen.
 */

/**
 * States in which the explorer is in command of the robot.
 *
 * `starting` belongs to the COCKPIT, not to `maze_explorer`: it covers the
 * window between clicking "start search" and the first status coming from
 * the Aquila. Without it that window counts as "no search running", and a
 * map click becomes a manual goal on top of a search the Aquila has already
 * accepted. It is the same family of defect the rest of this file guards
 * against: the cockpit believing one thing while the module does another.
 */
export const BUSY_STATES = Object.freeze([
  'starting', 'waiting_map', 'selecting', 'navigating', 'homing_exit',
]);

/** States in which the search has ended, and the HUD still has to say how. */
export const TERMINAL_STATES = Object.freeze([
  'completed', 'failed', 'cancelled',
]);

/**
 * Reads the JSON published by `maze_explorer`.
 *
 * An invalid payload must NOT silently become `null`: the panel would go
 * back to accepting manual clicks in the middle of a search that keeps
 * running on the Aquila.
 *
 * `invalid: true` exists because `state: 'failed'` alone is not enough.
 * `failed` is TERMINAL, and terminal RELEASES the manual goal -- which is
 * exactly what must not happen when the only thing known is that the
 * channel became unreadable. Broken JSON is not news about the robot; it is
 * news about the link. Deciding what to do with that is the store's job, in
 * `apply`.
 */
export function parseExplorationStatus(message) {
  const data = message?.data;
  if (typeof data !== 'string') {
    return { state: 'failed', message: 'search status missing', invalid: true };
  }
  try {
    const parsed = JSON.parse(data);
    if (!parsed || typeof parsed !== 'object' || Array.isArray(parsed)) {
      return { state: 'failed', message: 'search status invalid', invalid: true };
    }
    return parsed;
  } catch {
    return { state: 'failed', message: 'search status invalid', invalid: true };
  }
}

export function isExplorationActive(exploration) {
  return BUSY_STATES.includes(exploration?.state);
}

/**
 * The HUD parts the search adds, in reading order.
 *
 * `EXIT CONFIRMED` comes from `/demo/maze/escaped`, published by the
 * simulation-side ground-truth validator — never from `state === 'completed'`,
 * which only says the explorer got close to the marker. Confusing the two
 * would make the cockpit declare success without the robot having actually
 * crossed the opening.
 */
export function explorationHudParts(exploration, mazeEscaped, linkError) {
  const parts = [];
  if (exploration) {
    if (Number.isFinite(exploration.elapsed_s)) {
      parts.push(`${Math.round(exploration.elapsed_s)} s`);
    }
    if (Number.isFinite(exploration.frontier_count)) {
      parts.push(`${exploration.frontier_count} frontier(s)`);
    }
    if (exploration.marker_visible) parts.push('exit detected');
    if (exploration.message) parts.push(exploration.message);
  }
  // Communication error is its OWN line, alongside the last valid state, not
  // a state that replaces it. The operator needs to see both things: what
  // the robot was doing, and that the cockpit stopped knowing.
  if (linkError) parts.push(linkError);
  if (mazeEscaped) parts.push('EXIT CONFIRMED');
  return parts;
}

/** Keeps the last status seen and answers what the panel needs to draw. */
export function createExplorationStore() {
  let exploration = null;
  let mazeEscaped = false;
  let linkError = null;
  let pending = false;

  return {
    /**
     * Applies a message from `/demo/exploration/status`.
     *
     * An unreadable payload does NOT knock down a busy state. `navigating`
     * followed by broken JSON stays blocked; `selecting` followed by a
     * disconnect too. The last valid state is preserved and the
     * communication failure becomes a separate field -- because converting
     * to `failed` would release the manual goal on top of a search that
     * keeps running on the Aquila.
     *
     * The exception is when there is no valid state at all yet: then the
     * parser's `failed` is the best information available, and it is better
     * than a blank screen.
     */
    apply(message) {
      const parsed = parseExplorationStatus(message);
      if (parsed.invalid) {
        linkError = parsed.message;
        if (exploration === null) exploration = parsed;
        return;
      }
      linkError = null;
      pending = false;
      exploration = parsed;
    },
    /**
     * Marks a search command in flight, before any response.
     *
     * `starting` covers the window between the click and the first status
     * from the Aquila. `pending` covers the service promise, and is what
     * stops a double click from turning into two calls.
     */
    beginStart() {
      pending = true;
      linkError = null;
      exploration = { state: 'starting', message: 'starting search' };
    },
    /** A cancel in flight: does not change the state, only locks the door. */
    beginCancel() {
      pending = true;
    },
    /**
     * The service refused the start, or the call blew up.
     *
     * Undoes `starting` -- a state only the cockpit invented -- so the panel
     * does not stay stuck in a lock with no search on the other side. If a
     * real status has already arrived in the meantime, it wins: the refusal
     * becomes just a message, and the lock stays with whoever has
     * authority.
     */
    refuseStart(text) {
      pending = false;
      const message = text ?? 'search command refused';
      exploration = exploration?.state === 'starting'
        ? { state: 'failed', message }
        : { ...(exploration ?? {}), message };
    },
    /** The service answered (well or badly); the promise no longer locks anything. */
    endCommand() {
      pending = false;
    },
    /** Is there a command in flight or a search running? If so, no manual goal. */
    isBusy() {
      return pending || isExplorationActive(exploration);
    },
    linkError() {
      return linkError;
    },
    /** Applies an already-built object, like a service's refusal response. */
    merge(patch) {
      exploration = { ...(exploration ?? {}), ...patch };
    },
    setEscaped(value) {
      mazeEscaped = value === true;
    },
    isActive() {
      return isExplorationActive(exploration);
    },
    /** The search owns the HUD while it runs AND after it ends. */
    ownsHud() {
      return this.isActive() || TERMINAL_STATES.includes(exploration?.state);
    },
    label() {
      return `search: ${exploration?.state ?? 'idle'}`;
    },
    hudParts() {
      return explorationHudParts(exploration, mazeEscaped, linkError);
    },
    escaped() {
      return mazeEscaped;
    },
    snapshot() {
      return exploration;
    },
  };
}
