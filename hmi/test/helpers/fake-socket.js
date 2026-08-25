/**
 * WebSocket double for the rosbridge client tests.
 *
 * The client takes a socketFactory precisely so this can exist: the transport
 * logic that matters (reconnect, subscription replay, refusing to publish while
 * down) is the part that is hardest to exercise against a real server and the
 * part whose failure is silent in production.
 */

export class FakeSocket {
  constructor(url) {
    this.url = url;
    this.sent = [];
    this.closed = false;
    this.onopen = null;
    this.onclose = null;
    this.onerror = null;
    this.onmessage = null;
  }

  send(data) {
    this.sent.push(JSON.parse(data));
  }

  close() {
    if (this.closed) return;
    this.closed = true;
    this.onclose?.({});
  }

  // --- test-side drivers ---------------------------------------------------

  open() {
    this.onopen?.({});
  }

  /** Simulate the peer going away (container restart, cable pulled). */
  drop() {
    this.closed = true;
    this.onclose?.({});
  }

  deliver(payload) {
    this.onmessage?.({ data: JSON.stringify(payload) });
  }

  deliverRaw(data) {
    this.onmessage?.({ data });
  }

  opsSent(op) {
    return this.sent.filter((frame) => frame.op === op);
  }
}

/** Factory plus the list of sockets it produced, in creation order. */
export function trackingFactory() {
  const sockets = [];
  const factory = (url) => {
    const socket = new FakeSocket(url);
    sockets.push(socket);
    return socket;
  };
  return { factory, sockets, get latest() { return sockets.at(-1); } };
}

/** Deterministic timer stand-in: nothing fires until the test says so. */
export function manualTimers() {
  const pending = new Map();
  let nextId = 1;
  return {
    setTimeoutFn: (callback, delay) => {
      const id = nextId++;
      pending.set(id, { callback, delay });
      return id;
    },
    clearTimeoutFn: (id) => pending.delete(id),
    get size() {
      return pending.size;
    },
    delays() {
      return [...pending.values()].map((entry) => entry.delay);
    },
    /** Fire every pending timer once, oldest first. */
    runAll() {
      const entries = [...pending.entries()];
      pending.clear();
      for (const [, entry] of entries) entry.callback();
    },
  };
}
