/**
 * Minimal rosbridge v2 protocol client.
 *
 * Why this exists instead of roslibjs: plano-cockpit-web.md Decision 5. The
 * bundle has no build step and no node_modules, because the same files are
 * meant to be baked into an arm64 kiosk image for the Aquila in M3 and the npm
 * supply chain is exactly what should not travel there. The wire protocol is
 * small JSON; this is all of it that the cockpit uses.
 *
 * The class knows nothing about panels, canvases or the DOM. It is pure
 * transport plus connection state, which is what makes it testable under
 * `node --test` with a fake socket (hmi/test/rosbridge-client.test.js).
 *
 * Everything the browser environment provides is injectable for that reason:
 * socketFactory, setTimeoutFn, clearTimeoutFn.
 */

export const ConnectionState = Object.freeze({
  CONNECTING: 'connecting',
  CONNECTED: 'connected',
  DISCONNECTED: 'disconnected',
});

/**
 * Reconnect backoff, in milliseconds.
 *
 * Starts fast because the overwhelmingly common case on the bench is a
 * container restart that is back within a second, and ends slow so that a
 * cockpit left open overnight against a stopped stack does not spin. The last
 * value repeats forever — the client never gives up on its own, since the
 * operator's mental model is "the page reconnects when the robot comes back".
 */
export const RETRY_DELAYS_MS = Object.freeze([250, 500, 1000, 2000, 5000]);

const noop = () => {};

export class RosbridgeClient {
  /**
   * @param {object} options
   * @param {string} options.url                 ws:// or wss:// endpoint.
   * @param {(url: string) => object} [options.socketFactory]
   * @param {typeof setTimeout} [options.setTimeoutFn]
   * @param {typeof clearTimeout} [options.clearTimeoutFn]
   * @param {(message: string, error?: unknown) => void} [options.onError]
   * @param {(base64: string) => Promise<object>} [options.decodePng]
   *        Required only if a subscription asks for compression: 'png'.
   *        Injected rather than imported because decoding needs a canvas, and
   *        the transport itself must stay runnable under `node --test`.
   */
  constructor({
    url,
    socketFactory = (target) => new WebSocket(target),
    setTimeoutFn = setTimeout,
    clearTimeoutFn = clearTimeout,
    onError = noop,
    decodePng = null,
  }) {
    this.url = url;
    this._socketFactory = socketFactory;
    this._setTimeout = setTimeoutFn;
    this._clearTimeout = clearTimeoutFn;
    this._onError = onError;
    this._decodePng = decodePng;

    this._socket = null;
    this._state = ConnectionState.DISCONNECTED;
    this._stateListeners = new Set();

    /** topic -> {type, options, handlers:Set} */
    this._subscriptions = new Map();
    /** topic -> type */
    this._advertised = new Map();
    /** id -> {resolve, reject} */
    this._pendingServiceCalls = new Map();
    /** id -> {action, onFeedback, resolve, reject} */
    this._activeGoals = new Map();

    // PNG frames decode asynchronously, so two costmaps could in principle
    // finish out of order and repaint an older map over a newer one. At 0.5 Hz
    // with a ~5 ms decode that is close to impossible — and "close to
    // impossible" is the category of bug this project keeps paying for.
    this._pngSeq = 0;
    this._pngDispatched = new Map();

    this._retryIndex = 0;
    this._retryTimer = null;
    this._closedByUser = false;
    this._idCounter = 0;
  }

  get state() {
    return this._state;
  }

  /** @returns {() => void} unregister */
  onStateChange(listener) {
    this._stateListeners.add(listener);
    listener(this._state);
    return () => this._stateListeners.delete(listener);
  }

  connect() {
    this._closedByUser = false;
    this._openSocket();
  }

  /** Deliberate shutdown: no reconnect is attempted afterwards. */
  close() {
    this._closedByUser = true;
    this._cancelRetry();
    if (this._socket) {
      const socket = this._socket;
      this._socket = null;
      try {
        socket.close();
      } catch (error) {
        this._onError('failed to close socket', error);
      }
    }
    this._setState(ConnectionState.DISCONNECTED);
  }

  /**
   * Subscribe to a topic. Safe to call before the socket is open: the request
   * is replayed on every (re)connection, which is what makes a mid-demo
   * rosbridge restart invisible to the panels.
   *
   * @returns {() => void} unsubscribe
   */
  subscribe(topic, type, handler, options = {}) {
    let entry = this._subscriptions.get(topic);
    if (!entry) {
      entry = { type, options, handlers: new Set() };
      this._subscriptions.set(topic, entry);
      this._sendSubscribe(topic, entry);
    }
    entry.handlers.add(handler);

    return () => {
      const current = this._subscriptions.get(topic);
      if (!current) return;
      current.handlers.delete(handler);
      if (current.handlers.size === 0) {
        this._subscriptions.delete(topic);
        this._send({ op: 'unsubscribe', topic });
      }
    };
  }

  advertise(topic, type) {
    if (this._advertised.get(topic) === type) return;
    this._advertised.set(topic, type);
    this._send({ op: 'advertise', topic, type });
  }

  /**
   * Publish to an already-advertised topic.
   *
   * Returns false when the socket is down instead of queueing. Queued velocity
   * commands are a safety problem, not a feature: a burst of buffered
   * /demo/cmd_vel delivered on reconnect would drive a robot that the operator
   * believes is stopped.
   */
  publish(topic, msg) {
    if (!this._advertised.has(topic)) {
      this._onError(`publish to un-advertised topic ${topic}`);
      return false;
    }
    return this._send({ op: 'publish', topic, msg });
  }

  /** @returns {Promise<object>} the service response `values`. */
  callService(service, args = {}) {
    const id = this._nextId('call');
    return new Promise((resolve, reject) => {
      if (!this._send({ op: 'call_service', service, args, id })) {
        reject(new Error(`rosbridge is not connected (service ${service})`));
        return;
      }
      this._pendingServiceCalls.set(id, { resolve, reject });
    });
  }

  // --- internals ----------------------------------------------------------

  _nextId(prefix) {
    this._idCounter += 1;
    return `${prefix}:${this._idCounter}`;
  }

  _openSocket() {
    this._cancelRetry();
    this._setState(ConnectionState.CONNECTING);

    let socket;
    try {
      socket = this._socketFactory(this.url);
    } catch (error) {
      this._onError(`could not open ${this.url}`, error);
      this._scheduleRetry();
      return;
    }
    this._socket = socket;

    socket.onopen = () => {
      this._retryIndex = 0;
      this._setState(ConnectionState.CONNECTED);
      this._replayState();
    };
    socket.onmessage = (event) => this._handleMessage(event);
    socket.onerror = (event) => this._onError('websocket error', event);
    socket.onclose = () => {
      if (this._socket !== socket) return;
      this._socket = null;
      this._rejectPendingCalls('rosbridge connection closed');
      this._abandonGoals();
      this._setState(ConnectionState.DISCONNECTED);
      if (!this._closedByUser) this._scheduleRetry();
    };
  }

  /**
   * Re-send every advertise and subscribe after a reconnect.
   *
   * rosbridge keeps no state across connections, so without this the page
   * reconnects, the badge turns green, and no message ever arrives again. That
   * is the worst of the three possible outcomes because it looks healthy.
   */
  _replayState() {
    for (const [topic, type] of this._advertised) {
      this._send({ op: 'advertise', topic, type });
    }
    for (const [topic, entry] of this._subscriptions) {
      this._sendSubscribe(topic, entry);
    }
  }

  _sendSubscribe(topic, entry) {
    const request = {
      op: 'subscribe',
      topic,
      type: entry.type,
      // Depth 1 by default: for live views the cockpit always wants the newest
      // sample and never a backlog — without it a slow browser tab accumulates
      // frames and then replays history as if it were live.
      //
      // Logs are the exception and must override it. /rosout is bursty, and a
      // depth-1 queue silently discards everything but the last line of a
      // multi-line startup failure — which is precisely the burst an operator
      // opens the log panel to read.
      queue_length: entry.options.queueLength ?? 1,
      throttle_rate: entry.options.throttleRate ?? 0,
    };
    if (entry.options.compression) {
      request.compression = entry.options.compression;
    }
    this._send(request);
  }

  _send(payload) {
    if (!this._socket || this._state !== ConnectionState.CONNECTED) return false;
    try {
      this._socket.send(JSON.stringify(payload));
      return true;
    } catch (error) {
      this._onError('failed to send', error);
      return false;
    }
  }

  _handleMessage(event) {
    let payload;
    try {
      payload = JSON.parse(event.data);
    } catch (error) {
      this._onError('malformed frame from rosbridge', error);
      return;
    }

    if (payload.op === 'png') {
      this._handlePngFrame(payload);
      return;
    }

    this._dispatch(payload);
  }

  /**
   * Decode a PNG-compressed frame and dispatch the message it carries.
   *
   * The decoded payload is an ordinary {"op":"publish",...}; the PNG is only a
   * transport envelope. See js/ros/png-decompress.js.
   */
  _handlePngFrame(payload) {
    if (!this._decodePng) {
      this._onError('a png frame arrived but no decoder was configured');
      return;
    }
    this._pngSeq += 1;
    const seq = this._pngSeq;
    this._decodePng(payload.data)
      .then((message) => {
        const topic = message?.topic;
        if (topic) {
          if ((this._pngDispatched.get(topic) ?? 0) > seq) return;
          this._pngDispatched.set(topic, seq);
        }
        this._dispatch(message);
      })
      .catch((error) => this._onError('failed to decode a png frame', error));
  }

  _dispatch(payload) {
    if (payload.op === 'publish') {
      const entry = this._subscriptions.get(payload.topic);
      if (!entry) return;
      for (const handler of entry.handlers) {
        try {
          handler(payload.msg, payload.topic);
        } catch (error) {
          // One broken panel must not stop the others from updating.
          this._onError(`handler for ${payload.topic} threw`, error);
        }
      }
      return;
    }

    if (payload.op === 'service_response') {
      const pending = this._pendingServiceCalls.get(payload.id);
      if (!pending) return;
      this._pendingServiceCalls.delete(payload.id);
      if (payload.result === false) {
        pending.reject(new Error(payload.values ?? 'service call failed'));
      } else {
        pending.resolve(payload.values ?? {});
      }
      return;
    }

    if (payload.op === 'action_feedback') {
      const goal = this._activeGoals.get(payload.id);
      if (!goal) return;
      try {
        goal.onFeedback?.(payload.values ?? {});
      } catch (error) {
        this._onError('action feedback handler threw', error);
      }
      return;
    }

    if (payload.op === 'action_result') {
      const goal = this._activeGoals.get(payload.id);
      if (!goal) return;
      this._activeGoals.delete(payload.id);
      goal.resolve({
        // rosbridge sets result:false when the goal was rejected, aborted or
        // cancelled. It is NOT an error — a refused goal is normal operation
        // and the cockpit has to show it, not throw it away.
        succeeded: payload.result !== false,
        status: payload.status,
        values: payload.values ?? {},
      });
      return;
    }

    if (payload.op === 'status' && payload.level === 'error') {
      this._onError(`rosbridge: ${payload.msg}`);
    }
  }

  /**
   * Send a goal to a ROS 2 action server and follow it to completion.
   *
   * Verified against rosbridge 2.7.0 + Nav2 on 24/08/2026: the server registers
   * SendActionGoal / ActionFeedback / ActionResult, and a NavigateToPose goal
   * sent this way is accepted and executed.
   *
   * Two measured caveats, both load-bearing for the caller:
   *
   *  - feedback is NOT throttleable. Nav2 emitted ~100 frames per second, each
   *    carrying a full pose. Store the latest and render it on your own tick;
   *    do not repaint per frame.
   *  - cancel only works from the SAME WebSocket connection that sent the goal
   *    (rosbridge keeps the goal handle per client). After a reconnect the goal
   *    is still running on the robot and can no longer be cancelled from here.
   *
   * @returns {{id: string, result: Promise<object>, cancel: () => void}}
   */
  sendActionGoal(action, actionType, args, { onFeedback = null } = {}) {
    const id = this._nextId('goal');
    let settle;
    const result = new Promise((resolve) => {
      settle = resolve;
    });

    const sent = this._send({
      op: 'send_action_goal',
      id,
      action,
      action_type: actionType,
      args,
      feedback: Boolean(onFeedback),
    });

    if (!sent) {
      settle({ succeeded: false, status: null, values: {}, notSent: true });
      return { id, result, cancel: noop };
    }

    this._activeGoals.set(id, { action, onFeedback, resolve: settle });
    return {
      id,
      result,
      cancel: () => this.cancelActionGoal(id),
    };
  }

  cancelActionGoal(id) {
    const goal = this._activeGoals.get(id);
    if (!goal) return false;
    return this._send({ op: 'cancel_action_goal', id, action: goal.action });
  }

  /** Ids of goals this connection still considers in flight. */
  activeGoalIds() {
    return [...this._activeGoals.keys()];
  }

  _rejectPendingCalls(reason) {
    for (const { reject } of this._pendingServiceCalls.values()) {
      reject(new Error(reason));
    }
    this._pendingServiceCalls.clear();
  }

  /**
   * Settle in-flight goals when the socket dies.
   *
   * `lost: true` and NOT `succeeded: false`, because the two mean different
   * things to an operator: the goal is very probably still executing on the
   * robot, we simply stopped being able to see or cancel it. Reporting it as a
   * failure would be a lie in the dangerous direction.
   */
  _abandonGoals() {
    for (const goal of this._activeGoals.values()) {
      goal.resolve({ succeeded: false, status: null, values: {}, lost: true });
    }
    this._activeGoals.clear();
  }

  _scheduleRetry() {
    const delay = RETRY_DELAYS_MS[
      Math.min(this._retryIndex, RETRY_DELAYS_MS.length - 1)
    ];
    this._retryIndex += 1;
    this._retryTimer = this._setTimeout(() => {
      this._retryTimer = null;
      if (!this._closedByUser) this._openSocket();
    }, delay);
  }

  _cancelRetry() {
    if (this._retryTimer !== null) {
      this._clearTimeout(this._retryTimer);
      this._retryTimer = null;
    }
  }

  _setState(next) {
    if (this._state === next) return;
    this._state = next;
    for (const listener of this._stateListeners) {
      try {
        listener(next);
      } catch (error) {
        this._onError('state listener threw', error);
      }
    }
  }
}
