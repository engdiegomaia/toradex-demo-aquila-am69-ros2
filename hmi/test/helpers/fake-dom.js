/**
 * Minimal DOM double for the panels' wiring.
 *
 * The bundle has no build step and no jsdom — `package.json` exists for
 * `node --test` and nothing else (plano-cockpit-web.md, Decision 5). So what
 * this file provides is the EXACT surface the panels use: querySelector by
 * attribute, addEventListener, setAttribute/getAttribute, dataset and
 * textContent.
 *
 * It is not a DOM. It is enough to answer the one question that a button's
 * wiring raises and that a pure-function test does not reach: who wrote to
 * this attribute, the click or ROS?
 */

class FakeElement {
  constructor(selector, { text = '' } = {}) {
    /** The selector this element is found by, e.g. '[data-role="x"]'. */
    this.selector = selector;
    this.attributes = new Map();
    this.dataset = {};
    this.textContent = text;
    this.hidden = false;
    this.listeners = new Map();
  }

  addEventListener(type, handler) {
    const bucket = this.listeners.get(type) ?? [];
    bucket.push(handler);
    this.listeners.set(type, bucket);
  }

  setAttribute(name, value) {
    this.attributes.set(name, String(value));
  }

  getAttribute(name) {
    return this.attributes.has(name) ? this.attributes.get(name) : null;
  }

  querySelectorAll() {
    return [];
  }

  /** Fires the handlers and returns the promises they return. */
  emit(type, event = {}) {
    return (this.listeners.get(type) ?? []).map((handler) => handler(event));
  }
}

export function fakeRoot(selectors) {
  const elements = new Map(
    selectors.map((selector) => [selector, new FakeElement(selector)]),
  );
  return {
    elements,
    get(selector) {
      return elements.get(selector);
    },
    querySelector(selector) {
      return elements.get(selector) ?? null;
    },
    querySelectorAll(selector) {
      const found = elements.get(selector);
      return found ? [found] : [];
    },
  };
}

/**
 * RosbridgeClient double.
 *
 * `callService` returns whatever the test queued, and records the call: the
 * two things that matter about a service button are what it sent and what it
 * did with the response.
 */
export function fakeClient({ serviceResult = { success: true } } = {}) {
  const subscriptions = [];
  const calls = [];
  return {
    calls,
    subscriptions,
    advertise() {},
    publish() {},
    onStateChange() {
      return () => {};
    },
    subscribe(topic, type, handler) {
      const entry = { topic, type, handler, active: true };
      subscriptions.push(entry);
      return () => {
        entry.active = false;
      };
    },
    callService(service, args) {
      calls.push({ service, args });
      return Promise.resolve(serviceResult);
    },
    /** Delivers a message to whoever subscribed to that topic. */
    deliver(topic, message) {
      for (const entry of subscriptions) {
        if (entry.active && entry.topic === topic) entry.handler(message);
      }
    },
  };
}

/**
 * `window.clearTimeout`/`clearInterval` for Node.
 *
 * The panels' `stopHold` talks to `window.*` because that is what the browser
 * offers, and `destroy()` goes through it. Rewriting the module to fit the test
 * is not worth it: the shim is four lines and `node --test` has no DOM by
 * project decision (bundle with no build step — see plano-cockpit-web.md,
 * Decision 5).
 */
export function installWindowTimers() {
  globalThis.window ??= {
    setTimeout: (...args) => setTimeout(...args),
    setInterval: (...args) => setInterval(...args),
    clearTimeout: (id) => clearTimeout(id),
    clearInterval: (id) => clearInterval(id),
  };
}
