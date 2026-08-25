/**
 * Minimal DOM double for the panels' wiring.
 *
 * O bundle não tem etapa de build e não tem jsdom — o `package.json` existe
 * para o `node --test` e nada mais (plano-cockpit-web.md, Decisão 5). Então o
 * que este arquivo entrega é a superfície EXATA que os painéis usam:
 * querySelector por atributo, addEventListener, setAttribute/getAttribute,
 * dataset e textContent.
 *
 * Não é um DOM. É o suficiente para responder à única pergunta que a fiação de
 * um botão levanta e que um teste de função pura não alcança: quem escreveu
 * neste atributo, o clique ou o ROS?
 */

class FakeElement {
  constructor(selector, { text = '' } = {}) {
    /** O seletor por que este elemento é encontrado, ex. '[data-role="x"]'. */
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

  /** Dispara os handlers e devolve as promessas que eles retornarem. */
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
 * `callService` devolve o que o teste enfileirar, e registra a chamada: as duas
 * coisas que importam sobre um botão de serviço são o que ele mandou e o que ele
 * fez com a resposta.
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
    /** Entrega uma mensagem a quem se inscreveu naquele tópico. */
    deliver(topic, message) {
      for (const entry of subscriptions) {
        if (entry.active && entry.topic === topic) entry.handler(message);
      }
    },
  };
}

/**
 * `window.clearTimeout`/`clearInterval` para o Node.
 *
 * O `stopHold` dos painéis fala `window.*` porque é o que o navegador oferece, e
 * o `destroy()` passa por ele. Não vale reescrever o módulo para caber no teste:
 * o shim é de quatro linhas e o `node --test` não tem DOM por decisão de projeto
 * (bundle sem etapa de build — ver plano-cockpit-web.md, Decisão 5).
 */
export function installWindowTimers() {
  globalThis.window ??= {
    setTimeout: (...args) => setTimeout(...args),
    setInterval: (...args) => setInterval(...args),
    clearTimeout: (id) => clearTimeout(id),
    clearInterval: (id) => clearInterval(id),
  };
}
