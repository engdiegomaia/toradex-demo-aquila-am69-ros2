/**
 * A busca autônoma vista do cockpit.
 *
 * As regressões que este arquivo existe para pegar são todas da mesma família:
 * o cockpit acreditando numa coisa enquanto o Aquila faz outra.
 *
 *   - status ilegível tratado como "não há busca", e o clique no mapa volta a
 *     mandar meta manual por cima de um explorador que continua correndo;
 *   - `state: 'completed'` pintado como saída confirmada, quando quem confirma
 *     é o validador de ground truth e só ele;
 *   - o HUD perdido depois de uma queda do rosbridge, apesar de o tópico ser
 *     TRANSIENT_LOCAL e reentregar a última mensagem sozinho.
 */

import assert from 'node:assert/strict';
import { describe, it } from 'node:test';

import {
  BUSY_STATES,
  createExplorationStore,
  explorationHudParts,
  isExplorationActive,
  parseExplorationStatus,
  TERMINAL_STATES,
} from '../js/panels/exploration.js';

/** O mesmo payload que `maze_explorer._publish_status()` monta. */
function status(fields = {}) {
  return {
    data: JSON.stringify({
      state: 'navigating',
      elapsed_s: 12.4,
      frontier_count: 3,
      goal: null,
      blacklisted: 0,
      marker_visible: false,
      message: '',
      ...fields,
    }),
  };
}

describe('parseExplorationStatus', () => {
  it('lê o JSON que o explorador publica', () => {
    const parsed = parseExplorationStatus(status({ frontier_count: 7 }));
    assert.equal(parsed.state, 'navigating');
    assert.equal(parsed.frontier_count, 7);
  });

  it('JSON quebrado vira falha VISÍVEL, não ausência de busca', () => {
    // Um `null` aqui devolveria o clique manual ao operador no meio de uma
    // busca que segue correndo no módulo: duas fontes de meta no mesmo Nav2.
    const parsed = parseExplorationStatus({ data: '{nao é json' });
    assert.equal(parsed.state, 'failed');
    assert.ok(parsed.message);
  });

  it('mensagem sem campo `data` também vira falha', () => {
    assert.equal(parseExplorationStatus(undefined).state, 'failed');
    assert.equal(parseExplorationStatus({}).state, 'failed');
  });

  it('JSON válido que não é objeto não passa por status', () => {
    // `JSON.parse('4')` e `JSON.parse('[]')` não lançam. Aceitá-los faria
    // `state` virar undefined e a busca sumir da tela sem erro nenhum.
    assert.equal(parseExplorationStatus({ data: '4' }).state, 'failed');
    assert.equal(parseExplorationStatus({ data: '[]' }).state, 'failed');
    assert.equal(parseExplorationStatus({ data: 'null' }).state, 'failed');
  });
});

describe('isExplorationActive', () => {
  it('todo estado ocupado do nó bloqueia a meta manual', () => {
    for (const state of BUSY_STATES) {
      assert.equal(isExplorationActive({ state }), true, state);
    }
  });

  it('estado terminal libera o operador de volta', () => {
    for (const state of TERMINAL_STATES) {
      assert.equal(isExplorationActive({ state }), false, state);
    }
  });

  it('sem status nenhum, o cockpit é do operador', () => {
    assert.equal(isExplorationActive(null), false);
    assert.equal(isExplorationActive({ state: 'idle' }), false);
  });

  it('os estados ocupados são exatamente os do nó ROS', () => {
    // Divergir desta lista é a falha silenciosa da fiação: um estado novo no
    // maze_explorer que o cockpit não reconhece devolve o clique manual no
    // meio da busca. A lista vive em dois idiomas e precisa ser comparada.
    assert.deepEqual([...BUSY_STATES].sort(),
      ['homing_exit', 'navigating', 'selecting', 'waiting_map']);
  });
});

describe('explorationHudParts', () => {
  it('mostra tempo, fronteiras e mensagem na ordem de leitura', () => {
    const parsed = parseExplorationStatus(
      status({ elapsed_s: 41.6, frontier_count: 2, message: 'navegando' }));
    assert.deepEqual(explorationHudParts(parsed, false),
      ['42 s', '2 fronteira(s)', 'navegando']);
  });

  it('anuncia o marcador só quando a percepção o vê', () => {
    const seen = parseExplorationStatus(status({ marker_visible: true }));
    assert.ok(explorationHudParts(seen, false).includes('saída detectada'));
    const unseen = parseExplorationStatus(status({ marker_visible: false }));
    assert.ok(!explorationHudParts(unseen, false).includes('saída detectada'));
  });

  it('`completed` NÃO é saída confirmada', () => {
    // `completed` diz que o explorador chegou perto do painel magenta. Quem
    // confirma o cruzamento da abertura é /demo/maze/escaped, do validador de
    // ground truth, e ele é o único que pode escrever esse rótulo.
    const parsed = parseExplorationStatus(status({ state: 'completed' }));
    assert.ok(!explorationHudParts(parsed, false).includes('SAÍDA CONFIRMADA'));
    assert.ok(explorationHudParts(parsed, true).includes('SAÍDA CONFIRMADA'));
  });

  it('a saída confirmada sobrevive sem status de busca', () => {
    assert.deepEqual(explorationHudParts(null, true), ['SAÍDA CONFIRMADA']);
  });

  it('campos ausentes não viram NaN na tela', () => {
    const parsed = parseExplorationStatus({ data: '{"state":"selecting"}' });
    assert.deepEqual(explorationHudParts(parsed, false), []);
  });
});

describe('createExplorationStore', () => {
  it('nasce ocioso: o mapa é do operador', () => {
    const store = createExplorationStore();
    assert.equal(store.isActive(), false);
    assert.equal(store.ownsHud(), false);
    assert.equal(store.escaped(), false);
  });

  it('o HUD é da busca enquanto ela corre e depois que ela termina', () => {
    const store = createExplorationStore();
    store.apply(status({ state: 'navigating' }));
    assert.equal(store.ownsHud(), true);
    assert.equal(store.label(), 'busca: navigating');
    store.apply(status({ state: 'failed', message: 'prazo total excedido' }));
    assert.equal(store.isActive(), false);
    assert.equal(store.ownsHud(), true);
    assert.ok(store.hudParts().includes('prazo total excedido'));
  });

  it('reconexão do rosbridge reconstrói a tela pela última mensagem', () => {
    // O tópico é TRANSIENT_LOCAL: depois da queda, o assinante novo recebe o
    // último status. O store não pode ter memória própria a reconciliar --
    // aplicar a reentrega tem de bastar, e a busca no Aquila não é tocada.
    const before = createExplorationStore();
    before.apply(status({ state: 'homing_exit', elapsed_s: 88, message: 'x' }));

    const afterReconnect = createExplorationStore();
    afterReconnect.apply(status({ state: 'homing_exit', elapsed_s: 88, message: 'x' }));

    assert.deepEqual(afterReconnect.snapshot(), before.snapshot());
    assert.deepEqual(afterReconnect.hudParts(), before.hudParts());
    assert.equal(afterReconnect.isActive(), true);
  });

  it('recusa de serviço aparece sem apagar o estado corrente', () => {
    const store = createExplorationStore();
    store.apply(status({ state: 'navigating', frontier_count: 5 }));
    store.merge({ message: 'busca ja esta em andamento' });
    assert.equal(store.snapshot().state, 'navigating');
    assert.equal(store.snapshot().frontier_count, 5);
    assert.ok(store.hudParts().includes('busca ja esta em andamento'));
  });

  it('só `true` liga a saída confirmada', () => {
    const store = createExplorationStore();
    for (const value of [undefined, null, 0, '', 'true']) {
      store.setEscaped(value);
      assert.equal(store.escaped(), false, String(value));
    }
    store.setEscaped(true);
    assert.equal(store.escaped(), true);
  });

  it('a busca não tem nenhum conceito de zoom', () => {
    // O enquadramento é do operador. Este teste é a guarda contra alguém
    // resolver "centralizar no robô ao iniciar a busca": o zoom mora em
    // `state` no painel, e nada aqui pode alcançá-lo.
    const store = createExplorationStore();
    store.apply(status({ state: 'navigating' }));
    const surface = Object.keys(store).join(' ');
    assert.ok(!/zoom|center|centr|view/i.test(surface));
    assert.ok(!/zoom/i.test(JSON.stringify(store.snapshot())));
  });
});
