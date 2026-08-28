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

  it('os estados ocupados do nó ROS estão todos aqui', () => {
    // Divergir desta lista é a falha silenciosa da fiação: um estado novo no
    // maze_explorer que o cockpit não reconhece devolve o clique manual no
    // meio da busca. A lista vive em dois idiomas e precisa ser comparada.
    //
    // `starting` fica FORA da comparação de propósito: é o único estado desta
    // lista que o nó ROS não conhece. Ele cobre a janela entre o clique e o
    // primeiro status, que só existe do lado do cockpit. Um teste em
    // tests/test_maze_exploration_contract.py trava a outra metade: `starting`
    // não pode aparecer no vocabulário do maze_explorer.
    const fromRos = [...BUSY_STATES].filter((name) => name !== 'starting');
    assert.deepEqual(fromRos.sort(),
      ['homing_exit', 'navigating', 'selecting', 'waiting_map']);
    assert.ok(BUSY_STATES.includes('starting'));
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

describe('fail-safe entre o clique e o primeiro status', () => {
  it('a promessa do serviço em voo já bloqueia a meta manual', () => {
    // Entre o clique e a resposta do serviço, o Aquila pode já ter aceitado a
    // busca. Um clique no mapa nessa janela mandaria meta manual por cima dela.
    const store = createExplorationStore();
    assert.equal(store.isBusy(), false);

    store.beginStart();

    assert.equal(store.isBusy(), true);
    assert.equal(store.snapshot().state, 'starting');
    assert.ok(store.hudParts().includes('iniciando busca'));
  });

  it('`starting` esconde o início e mostra o cancelamento', () => {
    // São as mesmas duas perguntas que o painel faz para decidir os botões.
    const store = createExplorationStore();
    store.beginStart();

    assert.equal(store.isBusy(), true);
    assert.equal(store.ownsHud(), true);
  });

  it('o duplo clique não vira duas chamadas', () => {
    // O painel testa `isBusy()` ANTES de qualquer efeito colateral. Simular o
    // segundo clique é perguntar exatamente isso.
    const store = createExplorationStore();
    let calls = 0;
    const click = () => {
      if (store.isBusy()) return;
      store.beginStart();
      calls += 1;
    };

    click();
    click();
    click();

    assert.equal(calls, 1);
  });

  it('o primeiro status real substitui o `starting` local', () => {
    const store = createExplorationStore();
    store.beginStart();

    store.apply(status({ state: 'selecting' }));

    assert.equal(store.snapshot().state, 'selecting');
    assert.equal(store.isBusy(), true);
  });

  it('start recusado devolve o cockpit em vez de travar em `starting`', () => {
    // `starting` é um estado que só o cockpit inventou. Se o serviço recusa e
    // ninguém o desfaz, o mapa fica bloqueado sem busca do outro lado.
    const store = createExplorationStore();
    store.beginStart();

    store.refuseStart('busca já está em andamento');

    assert.equal(store.isBusy(), false);
    assert.ok(store.hudParts().includes('busca já está em andamento'));
  });
});

describe('status ilegível não devolve o mapa ao operador', () => {
  it('`navigating` seguido de JSON quebrado continua bloqueado', () => {
    // Converter para `failed` liberaria a meta manual em cima de uma busca que
    // continua correndo no Aquila. `failed` é terminal, e terminal libera.
    const store = createExplorationStore();
    store.apply(status({ state: 'navigating' }));

    store.apply({ data: '{nao é json' });

    assert.equal(store.isBusy(), true);
    assert.equal(store.snapshot().state, 'navigating');
  });

  it('`selecting` seguido de desconexão continua bloqueado', () => {
    const store = createExplorationStore();
    store.apply(status({ state: 'selecting' }));

    store.apply(undefined);

    assert.equal(store.isBusy(), true);
    assert.equal(store.snapshot().state, 'selecting');
  });

  it('a falha de comunicação aparece ao lado do último estado válido', () => {
    // O operador precisa ver as DUAS coisas: o que o robô estava fazendo, e
    // que o cockpit parou de saber.
    const store = createExplorationStore();
    store.apply(status({ state: 'navigating', message: 'navegando para fronteira' }));

    store.apply({ data: '[]' });

    const parts = store.hudParts();
    assert.ok(parts.includes('navegando para fronteira'));
    assert.ok(parts.includes('estado de busca inválido'));
    assert.equal(store.linkError(), 'estado de busca inválido');
  });

  it('um status válido depois limpa o erro de comunicação', () => {
    const store = createExplorationStore();
    store.apply(status({ state: 'navigating' }));
    store.apply({ data: 'null' });

    store.apply(status({ state: 'selecting' }));

    assert.equal(store.linkError(), null);
    assert.ok(!store.hudParts().includes('estado de busca inválido'));
  });

  it('sem nenhum estado válido ainda, o erro é o que há para mostrar', () => {
    // Aqui não há nada a preservar, e uma tela muda é pior que um erro visível.
    const store = createExplorationStore();

    store.apply({ data: '{nao é json' });

    assert.equal(store.snapshot().state, 'failed');
    assert.equal(store.isBusy(), false);
  });

  it('cancelamento explícito devolve o controle ao operador', () => {
    const store = createExplorationStore();
    store.apply(status({ state: 'navigating' }));
    store.beginCancel();
    assert.equal(store.isBusy(), true);

    store.endCommand();
    store.apply(status({ state: 'cancelled', message: 'busca cancelada pelo operador' }));

    assert.equal(store.isBusy(), false);
    assert.ok(store.hudParts().includes('busca cancelada pelo operador'));
  });
});
