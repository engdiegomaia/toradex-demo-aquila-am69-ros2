/**
 * O rótulo de estado da simulação.
 *
 * O que está sob teste é a única decisão real deste painel: distinguir
 * "pausado" de "sem simulador". Os dois parecem iguais de fora — nada acontece
 * na tela — e confundi-los faz o operador clicar em play num container morto.
 */

import assert from 'node:assert/strict';
import { describe, it } from 'node:test';

import {
  clockSeconds,
  createClockWatch,
  OFFLINE_AFTER_MS,
  SimState,
  STALL_AFTER_MS,
} from '../js/panels/sim-controls.js';

describe('clockSeconds', () => {
  it('junta sec e nanosec em segundos', () => {
    assert.equal(clockSeconds({ clock: { sec: 12, nanosec: 500_000_000 } }), 12.5);
  });

  it('trata mensagem vazia como zero em vez de NaN', () => {
    // NaN nunca é !== NaN, então uma mensagem malformada congelaria o relógio
    // em "mudou agora" para sempre e a simulação pareceria eternamente viva.
    assert.equal(clockSeconds({}), 0);
    assert.equal(clockSeconds(undefined), 0);
  });
});

describe('createClockWatch', () => {
  it('sem nenhuma amostra, não há simulador', () => {
    assert.equal(createClockWatch().state(1000), SimState.OFFLINE);
  });

  it('relógio avançando é simulação rodando', () => {
    const watch = createClockWatch();
    watch.sample(1.0, 1000);
    watch.sample(1.4, 1400);
    assert.equal(watch.state(1500), SimState.RUNNING);
  });

  it('amostras chegando com o MESMO tempo simulado é pausa', () => {
    const watch = createClockWatch();
    watch.sample(1.0, 1000);
    watch.sample(1.0, 1400);
    watch.sample(1.0, 1800);
    // A ponte está viva (as amostras chegam) mas o mundo não anda.
    assert.equal(watch.state(1000 + STALL_AFTER_MS + 1), SimState.PAUSED);
  });

  it('parar de receber amostras é ausência de simulador, não pausa', () => {
    const watch = createClockWatch();
    watch.sample(1.0, 1000);
    watch.sample(1.4, 1400);
    assert.equal(watch.state(1400 + OFFLINE_AFTER_MS + 1), SimState.OFFLINE);
  });

  it('a ausência vence a pausa quando as duas condições valem', () => {
    // Um container morto satisfaz as duas: o tempo parou E as amostras
    // pararam. O rótulo tem de ser o que manda o operador olhar o container.
    const watch = createClockWatch();
    watch.sample(1.0, 1000);
    assert.equal(watch.state(1000 + OFFLINE_AFTER_MS + 1), SimState.OFFLINE);
  });

  it('reset volta ao desconhecido', () => {
    const watch = createClockWatch();
    watch.sample(1.0, 1000);
    watch.reset();
    assert.equal(watch.state(1050), SimState.OFFLINE);
  });

  it('a primeira amostra depois de um reset não conta como avanço', () => {
    // Sem isto, reconectar com a simulação pausada mostraria "rodando" por
    // STALL_AFTER_MS, que é exatamente o instante em que alguém decide clicar.
    const watch = createClockWatch();
    watch.sample(7.0, 1000);
    watch.reset();
    watch.sample(7.0, 5000);
    watch.sample(7.0, 5400);
    assert.equal(watch.state(5000 + STALL_AFTER_MS + 1), SimState.PAUSED);
  });

  it('o limiar de pausa é maior que o intervalo entre amostras', () => {
    // Guarda de regressão sobre a constante, não sobre o código: com amostras a
    // cada 400 ms um limiar apertado piscaria "pausado" a cada jitter da rede.
    assert.ok(STALL_AFTER_MS > 400 * 2);
    assert.ok(OFFLINE_AFTER_MS > STALL_AFTER_MS);
  });
});

