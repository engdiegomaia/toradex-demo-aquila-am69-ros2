/**
 * Os comandos de câmera que saem do cockpit.
 *
 * O nó do simulador satura e integra; daqui só sai um delta. O que pode dar
 * errado deste lado é a FORMA da mensagem — um campo faltando num TwistStamped
 * não é erro em lugar nenhum da cadeia, só uma câmera que não se mexe.
 */

import assert from 'node:assert/strict';
import { describe, it } from 'node:test';

import { VIEW_STEPS, viewCommand } from '../js/panels/view-controls.js';

describe('viewCommand', () => {
  it('preenche os seis campos do twist, não só o que muda', () => {
    // O rosbridge não completa campos aninhados ausentes: um linear sem `z`
    // chega ao nó como uma mensagem incompleta, sem erro em lugar nenhum.
    const message = viewCommand('orbit-left', 'scene_iso');
    assert.deepEqual(Object.keys(message.twist.linear).sort(), ['x', 'y', 'z']);
    assert.deepEqual(Object.keys(message.twist.angular).sort(), ['x', 'y', 'z']);
    for (const axis of ['x', 'y', 'z']) {
      assert.equal(typeof message.twist.linear[axis], 'number');
      assert.equal(typeof message.twist.angular[axis], 'number');
    }
  });

  it('o frame_id escolhe a câmera', () => {
    assert.equal(viewCommand('zoom-in', 'scene_top').header.frame_id, 'scene_top');
  });

  it('carrega um stamp, porque o tipo é TwistStamped', () => {
    const { stamp } = viewCommand('zoom-in', 'scene_iso').header;
    assert.deepEqual(stamp, { sec: 0, nanosec: 0 });
  });

  it('aproximar reduz a distância ao alvo', () => {
    // linear.x é variação de distância: o sinal invertido aqui afastaria a
    // câmera no botão "+", que é o tipo de erro que ninguém lê no código.
    assert.ok(viewCommand('zoom-in', 'scene_iso').twist.linear.x < 0);
    assert.ok(viewCommand('zoom-out', 'scene_iso').twist.linear.x > 0);
  });

  it('comando desconhecido devolve null em vez de uma mensagem vazia', () => {
    // Uma mensagem de deltas zerados seria aceita pelo nó e não faria nada.
    assert.equal(viewCommand('nao-existe', 'scene_iso'), null);
  });

  it('cada par de botões é simétrico', () => {
    const pairs = [
      ['orbit-left', 'orbit-right'],
      ['pitch-up', 'pitch-down'],
      ['zoom-in', 'zoom-out'],
      ['pan-left', 'pan-right'],
      ['pan-forward', 'pan-back'],
    ];
    for (const [a, b] of pairs) {
      const ma = viewCommand(a, 'scene_iso').twist;
      const mb = viewCommand(b, 'scene_iso').twist;
      for (const kind of ['linear', 'angular']) {
        for (const axis of ['x', 'y', 'z']) {
          // `+ 0` normaliza -0: assert.equal distingue 0 de -0, e os eixos
          // não usados de um par simétrico caem justamente nesse caso.
          assert.equal(
            ma[kind][axis] + 0,
            -mb[kind][axis] + 0,
            `${a}/${b} divergem em ${kind}.${axis}`,
          );
        }
      }
    }
  });

  it('todo botão do pad tem passo definido', () => {
    // O pad é montado a partir do HTML; um data-command sem entrada aqui seria
    // um botão que não faz nada e não reclama.
    assert.equal(Object.keys(VIEW_STEPS).length, 10);
  });
});
