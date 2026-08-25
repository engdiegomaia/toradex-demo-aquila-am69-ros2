/**
 * O botão `seguir robô`: quem escreve o estado dele.
 *
 * A regressão que este arquivo existe para pegar é de uma linha: pintar
 * `aria-pressed` a partir do próprio clique em vez do tópico que o nó publica.
 * Ela passa em qualquer teste de "o botão chama o serviço" e mente exatamente
 * nos casos que importam — página recarregada, dois cockpits abertos, ou alguém
 * que desligou o seguimento por linha de comando.
 */

import assert from 'node:assert/strict';
import { describe, it } from 'node:test';

import {
  FOLLOW_DEFAULT,
  FOLLOW_VIEW_SERVICE,
  FOLLOWING_TOPIC,
  createViewControls,
} from '../js/panels/view-controls.js';
import { fakeClient, fakeRoot, installWindowTimers } from './helpers/fake-dom.js';

installWindowTimers();

const PAD = '[data-role="view-pad"]';
const FOLLOW = '[data-role="view-follow"]';

function mount(options = {}) {
  const root = fakeRoot([PAD, FOLLOW, '[data-role="view-reset"]']);
  const client = fakeClient(options);
  const controls = createViewControls({ root, client, onNotice: () => {} });
  return { root, client, controls, button: root.get(FOLLOW) };
}

describe('createViewControls: seguir o robô', () => {
  it('nasce pressionado, igual ao default do nó', () => {
    // Divergir daqui faz o primeiro clique mandar o valor que já vale: nada
    // acontece na tela e o botão parece quebrado.
    const { button, controls } = mount();
    assert.equal(button.getAttribute('aria-pressed'), String(FOLLOW_DEFAULT));
    assert.equal(controls.isFollowing(), FOLLOW_DEFAULT);
  });

  it('assina o tópico de estado publicado pelo nó', () => {
    const { client } = mount();
    const topics = client.subscriptions.map((entry) => entry.topic);
    assert.ok(topics.includes(FOLLOWING_TOPIC));
  });

  it('o clique pede pelo serviço, com o valor invertido', async () => {
    const { client, button } = mount();
    await Promise.all(button.emit('click'));

    assert.deepEqual(client.calls, [
      { service: FOLLOW_VIEW_SERVICE, args: { data: !FOLLOW_DEFAULT } },
    ]);
  });

  it('o clique NÃO pinta o botão — só o tópico pinta', async () => {
    const { client, button, controls } = mount();

    await Promise.all(button.emit('click'));
    assert.equal(
      button.getAttribute('aria-pressed'),
      String(FOLLOW_DEFAULT),
      'o botão mudou antes de o nó confirmar',
    );

    client.deliver(FOLLOWING_TOPIC, { data: false });
    assert.equal(button.getAttribute('aria-pressed'), 'false');
    assert.equal(controls.isFollowing(), false);
  });

  it('segue o nó mesmo quando ninguém clicou nesta tela', () => {
    // O caso do segundo cockpit aberto, e o do `ros2 service call` na bancada.
    const { client, button } = mount();
    client.deliver(FOLLOWING_TOPIC, { data: false });
    assert.equal(button.getAttribute('aria-pressed'), 'false');
  });

  it('uma resposta sem `data` lê como desligado, não como undefined', () => {
    // std_msgs/Bool com false vem do rosbridge como `{data: false}`, mas um
    // campo ausente não pode virar a string "undefined" num atributo ARIA.
    const { client, button } = mount();
    client.deliver(FOLLOWING_TOPIC, {});
    assert.equal(button.getAttribute('aria-pressed'), 'false');
  });

  it('destroy corta a inscrição de estado', () => {
    const { client, controls } = mount();
    controls.destroy();
    const following = client.subscriptions.find(
      (entry) => entry.topic === FOLLOWING_TOPIC,
    );
    assert.equal(following.active, false);
  });
});
