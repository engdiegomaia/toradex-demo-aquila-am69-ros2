/**
 * Girar, mover e aproximar a câmera de cena.
 *
 * O NAVEGADOR NÃO SABE ONDE A CÂMERA ESTÁ, E ISSO É O DESENHO
 *
 * Cada botão publica um DELTA em /demo/cockpit/scene/cmd_view. Quem guarda a
 * órbita (azimute, elevação, distância, alvo), satura os limites e escreve a
 * pose no Gazebo é o `scene_view_controller`, do lado do simulador.
 *
 * A alternativa — o cliente calcular a pose e mandar pronta — quebra de três
 * jeitos que já estariam aqui: dois cockpits abertos brigam pela pose, um F5
 * zera o enquadramento, e a matemática de órbita (o quaternion de roll zero em
 * particular) passaria a existir em duas linguagens que precisam concordar.
 * Deste lado só existe "gire um pouco para a esquerda".
 *
 * O `header.frame_id` escolhe a câmera. Ele acompanha o botão iso/topo do
 * cabeçalho do painel: comandar a câmera que não está na tela é o tipo de erro
 * que parece "os botões não funcionam".
 */

import { ConnectionState } from '../ros/rosbridge-client.js';

export const CMD_VIEW_TOPIC = '/demo/cockpit/scene/cmd_view';
export const CMD_VIEW_TYPE = 'geometry_msgs/msg/TwistStamped';

/** std_srvs/Trigger, servido pelo scene_view_controller. */
export const RESET_VIEW_SERVICE = '/demo/cockpit/scene/reset_view';

/**
 * Passo de cada comando, na unidade que o controlador integra.
 *
 * Medidos na cena, não escolhidos: com passo de órbita a 0,30 rad uma volta
 * completa leva ~21 cliques, o que dá para enquadrar sem contar cliques e sem
 * passar do ponto. O dolly é negativo para APROXIMAR porque `linear.x` é
 * variação de distância ao alvo.
 */
export const VIEW_STEPS = Object.freeze({
  'orbit-left': { angular: { z: 0.3 } },
  'orbit-right': { angular: { z: -0.3 } },
  'pitch-up': { angular: { y: 0.18 } },
  'pitch-down': { angular: { y: -0.18 } },
  'zoom-in': { linear: { x: -1.2 } },
  'zoom-out': { linear: { x: 1.2 } },
  'pan-left': { linear: { y: 0.8 } },
  'pan-right': { linear: { y: -0.8 } },
  'pan-forward': { linear: { z: 0.8 } },
  'pan-back': { linear: { z: -0.8 } },
});

/** Segurar o botão repete: enquadrar de longe custaria dezenas de cliques. */
export const HOLD_DELAY_MS = 340;
export const HOLD_INTERVAL_MS = 130;

/** Monta o TwistStamped completo — o rosbridge não preenche campos aninhados. */
export function viewCommand(command, camera) {
  const step = VIEW_STEPS[command];
  if (!step) return null;
  return {
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: camera },
    twist: {
      linear: { x: 0, y: 0, z: 0, ...step.linear },
      angular: { x: 0, y: 0, z: 0, ...step.angular },
    },
  };
}

export function createViewControls({ root, client, camera = 'scene_iso', onNotice }) {
  const pad = root.querySelector('[data-role="view-pad"]');
  const resetButton = root.querySelector('[data-role="view-reset"]');
  let active = camera;

  client.advertise(CMD_VIEW_TOPIC, CMD_VIEW_TYPE);

  const publish = (command) => {
    const message = viewCommand(command, active);
    if (message) client.publish(CMD_VIEW_TOPIC, message);
  };

  // --- segurar para repetir -------------------------------------------------
  // Um timer só, compartilhado por todos os botões: dois botões pressionados ao
  // mesmo tempo seria um comando ambíguo, não dois comandos.
  let holdTimer = null;
  let holdRepeat = null;

  const stopHold = () => {
    window.clearTimeout(holdTimer);
    window.clearInterval(holdRepeat);
    holdTimer = null;
    holdRepeat = null;
  };

  for (const button of pad.querySelectorAll('[data-role="view"]')) {
    const command = button.dataset.command;

    button.addEventListener('pointerdown', (event) => {
      // Sem isto o botão continua repetindo depois que o ponteiro sai dele, e a
      // câmera gira sozinha até alguém clicar em outro lugar.
      button.setPointerCapture?.(event.pointerId);
      publish(command);
      stopHold();
      holdTimer = window.setTimeout(() => {
        holdRepeat = window.setInterval(() => publish(command), HOLD_INTERVAL_MS);
      }, HOLD_DELAY_MS);
    });
    button.addEventListener('pointerup', stopHold);
    button.addEventListener('pointercancel', stopHold);
    button.addEventListener('lostpointercapture', stopHold);

    // Teclado não gera pointerdown. Sem isto o pad é inalcançável por teclado,
    // que é o modo como o kiosk sem mouse do M3 vai ser operado.
    button.addEventListener('keydown', (event) => {
      if (event.key === 'Enter' || event.key === ' ') publish(command);
    });
  }

  resetButton?.addEventListener('click', async () => {
    try {
      await client.callService(RESET_VIEW_SERVICE, {});
    } catch (error) {
      onNotice?.(`falha ao recentrar a câmera: ${error.message}`);
    }
  });

  // O pad se apaga com o link. Um botão que publica no vazio não deve parecer
  // vivo — e este é o único painel cujo efeito não aparece em lugar nenhum da
  // tela quando falha, porque o resultado dele É a imagem que continua igual.
  const unregister = client.onStateChange((state) => {
    const up = state === ConnectionState.CONNECTED;
    pad.dataset.enabled = String(up);
    if (!up) stopHold();
  });

  return {
    /** Chamado pelo botão iso/topo: os comandos seguem a imagem visível. */
    setCamera(name) {
      active = name;
    },

    destroy() {
      stopHold();
      unregister();
    },
  };
}
