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
 *
 * SEGUIR O ROBÔ
 *
 * O botão `seguir robô` é um SetBool, e vale para as duas vistas de uma vez —
 * o alvo da órbita é a pose do robô, e não há versão disso que faça sentido para
 * uma câmera só. Também não é o navegador que segue: quem lê /demo/odom e
 * reescreve a pose é o mesmo nó do simulador, pela mesma razão de sempre (o
 * cockpit não conhece geometria).
 *
 * O estado do botão vem de /demo/cockpit/scene/following, publicado pelo nó, e
 * NÃO do próprio clique. Mesmo raciocínio do rótulo de simulação em
 * sim-controls.js: recarregar a página, abrir o cockpit numa segunda tela, ou
 * abri-lo depois de alguém ter desligado o seguimento por linha de comando são
 * três casos em que o clique local não sabe a resposta. O tópico é latched, então
 * uma aba nova recebe o valor sem esperar a próxima mudança.
 */

import { ConnectionState } from '../ros/rosbridge-client.js';

export const CMD_VIEW_TOPIC = '/demo/cockpit/scene/cmd_view';
export const CMD_VIEW_TYPE = 'geometry_msgs/msg/TwistStamped';

/** std_srvs/Trigger, servido pelo scene_view_controller. */
export const RESET_VIEW_SERVICE = '/demo/cockpit/scene/reset_view';

/** std_srvs/SetBool, mesmo nó. Liga e desliga o seguimento das duas vistas. */
export const FOLLOW_VIEW_SERVICE = '/demo/cockpit/scene/follow';

/** Estado do seguimento, publicado pelo nó. Latched — ver o cabeçalho. */
export const FOLLOWING_TOPIC = '/demo/cockpit/scene/following';
export const FOLLOWING_TYPE = 'std_msgs/msg/Bool';

/**
 * Enquanto o tópico não chega, o botão assume o default do nó (parâmetro
 * `follow`, true em scene_cameras.launch.py). Assumir `false` aqui pintaria um
 * botão desligado sobre uma câmera que já está seguindo, e o primeiro clique
 * mandaria o valor que já vale — sem efeito visível.
 */
export const FOLLOW_DEFAULT = true;

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
  const followButton = root.querySelector('[data-role="view-follow"]');
  let active = camera;
  let following = FOLLOW_DEFAULT;

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

  const paintFollow = () => {
    if (followButton) followButton.setAttribute('aria-pressed', String(following));
  };

  // Quem escreve `following` é o nó, por este tópico. O clique abaixo só pede.
  const offFollowing = client.subscribe(
    FOLLOWING_TOPIC,
    FOLLOWING_TYPE,
    (message) => {
      following = Boolean(message?.data);
      paintFollow();
    },
  );

  followButton?.addEventListener('click', async () => {
    const wanted = !following;
    try {
      const result = await client.callService(FOLLOW_VIEW_SERVICE, { data: wanted });
      // success=false é o caminho útil aqui: o nó responde assim quando não
      // conseguiu escrever a pose. Sem esta ramificação o botão fica silencioso
      // justamente quando tem algo a dizer.
      if (result?.success === false) {
        onNotice?.(`o simulador recusou seguir=${wanted}: ${result.message ?? ''}`);
      }
    } catch (error) {
      onNotice?.(`falha ao alternar o seguimento da câmera: ${error.message}`);
    }
    // Nada de `following = wanted` aqui: o valor pintado é o que o nó publicar.
  });

  paintFollow();

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

    /** Exposto para teste: o botão pintado tem de refletir o nó, não o clique. */
    isFollowing() {
      return following;
    },

    destroy() {
      stopHold();
      unregister();
      offFollowing();
    },
  };
}
