/**
 * Simulação: play, pause e reset, a partir do cockpit.
 *
 * O QUE ISTO É E O QUE NÃO É
 *
 * O Gazebo roda na workstation x86 e vai continuar rodando lá. O AM69 expõe
 * apenas OpenGL ES 3.2 e Vulkan 1.2, o OGRE 2 do Gazebo precisa de OpenGL de
 * desktop, e a regra 1 do CLAUDE.md existe exatamente para impedir que alguém
 * tente. Estes botões controlam a simulação PELO cockpit — no M3 o cockpit é
 * servido pelo Aquila, então o clique sai do módulo — mas quem executa o
 * simulador segue sendo o host. O que atravessa é uma chamada de serviço no
 * grafo ROS, igual à meta do Nav2 hoje.
 *
 * POR QUE O ESTADO VEM DO /clock E NÃO DO BOTÃO
 *
 * O caminho óbvio seria pintar "pausado" quando o operador clica em pausar.
 * Isso mente em todos os casos que importam: o container `sim` caiu, a chamada
 * expirou, alguém pausou pela GUI do Gazebo, o mundo foi resetado por outra
 * pessoa. O relógio simulado é a única testemunha que sabe se o simulador está
 * de fato andando, então é ele que escreve o rótulo — mesmo quando discorda do
 * último clique.
 *
 * /clock publica a ~1 kHz e nada disso precisa dessa resolução: a inscrição é
 * estrangulada no servidor (throttle_rate), não filtrada no cliente, para que o
 * tráfego nem chegue ao navegador.
 */

/**
 * std_srvs/Trigger, um por ação, servidos pelo `sim_control_relay`.
 *
 * NÃO é `/demo/sim/control` (ros_gz_interfaces/srv/ControlWorld), e a tentativa
 * de chamá-lo direto daqui é o que motivou a fachada. O rosbridge monta o
 * pedido importando o pacote de interfaces dentro do próprio container:
 *
 *     call_service InvalidModuleException: Unable to import
 *     ros_gz_interfaces.srv from package ros_gz_interfaces
 *
 * O container do cockpit não tem esse pacote — e no modo `deploy`, onde não
 * existe Gazebo nenhum, não faria sentido ter. A fronteira do navegador fala
 * std_srvs, que é núcleo do ROS; a tradução acontece do lado do simulador.
 */
export const SIM_SERVICES = Object.freeze({
  play: '/demo/sim/play',
  pause: '/demo/sim/pause',
  reset: '/demo/sim/reset',
});

export const CLOCK_TOPIC = '/clock';
export const CLOCK_TYPE = 'rosgraph_msgs/msg/Clock';

/** ~2,5 Hz: suficiente para "andou ou não andou", irrelevante no transporte. */
export const CLOCK_THROTTLE_MS = 400;

/**
 * Sem avanço do relógio simulado por este tempo, a simulação está pausada.
 *
 * Precisa ser confortavelmente maior que CLOCK_THROTTLE_MS: com amostras a
 * cada 400 ms, um limiar apertado piscaria "pausado" a cada jitter da rede.
 */
export const STALL_AFTER_MS = 1100;

/** Sem NENHUMA amostra por este tempo, não há simulador do outro lado. */
export const OFFLINE_AFTER_MS = 3000;

/** Segundos que o botão de reset fica armado esperando a confirmação. */
export const RESET_ARM_MS = 4000;

export const SimState = Object.freeze({
  RUNNING: 'rodando',
  PAUSED: 'pausado',
  OFFLINE: 'sem simulador',
});

/**
 * Núcleo puro: recebe amostras do relógio simulado e responde em que estado a
 * simulação está. Separado do DOM porque é a única parte com lógica de verdade
 * e a única que dá para testar sem um navegador.
 */
export function createClockWatch() {
  let lastSeconds = null;
  let lastSampleAt = null;
  let lastChangeAt = null;

  return {
    /** @param {number} seconds tempo simulado @param {number} atMs relógio de parede */
    sample(seconds, atMs) {
      // Só o AVANÇO conta como sinal de vida. Uma amostra repetida prova que a
      // ponte está viva, não que o mundo está andando — e é justamente essa a
      // diferença entre "pausado" e "sem simulador".
      if (lastSeconds === null || seconds !== lastSeconds) {
        lastChangeAt = atMs;
      }
      lastSeconds = seconds;
      lastSampleAt = atMs;
    },

    /** Volta ao desconhecido: usado quando o WebSocket cai. */
    reset() {
      lastSeconds = null;
      lastSampleAt = null;
      lastChangeAt = null;
    },

    state(nowMs) {
      if (lastSampleAt === null) return SimState.OFFLINE;
      if (nowMs - lastSampleAt > OFFLINE_AFTER_MS) return SimState.OFFLINE;
      if (nowMs - lastChangeAt > STALL_AFTER_MS) return SimState.PAUSED;
      return SimState.RUNNING;
    },
  };
}

/** Segundos de um rosgraph_msgs/Clock, tolerando campos ausentes. */
export function clockSeconds(message) {
  const stamp = message?.clock ?? {};
  return (stamp.sec ?? 0) + (stamp.nanosec ?? 0) / 1e9;
}

export function createSimControls({ root, client, onNotice }) {
  const buttons = [...root.querySelectorAll('[data-role="sim"]')];
  const label = root.querySelector('[data-role="sim-state"]');
  const resetButton = buttons.find((button) => button.dataset.command === 'reset');
  const resetLabel = resetButton?.textContent ?? '';

  const watch = createClockWatch();
  let armedUntil = 0;

  const unsubscribe = client.subscribe(
    CLOCK_TOPIC,
    CLOCK_TYPE,
    (message) => watch.sample(clockSeconds(message), Date.now()),
    { throttleRate: CLOCK_THROTTLE_MS },
  );

  const disarm = () => {
    armedUntil = 0;
    if (!resetButton) return;
    resetButton.dataset.armed = 'false';
    resetButton.textContent = resetLabel;
  };

  async function send(command) {
    try {
      const result = await client.callService(SIM_SERVICES[command], {});
      // Trigger responde success=false quando o Gazebo recusou ou a ponte não
      // respondeu. Tratar isso como sucesso deixaria o botão silencioso na
      // única situação em que ele tem algo a dizer.
      if (result?.success === false) {
        onNotice?.(`o simulador recusou ${command}: ${result.message ?? ''}`);
      }
    } catch (error) {
      onNotice?.(`falha ao ${command} a simulação: ${error.message}`);
    }
  }

  for (const button of buttons) {
    button.addEventListener('click', () => {
      const command = button.dataset.command;

      // Reset cai no meio da demo e teleporta o robô para o ponto de
      // nascimento — o Nav2 perde a meta em curso e vê o robô saltar. Não é
      // destrutivo (o mundo e o relógio ficam), mas também não é algo que se
      // queira por clique acidental. Dois cliques.
      if (command === 'reset' && Date.now() > armedUntil) {
        armedUntil = Date.now() + RESET_ARM_MS;
        button.dataset.armed = 'true';
        button.textContent = 'confirmar';
        window.setTimeout(disarm, RESET_ARM_MS);
        return;
      }
      if (command === 'reset') disarm();

      send(command);
    });
  }

  return {
    tick() {
      const state = watch.state(Date.now());
      label.textContent = state;
      label.dataset.state = state;
    },

    onLinkDown() {
      watch.reset();
      disarm();
    },

    destroy() {
      unsubscribe();
    },
  };
}
