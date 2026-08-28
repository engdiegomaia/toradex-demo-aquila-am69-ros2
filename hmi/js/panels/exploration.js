/**
 * A decisão de exploração do painel de navegação, sem DOM e sem canvas.
 *
 * Mora fora de `nav-panel.js` por um motivo de teste, não de organização: o
 * painel só existe depois de um `canvas.getContext('2d')`, e o bundle não tem
 * jsdom por decisão de projeto (plano-cockpit-web.md, Decisão 5). Montar o
 * painel inteiro para perguntar "este clique deveria virar meta?" exigiria
 * dublar um contexto 2D inteiro, e o teste passaria a medir o dublê.
 *
 * O que está aqui são as três perguntas que o operador faz e que uma resposta
 * errada custa caro:
 *
 *   a busca está correndo?   -> se estiver, clique no mapa NÃO vira meta manual
 *   o que mostrar no HUD?    -> estado, tempo, fronteiras, marcador, falha
 *   a saída foi confirmada?  -> e isso vem do validador, nunca do explorador
 *
 * O estado de busca chega por um tópico TRANSIENT_LOCAL: depois de uma queda do
 * rosbridge, a última mensagem é reentregue e a tela se reconstrói sozinha. Por
 * isso este store não tem "esquecer": ele é uma função do último status visto.
 */

/**
 * Estados em que o explorador está no comando do robô.
 *
 * `starting` é do COCKPIT, não do `maze_explorer`: cobre a janela entre o
 * clique em "iniciar busca" e o primeiro status vindo do Aquila. Sem ele essa
 * janela conta como "não há busca", e um clique no mapa vira meta manual por
 * cima de uma busca que o Aquila já aceitou. É a mesma família de defeito que
 * o resto deste arquivo persegue: o cockpit acreditando numa coisa enquanto o
 * módulo faz outra.
 */
export const BUSY_STATES = Object.freeze([
  'starting', 'waiting_map', 'selecting', 'navigating', 'homing_exit',
]);

/** Estados em que a busca terminou, e o HUD ainda deve dizer como. */
export const TERMINAL_STATES = Object.freeze([
  'completed', 'failed', 'cancelled',
]);

/**
 * Lê o JSON publicado por `maze_explorer`.
 *
 * Um payload inválido NÃO pode virar `null` silencioso: o painel voltaria a
 * aceitar cliques manuais no meio de uma busca que continua correndo no Aquila.
 *
 * O `invalid: true` existe porque `state: 'failed'` sozinho não basta. `failed`
 * é TERMINAL, e terminal LIBERA a meta manual -- que é exatamente o que não se
 * pode fazer quando a única coisa que se sabe é que o canal ficou ilegível. Um
 * JSON quebrado não é notícia sobre o robô; é notícia sobre o enlace. Quem
 * decide o que fazer com isso é o store, em `apply`.
 */
export function parseExplorationStatus(message) {
  const data = message?.data;
  if (typeof data !== 'string') {
    return { state: 'failed', message: 'estado de busca ausente', invalid: true };
  }
  try {
    const parsed = JSON.parse(data);
    if (!parsed || typeof parsed !== 'object' || Array.isArray(parsed)) {
      return { state: 'failed', message: 'estado de busca inválido', invalid: true };
    }
    return parsed;
  } catch {
    return { state: 'failed', message: 'estado de busca inválido', invalid: true };
  }
}

export function isExplorationActive(exploration) {
  return BUSY_STATES.includes(exploration?.state);
}

/**
 * As partes do HUD que a busca acrescenta, na ordem de leitura.
 *
 * `SAÍDA CONFIRMADA` vem de `/demo/maze/escaped`, publicado pelo validador de
 * ground truth do lado da simulação — nunca de `state === 'completed'`, que só
 * diz que o explorador chegou perto do marcador. Confundir os dois faria o
 * cockpit declarar sucesso sem o robô ter atravessado a abertura.
 */
export function explorationHudParts(exploration, mazeEscaped, linkError) {
  const parts = [];
  if (exploration) {
    if (Number.isFinite(exploration.elapsed_s)) {
      parts.push(`${Math.round(exploration.elapsed_s)} s`);
    }
    if (Number.isFinite(exploration.frontier_count)) {
      parts.push(`${exploration.frontier_count} fronteira(s)`);
    }
    if (exploration.marker_visible) parts.push('saída detectada');
    if (exploration.message) parts.push(exploration.message);
  }
  // Erro de comunicação é uma linha PRÓPRIA, ao lado do último estado válido, e
  // não um estado que substitui aquele. O operador precisa ver as duas coisas:
  // o que o robô estava fazendo, e que o cockpit parou de saber.
  if (linkError) parts.push(linkError);
  if (mazeEscaped) parts.push('SAÍDA CONFIRMADA');
  return parts;
}

/** Guarda o último status visto e responde o que o painel precisa desenhar. */
export function createExplorationStore() {
  let exploration = null;
  let mazeEscaped = false;
  let linkError = null;
  let pending = false;

  return {
    /**
     * Aplica uma mensagem de `/demo/exploration/status`.
     *
     * Payload ilegível NÃO derruba um estado ocupado. `navigating` seguido de
     * JSON quebrado continua bloqueado; `selecting` seguido de desconexão
     * também. O último estado válido é preservado e a falha de comunicação vira
     * um campo separado -- porque converter para `failed` liberaria a meta
     * manual em cima de uma busca que continua correndo no Aquila.
     *
     * A exceção é não haver estado válido nenhum ainda: aí o `failed` do parse
     * é a melhor informação disponível, e é melhor que uma tela muda.
     */
    apply(message) {
      const parsed = parseExplorationStatus(message);
      if (parsed.invalid) {
        linkError = parsed.message;
        if (exploration === null) exploration = parsed;
        return;
      }
      linkError = null;
      pending = false;
      exploration = parsed;
    },
    /**
     * Marca um comando de busca em voo, antes de qualquer resposta.
     *
     * `starting` cobre a janela entre o clique e o primeiro status do Aquila.
     * `pending` cobre a promessa do serviço, e é o que impede o duplo clique de
     * virar duas chamadas.
     */
    beginStart() {
      pending = true;
      linkError = null;
      exploration = { state: 'starting', message: 'iniciando busca' };
    },
    /** Um cancelamento em voo: não muda o estado, só trava a porta. */
    beginCancel() {
      pending = true;
    },
    /**
     * O serviço recusou o start, ou a chamada explodiu.
     *
     * Desfaz o `starting` -- que é um estado que só o cockpit inventou -- para
     * que o painel não fique travado num bloqueio sem busca do outro lado. Se
     * um status real já tiver chegado nesse meio tempo, ele manda: a recusa
     * vira só mensagem, e o bloqueio continua com quem tem autoridade.
     */
    refuseStart(text) {
      pending = false;
      const message = text ?? 'comando de busca recusado';
      exploration = exploration?.state === 'starting'
        ? { state: 'failed', message }
        : { ...(exploration ?? {}), message };
    },
    /** O serviço respondeu (bem ou mal); a promessa não trava mais nada. */
    endCommand() {
      pending = false;
    },
    /** Há comando em voo ou busca correndo? Se sim, nada de meta manual. */
    isBusy() {
      return pending || isExplorationActive(exploration);
    },
    linkError() {
      return linkError;
    },
    /** Aplica um objeto já pronto, como a resposta recusada de um serviço. */
    merge(patch) {
      exploration = { ...(exploration ?? {}), ...patch };
    },
    setEscaped(value) {
      mazeEscaped = value === true;
    },
    isActive() {
      return isExplorationActive(exploration);
    },
    /** A busca domina o HUD enquanto corre E depois que termina. */
    ownsHud() {
      return this.isActive() || TERMINAL_STATES.includes(exploration?.state);
    },
    label() {
      return `busca: ${exploration?.state ?? 'idle'}`;
    },
    hudParts() {
      return explorationHudParts(exploration, mazeEscaped, linkError);
    },
    escaped() {
      return mazeEscaped;
    },
    snapshot() {
      return exploration;
    },
  };
}
