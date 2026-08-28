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

/** Estados em que o explorador está no comando do robô. */
export const BUSY_STATES = Object.freeze([
  'waiting_map', 'selecting', 'navigating', 'homing_exit',
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
 * Vira um estado de falha visível.
 */
export function parseExplorationStatus(message) {
  const data = message?.data;
  if (typeof data !== 'string') {
    return { state: 'failed', message: 'estado de busca ausente' };
  }
  try {
    const parsed = JSON.parse(data);
    if (!parsed || typeof parsed !== 'object' || Array.isArray(parsed)) {
      return { state: 'failed', message: 'estado de busca inválido' };
    }
    return parsed;
  } catch {
    return { state: 'failed', message: 'estado de busca inválido' };
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
export function explorationHudParts(exploration, mazeEscaped) {
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
  if (mazeEscaped) parts.push('SAÍDA CONFIRMADA');
  return parts;
}

/** Guarda o último status visto e responde o que o painel precisa desenhar. */
export function createExplorationStore() {
  let exploration = null;
  let mazeEscaped = false;

  return {
    /** Aplica uma mensagem de `/demo/exploration/status`. */
    apply(message) {
      exploration = parseExplorationStatus(message);
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
      return explorationHudParts(exploration, mazeEscaped);
    },
    escaped() {
      return mazeEscaped;
    },
    snapshot() {
      return exploration;
    },
  };
}
