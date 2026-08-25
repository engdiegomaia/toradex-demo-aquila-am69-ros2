/**
 * As cores dos canvas, lidas do CSS.
 *
 * O mapa de navegação e as caixas de detecção não são desenhados por CSS, mas
 * respondem às MESMAS decisões de contraste que o resto da tela: quando o
 * cockpit trocou o fundo preto pelo branco da identidade Toradex, o ciano do
 * plano e o amarelo do laser sumiram junto. Manter esses valores duplicados em
 * JavaScript garante que a próxima troca de tema conserte quatro painéis e
 * esqueça dois.
 *
 * Então tokens.css continua sendo a fonte única, inclusive para o canvas: os
 * tokens --map-* são lidos uma vez, na montagem do painel. Uma vez basta —
 * o tema não muda em tempo de execução, e chamar getComputedStyle a cada
 * quadro custaria um recálculo de estilo por repintura.
 *
 * Os fallbacks existem porque um canvas sem cor desenha em preto sobre um mapa
 * quase branco, o que é indistinguível de "funcionou".
 */

export const MAP_PALETTE_FALLBACK = Object.freeze({
  plan: '#00508c',
  robot: '#96c837',
  scan: '#b34000',
  goal: '#7b2fbe',
  grid: 'rgba(22, 35, 46, 0.09)',
  label: '#ffffff',
});

const TOKENS = Object.freeze({
  plan: '--map-plan',
  robot: '--map-robot',
  scan: '--map-scan',
  goal: '--map-goal',
  grid: '--map-grid',
  label: '--bg-panel',
});

/** @param {Element} [element] elemento de onde herdar as variáveis. */
export function readMapPalette(element) {
  const target = element ?? document.documentElement;
  const computed = window.getComputedStyle(target);
  const resolved = {};
  for (const [key, token] of Object.entries(TOKENS)) {
    const value = computed.getPropertyValue(token).trim();
    resolved[key] = value || MAP_PALETTE_FALLBACK[key];
  }
  return Object.freeze(resolved);
}
