/**
 * The canvas colours, read from CSS.
 *
 * The navigation map and the detection boxes are not drawn by CSS, but they
 * answer to the SAME contrast decisions as the rest of the screen: when the
 * cockpit swapped the black background for the white of the Toradex identity,
 * the plan cyan and the laser yellow vanished with it. Keeping those values
 * duplicated in JavaScript guarantees that the next theme change fixes four
 * panels and forgets two.
 *
 * So tokens.css remains the single source, canvas included: the --map-* tokens
 * are read once, when the panel mounts. Once is enough — the theme does not
 * change at run time, and calling getComputedStyle every frame would cost a
 * style recalculation per repaint.
 *
 * The fallbacks exist because a canvas with no colour draws in black over an
 * almost-white map, which is indistinguishable from "it worked".
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

/** @param {Element} [element] element to inherit the variables from. */
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
