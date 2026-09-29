/**
 * Control bar — link state, build identity, and the manual controls.
 *
 * F1 renders the controls DISABLED on purpose, and this is a decision rather
 * than an unfinished edge.
 *
 * Today Nav2 is the only publisher on /demo/cmd_vel. A teleop button that also
 * publishes there would give the demo two unarbitrated writers on the same
 * topic: the last message wins, at whatever rate each side happens to run, and
 * neither the operator nor Nav2 can tell that it lost. The previous cockpit
 * checkpoint recorded exactly this debt. It is closed in F4 with twist_mux
 * (plano-cockpit-web.md Decision 7), not with a hand-rolled mux here.
 *
 * So the buttons exist, are laid out, are keyboard reachable, and say why they
 * are inert. That is honest; a bar that moves the robot unpredictably is not.
 */

import { ConnectionState } from '../ros/rosbridge-client.js';

const STATE_LABELS = Object.freeze({
  [ConnectionState.CONNECTED]: 'connected',
  [ConnectionState.CONNECTING]: 'connecting',
  [ConnectionState.DISCONNECTED]: 'disconnected',
});

export const PENDING_CONTROL_HINT =
  'Manual control arrives in F4, with twist_mux arbitrating against Nav2.';

export function createControlBar({ root, client, endpoints, build, shell }) {
  const badge = root.querySelector('[data-role="badge"]');
  const badgeLabel = root.querySelector('[data-role="badge-label"]');
  const badgeDetail = root.querySelector('[data-role="badge-detail"]');
  const buildLabel = root.querySelector('[data-role="build"]');

  badgeDetail.textContent = endpoints.rosbridge.replace(/^wss?:\/\//, '');
  buildLabel.textContent = `build ${build}`;

  for (const button of root.querySelectorAll('[data-role="control"]')) {
    button.disabled = true;
    button.title = PENDING_CONTROL_HINT;
  }

  const unregister = client.onStateChange((state) => {
    badge.dataset.state = state;
    badgeLabel.textContent = STATE_LABELS[state] ?? state;
    // Mirrored onto the shell so the whole cockpit, not just an 11 px badge,
    // reports a dead link. Panels keep their last frame painted otherwise.
    shell.dataset.link = state;
  });

  return {
    destroy() {
      unregister();
    },
  };
}
