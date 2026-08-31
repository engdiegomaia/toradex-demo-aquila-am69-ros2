# Documentation

This index defines the project's canonical sources. Reports in `results/` are
historical evidence: they may describe rejected configurations and must not be
used as operational instructions without checking the current status.

## Start here

- [`guia-completo.md`](guia-completo.md): installation, operation, and cockpit.
- [`run-guide.md`](run-guide.md): concise checklist for running the demo.
- [`guia-hil-go2-labirinto.md`](guia-hil-go2-labirinto.md): running the Go2 in
  the maze under HIL.
- [`ml35/estado-fases.md`](ml35/estado-fases.md): current ML3.5 status,
  decisions, and blockers.
- [`ml35/guia-implementacao-fechamento-f5.md`](ml35/guia-implementacao-fechamento-f5.md):
  remaining gates for the autonomous exit demo.

## Implementation references

- [`development.md`](development.md): host development.
- [`ml35/guia-ml35-docker.md`](ml35/guia-ml35-docker.md): containerized
  architecture and operation.
- [`analise-sensores-navegacao.md`](analise-sensores-navegacao.md): sensor and
  navigation chain.
- [`guides/cenarios/README.md`](guides/cenarios/README.md): simulation
  scenarios.
- [`guides/go2-testes.md`](guides/go2-testes.md): Go2 test commands.

## Current plans

- [`ml35/plano-movimentacao.md`](ml35/plano-movimentacao.md): Go2 gait.
- [`ml35/plano-cockpit-web.md`](ml35/plano-cockpit-web.md): web cockpit.
- [`ml35/proximos-passos-navegacao.md`](ml35/proximos-passos-navegacao.md):
  F5 navigation investigation and decisions.

## Evidence

[`results/`](results/) holds the reports and raw data behind the decisions. Every
report must record commit, configuration, environment, and limitations. An old
result does not override the criteria currently in force in
`ml35/estado-fases.md`.

## Local content

The `.ai/` directory is local, ignored by Git, and not part of the published
documentation. Anything required to operate or contribute must live in this
`docs/` directory, in the root `README.md`, or in the root `CLAUDE.md`.
