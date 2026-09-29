# Engineering Log

This project was developed in milestones (ML1–ML3.5). Each milestone came with
a phase-status log, implementation plans and dated measurement reports with
raw CSV telemetry. That material was written as a lab notebook, mostly in
Portuguese, and records rejected hypotheses alongside accepted ones.

It was removed from the main tree for publication and is preserved unchanged
at an annotated git tag:

```bash
git tag -l 'archive/*'                     # archive/ml35-engineering-log
git show archive/ml35-engineering-log:docs/ml35/estado-fases.md
git ls-tree -r --name-only archive/ml35-engineering-log docs/
git switch --detach archive/ml35-engineering-log   # browse it as a tree
```

The condensed outcome is published in [validation.md](validation.md) and in the
[architecture decision records](decisions/README.md).

## Paths cited in source comments

Comments in the code, the parameter files and the tests cite evidence by paths
such as `docs/results/ml35-f5-footprint-ab.md` or
`docs/ml35/estado-fases.md`. Those paths resolve inside the archive tag:

```bash
git show archive/ml35-engineering-log:docs/results/ml35-f5-footprint-ab.md
```

## Map of the archived material

### Status and plans (`docs/ml35/`)

| File | Content |
| --- | --- |
| `estado-fases.md` | Authoritative phase-status log: phases F0–F6, exploration rounds R1–R17, decisions, invariant rules |
| `implementation-handoff.md` | Session handoff notes: delivery envelope, tagged commits, open items |
| `guia-ml35-docker.md` | Container and Compose architecture specification |
| `plano-cockpit-web.md` | Web cockpit plan: decisions, phase gates, risks |
| `plano-movimentacao.md` | Survey of gait-tuning options |
| `proximos-passos-navegacao.md` | Investigation of "robot spins instead of translating", ending in the connected-route diagnosis |
| `guia-implementacao-fechamento-f5.md` | Implementation plan for closing phase F5 |

### Measurement reports (`docs/results/`)

Each report `*.md` states what it measures. The CSVs with the same stem are
its raw telemetry.

| Report | Topic |
| --- | --- |
| `ml35-f1-execucao.md`, `ml35-f3-execucao.md`, `ml35-f4-parcial.md` | Containerized baseline, Go2 stand/walk gate, gait tuning |
| `ml35-caminhada-reta.md`, `ml35-postura-parada.md` | Curved-walk and standing-fall root causes (host) |
| `ml35-nav2-quadrupede.md`, `ml35-navegacao-maze11.md`, `ml35-regressao-navegacao.md` | Quadruped Nav2 integration and tuning (host) |
| `ml35-labirinto.md` | Offline maze geometry analysis |
| `ml35-target-preparacao.md` | First access to the AM69, native arm64 builds |
| `ml35-hil-aquila.md`, `ml35-hil-ethernet.md`, `ml35-hil-rota-ethernet0.md`, `ml35-f5-ethernet0-repeticao.md` | First HIL rounds over Wi-Fi and Ethernet, DDS interface selection, CPU saturation |
| `ml35-f5-camera-comprimida.md` | Compressed camera transport |
| `ml35-f5-clock-fanout.md`, `ml35-f5-tf-cpu-baseline.md`, `ml35-f5-ab-joint-states.md` | `/clock` fan-out, TF availability, joint-state rate A/B |
| `ml35-f5-mppi-amostragem.md`, `ml35-f5-memoria-costmap.md`, `ml35-f5-footprint-ab.md` | Controller sampling, costmap memory, footprint A/B |
| `ml35-f5-rota-conectada.md`, `ml35-f5-portao-tres-metas.md` | Connected-route diagnosis, short stability gate |
| `ml35-f5-slam-exploration.md`, `ml35-f5-slam-giro.md` | SLAM-integrated exploration, turn authority |
| `ml35-f5-perception-aquila.md`, `ml35-f5-apriltag-positioned.md` | Perception and exit-marker gates on the AM69 |
| `ml35-f5-busca-autonoma.md` | Autonomous exit search: feature and acceptance status |
| `ml35-f5-exploration-smoke.md`, `ml35-f5-exploration-r1.md` … `-r16.md` | HIL exploration rounds |
| `ml35-f5-homing-fall-analise.md` | Open investigation of the fall during blind homing |
| `ml35-f5-r17-demo-closeout.md` | Closeout of the supervised-demo release |
| `cockpit-standalone-parcial.md` | Abandoned X11 embedding cockpit |
| `cockpit-web-f1.md`, `cockpit-web-f3b.md`, `cockpit-web-ui-ajustes.md`, `cockpit-reset-nao-destrutivo.md` | Web cockpit phase gates and the non-destructive reset |

### Other

| File | Content |
| --- | --- |
| `docs/analise-sensores-navegacao.md` | Sensor rates and DWB → MPPI analysis on the original differential-drive robot |
| `docs/guides/go2-testes.md`, `docs/guides/go2-proximos-passos.md` | Go2 gait test manual and follow-up |
| `docs/guia-completo.md`, `docs/guia-hil-go2-labirinto.md`, `docs/run-guide.md` | Former operation guides, replaced by the current `docs/` |
| `docs/guides/cenarios/` | Former per-scenario guides, replaced by [scenarios.md](scenarios.md) |
