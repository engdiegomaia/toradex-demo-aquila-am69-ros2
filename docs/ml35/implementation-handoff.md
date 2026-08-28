# Implementation handoff — repository cleanup and ML3.5 F5 completion

This document is the execution plan for the next LLM. Treat the current working
tree as authoritative: it contains user-owned F5 evidence and cleanup changes
that must not be discarded or reset.

## 1. Current checkpoint

Completed and validated before this handoff:

- `joint_state_broadcaster` is limited to 50 Hz without changing the 1 kHz
  controller-manager loop or the 200 Hz gait controller;
- `/tf` fell by 86.3%, module load fell by 31%, and `odom <- lidar`
  availability reached 99.94%;
- `map_update_interval` is back at 1.0 s, with no material CPU regression;
- the short stability gate completed its first three goals in all three runs,
  with no falls, TF extrapolation, `worldToMap`, or invalid-source errors;
- the repository has an `origin` remote configured as
  `https://github.com/engdiegomaia/toradex-demo-aquila-am69-ros2.git`;
- `.ai/` is ignored and removed from the Git index while its local files remain;
- current tracked files no longer contain the bench hostname, real bench IPs,
  the project owner's corporate email, common private-key markers, or common
  GitHub/AWS/OpenAI token patterns;
- `README.md` and `docs/README.md` provide public documentation entry points;
- the obsolete `docs/ml35/plano-proximos-passos.md` was removed and references
  now point to the current movement plan or preserved result report.

The last complete local validation before the translation pass was:

```text
pytest -q tests    218 passed
git diff --check  passed
```

Run these again before accepting the checkpoint because documentation was
translated afterward.

## 2. Working-tree safety rules

Do not run `git reset`, `git checkout --`, `git clean`, or broad formatting.
Several F5 documents were already modified by the user before repository
cleanup began. Preserve all current changes.

`.ai/` appears as staged deletions because it was removed only from the index.
The files still exist locally and are ignored. This is intentional.

`artifacts/` remains untracked. Do not add or delete it without an explicit
decision about scratch-run evidence.

No commit and no push have been performed for this cleanup.

## 3. English documentation migration

The migration is intentionally incomplete. Two external translation providers
were tried: Claude reached its organization spend limit, while broad Codex
batches stopped without editing. Small, exact-file batches worked.

Already translated substantially or completely:

- `README.md`;
- `CLAUDE.md`;
- `docs/README.md`;
- `docs/analise-sensores-navegacao.md`;
- `docs/guia-completo.md`;
- `docs/guia-hil-go2-labirinto.md`;
- most files under `docs/guides/`.

Still requiring translation:

- most files under `docs/ml35/`;
- all or most reports under `docs/results/`;
- comments and docstrings in `scripts/`, `tests/`, `ros2_ws/src/`, `docker/`,
  and `hmi/`;
- a small number of Portuguese comments or literal labels may remain in the
  already translated guides.

Translation policy:

1. Translate explanatory prose, headings, table labels, source comments, and
   docstrings.
2. Preserve executable commands, syntax, paths, filenames, ROS topic/frame/node
   names, URLs, numeric evidence, and CSV data.
3. Preserve quoted runtime and log output unless the program itself is also
   intentionally changed and its tests are updated.
4. Do not translate user-visible runtime strings as part of the comment pass;
   they may be API, UI, or test contracts.
5. Use batches of one large file or at most three small files. Do not ask an
   external agent to translate an entire directory containing thousands of
   lines in one turn.
6. After each batch, scan only that scope for Portuguese and run
   `git diff --check`.

Suggested discovery command:

```bash
rg -n '[áàâãéêíóôõúçÁÀÂÃÉÊÍÓÔÕÚÇ]|\b(não|robô|módulo|navegação|saída|evidência|medido|configuração)\b' \
  README.md CLAUDE.md docs scripts tests ros2_ws/src docker hmi \
  --glob '!docs/results/*.csv' --glob '!artifacts/**' --glob '!.ai/**'
```

This scan is a candidate list, not proof: English words such as `façade`,
identifiers, quoted Portuguese logs, and proper names can be legitimate.

## 4. Sensitive-information closure

The current tree was sanitized, but old commits still contain the removed
`.ai/` files, bench identifiers, private-network addresses, and old maintainer
email. No strong secret/token pattern was found in 107 commits.

Do not push the full history to a public remote until the owner chooses one of
these options:

1. keep history private and push normally;
2. publish a new clean root history;
3. rewrite history with `git filter-repo`, then review every changed commit ID
   and force-push only with explicit authorization.

History rewriting is destructive and is not authorized by this handoff.

## 5. Functional F5 completion sequence

After repository cleanup is stable, resume the functional demonstration in this
order.

### Gate A — perception on the Aquila

Validate, using bounded captures rather than continuous log monitoring:

- `/demo/camera/camera_info` reaches the module;
- the exit detector processes images at the configured rate;
- at least three of five frames contain the expected detection;
- `/demo/perception/maze_exit/pose` is published in the correct camera frame;
- detector CPU does not cause TF availability to fall below 99.5%;
- the controller does not suffer sustained deadline misses.

If detector CPU is the blocker, change only `sample_stride` and repeat the same
protocol.

### Gate B — one exploration smoke test

Start from a clean map and initial pose. Record:

- exploration state transitions;
- `frontier_extract_ms`, frontier cells/clusters, and selection cycles;
- every Nav2 goal outcome and error code;
- blacklist growth, map coverage, path length, and forward-work ratio;
- TF availability and prohibited error counts;
- active `maze_explorer` and `nav2_container` CPU.

Use this run to measure the frontier optimization on the Aquila. Idle
`maze_explorer` CPU is not a blocker by itself. Profile it only if the active
run shows extraction above 100 ms p95, sustained controller misses, repeated
selection without progress, or a TF/perception regression.

### Gate C — three cold starts

Only after a successful smoke test:

1. restart simulation, SLAM, Nav2, and perception;
2. confirm there is no saved map or pose graph;
3. start exploration once, without intervention;
4. stop on `/demo/maze/escaped=true` or at 600 s;
5. preserve evidence before the next run.

Final acceptance remains three of three runs escaping within 600 s, with no
fall, manual intervention, hard-coded maze coordinates in the explorer,
recurrent TF error, or costmap/frame failure.

## 6. Final repository validation

Before committing:

```bash
git diff --check
pytest -q tests
cd hmi && npm test
cd ..
docker compose -f docker/compose.host.yml config
docker compose -f docker/compose.module.yml config
git status --short
git remote -v
```

Also run the sensitive-pattern scan again across tracked and untracked files,
without printing matched values. Review Markdown local links and confirm that
the only intended document deletion is the obsolete movement plan.

Prepare separate commits for:

1. sensitive-data cleanup and `.ai/` removal;
2. documentation consolidation and English migration;
3. any functional F5 changes and their evidence.

Do not combine history rewriting, functional navigation tuning, and translation
in one commit.
