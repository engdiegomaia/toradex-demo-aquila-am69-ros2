# Tools

Developer and evaluation tooling. None of it is needed to run the demo. The
operational scripts are in [`scripts/`](../scripts/).

| Directory | Content |
| --- | --- |
| [`evaluation/`](evaluation/) | Recorders, trial drivers, the A/B campaign runner and analysis of recorded runs |
| [`diagnostics/`](diagnostics/) | Read-only probes: scenario topic contract, sensor rates, TF availability |
| [`maze/`](maze/) | Offline analysis of the maze mesh and generation of the exit marker |

Usage, and the rules for producing comparable measurements, are in
[docs/evaluation.md](../docs/evaluation.md) and [docs/scenarios.md](../docs/scenarios.md).
The tools that talk to ROS need a ROS 2 environment: the `tools` container, or
a native Jazzy installation with `source scripts/env.sh`. Write their output to
`artifacts/`, which is ignored by git.

Several tools document their own design rationale in Portuguese. Their
command-line interfaces are self-describing (`--help`).
