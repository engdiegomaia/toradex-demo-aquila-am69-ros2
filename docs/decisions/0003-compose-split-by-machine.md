# ADR 0003: One Compose file per machine, modes via profiles

- **Status:** Accepted (supersedes the earlier `compose/{learn,emul,target}.yaml` layout)

## Context

The three operating modes (`learn`, `hil`, `deploy`) differ only in `platform:`
and in which machine starts each service. With one file per mode, each file
duplicated every service definition, and the copies drifted apart.

## Decision

`docker/compose.host.yml` describes the x86 host and
`docker/compose.module.yml` describes the Aquila AM69. A mode is selected by
which file is started on which machine, plus Compose profiles (`learn`,
`tools`). `ROBOT_TYPE=quadruped|diffdrive` selects the matching plant and
Nav2 stack identically in both files. The dedicated emulation mode was
dropped. QEMU is used only to build images.

## Consequences

- A change that needs different code per mode means the design is wrong.
- Structural tests in `tests/` keep the two files consistent. One example is
  the Nav2 parameter override, which must be exposed the same way on both
  machines.
