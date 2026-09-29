# ADR 0009: arm64 images are built natively on the module

- **Status:** Accepted

## Context

Building the arm64 images on the x86 host under QEMU took hours. The Aquila
AM69 has 8 Cortex-A72 cores, 31 GiB RAM and more than 100 GB of free storage.

## Decision

`scripts/module.sh sync` copies the sources and the module-side Dockerfiles to
the module. `scripts/module.sh build` builds `base`, `nav`, `perception` and
`tools` there natively, then runs the rendering-library check from
[ADR 0002](0002-simulator-stays-on-host.md). Multi-arch `buildx` builds remain
available for registry publishing.

## Consequences

- A module build takes minutes, not hours.
- Image size matters: images live on the module's data partition.
