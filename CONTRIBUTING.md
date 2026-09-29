# Contributing

Thank you for your interest in improving this demo.

## Before you start

- Read [docs/architecture.md](docs/architecture.md) and the
  [decision records](docs/decisions/README.md). Most constraints in this
  repository come from the hardware and are not stylistic.
- Say in the description of every change which machine it affects: the x86
  host, the Aquila AM69, or both.

## Hard rules

1. No desktop-OpenGL software (Gazebo, RViz2, anything on OGRE 2) in a module
   image, launch file or Compose service.
2. `rmw_cyclonedds_cpp` only, set in the base image.
3. Never document or script an OTA path to the Torizon OS baseline. The OS is
   installed with Toradex Easy Installer.
4. No performance conclusions from QEMU emulation.
5. Perception communicates only through the
   [topic contract](docs/architecture.md#topic-contract).
6. Hardware validation is claimed only when it was performed on a real
   Aquila AM69. Otherwise mark the result `PENDING EXECUTION`.
7. No addresses, hostnames or credentials in committed files.

## Workflow

1. Create a branch from `main`.
2. Make the smallest change that solves the problem. Put tuning in YAML, not in
   code.
3. Add or update tests. Run all suites that apply:

   ```bash
   python3 -m pytest tests -q
   (cd hmi && node --test "test/**/*.test.js")
   (cd ros2_ws && colcon build --symlink-install && colcon test && colcon test-result --verbose)
   ```

4. Update the documentation touched by the change. A new lasting decision gets
   an ADR in `docs/decisions/`.
5. Use [Conventional Commits](https://www.conventionalcommits.org/):
   `feat:`, `fix:`, `docs:`, `refactor:`, `test:`, `chore:`, `perf:`, `ci:`.

## Reporting results

Performance or behaviour results must include the machine, link type, commit,
parameter file and raw data. See [docs/evaluation.md](docs/evaluation.md).

## Vendored code

Changes to vendored packages must be minimal and recorded in the package's
`PROVENANCE.md` or `README.md`, together with the upstream reference.

## License

By contributing you agree that your contributions are licensed under the
[Apache License 2.0](LICENSE).
