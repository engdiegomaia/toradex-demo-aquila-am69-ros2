# Toradex Demo — Aquila AM69 + ROS 2

ROS 2 Jazzy demo in which Gazebo Harmonic runs on the x86 host while navigation
and perception run in arm64 containers on the Aquila AM69. The main scenario
uses a Go2 quadruped to map a maze and autonomously search for its exit.

## Documentation

- [Documentation index](docs/README.md)
- [Complete operations guide](docs/guia-completo.md)
- [Quick demo run guide](docs/run-guide.md)
- [Go2 maze HIL guide](docs/guia-hil-go2-labirinto.md)
- [Current ML3.5 status](docs/ml35/estado-fases.md)
- [Autonomous demo completion](docs/ml35/guia-implementacao-fechamento-f5.md)

The reports in [`docs/results/`](docs/results/) are historical evidence and may
record rejected conditions. For current decisions, consult the phase status
first.

## Security and configuration

Addresses, hostnames, credentials, and bench settings stay out of Git. Copy
`docker/.env.example` to `docker/.env` and provide the environment values.
Torizon OS 7.7.0 must be installed on Aquila V1.0A using Toradex Easy Installer;
do not use OTA to migrate an older base image to this baseline.
