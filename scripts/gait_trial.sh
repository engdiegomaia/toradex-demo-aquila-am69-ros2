#!/usr/bin/env bash
set -euo pipefail

# Run one gait trial inside the running sim container and save the CSV on the
# host.  All arguments are forwarded to scripts/gait_trial.py; run it with
# --help to see them.
#
#   ./scripts/gait_trial.sh docs/results/csv/run.csv --v-cmd 0.10 --cycles 5
#
# The Python file is piped in over stdin instead of being mounted, so the sim
# image does not have to be rebuilt to change the trial.  The CSV goes to
# stdout and the human summary to stderr, which is why the redirection below
# only captures one of them.

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
container_name="aquila-go2"

if [ "$#" -lt 1 ]; then
  echo "usage: $0 <csv-output-path> [gait_trial.py args...]" >&2
  exit 2
fi

csv_path="$1"
shift

if ! docker ps --format '{{.Names}}' | grep -qx "${container_name}"; then
  echo "Container ${container_name} não está em execução. Suba a simulação com" \
       "./scripts/run_quadruped_sim.sh primeiro." >&2
  exit 1
fi

mkdir -p "$(dirname "${csv_path}")"

# `_` is a placeholder for $0 inside the container shell, so that "$@" carries
# the trial arguments and not the script name.
docker exec -i "${container_name}" bash -lc '
    . /opt/ros/jazzy/setup.sh
    . /test/install/setup.sh
    exec python3 - "$@"
  ' _ "$@" < "${repo_dir}/scripts/gait_trial.py" > "${csv_path}"

echo "CSV: ${csv_path}" >&2
