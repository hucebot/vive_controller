#!/usr/bin/env bash
# Run a command next to SteamVR (used by `make setup` and `make identify`).
#
# Inside the running vive container if there is one, since two SteamVR servers cannot share the USB
# devices; otherwise in a one-off `drivers` container. The .env serials are passed along as a
# fallback for config/station.yaml.
#
#   scripts/vive_run.sh [-it] <command> [args...]      (-it: interactive, so Ctrl+C reaches it)
set -euo pipefail

exec_opts=()
if [[ "${1:-}" == "-it" ]]; then
  exec_opts=(-it)
  shift
fi

env_opts=()
for var in SERIAL_RIGHT SERIAL_LEFT REFERENCE_LIGHTHOUSE_SERIAL SETUP_PORT; do
  if [[ -n "${!var:-}" ]]; then
    env_opts+=(-e "$var=${!var}")
  fi
done

container=$(docker ps --format '{{.Names}}' | grep -m1 -E '^ros2_vive_(franka|tiago|tiago_pro|g1|drivers)$' || true)
if [[ -n "$container" ]]; then
  echo "Using SteamVR in the running container $container."
  exec docker exec "${exec_opts[@]}" "${env_opts[@]}" -e "VIVE_SETUP_ATTACHED_TO=$container" \
    "$container" /entrypoint.sh "$@"
fi
exec docker compose --profile drivers run --rm --no-deps "${env_opts[@]}" drivers "$@"
