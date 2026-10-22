#!/usr/bin/env bash
set -Eeuo pipefail
mkdir -p build/logs
exec > >(tee build/logs/setup-latest.log) 2>&1
printf 'Dependency setup started: %s\n' "$(date --iso-8601=seconds)"
trap 'code=$?; printf "SETUP FAILED: exit_code=%s\n" "$code" >&2; exit "$code"' ERR
python3 scripts/setup-dependencies.py
printf 'SETUP SUCCESS: pinned dependency commits and clean working trees verified.\n'
