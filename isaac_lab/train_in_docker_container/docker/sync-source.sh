#!/usr/bin/env bash
set -euo pipefail

readonly source_dir="${YERTLE_SOURCE_DIR:-/workspace/yertle}"
readonly git_ref="${YERTLE_GIT_REF:?YERTLE_GIT_REF is required}"

if [[ ! -d "${source_dir}/.git" ]]; then
    echo "No Git checkout exists at ${source_dir}; run source-init first." >&2
    exit 1
fi
if ! git -C "${source_dir}" diff --quiet || ! git -C "${source_dir}" diff --cached --quiet; then
    echo "Refusing to synchronize because the source checkout has uncommitted changes." >&2
    exit 1
fi
git -C "${source_dir}" fetch --depth 1 origin "${git_ref}"
git -C "${source_dir}" checkout --detach FETCH_HEAD
git -C "${source_dir}" rev-parse HEAD
