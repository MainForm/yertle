#!/usr/bin/env bash
set -euo pipefail

readonly source_dir="${YERTLE_SOURCE_DIR:-/workspace/yertle}"
readonly repository_url="${YERTLE_REPOSITORY_URL:?YERTLE_REPOSITORY_URL is required}"
readonly git_ref="${YERTLE_GIT_REF:?YERTLE_GIT_REF is required}"

if [[ -d "${source_dir}/.git" ]]; then
    echo "Yertle source already exists at ${source_dir}; leaving it unchanged."
else
    if [[ -n "$(find "${source_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
        echo "Refusing to clone: ${source_dir} is not empty and is not a Git checkout." >&2
        exit 1
    fi
    git clone --depth 1 --branch "${git_ref}" "${repository_url}" "${source_dir}"
fi

readonly urdf_path="${source_dir}/simulation/yertle.urdf"
readonly usd_path="${source_dir}/simulation/usd/yertle.usd"
if [[ ! -f "${usd_path}" ]]; then
    if [[ ! -f "${urdf_path}" ]]; then
        echo "URDF file was not found at ${urdf_path}." >&2
        exit 1
    fi
    echo "Converting the checked-out Yertle URDF to the required USD asset."
    mkdir -p "$(dirname "${usd_path}")"
    /workspace/isaaclab/isaaclab.sh -p /workspace/isaaclab/scripts/tools/convert_urdf.py \
        "${urdf_path}" "${usd_path}" --merge-joints --headless
fi

chmod -R a+rwX "${source_dir}"
git -C "${source_dir}" rev-parse HEAD
