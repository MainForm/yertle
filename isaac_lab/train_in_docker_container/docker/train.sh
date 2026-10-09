#!/usr/bin/env bash
set -euo pipefail

readonly source_dir="${YERTLE_SOURCE_DIR:-/workspace/yertle}"
readonly project_dir="${YERTLE_PROJECT_DIR:-${source_dir}/isaac_lab}"
readonly artifacts_dir="${YERTLE_ARTIFACTS_DIR:-/workspace/yertle-artifacts}"

if [[ ! -f "${project_dir}/train.py" ]]; then
    echo "train.py was not found at ${project_dir}. Check YERTLE_PROJECT_DIR." >&2
    exit 1
fi
mkdir -p "${artifacts_dir}/metadata"
git -C "${source_dir}" rev-parse HEAD > "${artifacts_dir}/metadata/source_commit.txt"
cd "${project_dir}"

# Local .env selects reset poses and other overrides; paths are relative to project_dir.
# .env.example selects configs/seated_api_reference.json (+75 thigh / -160 shin).
read -r -a extra_args <<< "${TRAIN_EXTRA_ARGS:-}"
exec /workspace/isaaclab/isaaclab.sh -p train.py \
    --task "${TRAIN_TASK:-flat}" \
    --num_envs "${TRAIN_NUM_ENVS:-1024}" \
    --max_iterations "${TRAIN_MAX_ITERATIONS:-2000}" \
    --headless \
    "${extra_args[@]}"
