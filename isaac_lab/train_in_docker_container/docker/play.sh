#!/usr/bin/env bash
set -euo pipefail

readonly source_dir="${YERTLE_SOURCE_DIR:-/workspace/yertle}"
readonly project_dir="${YERTLE_PROJECT_DIR:-${source_dir}/isaac_lab}"
readonly checkpoint="${PLAY_CHECKPOINT:-}"

if [[ -z "${checkpoint}" ]]; then
    echo "PLAY_CHECKPOINT is required, for example: /workspace/yertle/isaac_lab/runs/yertle_flat/<run>/model_1999.pt" >&2
    exit 2
fi
if [[ ! -f "${checkpoint}" ]]; then
    echo "Checkpoint was not found at ${checkpoint}." >&2
    exit 2
fi
if [[ ! -f "${project_dir}/play.py" ]]; then
    echo "play.py was not found at ${project_dir}. Check YERTLE_PROJECT_DIR." >&2
    exit 2
fi

cd "${project_dir}"
read -r -a extra_args <<< "${PLAY_EXTRA_ARGS:-}"
video_args=()
if [[ "${PLAY_VIDEO:-0}" == "1" ]]; then
    video_args=(--video --video_length "${PLAY_VIDEO_LENGTH:-400}")
fi
app_args=(--rendering_mode "${PLAY_RENDERING_MODE:-performance}")
if [[ "${PLAY_HEADLESS:-1}" == "1" ]]; then
    app_args+=(--headless)
fi

exec /workspace/isaaclab/isaaclab.sh -p play.py \
    --task "${PLAY_TASK:-flat}" \
    --checkpoint "${checkpoint}" \
    --num_envs "${PLAY_NUM_ENVS:-1}" \
    --steps "${PLAY_STEPS:-600}" \
    "${app_args[@]}" \
    "${video_args[@]}" \
    "${extra_args[@]}"
