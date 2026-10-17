#!/usr/bin/env bash
# Runs on a fresh, disposable Runpod Ubuntu pod, NEVER the local training pod.
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends ca-certificates git curl build-essential cmake \
    libvulkan1 libglu1-mesa libglib2.0-0 libx11-6 libxcb1 libxext6 libxrender1 \
    libsm6 libice6 libxt6 libgl1 libegl1 libxkbcommon0 libdbus-1-3 libnss3 \
    libxcomposite1 libxcursor1 libxdamage1 libxfixes3 libxi6 libxtst6 libxrandr2 \
    libasound2 libatk1.0-0 libatk-bridge2.0-0 libcups2 libgtk-3-0
python3 -m pip install --no-cache-dir 'uv==0.8.22'
uv python install 3.11
uv venv --python 3.11 /opt/yertle-venv
readonly python=/opt/yertle-venv/bin/python
uv pip install --python "$python" 'setuptools<81' wheel
uv pip install --python "$python" "isaacsim[all,extscache]==${YERTLE_SIM_VERSION:-5.1.0}" \
    --extra-index-url https://pypi.nvidia.com --index-strategy unsafe-best-match
uv pip install --python "$python" "torch==${YERTLE_TORCH_VERSION:-2.7.0}" \
    --index-url https://download.pytorch.org/whl/cu128
git clone --depth 1 --branch "v${YERTLE_LAB_VERSION:-2.3.2}" \
    https://github.com/isaac-sim/IsaacLab.git /opt/IsaacLab
uv pip install --python "$python" --no-build-isolation \
    -e /opt/IsaacLab/source/isaaclab \
    -e /opt/IsaacLab/source/isaaclab_assets \
    -e /opt/IsaacLab/source/isaaclab_tasks \
    -e /opt/IsaacLab/source/isaaclab_rl \
    "rsl-rl-lib==${YERTLE_RSL_VERSION:-3.1.2}"
# Lab's dependency solver can replace a local CUDA wheel with PyPI's cu126.
# Pin the exact local-container build AFTER installing all Lab dependencies.
uv pip install --python "$python" --reinstall --no-deps \
    "torch==${YERTLE_TORCH_VERSION:-2.7.0}+cu128" 'torchvision==0.22.0+cu128' \
    --index-url https://download.pytorch.org/whl/cu128
"$python" -c 'import torch,isaaclab; assert torch.cuda.is_available(); print("YERTLE_RUNTIME_READY",torch.__version__,isaaclab.__version__,flush=True)'
