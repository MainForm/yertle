# Yertle headless Isaac Lab training

This Compose project builds a new image from NVIDIA's official Isaac Sim
`5.1.0` image and installs the pinned Isaac Lab `v2.3.2` release. Those
versions were verified in the existing `yertle-isaac` container.

## First WSL run

```bash
cd /mnt/c/Users/ik/Desktop/yertle-compose/isaac_lab/train_in_docker_container
docker compose up --build -d container
```

This creates the image, initializes the editable source checkout, and keeps
the GPU-enabled `container` service running for VS Code or terminal access. It
does **not** start PPO training. Start a run explicitly when ready:

```bash
docker compose up --build train
```

The `TRAIN_*` values in `.env` are read when the `train` service starts. When
training finishes, its container remains running for result/log inspection;
stop it explicitly with `docker compose stop train`.

## Headless policy playback

Set `PLAY_CHECKPOINT` to the **container path** of a model, then start the
separate playback service. The default is one environment, 600 steps, and the
performance renderer. It runs headless; set `PLAY_VIDEO=1` to record a video
next to the checkpoint. Use the separate `play_in_window_native` bundle for a
Windows-native GUI.

```bash
PLAY_CHECKPOINT=/workspace/yertle/isaac_lab/runs/yertle_flat/<run>/model_1999.pt \
  docker compose up --build -d play
docker compose logs -f play
```

The Linux container does not support this project's Windows-only keyboard
control mode. After `PLAY_DONE`, the `play` container remains available for
inspection until you run `docker compose stop play`.

The default host mount is `$HOME/Desktop/yertle-isaaclab`; Docker creates it
automatically. To use another location, prefix the command with
`YERTLE_HOST_ROOT=/absolute/path`. On the first run,
`source-init` clones `https://github.com/MainForm/yertle.git` at
`feature/isaac-lab-rl-locomotion` into `$YERTLE_HOST_ROOT/source`. Later runs
keep that checkout unchanged: Compose never pulls source automatically. It also
converts the versioned URDF into the generated `simulation/usd/yertle.usd` file
when that asset does not already exist.

```
$YERTLE_HOST_ROOT/
├── source/       # Editable Git checkout
├── artifacts/    # source revision metadata and retained outputs
├── cache/        # Kit, shader, pip, and compute caches
├── data/
├── documents/
└── logs/
```

## Deliberately synchronizing source

Commit or stash local edits first. This operation refuses a dirty checkout and
checks out the configured remote branch at a single revision:

```bash
docker compose --profile maintenance run --rm sync-source
```

Use regular Git commands within `$YERTLE_HOST_ROOT/source` for development and
push only verified code. Each training launch records its exact source revision
in `artifacts/metadata/source_commit.txt`.

## Operational notes

- Docker Desktop needs WSL 2 integration and NVIDIA Container Toolkit support.
  Check GPU forwarding before a long run with `docker run --rm --gpus all
  nvidia/cuda:12.8.0-base-ubuntu22.04 nvidia-smi`.
- This is a headless training setup; it intentionally does not mount WSLg/X11.
- The first build downloads the NGC base image and Python dependencies.
- `TRAIN_*` values in `.env` are forwarded to this repository's `train.py`.
