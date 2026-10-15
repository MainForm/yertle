# Environment-aware checkpoint continuation

Flat-task `train.py` now uses a local `StatefulOnPolicyRunner`. Every regular
checkpoint and the final checkpoint contains `infos.yertle_continuation` in
the same `.pt` as the original model/optimizer keys. The file is written to a
temporary sibling and atomically replaced only after both snapshots succeed.
No external Isaac Lab / RSL-RL installation is patched. `play.py` can load the
model normally and ignores the training-only metadata.

## State and semantics

- Save after a completed PPO rollout/update, never in the middle of a rollout.
- Save world root/joint poses and velocities, joint targets, randomized mass,
  inertia, center-of-mass and material properties.
- Save environment episode/counters, command/resampling timers and metrics,
  action/current-previous history, reward sums and stateful reward history,
  contact force/air-time history, event timers and tensor state.
- Save the last observation (including its already sampled noise), per-env
  running episode returns/lengths and 100-episode logger buffers.
- Save Python, NumPy, Torch CPU and all CUDA RNG states, adaptive learning rate,
  total steps/time. Restore RNG immediately before subsequent rollout sampling.
- Checkpoint update k is already complete; full-state resume starts at k+1.
  Legacy checkpoints keep the previous iteration-label behavior.
- SIGINT/SIGTERM request completion of the current PPO iteration and a final
  snapshot. SIGKILL, power loss or Docker's forced timeout cannot save new state.

## CLI

`--checkpoint model_k.pt --resume-env-state auto` is the default: restore the
environment when metadata exists; clearly log legacy reset otherwise.
`required` rejects old checkpoints without metadata. `off` explicitly resets
the environment while loading the model/optimizer, for intentional new settings.

Full-state restoration requires identical environment configuration, environment
count, body/joint order and state layout. Mismatch fails rather than silently
loading incompatible state. Current support: single-process, non-recurrent PPO,
TensorBoard logger, single-robot flat task; no RND, observation history or active
external force pulses. PPO/model settings must also match on full-state resume.
Rough-task training retains its original runner and does not save env state.

## Limits

This is best-effort environment continuation, NOT an exact PhysX snapshot.
Solver warm-start caches/contact manifolds and GPU nondeterminism are not
restored; the first physical contacts may differ. Do not promise identical
future trajectories. No axes, API mapping, observation/action semantics or
reward definitions are changed. A live pre-change Python process is unaffected:
only a new training process uses this feature. Existing old checkpoints cannot
retroactively gain their lost environment states.

## Verification

- 128 CPU regression tests, including RNG roundtrip, tensor state/dtype/shape,
  atomic-save failure preserving the previous checkpoint, legacy/required/off
  loading and next-iteration/learning-rate restoration.
- Real two-env checkpoint creation at updates 0 and 1. New process restores
  model_1, checks saved root/joint poses/velocities, targets, masses/inertias/COM/
  materials, command and episode/action/reward/contact history before a physics
  step, then runs update 2 and saves model_2 with next_iteration=3.
- Container emitted Vulkan graphics warnings/errors during shutdown (one early
  test process segfaulted after TRAIN_DONE); final save/restore tests returned
  exit code 0 and checkpoint/buffer verification markers. This is not a claim
  of exact solver continuity or GUI validation.
