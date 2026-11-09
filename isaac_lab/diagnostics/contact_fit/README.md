# v6.22.17 real-goal replay

Offline PhysX diagnostic for `model_29000.pt` (SHA-256
`ef61f694f7740d60a6fe626ed5654b0174b49f3642c339cf6c8ea7e0382827a9`).
It replays acknowledged hardware motor goals in two straight-command windows
from `robot_20261109_210310_18104.jsonl`. It does not change the training task,
the checkpoint, or the motor and observation APIs. The committed `history/`
files preserve the previous friction, PWM, timing and static body-property
sweeps plus the independent yaw replay. The raw hardware log and generated
traces stay outside Git.

## Static body-property replay

Run `python prepare.py HARDWARE_LOG OUTPUT_DIR/inputs.json` with the paths
appropriate to the host. Then, from the container's source root:

```bash
export YERTLE_CONTACT_REPLAY_DIR=/workspace/yertle-artifacts/model29000-contact-fit/mass-properties
checkpoint=/workspace/yertle/isaac_lab/runs/yertle_flat/2026-11-08_12-22-33/model_29000.pt
/workspace/isaaclab/isaaclab.sh -p isaac_lab/diagnostics/contact_fit/run.py \
  --headless --device cpu --task flat --num_envs 30 --checkpoint "$checkpoint" \
  --signed-joint-reference --trot-frequency-hz 1.0 --trot-duty-factor 0.6 \
  --trot-support-penalties --explicit-linked-motor --start-pose json \
  --pose-json isaac_lab/configs/standing_api_reference.json
python isaac_lab/diagnostics/contact_fit/score.py
```

Score the 0.25 m/s fitting window and the independent 0.20 m/s window
separately. The scorer skips the first 0.5 s and compares accumulated yaw,
body inclination, and motor positions against the recorded feedback. The
initial foot clearance is estimated from the model because the hardware log
does not contain absolute body height. The fitted body properties represent
replay hypotheses, with no direct mass-distribution measurement.
