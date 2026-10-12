# v6.2 — standing-start forward-only locomotion

Requested 2026-10-10. Fresh PPO, no checkpoint, 7000 updates, 1024 environments,
seed 0, flat terrain, no automatic disturbance. Preserve all older runs.

Historical run specification: its angles below are unchanged records. The
current standing JSON was subsequently updated to 0/+34/-85 degrees; it must
not be treated as the historical v6.2 reset snapshot.

- Reset every episode to `standing_api_reference.json`: shoulder/thigh/knee
  0 / +34.4 / -51.6 degrees. Do not include the seated reset candidate.
- Commands: vx [0.0, +0.3] m/s, vy = 0 m/s, yaw = 0 rad/s; heading disabled.
- Terminate when absolute body roll OR pitch is >=45 degrees; backwards tilt
  is included, as are left/right tilt and upside-down poses. This is an
  individual roll/pitch threshold, not a combined cone-angle threshold.
- Existing 20-second timeout retained. Base-contact termination remains off;
  base-contact reward penalty remains -1.0.
- Torque 1.9 Nm, PhysX velocity limit 4.6 rad/s; existing 2.3 rad/s excess
  reward threshold and all reward weights unchanged.
- Fixed trot 2.2 Hz; policy observation/action dimensions stay 55/12.
- Expand simulation joint limits to include the signed standing pose. Preserve
  existing observation/action default-pose reference offsets.

Supervisor log: `isaac_lab/runs/v6_2_fresh_standing_vx_0_p0p3_7000/train.log`.
The actual TensorBoard/checkpoint timestamp directory is printed as TRAIN_LOGDIR.

Started run: `isaac_lab/runs/yertle_flat/2026-10-10_08-45-51` (UTC timestamp).
The separate 16-environment, 2-update validation run is
`2026-10-10_08-45-09`; do not merge its events into the v6.2 training curve.

## Corrected reference for a future fresh run

The historical run above changed only reset angles, leaving legacy action
offsets/model axes intact. It is NOT retroactively fixed by source edits.
For the corrected setup, add `--signed-joint-reference` to both train and play.
See `signed_joint_reference.md` for coordinate conversion and the remaining
CSV/standing-pose conflict. No corrected full-length run has been started yet.
