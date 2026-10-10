# Signed joint reference for new policies

Enable `--signed-joint-reference` in BOTH train and play for policies trained in
this new coordinate system. Leave it off for all historical checkpoints,
including the already-running v6.2 run started 2026-10-10_08-45-51.
Do not resume those checkpoints in the new system. Start a fresh run explicitly.

## One coherent reference

The default joint position becomes shoulder/thigh/shin = 0/+34.4/-51.6 degrees.
Isaac Lab's default-offset JointPositionAction therefore requests this pose at
action=0, and keeps scale 0.25 rad per action unit. The inherited relative joint
position observation subtracts the same reference; stand-still joint deviation
reward uses the same default. Observation/action dimensions remain 55/12.
Standing reset JSON already contains these exact angles, so reset and the first
zero-action target agree. This option does not silently rewrite supplied JSONs.

## Coordinates, not a mirrored leg

The legacy model has positive knee limits (+13..+90 degrees) and a negative
standing thigh angle. Simply inserting the new numeric pose into that model
changes its geometry and violates legacy limits. Generate a separate USD with
both joint frames reversed for each thigh/shin joint, negate/reorder the hard
limits, and negate authored drive position/velocity targets. This implements
q_new=-q_old for eight joints; shoulder coordinates, link/collision geometry,
joint origins, gains and motor effort/velocity limits are unchanged.

Original USD/URDF remain untouched. The derived, flattened USD is generated in
ignored `runs/model_variants` with a source-hash filename; each host generates
its own local path. Hardware raw conversion remains in API-signed coordinates,
with the existing seated reference and native-front mapping, not relative action
units. No USB/COM commands or physical robot movement are performed.

## Keep the measured envelope honest

The requested standing pose is NOT fully inside the measured CSV envelope.
Keep measured bounds, calibration and reward weight -10 unchanged; do not widen
the table to make the standing pose pass. The zero-action simulator check reports
per-motor raw excess so this conflict is visible. A measured-range-compatible
standing pose requires a separate user choice/physical validation.

## v6.2 corrected launch settings (not automatically started)

Fresh, 1024 envs, 7000 updates, vx [0,.3], vy/yaw zero, tilt termination 45 deg:
add `--signed-joint-reference` to the v6.2 training arguments and retain
`--reset-standing-pose-json isaac_lab/configs/standing_api_reference.json`.
The previous training process keeps its old loaded definitions and is not
stopped/restarted by this code change.

Verification: `tests/test_signed_joint_reference.py`, full Python regression
suite, and `tests/check_signed_joint_reference_sim.py --headless`. The real
simulation check verifies zero-action targets, relative observations, low-speed
joint-deviation reward, hard-limit inclusion, and three seconds of zero-action
hold, and reports remaining range violations rather than hiding them.

## Verified 2026-10-10

- Full Python regression suite: 72 tests passed.
- Real USD check: eight converted joints, three test angles each; equivalent
  legacy relative rotations after q_new=-q_old, unchanged local joint origins,
  negated/reordered hard limits.
- Real PhysX check: action=0 target equals the default standing pose, relative
  joint position observation is zero, stand-still deviation reward is zero,
  reference is within hard limits without reset-limit extension.
- Latest three-second zero-action hold: no reset, maximum gravity-vector body
  tilt 13.9178 degrees. This is a bounded simulation check with randomized
  physical parameters, not proof of long-term stability or hardware safety.
- Standing raw excess at thigh motor IDs 2/5/8/11:
  105.8508 / 170.7478 / 135.9719 / 169.2437 raw. These remain penalized.
- New signed train CLI: two environments, two fresh updates, correct command
  ranges and 45-degree termination; TRAIN_DONE and checkpoint saved in
  `runs/yertle_flat/2026-10-10_14-26-07`. This validation run is not part of
  the existing 7000-update v6.2 training curve.
