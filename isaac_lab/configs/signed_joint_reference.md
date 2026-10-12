# Signed joint reference for new policies

## CURRENT — coordinate reversal rolled back 2026-10-11

Keep the original USD axes, both local joint frames and angular-drive target
position/velocity values. The current requested reference is 0/+34/-85 degrees;
the changed joint limits are retained. The historical `--signed-joint-reference` option now
selects this ORIGINAL-AXES, limits-only variant, not the old axis-reversed one.
Its cache name is `yertle_original_axes_limits_v2_*`; old generated v1 files and
checkpoints are preserved, not overwritten. No original USD/URDF or C++ API
mapping is modified. Same numerical angles intentionally produce a different
physical pose from the old axis-reversed v6.3 model.

The already-running v6.3 process retains its loaded axis-reversed model, as the
user requested. Source rollback does not change that live process. Do not load
its checkpoint with the restored model and claim equivalent behavior.

Absolute-angle-to-raw conversion remains unchanged and was compared against
the actual C++ converter on 233 samples, all rounded raw values identical.
The API's default `fromPolicyActions()` offsets still use 0/-0.6/+0.9 radians;
`fromJointAnglesRad()` adds no action offsets. Preserving motor calibration
does not by itself prove policy-action compatibility. That separate integration
choice remains unresolved; no API/action-offset change is made by this rollback.

Verification: `check_signed_joint_reference_sim.py --headless --usd-only`
checks original axes/frames/drive targets and same-angle relative rotations,
while requiring the deliberately retained changed limits.

## Current standing adjustment — 2026-10-13

The active opt-in train/play reference and `standing_api_reference.json` now
both use shoulder/thigh/shin 0/+34/-85 degrees. Default-offset actions,
relative joint observations and stand-still deviation reward follow the same
reference automatically. Only the current reference changes; axes, frames,
drive targets, limits, action scale, observation dimensions, reward weights,
measured operating-range CSV and motor calibration remain unchanged.

The old 0/+34.4/-51.6 policies (v6.4/v6.5) are not equivalent under this new
reference, despite identical 55/12 dimensions. Preserve their old code/config
when evaluating them. Existing live training retains its previous loaded
reference; no interruption or new training is performed by this change.
Legacy no-flag configuration is retained for old checkpoints, not silently
switched to negative shin angles with the original positive-shin-limit USD.

Keep all historical sections/results below as historical evidence; their
angles, geometry measurements and range violations are not current results.

Current bounded verification: 92 CPU tests pass. The actual restored-axis
simulation confirms reference/action/relative-observation agreement and hard
limit inclusion; a 3-second zero-action hold had no resets and maximum gravity
tilt 5.6039 degrees. The new reference has zero measured-envelope excess on all
12 motors. At the 0.25 m initial spawn height, rubber sole Z is 0.052476 m while
shin-origin Z is 0.142225 m; this is before settling, not a ground-contact claim.
The actual sole transform and ten simulation steps pass independent checks.

## HISTORICAL — superseded axis-reversed implementation below

The remaining sections describe the previous implementation and its historical
tests. They are NOT the specification or stability results of the restored model.

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
