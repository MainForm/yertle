# Tilt AND nonsole ground contact termination

For new runs with `--body-tilt-termination-deg 45`, terminate only when both
conditions hold in the same control step:

1. `abs(roll) >= 45 deg OR abs(pitch) >= 45 deg`.
2. At least one forbidden body-ground contact exists: base, shoulder, thigh,
   or a shin contact outside the existing authored Rubber-surface allowance.

Use the existing ground-contact classifier and its 1 mm separation / 2 mm
Rubber-surface tolerances, without a force threshold. This is the existing
Rubber geometry definition, not a new sole-only mesh/calibration definition.
Read the last physics-substep contact snapshot, not all substeps or history.
The reward and termination share a cached classifier read for each control step.

Remove the old `body_tilt` term and register `body_tilt_and_nonsole_contact`;
otherwise an old tilt-only term would defeat the AND logic. Keep timeout and
existing contact rewards unchanged. A tilted robot with sole-only support or
no ground contact is allowed to continue; forbidden contact alone below 45 deg
also continues. This intentionally allows more recovery than tilt-only resets.

The CLI angle remains configurable; omitting it preserves existing behavior.
No policy observation/action, joint frame/reference or hardware mapping changes.
Play behavior is unchanged. Already running v6.11/v6.12 processes retain their
loaded tilt-only 45 deg condition until a new process is explicitly started.
Historical `body_tilt` logs must not be interpreted as the new conjunction rate.
Full environment-state checkpoint resume requires matching configuration; a
changed termination requires an intentional environment reset or fresh run.

Verification: tensor AND truth table, +/-45-degree roll/pitch boundaries,
inverted body, all 13 forbidden bodies, sole-only/airborne cases, current-step
non-latching, tolerance validation and term registration/timeout preservation.
The existing geometry/contact-classifier tests remain applicable. No new
Isaac Sim process is started alongside the two memory-constrained training runs.
