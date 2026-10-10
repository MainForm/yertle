# Verification — 2026-10-10

Historical verification below used the reverse-front mapping. Later on this
date, native front was restored (RF=rf, LF=lf, RR=rb, LR=lb). The preserved
before-state and new verification are in the parent project's measurements
directories `position_api_before_leg_rotation_restore_20261010` and
`position_api_leg_rotation_restored_20261010`. Do not reuse the old q/name
comparison as a current-frame result; signs and raw calibration are unchanged.

- `tests/test_motor_operating_range.py`: **14 tests passed** (CPU PyTorch).
- Existing `tests/test_joint_velocity_excess.py`: **6 tests passed**.
- New CSV's 54 data rows match the original CSV exactly, ignoring newline style.
  SHA256 of UTF-8 CSV with LF newlines and one final newline:
  `f08089adb9014ece9c4e82aa94ef9deffb2d8aff087247ef01b88afa592ac36c`.
- Actual current C++ `PolicyRawConverter` calibration (IDs, signs, references)
  matches the training transform. All **233** historical measured samples have
  identical rounded raw output; maximum continuous raw difference is
  **4.55e-13 raw**. The comparison script is saved in the parent project under
  `measurements/motor_operating_range_reward_20261010/verify_current_api.py`.
- Previously reconstructed actor target yielding motor 7 raw **4403** is now
  detected by the range cost; there is no wrapping/clamping to hide that target.
- AST comparison against repository HEAD confirms every existing reward
  definition/weight is unchanged; only the new term registration was added.
- Installed Isaac Lab source confirms `JointPositionAction._joint_names` is
  the resolved processed-action order, action manager initializes before reward
  manager, and the class-based reward signature matches its registration rules.
  Registration/calculation was tested with manager stubs, not a real Kit env.
- **No new Isaac Sim rollout:** GPU was using 7705/8151 MiB (446 MiB free).
  A second Kit instance was not launched to avoid disturbing ongoing training.
  No new training, COM command, robot movement, commit or push occurred.

## Existing pose incompatibility (not silently changed)

The historical action default (thigh -0.6, shin +0.9 rad) already lies outside
the CSV envelope in all four shin/thigh pairs. Its unweighted cost is about
465.6685 (weight -10, dt .02 => about -93.13 per control step when both requested
and measured positions equal it).

The signed API standing pose (thigh +0.6, shin -0.9 rad) has thigh IDs 2/5/8/11
outside their interpolated ranges by approximately 106/171/136/170 raw.
Its unweighted cost is about 12.7904 (weighted timestep cost about -2.5581).

These are CSV-based simulation diagnostics, not additional physical collision
measurements. Reset/action reference poses remain unchanged as requested scope
was reward-only. Before a new run, choose explicitly whether to also adopt a
measured-range-compatible reset pose. A reward is not a hard range guarantee.
