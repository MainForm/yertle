# Current PWM diagnostic: direct received goal

The production PWM C PID now directly compares received absolute raw goal
with calibrated measured position. It accepts NO Speed/ACC, stores no reference
velocity, and has no intermediate target, PWM slew limiter or D filter.
Speed/ACC remain in0x06 packets for native position mode only.

## Current reproduction

Use the rebuilt library under `/workspace/yertle-artifacts/pwm_direct_goal_20261027`.
The diagnostic rejects historical profiled libraries (`pid_direct_goal()==1`
and C state-size checks) and asserts every valid PID reference equals the goal.

```powershell
docker exec yertle-isaaclab-container-1 /isaac-sim/python.sh -u /workspace/yertle/isaac_lab/diagnose_error_contact.py --data-dir /workspace/yertle-artifacts/pwm_direct_goal_20261027 --out-dir /workspace/yertle-artifacts/pwm_direct_goal_20261027 --backend inverse-compare --kp 5 --ki 3 --kd .1 --plant dc --motor-gain 1 --physics-dt .0005 --seconds 1 --initial-integral log --sync-start --repeats 1 --command -0.1 0 0 --label NEW_LABEL --headless --device cuda:0
```

Use a new label. Require `_SUCCESS`, valid JSON/NPZ and assertions: Isaac shutdown
can mask Python failures behind exit0. Headless computation, not GUI, is tested.

## Backends

- `full-pid`: production raw/PWM P5/I3/D0.10, I cap100/PWM cap800.
- `converted-pid`: independent motor-angle-radians / commanded-Nm PID.
- `inverse-compare`: both with the same direct goal and motor plant.
- `implicit-pd`: separate native joint-PD reference, not physical F103 PID.

`convert_raw_pid()` gives P4.17215134Nm/rad, I2.50329080Nm/(rad*s),
D0.0834430268Nm*s/rad, I cap0.128Nm/output cap1.024Nm. This uses the EXISTING
ASSUMED0.00128Nm/PWMraw, not measured torque calibration. The coordinate is
motor encoder angle. Thigh/shin position coupling, motor polarity and existing
individual-joint torque mapping stay unchanged. This triplet is not silently
substituted on twelve independent anatomical joints. Save shadow C/angle outputs.

## Fixed setup and explicit assumptions

Response gain is locked1.0. Same v6.22.2/model_6999.pt,55D/12D,
reference0/+34/-85deg/scale0.25, trot1.7Hz/duty0.6, PhysX1.024Nm/2.3rad/s,
20ms control and contact sensor5ms/history3. Default command0/0/0;
`--command` changes requested motion only. Linear XYZ observation stays0,
matching the real CLI ablation; IMU/gravity/joints/last actor action remain.

Existing ASSUMED motor model, not identified real hardware:

```text
drive = PWM * physical_PWM_polarity /1000
ideal: tau = clamp(1.28 * drive, +/-1.024 Nm)
dc: tau = clamp(1.28 * (drive - motor_velocity/2.3), +/-1.024 Nm)
```

The1.28 constant and DC damping term were not measured from PWM/current/torque.
A1s native25/0.5 standing warmup, optional clone synchronization and optional
logged initial I are experimental choices. They do not reproduce unknown real
hand-support forces/initial conditions. This removal does not tune those choices,
mapping, checkpoint, training defaults or physical hardware.

## Guards and interpretation

The existing512raw following-error guard remains and NOW checks the DIRECT goal.
Raw0..4095 and I/PWM caps remain. Communication/voltage/temperature stop logic
is not fully emulated. No-contact does not validate every firmware safety path.
Evaluate the original base_link force-history>1N termination predicate. Suppress
only auto-reset; first contact/PID guard is terminal for statistics. Exclude
later samples. Contact timing resolution20ms; error uses pre-output measurement.
Measure actual velocity/displacement before calling a test successful walking.

Record exact script/config, hashes, gains, mapping checks, raw goals/reference/
measurements/PWM, angular PID terms, force history and root state. Require the
completion marker. `AUDIT.md` in the new artifact folder lists other separate
implementation choices and request provenance; this change does not remove them.

## Historical results

`real_sim_tracking_20261027`, `error_contact_sweep_20261027` and
`inverse_pid_test_20261027` preserve old Speed/ACC-profile data/libraries/source.
Their results are not current direct-goal PID results. Use saved old scripts to
reproduce history. `diagnose_real_tracking.py` is also a historical profile/native
PD comparison, not the current PWM controller. Response0.320/0.318 runs additionally
changed the motor model and are not PID-only evidence.
