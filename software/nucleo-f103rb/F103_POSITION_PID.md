# F103 PWM / direct-goal position PID (current implementation)

PC retains SET_JOINT_TARGETS0x06: twelve position-u16 / Speed-u16 / ACC-u8
records. Native position mode still forwards Speed/ACC to the servo. In the
PWM backend the actuator uses **received absolute goal minus calibrated
measured position immediately**. Speed/ACC are NOT inputs to position_pid_step;
there is no reference velocity, acceleration/braking profile, output slew
limiter or derivative low-pass filter. Protocol fields remain for native-mode
compatibility. PID telemetry's reference field now means the received goal.

## Controller

- P10, I3, D0.13; I-output cap100 PWMraw, output cap900/1000.
- Both ordinary PWM position control and return-to-2048 use these defaults.
- Signed error; calibrated encoder raw0..4095; no modulo/absolute-error control.
- D is negative measured-position derivative. Fresh-sample MCU dt is used;
  repeat samples do not integrate again. New goals retain controller history;
  Ki=3 accumulates error subject to the I-output cap and anti-windup.
- Conditional integral anti-windup at output saturation remains.
- All12 physical PWM polarity values are-1, independently of joint/raw signs.
- The measured position uses cached position-mode calibration in PWM mode.
- Mode changes, stop and an explicit direct-PWM-to-PID restart reset state.

These gains are F103 software gains, not servo EEPROM PID register values.
No API mapping, ONNX tensor layout, action reference, model or motor register
is changed by removing the reference generator.

## Modes and wire contract

SET_CONTROL_MODE0x0E retains PREPARE/COMMIT/CANCEL with schema3. PC performs
mode-register SETs through SET_MOTOR_REGISTER0x0D. Transaction verification,
Flash preference/calibration and pre-arm rollback remain. H does not change mode.
SET_MOTOR_PWM0x0F also exists: PWM mode accepts signed direct duty; position
mode rejects it. PWM mode accepts0x06 positions through this F103 PID.
Mode ACK is preparation completion, not arming or physical arrival.

PWM0 can enable servo torque. Stop/preparation must separately send explicit
torqueOFF and read it back; PWM0 alone is never treated as OFF proof.

## Current independent stop conditions

- Target TTL500ms remains. Every accepted0x06 or armed direct0x0F command
  refreshes it, even if the target value is unchanged. Motor position feedback
  and sensor queries do NOT refresh target time.
- The user removed the30ms actuator-service deadline and50ms motor-feedback
  age/PID-interval cutoffs in active PID, direct PWM and2048 return paths.
  No larger replacement ceiling, dt clamp or fixed-dt substitution is used.
  Actual positive forward MCU time deltas are used; duplicate samples do not
  integrate. Zero/backward time and nonfinite/invalid data are still rejected.
- Goal/current raw must remain0..4095; absolute PID goal error>512raw rejects
  the step. Normal control uses the direct received goal; return-to-2048 uses
  a feedback-limited reference advanced toward center.
- All12 feedback valid, no servo error, voltage greater than 4.0V and at most 7.4V.
- Same motor>=60C on three successive fresh feedback sets triggers stop.
- Current limits are PC-only: any motor>=2A or sum of absolute feedback>6A.
  F103 has no duplicate per-motor/summed current cutoff.

Some numerical guards above were implementation choices rather than values
explicitly requested by the user. They are disclosed in the workspace's
`artifacts/pwm_direct_goal_20261027/AUDIT.md`; they are not silently removed.

PWM stop first requests2048 return on eligible previously driven/calibrated
axes, then explicitly zeroes PWM, torqueOFF and verifies OFF. Failed or
ineligible return falls back to OFF without re-arming. Existing return bounds
are15s, no progress500ms, progress12raw, arrival within16raw for3fresh sets.
The512raw direct-goal guard and feedback/voltage/temperature guards still
apply. PC waits up to20s for completion. See audit for limitations.

## Validation and build

`tests/pwm` covers first-tick direct-goal P/I output, immediate reversals,
unfiltered measurement D, saturation, retained anti-windup, resets, polarity,
synthetic convergence, host/actuator integration and native wire contract.
`docker compose run --rm f103-build` builds the unchanged artifact name:
`build/stm32f103rb/firmware/yertle_f103rb.bin`.

Old reports/artifact snapshots describe historical profiled controllers.
They remain reproducible history and do not describe this current binary.
No board flashing or real motor run is performed by this build/test workflow.

## PID detail timing after removing the interval ceiling

EVENT7 detail schema2 still uses SIX64-byte ID-pair payloads per12-axis set:
18-byte header with actual dt as u32 milliseconds, followed by TWO23-byte
axis records (five floats, signedPWM,flags). Removing the former per-axis
reserved bytes keeps traffic length unchanged. EVENT8 statistics stays v1.
PC accepts both old schema1 and new schema2, including dt>255/65535ms;
the numerical PID uses the actual interval, not a byte-truncated value.
The status field feedback_age is diagnostic u16, saturated at65535 (not a
control cutoff). Thermal debounce still requires3successive fresh sequences,
but no longer resets solely because their time gap exceeded50ms.

Scope: these changes remove F103 PWM timing cutoffs. PC sensor-request timeout,
50ms host/input age checks, heartbeat/target watchdogs, invalid-data checks,
temperature/voltage/current rules and return progress/total timeout remain.
Native position-mode150ms return feedback rule is also unchanged.
