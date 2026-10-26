# PWM position reference normalization

## Position convention

`calibrated = (wire_position - signed_calibration_offset) mod 4096`

Offsets retain the currently supported STS encoding: bit11 sign, bits0..10
magnitude. IDs and motor API mappings are unchanged. This is a zero-reference
correction, not PWM polarity reversal, position target clamping or re-calibration.
Mode-0 position feedback is already calibrated and must not be corrected again.

## F103

The sensor cache and MOTOR_FEEDBACK 0x8B payload remain original motor values.
Only masked mode-2 PID axes convert the current position, using the RAM offsets
loaded from Flash/updated on reads. The converted position initializes the PID
history and is used for position error and measured derivatives. Targets, gains,
limits and direct PWM commands are unchanged. No Flash or UART access is added
to this conversion. Missing calibration blocks PWM PID arming and, if a PID
cycle is nevertheless reached, stops it with reason15 CALIBRATION before a PWM
write. Stopping/zero/off does not depend on calibration being available.

## Active PC CLI/API

The CLI enables `Client::usePositionModeCoordinates()` once on connection.
Raw tools retain their prior behavior unless explicitly opting in. The same
Client is used by monitoring, H/current hold, posture arrival guards, foot
kinematics, state estimation and the ONNX controller, so these paths receive
the same position-mode coordinates. No model/checkpoint, action scale, motor
ID/sign mapping, pose reference or 55D observation layout is changed.

Mode inspection reads actual 0x21 for each ID and retrieves offsets with
GET_MOTOR_REGISTER 31/2 for mode-2 IDs. PWM preparation reads offsets while
DISARMED before mode writes. F8 remains the explicit user-triggered capture
that writes PC calibration JSONL. Cache/mode changes discard old sensor
assemblies. Each public snapshot is normalized as a COPY of the raw assembly;
repeated latestSensors() calls cannot double-apply an offset.

`MotorSample.position` is the usable position, and `wire_position` preserves
the original encoder reading. Snapshot masks record PWM, corrected and
unavailable position references. An unknown mode/missing offset yields
position=-1 and UI `--`, never a fabricated zero offset. Motor current/load
validity remains intact so the existing PC overcurrent guard still operates.
Telemetry records position, position_wire and position_reference per ID;
protocol payload logs remain original bytes.

Updated F103 advertises capability0x400 (calibrated PWM PID); PC rejects PWM
preparation/arming against a firmware without it. Native position mode remains
usable. Deploy matching firmware and CLI together; neither is flashed or run
against hardware by these mock/host tests.

## Verification

Recorded offsets -963/+1328/+1432 turn PWM center readings1085/3376/3480 into
2048. F103 service tests require zero PWM when the target is2048 in those
cases, and stop before output with missing calibration. Wrap and invalid
range cases are tested. PC tests compare the same physical pose in native,
PWM and returned-native modes, preserve wire values, repeat getters and
exercise old-firmware/storage failure rejection. CLI native/PWM ONNX mock
runs validate the full control/policy path; these are not physical direction
or dynamics tests.
