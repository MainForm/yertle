# SET_MOTOR_PWM — 0x0F

PC/RPi sends a 24-byte payload: signed int16 little-endian PWM for motor
IDs1..12 in that order. Existing COBS/CRC/sequence framing is unchanged.
Accepted range is -800..800, preserving the existing driver output ceiling.
Values are rejected outside that range, never silently clamped or profiled.
The servo bus packet uses the existing signed-magnitude conversion and
SyncWrite register0x2C. No calibrated joint ID/sign/raw mapping is changed.

| F103 backend | SET_JOINT_TARGETS 0x06 | SET_MOTOR_PWM 0x0F |
| --- | --- | --- |
| Servo POSITION | Existing position packets | NACK22 pwm_mode_required |
| PWM / F103 PID | Existing position->PID->PWM | Exact requested PWM, bypass PID/profile/slew |

Nonzero PWM requires the existing ARMED lease and verified PWM readiness.
An all-zero DISARMED diagnostic is additionally accepted in PWM READY/STAGED,
after the actuator verifies the actual servo modes. It never sends torqueON
and invalidates ARM permission: PWM0 is not proof of torqueOFF. Normal
mode/OFF verification is required again before arming.

Direct writes run on the actuator task. ACK0x80 is deferred until UART TX
completes; failed TX is NACK14. This is not a per-servo ACK or proof that
torqueOFF stayed OFF; use GET_MOTOR_REGISTER for that independent observation.
Payload errors are NACK2/8, state errors NACK5 and pending jobs NACK9/11.
The unchanged HELLO gains capability0x20; the PC refuses older firmware
before sending a direct command.

The latest input type selects the output source. Position->direct bypasses
PID completely. Direct->position resets PID history to measured state once;
ordinary successive position commands keep their integrator/history.
Direct output retains target TTL, heartbeat loss, sensor freshness,
voltage/temperature checks and the zero/OFF stop path. Current thresholds
remain PC-owned. Two consecutive PWM ACK timeouts stop the PC heartbeat lease.

PC API: Client::sendRawPwm(MotorPwm, timeout).

CLI:

- :pwm-zero sends all zero once when released, or streams zeros if controlling.
- :pwm ID DUTY streams one selected ID at50Hz; all other IDs receive0.
  Requires an already active control session; does not ARM automatically.
- H returns to position posture control; F2 releases.
- DIRECT_PWM replaces policy/pose output while selected. Existing current,
  heartbeat and sensor guards remain active; signed requests are logged under
  direct_pwm_raw with normal correlated ACK/NACK records.

Production firmware implements this API. The earlier opt-in A/B overlay
continues to reject raw commands, like its existing ARM/position restrictions.
It is not the new direct-PWM production firmware.
