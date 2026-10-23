# Dual position / RAW PWM contract v1

The legacy position path remains the default. RAW PWM is a deliberately limited
**single-axis supported-bench mode**, not walking, torque regulation, or an outer PD.
No flashing or physical operation is performed by a build/test command.

Only two new request types: SET_CONTROL_MODE0x0E and SET_MOTOR_PWM0x0F.
Existing ACK/NACK/EVENT0x83 are reused. HELLO_ACK capability bit0x02 advertises
the complete feature; do not advertise it for a contract-only/incomplete build.

## Bytes (little endian)

- MODE8B: schema1 u8, backend u8 (0 native,1 RAW), mask u16, epoch u32.
  Epoch is a fresh nonzero host-generated token, never a fixed MCU-boot counter.
  RAW requires exactly one motor; native restoration requires mask0x0FFF.
- PWM37B: epoch u32, command_id u32, mask u16, TTL_ms u16, flags u8,
  signed int16 PWM[12] in motor-ID order. Normal flag0; RELEASE flag1.
  Normal TTL20..100ms, duty -100..100 (10%), nonselected entries0. Reject out of
  range rather than silently saturating protocol input. The selected mask must
  equal the prepared mask. Duplicate/older IDs never execute or refresh a TTL.
- RELEASE canonical mask0,TTL0,all PWM0. It never arms, ignores epoch/lease for
  stopping, and invalidates prepared state before acknowledging. Normal PWM0
  is NOT release. Stop takes priority over queued motion, retries are idempotent.
- EVENT38B: code4,schema1,backend,phase,request_seq u16,result,stop_reason,
  epoch u32,mask u16,verified_mode_mask u16,torque_requested_mask u16,
  torque_unknown_mask u16,last_command_id u32,duty_limit u16,TTLmax u16,
  travel_limit u16,feedback_age_ms u16,last_sent_PWM i16,MCUtime_ms u32.
  Query cached status through existing GET_UART_STATS; never poll EEPROM while moving.

Mode ACK is returned only after transaction completion + independent mode/lock
readback. Target ACK means F103 acceptance, NOT motor delivery or torque. RELEASE
ACK means request accepted; only status/readback can confirm torqueOFF. UART loss
must remain unknown, never fabricated OFF. Existing message layouts stay unchanged.

## Safety invariants

Mode transitions only DISARMED/torqueOFF, mode0x21 is not added to generic SET
whitelist. Preserve original lock, track partial failures, block arm on uncertainty.
Native arming requires actual native-mode verification, not software defaults.
PWM needs new HELLO→explicit mode preparation→heartbeat/ARMED→fresh sensor + target.
No heartbeat-only PWM arm, no automatic mode2→position-center fallback.

Immutable initial bench limits: max duty10%, TTL100ms, sensor age50ms, selected
shaft travel32 steps from arming anchor, total powered session1000ms. Nonselected
motors torqueOFF. Current guard >=308raw per motor or abs sum>=924raw, plus fresh
sensor/servo/error/speed/voltage/temperature/deadline checks. No claiming these
software checks are an instantaneous current clamp or calibrated safe operating point.

PWM stop: reject goals→PWM0→torqueOFF→readback. Never issue center2048, re-enable
torque during fault, or reuse legacy positional safe-stop for RAW. MCU/bus failure
still requires independent power cutoff and physical support. Existing position
PID/startup configuration and calibration are not silently changed.

The vendor's old embedded table says direction bit11; current official STS table
says BIT10(0x0400). Unit tests use the latter; actual low-duty direction/zero/OFF
validation is mandatory before loaded use. No claim of hardware verification.

External position PD/policy integration is deliberately deferred until the manual
PWM bench gates pass. Internal servo PID registers do not configure an F103 PD.

Reference: https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sts/
