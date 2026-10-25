# Explicit torque OFF after PWM0 (2026-10-26)

## Evidence
Real logs robot_20261026_000013_14644.jsonl and robot_20261026_000845_13376.jsonl:
all torque switches0 before PWM0, ID1=0 and IDs2-12=1 after PWM0, all0 after explicit torque OFF. PWM0 is a zero-output command, not verified torque release.

## Production correction
Normal SET_CONTROL_MODE COMMIT for the F103-PID backend now:
1. Initial OFF and readback, then servo-mode verification.
2. PWM0 transmission.
3. Explicit torque OFF for all12 motors, even if the PWM TX failed/was partial.
4. Independent OFF readback.
5. Only if all operations succeeded: persist preference and publish READY.
Boot restoration of a saved PWM backend uses the same PWM0 -> explicit OFF -> OFF readback order.
The first primary error remains in diagnostics; a later successful OFF read does not erase a PWM/off TX failure. Preparation/boot failure does not allow ARM. Native-position preparation does not send PWM.

## Retired experiment
F7, :sequence-test, its API, operation3/capability0x100, EVENT6 emission and separate source-injection firmware are removed in dedicated removal commits. Former test sources remain recoverable from Git; raw hardware logs and historical notes are preserved. Protocol schema3 still has PREPARE/COMMIT/CANCEL, and production HELLO capabilities return to249. Same-epoch rollback recovery remains.

## Verification
F103 native suite5/5 (control, host integration, PID, wire contract and Flash journal).
Regression cases emulate torque activation on PWM writes, preparation and boot readiness with torque OFF, PWM TX failure cleanup, OFF TX failure and final OFF-read failure. Host integration independently models PWM enabling torque and checks configure-only never arms.
PC suite4/4: PC-owned mode retries, idle configuration/recovery, native key handling including inactive F7, actual executable F5/F6 without motion goals.
Standard ARM build and sensor stack-budget check passed. Standard CLI Release build passed.
No real COM session, hardware commands or flashing were performed in this change.

## Additional items
- Hardware validation: flash normal build/stm32f103rb/firmware/yertle_f103rb.bin, then F3 -> F5. Verify READY/DISARMED and torque OFF for all12 before H. Reboot and verify saved-PWM startup as well.
- Direct PWM0 (:pwm-zero / 0x0F) still means zero output. Its existing DISARMED zero-preview exception can activate servo torque; it reports torque state as unknown and disables ARM permission. Decide whether this bench exception is still required or should be removed/replaced with explicit release. It is not changed here, so active direct-PWM semantics remain stable.
- Ordinary release and rollback already send explicit OFF and verify it. Keep these guards.
- The observed first ID4 restore-write response error is distinct from the final-OFF sequencing fix. Subsequent retry succeeded. More USART1/servo response detail would be required to distinguish timeout, receive error or invalid reply; do not infer a persistent motor or stack fault.

