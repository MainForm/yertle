> Historical experiment, retired 2026-10-26. F7, operation3/capability0x100,
> EVENT6 production emission and the separate test overlay are removed.
> The observations below are retained as evidence; the old run/flash instructions
> no longer apply. Normal same-epoch rollback recovery remains supported.

# F7 torque sequence and same-boot rollback recovery

## Protocol and behavior
- No new message type. Existing SET_CONTROL_MODE (0x0E), schema 3, gains operation 3 = SEQUENCE_AUDIT. HELLO capability 0x100 identifies support (normal firmware bits now 505).
- F7 no longer calls normal COMMIT. It PREPAREs/snapshots, uses existing 0x0D mode writes, then requests AUDIT while DISARMED and STAGED.
- AUDIT sends torque OFF and checks the baseline/modes, records EVENT6 step 1; sends PWM0, records step 2; explicitly sends torque OFF, records step 3. Step 2 torque=1 is recorded, never a failure on its own.
- Step 2 read/TX errors still permit the final explicit OFF attempt and its measurement. Missing reads or failed final OFF keep the test failed and block ARM.
- Successful audit remains STAGED, never writes F103 preference Flash and never ARMs. PC sends the existing CANCEL to restore the original servo mode and then independently checks DISARMED. Measurement completion and restoration success are logged separately.
- F5/F6 keep their existing configuration/COMMIT path. A normal COMMIT's strict final-OFF check is intentionally unchanged.

## Recovery
- Failed rollback retains the original servo snapshot, prior Flash preference, epoch/backend/mask and rollback_failed.
- Existing same-epoch CANCEL is admitted from FAILED + rollback_failed. It retries OFF, mode restoration, final OFF/lock verification, and prior Flash preference restoration if needed.
- Only successful verification clears rollback_failed and releases the snapshot. No request-receipt-only reset.
- Link loss/release cannot erase failed-recovery context. Wrong-epoch CANCEL is rejected.
- PC receiving NACK21 to PREPARE queries the failed transaction, CANCELs that actual epoch once within the user's total deadline, verifies ACK + CANCELED/OFF status, then starts a fresh PREPARE.
- Failed recovery sends no new PREPARE/register writes. Flash-read failure (20) remains separate and is not cleared by recovery.

## Validation
Offline only; no COM port, physical PWM experiment or flashing was performed.
- F103 native suite: 5/5 cases. Includes repeated rollback failure then successful same-boot recovery, wrong epoch, preserved snapshots after link loss, Flash restoration retry, A=OFF/A=ON followed by B=OFF, PWM TX failure cleanup and final-OFF failure blocking.
- PC selected suite: 5/5 cases, including automatic recovery success/failure, old-firmware audit refusal before writes, existing PC-owned retry behavior and actual executable F7 physical-key injection in an isolated mock console.
- Preserved legacy isolated audit hypothesis test: 1/1.
- Normal ARM firmware and normal CLI Release build passed. New ARM frames: sequence_capture 80 B; prepare_mode 448 B; pwm_control_service 440 B. Existing sensor stack budget checks passed.
- The mock A=ON result is NOT evidence about a real STS3215 motor. Flash and run F7 to obtain actual data.

## Reproduce on hardware
1. Flash build/stm32f103rb/firmware/yertle_f103rb_F7_RECOVERY.bin (also identical to the normal yertle_f103rb.bin in the same build).
2. Run the rebuilt normal robot_policy_cli.exe with --port COM10 --baud 1000000 --enable-control --control-backend f103-pid.
3. With the robot supported/unloaded, F3 CONNECT, then F7 directly. F5 is not a prerequisite.
4. Inspect sequence_audit_step for step 1/2/3, per-motor torque_switch, and sequence_audit_result complete/restored separately.
5. A successful F7 leaves control released; use F5/F6 before H if continuing normal control.
6. The old optional sequence overlay lacks capability 0x100 and is deliberately rejected by this F7 implementation.

