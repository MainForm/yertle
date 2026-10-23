# PWM/PID mode preparation diagnostics

Existing schema2/capability4 MODE0x0E, status EVENT4 and four-byte ACK/NACK
are unchanged. MODE_FAILED remains14. No new host command or arming bypass.

Mode completion now publishes EVENT5/version1 (64 bytes, little endian;
layout in firmware/include/pwm_mode_diagnostics.h). It identifies request
sequence AND epoch, backend, first failing phase, motor ID, register address,
expected/observed values, I/O status domain, primary read/write statuses and
addresses, last read value, mode transaction-completed mask, independently
verified mode mask, torque unknown mask at failure, EEPROM lock unknown mask,
timings, transaction status/stage and lock-recovery summary.

Stages: torque_off_tx, torque_off_verify, mode_transaction, mode_verify,
zero_pwm_tx, final_torque_off_verify, complete, canceled. Motor ID0 means no
specific motor identified (for example a broadcast TX failure). Byte/status255
means unavailable, NOT measured zero. Status domains motor_device and
motor_register have different numeric values and must not be conflated.

Failed mode-register transactions also publish existing0x90/version1/64-byte
REGISTER_WRITE_DIAGNOSTICS, correlated by MODE request sequence + motor ID +
address0x21. It preserves primary vs cleanup error, first lock failure,
read/write/servo errors, retry counts and lock time budget. Its completed/failed
counters are0 for this dedicated MODE path, not generic register-job counters.
Normally the diagnostic pair is sent BEFORE the correlated ACK/NACK. A canceled
transaction that is still unwinding may publish detail after its earlier NACK.
GET_UART_STATS also replays the last completed mode detail (until next MODE).
Replay is observational and may repeat identical sequence/epoch records.

The first failure is retained even if final torque-OFF verification also fails.
Later safe-stop verification remains in EVENT4; transaction_ok_mask is NOT a
claim that all motors are still in one mode after a partial transition.
No rollback, retry or timeout was added. Mode transactions, lock restoration,
servo error rejection and torque-OFF readback retain their prior behavior.
Current threshold enforcement remains PC-only. PID gains/slew/PWM ceiling,
motor ID/sign/raw calibration, policy and observation semantics are unchanged.

Validation: tests/pwm covers every preparation phase, partial completion,
primary-error preservation, cancellation, replay and diagnostic-before-NACK
ordering. tests/diagnostics exercises the real mode driver on simulated bus
memory, including target WRITE loss, mode readback mismatch and lock recovery.
These tests do not certify hardware mode transition or physical motion safety.
An unoptimized Cortex-M3 -fstack-usage audit reports actuator_task376,
pwm_control_service312, prepare_mode296, write_register_transaction144,
read_for_report24 and write_for_report32 bytes individually; host task120,
service_pwm72 and send_pwm_mode_diagnostics152 bytes individually. These are
per-function static frames, NOT runtime high-water or complete worst-case sums.
The existing actuator2560-byte/host2048-byte task allocations are unchanged.
The matching PC CLI logs raw payload and structured diagnostic fields; old
clients can ignore EVENT5 and0x90 without treating either as a control ACK.
