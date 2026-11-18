# PWM/PID mode preparation diagnostics

Schema3/capability8 MODE0x0E and status EVENT4 are defined in MODE_TRANSACTION.md.
The additive detail codec and four-byte ACK/NACK
are unchanged. MODE_FAILED remains14. No new host command or arming bypass.

Mode transitions publish EVENT5/version2 (64 bytes, little endian;
layout in firmware/include/pwm_mode_diagnostics.h). It identifies request
sequence AND epoch, backend, first failing phase, motor ID, register address,
expected/observed values, I/O status domain, primary read/write statuses and
addresses, last read value, verified mode mask, torque unknown mask at failure,
EEPROM lock unknown mask,
timings, transaction status/stage and lock-recovery summary. Bytes 14–15 retain
the old transaction-completed field and are zero in current firmware.

Current stages are torque_off_tx, torque_off_verify, mode_transaction,
mode_verify, final_torque_off_verify, complete, canceled and rollback.
Stage 5 (zero_pwm_tx) is reserved for older captures. Motor ID 0 means no
specific motor was identified. Byte/status 255 means unavailable. The device and
register I/O status domains have different numeric values.

EVENT5 carries mode-transition detail. When a mode-register write fails during
a transition, byte 30 marks the embedded register report. Each actual
SET_MOTOR_REGISTER request separately emits 0x90/version 1 (64 bytes) with its
own request sequence, motor ID, register address and host-task write counters.
0x90 precedes the register ACK/NACK. The pending EVENT5 is serviced before the
mode ACK/NACK in the same host pass, and a rollback detail can follow later.
GET_UART_STATS can replay the cached 0x90 and current EVENT5 detail.
Replay may repeat identical sequence/epoch records.

EVENT5 retains the first failure when the final torque-OFF recheck also fails.
EVENT4 reports the current verified mode and unknown torque masks, including
after rollback or safe stop. An unsuccessful BOOT check reports FAILED in
EVENT4 without a mode request sequence.

Validation: tests/pwm covers every preparation phase, partial completion,
primary-error preservation, cancellation, replay and diagnostic-before-NACK
ordering. tests/diagnostics exercises the real mode driver on simulated bus
memory, including target WRITE loss, mode readback mismatch and lock recovery.
These tests do not certify hardware mode transition or physical motion safety.
The F103 build checks static stack usage for selected actuator and host call
paths. Hardware stack high-water readings come from the running firmware.
The matching PC CLI logs raw payload and structured diagnostic fields; old
clients can ignore EVENT5 and0x90 without treating either as a control ACK.
