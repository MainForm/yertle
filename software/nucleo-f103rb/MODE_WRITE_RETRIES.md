# PC-owned mode retries; one firmware transaction per request

motor_device_write_mode uses write_mode_once: one 0x0D request executes one
torque-OFF/read-mode/read-lock/unlock/write/readback/relock transaction. F103
does not repeat the target write. The PC decides whether to send a new request.
Generic register writes are unchanged. Mandatory lock recovery remains bounded
to two attempts/100ms; this is safety cleanup, not a target-write retry. Rollback
also executes one transaction per ID within its existing bounded cleanup budget.

For a safe transient IO14 or mismatch15 failure with no servo fault and verified
lock cleanup, the transaction stays DISARMED/STAGED for an explicit PC retry or
CANCEL. Unsafe failures roll back immediately. No ARM is allowed while STAGED.
DISARMED preparation TTL is120000ms. Silent-link rollback remains active:
20000ms only while DISARMED PREPARING/STAGED,1000ms elsewhere including ARMED.

PC defaults: max5 requests per ID,5000ms response wait,200ms retry delay,
30000ms per-ID budget,120000ms overall timeout; proof register reads2s/attempt.
Budget or safety checks may stop
before attempt5. A missing ACK is reconciled through fresh epoch/STAGED/FSM,
lock, torque-OFF and mode reads; an applied value is not blindly written again.
BUSY reconciliation only repeats reads, never concurrent target writes.

register_write_diagnostics0x90 v2 appends attempt count/limit, IO detail and first
mode status at bytes60..63. Firmware mode attempts/limit are now1/1. This is
separate from max2 LOCK cleanup and the PC's mode_register_attempt events.
EVENT5 v2 appends IO detail/attempts/limit at61..63. External0x0D failure now records
the actual failed ID/stage instead of replaying a successful PREPARE diagnostic.
A separate rollback-stage9 EVENT reports the restored mask and its first failing
ID/report, without destroying the primary cause. ACK/NACK and mode schema3 stay
unchanged. HELLO0x40 signals PC retry;0x80 signals long preparation (combined0xD9).

The CLI's pinned API accepts diagnostic v1 and v2, and requires capability0x40
and0x80 for standalone mode configuration. Old0x39/0x59 firmware is rejected before writes
to avoid multiplying PC and firmware retries. Old readers must decode
v2; normal ACK/NACK and position-goal protocol are unchanged.

Mock coverage is in tests/diagnostics/pwm_driver_test.c and tests/pwm/
host_integration_test.c. Verify actual timing/reliability on the board separately.
