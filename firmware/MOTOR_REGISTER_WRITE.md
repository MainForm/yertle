# SET_MOTOR_REGISTER — firmware implementation

This is the F103 side of the shared 2026-10-20 SET_MOTOR_REGISTER contract.
PC API/CLI changes, flashing and physical writes are separate work. Protocol
v1, flags=0, COBS/CRC16, USART1/USART2 DMA and 1 Mbps are unchanged.

## Wire format

Request type: `0x0D`. Payload is exactly `4 + length` bytes:

```text
u8 motor_id        # 1..12, never broadcast
u8 address         # start address
u8 length          # 1..32, address+length <=256 without u8 wrap
u8 options         # bit0=ALLOW_EEPROM; bits1..7 must be zero
u8 data[length]    # opaque bytes; 2-byte allowed values use little-endian
```

Only the exact register widths below are currently allowed; a range spanning
multiple registers is not supported. The 32-byte wire bound is not an arbitrary
32-byte write permission. Example payload `01 15 01 01 20` encodes ID1,
address 21, one byte 32, EEPROM explicitly allowed. It is a protocol example,
not a recommended PID setting or a physical write instruction.

ACK/NACK payload is unchanged: `[0x0D, request_sequence_lo,
request_sequence_hi, code]`. There is **no acceptance ACK**. ACK(code 0) means
a fresh read confirmed equality and any required EEPROM lock restoration was
verified. It does not certify persistence after power cycling. Failures or
lost replies may follow an already-applied write: applied state is UNKNOWN
until separately queried, never assumed unchanged. Correlate by payload request
type/sequence, not the response frame sequence.

| Code | Meaning |
|---:|---|
| 2 | INVALID_LENGTH: payload size does not match 4+declared length |
| 5 | INVALID_STATE: not DISARMED, pending stop/fault/timeout, state changed before write, or torque OFF could not be directly confirmed |
| 7 | INVALID_FLAGS: nonzero header flags (not payload options) |
| 11 | QUERY_BUSY: shared GET/SET/offset slot, bus mutex or write replay cache unavailable |
| 12 | INVALID_REGISTER: ID/span/length/options/exact field width/value invalid; retained request sequence reused with different payload |
| 13 | REGISTER_WRITE_FORBIDDEN: outside allow-list or missing EEPROM permission |
| 14 | REGISTER_IO_ERROR: TX/RX failure, timeout, invalid reply/ID/length/checksum |
| 15 | REGISTER_VERIFY_MISMATCH: fresh read does not match the requested value or unlock does not read back as zero |
| 16 | REGISTER_SERVO_ERROR: valid target/management reply with servo error bits |
| 17 | REGISTER_LOCK_RESTORE_FAILED: original EEPROM lock could not be restored/confirmed without servo errors |

No servo bits or read-back bytes are appended to ACK/NACK. Existing GET
`0x0C`/`0x8F`, offset query `0x0B`/`0x8E` and their status enums are unchanged.

## Allow-list and limits

| Register | Address | Width | Raw value | Storage / permission |
|---|---:|---:|---:|---|
| Maximum torque | 0x10 | 2 | 0..1000 | EEPROM, options bit0 required |
| Position P | 0x15 | 1 | 0..254 | EEPROM, options bit0 required |
| Position D | 0x16 | 1 | 0..254 | EEPROM, options bit0 required |
| Position I | 0x17 | 1 | 0..254 | EEPROM, options bit0 required |
| Minimum startup torque | 0x18 | 1 | 0..254 | EEPROM, options bit0 required |
| Current torque limit setting | 0x30 | 2 | 0..1000 | SRAM |

For the original five entries, widths/addresses match
`third_party/stm32-sts3215-lib/inc/sts3215_regs.h`; numeric ranges match its
`docs/sts3215_memory_table.md` and the manufacturer
[FEETECH STS memory table](https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sts/)
(checked 2026-10-20). Defaults are not used as maximum values. These torque
settings are output limits, not measured output torque.

For newly allowed **0x18**, use the current manufacturer table's **one byte,
0..254** definition. The vendored v3.6 header/table instead describes an older
two-byte 0..1000 field. Do NOT use that vendor width helper for this SET: the
current table assigns **0x19 to an independent integral-limit register**. Two-byte
0x18 requests are rejected with NACK12 to avoid overwriting 0x19; direct writes
to 0x19 remain forbidden. Payload example for ID1/startup16 is `01 18 01 01 10`.
The PC API must use the same one-byte policy. No model-version auto-detection is
added; confirm register layout before use on a device with older firmware.

All other addresses are forbidden, including ID, baud, response configuration,
mode/phase/direction, zero calibration, angle/protection limits, torque enable,
motion targets, direct lock writes and feedback/reserved addresses. Motion
must use SET_JOINT_TARGETS; this API cannot bypass that control path. ALLOW_EEPROM
on a permitted SRAM request has no extra effect.

## Runtime and failure behavior

1. Host task validates protocol/policy and safe DISARMED state, reserves the
   exclusive job slot and inhibits automatic ARMED transition. No UART1 I/O
   occurs in the host request handler and no heartbeat/target deadline renews.
2. Sensor worker rechecks state. Under the existing bus mutex it reads the
   target motor's torque switch (40): only exactly zero permits writing.
   A missing/corrupt/error reply or nonzero torque value returns code 5 and
   performs no target or unlock write. It never turns torque OFF automatically.
3. Read response level (8) without modifying it. Level 0 expects no WRITE
   response; level 1 requires a valid zero-data response from the matching ID.
   Both require a separate fresh read-back. Other values are rejected.
4. Read current target bytes and, for EEPROM, original lock (55, 0 or 1).
   Already-equal values skip target WRITE/unlock; equality was freshly read,
   and EEPROM lock state was validated. No EEPROM wear is added in that case.
5. For changed EEPROM data, save the original lock before attempting unlock;
   mark restoration unverified. If originally 1, WRITE 0 and verify unlock.
   Recheck the state callback before target WRITE. Issue at most one ordinary
   vendor BuildWriteRaw/WRITE_DATA(0x03) to the user's target register. Never
   use broadcast, SYNC_WRITE, REG_WRITE or ACTION for this feature.
6. After an EEPROM target write allow a bounded 10 ms settling interval, then
   read the exact target width and compare bytes. The delay is an implementation
   guard, not a guarantee of nonvolatile persistence. No automatic target retry.
7. On success OR failure after possible unlock, perform bounded cleanup under
   the same bus mutex even if a new fault/stop arrived. Read the original lock;
   if necessary attempt one restore WRITE and read it again. An independent
   valid matching read-back can confirm restoration even if its WRITE ACK was
   lost. Servo error bits are retained and cannot be silently converted into
   a successful cleanup. Preserve an originally open lock as open, not locked.
8. Worker publishes the final report, without holding the snapshot mutex
   during UART waits. Host task queues the final ACK/NACK in the existing result
   TX FIFO. Result-TX retry never runs the job again. Normal host safety
   processing and IMU/feedback remain active; the short maintenance transaction
   holds the bus and can delay a feedback cycle. It is not for ARMED operation.

Driver uses private `_locked` reads/writes, never recursively calls the public
read API while holding the nonrecursive bus mutex. Bus acquisition waits at
most 20 ms; each UART TX/RX wait is at most 5 ms plus scheduler latency. There
is no endless cleanup loop. A safety request prevents subsequent target writes;
an already-started hardware write cannot be undone. Cleanup restores the lock,
not the old target value: failures are not rollback guarantees.

## Lock uncertainty and diagnostics

`motor_device_register_lock_uncertain_mask()` exposes one bit per ID. Any
failed EEPROM cleanup leaves its bit set and the FSM's `can_arm()` returns
false, independently of the final reply reaching the PC. The saved original
lock value is not replaced by a later accidentally open reading.

A NEW explicit, permitted SET for that ID may first restore/verify that saved
lock before proceeding; it is not an automatic retry of the old target. A
separate valid GET of address55/length1 also clears the bit if it confirms the
saved original state after an external recovery. Other GETs/heartbeats or ACK
transmission cannot clear it. The latch is RAM-only: it does not survive reset
or power loss and must not be described as crash-safe EEPROM journaling. After
a power/reset interruption, inspect actual register/lock state before control.

`motor_device_get_register_write_diagnostics()` provides a bus-mutex-protected
C snapshot of last ID/address, completed/failed counts and the report:
final status, primary failure before cleanup, failure stage, target and cleanup
servo error bits, target-WRITE-attempted and lock-restore-attempted/verified.
Storage is also debugger-inspectable as `register_write_diagnostics`; no new
diagnostic wire frame is added. Existing UART diagnostic reason counters still
record transaction failures. PC obtains actual bytes via a separate GET.

## Duplicate suppression and bounded storage

The host task retains **four accepted SET payloads/results**, with completed
results retained for **60 seconds** (tick-wrap-safe). An in-flight entry never
expires. Identical type+sequence+payload received during execution creates no
second job and no acceptance ACK. A completed duplicate only requeues its
cached final ACK/NACK; a different payload using a retained sequence gets
NACK12. Replay of a completed result may occur after the FSM changes because
it performs no motor I/O. Pending request/result cannot be overwritten.

When all four entries are unexpired, new writes get NACK11, not eviction of an
entry that could permit a duplicate write. Retry only the response, not SET.
PC must never automatically resend a SET on NACK, timeout, partial TX or link
loss; use bounded GETs to establish current state first. After 60 seconds or
firmware reset/reboot, the cache cannot suppress old duplicates: choose fresh
request sequences and do not retransmit old writes. There is no reboot-safe
exactly-once guarantee. Reconnection does not clear the cache.

## Verification

Host tests cover the document payload vector, COBS/CRC round-trip, width/value
and state validation, exclusive jobs, pending/completed duplicate suppression,
different-payload sequence rejection, cache saturation/expiry, failed final
ACK local-TX retry without re-execution, safety cancellation and re-arm latch.
Driver tests use the real vendor builders/parser and a fake register map to
inject TX/RX failure, checksum, ID, length and servo errors at every normal
EEPROM step, plus mismatch, no-WRITE-response mode, no-op and cleanup recovery.
These tests and a build do not verify physical EEPROM writes or motor safety.
