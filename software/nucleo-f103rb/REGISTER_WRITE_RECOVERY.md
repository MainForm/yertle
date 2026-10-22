# Bounded EEPROM lock recovery and PC diagnostics

Only the saved-original lock register0x37 can be retried, at most2 iterations
per recovery invocation, nominal100ms total budget. IO budget is capped10ms
per UART TX/RX wait and reduced as the budget runs out; task scheduling can
overrun wall-clock budgets, so this is not a hard-real-time guarantee.
After lock WRITE wait2ms before independent GET. These are conservative software
settings requiring physical validation, not manufacturer-guaranteed timings.
Motion/feedback UART waits remain5ms and50Hz scheduling is unchanged.

Read first; do not WRITE if the saved lock value is already verified. A lost
lock WRITE ACK can be resolved by a successful independent GET. A servo fault
still fails cleanup. The user target WRITE is NEVER automatically repeated.
Persistent failure retains the original lock value/uncertain-mask and blocks
ARMED. A NEW explicit supported SET can recover the saved lock before checking
whether the target already equals the request. Recover-before-target and later
cleanup can each run2 iterations: diagnostic counts may total4 for one request.

The most recent completed job and each60s replay-cache entry retain a64-byte
`MOTOR_REGISTER_WRITE_DIAGNOSTICS (0x90)` frame. Emit before completion ACK/NACK,
replay with duplicate results without rerunning motor I/O, and append the latest
snapshot to GET_UART_STATS. Existing protocol version/ACK/NACK layout is unchanged.
Diagnostic send failure must not turn a successful ACK into failure; clients
report the missing optional detail. Existing completion ACK retry FIFO remains.
Validation rejections before worker acceptance have no new completion diagnostic.

## Version1 payload, little-endian

|Offset|Width|Meaning|
|---:|---:|---|
|0|1|Schema1|
|1|1|Flags: valid1,target-attempted2,lock-write-attempted4,lock-verified8,budget-exhausted16|
|2|2|Original SET request sequence|
|4..6|3|Motor ID, target address, target width|
|7..9|3|Final status, primary target status, failure stage0..9|
|10..11|2|Primary and cleanup servo errors|
|12..15|4|Expected/last actual lock, last lock READ/WRITE status|
|16..19|4|Iterations, last READ/WRITE servo errors, first failure reason|
|20,24|4 each|Total elapsed and accumulated recovery elapsed [ms]|
|28,32|4 each|Host completed and failed job counters (replay does not increment)|
|36..39|4|First failure READ/WRITE status, actual lock and servo error|
|40|4|Lock WRITE attempts|
|44,48,52,56|4 each|Configured lock UART wait, recovery budget, settle [ms], attempt limit|
|60|4|Reserved zero|

Unavailable status/lock bytes are255, never fabricated0. IO status0..4 uses the
GET register status enum, not NACK codes. First failure reasons:0none,1read,
2write,3mismatch,4servo,5budget. Failure stages:0none,1torque,2response-level,
3recover-lock,4read-initial,5read-lock,6unlock,7target,8verify,9restore.
The host-owned cache is static storage rather than the2KiB task stack.

## Verification and hardware boundary

Native driver/host tests cover transient restore failure, persistent failure,
lost lock ACK with valid GET, budget exhaustion, explicit recovery, exact-width
0x18 and preserved0x19, replay and unchanged safety gates. PC tests decode the
actual C encoder, reject malformed/foreign reports, preserve NACK17 despite
matching target GET and log diagnostics on success/failure. Firmware ARM build
and native/mock tests do not certify physical servo timing.

No flashing or real motor writes are part of this change. Last physical result
before implementation: ID1 startup48/lock1, ID2 startup48/lock0, ID3 startup16/lock1,
all torque OFF. An F103-only reset loses its RAM recovery state while powered
servos may remain unlocked. Recover/verify ID2 with the currently running
firmware before resetting/flashing; do not treat a reset as lock recovery.
