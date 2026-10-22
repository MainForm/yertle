# Yertle PC/RPi host protocol v1

## DMA transport measurement (0x8A)

GET_UART_STATS also returns MOTOR_TRANSPORT_STATS, 64 bytes of little-endian
u32 words: uptime_ms, mode (1 = F103 DMA port of Mowibox pattern), TX starts,
TX wire completions observed by task, UART IDLE events, RX buffer-full events,
TX DMA errors, RX DMA errors, SyncRead attempts, complete twelve-ID SyncReads,
feedback cycle attempts, complete two-block/twelve-ID feedback cycles, RX buffer
capacity (256), host RX mode (0=legacy ring, 1=circular DMA), host RX capacity
(1024 in circular mode), host RX DMA error count. No new request is needed.
Each counter has one serialized writer (bus task or equal-priority ISRs).
Complete SyncRead means all IDs 1..12 parsed; complete feedback means both
motion/current blocks for all twelve. Counts are since boot; PC summary subtracts
first/last samples and prints the actual MCU span. Boundary snapshots can catch
an in-progress transaction, so attempts/completions are not an atomic pair.
DMA full is fatal, not circular ring overflow. In DMA mode UART_STATS RX bytes
count bytes moved to memory, including discarded data and stop residue, not DR
reads by CPU. rx_interrupts counts UART IDLE/error events; UART_ISR_STATS port1
aggregates USART1 plus DMA1 Channel4/5 handler bodies. DMA errors are distinct
from UART Overrun/Framing/Noise/Parity flags. Do not interpret error-events per
DMA byte as corrupted byte count, bit error rate or packet failure rate.
USART2 circular mode uses DMA1 Channel6. Its RX bytes are DMA progress observed
by the host task, including later discarded bytes; rx_irq counts UART error
events only. Its ISR timing aggregates USART2 and DMA6 half/full/error handlers.
RX peak is task-observed backlog (capped at capacity), not an exact historical
hardware high-water mark. Overflows count detected DMA overwrite events, not
lost bytes. UART errors keep their existing dedicated flag counters. All six
diagnostic frames remain <=64 bytes. Older clients can ignore the last three
metadata words, but must not label the new buffer as a 255-byte IRQ ring.

## Motor transaction diagnostics

### UART receive-error detail (0x89)

GET_UART_STATS additionally returns UART_RX_ERRORS (0x89), a 64-byte payload:
u32 uptime_ms, then USART1 and USART2 each with seven u32 fields (ORE count,
FE count, NE count, PE count, last error SR, last error DR byte, last error
DWT cycle32), followed by u32 diagnostic snapshot maximum unmasked elapsed cycles.
All fields are little-endian. Counts accumulate since boot and wrap at 32 bits.
One error event may set multiple flags, so their sum need not equal rx_errors.
The SR/DR/cycle values are historical, captured in the ISR before clearing flags;
they do not describe the current UART state. DR data on errors is not reliable.
DWT cycle32 wraps and is not a timestamp in milliseconds.

Diagnostic copies and measurement hooks no longer mask interrupts. Snapshot
timing includes preemption during the copy, not an interrupt blackout or pure
CPU cost. Update firmware and PC client together: the final 0x89 field now has
this meaning rather than masked-body time. DWT availability is carried by
UART_ISR_STATS. No ISR RTOS API, printing or motor control change was added.
ISR-owned RX counters and task-owned TX/timeout counters are separate. The
host task reads independent aligned 32-bit fields; snapshots are not globally
simultaneous. ISR cycle accumulation uses two 32-bit words, read high/low/high
with up to three retries and a last-good fallback. Motor diagnostic writes are
serialized by the existing bus mutex; trace/last-error tuples are best-effort
and must not be used as coherent safety decisions. USART2 RX reconfiguration protection
now applies only during DMA/peripheral restart, not during normal byte reads.
Both UARTs use DMA RX, with USART1 transaction setup/teardown and USART2 circular
mode. The two buffer lifecycles are different.
Measurement storage/serialization remains in src/uart_diagnostics.c. Existing
0x85..0x88 layouts remain unchanged. Actual failure cause requires board tests.

GET_UART_STATS sends six responses: 0x85 UART_STATS, 0x86 UART_ISR_STATS,
0x87 MOTOR_DIAGNOSTICS, 0x88 MOTOR_TRACE, 0x89 UART_RX_ERRORS and 0x8A MOTOR_TRANSPORT_STATS. Existing 0x85/0x86 layouts remain
unchanged. No extra request or automatic motor motion is introduced.
Measurement storage, ISR accounting and diagnostic serialization are separated
into src/uart_diagnostics.c. Control/driver code calls measurement hooks only.

0x87 has 64 bytes: u32 uptime, u32 last reason (0xffffffff means none),
u32 motion/current masks (bits 0..11 for motion IDs 1..12, bits 16..27 for current),
u32 live USART1 SR, then twelve cumulative u32 counters in this order:
build_failed, tx_timeout, no_response, header_timeout, body_timeout, rx_error,
bad_length, bad_checksum, parse_error, id_or_data_length, servo_fault, valid_reply.
no_response means no byte was consumed before the RX deadline;
header_timeout means bytes arrived but a complete FF FF header was not found;
body_timeout means the header was found but the remaining frame was incomplete.
rx_error includes UART/DMA receive failure or exhausted RX DMA buffer;
UART_STATS distinguishes hardware errors from buffer overflow. Checksums are
the servo protocol's checksum, not host CRC16. Valid_reply counts parsed reply
frames, not complete twelve-motor feedback sets. Masks show the last completed read.

0x88 has 64 bytes: u32 uptime, attempted TX length, consumed RX length since
that TX, first 16 attempted TX bytes, first 16 consumed RX bytes, then u32
USART1 BRR, CR1, CR3, GPIOA CRH, and USART1 NVIC enabled (0/1).
All integers are little-endian. Previews are prefixes, not complete packet logs;
the current transaction can be in progress when sampled. TX preview does not
prove physical transmission, and RX preview excludes ISR-discarded bytes.
Snapshots never hold IRQ masking while sending responses. Host task stack is
2048 bytes to accommodate the result FIFO and nested diagnostic serialization.
These diagnostics identify the internal failure stage, not a definitive wiring,
motor power or baud-rate root cause. ISR measurement itself adds CPU overhead.

## UART ISR profiling

GET_UART_STATS (0x08) now returns the unchanged UART_STATS (0x85) followed
by UART_ISR_STATS (0x86). The latter has 44 little-endian bytes:
u32 uptime_ms, u32 CPU clock Hz, then USART1 and USART2 each containing
u32 ISR calls, u64 cumulative measured cycles (low word first), u32 maximum
measured cycles since boot; finally u32 enabled mask (bit 0 USART1, bit 1 USART2).
All counters are cumulative since boot. Calls wrap at 32 bits; cycles use 64 bits.
DWT CYCCNT is enabled once without resetting the shared counter. Each ISR exit,
including errors and buffer overflow, records an unsigned 32-bit cycle delta.
Disabled/non-running DWT is reported via the mask, not as zero CPU usage.

The measured span excludes hardware exception entry/return, compiler prologue/
epilogue and the final accounting operations. It includes most diagnostic
instrumentation and any preempting higher-priority ISR. USART1/2 and DMA1 Ch4/5 have the same
priority and do not preempt each other. These are approximate instrumented ISR
body costs, not total interrupt overhead or task deadline/latency measurements.
No printing or blocking operation is performed inside the ISR.
PC interval load (%) = delta cycles * 100000 / (CPU Hz * delta uptime_ms).
Average microseconds = delta cycles * 1000000 / (CPU Hz * delta calls).
Maximum is since boot, not the current sampling interval.

The NUCLEO-F103RB exposes this binary protocol on `USART2` through the ST-LINK
Virtual COM Port at 1000000 baud, 8 data bits, no parity, and one stop bit.
The transport baud changes together with the PC client; v1 message IDs,
COBS framing and CRC remain unchanged. See [HOST_LINK.md](HOST_LINK.md).

## Frame format

Each packet uses COBS framing with a `0x00` delimiter at both ends.

```text
0x00 | COBS(raw frame) | 0x00
```

The decoded little-endian frame is:

```text
u8   protocol_version     # 1
u8   message_type
u8   flags                # reserved, must be 0
u16  sequence
u16  payload_length       # phase 1 requests use 0
u8[] payload
u16  crc16_ccitt          # CRC-16/CCITT-FALSE over preceding raw bytes
```

## Implemented requests

| PC/RPi → F103 | Value | Result |
|---|---:|---|
| `HELLO` | `0x01` | Marks host link connected, then sends `HELLO_ACK` and `ACK` |
| `HEARTBEAT` | `0x02` | Refreshes the 1 s link timer and grants a control lease; the F103 FSM may enter `ARMED` after a safe stop completes |
| `GET_UART_STATS` | `0x08` | Read-only cumulative counters; replies with `UART_STATS`, no ARM or lease refresh |
| `GET_FSM_STATE` | `0x0A` | Read-only FSM query; replies with `FSM_STATE`, without refreshing the lease |
| `GET_POSITION_OFFSETS` | `0x0B` | Empty payload; schedules EEPROM offset reads for IDs 1..12, then returns `POSITION_OFFSETS` |
| `GET_MOTOR_REGISTER` | `0x0C` | Three bytes: ID, start address, length; DISARMED-only raw READ, returns `MOTOR_REGISTER` without an ACK |
| `SET_MOTOR_REGISTER` | `0x0D` | Allow-listed register write, DISARMED and directly verified torque OFF; final ACK/NACK only after read-back and EEPROM cleanup |

`SET_JOINT_TARGETS` (`0x06`) has a fixed 60-byte payload: for IDs 1..12 in
ascending order, five bytes each: position u16 LE [step], speed u16 LE [raw],
acceleration u8 [raw]. All 12 targets are required. It is accepted only while
connected and ARMED with no pending fault, target timeout or SAFE_STOP request;
position must be 0..4095 and speed 0..3400. ACK means
queued, not that the servo reached its goal. Invalid length/state/target or
queue failure returns NACK codes 2/5/8/9 respectively.
The actuator repeats its most recent valid target every 10 ms, but enters
DISARMED and starts a bounded 2048-step return before torque-off after 500 ms
without a fresh target.
The deadline is measured from successful target acceptance into the actuator
queue. That timestamp travels with the targets; dequeue never starts a new
500 ms interval. Failed/rejected enqueues and repeated motor-bus writes do not
refresh it. A queued target already 500 ms old is discarded before execution.
That target timeout invalidates the host lease; a later fresh HEARTBEAT can
make the FSM ready again once the return and torque-off have completed.
An independent link timeout remains 1000 ms and is refreshed ONLY by an
accepted PC HEARTBEAT. Initial HELLO starts a first-heartbeat grace period;
repeated HELLO on an established link does not refresh it. F103 sends
`F103_HEARTBEAT` every 1000 ms for PC status monitoring only: neither successful
nor failed TX refreshes the F103 watchdog. UART TC (final stop bit transmitted)
does not confirm PC receipt. The separate 500 ms target watchdog and
fresh-PC-heartbeat re-arm rule remain unchanged.

`GET_SENSOR_FEEDBACK` (`0x09`) is read-only and returns three `MOTOR_FEEDBACK`
(`0x8B`) frames, four IDs per frame. Each 56-byte payload contains sample
sequence u32 LE, sample uptime_ms u32 LE, valid mask u16 LE, first ID u8,
count u8 (=4), then four 11-byte entries: signed position/speed/load/current
u16 LE, voltage u8 (0.1 V), temperature u8 (deg C), servo error u8. The three
frames share one cached sensor snapshot. A zero bit in valid mask means that
ID's data must not be used. If no snapshot exists, NACK code 10 is returned.

The same request also returns one `IMU_FEEDBACK` (`0x8C`) frame with a 24-byte
payload: sample sequence u32 LE, sample uptime_ms u32 LE, valid u8, reserved
u8 (=0), then signed i16 LE accel X/Y/Z, temperature, gyro X/Y/Z in that
order. Sequence and uptime match the three motor frames. Raw scale is MPU-6050
at +/-2 g (16384 LSB/g) and +/-250 deg/s (131 LSB/(deg/s)); temperature in
degrees C is raw/340 + 36.53. When the IMU is absent or a read fails, valid=0
and all raw fields are zero; motor feedback remains available. The IMU uses
PB6=SCL and PB7=SDA with external pull-ups to 3.3 V; AD0 must be low for
I2C address 0x68. It is probed again once per second after a failure.

## EEPROM position-offset query

`GET_POSITION_OFFSETS` (`0x0B`) is read-only in every FSM state. It does not
connect the host, refresh a control lease, submit a target, change torque, or
unlock/write EEPROM. `ACK` means the query was scheduled, not completed.
Only one query/result may be pending; another request gets `NACK` code 11
(query busy). The sensor task makes one attempt per iteration, interleaved with
normal feedback; the host task continues processing the FSM and host traffic.
Each transaction uses the existing motor-bus mutex and DMA. Healthy completion
normally takes about 240 ms, but bus contention/failures can take longer.
Each ID gets up to five total attempts (initial attempt included), stopping
at its first valid reply. Failed attempts do not advance to the next ID until
the fifth failure. On exhaustion the last attempt's raw/servo error/status is
returned with that ID invalid; other IDs are still read. Normal all-success
queries still use just 12 attempts; worst case uses 60 attempts (nominally
1.2 s at the 20 ms sensor period, not a guaranteed bound with contention/errors).
Retry counters reset at each ID and each new job. No prior query's values are
reused. The valid-value criteria below are unchanged. The PC default wait is
5 s with early completion; an explicit positive `--listen-ms` overrides it.

The eventual `POSITION_OFFSETS` (`0x8E`) payload is exactly 56 bytes, all
multi-byte fields little-endian:

```text
u16 request_sequence
u32 completion_uptime_ms
u16 valid_mask                 # bits 0..11 correspond to IDs 1..12
12 entries, ascending ID order:
    u16 raw_offset             # EEPROM addresses 31..32 (0x1F..0x20)
    u8  servo_error
    u8  driver_status
```

Driver status uses `motor_device_status_t`: 0 OK, 1 invalid argument,
2 not initialized, 3 bus busy, 4 protocol error, 5 UART error,
6 partial feedback (not emitted by this single-ID read API).
Valid requires status 0, matching reply ID, exactly two data bytes, valid
checksum, servo_error 0, and reserved bits 12..15 zero. Missing/failed motors
remain invalid; their zero-initialized values are NOT a measured zero offset.
The result is returned even if some or all IDs fail. Existing motor diagnostics
provide the detailed UART/parse failure reason.

Decode magnitude as `raw_offset & 0x07FF`; bit 11 (`0x0800`) denotes negative.
This is NOT a two's-complement integer. Range is -2047..2047 step; display
angle is `offset_steps * 360 / 4096` degrees. Zero is a legitimate stored
offset, not proof that calibration was never run. The value does not establish
physical calibration accuracy. Motors are read sequentially, not atomically.
Update firmware and client together; older firmware rejects request 0x0B.

## Read-only motor register query

`GET_MOTOR_REGISTER` (`0x0C`) adds a single-ID, read-only transaction. Existing
message values and layouts, including `0x0B` / `0x8E`, are unchanged. The
three-byte request payload is:

```text
u8 motor_id    # 1..12
u8 address     # 0..255
u8 length      # 1..32; address + length must be <=256 (no wrap)
```

The request is allowed only in DISARMED, with no pending fault, target timeout
or SAFE_STOP. Invalid payload size returns NACK code 2; invalid ID/length/address
span returns NACK code 12 (`ERROR_INVALID_REGISTER`); a prohibited state or
pending safety action returns NACK code 5. NACK retains its existing four-byte
layout: request type u8, request sequence u16 LE, error code u8.

A valid request gets `MOTOR_REGISTER` (`0x8F`), with no separate success ACK:

```text
u16 request_sequence   # little-endian, correlates with the request
u8  motor_id           # echoed
u8  address            # echoed
u8  length             # requested length, echoed even on failure
u8  status
u8  servo_error        # validated servo error bits, otherwise zero
u8  data[length]       # present ONLY when status=0
```

| status | Meaning |
|---:|---|
| 0 | OK: matching ID, exact data length, valid checksum, no servo error |
| 1 | BUSY: another query/result occupies the slot, snapshot/bus mutex unavailable, or a safety/state change cancels the accepted job before I/O |
| 2 | COMMUNICATION_ERROR: driver not initialized, TX/RX failure or timeout |
| 3 | REPLY_ERROR: invalid complete reply, checksum, ID or data-length mismatch |
| 4 | SERVO_ERROR: valid reply from the requested ID with nonzero error bits |

Successful payloads are `7 + length` bytes (maximum 39); failures are exactly
7 bytes, not padded with fake data. These status numbers are specific to this
message and are NOT the existing POSITION_OFFSETS driver-status enum.

The host task validates/enqueues; the sensor task performs one bounded READ
using the existing USART1 DMA/bus mutex, outside the feedback-snapshot mutex;
the host task sends the result over USART2. Regular feedback and IMU collection
remain enabled. Offset and register queries share one exclusive pending/result
slot: a concurrent valid register request gets status 1 without overwriting the
first request; an offset request retains its existing NACK code 11 behavior.
Re-arming is inhibited from acceptance through result collection. A fault/stop
before the worker starts skips the READ and yields status 1. A query never
refreshes heartbeat/target timers, establishes a lease, submits targets, changes
torque, calibrates, or unlocks/writes registers. Normal FSM processing remains
active; a separate valid heartbeat can permit re-arming after the query finishes.

The vendor parser validates the entire servo frame but only copies eight data
bytes to its reply structure. The driver therefore uses a 38-byte RX buffer
and copies raw data directly from the validated frame for this query. The
vendor submodule and existing feedback/offset decoding are unchanged. Sign,
units and register-specific interpretation belong to the PC/RPi client.

There is one firmware READ attempt and one bounded local response-TX attempt
per accepted request, not an internal retry loop. A lost response is recovered
by the client's response-timeout retry (up to two total attempts), correlated
by request sequence; failed statuses/NACKs are not response timeouts. The
PC/RPi API/CLI for this message is implemented separately, not in this RTOS
change. Existing clients can continue to use existing messages.

Example: ID 1, address 48, length 2 has request payload `01 30 02`.

## Verified motor register write

`SET_MOTOR_REGISTER` (`0x0D`) uses the existing ACK/NACK envelope and result TX
retry FIFO. There is no new response type. See
[MOTOR_REGISTER_WRITE.md](MOTOR_REGISTER_WRITE.md) for the exact payload,
allow-list, failure codes 13..17, bounded lock recovery, diagnostics and the
four-entry/60-second duplicate-suppression contract. GET/SET/offset work shares
one exclusive job slot. Register jobs inhibit re-arm until their result has
been collected; an unverified EEPROM lock restore additionally inhibits re-arm
until verified recovery. Existing wire values/layouts are unchanged.

## Implemented responses

| F103 → PC/RPi | Value | Payload |
|---|---:|---|
| `ACK` | `0x80` | request type, request sequence, result code |
| `NACK` | `0x81` | request type, request sequence, error code |
| `HELLO_ACK` | `0x82` | firmware version, protocol version, capability bits |
| `EVENT` | `0x83` | state transition, host-link change, or unsafe diagnostic report |
| `F103_HEARTBEAT` | `0x84` | uptime, FSM state, host-link status, requested safety action, unsafe reason, fault reason |
| `UART_STATS` | `0x85` | 60-byte UART diagnostic snapshot described below |
| `MOTOR_FEEDBACK` | `0x8B` | One of three four-motor cached feedback blocks described above |
| `IMU_FEEDBACK` | `0x8C` | Cached IMU sample described above |
| `FSM_STATE` | `0x8D` | Request sequence u16 LE, FSM state u8; no ACK for the query |
| `POSITION_OFFSETS` | `0x8E` | Correlated 56-byte result for all 12 EEPROM offset reads; see above |
| `MOTOR_REGISTER` | `0x8F` | Request sequence, ID, address, length, status, servo error, then raw bytes on success only |

### ACK/NACK local-TX retry

The ACK/NACK payload is unchanged: request type u8, request sequence u16 LE,
result code u8. Success ACK reports request acceptance, not servo execution or
arrival. If local UART TX fails, the original complete response is retained in
an eight-entry fixed FIFO. Up to three attempts total are made, with at least
10 ms between a failed attempt's completion and its retry. Responses age out
at 250 ms (checked when serviced); every individual UART attempt remains bounded
to 20 ms. Success removes the response, while expiry or exhausted attempts drop
it. A new frame begins with a zero delimiter to recover from partial TX.

Retries keep the original F103 frame sequence AND request/result payload.
The target command is not re-executed, and neither watchdog is refreshed by
response transmission. Retries/other telemetry can make F103 frame sequences
arrive out of numerical order; match replies using the payload request sequence.
When the FIFO is full, request consumption pauses until space is available;
safety checks and RX interrupts continue. Sustained overload can still exceed
the RX ring capacity. USART2 timeout diagnostics record failed UART attempts.
No new message type, PC retry policy, or remote receipt acknowledgement is added.
Local success with loss downstream still causes the PC's existing ACK timeout.

## UART statistics

Request payload is empty. The response contains `u32 uptime_ms`, then two
28-byte blocks in USART1, USART2 order. Each block has seven little-endian
`u32` fields: tx_bytes, rx_bytes, rx_interrupts, rx_errors, rx_overflows,
rx_peak, timeouts. Counters accumulate since boot and wrap at 32 bits.

Both ports count observed DMA traffic including data later discarded. USART1
uses transaction DMA; USART2 uses polled circular DMA progress. RX errors count
UART receive-error events and DMA transfer errors. Overflow counts buffer-full
events on USART1 or detected overwrite events on USART2, not lost bytes. Peak
is the since-boot observed DMA received position for USART1 (capacity 256), or
task-observed backlog for USART2 (capacity 1024, capped at capacity).
Timeouts count motor TX waits/RX byte waits that expire.
USART2 timeouts count failed bounded TX waits (20 ms per frame, including
final UART TC); successful local TX does not confirm remote receipt.

PC rates use differences between snapshots divided by MCU elapsed time.
For 8N1, wire utilization is bytes/sec * 10 / baud * 100. USART1 reports
TX+RX utilization of the shared servo bus; USART2 reports each direction
separately because it is full duplex. Turnaround/idle time, bus mutex waits,
CPU utilization and ISR execution time are not part of this utilization.
The stats request/reply traffic itself is included in USART2 counters.

## FSM

```text
BOOT --initialization complete--> DISARMED
DISARMED --healthy host lease, safe stop complete, no fault--> ARMED
ARMED --target timeout (500 ms)--> DISARMED + safe stop request
ARMED --heartbeat timeout--> DISARMED + safe stop request
ARMED or DISARMED --fault report--> FAULT
FAULT --safe stop or fallback torque-off TX succeeds--> DISARMED
```

FSM values are `BOOT=0`, `DISARMED=1`, `ARMED=2`, and `FAULT=5`. Values 3 and
4 remain unused so an earlier v1 client still decodes `FAULT` correctly.

`DISARMED` means that external target control is not accepted. The F103 alone
owns FSM transitions. `HELLO` establishes the link; a valid `HEARTBEAT` grants
a lease, after which the F103 enters `ARMED` only when safe-stop has completed
and no fault is latched. A target timeout revokes the lease until a fresh
heartbeat. FAULT is released into DISARMED when safe stop or fallback torque-off
transmission succeeds; this clears the fault reason and revokes the host lease,
requiring a heartbeat received after completion before re-arming. There is no
remote fault reset command. The PC cannot command immediate disarm/stop. Ceasing targets
triggers the 500 ms target watchdog; no accepted PC heartbeat RX for 1000 ms
triggers the link watchdog, regardless of outgoing F103 heartbeat TX.
Expiry and pending safety requests are handled before each valid request, not
after accepting a target. Once the link expires, HEARTBEAT alone is rejected;
HELLO must establish a new connection first. This does not change wire layouts.
RX processing yields after 128 bytes, two delimited frames, or an 8 ms budget
checked between requests. A request's complete reply may exceed that budget.
Neither means torque is removed immediately: the F103 first
attempts to return to 2048. An independent physical emergency stop is needed
where immediate torque removal is required.

`host_protocol_latch_unsafe()` and `host_protocol_latch_fault()` are integration
points for the later motor/safety task. An unsafe report sends an `EVENT` and
appears in F103 heartbeat telemetry, but does not disarm the host or request a
safe stop; this preserves the RPi policy's ability to issue recovery targets.
`host_protocol_clear_unsafe()` clears the diagnostic once the later safety task
has verified recovery. A fault remains an F103-owned hard stop.

`actuator_task` owns target execution and safe stopping. On a target or host
timeout with commanded torque active, a safe stop discards queued targets,
repeatedly sends a synchronized 2048-step goal (speed 200 raw, acceleration
10 raw) and requires three fresh complete feedback samples with all twelve
axes within 16 steps. It then sends broadcast torque-off and calls
`host_protocol_complete_safe_stop()`. The return attempt is bounded to 15 s
overall and 0.5 s without measurable progress; repeated motor-bus failures,
missing feedback or a blocked joint trigger FAULT. At boot with no commanded
torque, it skips movement and attempts torque-off immediately.
The stop request is published before the DISARMED/FAULT state and its EVENT;
the actuator also observes pending fault/timeout flags before the host task
publishes a state. Progress is checked per motor, not by summed center error:
each motor outside the 16-step tolerance must improve its best error by at
least 12 steps within 500 ms. Fresh complete error-free samples refresh the
timers of already-centered motors. Missing/invalid feedback refreshes none.
Motion/current reply servo-error bits are OR-combined per feedback cycle; the
next cycle starts clear. Feedback validity is not a fault-free guarantee.
In FAULT, the actuator performs one additional full return-and-torque-off
attempt. It sends the center goal before explicitly enabling torque, uses
fresh feedback, and has independent 0.5 s no-progress and 15 s total deadlines.
Three consecutive transmission failures also fail an attempt, including failures
to send torque-off. Successful recovery clears the pending safety action and
fault reason, then transitions to DISARMED (state-change EVENT cause 9).
If the single attempt fails, torque-off is repeatedly transmitted every 10 ms
actuator cycle; position and torque-on commands stop. FAULT and the safety action
stay active while transmission fails. The first successful local torque-off
transmission terminates this fallback loop and also transitions to DISARMED.
Broadcast success confirms transmission, not individual servo compliance.
The host must send fresh targets after the F103 autonomously re-enters ARMED.

Target and feedback messages are implemented as described above. IMU feedback
is telemetry only; this version does not use it for actuator safety decisions.
Current/voltage/load automatic limits are not enabled; their measurements are
provided to the PC for host-side handling. No numeric unsafe threshold is added.
