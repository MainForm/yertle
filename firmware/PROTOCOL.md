# Yertle PC/RPi host protocol v1

## DMA transport measurement (0x8A)

GET_UART_STATS also returns MOTOR_TRANSPORT_STATS, 64 bytes of little-endian
u32 words: uptime_ms, mode (1 = F103 DMA port of Mowibox pattern), TX starts,
TX wire completions observed by task, UART IDLE events, RX buffer-full events,
TX DMA errors, RX DMA errors, SyncRead attempts, complete six-ID SyncReads,
feedback cycle attempts, complete two-block/six-ID feedback cycles, RX buffer
capacity (256), three reserved zero words. No new request is needed.
Each counter has one serialized writer (bus task or equal-priority ISRs).
Complete SyncRead means all IDs 1..12 parsed; complete feedback means both
motion/current blocks for all six. Counts are since boot; PC summary subtracts
first/last samples and prints the actual MCU span. Boundary snapshots can catch
an in-progress transaction, so attempts/completions are not an atomic pair.
DMA full is fatal, not circular ring overflow. In DMA mode UART_STATS RX bytes
count bytes moved to memory, including discarded data and stop residue, not DR
reads by CPU. rx_interrupts counts UART IDLE/error events; UART_ISR_STATS port1
aggregates USART1 plus DMA1 Channel4/5 handler bodies. DMA errors are distinct
from UART Overrun/Framing/Noise/Parity flags. Do not interpret error-events per
DMA byte as corrupted byte count, bit error rate or packet failure rate.
USART2 semantics remain unchanged. All six diagnostic frames remain <=64 bytes.

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
and must not be used as coherent safety decisions. USART2 RX-ring protection
is unchanged; USART1 now has DMA setup/teardown protection rather than a ring.
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
1536 bytes to accommodate the additional bounded snapshot buffers.
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
Virtual COM Port at 115200 baud, 8 data bits, no parity, and one stop bit.

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
| `HEARTBEAT` | `0x02` | Refreshes the 1 s host lease; the F103 FSM may autonomously enter `ARMED` after a safe stop completes |
| `GET_UART_STATS` | `0x08` | Read-only cumulative counters; replies with `UART_STATS`, no ARM or lease refresh |
| `GET_FSM_STATE` | `0x0A` | Read-only FSM query; replies with `FSM_STATE`, without refreshing the lease |

`SET_JOINT_TARGETS` (`0x06`) has a fixed 60-byte payload: for IDs 1..12 in
ascending order, five bytes each: position u16 LE [step], speed u16 LE [raw],
acceleration u8 [raw]. All 12 targets are required. It is accepted only while
connected and ARMED; position must be 0..4095 and speed 0..3400. ACK means
queued, not that the servo reached its goal. Invalid length/state/target or
queue failure returns NACK codes 2/5/8/9 respectively.
The actuator repeats its most recent valid target every 10 ms, but enters
DISARMED and starts a bounded 2048-step return before torque-off after 500 ms
without a fresh target.
That target timeout invalidates the host lease; a later fresh HEARTBEAT can
make the FSM ready again once the return and torque-off have completed.
An independent host HEARTBEAT timeout remains 1000 ms.

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

## UART statistics

Request payload is empty. The response contains `u32 uptime_ms`, then two
28-byte blocks in USART1, USART2 order. Each block has seven little-endian
`u32` fields: tx_bytes, rx_bytes, rx_interrupts, rx_errors, rx_overflows,
rx_peak, timeouts. Counters accumulate since boot and wrap at 32 bits.

USART2 RX bytes count register reads with RXNE, including errored/discarded
bytes; USART1 DMA bytes and IRQ timing have the semantics documented above.
RX errors count UART IRQs with one or more receive-error flags. Overflow counts
buffer-full events, not lost bytes. Peak is the observed since-boot DMA received
position for USART1 (capacity 256), or queued RX bytes for USART2 (capacity 255).
Timeouts count motor TX waits/RX byte waits that expire.
USART2 timeouts are zero because that driver has no timed waits.

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
triggers the 500 ms target watchdog; loss of heartbeat triggers the 1000 ms
link watchdog. Neither means torque is removed immediately: the F103 first
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
