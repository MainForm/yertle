# Yertle PC/RPi host protocol v1

## Motor transaction diagnostics

### UART receive-error detail (0x89)

GET_UART_STATS additionally returns UART_RX_ERRORS (0x89), a 64-byte payload:
u32 uptime_ms, then USART1 and USART2 each with seven u32 fields (ORE count,
FE count, NE count, PE count, last error SR, last error DR byte, last error
DWT cycle32), followed by u32 diagnostic snapshot maximum masked-body cycles.
All fields are little-endian. Counts accumulate since boot and wrap at 32 bits.
One error event may set multiple flags, so their sum need not equal rx_errors.
The SR/DR/cycle values are historical, captured in the ISR before clearing flags;
they do not describe the current UART state. DR data on errors is not reliable.
DWT cycle32 wraps and is not a timestamp in milliseconds.

Snapshot timing measures the bounded copy/register-read body inside its critical
section; it excludes entry/exit, final accounting, other critical sections and
interrupt entry latency. It helps assess diagnostic-induced masking, but is not
a complete worst-case IRQ-latency measurement. DWT availability is carried by
UART_ISR_STATS. No ISR RTOS API, printing or motor control change was added.
Measurement storage/serialization remains in src/uart_diagnostics.c. Existing
0x85..0x88 layouts remain unchanged. Actual failure cause requires board tests.

GET_UART_STATS now sends four responses: 0x85 UART_STATS, 0x86 UART_ISR_STATS,
0x87 MOTOR_DIAGNOSTICS and 0x88 MOTOR_TRACE. Existing 0x85/0x86 formats remain
unchanged. No extra request or automatic motor motion is introduced.
Measurement storage, ISR accounting and diagnostic serialization are separated
into src/uart_diagnostics.c. Control/driver code calls measurement hooks only.

0x87 has 64 bytes: u32 uptime, u32 last reason (0xffffffff means none),
u32 motion/current masks (bits 0..5 for motion IDs 1..6, bits 8..13 for current),
u32 live USART1 SR, then twelve cumulative u32 counters in this order:
build_failed, tx_timeout, no_response, header_timeout, body_timeout, rx_error,
bad_length, bad_checksum, parse_error, id_or_data_length, servo_fault, valid_reply.
no_response means no byte was consumed before the RX deadline;
header_timeout means bytes arrived but a complete FF FF header was not found;
body_timeout means the header was found but the remaining frame was incomplete.
rx_error includes ring discontinuity caused by UART errors or overflow;
UART_STATS distinguishes hardware errors from buffer overflow. Checksums are
the servo protocol's checksum, not host CRC16. Valid_reply counts parsed reply
frames, not complete six-motor feedback sets. Masks show the last completed read.

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
instrumentation and any preempting higher-priority ISR. USART1/2 have the same
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
| `HEARTBEAT` | `0x02` | Refreshes the 1 s host timeout; when safely `DISARMED`, it also attempts automatic re-ARM |
| `ARM` | `0x03` | Explicit re-ARM, including after an operator `STOP` |
| `DISARM` | `0x04` | Requests safe stop and enters `DISARMED` |
| `STOP` | `0x05` | Requests safe stop and enters `DISARMED` |
| `RESET_FAULT` | `0x07` | `ACK` only from clearable `FAULT`; requests safe stop |
| `GET_UART_STATS` | `0x08` | Read-only cumulative counters; replies with `UART_STATS`, no ARM or lease refresh |

`0x06` is reserved for the later `SET_JOINT_TARGETS` phase and returns `NACK`
in phase 1.

## Implemented responses

| F103 → PC/RPi | Value | Payload |
|---|---:|---|
| `ACK` | `0x80` | request type, request sequence, result code |
| `NACK` | `0x81` | request type, request sequence, error code |
| `HELLO_ACK` | `0x82` | firmware version, protocol version, capability bits |
| `EVENT` | `0x83` | state transition, host-link change, or unsafe diagnostic report |
| `F103_HEARTBEAT` | `0x84` | uptime, FSM state, host-link status, requested safety action, unsafe reason, fault reason |
| `UART_STATS` | `0x85` | 60-byte UART diagnostic snapshot described below |

## UART statistics

Request payload is empty. The response contains `u32 uptime_ms`, then two
28-byte blocks in USART1, USART2 order. Each block has seven little-endian
`u32` fields: tx_bytes, rx_bytes, rx_interrupts, rx_errors, rx_overflows,
rx_peak, timeouts. Counters accumulate since boot and wrap at 32 bits.

RX bytes count register reads with RXNE, including errored/discarded bytes;
RX errors count IRQs with one or more receive-error flags. Overflow counts
buffer-full events, not lost bytes. Peak is the maximum queued RX bytes since
boot (capacity 255); timeouts count motor TX waits/RX byte waits that expire.
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
DISARMED --valid HEARTBEAT with host link, no pending safe stop, no fault--> ARMED
DISARMED --explicit ARM with host link, no pending safe stop, no fault--> ARMED
ARMED --DISARM or STOP--> DISARMED + safe stop request
ARMED --heartbeat timeout--> DISARMED + safe stop request
ARMED or DISARMED --fault report--> FAULT
FAULT --fault clearable and RESET_FAULT--> DISARMED + safe stop request
```

FSM values are `BOOT=0`, `DISARMED=1`, `ARMED=2`, and `FAULT=5`. Values 3 and
4 remain unused so an earlier v1 client still decodes `FAULT` correctly.

`DISARMED` means that external target control is not accepted. A valid
heartbeat automatically re-enters `ARMED` as soon as no safe-stop action is
pending and no fault is latched. `STOP` is intentionally different: it latches
external control off until an explicit `ARM` request arrives.

`host_protocol_latch_unsafe()` and `host_protocol_latch_fault()` are integration
points for the later motor/safety task. An unsafe report sends an `EVENT` and
appears in F103 heartbeat telemetry, but does not disarm the host or request a
safe stop; this preserves the RPi policy's ability to issue recovery targets.
`host_protocol_clear_unsafe()` clears the diagnostic once the later safety task
has verified recovery. A fault remains an F103-owned hard stop.

`actuator_task` owns target execution and safe stopping. A safe stop disables
torque through `motor_device` before calling
`host_protocol_complete_safe_stop()`. New targets are discarded during the
transition, so the host must send a fresh target after re-arming.

The stage-2 target and feedback messages are not part of the wire format yet.
The task and device-driver APIs are present, but `0x06` remains reserved and is
rejected until the corresponding host-side serialization is added.
