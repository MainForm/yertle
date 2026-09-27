# Yertle PC/RPi host protocol v1

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
