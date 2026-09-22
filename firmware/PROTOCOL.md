# Yertle PC/RPi host protocol v1, phase 1

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

## Phase 1 requests

| PC/RPi → F103 | Value | Result |
|---|---:|---|
| `HELLO` | `0x01` | Marks host link connected, then sends `HELLO_ACK` and `ACK` |
| `HEARTBEAT` | `0x02` | Refreshes the 1 s host timeout, then sends `ACK` |
| `ARM` | `0x03` | `ACK` only from connected, safe `DISARMED` |
| `DISARM` | `0x04` | Requests safe stop and enters `DISARMED` |
| `STOP` | `0x05` | Requests safe stop and enters `DISARMED` |
| `RESET_FAULT` | `0x07` | `ACK` only from clearable `FAULT`; requests safe stop |

`0x06` is reserved for the later `SET_JOINT_TARGETS` phase and returns `NACK`
in phase 1.

## Phase 1 responses

| F103 → PC/RPi | Value | Payload |
|---|---:|---|
| `ACK` | `0x80` | request type, request sequence, result code |
| `NACK` | `0x81` | request type, request sequence, error code |
| `HELLO_ACK` | `0x82` | firmware version, protocol version, capability bits |
| `EVENT` | `0x83` | state transition or host-link change |
| `F103_HEARTBEAT` | `0x84` | uptime, FSM state, host-link status, requested safety action, unsafe reason, fault reason |

## FSM

```text
BOOT --initialization complete--> DISARMED
DISARMED --ARM with host link, no pending safe stop, no unsafe or fault--> ARMED
ARMED --DISARM or STOP--> DISARMED + safe stop request
ARMED --heartbeat timeout--> DISARMED + safe stop request
ARMED --unsafe limit report--> DISARMED + safe stop request
ARMED or DISARMED --fault report--> FAULT
FAULT --fault clearable and RESET_FAULT--> DISARMED + safe stop request
```

FSM values are `BOOT=0`, `DISARMED=1`, `ARMED=2`, and `FAULT=5`. Values 3 and
4 remain unused so an earlier v1 client still decodes `FAULT` correctly.

`DISARMED` does not alone mean that a new ARM is accepted. When the protocol
requests `SAFE_STOP`, the later motor/safety task must complete it through
`host_protocol_complete_safe_stop()`. Until then, the F103 rejects `ARM`.

`host_protocol_latch_unsafe()` and `host_protocol_latch_fault()` are integration
points for the later motor/safety task. They are not active hardware-limit
implementations in phase 1. The unsafe reason stays in F103 heartbeat telemetry
as a diagnostic cause; it is not an FSM state.
