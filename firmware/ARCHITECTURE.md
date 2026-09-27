# Firmware task and motor-driver architecture

## Runtime tasks

| Task | Priority | Period | Responsibility |
|---|---:|---:|---|
| `actuator_task` | idle + 4 | 10 ms | Latest-target execution, torque state and SAFE_STOP |
| `host_protocol_task` | idle + 3 | 2 ms buffer polling | Host framing, commands, heartbeat and FSM |
| `sensor_task` | idle + 2 | 20 ms | Six-servo feedback collection and cached snapshot |
| Idle task | idle | kernel managed | FreeRTOS idle processing |

The FSM is owned by `host_protocol_task`; there is no separate FSM task.

## Host receive sequence

USART2 RXNE/error interrupts invoke `USART2_IRQHandler` through the startup
vector table. The ISR reads SR/DR and stores bytes in a 256-byte ring buffer
(255 usable bytes). It performs no protocol parsing and calls no FreeRTOS API.
`host_protocol_task` consumes the buffer every 2 ms through `host_uart_try_read`;
the driver masks USART2 IRQ briefly while updating the consumer index.
On overflow or a UART receive error, queued partial data is flushed, the task
is told to clear its partial COBS frame, and the ISR discards input through the
next zero delimiter. RX is interrupt-driven; host TX remains polling.

## Motor stack

```text
actuator_task ------- write/torque ----+
                                       |
sensor_task --------- feedback read ---+--> motor_device mutex
                                               |
                                               +--> Mowibox protocol builder/parser
                                               +--> motor_uart (USART1, 1 Mbps)
                                               +--> FE-URT --> STS3215 bus
```

`motor_device` locks the bus for an entire request/reply transaction. A task
must never access USART1 directly. Priority inheritance on the FreeRTOS mutex
prevents a sensor transaction from permanently blocking a higher-priority
actuator request.

## Control behavior

- `actuator_task` accepts only validated sets containing all six motor targets.
- Position is restricted to 0..4095 steps and speed to 0..3400 raw.
- A received target is written with one seven-byte-per-servo SYNC_WRITE frame:
  acceleration, position, running time (zero), and speed.
- Torque stays disabled until a valid target exists. The target registers are
  written first, then torque is enabled, preventing an uncommanded jump to an
  arbitrary startup target.
- DISARM, STOP, heartbeat timeout, or FAULT discards queued targets and sends
  broadcast torque-off. SAFE_STOP completes only after that frame is sent.
- Re-arming does not restore a stale target; a new target set is required.

## Feedback behavior

At 50 Hz, `sensor_task` performs two SYNC_READ transactions for IDs 1..6:

1. registers `0x38..0x3F`: position, speed, load, voltage, temperature;
2. registers `0x45..0x46`: current.

A motor is marked valid only when both replies were received and parsed. The
latest snapshot and a six-bit validity mask are protected by a separate cache
mutex. Missing feedback is recorded as invalid but is not yet promoted to a
FAULT; that policy belongs to the later hard-safety stage.

## Clocking

The target uses the internal HSI oscillator through the PLL at 64 MHz. APB1 is
32 MHz and APB2 is 64 MHz. This avoids dependence on the ST-LINK MCO/HSE path
and produces an exact USART1 divisor for the 1 Mbps servo bus.
