# Firmware task and motor-driver architecture

## Runtime tasks

| Task | Priority | Period | Responsibility |
|---|---:|---:|---|
| `actuator_task` | idle + 4 | 10 ms | Latest-target execution, torque state and SAFE_STOP |
| `host_protocol_task` | idle + 3 | 2 ms buffer polling | Host framing, commands, heartbeat and FSM |
| `sensor_task` | idle + 2 | 20 ms | Twelve-servo feedback collection and cached snapshot |
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

USART1 uses the F103 DMA port of Mowibox's transaction pattern, implemented in
`motor_dma.c`, not the G4 HAL source itself. RX DMA1 Channel5 is armed before
TX DMA1 Channel4; owned buffers stay valid until transaction completion/abort.
`motor_uart_write` preserves the existing synchronous task API, but sleeps while
DMA sends and waits for both DMA completion and UART TC (wire completion).
It no longer supplies each byte by polling. USART1 RXNE interrupts and the old
motor RX ring are removed. UART IDLE/error and DMA complete/error handlers do
minimal accounting/state updates with no FreeRTOS calls or packet parsing.
`motor_uart_read_frame` assembles from DMA progress (CNDTR), and the existing
Mowibox parser runs in task context. Multiple IDLE events between servo replies
do NOT terminate a six-servo transaction. The normal-mode 256-byte RX buffer is
reset for each request; exhaustion/errors abort instead of wrapping/overwriting.
The bus mutex remains held throughout TX/replies; empty reads sleep one tick.
Normal DMA is stopped and residual byte counts finalized at each transaction
end, including broadcasts, partial feedback and abort. Setup/teardown uses a
short peripheral-specific IRQ guard; diagnostic snapshots remain unmasked.
The task-owned residual counter is separate from ISR-owned RX accounting.
No new task, HAL/RS485 DE layer, pin, baud, clock or FSM change is introduced.

```text
actuator_task ------- write/torque ----+
                                       |
sensor_task --------- feedback read ---+--> motor_device mutex
                                               |
                                               +--> Mowibox protocol builder/parser
                                               +--> motor_uart (USART1, 1 Mbps)
                                               +--> motor_dma (DMA1 Ch4 TX / Ch5 RX)
                                               +--> FE-URT --> STS3215 bus
```

`motor_device` locks the bus for an entire request/reply transaction. A task
must never access USART1 directly. Priority inheritance on the FreeRTOS mutex
prevents a sensor transaction from permanently blocking a higher-priority
actuator request.

## Control behavior

- `actuator_task` accepts only validated sets containing all twelve motor targets.
- Position is restricted to 0..4095 steps and speed to 0..3400 raw.
- A received target is written with one seven-byte-per-servo SYNC_WRITE frame:
  acceleration, position, running time (zero), and speed.
- Torque stays disabled until a valid target exists. The target registers are
  written first, then torque is enabled, preventing an uncommanded jump to an
  arbitrary startup target.
- Target or heartbeat timeout discards queued targets, then commands all twelve
  motors toward position 2048 at speed 200 raw and acceleration 10 raw. Three
  fresh, complete feedback samples with every motor within 16 steps are needed
  before broadcast torque-off completes SAFE_STOP. Recovery is bounded by a
  15 s total timeout and a 0.5 s no-progress timeout; failed writes also abort.
- At boot (no commanded torque), SAFE_STOP only requests broadcast torque-off.
- FAULT starts one additional full SAFE_STOP attempt: command position 2048,
  explicitly enable torque, verify settling, then disable torque. Each attempt
  resets its timers and feedback history. Three consecutive command/torque
  transmission failures, 0.5 s without progress, or 15 s overall fail an attempt.
- Successful FAULT recovery completes SAFE_STOP and transitions to DISARMED.
  If the single attempt fails, only torque-off is retried once per 10 ms actuator
  cycle, without position or torque-on commands. The first successful local
  torque-off transmission also completes SAFE_STOP and transitions to DISARMED.
  While OFF keeps failing, FAULT and the pending safety action remain active.
  A successful broadcast write does not prove that every servo disabled torque.
- FAULT completion clears the fault reason and revokes the host lease. A new
  heartbeat after completion is required before automatic re-arming; torque
  remains off until a fresh target. A subsequent fault report starts a new cycle.
- The 0.5 s no-progress limit applies separately to each attempt: an initial
  failed stop followed by one stalled FAULT retry can total about 1.0 s.
- Re-arming does not restore a stale target; a new target set is required.

## Feedback behavior

At 50 Hz, `sensor_task` performs two SYNC_READ transactions for IDs 1..12:

1. registers `0x38..0x3F`: position, speed, load, voltage, temperature;
2. registers `0x45..0x46`: current.

A motor is marked valid only when both replies were received and parsed. The
latest snapshot and a twelve-bit validity mask are protected by a separate cache
mutex. Missing feedback is recorded as invalid but is not yet promoted to a
FAULT; that policy belongs to the later hard-safety stage.

## Clocking

The target uses the internal HSI oscillator through the PLL at 64 MHz. APB1 is
32 MHz and APB2 is 64 MHz. This avoids dependence on the ST-LINK MCO/HSE path
and produces an exact USART1 divisor for the 1 Mbps servo bus.
