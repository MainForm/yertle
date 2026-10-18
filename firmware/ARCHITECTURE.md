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
`host_protocol_task` consumes the buffer through `host_uart_try_read`, then
sleeps for 2 ms. Each receive pass is bounded to 128 bytes, two delimited frames,
or an 8 ms budget checked between reads/requests. A single request/response may
exceed 8 ms; transmission is not interrupted in the middle of a frame. Partial
COBS data survives across passes. Safety requests and link expiry are processed
before/after the pass and before every valid request, so continuous input cannot
indefinitely postpone the FSM or keep the lower-priority sensor task from running.
The driver masks USART2 IRQ briefly while updating the consumer index.
On overflow or a UART receive error, queued partial data is flushed, the task
is told to clear its partial COBS frame, and the ISR discards input through the
next zero delimiter. RX is interrupt-driven; host TX remains polling.

## Host heartbeat and timeouts

F103 sends `F103_HEARTBEAT` every 1000 ms (task scheduling can add jitter).
It is status telemetry for the PC and never refreshes the F103 link timeout.
Only an accepted PC HEARTBEAT refreshes that 1000 ms watchdog. Initial HELLO
starts a first-heartbeat grace period; repeated HELLO while connected does not
refresh the timer. CRC-invalid frames and read-only queries do not refresh it.
Host TX now returns success only after UART TC (final stop bit), with a 20 ms
bounded wait per frame and no interrupt masking during TX. Failed TX does not
refresh the link timer, and neither does successful TX. Wire framing and message
types are unchanged.

Local UART TC can succeed with the PC disconnected; it is not a liveness signal.
Missing PC heartbeat therefore expires the connection even while F103 telemetry
TX succeeds. Initial arming and recovery still require a valid PC heartbeat.
After link expiry, a new HELLO is required before a HEARTBEAT can be accepted.
The independent 500 ms fresh-target watchdog and SAFE_STOP are unchanged.

## ACK/NACK transmission recovery

The host task has a 2048-byte stack (up from 1536 bytes) for the fixed result
queue and nested diagnostic serialization. ARM compiler stack-usage inspection
identified a diagnostic call chain exceeding the old allocation. This is a
static sizing check, not a measured on-board high-water mark.

Successful target acceptance still means queue registration, not motor arrival.
`send_result` reserves one of eight fixed FIFO slots and checks the UART TX result.
The first result is attempted immediately; behind an older pending result, new
results wait in order. A failed result retains its original payload and F103
frame sequence. `service_result_tx` tries it again no earlier than 10 ms after
the previous attempt finished, at most three attempts total and at most 250 ms
of queue age. Each service call does at most one bounded UART TX; no busy retry
loop or heap allocation is introduced. An expired/exhausted result is dropped.
The next attempt's leading COBS zero delimiter terminates any partial old frame.

Safety/link checks run before and after the retry service. When all eight slots
are occupied, RX consumption pauses rather than accepting another command with
nowhere to store its result; bounded retries/expiry free slots. The USART2 ISR
continues receiving, but sustained overload may still overflow its 255-byte ring
and invoke the documented resynchronization. Results are only response records:
retry never submits a target again or refreshes heartbeat/target timestamps.
Telemetry/events remain best-effort and can have later frame sequences before
an older retried ACK. The PC correlates ACK/NACK by the request sequence inside
the payload, not by increasing F103 frame sequence.

Host-context counters `result_tx_failures`, `result_retry_attempts`,
`result_retry_successes`, and `result_dropped` are debugger-visible; they are not
new wire telemetry. Actual UART TX timeouts still increment USART2 UART_STATS
timeouts. UART TC success proves local transmission only: PC reception is not
acknowledged, so this is bounded local-TX recovery, not guaranteed delivery.

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
do NOT terminate a twelve-servo transaction. The normal-mode 256-byte RX buffer is
reset for each request; exhaustion/errors abort instead of wrapping/overwriting.
The bus mutex remains held throughout TX/replies; empty reads sleep one tick.
Normal DMA is stopped and residual byte counts finalized at each transaction
end, including broadcasts, partial feedback and abort. Setup/teardown uses a
short peripheral-specific IRQ guard; diagnostic snapshots remain unmasked.
The task-owned residual counter is separate from ISR-owned RX accounting.
No new task, HAL/RS485 DE layer, pin, baud, clock or FSM change is introduced.

DMA replaces per-byte USART1 RX interrupts, not all interrupts. The active
handlers are USART1 IDLE/error, DMA1 Channel4 TX completion/error and DMA1
Channel5 RX full/error. USART1 RXNEIE stays disabled. USART2 still uses its
own RX ISR/ring and polling TX. Its implementation is not a retired motor path.
Upstream submodules remain unmodified; the G4 HAL source is excluded by CMake.
Diagnostic snapshots directly read uart_diagnostics storage, not UART-driver
wrapper APIs. Transaction ownership uses one active flag; separate completion
and error flags retain their actual distinct meanings.

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
- Pending fault/target-timeout/SAFE_STOP requests reject new targets even before
  the host task publishes the FSM transition. Timeout/fault handling publishes
  SAFE_STOP first, then changes the state and sends the UART event. The actuator
  observes these requests directly, avoiding a DISARMED/immediate-torque-off race.
- A successful `actuator_submit_targets` enqueue stores an acceptance tick with
  the target set. The 500 ms target watchdog uses that tick, not dequeue time.
  Rejected/failed enqueues and repeated UART target writes do not refresh it.
  An already-expired queued target is discarded before any motor write.
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
- Return progress is tracked separately for all twelve motors: a fresh complete
  error-free sample must reduce that motor's best center error by at least 12
  steps within 500 ms. A motor already within 16 steps refreshes its timer with
  each valid new sample. Any stalled motor fails the attempt; movement of other
  motors cannot hide it. Missing/invalid samples refresh no motor's timer.
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

A motor is marked valid only when both replies were received and parsed.
The servo-error bits from the motion and current replies are OR-combined within each
feedback cycle, then reset on the next cycle. A zero-error current reply cannot
erase an error from the motion reply. Valid indicates parsed data, not absence
of a servo fault.
The latest snapshot and a twelve-bit validity mask are protected by a separate cache
mutex. Missing feedback is recorded as invalid but is not yet promoted to a
FAULT; that policy belongs to the later hard-safety stage.

Current/voltage/load automatic limit checks are intentionally not enabled.
These values remain PC telemetry for host-side measurement and decisions; no
new unsafe trigger or motor cutoff based on their thresholds is added here.

## Read-only EEPROM offset jobs

`host_protocol_task` accepts `GET_POSITION_OFFSETS` and calls
`sensor_request_position_offsets(request_sequence)`; it never performs the
UART transactions itself. `sensor_task` makes one attempt after each regular
feedback/IMU iteration, calling `motor_device_read_position_offset(id, &offset)`.
The driver serializes READ 0x1F/2 bytes with the existing bus mutex and DMA,
checks the reply and decodes bit-11 sign/magnitude. No write/calibration or
torque operation is involved.
The sensor job retries a failing ID on later iterations, with up to five total
attempts, and accepts the first valid reply. Success or five failures advances
to the next ID. Normal feedback is collected between retries. The single-ID
driver API remains a single transaction; only the batch job adds retries.

The job/result cache and its pending/ready flags share the existing feedback
mutex. UART I/O is outside that cache lock; no interrupt masking is added.
The host consumes a completed result with `sensor_take_position_offsets()` and
sends one `POSITION_OFFSETS` frame, including a validity bit and driver/servo
error per motor. Duplicate requests cannot overwrite a pending/unconsumed job.
Failed IDs do not discard successful IDs. This is on-demand EEPROM reading,
not an addition to every host feedback response. One extra transaction per
sensor iteration can add bus/CPU time while the query is active; 50 Hz is a
nominal period, not a guaranteed rate under contention or errors.

## Clocking

The target uses the internal HSI oscillator through the PLL at 64 MHz. APB1 is
32 MHz and APB2 is 64 MHz. This avoids dependence on the ST-LINK MCO/HSE path
and produces an exact USART1 divisor for the 1 Mbps servo bus.
