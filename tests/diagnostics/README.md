# Diagnostic ownership tests

Windows native (no running Docker engine required):

```powershell
cmake -S tests/diagnostics -B build/diagnostic-tests
cmake --build build/diagnostic-tests --config Release
ctest --test-dir build/diagnostic-tests -C Release --output-on-failure
```

Run inside the development container from /workspace:

```sh
gcc -std=c11 -Wall -Wextra -Werror -Itests/diagnostics -Ifirmware/include tests/diagnostics/diagnostics_test.c firmware/src/uart_diagnostics.c -o /tmp/yertle-diagnostics-test
/tmp/yertle-diagnostics-test
```

Tests compile the real diagnostic implementation with register/RTOS stubs and
check port ownership, 64-bit accumulation carry, error counts and all six
response layouts. Diagnostic critical-section macros intentionally fail to
compile. This does not simulate interrupt interleavings or validate electrical
signals, interrupt latency, or the driver's boundary IRQ protection.

`firmware_dma_test` compiles the real DMA driver, task-side frame reader,
motor_device and Mowibox protocol using mock registers/RTOS calls. It covers
multiple IDLE events, twelve replies, missing IDs, buffer ownership, wire-TC wait,
RX residue/no double counting, UART errors, both DMA errors, buffer exhaustion,
no response/partial response timeout, restart and transport/cycle counters.
It also verifies motion/current servo-error bit merging and next-cycle clearing.
It does not emulate DMA bus arbitration or electrical/interruption timing.

The safe-stop test also covers single FAULT return attempts, torque-off-only
fallback, successful completion and continued OFF failure. These tests do not
execute the host FSM or the physical board.

`firmware_heartbeat_test` exercises the real host frame handler and heartbeat
service with a mock UART: 1000 ms telemetry cadence, PC-heartbeat-only refresh,
timeout despite successful TX, TX failure independence, fresh-PC-lease isolation,
repeated HELLO/query/invalid CRC rejection and tick wrap. It does not verify
physical UART TC timing or remote receipt.

`firmware_target_timeout_test` runs the real actuator loop with queue/clock
stubs: enqueue timestamps, delayed dequeue, stale-command rejection before
motor I/O, equal-target refresh, latest-only overwrite, failed/invalid/unarmed
submissions and tick wrap. It does not simulate actual RTOS task interleavings.

`firmware_control_safety_test` combines the actual host FSM and actuator source
with mocked queue/UART/clock/feedback. It covers expired-link target rejection,
pending fault/timeout target rejection, SAFE_STOP publication before state-event
TX, return before torque-off during a forced task switch, bounded query/garbage
RX draining, byte/frame/time budgets, and each of twelve blocked motors while
the others move. It checks centered-motor timer refresh and all-or-nothing
progress updates for invalid samples. Interleavings are selected simulations,
not a proof of every possible RTOS race or a physical-board safety test.

`firmware_result_tx_test` covers immediate ACK, failed-TX recovery, exact original
frame/sequence preservation, NACK retry, no repeated target enqueue or watchdog
refresh, maximum attempts, queue-age expiry, FIFO order/backpressure, fault/link
handling while the queue is full, and tick wrap. UART success/failure and elapsed
time are simulated; remote receipt and physical UART timing are not verified.

`firmware_host_rx_dma_test` compiles the actual host UART and circular RX DMA
driver with register stubs. It checks BRR and disabled RXNE/IDLE interrupts,
short bursts without a boundary IRQ, multiple wraps, pending/preempted TC,
DMA overwrite before/during a task copy, delimiter resynchronization, UART
Overrun/Framing errors, DMA transfer-error restart and 0x8A host metadata.
It verifies byte counts and separate event/overflow semantics. Selected DMA/IRQ
interleavings are simulated, not physical DMA bus timing or USB serial quality.

Run all tests under GCC in the container:

```sh
cmake -S tests/diagnostics -B build/diagnostic-tests-linux
cmake --build build/diagnostic-tests-linux
ctest --test-dir build/diagnostic-tests-linux --output-on-failure
```
