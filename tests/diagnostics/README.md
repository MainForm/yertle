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
It does not emulate DMA bus arbitration or electrical/interruption timing.

The safe-stop test also covers single FAULT return attempts, torque-off-only
fallback, successful completion and continued OFF failure. These tests do not
execute the host FSM or the physical board.

Run all tests under GCC in the container:

```sh
cmake -S tests/diagnostics -B build/diagnostic-tests-linux
cmake --build build/diagnostic-tests-linux
ctest --test-dir build/diagnostic-tests-linux --output-on-failure
```
