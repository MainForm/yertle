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
check port ownership, 64-bit accumulation carry, error counts and all five
response layouts. Diagnostic critical-section macros intentionally fail to
compile. This does not simulate interrupt interleavings or validate electrical
signals, interrupt latency, or the existing driver's RX-ring protection.
