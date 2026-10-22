# PC/RPi host link: 1 Mbps

The PC/RPi USART2 link uses **1,000,000 baud, 8N1**, with no hardware or software
flow control. Update firmware and client together. Existing executables built
for 115200 cannot talk to this firmware. USART1's 1 Mbps motor-bus configuration,
motor IDs/calibration and all binary message layouts remain unchanged.

## Configuration

- Firmware: `HOST_UART_BAUD` in `include/host_uart.h`.
- PC CLI and API: `yertle::kHostBaud` in each project's `include/protocol.hpp`.
- Public `Client(port)` and `serialTransport(port)` use that default. An explicit
  baud argument still overrides the API default; it must match the firmware.
- Windows passes 1,000,000 to `SetCommState`. Linux/RPi selects `B1000000` when
  supplied by termios. No automatic baud negotiation or fallback is added.
- USART2 uses the 32 MHz APB1 clock. The integer BRR value is 32 (`0x20`), so
  divider rounding contributes zero error. HSI oscillator tolerance remains.
- PC UART utilization percentages now assume 1 Mbps for USART2, independently
  in each direction. No diagnostic payload format changes are required.

## Task periods and deadlines

| Work | Current setting | Migration decision |
|---|---|---|
| Host protocol | Priority 3, 2 ms delay after each bounded pass | Keep; baud is not a task rate |
| Actuator | Priority 4, nominal 10 ms | Keep; latest target is applied/repeated at the existing cadence |
| Sensor | Priority 2, nominal 20 ms | Keep; fresh motor/IMU snapshots are nominally 50 Hz |
| PC heartbeat | Existing API default 250 ms | Keep |
| Host-link watchdog | 1000 ms, accepted PC heartbeat RX only | Keep |
| Target watchdog | 500 ms since successful target enqueue | Keep |
| F103 heartbeat | 1000 ms status report | Keep |
| Host TX / ACK retry | 20 ms per TX, 10 ms retry interval, 3 attempts, 250 ms age | Keep bounded failure behavior |

Actual loop times include bus mutex waits, software I2C, UART writes and task
preemption. A higher baud does not change the servo's position-control loop,
increase the 50 Hz sensor acquisition rate, or guarantee a shorter USB latency.
Do not shorten safety/watchdog deadlines simply in proportion to baud.

## Bandwidth and remaining receive limits

8N1 uses ten line bits per byte. Per-direction line capacity rises from
11,520 to **100,000 byte/s**. A 12-target request is 72 bytes on the wire, about
0.72 ms at 1 Mbps. Three motor feedback frames and one IMU frame total 240 bytes,
about 2.4 ms, excluding ACK/heartbeat and other processing. Sending that sensor
set at 50 Hz consumes about 12,000 byte/s in the F103-to-PC direction.

USART2 now uses **DMA1 Channel6 circular RX with a 1024-byte buffer**. At
continuous full-rate RX the buffer covers **10.24 ms**. RXNEIE and IDLEIE are
disabled. The task polls CNDTR for partial bursts; half/full DMA interrupts
maintain wrap bookkeeping, and the USART IRQ handles hardware errors only.
DMA does not depend on the task reading DR within a 10 us byte interval.
Normal reads use a bounded lock-free producer snapshot and check that DMA did
not overwrite the candidate while the task was preempted. Only hardware
restart briefly guards the USART2/DMA6 IRQs; TX and diagnostic copies never do.
There is no hardware flow control. DMA alone does not guarantee sustained
100,000-byte/s parsing: the host pass consumes at most 128 bytes / two frames /
8 ms and then delays 2 ms. Diagnostic TX and higher-priority actuator work can
delay draining further. Pace application requests and do not flood the line.

Inspect Overrun Error, DMA overwrite count, RX peak and ISR CPU time under the
actual combined target/feedback workload. The task must drain the buffer, and
the DMA full interrupt must be serviced at least once per buffer revolution
for wrap accounting. A delayed IRQ cannot reconstruct arbitrarily many wraps.
On UART/DMA errors RX pauses, the task rearms it and discards through the next
COBS delimiter. Overwritten data is also discarded with a parser discontinuity.
Parser budget/overload behavior still needs review if sustained maximum RX is
required. USART2 RX channel6 is independent of motor channels5/4; channel7 TX
is unused and host TX stays polling. Task periods and timeouts are unchanged.

`GET_UART_STATS` now uses former reserved 0x8A words13..15 for host RX mode,
capacity and DMA errors. USART2 rx_irq counts UART error events, not bytes;
ISR profiling includes DMA6 half/full/error handlers. RX bytes and peak come
from task-observed DMA progress/backlog. Rebuild either PC client to display
capacity1024 and these semantics correctly.

The DMA regression suite includes short bursts, repeated/pending/preempted
wraps, overwrite before/during CPU copying, error pause/restart and delimiter
resynchronization. USART2 DMA ISR bodies do not parse frames or use RTOS APIs.

## Verification and rollout

Build firmware with the existing `f103-build` Compose service, then flash
`build/stm32f103rb/firmware/yertle_f103rb.bin` at `0x08000000`. Build the matching
PC executable/API and restart applications that link it. Existing caller
binaries using C++ default arguments must also be recompiled.

Read-only PC test (from `pc-connect`, no motion command):

```powershell
.\build\Release\yertle_pc_connect.exe --port COM10 uart-monitor --duration-ms 60000 --interval-ms 1000
```

This diagnostic proves only the traffic exercised by that test. A second test
must exercise the intended target/feedback rate before claiming that workload
is reliable. ST-LINK VCP driver/firmware and physical board behavior require
hardware testing; build/unit tests do not establish error-free 1 Mbps operation.

## Code review order

Existing working changes were split before the baud migration. These are local
commits in separate repositories, not one combined snapshot:

| Repository | Commit | Review unit / entry points |
|---|---|---|
| freeRTOS | `7f8d4f5` | Offset driver and sensor job: `motor_device_read_position_offset`, `collect_next_position_offset` |
| freeRTOS | `2777491` | Host reliability: `host_uart_write_bytes`, `service_result_tx`, `expire_host_link`, `process_host_rx` |
| freeRTOS | `e34e23d` | Actuator safety: `actuator_submit_targets`, `stop_has_stalled`, `center_reached` |
| pc-connect | `e6c6285` | Position-offset message decoder and protocol tests |
| pc-connect | `32ea14e` | Motor/IMU CSV capture and timed IMU monitor |
| pc-connect | `733fe95` | Saved poses, named sequences, position monitor and CLI integration |
| pc-connect-api | `efb917e` | Windows serial I/O and first-error retention |
| pc-connect-api | `c8c2913` | Body-velocity support transitions and zero-velocity diagnostics |

The next commit in each repository contains only that repository's 1 Mbps
configuration, related diagnostics/tests and documentation. Initial 1 Mbps
migration verification (before circular RX DMA):
F103RB Compose build/artifact checks passed, firmware CTest 8/8, PC CLI CTest
40/40 and API CTest 35/35 (including the installed external consumer). Linux
serial/transport syntax checks passed with `B1000000`. Physical serial traffic
was not exercised, and no firmware flashing or motor command was performed.

Circular RX DMA verification: F103RB Compose build/artifact checks passed,
firmware tests 9/9 on both Windows and GCC, PC CLI tests 40/40 and API tests
35/35 including the installed consumer. ELF symbols resolve USART2 and DMA6 to
the new strong handlers. RX-only transport changes leave TX, task periods,
watchdogs and motor bus configuration unchanged. Hardware error rate after
flashing remains to be measured.
