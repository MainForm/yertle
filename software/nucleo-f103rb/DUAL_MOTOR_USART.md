# Dual motor bus (USART1 / USART3)

The `codex/dual-motor-usart` branch retains the PC protocol, 12-motor order,
calibration, PID parameters and one motor bus mutex. Motor transactions are
serialized: two UARTs do not imply simultaneous motor communication.

| Motor IDs | MCU UART | MCU pins, default mapping | DMA1 TX / RX |
| --- | --- | --- | --- |
| 1-6 | USART1 | PA9 TX / PA10 RX | channels 4 / 5 |
| 7-12 | USART3 | PB10 TX / PB11 RX | channels 2 / 3 |

USART2 remains the PC link. Both motor UARTs use 1 Mbps. The FE-URT-2 UART
header must be set to the MCU's 3.3 V logic level; connect TX to RX, RX to TX,
and signal ground. Each board has two TTL servo sockets for its assigned two
legs. Inspect the physical Nucleo board, FE-URT-2 revision, pin assignments,
and power routing with power disconnected before energizing. Do not connect
the two independent servo data buses or inadvertently parallel separate
power supplies. Motor supply current and cable/connector limits still apply.

Sync write/read requests are split by bus and replies merged by motor ID.
Mode/register/calibration requests use the ID's assigned bus. Feedback
requires both the motion and current block for a motor; a missing bus yields
a partial valid mask. On a transport failure the existing global failure
result is returned; both buses are attempted for torque-off. Broadcast TX
success is not confirmation that a motor received a packet or disabled torque.
Verify per-motor state where possible; communication loss prevents confirming
the disconnected motors. Existing UART motor diagnostics currently aggregate
both motor buses in the original slot rather than identifying the failing bus.

The host feedback timestamp is assigned after sequential queries across both
buses, not simultaneously per motor. End-to-end update rate, inter-bus skew,
DMA errors, runtime RAM/stack high-water and stop behavior require hardware
measurement. The tests are mocks, not a board or gait validation.

Build without overwriting the existing single-bus firmware:

```powershell
docker compose run --rm freertos-dev bash -lc 'cmake -S . -B build/dual-usart -G Ninja -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/toolchains/stm32f103rb-gcc.cmake && cmake --build build/dual-usart --target firmware'
```

Output: `build/dual-usart/firmware/yertle_f103rb.bin`. Do not use this build
with all 12 motors on USART1: IDs 7-12 are queried on USART3. Keep the prior
single-bus firmware as a separate recovery artifact.
