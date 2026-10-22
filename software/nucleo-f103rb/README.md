# Yertle FreeRTOS build container

This directory is the host workspace. Docker supplies the ARM build tools;
source code and build outputs remain on the host through the `/workspace` bind mount.

The complete original FreeRTOS application's history through `a0ac6b6` was
imported here using **git subtree without squash**. The original standalone
checkout is unchanged. New firmware work belongs in this directory; firmware
behavior, motor mappings and wire protocols were not changed by the import.

## Dependency setup (not vendoring or submodules)

`dependencies.lock.json` records the public upstream URL and exact commit for
FreeRTOS-Kernel, CMSIS_5, cmsis-device-f1 and stm32-sts3215-lib. These commits
match the original firmware's four submodule pins. The final repository does
not track library sources or gitlinks: no `git clone --recurse-submodules` is
needed. Historical imported commits retain their original submodule records.

Compose provides these services:

| Service | Purpose |
|---|---|
| `setup` | Fetch/checkout exact library commits into ignored `third_party/`; verify origin, revision and clean source trees |
| `f103-build` | Wait for setup success, recheck dependencies, configure/build/verify the ARM firmware |
| `freertos-dev` | Development shell using the same tools/dependencies |

To run just setup from this directory:

```powershell
docker compose run --build --rm setup
```

Each build also invokes the idempotent setup script, so a reused completed
Compose setup container cannot silently bypass checks. Clean, correct cached
checkouts are reused without network access. First setup requires access to
the four public GitHub repositories; initial image construction also needs
Docker Hub/Debian packages. Compiler/Debian package versions remain governed
by the existing Dockerfile; only library source commits are pinned by the lock.

Dirty/wrong-origin/wrong-revision or unmanaged library directories cause an
error: setup never resets or overwrites local work. Preserve/move that checkout
aside before preparing a changed lock version. Only newly created temporary
download directories are automatically cleaned. Concurrent setup processes
serialize through a Linux file lock in `build/`. Run the scripts in Docker.

The libraries' upstream LICENSE/copyright files remain in their fetched trees.
Do not commit `third_party/`, `build/`, container caches or firmware binaries.
Logs remain on the host at `build/logs/setup-latest.log` and
`build/logs/f103-build-latest.log`.

Setup unit tests and the existing firmware host/mock regression tests:

```powershell
docker compose run --rm freertos-dev bash -lc "python3 -m unittest discover -s tests/setup -v && cmake -S tests/diagnostics -B build/diagnostic-tests-linux && cmake --build build/diagnostic-tests-linux --parallel 4 && ctest --test-dir build/diagnostic-tests-linux --output-on-failure"
```

This repository's Git history contains both the Yertle history and the original
firmware history. The subtree import preserved original commit IDs; the later
build/setup commit adapts dependency management at the new prefix. Future
automatic subtree pulls from the standalone repository must account for that
adaptation rather than restoring old gitlinks or overwriting the lock policy.

## First build

```powershell
cd C:\Users\ik\Desktop\yertle-RTOS\temp_low_level_controller\software\nucleo-f103rb
docker compose run --build --rm f103-build
```

## Start the persistent development container

```powershell
cd C:\Users\ik\Desktop\yertle-RTOS\temp_low_level_controller\software\nucleo-f103rb
docker compose up -d freertos-dev
```

## Open the development shell

```powershell
cd C:\Users\ik\Desktop\yertle-RTOS\temp_low_level_controller\software\nucleo-f103rb
docker compose exec freertos-dev bash
```

Verify the toolchain inside the container:

```bash
arm-none-eabi-gcc --version
cmake --version
ninja --version
```

## Build the NUCLEO-F103RB FreeRTOS firmware image

The build produces a NUCLEO-F103RB image using the official Cortex-M3 FreeRTOS
port, STM32 startup code and a 128 KB flash / 20 KB RAM linker script. It runs
the host protocol/FSM, actuator and sensor tasks. USART2 remains the ST-LINK
host link; USART1 drives the FE-URT UART interface at 1 Mbps. The Mowibox pure-C
protocol layer builds and parses STS3215 packets, while the project-specific
driver serializes bus access with a FreeRTOS mutex.

```powershell
cd C:\Users\ik\Desktop\yertle-RTOS\temp_low_level_controller\software\nucleo-f103rb
docker compose run --rm f103-build
```

The service prints `BUILD SUCCESS` only after configure/build succeed and the
ELF and BIN files are nonempty. It reports ARM image memory usage, the BIN
size and modification time. Failures print `BUILD FAILED` with the failing
stage and return a nonzero exit code. `ninja: no work to do` is a valid
incremental build. These checks do not test execution on the physical board.

Use `docker compose run --build --rm f103-build` for foreground logs and
the final build result.
The Docker extension's detached Compose Up command starts containers without
waiting for the foreground build result; use the command above for logs/exit status.

Each `f103-build` run replaces `build/logs/f103-build-latest.log`, while also
streaming the same output to its container log. Open that file directly to
inspect the latest build progress and result.

The same build can be run manually in the development shell:

```powershell
docker compose exec freertos-dev bash
cmake -S . -B build/stm32f103rb -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/stm32f103rb-gcc.cmake
cmake --build build/stm32f103rb --target firmware
```

Generated firmware artifacts:

```text
build/stm32f103rb/firmware/yertle_f103rb      # ELF
build/stm32f103rb/firmware/yertle_f103rb.bin  # flash image
build/stm32f103rb/firmware/yertle_f103rb.map  # linker map
```

## PC/RPi protocol test

`USART2` is now reserved for the binary PC/RPi protocol, not the text shell.
On a NUCLEO-F103RB it is connected to the ST-LINK Virtual COM port. Flash the
BIN at `0x08000000`, then use the separately maintained C++ PC/RPi client
(the original local `yertle-RTOS/pc-connect` or `pc-connect-api`). Neither PC
project is imported here. The wire
format and phase-1 message set are documented in
[`firmware/PROTOCOL.md`](firmware/PROTOCOL.md).

Stop the persistent development container when it is not needed:

```powershell
docker compose down
```

For an isolated one-off shell instead, use:

```powershell
docker compose run --rm freertos-dev bash
```

## Source layout

```text
nucleo-f103rb/
├─ Dockerfile
├─ docker-compose.yml
├─ firmware/          # STM32F103 FreeRTOS application and board drivers
├─ dependencies.lock.json  # pinned upstream source revisions
├─ third_party/       # downloaded/verified by setup; ignored by Git
└─ build/             # generated; ignored by Docker build context
```

The Mowibox HAL/RS485 transport is intentionally not compiled. The FE-URT
module exposes separate UART TX/RX pins, so this firmware uses its own
F103 DMA port of Mowibox's transport pattern and its original hardware-independent
packet protocol. The G4-specific `sts3215_hal.c` is NOT linked unchanged.
RX DMA1 Channel5 is armed before TX DMA1 Channel4. Task-owned mutex/parsing,
50 Hz feedback and the 1 Mbps motor bus remain unchanged. The host link now
uses 1 Mbps (8N1); rebuild the matching PC client and flash the new firmware.
See [host-link timing and migration](firmware/HOST_LINK.md).

## 60-second DMA comparison

Flash `build/stm32f103rb/firmware/yertle_f103rb.bin` at 0x08000000 and use the
rebuilt pc-connect client from its own directory:

```powershell
.\measure-uart.ps1 -Port COM10
```

Replace COM10 with ST-LINK VCP, not the servo module USB. Keep the already-working
UART wiring and disconnect servo-module USB/other bus masters. No ARM/heartbeat
or motion commands are sent by the monitor. Existing baseline logs are untouched.
The terminal must show `MOTOR transport=Mowibox-pattern F103 DMA port`.
Summary adds complete twelve-servo SyncRead and full two-block feedback cycle
percentages. RX IRQ counts are now IDLE/error events, not byte interrupts; motor
ISR time now includes UART plus TX/RX DMA handler bodies. Compare error deltas,
valid responses/timeouts and complete-cycle rates, NOT IRQ count per byte.
This build/test does not verify physical board execution or certify safety.

## Motor interface wiring

| NUCLEO-F103RB | Peer signal direction (not printed module pin name) | Purpose |
|---|---|---|
| PA9 / USART1_TX | RX | F103 to servo adapter |
| PA10 / USART1_RX | TX | Servo adapter to F103 |
| GND | GND | Common signal ground |

The servo supply must come from the external servo power source, not from the
Nucleo 3.3 V rail. Verify the FE-URT UART logic voltage before connecting it.
The generic RX/TX directions above are not the FE-URT-2 header labels. Retain the
confirmed working FE-URT-2 hookup; a DMA firmware change does not require rewiring.
