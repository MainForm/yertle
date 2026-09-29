# Yertle FreeRTOS build container

This directory is the host workspace. Docker supplies the ARM build tools;
source code and build outputs remain on the host through the `/workspace` bind mount.

## First build

```powershell
cd C:\Users\ik\Desktop\yertle-RTOS\freeRTOS
docker compose build
```

## Start the persistent development container

```powershell
cd C:\Users\ik\Desktop\yertle-RTOS\freeRTOS
docker compose up -d
```

## Open the development shell

```powershell
cd C:\Users\ik\Desktop\yertle-RTOS\freeRTOS
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
cd C:\Users\ik\Desktop\yertle-RTOS\freeRTOS
docker compose run --rm f103-build
```

The service prints `BUILD SUCCESS` only after configure/build succeed and the
ELF and BIN files are nonempty. It reports ARM image memory usage, the BIN
size and modification time. Failures print `BUILD FAILED` with the failing
stage and return a nonzero exit code. `ninja: no work to do` is a valid
incremental build. These checks do not test execution on the physical board.

In VS Code, open this directory and press `Ctrl+Shift+B` to run
`F103: build and verify`. This task runs Compose in the foreground, streams
image/firmware progress into the same terminal, and waits for the final result.
The equivalent shell command is `docker compose run --build --rm f103-build`.
The Docker extension's detached Compose Up command only starts a container;
use the build task when you want its firmware logs and exit result inline.

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
BIN at `0x08000000`, then use the C++ client in `../pc-connect`. The wire
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
freeRTOS/
├─ Dockerfile
├─ docker-compose.yml
├─ firmware/          # STM32F103 FreeRTOS application and board drivers
├─ third_party/       # FreeRTOS, CMSIS and Mowibox STS3215 protocol
└─ build/             # generated; ignored by Docker build context
```

The Mowibox HAL/RS485 transport is intentionally not compiled. The FE-URT
module exposes separate UART TX/RX pins, so this firmware uses its own
F103 DMA port of Mowibox's transport pattern and its original hardware-independent
packet protocol. The G4-specific `sts3215_hal.c` is NOT linked unchanged.
RX DMA1 Channel5 is armed before TX DMA1 Channel4. Task-owned mutex/parsing,
50 Hz feedback, 1 Mbps motor bus and 115200 host link remain unchanged.

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
Summary adds complete six-servo SyncRead and full two-block feedback cycle
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
