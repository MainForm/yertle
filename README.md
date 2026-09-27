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
CMSIS-based USART1 transport and reuses only Mowibox's hardware-independent
packet protocol.

## Motor interface wiring

| NUCLEO-F103RB | FE-URT UART side | Purpose |
|---|---|---|
| PA9 / USART1_TX | RX | F103 to servo adapter |
| PA10 / USART1_RX | TX | Servo adapter to F103 |
| GND | GND | Common signal ground |

The servo supply must come from the external servo power source, not from the
Nucleo 3.3 V rail. Verify the FE-URT UART logic voltage before connecting it.
