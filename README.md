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

The build produces a base NUCLEO-F103RB image using the official Cortex-M3
FreeRTOS port, STM32 startup code and a 128 KB flash / 20 KB RAM linker script.
It starts the FreeRTOS host-protocol task. Its explicit safety state machine is
`BOOT → DISARMED`, with `ARMED` and `FAULT` as the remaining runtime states.
Timeouts and recoverable safety limits transition directly to `DISARMED` while
requesting a safe stop. HAL/LL, servo UART, Mowibox and robot-control tasks are
added later.

```powershell
cd C:\Users\ik\Desktop\yertle-RTOS\freeRTOS
docker compose run --rm f103-build
```

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

## Intended source layout

```text
freeRTOS/
├─ Dockerfile
├─ docker-compose.yml
├─ firmware/          # STM32F103 FreeRTOS application (next step)
├─ third_party/       # FreeRTOS-Kernel, Mowibox, STM32 HAL, etc.
└─ build/             # generated; ignored by Docker build context
```

The image contains only generic ARM tooling. Mowibox and the F103 servo-UART
adapter are deliberately not included yet; they will be added to `firmware/`
and `third_party/` as source code.
