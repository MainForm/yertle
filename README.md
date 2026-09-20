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
It currently runs a scheduler smoke-test task only; HAL/LL, servo UART, Mowibox
and robot-control tasks are added in later steps.

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

## Flash verification shell

The base image exposes a small FreeRTOS shell on `USART2`, 115200 baud, 8N1.
On a NUCLEO-F103RB, this is normally connected to the ST-Link Virtual COM port;
on a standalone STM32F103RE board, connect a 3.3 V USB-UART adapter to PA2 (TX),
PA3 (RX), and GND. After flashing the BIN at `0x08000000` and resetting, open
the serial port and use `help`, `status`, `heap`, or `reboot`.

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

The image contains only generic ARM tooling. STM32 startup files, linker script,
FreeRTOS kernel sources, Mowibox, and the F103 UART adapter are deliberately not
included yet; they will be added to `firmware/` and `third_party/` as source code.
