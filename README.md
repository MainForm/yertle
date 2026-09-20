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

## Build the STM32F103 FreeRTOS kernel

The first build compiles the official FreeRTOS kernel for Cortex-M3 into a
static library. It intentionally does not create a flashable firmware image
yet: that needs the chosen board's startup file, linker script and HAL source.

```powershell
cd C:\Users\ik\Desktop\yertle-RTOS\freeRTOS
docker compose run --rm freertos-build
```

The same build can be run manually in the development shell:

```powershell
docker compose exec freertos-dev bash
cmake -S . -B build/stm32f103re -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/stm32f103re-gcc.cmake
cmake --build build/stm32f103re --target kernel
```

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
