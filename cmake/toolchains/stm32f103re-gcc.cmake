set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m3)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_RANLIB arm-none-eabi-ranlib)

set(STM32F103RE_CPU_FLAGS "-mcpu=cortex-m3 -mthumb -mfloat-abi=soft")
set(CMAKE_C_FLAGS_INIT "${STM32F103RE_CPU_FLAGS} -ffreestanding -fdata-sections -ffunction-sections")
set(CMAKE_ASM_FLAGS_INIT "${STM32F103RE_CPU_FLAGS}")
