cmake_minimum_required(VERSION 3.15)

project(yertle_freertos_f103rb LANGUAGES C ASM)

# The STM32F103RB uses an ARM Cortex-M3 core.
set(FREERTOS_PORT GCC_ARM_CM3 CACHE STRING "FreeRTOS portable layer")
set(FREERTOS_HEAP 4 CACHE STRING "FreeRTOS heap implementation")

# FreeRTOS-Kernel requires this interface target to locate FreeRTOSConfig.h.
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/firmware/config)
target_compile_definitions(freertos_config INTERFACE projCOVERAGE_TEST=0)

add_subdirectory(third_party/FreeRTOS-Kernel)

add_custom_target(kernel DEPENDS freertos_kernel)

add_subdirectory(firmware)
