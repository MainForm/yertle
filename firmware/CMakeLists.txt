set(STM32F1_DEVICE_DIR ${CMAKE_SOURCE_DIR}/third_party/cmsis-device-f1)
set(CMSIS_CORE_DIR ${CMAKE_SOURCE_DIR}/third_party/CMSIS_5/CMSIS/Core/Include)
set(STS3215_DIR ${CMAKE_SOURCE_DIR}/third_party/stm32-sts3215-lib)
set(STM32F103RB_LINKER_SCRIPT
    ${STM32F1_DEVICE_DIR}/Source/Templates/gcc/linker/STM32F103XB_FLASH.ld)

add_executable(yertle_f103rb
    src/main.c
    src/board_clock.c
    src/host_uart.c
    src/host_protocol.c
    src/motor_uart.c
    src/uart_diagnostics.c
    src/motor_device.c
    src/actuator_task.c
    src/sensor_task.c
    ${STS3215_DIR}/src/sts3215_protocol.c
    ${STM32F1_DEVICE_DIR}/Source/Templates/system_stm32f1xx.c
    ${STM32F1_DEVICE_DIR}/Source/Templates/gcc/startup_stm32f103xb.s)

target_compile_definitions(yertle_f103rb PRIVATE STM32F103xB)
target_compile_options(yertle_f103rb PRIVATE -Wall -Wextra)
target_include_directories(yertle_f103rb PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${STS3215_DIR}/inc
    ${STM32F1_DEVICE_DIR}/Include
    ${CMSIS_CORE_DIR})
target_link_libraries(yertle_f103rb PRIVATE freertos_kernel)
target_link_options(yertle_f103rb PRIVATE
    -nostartfiles
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,--gc-sections
    -Wl,-T,${STM32F103RB_LINKER_SCRIPT}
    -Wl,-Map,${CMAKE_CURRENT_BINARY_DIR}/yertle_f103rb.map)

add_custom_command(TARGET yertle_f103rb POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:yertle_f103rb>
            ${CMAKE_CURRENT_BINARY_DIR}/yertle_f103rb.bin
    BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/yertle_f103rb.bin
    COMMENT "Generating yertle_f103rb.bin")

add_custom_target(firmware DEPENDS yertle_f103rb)
