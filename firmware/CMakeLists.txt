set(STM32F1_DEVICE_DIR ${CMAKE_SOURCE_DIR}/third_party/cmsis-device-f1)
set(CMSIS_CORE_DIR ${CMAKE_SOURCE_DIR}/third_party/CMSIS_5/CMSIS/Core/Include)
set(STM32F103RE_LINKER_SCRIPT
    ${STM32F1_DEVICE_DIR}/Source/Templates/gcc/linker/STM32F103XE_FLASH.ld)

add_executable(yertle_f103
    src/main.c
    ${STM32F1_DEVICE_DIR}/Source/Templates/system_stm32f1xx.c
    ${STM32F1_DEVICE_DIR}/Source/Templates/gcc/startup_stm32f103xe.s)

target_compile_definitions(yertle_f103 PRIVATE STM32F103xE)
target_include_directories(yertle_f103 PRIVATE
    ${STM32F1_DEVICE_DIR}/Include
    ${CMSIS_CORE_DIR})
target_link_libraries(yertle_f103 PRIVATE freertos_kernel)
target_link_options(yertle_f103 PRIVATE
    -nostartfiles
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,--gc-sections
    -Wl,-T,${STM32F103RE_LINKER_SCRIPT}
    -Wl,-Map,${CMAKE_CURRENT_BINARY_DIR}/yertle_f103.map)

add_custom_command(TARGET yertle_f103 POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:yertle_f103>
            ${CMAKE_CURRENT_BINARY_DIR}/yertle_f103.bin
    BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/yertle_f103.bin
    COMMENT "Generating yertle_f103.bin")

add_custom_target(firmware DEPENDS yertle_f103)
