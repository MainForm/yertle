# A bounded ARM call-path check, not a substitute for live stack-water marks.
# Reserve includes RTOS/library callees and Cortex-M task/exception context.
if(NOT DEFINED BUILD_DIR OR NOT DEFINED SOURCE_DIR)
    message(FATAL_ERROR "BUILD_DIR and SOURCE_DIR required")
endif()
file(READ "${SOURCE_DIR}/firmware/src/main.c" main_source)
file(READ "${SOURCE_DIR}/firmware/config/FreeRTOSConfig.h" config_source)
if(NOT main_source MATCHES "xTaskCreate\\(sensor_task,[^;]*configMINIMAL_STACK_SIZE \\* ([0-9]+)U")
    message(FATAL_ERROR "Cannot determine sensor stack multiplier")
endif()
set(multiplier "${CMAKE_MATCH_1}")
if(NOT config_source MATCHES "#define configMINIMAL_STACK_SIZE[ \t]+([0-9]+)")
    message(FATAL_ERROR "Cannot determine minimal stack words")
endif()
math(EXPR stack_bytes "${multiplier} * ${CMAKE_MATCH_1} * 4")
set(sensor_stack_bytes ${stack_bytes})
file(GLOB_RECURSE reports "${BUILD_DIR}/firmware/*.su")
if(NOT reports)
    message(FATAL_ERROR "ARM .su reports missing; compile with -fstack-usage")
endif()
foreach(report IN LISTS reports)
    file(STRINGS "${report}" lines)
    foreach(line IN LISTS lines)
        if(line MATCHES ":([^:\t]+)\t([0-9]+)\tstatic$")
            set("frame_${CMAKE_MATCH_1}" "${CMAKE_MATCH_2}")
        endif()
    endforeach()
endforeach()
function(check_path label)
    set(total 0)
    foreach(name IN LISTS ARGN)
        if(NOT DEFINED "frame_${name}")
            message(FATAL_ERROR "Missing static frame for ${name}; review call graph/compiler")
        endif()
        math(EXPR total "${total} + ${frame_${name}}")
    endforeach()
    math(EXPR reserved "${total} + 256")
    if(reserved GREATER stack_bytes)
        message(FATAL_ERROR "Sensor ${label}: ${total}+256 reserve > ${stack_bytes} bytes")
    endif()
    message(STATUS "Sensor ${label}: ${total}+256 reserve <= ${stack_bytes} bytes")
endfunction()
if(NOT main_source MATCHES "xTaskCreate\\(actuator_task,[^)]*configMINIMAL_STACK_SIZE \\* ([0-9]+)U")
 message(FATAL_ERROR "Cannot determine actuator stack multiplier")
endif()
set(actuator_multiplier "${CMAKE_MATCH_1}")
if(NOT config_source MATCHES "#define configMINIMAL_STACK_SIZE[ \t]+([0-9]+)")
 message(FATAL_ERROR "Cannot determine minimal stack words")
endif()
math(EXPR stack_bytes "${actuator_multiplier} * ${CMAKE_MATCH_1} * 4")
check_path(actuator_calibrated_position actuator_task motor_control_service service_drive service_pid
    calibration_cache_position calibration_cache_get)
check_path(actuator_pid_publish actuator_task motor_control_service service_drive service_pid publish_pid_trace trace_float)
check_path(actuator_position_output actuator_task motor_control_service service_drive send_drive_output
    motor_device_write_targets send_broadcast motor_uart_write)
check_path(actuator_pwm_output actuator_task motor_control_service service_drive service_pid send_drive_output
    motor_device_write_pwm send_broadcast motor_uart_write)
check_path(actuator_torque_on actuator_task motor_control_service service_drive service_pid send_drive_output
    motor_device_torque_mask send_broadcast motor_uart_write)
if(NOT main_source MATCHES "xTaskCreate\\(host_protocol_task,[^)]*configMINIMAL_STACK_SIZE \\* ([0-9]+)U")
 message(FATAL_ERROR "Cannot determine host stack multiplier")
endif()
set(host_multiplier "${CMAKE_MATCH_1}")
if(NOT config_source MATCHES "#define configMINIMAL_STACK_SIZE[ \t]+([0-9]+)")
 message(FATAL_ERROR "Cannot determine minimal stack words")
endif()
math(EXPR stack_bytes "${host_multiplier} * ${CMAKE_MATCH_1} * 4")
set(host_stack_bytes ${stack_bytes})
set(common host_protocol_task service_motor_register_write motor_device_write_register write_register_transaction)
check_path(permission ${common} register_access_allowed)
check_path(lock_read ${common} restore_register_lock_locked read_register_timed
    motor_uart_read_frame read_byte)
check_path(lock_write ${common} restore_register_lock_locked write_register_timed
    motor_uart_write)
check_path(target_read ${common} read_for_report read_register_timed
    motor_uart_read_frame read_byte)
check_path(target_write ${common} write_for_report write_register_timed motor_uart_write)
# Catch a full control_context snapshot returning to this hot callback, even
# when a larger task allocation would otherwise conceal the regression.
if(frame_register_access_allowed GREATER 64)
    message(FATAL_ERROR "Mode-write permission frame exceeds 64bytes; avoid full context copies")
endif()


# A calibration-covering generic GET persists below the bus-lock read path.
check_path(calibration_query host_protocol_task service_motor_register motor_device_read_register
    read_register_locked read_register_timed calibration_cache_observe calibration_store_save
    calibration_scan checksum)
check_path(calibration_program host_protocol_task service_motor_register motor_device_read_register
    read_register_locked read_register_timed calibration_cache_observe calibration_store_save
    calibration_flash_program flash_operation)
check_path(calibration_dedicated host_protocol_task service_position_offsets motor_device_read_position_offset
    calibration_cache_observe calibration_store_save calibration_scan checksum)
set(calibration_write host_protocol_task service_motor_register_write motor_device_write_register write_register_transaction)
check_path(calibration_invalidate ${calibration_write} calibration_cache_invalidate
    calibration_store_save calibration_scan checksum)
check_path(calibration_verify ${calibration_write} calibration_cache_observe
    calibration_store_save calibration_scan checksum)

check_path(host_pid_telemetry host_protocol_task report_host_state service_pid_detail send_frame send_frame_sequence
    host_uart_write_bytes wait_tx_flag)

math(EXPR stack_bytes "${actuator_multiplier} * ${CMAKE_MATCH_1} * 4")
set(actuator_verify actuator_task motor_control_service)
set(mode_read motor_device_read_register read_register_locked read_register_timed motor_uart_read_frame read_byte)
check_path(actuator_boot_mode_verify ${actuator_verify} service_motor_boot motor_device_read_modes ${mode_read})
check_path(actuator_boot_off_verify ${actuator_verify} service_motor_boot
    motor_device_verify_torque_off motor_device_verify_torque_off_detailed ${mode_read})
set(actuator_stop actuator_task motor_control_service service_stop)
set(stop_off_verify motor_safe_stop_service motor_device_verify_torque_off motor_device_verify_torque_off_detailed
    motor_device_read_register read_register_locked read_register_timed motor_uart_read_frame read_byte)
check_path(actuator_return_pid ${actuator_stop} motor_safe_stop_service service_return send_pwm_return position_pid_step)
check_path(actuator_return_calibration ${actuator_stop} motor_safe_stop_service service_return prepare_stop_feedback
    calibration_cache_position calibration_cache_get)
check_path(actuator_pwm_return_tx ${actuator_stop} motor_safe_stop_service service_return send_pwm_return
    motor_device_write_pwm STS3215_BuildSyncWrite prv_pack_frame prv_checksum)
check_path(actuator_position_return_tx ${actuator_stop} motor_safe_stop_service service_return send_position_return
    motor_device_write_targets STS3215_BuildSyncWrite prv_pack_frame prv_checksum)
check_path(actuator_stop_off_verify ${actuator_stop} ${stop_off_verify})
check_path(actuator_stop_mode_verify ${actuator_stop} motor_device_read_modes ${mode_read})
check_path(actuator_return_telemetry ${actuator_stop} publish_stop)

set(stack_bytes ${sensor_stack_bytes})
set(imu_recovery sensor_task imu_device_init imu_i2c_init recover_lines)
check_path(imu_bus_clear ${imu_recovery} clear_stuck_sda wait_pin us_to_cycles)
check_path(imu_busy_workaround ${imu_recovery} apply_busy_erratum_workaround wait_pin us_to_cycles)
check_path(imu_probe sensor_task imu_device_init imu_i2c_read select_register address_device wait_event wait_status us_to_cycles)
check_path(imu_sample sensor_task imu_device_read imu_i2c_read receive_multiple wait_event wait_status us_to_cycles)

check_path(optical_init_read sensor_task pmw3901_device_init check_id read_spi
    pmw3901_spi_read exchange wait_flag capture_failure)
check_path(optical_init_write sensor_task pmw3901_device_init write_spi
    pmw3901_spi_write exchange wait_flag capture_failure)
check_path(optical_burst sensor_task pmw3901_device_service read_spi
    pmw3901_spi_read exchange wait_flag)

set(stack_bytes ${host_stack_bytes})
check_path(host_optical_reply host_protocol_task process_host_rx handle_frame send_frame send_frame_sequence host_uart_write_bytes wait_tx_flag)

set(stack_bytes ${sensor_stack_bytes})
check_path(feedback_failure_capture sensor_task motor_device_read_feedback read_sync_block
    motor_uart_read_frame read_byte motor_uart_failure_note motor_dma_failure_snapshot)
set(stack_bytes ${host_stack_bytes})
check_path(host_uart_failure_reply host_protocol_task process_host_rx handle_frame
    uart_diagnostics_send_motor_buses send_diagnostic_frame send_frame send_frame_sequence
    host_uart_write_bytes wait_tx_flag)

set(stack_bytes ${sensor_stack_bytes})
check_path(optical_failure_capture sensor_task pmw3901_device_service check_id read_spi
    pmw3901_spi_read exchange wait_flag capture_failure)
set(stack_bytes ${host_stack_bytes})
check_path(host_spi_failure_reply host_protocol_task process_host_rx handle_frame
 pmw3901_spi_send_diagnostics send_diagnostic_frame send_frame send_frame_sequence host_uart_write_bytes wait_tx_flag)

set(stack_bytes ${host_stack_bytes})
check_path(host_automatic_spi host_protocol_task report_host_state service_optical_failure send_frame send_frame_sequence host_uart_write_bytes wait_tx_flag)
set(stack_bytes ${sensor_stack_bytes})
check_path(optical_budget_end sensor_task pmw3901_device_init check_id read_spi
    pmw3901_spi_call_end capture_failure)

file(READ "${SOURCE_DIR}/firmware/linker/stm32f103rb_settings.ld" linker_source)
if(NOT linker_source MATCHES "_Min_Stack_Size = (0x[0-9a-fA-F]+);")
    message(FATAL_ERROR "Cannot determine startup stack reservation")
endif()
math(EXPR stack_bytes "${CMAKE_MATCH_1}")
check_path(optical_startup_read main pmw3901_device_init check_id read_spi
    pmw3901_spi_read exchange wait_flag capture_failure)
check_path(optical_startup_write main pmw3901_device_init write_spi
    pmw3901_spi_write exchange wait_flag capture_failure)
check_path(optical_startup_budget main pmw3901_device_init check_id read_spi
    pmw3901_spi_call_end capture_failure)

set(imu_startup main imu_device_init imu_i2c_init recover_lines)
check_path(imu_startup_bus_clear ${imu_startup} clear_stuck_sda wait_pin us_to_cycles)
check_path(imu_startup_busy_workaround ${imu_startup} apply_busy_erratum_workaround wait_pin us_to_cycles)
check_path(imu_startup_probe main imu_device_init imu_i2c_read select_register address_device wait_event wait_status us_to_cycles)
check_path(imu_startup_write main imu_device_init imu_i2c_write select_register address_device wait_event wait_status us_to_cycles)
check_path(imu_startup_read_abort main imu_device_init imu_i2c_read abort_transfer finish_transfer wait_status us_to_cycles)
check_path(imu_startup_write_abort main imu_device_init imu_i2c_write abort_transfer finish_transfer wait_status us_to_cycles)
