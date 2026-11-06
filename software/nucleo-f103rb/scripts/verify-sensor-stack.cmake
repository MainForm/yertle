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
set(common sensor_task collect_motor_register_write motor_device_write_mode
    write_mode_once write_register_transaction)
check_path(permission ${common} register_write_allowed pwm_control_mode_write_allowed)
check_path(lock_read ${common} restore_register_lock_locked read_register_timed
    motor_uart_read_frame read_byte)
check_path(lock_write ${common} restore_register_lock_locked write_register_timed
    motor_uart_write)
check_path(target_read ${common} read_for_report read_register_timed
    motor_uart_read_frame read_byte)
check_path(target_write ${common} write_for_report write_register_timed motor_uart_write)
# Catch a full control_context snapshot returning to this hot callback, even
# when a larger task allocation would otherwise conceal the regression.
if(frame_pwm_control_mode_write_allowed GREATER 64)
    message(FATAL_ERROR "Mode-write permission frame exceeds 64bytes; avoid full context copies")
endif()
message(STATUS "Sensor ARM stack budget passed (live high-water validation still required)")

# A calibration-covering generic GET persists below the bus-lock read path.
check_path(calibration_query sensor_task collect_motor_register motor_device_read_register
    read_register_locked read_register_timed calibration_cache_observe calibration_store_save
    calibration_scan checksum)
check_path(calibration_program sensor_task collect_motor_register motor_device_read_register
    read_register_locked read_register_timed calibration_cache_observe calibration_store_save
    calibration_flash_program flash_operation)
check_path(calibration_dedicated sensor_task collect_next_position_offset motor_device_read_position_offset
    calibration_cache_observe calibration_store_save calibration_scan checksum)

if(NOT main_source MATCHES "xTaskCreate\\(actuator_task,[^)]*configMINIMAL_STACK_SIZE \\* ([0-9]+)U")
 message(FATAL_ERROR "Cannot determine actuator stack multiplier")
endif()
set(actuator_multiplier "${CMAKE_MATCH_1}")
if(NOT config_source MATCHES "#define configMINIMAL_STACK_SIZE[ \t]+([0-9]+)")
 message(FATAL_ERROR "Cannot determine minimal stack words")
endif()
math(EXPR stack_bytes "${actuator_multiplier} * ${CMAKE_MATCH_1} * 4")
check_path(actuator_calibrated_position actuator_task pwm_control_service service_pid
    calibration_cache_position calibration_cache_get)
check_path(actuator_pid_publish actuator_task pwm_control_service service_pid publish_pid_trace trace_float)
if(NOT main_source MATCHES "xTaskCreate\\(host_protocol_task,[^)]*configMINIMAL_STACK_SIZE \\* ([0-9]+)U")
 message(FATAL_ERROR "Cannot determine host stack multiplier")
endif()
set(host_multiplier "${CMAKE_MATCH_1}")
if(NOT config_source MATCHES "#define configMINIMAL_STACK_SIZE[ \t]+([0-9]+)")
 message(FATAL_ERROR "Cannot determine minimal stack words")
endif()
math(EXPR stack_bytes "${host_multiplier} * ${CMAKE_MATCH_1} * 4")
set(host_stack_bytes ${stack_bytes})
check_path(host_pid_telemetry host_protocol_task service_pid_detail send_frame send_frame_sequence
    host_uart_write_bytes wait_tx_flag)

math(EXPR stack_bytes "${actuator_multiplier} * ${CMAKE_MATCH_1} * 4")
check_path(actuator_return_pid actuator_task pwm_control_service service_stop position_pid_step)
check_path(actuator_return_calibration actuator_task pwm_control_service service_stop calibration_cache_position calibration_cache_get)
check_path(actuator_return_telemetry actuator_task pwm_control_service service_stop publish_stop)

# Optical acquisition uses small fixed bursts, never a full protocol context.
set(stack_bytes ${sensor_stack_bytes})
check_path(optical_burst sensor_task pmw3901_device_service pmw3901_spi_read exchange wait_flag)

set(stack_bytes ${host_stack_bytes})
check_path(host_optical_reply host_protocol_task process_host_rx handle_frame send_frame send_frame_sequence host_uart_write_bytes wait_tx_flag)

set(stack_bytes ${sensor_stack_bytes})
check_path(feedback_failure_capture sensor_task motor_device_read_feedback read_sync_block
    motor_uart_read_frame read_byte motor_uart_failure_note motor_dma_failure_snapshot)
set(stack_bytes ${host_stack_bytes})
check_path(host_uart_failure_reply host_protocol_task process_host_rx handle_frame
    uart_diagnostics_send_motor_buses send_diagnostic_frame send_frame send_frame_sequence
    host_uart_write_bytes wait_tx_flag)
