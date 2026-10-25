cmake_minimum_required(VERSION 3.19)
project(yertle_sequence_audit_test LANGUAGES C)
enable_testing()
add_executable(sequence_audit_test audit_test.c pwm_control.c
    ../../firmware/src/position_pid.c ../../firmware/src/mode_store.c
    ../../tests/pwm/mode_memory.c)
target_include_directories(sequence_audit_test PRIVATE ../../tests/pwm ../../firmware/include)
if(MSVC)
  target_compile_options(sequence_audit_test PRIVATE /W4 /UNDEBUG /utf-8)
else()
  target_compile_options(sequence_audit_test PRIVATE -Wall -Wextra -UNDEBUG)
  target_link_libraries(sequence_audit_test PRIVATE m)
endif()
add_test(NAME zero_pwm_off_sequence_hypotheses COMMAND sequence_audit_test)
