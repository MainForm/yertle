# Test-only overlay; never modifies the production source/standard BIN.
function(yertle_use_sequence_audit)
    get_target_property(sources yertle_f103rb SOURCES)
    list(FILTER sources EXCLUDE REGEX "(^|/)pwm_control\\.c$")
    set_property(TARGET yertle_f103rb PROPERTY SOURCES "${sources}")
    target_sources(yertle_f103rb PRIVATE "${YERTLE_SEQUENCE_AUDIT_DIR}/pwm_control.c")
endfunction()
cmake_language(DEFER DIRECTORY "${CMAKE_SOURCE_DIR}" CALL yertle_use_sequence_audit)
