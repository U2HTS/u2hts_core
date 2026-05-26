find_package(Git QUIET)
if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --tags --dirty --always
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        OUTPUT_VARIABLE U2HTS_GIT_SHA
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE GIT_RET
    )
endif()

if(NOT GIT_FOUND OR NOT GIT_RET EQUAL 0 OR U2HTS_GIT_SHA STREQUAL "")
    set(U2HTS_GIT_SHA "UNKNOWN")
endif()

add_library(u2hts_core_settings INTERFACE)

set(U2HTS_MAX_TPS 10 CACHE STRING "U2HTS max touch points")

target_compile_definitions(u2hts_core_settings INTERFACE 
    U2HTS_MAX_TPS=${U2HTS_MAX_TPS}
)

# override U2HTS_HID_REPORT_DESCRIPTOR
function(u2hts_generate_hid_report_descriptor)
foreach(i RANGE 1 ${U2HTS_MAX_TPS})
    list(APPEND U2HTS_TPS "U2HTS_HID_REPORT_TP")
endforeach()

list(JOIN U2HTS_TPS ", " U2HTS_TPS_STR)

set(U2HTS_HID_REPORT_DESCRIPTOR_DEF
    "U2HTS_HID_REPORT_DESCRIPTOR=U2HTS_HID_REPORT_DESCRIPTOR_START, \
    ${U2HTS_TPS_STR}, \
    U2HTS_HID_REPORT_TP_INFO, \
    U2HTS_HID_REPORT_TP_MAX_COUNT_DESC(U2HTS_HID_REPORT_TP_MAX_COUNT_ID), \
    U2HTS_HID_REPORT_TP_MS_THQA_CERT_DESC(U2HTS_HID_REPORT_TP_MS_THQA_CERT_ID), \
    U2HTS_HID_REPORT_DESCRIPTOR_END"
)

target_compile_definitions(u2hts_core_settings INTERFACE
    ${U2HTS_HID_REPORT_DESCRIPTOR_DEF}
)

endfunction(u2hts_generate_hid_report_descriptor)

add_library(u2hts_core STATIC 
    ${CMAKE_CURRENT_LIST_DIR}/u2hts_core.c
)

target_include_directories(u2hts_core PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(u2hts_core PUBLIC
    u2hts_core_settings
)

target_compile_definitions(u2hts_core PRIVATE
    U2HTS_GIT_SHA="${U2HTS_GIT_SHA}"
)