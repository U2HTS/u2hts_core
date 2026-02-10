find_package(Git QUIET)
if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE U2HTS_GIT_SHA
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
else()
    set(U2HTS_GIT_SHA "UNKNOWN")
endif()

add_library(u2hts_core STATIC 
    ${CMAKE_CURRENT_LIST_DIR}/u2hts_core.c
)

target_include_directories(u2hts_core PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(u2hts_core PUBLIC
    u2hts_core_settings)

target_compile_definitions(u2hts_core PRIVATE
    U2HTS_GIT_SHA="${U2HTS_GIT_SHA}"
)