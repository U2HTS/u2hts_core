add_library(u2hts_core STATIC 
    ${CMAKE_CURRENT_LIST_DIR}/u2hts_core.c
)

target_include_directories(u2hts_core PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(u2hts_core PRIVATE
    u2hts_core_settings)