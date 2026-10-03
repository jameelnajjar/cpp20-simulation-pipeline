# Algorithm/CMakeLists.txt - builds the dynamically loaded mapping-algorithm plugin.
# The assignment requires the .so name Algorithm_<id1>_<id2>.so with no "lib" prefix.

set(ALGO_SO Algorithm_213309941_213727837)

# MODULE = dlopen-only .so, matching recitation 10. Not linked into the executable.
add_library(${ALGO_SO} MODULE
    src/MappingAlgorithmImpl.cpp
)

set_target_properties(${ALGO_SO} PROPERTIES
    PREFIX ""
    OUTPUT_NAME ${ALGO_SO}
    CXX_VISIBILITY_PRESET default
)

target_include_directories(${ALGO_SO}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${USERCOMMON_INCLUDE}
)

target_link_libraries(${ALGO_SO} PRIVATE common::common)
drone_warnings(${ALGO_SO})
