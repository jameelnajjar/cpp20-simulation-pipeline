# MissionControl/CMakeLists.txt - builds the dynamically loaded mission-control plugin.
# The assignment requires the .so name MissionControl_<id1>_<id2>.so with no "lib" prefix.

set(MC_SO MissionControl_213309941_213727837)

# MODULE = dlopen-only .so, matching recitation 10. Not linked into the executable.
add_library(${MC_SO} MODULE
    src/MissionControlImpl.cpp
    src/DroneControlImpl.cpp
    src/ScanResultToVoxels.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/Logger.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/TimeUtils.cpp
)

set_target_properties(${MC_SO} PROPERTIES
    PREFIX ""
    OUTPUT_NAME ${MC_SO}
    CXX_VISIBILITY_PRESET default
)

target_include_directories(${MC_SO}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/common_mission_control/include
        ${USERCOMMON_INCLUDE}
)

target_link_libraries(${MC_SO} PRIVATE common::common Threads::Threads)
drone_warnings(${MC_SO})
