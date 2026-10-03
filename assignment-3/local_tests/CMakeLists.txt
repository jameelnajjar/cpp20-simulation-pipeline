# Local-only extra plugins. Not the official Algorithm_*/MissionControl_* names.
# Built only when HW3_LOCAL_TESTS=ON.

set(TEST_PLUGIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/plugins")
set(MC_SRC_DIR "${CMAKE_SOURCE_DIR}/MissionControl")

function(hw3_test_algo name)
    add_library(${name} MODULE ${TEST_PLUGIN_DIR}/${name}.cpp)
    set_target_properties(${name} PROPERTIES
        PREFIX ""
        OUTPUT_NAME ${name}
        CXX_VISIBILITY_PRESET default
    )
    target_include_directories(${name} PRIVATE ${TEST_PLUGIN_DIR} ${USERCOMMON_INCLUDE})
    target_link_libraries(${name} PRIVATE common::common)
    drone_warnings(${name})
endfunction()

function(hw3_test_mc name)
    add_library(${name} MODULE ${TEST_PLUGIN_DIR}/${name}.cpp)
    set_target_properties(${name} PROPERTIES
        PREFIX ""
        OUTPUT_NAME ${name}
        CXX_VISIBILITY_PRESET default
    )
    target_include_directories(${name} PRIVATE ${TEST_PLUGIN_DIR} ${USERCOMMON_INCLUDE})
    target_link_libraries(${name} PRIVATE common::common)
    drone_warnings(${name})
endfunction()

hw3_test_algo(FinishNowAlgo)
hw3_test_algo(NoopAlgo)
hw3_test_algo(InvalidAlgo)
hw3_test_algo(OversizeAlgo)
hw3_test_algo(OobAlgo)
hw3_test_algo(WallAlgo)
hw3_test_algo(HoverScanAlgo)
hw3_test_algo(NeverFinishAlgo)
hw3_test_algo(RotateOnlyAlgo)
hw3_test_algo(ElevateFarAlgo)
hw3_test_algo(HoverMoveAlgo)

hw3_test_mc(AbortMissionControl)
hw3_test_mc(ThrowingMissionControl)

add_library(SilentPlugin MODULE ${TEST_PLUGIN_DIR}/SilentPlugin.cpp)
set_target_properties(SilentPlugin PROPERTIES
    PREFIX ""
    OUTPUT_NAME SilentPlugin
    CXX_VISIBILITY_PRESET default
    LINKER_LANGUAGE CXX
)
drone_warnings(SilentPlugin)

add_library(TwinMissionControl MODULE
    ${TEST_PLUGIN_DIR}/TwinMissionControl.cpp
    ${MC_SRC_DIR}/src/DroneControlImpl.cpp
    ${MC_SRC_DIR}/src/ScanResultToVoxels.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/Logger.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/TimeUtils.cpp
)
set_target_properties(TwinMissionControl PROPERTIES
    PREFIX ""
    OUTPUT_NAME TwinMissionControl
    CXX_VISIBILITY_PRESET default
)
target_include_directories(TwinMissionControl
    PRIVATE
        ${TEST_PLUGIN_DIR}
        ${MC_SRC_DIR}/include
        ${MC_SRC_DIR}/common_mission_control/include
        ${USERCOMMON_INCLUDE}
)
target_link_libraries(TwinMissionControl PRIVATE common::common Threads::Threads)
drone_warnings(TwinMissionControl)
