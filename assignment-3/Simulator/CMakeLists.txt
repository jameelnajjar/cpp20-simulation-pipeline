# Simulator/CMakeLists.txt - builds simulator_<ids>. ENABLE_EXPORTS publishes the
# MappingAlgorithmRegistration / MissionControlRegistration constructors so that
# dlopen'ed plugins can find them (recitation 10 pattern).

set(SIM_EXE simulator_213309941_213727837)

add_executable(${SIM_EXE}
    src/main.cpp
    src/CommandLine.cpp
    src/Registrar.cpp
    src/MappingAlgorithmRegistration.cpp
    src/MissionControlRegistration.cpp
    src/NpyArray.cpp
    src/Map3DImpl.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/MapsComparison.cpp
    src/YamlParser.cpp
    src/ReportWriter.cpp
    src/SimulationRunImpl.cpp
    src/SimulationRunFactoryImpl.cpp
    src/SimulationManager.cpp
    src/Simulator.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/Logger.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/TimeUtils.cpp
)

set_target_properties(${SIM_EXE} PROPERTIES
    ENABLE_EXPORTS ON
    OUTPUT_NAME ${SIM_EXE}
)

target_include_directories(${SIM_EXE}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
        ${USERCOMMON_INCLUDE}
)

target_link_libraries(${SIM_EXE}
    PRIVATE
        common::common
        yaml-cpp::yaml-cpp
        Threads::Threads
        ${CMAKE_DL_LIBS}
)

drone_warnings(${SIM_EXE})

add_dependencies(${SIM_EXE}
    Algorithm_213309941_213727837
    MissionControl_213309941_213727837
)
