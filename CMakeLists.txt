cmake_minimum_required(VERSION 3.22)
project(drone_mapper LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Dependencies via FetchContent
# ---------------------------------------------------------------------------
include(FetchContent)

FetchContent_Declare(
    mp-units
    GIT_REPOSITORY https://github.com/mpusz/mp-units.git
    GIT_TAG        v2.5.0
    GIT_SHALLOW    TRUE
    SOURCE_SUBDIR  src
)

FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG        0.8.0
    GIT_SHALLOW    TRUE
)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.15.2
    GIT_SHALLOW    TRUE
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(MP_UNITS_API_CONTRACTS "NONE" CACHE STRING "")
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(mp-units yaml-cpp googletest)

# ---------------------------------------------------------------------------
# drone_mapper library (all implementation sources)
# ---------------------------------------------------------------------------
add_library(drone_mapper_lib STATIC
    src/DroneControlImpl.cpp
    src/ErrorHandler.cpp
    src/Map3DImpl.cpp
    src/MappingAlgorithmImpl.cpp
    src/MapsComparison.cpp
    src/MissionControlImpl.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/NpyArray.cpp
    src/ScanResultToVoxels.cpp
    src/SimulationManager.cpp
    src/SimulationRunFactoryImpl.cpp
    src/SimulationRunImpl.cpp
    src/YamlParser.cpp
)

target_compile_features(drone_mapper_lib PUBLIC cxx_std_20)
target_compile_options(drone_mapper_lib
    PRIVATE
    -Wall
    -Wextra
    -Werror
    -pedantic
)
target_include_directories(drone_mapper_lib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(drone_mapper_lib
    PUBLIC
    mp-units::mp-units
    yaml-cpp::yaml-cpp
)

# ---------------------------------------------------------------------------
# Main simulation executable (assignment submission entry point)
# ---------------------------------------------------------------------------
add_executable(drone_mapper src/drone_mapper_main.cpp)
target_compile_options(drone_mapper PRIVATE -Wall -Wextra -Werror -pedantic)
target_include_directories(drone_mapper PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(drone_mapper PRIVATE drone_mapper_lib)

# Legacy executable (kept for compatibility)
add_executable(drone_mapper_simulation src/drone_mapper_simulation_main.cpp)
target_compile_options(drone_mapper_simulation PRIVATE -Wall -Wextra -Werror -pedantic)
target_include_directories(drone_mapper_simulation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(drone_mapper_simulation PRIVATE drone_mapper_lib)

# ---------------------------------------------------------------------------
# Maps comparison utility
# ---------------------------------------------------------------------------
add_executable(maps_comparison src/maps_comparison_main.cpp)
target_compile_options(maps_comparison PRIVATE -Wall -Wextra -Werror -pedantic)
target_include_directories(maps_comparison PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(maps_comparison PRIVATE drone_mapper_lib)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
enable_testing()

add_executable(drone_mapper_simulation_test
    tests/components/test_DroneControl.cpp
    tests/components/test_MappingAlgorithm.cpp
    tests/components/test_MapsComparison.cpp
    tests/components/test_MissionControl.cpp
    tests/components/test_MockLidar.cpp
    tests/components/test_ScanResultToVoxels.cpp
    tests/components/test_SimulationManager.cpp
    tests/components/test_SimulationRun.cpp
    tests/integration/test_integration_real.cpp
    tests/integration/test_integration_mock_algo.cpp
)
target_compile_features(drone_mapper_simulation_test PRIVATE cxx_std_20)
target_compile_options(drone_mapper_simulation_test PRIVATE -Wall -Wextra -Werror -pedantic)
target_include_directories(drone_mapper_simulation_test
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(drone_mapper_simulation_test
    PRIVATE
    drone_mapper_lib
    GTest::gtest_main
    GTest::gmock_main
)

include(GoogleTest)
gtest_discover_tests(drone_mapper_simulation_test DISCOVERY_MODE PRE_TEST)
