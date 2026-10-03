# Root CMakeLists.txt - skeleton file, extended to find extra packages and to
# expose the UserCommon include tree to every sub-project. We do not add a
# makefile inside UserCommon itself (assignment forbids that).
cmake_minimum_required(VERSION 3.15)
project(assignment_3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(FetchContent)

# Course skeleton used vcpkg find_package(mp-units). We keep that, and fall back
# to FetchContent so a machine without VCPKG_ROOT can still build (HW2 pattern).
find_package(mp-units CONFIG QUIET)
if(NOT mp-units_FOUND)
    set(MP_UNITS_API_CONTRACTS "NONE" CACHE STRING "")
    FetchContent_Declare(mp-units
        GIT_REPOSITORY https://github.com/mpusz/mp-units.git
        GIT_TAG v2.5.0
        GIT_SHALLOW TRUE
        SOURCE_SUBDIR src)
    FetchContent_MakeAvailable(mp-units)
endif()

find_package(yaml-cpp CONFIG QUIET)
if(NOT yaml-cpp_FOUND)
    find_package(yaml-cpp QUIET)
endif()
if(NOT yaml-cpp_FOUND)
    set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG 0.8.0
        GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(yaml-cpp)
endif()
# FetchContent 0.8 exports `yaml-cpp`; vcpkg exports `yaml-cpp::yaml-cpp`.
if(TARGET yaml-cpp AND NOT TARGET yaml-cpp::yaml-cpp)
    add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()

find_package(Threads REQUIRED)

function(drone_warnings target)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Werror -pedantic)
endfunction()

# Shared include path for our UserCommon headers. Compiled .cpp files from
# UserCommon are listed explicitly by the project that needs them.
set(USERCOMMON_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/UserCommon/include")

add_subdirectory(common)
add_subdirectory(Algorithm)
add_subdirectory(MissionControl)
add_subdirectory(Simulator)

# Extra algorithms / MissionControls used only for local testing. Off by default
# so a staff/grader build of the five required folders is unchanged.
option(HW3_LOCAL_TESTS "Build extra local_tests plugins (not part of the submission .so names)" OFF)
if(HW3_LOCAL_TESTS)
    add_subdirectory(local_tests)
endif()
