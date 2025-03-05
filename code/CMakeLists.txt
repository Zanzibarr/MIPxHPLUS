cmake_minimum_required(VERSION 3.20)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Enforce out-of-source build
if ("${CMAKE_CURRENT_SOURCE_DIR}" STREQUAL "${CMAKE_CURRENT_BINARY_DIR}")
    message(FATAL_ERROR "Please build the project from a build folder.\nCreate a separate build directory and run CMake from there:\n  mkdir build\n  cd build\n  cmake <build_options> ..\n\nYou may delete the CMakeCache.txt file and the CMakeFiles folder that has been created.\n" )
endif()

project(main)

# Set default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# Compiler flags for release and debug builds
set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG -Ofast")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -ggdb")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -Werror")

# Verbosity, warnings, and integrity checks
set(VERBOSE "5" CACHE STRING "Verbose setting set from command line")
message(STATUS "VERBOSE is set to ${VERBOSE}")
set(WARN "1" CACHE STRING "Flag to enable warning messages")
message(STATUS "WARN is set to ${WARN}")
set(INTCHECK "0" CACHE STRING "Flag to enable integrity checks")
message(STATUS "INTCHECK is set to ${INTCHECK}")

# CPLEX directories
set(CPLEX_DIR "/Applications/CPLEX_Studio2211/cplex/lib/arm64_osx/static_pic" CACHE STRING "Path to the CPLEX library")
set(CPLEX_INCLUDE "/Applications/CPLEX_Studio2211/cplex/include/ilcplex" CACHE STRING "Path to the CPLEX headers")

if(NOT EXISTS ${CPLEX_DIR})
    message(FATAL_ERROR "Directory ${CPLEX_DIR} doesn't exist... Please set with -DCPLEX_DIR=<path> the correct path to CPLEX's static pic folder.")
endif()
if(NOT EXISTS ${CPLEX_INCLUDE})
    message(FATAL_ERROR "Directory ${CPLEX_INCLUDE} doesn't exist... Please set with -DCPLEX_INCLUDE=<path> the correct path to CPLEX's headers folder.")
endif()

# Directory setup
cmake_path(GET CMAKE_HOME_DIRECTORY PARENT_PATH HOME_DIR)
set(LOGS_DIR "${HOME_DIR}/logs")
set(CPXOUT_DIR "${LOGS_DIR}/cpxout")
set(CPXLOG_DIR "${CPXOUT_DIR}/log")
set(CPXLP_DIR "${CPXOUT_DIR}/lp")

foreach(DIR ${LOGS_DIR} ${CPXOUT_DIR} ${CPXLOG_DIR} ${CPXLP_DIR} "${LOGS_DIR}/AAA_output_logs")
    if(NOT EXISTS ${DIR})
        file(MAKE_DIRECTORY ${DIR})
        message(STATUS "Directory ${DIR} created.")
    endif()
endforeach()

# Threads support
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Include directories
include_directories(include)
include_directories(external)
include_directories(${CPLEX_INCLUDE})

# Libraries

add_library(
    hplus_instance
    src/hplus_instance.cpp
    include/hplus_instance.hpp
)

add_library(
    algorithms
    src/algorithms.cpp
    include/algorithms.hpp
)

find_library(CPLEX_LIBRARY
    NAMES cplex
    PATHS ${CPLEX_DIR}
    NO_DEFAULT_PATH
)

if(NOT CPLEX_LIBRARY)
    message(FATAL_ERROR "CPLEX library not found in ${CPLEX_DIR}")
endif()

# Main executable
add_executable(
    main
    src/main.cpp
)

target_link_libraries(
    main
    hplus_instance
    algorithms
    ${CPLEX_LIBRARY}
    Threads::Threads
    dl
)

# Definitions
add_definitions(-DHPLUS_HOME_DIR="${HOME_DIR}" -DHPLUS_VERBOSE=${VERBOSE} -DHPLUS_WARN=${WARN} -DHPLUS_INTCHECK=${INTCHECK})

# Custom build targets
add_custom_target(opt
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --config Release
)

add_custom_target(debug
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --config Debug
)

add_custom_target(clear
    COMMAND rm -r ${CMAKE_HOME_DIRECTORY}/build/*
)

