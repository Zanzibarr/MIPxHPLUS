cmake_minimum_required(VERSION 3.20)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Enforce out-of-source build
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "Out-of-source build required. Create a 'build' directory and run cmake from there.")
endif()

project(hplus)

# Set default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# Compiler flags for release and debug builds
set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG -O3")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -ggdb")
set(COMPILEWARNS OFF CACHE BOOL "Flag to enable compile warnings")
if(COMPILEWARNS)
    add_compile_options(-Wall -Wextra -pedantic -Werror)
endif()

# Verbosity, warnings, and integrity checks
set(VERBOSE "5" CACHE STRING "Verbose setting set from command line")
message(STATUS "VERBOSE is set to ${VERBOSE}")
set(WARN "1" CACHE STRING "Flag to enable warning messages")
message(STATUS "WARN is set to ${WARN}")
set(INTCHECK "0" CACHE STRING "Flag to enable integrity checks")
message(STATUS "INTCHECK is set to ${INTCHECK}")

# Detect machine and set CPLEX paths accordingly
if(EXISTS "/Applications/CPLEX_Studio2211/cplex/lib/arm64_osx/static_pic")                  # Mac Laptop
    set(CPLEX_DIR "/Applications/CPLEX_Studio2211/cplex/lib/arm64_osx/static_pic")
    set(CPLEX_INCLUDE "/Applications/CPLEX_Studio2211/cplex/include/ilcplex")
elseif(EXISTS "/opt/ibm/ILOG/CPLEX_Studio2211/cplex/lib/x86-64_linux/static_pic")           # Linux PC
    set(CPLEX_DIR "/opt/ibm/ILOG/CPLEX_Studio2211/cplex/lib/x86-64_linux/static_pic")
    set(CPLEX_INCLUDE "/opt/ibm/ILOG/CPLEX_Studio2211/cplex/include/ilcplex")
elseif(EXISTS "/nfsd/rop/sw/ibm/cos221/cplex/lib/x86-64_linux/static_pic")                  # Cluster DEI
    set(CPLEX_DIR "/nfsd/rop/sw/ibm/cos221/cplex/lib/x86-64_linux/static_pic")
    set(CPLEX_INCLUDE "/nfsd/rop/sw/ibm/cos221/cplex/include/ilcplex")
else()
    message(FATAL_ERROR "CPLEX directory not found on this machine. Please set CPLEX_DIR manually.")
endif()

if(NOT EXISTS ${CPLEX_DIR})
    message(FATAL_ERROR "Directory ${CPLEX_DIR} doesn't exist... Please set with -DCPLEX_DIR=<path> the correct path to CPLEX's static pic folder.")
endif()
if(NOT EXISTS ${CPLEX_INCLUDE})
    message(FATAL_ERROR "Directory ${CPLEX_INCLUDE} doesn't exist... Please set with -DCPLEX_INCLUDE=<path> the correct path to CPLEX's headers folder.")
endif()

message(STATUS "Using CPLEX_DIR: ${CPLEX_DIR}")
message(STATUS "Using CPLEX_INCLUDE: ${CPLEX_INCLUDE}")

include_directories(${CPLEX_INCLUDE_DIR})

# Directory setup
cmake_path(GET CMAKE_HOME_DIRECTORY PARENT_PATH HOME_DIR)
set(LOGS_DIR "${HOME_DIR}/logs")
foreach(DIR ${LOGS_DIR} ${LOGS_DIR}/output_logs ${LOGS_DIR}/cpxout ${LOGS_DIR}/cpxout/log ${LOGS_DIR}/cpxout/lp)
    if(NOT EXISTS ${DIR})
        file(MAKE_DIRECTORY ${DIR})
        if(NOT EXISTS ${DIR})
            message(FATAL_ERROR "Failed to create directory ${DIR}")
        endif()
        message(STATUS "Created directory: ${DIR}")
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
    hplus
    src/main.cpp
)

target_link_libraries(
    hplus
    hplus_instance
    algorithms
    ${CPLEX_LIBRARY}
    Threads::Threads
    dl
)

# Definitions
add_definitions(-DHPLUS_HOME_DIR="${HOME_DIR}" -DHPLUS_VERBOSE=${VERBOSE} -DHPLUS_WARN=${WARN} -DHPLUS_INTCHECK=${INTCHECK})

# Custom build targets
add_custom_target(opt COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --config Release)
add_custom_target(debug COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --config Debug)

add_custom_target(clear
    COMMAND rm -r ${CMAKE_HOME_DIRECTORY}/build/*
)