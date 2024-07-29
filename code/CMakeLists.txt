cmake_minimum_required(VERSION 3.20)
project(main)

set(CMAKE_CXX_STANDARD 11)

# setting flags for optimization and debug builds
set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG -O3 -flto")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -ggdb")
set(CPLEX_DIR "/Applications/CPLEX_Studio2211/cplex/lib/arm64_osx/static_pic" CACHE STRING "Path to the CPLEX library")
set(CPLEX_INCLUDE "/Applications/CPLEX_Studio2211/cplex/include/ilcplex" CACHE STRING "Path to the CPLEX headers")

# setting the default verbose/warn/intchecks (edited with the -D flag)
set(VERBOSE "10" CACHE STRING "Verbose setting set from command line")
message(STATUS "VERBOSE is set to ${VERBOSE}")
set(WARN "1" CACHE STRING "Flag to enable warning messages")
message(STATUS "WARN is set to ${WARN}")
set(INTCHECK "1" CACHE STRING "Flag to enable integrity checks")
message(STATUS "INTCHECK is set to ${INTCHECK}")

# check if all necessary folders exist
cmake_path(GET CMAKE_HOME_DIRECTORY PARENT_PATH HOME_DIR)
set(LOGS_DIR "${HOME_DIR}/code/logs")
set(INST_DIR "${HOME_DIR}/code/instances")
if(NOT EXISTS ${LOGS_DIR})
    file(MAKE_DIRECTORY ${LOGS_DIR})
    message(STATUS "Directory ${LOGS_DIR} created.")
endif()
if(NOT EXISTS ${INST_DIR})
    message(FATAL_ERROR "Directory ${INST_DIR} doesn't exist... Please create it and put the instances in there")
endif()

# build

include_directories(include)
include_directories(${CPLEX_INCLUDE})

add_library(
    Utils
    src/utils.cpp
    include/utils.hpp
)

add_library(
    HPLUS_instance
    src/hplus_instance.cpp
    include/hplus_instance.hpp
)

find_library(CPLEX_LIBRARY
    NAMES cplex
    PATHS ${CPLEX_DIR}
    NO_DEFAULT_PATH
)
if(NOT CPLEX_LIBRARY)
    message(FATAL_ERROR "CPLEX library not found in ${CPLEX_DIR}")
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_executable(
    main
    src/main.cpp
)

target_link_libraries(
    main
    Utils
    HPLUS_instance
    ${CPLEX_LIBRARY}
)

# adding definitions
add_definitions(-DHPLUS_VERBOSE=${VERBOSE} -DHPLUS_HOME_DIR="${HOME_DIR}" -DHPLUS_WARN=${WARN} -DHPLUS_INTCHECK=${INTCHECK})

# custom builds
add_custom_target(opt
    COMMAND echo "flags used: ${CMAKE_CXX_FLAGS_RELEASE}"
    COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Release ${CMAKE_SOURCE_DIR}
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target main
)

add_custom_target(debug
    COMMAND echo "flags used: ${CMAKE_CXX_FLAGS_DEBUG}"
    COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Debug ${CMAKE_SOURCE_DIR}
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target main
)

add_custom_target(clear
    COMMAND rm -r ${CMAKE_HOME_DIRECTORY}/build/*
)