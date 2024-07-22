cmake_minimum_required(VERSION 3.20)
project(main)

set(CMAKE_CXX_STANDARD 11)

# setting flags for optimization and debug builds
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-g")

# setting the default verbose (edited with the -D flag)
set(VERBOSE "10" CACHE STRING "Verbose setting set from command line")
message(STATUS "VERBOSE is set to ${VERBOSE}")

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

add_executable(
    main
    src/main.cpp
)

target_link_libraries(
    main
    Utils
    HPLUS_instance
)

# define HPLUS_VERBOSE and HPLUS_HOME_DIR
add_definitions(-DHPLUS_VERBOSE=${VERBOSE} -DHPLUS_HOME_DIR="${HOME_DIR}")

# custom builds
add_custom_target(opt
    COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Release ${CMAKE_SOURCE_DIR}
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target main
)

add_custom_target(debug
    COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Debug ${CMAKE_SOURCE_DIR}
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target main
)

add_custom_target(clear
    COMMAND rm -r ${CMAKE_HOME_DIRECTORY}/build/*
)