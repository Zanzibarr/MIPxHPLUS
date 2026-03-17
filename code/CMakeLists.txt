cmake_minimum_required(VERSION 3.20)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_path(GET CMAKE_HOME_DIRECTORY PARENT_PATH HOME_DIR)
project(hplus)

# Enforce out-of-source build
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "Out-of-source build required. Create a 'build' directory and run cmake from there.")
endif()

# Build type configuration - default to Debug for development
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

# Build flags - with sanitizer support for debug builds
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -fsanitize=address -fno-omit-frame-pointer")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Compiler flags with option for customization
option(ENABLE_WARNINGS "Enable compiler warnings" OFF)
option(TREAT_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

if(ENABLE_WARNINGS)
    add_compile_options(-Wall -Wextra -pedantic)
    if(TREAT_WARNINGS_AS_ERRORS)
        add_compile_options(-Werror)
    endif()
endif()

if(NOT DEFINED CPLEX_PATH)
    # CPLEX detection
    set(CPLEX_POSSIBLE_PATHS
        "/Applications/CPLEX_Studio2211"
        "/opt/ibm/ILOG/CPLEX_Studio2211"
        "/nfsd/rop/sw/ibm/cos221"
    )
    foreach(PATH ${CPLEX_POSSIBLE_PATHS})
        if(EXISTS "${PATH}/cplex/lib")
            set(CPLEX_PATH ${PATH})
            break()
        endif()
    endforeach()
endif()

# Detect architecture and OS
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    set(ARCH "arm64_osx")
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
    set(ARCH "x86-64_linux")
else()
    message(FATAL_ERROR "Unsupported operating system")
endif()

set(CPLEX_DIR "${CPLEX_PATH}/cplex/lib/${ARCH}/static_pic" CACHE PATH "CPLEX library directory")
set(CPLEX_INCLUDE "${CPLEX_PATH}/cplex/include/ilcplex" CACHE PATH "CPLEX include directory")
message(STATUS "Found CPLEX: ${CPLEX_PATH}")

# Validate CPLEX paths
if(NOT EXISTS ${CPLEX_DIR})
    message(FATAL_ERROR "CPLEX library directory not found: ${CPLEX_DIR}")
endif()
if(NOT EXISTS ${CPLEX_INCLUDE})
    message(FATAL_ERROR "CPLEX include directory not found: ${CPLEX_INCLUDE}")
endif()

# Logging directory setup
set(LOGS_SUBDIRS 
    "logs"
    "logs/output_logs"
    "logs/cpxout"
    "logs/cpxout/log"
    "logs/cpxout/lp"
)
foreach(SUBDIR ${LOGS_SUBDIRS})
    set(FULL_DIR "${HOME_DIR}/${SUBDIR}")
    if(NOT EXISTS ${FULL_DIR})
        file(MAKE_DIRECTORY ${FULL_DIR})
        message(STATUS "Created directory: ${FULL_DIR}")
    endif()
endforeach()

# Threads and other dependencies
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Include directories
include_directories(
    external
    utils
    domain
    preprocessing
    landmarks
    search_heuristics
    cut_separators
    search_exact
    ${CPLEX_INCLUDE}
)

# Generate libraries
set(SOURCE_DIRS "domain" "search_heuristics" "cut_separators" "preprocessing" "landmarks" "search_exact")
foreach(SOURCE_DIR ${SOURCE_DIRS})
    file(GLOB SOURCES "${SOURCE_DIR}/*.cpp")
    if(SOURCES)
        add_library(${SOURCE_DIR}_lib ${SOURCES})
    endif()
endforeach()

# Find CPLEX library
find_library(CPLEX_LIBRARY
    NAMES cplex
    PATHS ${CPLEX_DIR}
    NO_DEFAULT_PATH
)
if(NOT CPLEX_LIBRARY)
    message(FATAL_ERROR "CPLEX library not found in ${CPLEX_DIR}")
endif()

# Create a dummy file to force recompilation
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/timestamp.h.tmp
    COMMAND ${CMAKE_COMMAND} -E echo "// Forcing recompilation" >> ${CMAKE_BINARY_DIR}/timestamp.h.tmp
    VERBATIM
)

add_custom_target(force_timestamp ALL
    DEPENDS ${CMAKE_BINARY_DIR}/timestamp.h.tmp
    COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_SOURCE_DIR}/main.cpp
)

# Executables
add_executable(hplus main.cpp)
add_dependencies(hplus force_timestamp)

if(UNIX AND NOT APPLE)
    # GNU ld does a single left-to-right pass over static archives, so
    # --start-group/--end-group forces multiple passes to resolve circular
    # or out-of-order symbol dependencies between the internal libraries.
    target_link_libraries(hplus
        -Wl,--start-group
        domain_lib
        preprocessing_lib
        landmarks_lib
        search_heuristics_lib
        cut_separators_lib
        search_exact_lib
        -Wl,--end-group
        ${CPLEX_LIBRARY}
        Threads::Threads
        dl
    )
else()
    target_link_libraries(hplus
        domain_lib
        preprocessing_lib
        landmarks_lib
        search_heuristics_lib
        cut_separators_lib
        search_exact_lib
        ${CPLEX_LIBRARY}
        Threads::Threads
        dl
    )
endif()

# Define flags and default values
add_definitions(
    -DHPLUS_HOME_DIR="${HOME_DIR}" 
)

# Print build configuration
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Debug flags: ${CMAKE_CXX_FLAGS_DEBUG}")
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "Release flags: ${CMAKE_CXX_FLAGS_RELEASE}")
endif()