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

# Automated build type configuration
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# Automated compiler flags with option for customization
option(ENABLE_WARNINGS "Enable compiler warnings" OFF)
option(TREAT_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG -O3")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -ggdb")

if(ENABLE_WARNINGS)
    add_compile_options(-Wall -Wextra -pedantic)
    if(TREAT_WARNINGS_AS_ERRORS)
        add_compile_options(-Werror)
    endif()
endif()

# Automated verbosity and debug settings with environment variable fallback
set(VERBOSE "5" CACHE STRING "Verbose setting")
set(WARN "1" CACHE STRING "Warning message level")
set(INTCHECK "0" CACHE STRING "Integrity checks level")

# Automated CPLEX detection with more flexible paths
set(CPLEX_POSSIBLE_PATHS
    "/Applications/CPLEX_Studio2211"
    "/opt/ibm/ILOG/CPLEX_Studio2211"
    "/nfsd/rop/sw/ibm/cos221/cplex"
)

# Function to find CPLEX
function(find_cplex)
    foreach(CPLEX_PATH ${CPLEX_POSSIBLE_PATHS})
        if(EXISTS "${CPLEX_PATH}/cplex/lib")
            # Detect architecture and OS
            if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
                set(ARCH "arm64_osx")
            elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
                set(ARCH "x86-64_linux")
            elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
                set(ARCH "x64_windows")
            else()
                message(FATAL_ERROR "Unsupported operating system")
            endif()

            set(CPLEX_DIR "${CPLEX_PATH}/cplex/lib/${ARCH}/static_pic" CACHE PATH "CPLEX library directory")
            set(CPLEX_INCLUDE "${CPLEX_PATH}/cplex/include/ilcplex" CACHE PATH "CPLEX include directory")
            
            message(STATUS "Found CPLEX: ${CPLEX_PATH}")
            return()
        endif()
    endforeach()

    message(FATAL_ERROR "CPLEX not found. Please specify CPLEX_DIR and CPLEX_INCLUDE manually.")
endfunction()

# Call the CPLEX detection function
find_cplex()

# Validate CPLEX paths
if(NOT EXISTS ${CPLEX_DIR})
    message(FATAL_ERROR "CPLEX library directory not found: ${CPLEX_DIR}")
endif()
if(NOT EXISTS ${CPLEX_INCLUDE})
    message(FATAL_ERROR "CPLEX include directory not found: ${CPLEX_INCLUDE}")
endif()

# Automated logging directory setup
cmake_path(GET CMAKE_HOME_DIRECTORY PARENT_PATH HOME_DIR)
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

# Automated include directories
include_directories(
    .
    exact
    heuristics
    external
    ${CPLEX_INCLUDE}
)

# Macro to add libraries from source directories
macro(add_libraries_from_sources)
    set(SOURCE_DIRS "exact" "heuristics")
    foreach(SOURCE_DIR ${SOURCE_DIRS})
        file(GLOB SOURCES "${SOURCE_DIR}/*.cpp")
        foreach(SOURCE ${SOURCES})
            get_filename_component(LIBNAME ${SOURCE} NAME_WE)
            add_library(${LIBNAME} ${SOURCE} "${SOURCE_DIR}/${LIBNAME}.hpp")
        endforeach()
    endforeach()

    # Special case libraries
    file(GLOB SPECIAL_SOURCES "*.cpp")
    foreach(SOURCE ${SPECIAL_SOURCES})
        get_filename_component(LIBNAME ${SOURCE} NAME_WE)
        if(LIBNAME MATCHES "^(hplus_instance|algorithms)$")
            add_library(${LIBNAME} ${SOURCE} "${LIBNAME}.hpp")
        endif()
    endforeach()
endmacro()

# Generate libraries
add_libraries_from_sources()

# Find CPLEX library
find_library(CPLEX_LIBRARY
    NAMES cplex
    PATHS ${CPLEX_DIR}
    NO_DEFAULT_PATH
)

if(NOT CPLEX_LIBRARY)
    message(FATAL_ERROR "CPLEX library not found in ${CPLEX_DIR}")
endif()

# Explicitly define the libraries to link
set(HPLUS_LIBRARIES 
    hplus_instance
    imai
    rankooh
    rankooh_dynamic
    greedycost
    greedycxe
    hmax_hadd
)

# Main executable
add_executable(hplus main.cpp)

# Ensure these libraries are built before the main executable
add_dependencies(hplus ${HPLUS_LIBRARIES})

target_link_libraries(hplus 
    algorithms
    ${HPLUS_LIBRARIES}
    ${CPLEX_LIBRARY}
    Threads::Threads
    dl
)

# Automated definitions
add_definitions(
    -DHPLUS_HOME_DIR="${HOME_DIR}" 
    -DHPLUS_VERBOSE=${VERBOSE} 
    -DHPLUS_WARN=${WARN} 
    -DHPLUS_INTCHECK=${INTCHECK}
)

# Custom build targets with error handling
add_custom_target(opt 
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --config Release
    COMMAND_EXPAND_LISTS
)

add_custom_target(debug 
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --config Debug
    COMMAND_EXPAND_LISTS
)

# Cleanup target
add_custom_target(clear
    COMMAND rm -r ${CMAKE_HOME_DIRECTORY}/build/*
)