cmake_minimum_required(VERSION 3.20)

project(hplus
    VERSION 4.2.2
    LANGUAGES CXX
)

# ── Language standard ────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Global CMake settings ────────────────────────────────────────────────────
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── Paths ────────────────────────────────────────────────────────────────────
# CMakeLists.txt lives in <repo>/code/, so the repo root is one level up.
cmake_path(GET CMAKE_SOURCE_DIR PARENT_PATH REPO_ROOT)

# ── Enforce out-of-source build ──────────────────────────────────────────────
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR
        "In-source builds are not allowed.\n"
        "Run cmake from the build directory:\n"
        "  cd build && cmake ..")
endif()

# ── Build type ───────────────────────────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# ── Options ──────────────────────────────────────────────────────────────────
option(ENABLE_SANITIZERS "Enable AddressSanitizer and UBSan (Debug only)" ON)
option(ENABLE_ASSERTS "Enable Asserts even in Release builds" OFF)

# ── Asserts in Release (strip -DNDEBUG so assert/myassert stay active) ───────
if(ENABLE_ASSERTS)
    foreach(_flags IN ITEMS CMAKE_CXX_FLAGS_RELEASE CMAKE_CXX_FLAGS_RELWITHDEBINFO CMAKE_CXX_FLAGS_MINSIZEREL)
        string(REPLACE "-DNDEBUG" "" ${_flags} "${${_flags}}")
    endforeach()
    message(STATUS "Asserts enabled in all build types")
endif()

# ── Compiler warnings (interface library so targets opt in) ──────────────────
add_library(project_warnings INTERFACE)
target_compile_options(project_warnings INTERFACE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:
        -Wall -Wextra -Wpedantic
        -Wuninitialized -Wstrict-overflow=2
        -Wshadow -Wnon-virtual-dtor -Woverloaded-virtual
        -Wnull-dereference -Wcast-align -Wcast-qual
        -Wdouble-promotion -Wformat=2 -Wunused
    >
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive->
)

# ── Sanitizers (Debug only, when ENABLE_SANITIZERS=ON) ───────────────────────
add_library(project_sanitizers INTERFACE)
if(ENABLE_SANITIZERS)
    target_compile_options(project_sanitizers INTERFACE
        $<$<AND:$<CONFIG:Debug>,$<CXX_COMPILER_ID:GNU,Clang,AppleClang>>:
            -fsanitize=address,undefined
            -fno-omit-frame-pointer
            -fno-sanitize-recover=all
        >
    )
    target_link_options(project_sanitizers INTERFACE
        $<$<AND:$<CONFIG:Debug>,$<CXX_COMPILER_ID:GNU,Clang,AppleClang>>:
            -fsanitize=address,undefined
        >
    )
endif()

# ── Dependencies ─────────────────────────────────────────────────────────────
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# ── CPLEX ────────────────────────────────────────────────────────────────────
# Override auto-detection with -DCPLEX_PATH=/path/to/CPLEX_Studio
if(NOT DEFINED CPLEX_PATH)
    set(_cplex_candidates
        "/Applications/CPLEX_Studio2211"
        "/opt/ibm/ILOG/CPLEX_Studio2211"
        "/nfsd/rop/sw/ibm/cos221"
    )
    foreach(_candidate IN LISTS _cplex_candidates)
        if(EXISTS "${_candidate}/cplex/lib")
            set(CPLEX_PATH "${_candidate}")
            break()
        endif()
    endforeach()
endif()
if(NOT DEFINED CPLEX_PATH)
    message(FATAL_ERROR "CPLEX not found in any known location; pass -DCPLEX_PATH=/path/to/CPLEX_Studio")
endif()
message(STATUS "Found CPLEX: ${CPLEX_PATH}")

if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    set(CPLEX_ARCH "arm64_osx")
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
    set(CPLEX_ARCH "x86-64_linux")
else()
    message(FATAL_ERROR "Unsupported operating system for CPLEX")
endif()

set(CPLEX_DIR "${CPLEX_PATH}/cplex/lib/${CPLEX_ARCH}/static_pic" CACHE PATH "CPLEX library directory")
set(CPLEX_INCLUDE "${CPLEX_PATH}/cplex/include/ilcplex" CACHE PATH "CPLEX include directory")
if(NOT EXISTS ${CPLEX_DIR})
    message(FATAL_ERROR "CPLEX library directory not found: ${CPLEX_DIR}")
endif()
if(NOT EXISTS ${CPLEX_INCLUDE})
    message(FATAL_ERROR "CPLEX include directory not found: ${CPLEX_INCLUDE}")
endif()

find_library(CPLEX_LIBRARY
    NAMES cplex
    PATHS ${CPLEX_DIR}
    NO_DEFAULT_PATH
)
if(NOT CPLEX_LIBRARY)
    message(FATAL_ERROR "CPLEX library not found in ${CPLEX_DIR}")
endif()

# Interface target so modules and the executable opt in with one link line.
# SYSTEM include suppresses warnings from CPLEX headers (#include <cplex.h>).
add_library(cplex INTERFACE)
target_include_directories(cplex SYSTEM INTERFACE ${CPLEX_INCLUDE})
target_link_libraries(cplex INTERFACE ${CPLEX_LIBRARY} Threads::Threads ${CMAKE_DL_LIBS})

# ── bliss (graph automorphism/symmetry detection) ───────────────────────────
# Override auto-detection with -DBLISS_PATH=/path/to/bliss-<version>
if(NOT DEFINED BLISS_PATH)
    set(BLISS_PATH "/opt/sw/bliss-0.77")
endif()
if(NOT EXISTS "${BLISS_PATH}/src/digraph.hh")
    message(FATAL_ERROR "bliss headers not found in ${BLISS_PATH}/src; pass -DBLISS_PATH=/path/to/bliss")
endif()
find_library(BLISS_LIBRARY
    NAMES bliss_static bliss
    PATHS ${BLISS_PATH}
    NO_DEFAULT_PATH
)
if(NOT BLISS_LIBRARY)
    message(FATAL_ERROR "bliss library not found in ${BLISS_PATH}")
endif()
message(STATUS "Found bliss: ${BLISS_PATH}")

# SYSTEM include suppresses warnings from bliss headers (#include <digraph.hh>).
add_library(bliss INTERFACE)
target_include_directories(bliss SYSTEM INTERFACE "${BLISS_PATH}/src")
target_link_libraries(bliss INTERFACE ${BLISS_LIBRARY})

# Example: fetch an external library with FetchContent
# include(FetchContent)
# FetchContent_Declare(<lib> GIT_REPOSITORY https://github.com/... GIT_TAG v1.0.0)
# FetchContent_MakeAvailable(<lib>)

# Example: find an installed system package
# find_package(<PackageName> REQUIRED)

# ── Auto-discover modules ─────────────────────────────────────────────────────
# Every direct subdirectory whose src/ folder contains at least one .cpp file
# is compiled into a static library and linked to the executable automatically.
# CONFIGURE_DEPENDS → cmake re-configures when source files are added/removed.
file(GLOB _top_dirs CONFIGURE_DEPENDS LIST_DIRECTORIES true "${CMAKE_SOURCE_DIR}/*")
set(AUTO_MODULE_LIBS "")
foreach(_dir IN LISTS _top_dirs)
    cmake_path(GET _dir FILENAME _dname)
    file(GLOB_RECURSE _srcs CONFIGURE_DEPENDS "${_dir}/src/*.cpp")
    if(_srcs)
        add_library(${_dname}_lib STATIC ${_srcs})
        target_include_directories(${_dname}_lib PUBLIC "${_dir}/include")
        target_link_libraries(${_dname}_lib PUBLIC cplex PRIVATE project_warnings project_sanitizers)
        list(APPEND AUTO_MODULE_LIBS ${_dname}_lib)
    endif()
endforeach()

# ── Version header ───────────────────────────────────────────────────────────
configure_file(utils/include/version.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/utils/include/version.hpp @ONLY)

# ── Executable ───────────────────────────────────────────────────────────────
file(GLOB_RECURSE _src_srcs CONFIGURE_DEPENDS "src/*.cpp")
add_executable(hplus ${_src_srcs})

# project-wide headers + every module's include/ dir (auto-collected above)
file(GLOB _module_incs CONFIGURE_DEPENDS LIST_DIRECTORIES true "${CMAKE_SOURCE_DIR}/*/include")
target_include_directories(hplus PRIVATE include ${_module_incs} ${CMAKE_CURRENT_BINARY_DIR}/utils/include)
# Suppress warnings from third-party headers in external/
target_include_directories(hplus SYSTEM PRIVATE external)

# Expose the repo root as a compile-time string so code can build absolute
# paths (e.g. for log files) without depending on the working directory.
target_compile_definitions(hplus PRIVATE REPO_ROOT="${REPO_ROOT}")

# ── Build timestamp ───────────────────────────────────────────────────────────
# Touch src/main.cpp before every build so the compiler's __DATE__ / __TIME__
# macros always reflect the actual compilation time.
add_custom_target(gen_build_timestamp ALL
    COMMAND ${CMAKE_COMMAND} -E touch "${CMAKE_SOURCE_DIR}/src/main.cpp"
    COMMENT "Refreshing build timestamp"
    VERBATIM
)
add_dependencies(hplus gen_build_timestamp)

target_link_libraries(hplus PRIVATE
    project_warnings
    project_sanitizers
    cplex
    bliss
    ${AUTO_MODULE_LIBS}
    # <PackageName>::<PackageName>
)

# On Linux, GNU ld resolves static archive symbols in a single left-to-right
# pass; --start-group/--end-group forces multiple passes for circular deps.
if(UNIX AND NOT APPLE)
    target_link_options(hplus PRIVATE
        -Wl,--start-group -Wl,--end-group
    )
endif()
