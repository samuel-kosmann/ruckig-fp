cmake_minimum_required(VERSION 3.13)

# ---------------------------------------------------------------------------
# Project declaration
# ---------------------------------------------------------------------------
project(ruckig_fp
    VERSION 1.0.0
    DESCRIPTION "Fixed-point S-curve motion library for RP2040"
    LANGUAGES C
)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Optional: Raspberry Pi Pico SDK integration
#
# If the environment variable PICO_SDK_PATH is set and pico_sdk_import.cmake
# is available, include it so the library can be linked into Pico targets.
# Otherwise the library is built as a plain host-compatible static library.
# ---------------------------------------------------------------------------
if(DEFINED ENV{PICO_SDK_PATH} AND EXISTS "$ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake")
    include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
    pico_sdk_init()
    set(PICO_SDK_AVAILABLE TRUE)
else()
    set(PICO_SDK_AVAILABLE FALSE)
endif()

# ---------------------------------------------------------------------------
# Build options
# ---------------------------------------------------------------------------
option(BUILD_EXAMPLES
    "Build the example program (examples/main.c) that exercises the library"
    OFF
)

# ---------------------------------------------------------------------------
# ruckig_fp static library
# ---------------------------------------------------------------------------
add_library(ruckig_fp STATIC
    src/fixed_point.c
    src/profile.c
    src/ruckig_fp.c
    src/quantise.c
)

# Public include directory: consumers #include "ruckig_fp/..."
target_include_directories(ruckig_fp
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Strict compile options for the library itself
target_compile_options(ruckig_fp PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

# If building for Pico, link the Pico standard library
if(PICO_SDK_AVAILABLE)
    target_link_libraries(ruckig_fp PRIVATE pico_stdlib)
endif()

# ---------------------------------------------------------------------------
# Optional example executable
# ---------------------------------------------------------------------------
if(BUILD_EXAMPLES)
    add_executable(ruckig_fp_example
        examples/main.c
    )
    target_link_libraries(ruckig_fp_example PRIVATE ruckig_fp)

    if(PICO_SDK_AVAILABLE)
        # Generate UF2 and map files for Pico deployment
        pico_add_extra_outputs(ruckig_fp_example)
        target_link_libraries(ruckig_fp_example PRIVATE pico_stdlib)
    endif()
endif()

# ---------------------------------------------------------------------------
# Install rules (optional, for use as a CMake package)
# ---------------------------------------------------------------------------
install(TARGETS ruckig_fp
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ruckig_fp
    DESTINATION include
)
