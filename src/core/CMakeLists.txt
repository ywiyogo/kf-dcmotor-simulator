# Core library CMakeLists.txt
cmake_minimum_required(VERSION 3.28)

# Collect all source files
file(GLOB_RECURSE CORE_SOURCES
    "*.cpp"
    "*.c"
)

file(GLOB_RECURSE CORE_HEADERS
    "*.hpp"
    "*.h"
)

# Create the core library
add_library(KalmanFilterCore STATIC
    ${CORE_SOURCES}
    ${CORE_HEADERS}
)

# Set C++23 standard
set_target_properties(KalmanFilterCore PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
)

# Include directories
target_include_directories(KalmanFilterCore PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

# Find and link dependencies
find_package(Threads REQUIRED)
target_link_libraries(KalmanFilterCore PUBLIC
    Threads::Threads
)

# Handle fmt dependency
if(EMSCRIPTEN OR DEFINED __EMSCRIPTEN__)
    # For web builds, get fmt through FetchContent
    include(FetchContent)
    FetchContent_Declare(
        fmt
        GIT_REPOSITORY https://github.com/fmtlib/fmt.git
        GIT_TAG 10.1.1
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(fmt)
    target_link_libraries(KalmanFilterCore PUBLIC fmt::fmt)
else()
    # For native builds, try to find system fmt first
    find_package(fmt QUIET)
    if(fmt_FOUND)
        target_link_libraries(KalmanFilterCore PUBLIC fmt::fmt)
    else()
        # Fallback to FetchContent
        include(FetchContent)
        FetchContent_Declare(
            fmt
            GIT_REPOSITORY https://github.com/fmtlib/fmt.git
            GIT_TAG 10.1.1
            GIT_SHALLOW TRUE
        )
        FetchContent_MakeAvailable(fmt)
        target_link_libraries(KalmanFilterCore PUBLIC fmt::fmt)
    endif()
endif()

# Enable parallel algorithms with TBB if available
find_package(TBB QUIET)
if(TBB_FOUND)
    message(STATUS "TBB found for core library, enabling parallel execution")
    target_compile_definitions(KalmanFilterCore PUBLIC HAS_TBB)
    target_link_libraries(KalmanFilterCore PUBLIC TBB::tbb)
endif()

# Compiler-specific optimizations
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(KalmanFilterCore PRIVATE
        -Wall -Wextra -Wpedantic
        -O3 -march=native
        -ffast-math
    )
    if(NOT EMSCRIPTEN)
        target_compile_options(KalmanFilterCore PRIVATE -stdlib=libc++)
        target_link_options(KalmanFilterCore PRIVATE -stdlib=libc++)
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "NVHPC")
    target_compile_options(KalmanFilterCore PRIVATE
        -O3 -fast
        -gpu=cc70,cc75,cc80,cc86,cc89,cc90
        -stdpar=gpu
    )
    target_compile_definitions(KalmanFilterCore PUBLIC HAS_NVHPC_GPU)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(KalmanFilterCore PRIVATE
        -Wall -Wextra -Wpedantic
        -O3 -march=native
        -ffast-math
    )
endif()

# Platform-specific definitions
if(EMSCRIPTEN OR DEFINED __EMSCRIPTEN__)
    target_compile_definitions(KalmanFilterCore PUBLIC
        __EMSCRIPTEN__
        EMSCRIPTEN_BUILD
    )
endif()

# Enable modern C++ features
target_compile_definitions(KalmanFilterCore PUBLIC
    KF_USE_STD_EXECUTION
    KF_ENABLE_SIMD
)

# Debug/Release specific settings
target_compile_definitions(KalmanFilterCore PUBLIC
    $<$<CONFIG:Debug>:KF_DEBUG_MODE>
    $<$<CONFIG:Release>:KF_RELEASE_MODE>
    $<$<CONFIG:RelWithDebInfo>:KF_RELEASE_MODE>
)

# Export compile commands for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
