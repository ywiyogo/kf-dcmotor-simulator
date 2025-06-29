# UI library CMakeLists.txt
cmake_minimum_required(VERSION 3.28)

# Collect all source files
file(GLOB_RECURSE UI_SOURCES
    "*.cpp"
    "*.c"
)

file(GLOB_RECURSE UI_HEADERS
    "*.hpp"
    "*.h"
)

# Create the UI library
add_library(KalmanFilterUI STATIC
    ${UI_SOURCES}
    ${UI_HEADERS}
)

# Set C++23 standard
set_target_properties(KalmanFilterUI PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
)

# Include directories
target_include_directories(KalmanFilterUI PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

# Link with core library
target_link_libraries(KalmanFilterUI PUBLIC
    KalmanFilterCore
)

# Link with ImGui and ImPlot
target_link_libraries(KalmanFilterUI PUBLIC
    imgui
    imgui_sdl_backends
    implot
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
    target_link_libraries(KalmanFilterUI PUBLIC fmt::fmt)
else()
    # For native builds, try to find system fmt first
    find_package(fmt QUIET)
    if(fmt_FOUND)
        target_link_libraries(KalmanFilterUI PUBLIC fmt::fmt)
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
        target_link_libraries(KalmanFilterUI PUBLIC fmt::fmt)
    endif()
endif()

# SDL2 configuration
if(NOT (EMSCRIPTEN OR DEFINED __EMSCRIPTEN__))
    find_package(SDL2 REQUIRED)
    target_include_directories(KalmanFilterUI PUBLIC ${SDL2_INCLUDE_DIRS})
    target_link_libraries(KalmanFilterUI PUBLIC ${SDL2_LIBRARIES})
endif()

# Compiler-specific optimizations
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(KalmanFilterUI PRIVATE
        -Wall -Wextra -Wpedantic
        -O3 -march=native
    )
    if(NOT EMSCRIPTEN)
        target_compile_options(KalmanFilterUI PRIVATE -stdlib=libc++)
        target_link_options(KalmanFilterUI PRIVATE -stdlib=libc++)
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "NVHPC")
    target_compile_options(KalmanFilterUI PRIVATE
        -O3 -fast
        -gpu=cc70,cc75,cc80,cc86,cc89,cc90
    )
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(KalmanFilterUI PRIVATE
        -Wall -Wextra -Wpedantic
        -O3 -march=native
    )
endif()

# Platform-specific definitions
if(EMSCRIPTEN OR DEFINED __EMSCRIPTEN__)
    target_compile_definitions(KalmanFilterUI PUBLIC
        __EMSCRIPTEN__
        EMSCRIPTEN_BUILD
    )
endif()

# UI-specific compile definitions
target_compile_definitions(KalmanFilterUI PUBLIC
    IMGUI_ENABLE_FREETYPE
    IMPLOT_BACKEND_ENABLE_OPENGL3
    KF_UI_ENABLE_THEMES
)

# Debug/Release specific settings
target_compile_definitions(KalmanFilterUI PUBLIC
    $<$<CONFIG:Debug>:KF_DEBUG_MODE>
    $<$<CONFIG:Release>:KF_RELEASE_MODE>
    $<$<CONFIG:RelWithDebInfo>:KF_RELEASE_MODE>
)

# Export compile commands for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
