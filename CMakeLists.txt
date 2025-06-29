cmake_minimum_required(VERSION 3.28)
project(KalmanFilterSimulation
    VERSION 1.0.0
    DESCRIPTION "Modern C++23 Kalman Filter Simulation for DC Motor"
    LANGUAGES CXX
)

# Set C++23 standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable export of compile commands for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build type configuration
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Platform detection
if(EMSCRIPTEN OR DEFINED __EMSCRIPTEN__)
    set(PLATFORM_EMSCRIPTEN ON)
    message(STATUS "Building for Emscripten/WebAssembly")
else()
    set(PLATFORM_EMSCRIPTEN OFF)
    message(STATUS "Building for native platform")
endif()

# Compiler-specific global settings
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    if(NOT PLATFORM_EMSCRIPTEN)
        add_compile_options(-stdlib=libc++)
        add_link_options(-stdlib=libc++)
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "NVHPC")
    add_compile_options(-O3 -fast -gpu=cc70,cc75,cc80,cc86,cc89,cc90)
    add_compile_definitions(HAS_NVHPC_GPU)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Emscripten specific settings
if(PLATFORM_EMSCRIPTEN)
    set(EMSCRIPTEN_FLAGS
        "-s USE_SDL=2"
        "-s WASM=1"
        "-s ALLOW_MEMORY_GROWTH=1"
        "-s DISABLE_EXCEPTION_CATCHING=0"
        "-s NO_EXIT_RUNTIME=0"
        "-s EXPORTED_RUNTIME_METHODS=['ccall']"
    )

    string(JOIN " " EMSCRIPTEN_FLAGS_STR ${EMSCRIPTEN_FLAGS})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EMSCRIPTEN_FLAGS_STR}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EMSCRIPTEN_FLAGS_STR}")

    add_compile_definitions(__EMSCRIPTEN__ EMSCRIPTEN_BUILD)
endif()

# Find system dependencies
find_package(Threads REQUIRED)

# SDL2 configuration
if(NOT PLATFORM_EMSCRIPTEN)
    find_package(SDL2 REQUIRED)
    if(SDL2_FOUND)
        message(STATUS "Found SDL2: ${SDL2_LIBRARIES}")
    endif()
endif()

# Enable parallel algorithms with TBB if available
find_package(TBB QUIET)
if(TBB_FOUND)
    message(STATUS "TBB found, enabling parallel execution")
    add_compile_definitions(HAS_TBB)
endif()

# Add external dependencies (ImGui, ImPlot)
add_subdirectory(third_party)

# Add core and UI libraries
add_subdirectory(src/core)
add_subdirectory(src/ui)

# Create main executable
add_executable(${PROJECT_NAME}
    src/main.cpp
)

# Link libraries to main executable
target_link_libraries(${PROJECT_NAME} PRIVATE
    KalmanFilterCore
    KalmanFilterUI
    imgui
    imgui_sdl_backends
    implot
    Threads::Threads
)

# Platform-specific linking
if(NOT PLATFORM_EMSCRIPTEN)
    if(TARGET SDL2::SDL2)
        target_link_libraries(${PROJECT_NAME} PRIVATE SDL2::SDL2)
    else()
        target_link_libraries(${PROJECT_NAME} PRIVATE ${SDL2_LIBRARIES})
        target_include_directories(${PROJECT_NAME} PRIVATE ${SDL2_INCLUDE_DIRS})
    endif()
endif()

# Link TBB if available
if(TBB_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE TBB::tbb)
endif()

# Set target properties
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Debug/Release configurations
target_compile_definitions(${PROJECT_NAME} PRIVATE
    $<$<CONFIG:Debug>:KF_DEBUG_MODE>
    $<$<CONFIG:Release>:KF_RELEASE_MODE>
    $<$<CONFIG:RelWithDebInfo>:KF_RELEASE_MODE>
)

# Emscripten web build specific settings
if(PLATFORM_EMSCRIPTEN)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        SUFFIX ".html"
        LINK_FLAGS "${EMSCRIPTEN_FLAGS_STR} --preload-file ${CMAKE_SOURCE_DIR}/resources@/resources"
    )

    # Create resources directory if it doesn't exist
    file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/resources)
endif()

# Copy resources to build directory for native builds
if(NOT PLATFORM_EMSCRIPTEN)
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/resources)
    if(EXISTS ${CMAKE_SOURCE_DIR}/resources)
        file(COPY ${CMAKE_SOURCE_DIR}/resources/
             DESTINATION ${CMAKE_BINARY_DIR}/resources)
    endif()
endif()

# Installation rules
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)

if(EXISTS ${CMAKE_SOURCE_DIR}/resources)
    install(DIRECTORY resources/
        DESTINATION share/${PROJECT_NAME}/resources
    )
endif()

# Development tools
find_program(CLANG_FORMAT_EXE NAMES clang-format)
if(CLANG_FORMAT_EXE)
    file(GLOB_RECURSE ALL_SOURCE_FILES
        "${CMAKE_SOURCE_DIR}/src/*.cpp"
        "${CMAKE_SOURCE_DIR}/src/*.hpp"
        "${CMAKE_SOURCE_DIR}/src/*.h"
        "${CMAKE_SOURCE_DIR}/src/*.c"
    )

    add_custom_target(format
        COMMAND ${CLANG_FORMAT_EXE} -i -style=file ${ALL_SOURCE_FILES}
        COMMENT "Running clang-format on source files"
        VERBATIM
    )
endif()

# Add a target to run the application
if(NOT PLATFORM_EMSCRIPTEN)
    add_custom_target(run
        COMMAND ${CMAKE_BINARY_DIR}/bin/${PROJECT_NAME}
        DEPENDS ${PROJECT_NAME}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Running ${PROJECT_NAME}"
        VERBATIM
    )
endif()

# Print build configuration summary
message(STATUS "=== Build Configuration Summary ===")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Platform: ${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_VERSION}")
if(PLATFORM_EMSCRIPTEN)
    message(STATUS "Emscripten: ON")
else()
    message(STATUS "Emscripten: OFF")
    message(STATUS "SDL2: ${SDL2_FOUND}")
endif()
message(STATUS "TBB: ${TBB_FOUND}")
message(STATUS "====================================")
