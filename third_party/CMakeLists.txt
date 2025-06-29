include(FetchContent)

# Declare ImGui dependency
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG docking # Specify a version tag for stability
    GIT_SHALLOW TRUE # Only fetch the specified commit
)

# Declare ImPlot dependency
FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG master # Use master branch for ImGui docking compatibility
    GIT_SHALLOW TRUE
)

# Make dependencies available
FetchContent_MakeAvailable(imgui implot)

# Generate a custom header for SDL include
if(EMSCRIPTEN OR DEFINED __EMSCRIPTEN__)
    file(WRITE ${CMAKE_BINARY_DIR}/imgui_sdl_include.h
        "#include <SDL.h>\n"
        "#include \"imgui.h\"\n"
    )
    set(SDL_INCLUDE_PATH "SDL.h")
else()
    file(WRITE ${CMAKE_BINARY_DIR}/imgui_sdl_include.h
        "#include <SDL2/SDL.h>\n"
        "#include \"imgui.h\"\n"
    )
    set(SDL_INCLUDE_PATH "SDL2/SDL.h")
endif()

# Create wrapper for SDL2 backend
file(WRITE ${CMAKE_BINARY_DIR}/imgui_impl_sdl2_wrapper.h
    "#include <${SDL_INCLUDE_PATH}>\n"
    "#include \"imgui.h\"\n"
    "#include \"${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.h\"\n"
)

# Create wrapper for SDL2 Renderer backend
file(WRITE ${CMAKE_BINARY_DIR}/imgui_impl_sdlrenderer2_wrapper.h
    "#include <${SDL_INCLUDE_PATH}>\n"
    "#include \"imgui.h\"\n"
    "#include \"${imgui_SOURCE_DIR}/backends/imgui_impl_sdlrenderer2.h\"\n"
)

# Create ImGui library with our specific requirements
set(IMGUI_SOURCES
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
)

# Create a separate library for SDL backends
add_library(imgui_sdl_backends STATIC
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdlrenderer2.cpp
)

# Create ImGui main library
add_library(imgui STATIC ${IMGUI_SOURCES})

# Create ImPlot library
add_library(implot STATIC
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_demo.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
)

# Configure include directories
target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
)

target_include_directories(imgui_sdl_backends PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${CMAKE_BINARY_DIR} # For wrapper headers
)

target_include_directories(implot PUBLIC
    ${implot_SOURCE_DIR}
    ${imgui_SOURCE_DIR}
)

# Configure SDL2 for different platforms
if(EMSCRIPTEN OR DEFINED __EMSCRIPTEN__)
    # Emscripten provides SDL2 built-in
    target_compile_definitions(imgui_sdl_backends PUBLIC
        USE_SDL2_RENDERER
    )
    target_compile_definitions(imgui PUBLIC
        IMGUI_DISABLE_OBSOLETE_FUNCTIONS
        IMGUI_DISABLE_WIN32_DEFAULT_CLIPBOARD_FUNCTIONS
        IMGUI_DISABLE_WIN32_DEFAULT_IME_FUNCTIONS
    )
else()
    # Native builds need to find SDL2
    find_package(SDL2 REQUIRED)
    target_include_directories(imgui_sdl_backends PUBLIC ${SDL2_INCLUDE_DIRS})
    target_link_libraries(imgui_sdl_backends PUBLIC ${SDL2_LIBRARIES})
endif()

# Link libraries
target_link_libraries(imgui_sdl_backends PUBLIC imgui)
target_link_libraries(implot PUBLIC imgui)

# Set compile definitions
target_compile_definitions(imgui PUBLIC
    IMGUI_DISABLE_OBSOLETE_FUNCTIONS
)

target_compile_definitions(implot PUBLIC
    IMPLOT_DISABLE_OBSOLETE_FUNCTIONS
)

# Export targets for use by main project
set_target_properties(imgui PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

set_target_properties(imgui_sdl_backends PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

set_target_properties(implot PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)
