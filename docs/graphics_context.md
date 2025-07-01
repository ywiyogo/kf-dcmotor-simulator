# GraphicsContext - Clean SDL Abstraction Layer

## Overview

The `GraphicsContext` class provides a clean, modern C++ abstraction over SDL2 for window and renderer management. It encapsulates all SDL implementation details, providing a simple, RAII-compliant interface that hides complexity from your application code.

## Key Benefits

### 🎯 **Clean Separation of Concerns**
- **No SDL code in main application** - All SDL details are encapsulated
- **Clear ownership model** - Graphics context owns all SDL resources
- **RAII compliance** - Automatic resource cleanup

### 🔧 **Easy to Use**
- **Simple initialization** - One function call to set up graphics
- **Event callbacks** - Clean callback system for events
- **Automatic timing** - Built-in frame timing and FPS calculation

### 🚀 **Modern C++ Design**
- **Exception safe** - Proper error handling throughout
- **Move semantics** - Efficient resource transfer
- **Type safety** - Strong typing prevents common errors

### 🧪 **Testable Architecture**
- **Dependency injection ready** - Easy to mock for testing
- **Clear interfaces** - Well-defined boundaries
- **Modular design** - Components can be tested independently

## Quick Start

### Basic Usage

```cpp
#include "ui/GraphicsContext.hpp"

int main() {
    // Configure graphics context - no SDL code needed!
    kf::ui::GraphicsContext::Config config;
    config.width = 1280;
    config.height = 720;
    config.title = "My Application";
    config.vsync = true;

    // Create graphics context
    auto graphics = std::make_unique<kf::ui::GraphicsContext>(config);

    // Initialize (handles all SDL setup internally)
    if (!graphics->initialize()) {
        std::cerr << "Failed to initialize graphics\n";
        return -1;
    }

    // Main loop
    while (!graphics->should_close()) {
        // Process events
        if (!graphics->process_events()) break;

        // Clear screen
        graphics->clear(64, 128, 255); // Nice blue

        // Your rendering code here...

        // Present frame
        graphics->present();
    }

    // Cleanup is automatic via RAII
    return 0;
}
```

### With ImGui Integration

```cpp
#include "ui/GraphicsContext.hpp"
#include <imgui.h>
#include <backends/imgui_impl_sdl2.h>
#include <backends/imgui_impl_sdlrenderer2.h>

class Application {
public:
    bool initialize() {
        // Create graphics context
        kf::ui::GraphicsContext::Config config;
        config.width = 1920;
        config.height = 1080;
        config.title = "ImGui Application";

        graphics_ = std::make_unique<kf::ui::GraphicsContext>(config);

        if (!graphics_->initialize()) {
            return false;
        }

        // Initialize ImGui with graphics context
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();

        ImGui_ImplSDL2_InitForSDLRenderer(
            graphics_->get_window(),
            graphics_->get_renderer()
        );
        ImGui_ImplSDLRenderer2_Init(graphics_->get_renderer());

        return true;
    }

    void run() {
        while (!graphics_->should_close()) {
            // Process events
            graphics_->process_events();

            // Start ImGui frame
            ImGui_ImplSDLRenderer2_NewFrame();
            ImGui_ImplSDL2_NewFrame();
            ImGui::NewFrame();

            // Your ImGui code
            if (ImGui::Begin("Hello World")) {
                ImGui::Text("FPS: %.1f", graphics_->get_fps());
                ImGui::Text("Frame Time: %.3f ms",
                           graphics_->get_delta_time() * 1000.0f);
            }
            ImGui::End();

            // Render
            ImGui::Render();
            graphics_->clear(45, 55, 72);

            ImGui_ImplSDLRenderer2_RenderDrawData(
                ImGui::GetDrawData(),
                graphics_->get_renderer()
            );

            graphics_->present();
        }
    }

private:
    std::unique_ptr<kf::ui::GraphicsContext> graphics_;
};
```

## Configuration Options

The `GraphicsContext::Config` structure provides comprehensive configuration:

```cpp
struct Config {
    int width = 1920;                              // Window width
    int height = 1080;                             // Window height
    std::string title = "Application";             // Window title
    bool resizable = true;                         // Allow window resize
    bool high_dpi = true;                          // Enable high DPI support
    bool vsync = true;                             // Enable vertical sync
    bool accelerated = true;                       // Use hardware acceleration
    int pos_x = SDL_WINDOWPOS_CENTERED;           // Window X position
    int pos_y = SDL_WINDOWPOS_CENTERED;           // Window Y position
};
```

## Event Handling

### Keyboard Events

```cpp
graphics->set_keyboard_callback([](int key, bool pressed) -> bool {
    if (pressed) {
        switch (key) {
            case SDLK_ESCAPE:
                std::cout << "Escape pressed\n";
                return true; // Handled
            case SDLK_F11:
                // Fullscreen toggle (handled automatically)
                return false; // Let GraphicsContext handle it
            default:
                return false; // Not handled
        }
    }
    return false;
});
```

### Window Events

```cpp
graphics->set_window_resize_callback([](int width, int height) {
    std::cout << "Window resized to " << width << "x" << height << "\n";
    // Update your viewport, UI layout, etc.
});
```

## Advanced Features

### Fullscreen Management

```cpp
// Toggle fullscreen mode
graphics->toggle_fullscreen();

// Set specific mode
graphics->set_fullscreen(true);  // Fullscreen
graphics->set_fullscreen(false); // Windowed

// Check current state
bool is_fullscreen = graphics->is_fullscreen();
```

### Window Management

```cpp
// Get/set window size
int width, height;
graphics->get_window_size(width, height);
graphics->set_window_size(1280, 720);

// Get/set window position
int x, y;
graphics->get_window_position(x, y);
graphics->set_window_position(100, 100);

// Change window title
graphics->set_window_title("New Title");
```

### Performance Monitoring

```cpp
// Get current performance metrics
float fps = graphics->get_fps();
float frame_time = graphics->get_delta_time();

// Get renderer information
std::string info = graphics->get_renderer_info();
std::cout << "Using: " << info << "\n";
```

### VSync Control

```cpp
// Enable/disable VSync (requires SDL 2.0.18+)
graphics->set_vsync(true);

// Check VSync status
bool vsync_enabled = graphics->is_vsync_enabled();
```

## Error Handling

The GraphicsContext provides comprehensive error handling:

```cpp
try {
    auto graphics = std::make_unique<kf::ui::GraphicsContext>(config);

    if (!graphics->initialize()) {
        std::cerr << "Graphics initialization failed\n";
        return -1;
    }

    // Use graphics context...

} catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << "\n";
    return -1;
}
```


## Common Patterns

### Game Loop Pattern

```cpp
class Game {
    std::unique_ptr<kf::ui::GraphicsContext> graphics_;
    bool running_ = true;

public:
    void run() {
        while (running_ && !graphics_->should_close()) {
            float delta_time = graphics_->get_delta_time();

            graphics_->process_events();
            update(delta_time);
            render();
            graphics_->present();
        }
    }

private:
    void update(float delta_time) {
        // Game logic
    }

    void render() {
        graphics_->clear(32, 64, 128);
        // Rendering code
    }
};
```

### Application Framework Pattern

```cpp
class Application {
protected:
    std::unique_ptr<kf::ui::GraphicsContext> graphics_;

    virtual void on_update(float delta_time) = 0;
    virtual void on_render() = 0;
    virtual bool on_keyboard(int key, bool pressed) { return false; }

public:
    bool initialize(const kf::ui::GraphicsContext::Config& config) {
        graphics_ = std::make_unique<kf::ui::GraphicsContext>(config);

        graphics_->set_keyboard_callback([this](int key, bool pressed) {
            return on_keyboard(key, pressed);
        });

        return graphics_->initialize();
    }

    void run() {
        while (!graphics_->should_close()) {
            graphics_->process_events();

            float delta_time = graphics_->get_delta_time();
            on_update(delta_time);

            graphics_->clear();
            on_render();
            graphics_->present();
        }
    }
};

class MyApp : public Application {
    void on_update(float delta_time) override {
        // App-specific update logic
    }

    void on_render() override {
        // App-specific rendering
    }
};
```
