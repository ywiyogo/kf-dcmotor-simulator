#include "core/config/Configuration.hpp"
#include "core/simulation/SimulationEngine.hpp"
#include "ui/UIRenderer.hpp"
#include "ui/themes/PastelTheme.hpp"
#include "ui/GraphicsContext.hpp"


#ifdef __EMSCRIPTEN__
    #include <emscripten.h>
    #include <emscripten/html5.h>
#endif

#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

namespace {
    // Application state
    struct AppState {
        std::unique_ptr<kf::ui::GraphicsContext> graphics_context;
        std::unique_ptr<kf::ui::UIRenderer> ui_renderer;
        std::unique_ptr<kf::simulation::SimulationEngine<double>> simulation_engine;
        std::unique_ptr<kf::ui::themes::PastelTheme> theme;
    };

    AppState g_app_state;
}


// Initialize application components
bool initialize_application() {
    try {
        // Initialize configuration
        auto& config = kf::config::config();

        // Try to load saved configuration
        auto load_result = config.load_from_file("kalman_filter_config.ini");
        if (!load_result) {
            std::cout << "Using default configuration: " << load_result.error() << "\n";
        }

        // Validate configuration
        auto validation_result = config.validate();
        if (!validation_result) {
            std::cerr << "Configuration validation failed: " << validation_result.error() << "\n";
            std::cout << "Using default configuration instead.\n";
            config.reset_to_defaults();
        }

        // Create and apply pastel theme
        g_app_state.theme = std::make_unique<kf::ui::themes::PastelTheme>(
            kf::ui::themes::PastelTheme::Variant::LIGHT);
        g_app_state.theme->apply();

        // Create simulation engine
        g_app_state.simulation_engine = std::make_unique<kf::simulation::SimulationEngine<double>>(config);

        // Create UI renderer
        g_app_state.ui_renderer = std::make_unique<kf::ui::UIRenderer>(
            *g_app_state.simulation_engine, *g_app_state.theme);

        // Initialize simulation with default input profile
        g_app_state.simulation_engine->set_input_profile(
            kf::simulation::SimulationEngine<double>::InputProfile::STEP, {12.0, 1.0});

        std::cout << "Application initialized successfully!\n";
        return true;

    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize application: " << e.what() << "\n";
        return false;
    }
}

// Handle keyboard events through graphics context callback
bool handle_keyboard_events(kf::ui::Key key, bool pressed) {
    if (!pressed) return false; // Only handle key press events

    switch (key) {
        case kf::ui::Key::F5:
            // Reload configuration
            if (g_app_state.simulation_engine) {
                kf::config::config().reset_to_defaults();
                g_app_state.simulation_engine->initialize();
                std::cout << "Configuration reloaded and simulation reset.\n";
            }
            return true;

        case kf::ui::Key::F1:
            // Show help/about (will be handled by UI)
            if (g_app_state.ui_renderer) {
                g_app_state.ui_renderer->show_help_dialog(true);
            }
            return true;

        case kf::ui::Key::SPACE:
            // Toggle simulation (if not handled by UI)
            if (g_app_state.simulation_engine) {
                if (g_app_state.simulation_engine->is_running()) {
                    g_app_state.simulation_engine->stop();
                } else {
                    auto result = g_app_state.simulation_engine->start(
                        kf::simulation::SimulationEngine<double>::SimulationMode::REAL_TIME);
                    if (!result) {
                        std::cerr << "Failed to start simulation: " << result.error() << "\n";
                    }
                }
            }
            return true;

        default:
            return false; // Not handled
    }
}

// Handle window resize events through graphics context callback
void handle_window_resize(int width, int height) {
    if (g_app_state.ui_renderer) {
        g_app_state.ui_renderer->on_window_resize(width, height);
    }
}

// Main render function
void render_frame() {
    if (!g_app_state.graphics_context || !g_app_state.graphics_context->is_valid()) {
        return;
    }

    // Process events through graphics context
    if (!g_app_state.graphics_context->process_events()) {
        return; // Should close
    }

    // Get delta time from graphics context
    float delta_time = g_app_state.graphics_context->get_delta_time();

    // Start the Dear ImGui frame
    g_app_state.graphics_context->imgui_new_frame();

    // Render UI
    if (g_app_state.ui_renderer) {
        g_app_state.ui_renderer->render(delta_time);
    }

    // Clear with a nice background color
    const auto& bg_color = kf::ui::themes::PastelColors::BACKGROUND;
    g_app_state.graphics_context->clear(
        static_cast<uint8_t>(bg_color.x * 255),
        static_cast<uint8_t>(bg_color.y * 255),
        static_cast<uint8_t>(bg_color.z * 255),
        static_cast<uint8_t>(bg_color.w * 255)
    );

    // Render ImGui and present
    g_app_state.graphics_context->imgui_render();
    g_app_state.graphics_context->present();
}

// Main render loop
void render_loop() {
#ifdef __EMSCRIPTEN__
    // For Emscripten, we use the main loop callback
    emscripten_set_main_loop([]() {
        render_frame();
        if (g_app_state.graphics_context->should_close() ||
            (g_app_state.ui_renderer && g_app_state.ui_renderer->should_close())) {
            emscripten_cancel_main_loop();
        }
    }, 0, 1);
#else
    // Native render loop
    while (!g_app_state.graphics_context->should_close() &&
           (!g_app_state.ui_renderer || !g_app_state.ui_renderer->should_close())) {
        render_frame();

        // Limit frame rate to reduce CPU usage
        std::this_thread::sleep_for(std::chrono::microseconds(16667)); // ~60 FPS
    }
#endif
}

// Cleanup application resources
void cleanup() {
    // Save configuration before exit
    if (g_app_state.simulation_engine) {
        auto save_result = kf::config::config().save_to_file("kalman_filter_config.ini");
        if (!save_result) {
            std::cerr << "Failed to save configuration: " << save_result.error() << "\n";
        }
    }

    // Stop simulation
    if (g_app_state.simulation_engine) {
        g_app_state.simulation_engine->stop();
    }

    // Cleanup UI
    g_app_state.ui_renderer.reset();
    g_app_state.simulation_engine.reset();
    g_app_state.theme.reset();

    // Cleanup graphics context (this will handle SDL and ImGui cleanup)
    g_app_state.graphics_context.reset();

    std::cout << "Application cleanup completed.\n";
}

// Print application information
void print_application_info() {
    std::cout << "================================================\n";
    std::cout << "  Kalman Filter DC Motor Simulation\n";
    std::cout << "  Modern C++23 Implementation with Graphics Abstraction\n";
    std::cout << "================================================\n";
    std::cout << "Features:\n";
    std::cout << "  • Real-time Kalman filter visualization\n";
    std::cout << "  • Physics-based DC motor simulation\n";
    std::cout << "  • Interactive parameter tuning\n";
    std::cout << "  • Performance analytics\n";
    std::cout << "  • Modern pastel UI theme\n";
    std::cout << "  • Parallel STL execution support\n";
    std::cout << "  • Clean graphics abstraction layer\n";
    std::cout << "\n";
    std::cout << "Controls:\n";
    std::cout << "  F1    - Show help/about dialog\n";
    std::cout << "  F5    - Reload configuration\n";
    std::cout << "  F11   - Toggle fullscreen\n";
    std::cout << "  SPACE - Start/Stop simulation\n";
    std::cout << "  ESC   - Exit application\n";
    std::cout << "================================================\n";
    std::cout << "\n";
}

// Application entry point
int main(int /*argc*/, char* /*argv*/[]) {
    // Print application information
    print_application_info();

    try {
        // Create graphics context configuration
        kf::ui::GraphicsContext::Config graphics_config;
        graphics_config.width = 1920;
        graphics_config.height = 1080;
        graphics_config.title = "Kalman Filter DC Motor Simulation - Modern C++23";
        graphics_config.resizable = true;
        graphics_config.high_dpi = true;
        graphics_config.vsync = true;
        graphics_config.accelerated = true;

        // Create graphics context - SDL details are completely hidden
        g_app_state.graphics_context = std::make_unique<kf::ui::GraphicsContext>(graphics_config);

        // Set up event callbacks
        g_app_state.graphics_context->set_keyboard_callback(handle_keyboard_events);
        g_app_state.graphics_context->set_window_resize_callback(handle_window_resize);

        // Initialize graphics context
        if (!g_app_state.graphics_context->initialize()) {
            std::cerr << "Failed to initialize graphics context\n";
            return -1;
        }

        // Initialize ImGui through graphics context
        if (!g_app_state.graphics_context->initialize_imgui()) {
            std::cerr << "Failed to initialize ImGui\n";
            cleanup();
            return -1;
        }

        // Initialize application
        if (!initialize_application()) {
            std::cerr << "Failed to initialize application\n";
            cleanup();
            return -1;
        }

        std::cout << "Starting main render loop...\n";
        std::cout << "Graphics: " << g_app_state.graphics_context->get_renderer_info() << "\n";

        // Main render loop
        render_loop();

        std::cout << "Application shutting down...\n";

    } catch (const std::exception& e) {
        std::cerr << "Unhandled exception: " << e.what() << "\n";
        cleanup();
        return -1;
    } catch (...) {
        std::cerr << "Unknown exception caught\n";
        cleanup();
        return -1;
    }

    // Cleanup
    cleanup();

    std::cout << "Application exited successfully.\n";
    return 0;
}
