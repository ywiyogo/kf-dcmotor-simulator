#include "core/config/Configuration.hpp"
#include "core/simulation/SimulationEngine.hpp"
#include "ui/UIRenderer.hpp"
#include "ui/themes/PastelTheme.hpp"

#include <imgui.h>
#include <backends/imgui_impl_sdl2.h>
#include <backends/imgui_impl_sdlrenderer2.h>

#ifdef __EMSCRIPTEN__
    #include <SDL.h>
    #include <emscripten.h>
    #include <emscripten/html5.h>
#else
    #include <SDL2/SDL.h>
#endif

#include <iostream>
#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>

namespace {
    // Application configuration
    constexpr int INITIAL_WINDOW_WIDTH = 1920;
    constexpr int INITIAL_WINDOW_HEIGHT = 1080;
    constexpr const char* WINDOW_TITLE = "Kalman Filter DC Motor Simulation - Modern C++23";
    
    // Application state
    struct AppState {
        SDL_Window* window = nullptr;
        SDL_Renderer* renderer = nullptr;
        std::unique_ptr<kf::ui::UIRenderer> ui_renderer;
        std::unique_ptr<kf::simulation::SimulationEngine<double>> simulation_engine;
        std::unique_ptr<kf::ui::themes::PastelTheme> theme;
        bool should_close = false;
        bool is_fullscreen = false;
        int windowed_width = INITIAL_WINDOW_WIDTH;
        int windowed_height = INITIAL_WINDOW_HEIGHT;
        int windowed_pos_x = 100;
        int windowed_pos_y = 100;
    };
    
    AppState g_app_state;
}

// Handle SDL events
bool handle_sdl_event(const SDL_Event& event) {
    ImGui_ImplSDL2_ProcessEvent(&event);
    
    switch (event.type) {
        case SDL_QUIT:
            g_app_state.should_close = true;
            return true;
            
        case SDL_KEYDOWN:
            switch (event.key.keysym.sym) {
                case SDLK_ESCAPE:
                    g_app_state.should_close = true;
                    return true;
                    
                case SDLK_F11:
                    // Toggle fullscreen
                    if (!g_app_state.is_fullscreen) {
                        // Save windowed position and size
                        SDL_GetWindowPosition(g_app_state.window, &g_app_state.windowed_pos_x, &g_app_state.windowed_pos_y);
                        SDL_GetWindowSize(g_app_state.window, &g_app_state.windowed_width, &g_app_state.windowed_height);
                        
                        // Set fullscreen
                        SDL_SetWindowFullscreen(g_app_state.window, SDL_WINDOW_FULLSCREEN_DESKTOP);
                        g_app_state.is_fullscreen = true;
                    } else {
                        // Restore windowed mode
                        SDL_SetWindowFullscreen(g_app_state.window, 0);
                        SDL_SetWindowPosition(g_app_state.window, g_app_state.windowed_pos_x, g_app_state.windowed_pos_y);
                        SDL_SetWindowSize(g_app_state.window, g_app_state.windowed_width, g_app_state.windowed_height);
                        g_app_state.is_fullscreen = false;
                    }
                    return true;
                    
                case SDLK_F5:
                    // Reload configuration
                    if (g_app_state.simulation_engine) {
                        kf::config::config().reset_to_defaults();
                        g_app_state.simulation_engine->initialize();
                        std::cout << "Configuration reloaded and simulation reset." << std::endl;
                    }
                    return true;
                    
                case SDLK_F1:
                    // Show help/about (will be handled by UI)
                    if (g_app_state.ui_renderer) {
                        g_app_state.ui_renderer->show_help_dialog(true);
                    }
                    return true;
            }
            break;
            
        case SDL_WINDOWEVENT:
            if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
                if (g_app_state.ui_renderer) {
                    g_app_state.ui_renderer->on_window_resize(event.window.data1, event.window.data2);
                }
            }
            break;
    }
    
    return false;
}

// Initialize SDL and create window
bool initialize_graphics() {
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0) {
        std::cerr << "Error: " << SDL_GetError() << std::endl;
        return false;
    }

    // Create window with SDL2 renderer
    SDL_WindowFlags window_flags = static_cast<SDL_WindowFlags>(
        SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI
    );
    
    g_app_state.window = SDL_CreateWindow(
        WINDOW_TITLE,
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        INITIAL_WINDOW_WIDTH,
        INITIAL_WINDOW_HEIGHT,
        window_flags
    );
    
    if (!g_app_state.window) {
        std::cerr << "Error: SDL_CreateWindow(): " << SDL_GetError() << std::endl;
        return false;
    }

    // Create SDL2 renderer
    g_app_state.renderer = SDL_CreateRenderer(
        g_app_state.window, 
        -1, 
        SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED
    );
    
    if (!g_app_state.renderer) {
        std::cerr << "Error: SDL_CreateRenderer(): " << SDL_GetError() << std::endl;
        return false;
    }

    // Print renderer information
    SDL_RendererInfo renderer_info;
    SDL_GetRendererInfo(g_app_state.renderer, &renderer_info);
    std::cout << "SDL2 Renderer: " << renderer_info.name << std::endl;
    std::cout << "SDL2 Renderer Flags: " << renderer_info.flags << std::endl;

    return true;
}

// Initialize ImGui
bool initialize_imgui() {
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    
    // Enable features
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;         // Enable Docking
    
    // Note: ViewportsEnable is not supported with SDL2 renderer
    // io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;    // Disable for SDL2 renderer
    
    // Font configuration
    // Note: FontGlobalScale has been deprecated in newer ImGui versions
    
    // Load fonts
    io.Fonts->AddFontDefault();
    
    // Setup Platform/Renderer backends
    if (!ImGui_ImplSDL2_InitForSDLRenderer(g_app_state.window, g_app_state.renderer)) {
        std::cerr << "Failed to initialize ImGui SDL2 backend" << std::endl;
        return false;
    }
    
    if (!ImGui_ImplSDLRenderer2_Init(g_app_state.renderer)) {
        std::cerr << "Failed to initialize ImGui SDL2 Renderer backend" << std::endl;
        return false;
    }

    return true;
}

// Initialize application components
bool initialize_application() {
    try {
        // Initialize configuration
        auto& config = kf::config::config();
        
        // Try to load saved configuration
        auto load_result = config.load_from_file("kalman_filter_config.ini");
        if (!load_result) {
            std::cout << "Using default configuration: " << load_result.error() << std::endl;
        }

        // Validate configuration
        auto validation_result = config.validate();
        if (!validation_result) {
            std::cerr << "Configuration validation failed: " << validation_result.error() << std::endl;
            std::cout << "Using default configuration instead." << std::endl;
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

        std::cout << "Application initialized successfully!" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize application: " << e.what() << std::endl;
        return false;
    }
}

// Main render function
void render_frame() {
    // Calculate delta time
    static auto last_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    auto delta_time = std::chrono::duration<float>(current_time - last_time).count();
    last_time = current_time;

    // Poll and handle events
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        handle_sdl_event(event);
    }

    // Start the Dear ImGui frame
    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    // Render UI
    if (g_app_state.ui_renderer) {
        g_app_state.ui_renderer->render(delta_time);
        
        // Check if UI requested application close
        if (g_app_state.ui_renderer->should_close()) {
            g_app_state.should_close = true;
        }
    }

    // Rendering
    ImGui::Render();
    
    // Clear with a nice background color
    const auto& bg_color = kf::ui::themes::PastelColors::BACKGROUND;
    SDL_SetRenderDrawColor(g_app_state.renderer, 
        static_cast<Uint8>(bg_color.x * 255), 
        static_cast<Uint8>(bg_color.y * 255), 
        static_cast<Uint8>(bg_color.z * 255), 
        static_cast<Uint8>(bg_color.w * 255));
    SDL_RenderClear(g_app_state.renderer);
    
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), g_app_state.renderer);
    SDL_RenderPresent(g_app_state.renderer);
}

// Main render loop
void render_loop() {
#ifdef __EMSCRIPTEN__
    // For Emscripten, we use the main loop callback
    emscripten_set_main_loop([]() {
        render_frame();
        if (g_app_state.should_close) {
            emscripten_cancel_main_loop();
        }
    }, 0, 1);
#else
    // Native render loop
    while (!g_app_state.should_close) {
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
            std::cerr << "Failed to save configuration: " << save_result.error() << std::endl;
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

    // Cleanup ImGui
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    // Cleanup SDL
    if (g_app_state.renderer) {
        SDL_DestroyRenderer(g_app_state.renderer);
    }
    if (g_app_state.window) {
        SDL_DestroyWindow(g_app_state.window);
    }
    SDL_Quit();

    std::cout << "Application cleanup completed." << std::endl;
}

// Print application information
void print_application_info() {
    std::cout << "================================================" << std::endl;
    std::cout << "  Kalman Filter DC Motor Simulation" << std::endl;
    std::cout << "  Modern C++23 Implementation" << std::endl;
    std::cout << "================================================" << std::endl;
    std::cout << "Features:" << std::endl;
    std::cout << "  • Real-time Kalman filter visualization" << std::endl;
    std::cout << "  • Physics-based DC motor simulation" << std::endl;
    std::cout << "  • Interactive parameter tuning" << std::endl;
    std::cout << "  • Performance analytics" << std::endl;
    std::cout << "  • Modern pastel UI theme" << std::endl;
    std::cout << "  • Parallel STL execution support" << std::endl;
    std::cout << "  • SDL2 cross-platform support" << std::endl;
    std::cout << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  F1  - Show help/about dialog" << std::endl;
    std::cout << "  F5  - Reload configuration" << std::endl;
    std::cout << "  F11 - Toggle fullscreen" << std::endl;
    std::cout << "  ESC - Exit application" << std::endl;
    std::cout << "================================================" << std::endl;
    std::cout << std::endl;
}

// Application entry point
int main(int /*argc*/, char* /*argv*/[]) {
    // Print application information
    print_application_info();

    try {
        // Initialize graphics system
        if (!initialize_graphics()) {
            std::cerr << "Failed to initialize graphics system" << std::endl;
            return -1;
        }

        // Initialize ImGui
        if (!initialize_imgui()) {
            std::cerr << "Failed to initialize ImGui" << std::endl;
            cleanup();
            return -1;
        }

        // Initialize application
        if (!initialize_application()) {
            std::cerr << "Failed to initialize application" << std::endl;
            cleanup();
            return -1;
        }

        std::cout << "Starting main render loop..." << std::endl;

        // Main render loop
        render_loop();

        std::cout << "Application shutting down..." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Unhandled exception: " << e.what() << std::endl;
        cleanup();
        return -1;
    } catch (...) {
        std::cerr << "Unknown exception caught" << std::endl;
        cleanup();
        return -1;
    }

    // Cleanup
    cleanup();
    
    std::cout << "Application exited successfully." << std::endl;
    return 0;
}