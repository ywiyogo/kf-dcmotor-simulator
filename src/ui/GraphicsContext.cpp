#include "GraphicsContext.hpp"
#include <iostream>
#include <sstream>
#include <stdexcept>

#include <imgui.h>
#include <backends/imgui_impl_sdl2.h>
#include <backends/imgui_impl_sdlrenderer2.h>

namespace kf::ui {

GraphicsContext::GraphicsContext(const Config& config) 
    : config_(config) {
    // Get initial timing
    last_frame_time_ = SDL_GetPerformanceCounter();
    fps_last_time_ = last_frame_time_;
}

GraphicsContext::GraphicsContext() 
    : config_(Config{}) {
    // Get initial timing
    last_frame_time_ = SDL_GetPerformanceCounter();
    fps_last_time_ = last_frame_time_;
}

GraphicsContext::~GraphicsContext() {
    cleanup();
}

GraphicsContext::GraphicsContext(GraphicsContext&& other) noexcept 
    : config_(std::move(other.config_))
    , window_(other.window_)
    , renderer_(other.renderer_)
    , initialized_(other.initialized_)
    , should_close_(other.should_close_)
    , is_fullscreen_(other.is_fullscreen_)
    , windowed_width_(other.windowed_width_)
    , windowed_height_(other.windowed_height_)
    , windowed_pos_x_(other.windowed_pos_x_)
    , windowed_pos_y_(other.windowed_pos_y_)
    , window_resize_callback_(std::move(other.window_resize_callback_))
    , keyboard_callback_(std::move(other.keyboard_callback_))
    , last_frame_time_(other.last_frame_time_)
    , delta_time_(other.delta_time_)
    , fps_(other.fps_)
    , frame_count_(other.frame_count_)
    , fps_last_time_(other.fps_last_time_) {
    
    // Reset moved-from object
    other.window_ = nullptr;
    other.renderer_ = nullptr;
    other.initialized_ = false;
}

GraphicsContext& GraphicsContext::operator=(GraphicsContext&& other) noexcept {
    if (this != &other) {
        // Cleanup current resources
        cleanup();
        
        // Move data
        config_ = std::move(other.config_);
        window_ = other.window_;
        renderer_ = other.renderer_;
        initialized_ = other.initialized_;
        should_close_ = other.should_close_;
        is_fullscreen_ = other.is_fullscreen_;
        windowed_width_ = other.windowed_width_;
        windowed_height_ = other.windowed_height_;
        windowed_pos_x_ = other.windowed_pos_x_;
        windowed_pos_y_ = other.windowed_pos_y_;
        window_resize_callback_ = std::move(other.window_resize_callback_);
        keyboard_callback_ = std::move(other.keyboard_callback_);
        last_frame_time_ = other.last_frame_time_;
        delta_time_ = other.delta_time_;
        fps_ = other.fps_;
        frame_count_ = other.frame_count_;
        fps_last_time_ = other.fps_last_time_;
        
        // Reset moved-from object
        other.window_ = nullptr;
        other.renderer_ = nullptr;
        other.initialized_ = false;
    }
    return *this;
}

bool GraphicsContext::initialize() {
    if (initialized_) {
        std::cerr << "GraphicsContext: Already initialized\n";
        return true;
    }

    try {
        if (!initialize_sdl()) {
            std::cerr << "GraphicsContext: Failed to initialize SDL\n";
            return false;
        }

        if (!create_window()) {
            std::cerr << "GraphicsContext: Failed to create window\n";
            cleanup();
            return false;
        }

        if (!create_renderer()) {
            std::cerr << "GraphicsContext: Failed to create renderer\n";
            cleanup();
            return false;
        }

        initialized_ = true;
        
        // Store initial windowed mode settings
        windowed_width_ = config_.width;
        windowed_height_ = config_.height;
        windowed_pos_x_ = config_.pos_x;
        windowed_pos_y_ = config_.pos_y;

        std::cout << "GraphicsContext: Initialized successfully\n";
        std::cout << "  " << get_renderer_info() << "\n";
        
        return true;

    } catch (const std::exception& e) {
        std::cerr << "GraphicsContext: Exception during initialization: " << e.what() << "\n";
        cleanup();
        return false;
    }
}

bool GraphicsContext::is_valid() const {
    return initialized_ && window_ != nullptr && renderer_ != nullptr;
}

void GraphicsContext::get_window_size(int& width, int& height) const {
    if (window_) {
        SDL_GetWindowSize(window_, &width, &height);
    } else {
        width = height = 0;
    }
}

void GraphicsContext::set_window_size(int width, int height) {
    if (window_) {
        SDL_SetWindowSize(window_, width, height);
        config_.width = width;
        config_.height = height;
    }
}

void GraphicsContext::get_window_position(int& x, int& y) const {
    if (window_) {
        SDL_GetWindowPosition(window_, &x, &y);
    } else {
        x = y = 0;
    }
}

void GraphicsContext::set_window_position(int x, int y) {
    if (window_) {
        SDL_SetWindowPosition(window_, x, y);
        config_.pos_x = x;
        config_.pos_y = y;
    }
}

void GraphicsContext::toggle_fullscreen() {
    set_fullscreen(!is_fullscreen_);
}

void GraphicsContext::set_fullscreen(bool fullscreen) {
    if (!window_ || is_fullscreen_ == fullscreen) {
        return;
    }

    if (fullscreen) {
        // Save current windowed position and size
        SDL_GetWindowPosition(window_, &windowed_pos_x_, &windowed_pos_y_);
        SDL_GetWindowSize(window_, &windowed_width_, &windowed_height_);

        // Set fullscreen
        if (SDL_SetWindowFullscreen(window_, SDL_WINDOW_FULLSCREEN_DESKTOP) == 0) {
            is_fullscreen_ = true;
            std::cout << "GraphicsContext: Switched to fullscreen mode\n";
        } else {
            std::cerr << "GraphicsContext: Failed to set fullscreen: " << SDL_GetError() << "\n";
        }
    } else {
        // Restore windowed mode
        if (SDL_SetWindowFullscreen(window_, 0) == 0) {
            SDL_SetWindowPosition(window_, windowed_pos_x_, windowed_pos_y_);
            SDL_SetWindowSize(window_, windowed_width_, windowed_height_);
            is_fullscreen_ = false;
            std::cout << "GraphicsContext: Switched to windowed mode\n";
        } else {
            std::cerr << "GraphicsContext: Failed to set windowed mode: " << SDL_GetError() << "\n";
        }
    }
}

bool GraphicsContext::process_events() {
    if (!initialized_) {
        return false;
    }

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        // Pass events to ImGui first if initialized
        if (imgui_initialized_) {
            ImGui_ImplSDL2_ProcessEvent(&event);
        }

        switch (event.type) {
            case SDL_QUIT:
                should_close_ = true;
                return false;

            case SDL_WINDOWEVENT:
                handle_window_event(event);
                break;

            case SDL_KEYDOWN:
            case SDL_KEYUP:
                handle_keyboard_event(event);
                break;

            default:
                // Other events handled by ImGui above
                break;
        }
    }

    return !should_close_;
}

void GraphicsContext::present() {
    if (renderer_) {
        SDL_RenderPresent(renderer_);
        update_frame_timing();
    }
}

void GraphicsContext::clear(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    if (renderer_) {
        SDL_SetRenderDrawColor(renderer_, r, g, b, a);
        SDL_RenderClear(renderer_);
    }
}

std::string GraphicsContext::get_renderer_info() const {
    if (!renderer_) {
        return "No renderer available";
    }

    SDL_RendererInfo info;
    if (SDL_GetRendererInfo(renderer_, &info) != 0) {
        return "Failed to get renderer info: " + std::string(SDL_GetError());
    }

    std::ostringstream oss;
    oss << "Renderer: " << info.name;
    oss << " | Flags: ";
    
    if (info.flags & SDL_RENDERER_SOFTWARE) oss << "SOFTWARE ";
    if (info.flags & SDL_RENDERER_ACCELERATED) oss << "ACCELERATED ";
    if (info.flags & SDL_RENDERER_PRESENTVSYNC) oss << "VSYNC ";
    if (info.flags & SDL_RENDERER_TARGETTEXTURE) oss << "TARGETTEXTURE ";

    oss << "| Max Texture: " << info.max_texture_width << "x" << info.max_texture_height;

    return oss.str();
}

float GraphicsContext::get_delta_time() const {
    return delta_time_;
}

float GraphicsContext::get_fps() const {
    return fps_;
}

void GraphicsContext::set_vsync(bool enabled) {
    if (renderer_) {
        // Note: SDL_RenderSetVSync is available in SDL 2.0.18+
        // For older versions, VSync must be set during renderer creation
#if SDL_VERSION_ATLEAST(2, 0, 18)
        if (SDL_RenderSetVSync(renderer_, enabled ? 1 : 0) == 0) {
            config_.vsync = enabled;
            std::cout << "GraphicsContext: VSync " << (enabled ? "enabled" : "disabled") << "\n";
        } else {
            std::cerr << "GraphicsContext: Failed to set VSync: " << SDL_GetError() << "\n";
        }
#else
        std::cerr << "GraphicsContext: VSync control requires SDL 2.0.18 or later\n";
        (void)enabled; // Suppress unused parameter warning
#endif
    }
}

void GraphicsContext::set_window_title(const std::string& title) {
    if (window_) {
        SDL_SetWindowTitle(window_, title.c_str());
        config_.title = title;
    }
}

std::string GraphicsContext::get_window_title() const {
    if (window_) {
        const char* title = SDL_GetWindowTitle(window_);
        return title ? std::string(title) : "";
    }
    return "";
}

bool GraphicsContext::initialize_imgui() {
    if (!is_valid()) {
        std::cerr << "GraphicsContext: Cannot initialize ImGui - graphics context is not valid\n";
        return false;
    }

    if (imgui_initialized_) {
        std::cerr << "GraphicsContext: ImGui already initialized\n";
        return true;
    }

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();

    // Enable features
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;         // Enable Docking

    // Load fonts
    io.Fonts->AddFontDefault();

    // Setup Platform/Renderer backends
    if (!ImGui_ImplSDL2_InitForSDLRenderer(window_, renderer_)) {
        std::cerr << "GraphicsContext: Failed to initialize ImGui SDL2 backend\n";
        return false;
    }

    if (!ImGui_ImplSDLRenderer2_Init(renderer_)) {
        std::cerr << "GraphicsContext: Failed to initialize ImGui SDL2 Renderer backend\n";
        ImGui_ImplSDL2_Shutdown();
        return false;
    }

    imgui_initialized_ = true;
    std::cout << "GraphicsContext: ImGui initialized successfully\n";
    return true;
}

void GraphicsContext::cleanup_imgui() {
    if (imgui_initialized_) {
        ImGui_ImplSDLRenderer2_Shutdown();
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext();
        imgui_initialized_ = false;
    }
}

void GraphicsContext::imgui_new_frame() {
    if (imgui_initialized_) {
        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
    }
}

void GraphicsContext::imgui_render() {
    if (imgui_initialized_) {
        ImGui::Render();
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer_);
    }
}

Key GraphicsContext::sdl_key_to_key(int sdl_key) {
    switch (sdl_key) {
        case SDLK_ESCAPE: return Key::ESCAPE;
        case SDLK_F1: return Key::F1;
        case SDLK_F5: return Key::F5;
        case SDLK_F11: return Key::F11;
        case SDLK_SPACE: return Key::SPACE;
        default: return Key::UNKNOWN;
    }
}

bool GraphicsContext::initialize_sdl() {
    // Initialize SDL
    Uint32 init_flags = SDL_INIT_VIDEO | SDL_INIT_TIMER;
    
    if (SDL_Init(init_flags) != 0) {
        std::cerr << "GraphicsContext: SDL_Init failed: " << SDL_GetError() << "\n";
        return false;
    }

    // Set SDL hints for better performance and compatibility
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1"); // Linear filtering
    
    if (config_.high_dpi) {
        SDL_SetHint(SDL_HINT_VIDEO_ALLOW_SCREENSAVER, "0");
        SDL_SetHint(SDL_HINT_VIDEO_HIGHDPI_DISABLED, "0");
    }

    return true;
}

bool GraphicsContext::create_window() {
    // Setup window flags
    SDL_WindowFlags window_flags = static_cast<SDL_WindowFlags>(0);
    
    if (config_.resizable) {
        window_flags = static_cast<SDL_WindowFlags>(window_flags | SDL_WINDOW_RESIZABLE);
    }
    
    if (config_.high_dpi) {
        window_flags = static_cast<SDL_WindowFlags>(window_flags | SDL_WINDOW_ALLOW_HIGHDPI);
    }

    // Create window
    window_ = SDL_CreateWindow(
        config_.title.c_str(),
        config_.pos_x,
        config_.pos_y,
        config_.width,
        config_.height,
        window_flags
    );

    if (!window_) {
        std::cerr << "GraphicsContext: SDL_CreateWindow failed: " << SDL_GetError() << "\n";
        return false;
    }

    return true;
}

bool GraphicsContext::create_renderer() {
    // Setup renderer flags
    Uint32 renderer_flags = 0;
    
    if (config_.accelerated) {
        renderer_flags |= SDL_RENDERER_ACCELERATED;
    }
    
    if (config_.vsync) {
        renderer_flags |= SDL_RENDERER_PRESENTVSYNC;
    }

    // Create renderer
    renderer_ = SDL_CreateRenderer(window_, -1, renderer_flags);

    if (!renderer_) {
        std::cerr << "GraphicsContext: SDL_CreateRenderer failed: " << SDL_GetError() << "\n";
        return false;
    }

    // Set blend mode for alpha blending
    SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);

    return true;
}

void GraphicsContext::handle_window_event(const SDL_Event& event) {
    switch (event.window.event) {
        case SDL_WINDOWEVENT_RESIZED:
        case SDL_WINDOWEVENT_SIZE_CHANGED:
            if (window_resize_callback_) {
                window_resize_callback_(event.window.data1, event.window.data2);
            }
            // Update config if not in fullscreen
            if (!is_fullscreen_) {
                config_.width = event.window.data1;
                config_.height = event.window.data2;
            }
            break;

        case SDL_WINDOWEVENT_MOVED:
            // Update config if not in fullscreen
            if (!is_fullscreen_) {
                config_.pos_x = event.window.data1;
                config_.pos_y = event.window.data2;
            }
            break;

        case SDL_WINDOWEVENT_CLOSE:
            should_close_ = true;
            break;

        default:
            break;
    }
}

void GraphicsContext::handle_keyboard_event(const SDL_Event& event) {
    if (keyboard_callback_) {
        Key key = sdl_key_to_key(event.key.keysym.sym);
        if (key != Key::UNKNOWN) {
            bool handled = keyboard_callback_(
                key, 
                event.type == SDL_KEYDOWN
            );
            
            // If not handled by callback, process common keys
            if (!handled && event.type == SDL_KEYDOWN) {
                switch (key) {
                    case Key::ESCAPE:
                        should_close_ = true;
                        break;
                    case Key::F11:
                        toggle_fullscreen();
                        break;
                    default:
                        break;
                }
            }
        }
    }
}

void GraphicsContext::update_frame_timing() {
    uint64_t current_time = SDL_GetPerformanceCounter();
    uint64_t frequency = SDL_GetPerformanceFrequency();
    
    // Calculate delta time
    delta_time_ = static_cast<float>(current_time - last_frame_time_) / static_cast<float>(frequency);
    last_frame_time_ = current_time;
    
    // Calculate FPS (update every second)
    frame_count_++;
    if (current_time - fps_last_time_ >= frequency) {
        fps_ = static_cast<float>(frame_count_) * static_cast<float>(frequency) / static_cast<float>(current_time - fps_last_time_);
        frame_count_ = 0;
        fps_last_time_ = current_time;
    }
}

void GraphicsContext::cleanup() {
    // Cleanup ImGui first
    cleanup_imgui();

    if (renderer_) {
        SDL_DestroyRenderer(renderer_);
        renderer_ = nullptr;
    }

    if (window_) {
        SDL_DestroyWindow(window_);
        window_ = nullptr;
    }

    if (initialized_) {
        SDL_Quit();
        initialized_ = false;
    }

    should_close_ = false;
    is_fullscreen_ = false;
}

} // namespace kf::ui