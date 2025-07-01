#pragma once

#include <string>
#include <functional>

#ifdef __EMSCRIPTEN__
#include <SDL.h>
#else
#include <SDL2/SDL.h>
#endif

namespace kf::ui {

/**
 * @brief Key codes abstraction to avoid SDL dependencies in application code
 */
enum class Key {
    ESCAPE,
    F1,
    F5,
    F11,
    SPACE,
    UNKNOWN
};

/**
 * @brief Graphics context abstraction that encapsulates SDL window and renderer management
 *
 * This class provides a clean interface for graphics initialization, event handling,
 * and resource management, hiding SDL implementation details from the rest of the application.
 */
class GraphicsContext {
public:
    /**
     * @brief Configuration structure for graphics context initialization
     */
    struct Config {
        int width = 1920;
        int height = 1080;
        std::string title = "Application";
        bool resizable = true;
        bool high_dpi = true;
        bool vsync = true;
        bool accelerated = true;
        int pos_x = SDL_WINDOWPOS_CENTERED;
        int pos_y = SDL_WINDOWPOS_CENTERED;
    };

    /**
     * @brief Window event callback type
     */
    using WindowEventCallback = std::function<void(int width, int height)>;

    /**
     * @brief Keyboard event callback type
     */
    using KeyboardEventCallback = std::function<bool(Key key, bool pressed)>;

    /**
     * @brief Constructor
     * @param config Graphics configuration
     */
    explicit GraphicsContext(const Config& config);

    /**
     * @brief Constructor with default configuration
     */
    GraphicsContext();

    /**
     * @brief Destructor - handles cleanup automatically
     */
    ~GraphicsContext();

    // Non-copyable, non-assignable
    GraphicsContext(const GraphicsContext&) = delete;
    GraphicsContext& operator=(const GraphicsContext&) = delete;

    // Movable
    GraphicsContext(GraphicsContext&& other) noexcept;
    GraphicsContext& operator=(GraphicsContext&& other) noexcept;

    /**
     * @brief Initialize the graphics context
     * @return true if initialization succeeded, false otherwise
     */
    bool initialize();

    /**
     * @brief Check if the graphics context is valid
     * @return true if context is valid and ready to use
     */
    bool is_valid() const;

    /**
     * @brief Get SDL window handle
     * @return SDL_Window pointer or nullptr if not initialized
     */
    SDL_Window* get_window() const { return window_; }

    /**
     * @brief Get SDL renderer handle
     * @return SDL_Renderer pointer or nullptr if not initialized
     */
    SDL_Renderer* get_renderer() const { return renderer_; }

    /**
     * @brief Get current window dimensions
     * @param width Output parameter for width
     * @param height Output parameter for height
     */
    void get_window_size(int& width, int& height) const;

    /**
     * @brief Set window size
     * @param width New window width
     * @param height New window height
     */
    void set_window_size(int width, int height);

    /**
     * @brief Get window position
     * @param x Output parameter for x position
     * @param y Output parameter for y position
     */
    void get_window_position(int& x, int& y) const;

    /**
     * @brief Set window position
     * @param x New window x position
     * @param y New window y position
     */
    void set_window_position(int x, int y);

    /**
     * @brief Check if window should close
     * @return true if close was requested
     */
    bool should_close() const { return should_close_; }

    /**
     * @brief Request window to close
     */
    void request_close() { should_close_ = true; }

    /**
     * @brief Check if window is in fullscreen mode
     * @return true if fullscreen, false if windowed
     */
    bool is_fullscreen() const { return is_fullscreen_; }

    /**
     * @brief Toggle fullscreen mode
     */
    void toggle_fullscreen();

    /**
     * @brief Set fullscreen mode
     * @param fullscreen true for fullscreen, false for windowed
     */
    void set_fullscreen(bool fullscreen);

    /**
     * @brief Process SDL events
     * @return true if events were processed, false if should quit
     */
    bool process_events();

    /**
     * @brief Present the rendered frame
     */
    void present();

    /**
     * @brief Clear the screen with specified color
     * @param r Red component (0-255)
     * @param g Green component (0-255)
     * @param b Blue component (0-255)
     * @param a Alpha component (0-255)
     */
    void clear(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 255);

    /**
     * @brief Set window resize callback
     * @param callback Function to call when window is resized
     */
    void set_window_resize_callback(WindowEventCallback callback) {
        window_resize_callback_ = std::move(callback);
    }

    /**
     * @brief Set keyboard event callback
     * @param callback Function to call for keyboard events
     */
    void set_keyboard_callback(KeyboardEventCallback callback) {
        keyboard_callback_ = std::move(callback);
    }

    /**
     * @brief Get renderer information
     * @return String containing renderer details
     */
    std::string get_renderer_info() const;

    /**
     * @brief Get current frame time in seconds
     * @return Delta time since last frame
     */
    float get_delta_time() const;

    /**
     * @brief Get frames per second
     * @return Current FPS
     */
    float get_fps() const;

    /**
     * @brief Enable/disable VSync
     * @param enabled true to enable VSync, false to disable
     */
    void set_vsync(bool enabled);

    /**
     * @brief Check if VSync is enabled
     * @return true if VSync is enabled
     */
    bool is_vsync_enabled() const { return config_.vsync; }

    /**
     * @brief Set window title
     * @param title New window title
     */
    void set_window_title(const std::string& title);

    /**
     * @brief Get current window title
     * @return Current window title
     */
    std::string get_window_title() const;

    /**
     * @brief Initialize ImGui with this graphics context
     * @return true if initialization succeeded
     */
    bool initialize_imgui();

    /**
     * @brief Cleanup ImGui
     */
    void cleanup_imgui();

    /**
     * @brief Start new ImGui frame
     */
    void imgui_new_frame();

    /**
     * @brief Render ImGui draw data
     */
    void imgui_render();

    /**
     * @brief Check if ImGui is initialized
     * @return true if ImGui is ready to use
     */
    bool is_imgui_initialized() const { return imgui_initialized_; }

private:
    /**
     * @brief Initialize SDL subsystems
     * @return true if successful
     */
    bool initialize_sdl();

    /**
     * @brief Create SDL window
     * @return true if successful
     */
    bool create_window();

    /**
     * @brief Create SDL renderer
     * @return true if successful
     */
    bool create_renderer();

    /**
     * @brief Handle SDL window events
     * @param event SDL event to handle
     */
    void handle_window_event(const SDL_Event& event);

    /**
     * @brief Handle SDL keyboard events
     * @param event SDL event to handle
     */
    void handle_keyboard_event(const SDL_Event& event);

    /**
     * @brief Update frame timing
     */
    void update_frame_timing();

    /**
     * @brief Convert SDL key code to our Key enum
     * @param sdl_key SDL key code
     * @return Corresponding Key enum value
     */
    Key sdl_key_to_key(int sdl_key);

    /**
     * @brief Cleanup resources
     */
    void cleanup();

    // Configuration
    Config config_;

    // SDL resources
    SDL_Window* window_ = nullptr;
    SDL_Renderer* renderer_ = nullptr;

    // State
    bool initialized_ = false;
    bool should_close_ = false;
    bool is_fullscreen_ = false;

    // Windowed mode state (for fullscreen toggle)
    int windowed_width_ = 1920;
    int windowed_height_ = 1080;
    int windowed_pos_x_ = 100;
    int windowed_pos_y_ = 100;

    // Callbacks
    WindowEventCallback window_resize_callback_;
    KeyboardEventCallback keyboard_callback_;

    // Frame timing
    mutable uint64_t last_frame_time_ = 0;
    mutable float delta_time_ = 0.0f;
    mutable float fps_ = 0.0f;
    mutable uint32_t frame_count_ = 0;
    mutable uint64_t fps_last_time_ = 0;

    // ImGui state
    bool imgui_initialized_ = false;
};

} // namespace kf::ui
