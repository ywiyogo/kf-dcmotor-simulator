#pragma once

#include "../core/simulation/SimulationEngine.hpp"
#include "themes/PastelTheme.hpp"
#include <memory>
#ifdef __EMSCRIPTEN__
#include <SDL.h>
#else
#include <SDL2/SDL.h>
#endif

namespace kf::ui {

/**
 * @brief Main UI renderer class for the Kalman Filter application
 *
 * Manages the overall UI layout, handles user interactions, and coordinates
 * between different UI panels and components.
 */
class UIRenderer {
public:
    /**
     * @brief Constructor
     * @param simulation_engine Reference to the simulation engine
     * @param theme Reference to the UI theme
     */
    explicit UIRenderer(simulation::SimulationEngined& simulation_engine,
                       themes::PastelTheme& theme);

    /**
     * @brief Destructor
     */
    ~UIRenderer();

    /**
     * @brief Render the complete UI
     * @param delta_time Time since last frame in seconds
     */
    void render(float delta_time);

    /**
     * @brief Handle window resize
     * @param width New window width
     * @param height New window height
     */
    void on_window_resize(int width, int height);

    /**
     * @brief Check if application should close
     * @return true if close was requested
     */
    bool should_close() const { return should_close_; }

    /**
     * @brief Show/hide help dialog
     * @param show Whether to show the dialog
     */
    void show_help_dialog(bool show) { show_help_ = show; }

private:
    /**
     * @brief Render main menu bar
     */
    void render_menu_bar();

    /**
     * @brief Render main docking space
     */
    void render_main_docking_space();

    /**
     * @brief Setup initial docking layout
     * @param dockspace_id The docking space ID
     */
    void setup_initial_docking_layout(ImGuiID dockspace_id);

    /**
     * @brief Render parameter panel
     */
    void render_parameter_panel();

    /**
     * @brief Render plot panel
     */
    void render_plot_panel();

    /**
     * @brief Render statistics panel
     */
    void render_statistics_panel();

    /**
     * @brief Render configuration panel
     */
    void render_configuration_panel();

    /**
     * @brief Render help/about dialog
     */
    void render_help_dialog();

    /**
     * @brief Update plot animation state
     * @param delta_time Time since last frame in seconds
     */
    void update_plot_animation(float delta_time);

    /**
     * @brief Reset plot animation to beginning
     */
    void reset_plot_animation();

    /**
     * @brief Get filtered data for current animation time
     * @param full_data Complete simulation data
     * @return Data points up to current animation time
     */
    std::vector<simulation::SimulationDataPoint<double>> 
    get_animated_data(const std::vector<simulation::SimulationDataPoint<double>>& full_data);

    simulation::SimulationEngined& simulation_engine_;
    themes::PastelTheme& theme_;

    bool should_close_ = false;
    bool show_help_ = false;
    bool show_demo_window_ = false;

    int window_width_ = 1920;
    int window_height_ = 1080;

    // Plot animation state
    bool plot_animation_enabled_ = true;
    bool plot_is_playing_ = false;
    float plot_playback_time_ = 0.0f;
    float plot_animation_speed_ = 1.0f;
    float plot_max_time_ = 0.0f;
    size_t plot_data_index_ = 0;
    
    // Plot update rate control
    float plot_update_rate_ = 10.0f;  // Hz
    float plot_last_update_time_ = 0.0f;
    float plot_accumulated_time_ = 0.0f;
    
    // Rolling window controls
    bool use_rolling_window_ = false;
    float window_duration_ = 10.0f;  // seconds
};

} // namespace kf::ui
