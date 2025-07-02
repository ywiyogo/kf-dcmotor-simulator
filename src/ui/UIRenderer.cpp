#include "UIRenderer.hpp"
#ifdef __EMSCRIPTEN__
#include <SDL.h>
#else
#include <SDL2/SDL.h>
#endif
#include <imgui.h>
#include <implot.h>
#include <iostream>

namespace kf::ui {

UIRenderer::UIRenderer(simulation::SimulationEngined& simulation_engine,
                      themes::PastelTheme& theme)
    : simulation_engine_(simulation_engine), theme_(theme) {
    // Initialize ImPlot context
    ImPlot::CreateContext();

    std::cout << "UIRenderer initialized successfully\n";
}

UIRenderer::~UIRenderer() {
    // Cleanup ImPlot context
    ImPlot::DestroyContext();
}

void UIRenderer::render(float delta_time) {
    // Update plot animation state
    update_plot_animation(delta_time);

    // Apply theme for this frame
    theme_.apply();

    // Render main menu bar
    render_menu_bar();

    // Render main docking space
    render_main_docking_space();

    // Render dialogs
    if (show_help_) {
        render_help_dialog();
    }

    // Show ImGui demo window if enabled
    if (show_demo_window_) {
        ImGui::ShowDemoWindow(&show_demo_window_);
    }
}

void UIRenderer::on_window_resize(int width, int height) {
    window_width_ = width;
    window_height_ = height;
}

void UIRenderer::render_menu_bar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("New Configuration")) {
                // TODO: Implement new configuration
            }
            if (ImGui::MenuItem("Load Configuration...")) {
                // TODO: Implement load configuration
            }
            if (ImGui::MenuItem("Save Configuration...")) {
                // TODO: Implement save configuration
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Export Data...")) {
                // TODO: Implement data export
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Exit", "ESC")) {
                should_close_ = true;
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Simulation")) {
            if (ImGui::MenuItem("Start", "Space")) {
                auto result = simulation_engine_.start(simulation::SimulationEngine<double>::SimulationMode::REAL_TIME);
                if (!result) {
                    std::cerr << "Failed to start simulation: " << result.error() << std::endl;
                }
            }
            if (ImGui::MenuItem("Stop")) {
                simulation_engine_.stop();
            }
            if (ImGui::MenuItem("Reset", "F5")) {
                simulation_engine_.initialize();
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Pause/Resume")) {
                if (simulation_engine_.is_paused()) {
                    simulation_engine_.resume();
                } else {
                    simulation_engine_.pause();
                }
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Parameters Panel", nullptr, nullptr);
            ImGui::MenuItem("Plot Panel", nullptr, nullptr);
            ImGui::MenuItem("Statistics Panel", nullptr, nullptr);
            ImGui::MenuItem("Configuration Panel", nullptr, nullptr);
            ImGui::Separator();
            ImGui::MenuItem("ImGui Demo", nullptr, &show_demo_window_);
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Help")) {
            if (ImGui::MenuItem("About", "F1")) {
                show_help_ = true;
            }
            if (ImGui::MenuItem("Documentation")) {
                // TODO: Open documentation
            }
            ImGui::EndMenu();
        }

        // Show simulation status in menu bar
        ImGui::Separator();
        if (simulation_engine_.is_running()) {
            ImGui::TextColored(themes::PastelColors::SUCCESS, "RUNNING");
        } else if (simulation_engine_.is_paused()) {
            ImGui::TextColored(themes::PastelColors::WARNING, "PAUSED");
        } else {
            ImGui::TextColored(themes::PastelColors::TEXT_SECONDARY, "STOPPED");
        }

        // Show current time
        ImGui::Text("Time: %.3fs", simulation_engine_.get_current_time());

        // Show progress
        float progress = simulation_engine_.get_progress();
        if (progress > 0.0f) {
            ImGui::Text("Progress: %.1f%%", progress * 100.0f);
        }

        ImGui::EndMainMenuBar();
    }
}

void UIRenderer::render_main_docking_space() {
    // Create full-screen docking space
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::SetNextWindowViewport(viewport->ID);

    ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
    window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse;
    window_flags |= ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

    if (ImGui::Begin("DockSpace", nullptr, window_flags)) {
        ImGui::PopStyleVar(3);

        // Create docking space with enhanced styling
        ImGuiID dockspace_id = ImGui::GetID("MainDockSpace");

        // Setup initial docking layout if needed
        static bool first_time = true;
        if (first_time) {
            first_time = false;
            setup_initial_docking_layout(dockspace_id);
        }

        // Push enhanced docking styles
        ImGui::PushStyleColor(ImGuiCol_Separator, themes::PastelColors::OUTLINE);
        ImGui::PushStyleColor(ImGuiCol_SeparatorHovered, themes::PastelColors::PRIMARY);
        ImGui::PushStyleColor(ImGuiCol_SeparatorActive, themes::PastelColors::PRIMARY_ACCENT);
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4.0f, 4.0f));

        ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_None);

        // Pop the enhanced styles
        ImGui::PopStyleVar(1);
        ImGui::PopStyleColor(3);

        // Render panels with proper docking setup
        render_parameter_panel();
        render_configuration_panel();
        render_plot_panel();
        render_statistics_panel();

    } else {
        ImGui::PopStyleVar(3);
    }
    ImGui::End();
}

void UIRenderer::setup_initial_docking_layout(ImGuiID dockspace_id) {
    (void)dockspace_id; // Suppress unused parameter warning

    // Set up initial window positions and sizes for proper layout
    static bool first_setup = true;
    if (first_setup) {
        first_setup = false;

        ImGuiViewport* viewport = ImGui::GetMainViewport();
        float width = viewport->WorkSize.x;
        float height = viewport->WorkSize.y;
        float menu_height = 22.0f; // Account for menu bar

        // Left sidebar - Parameters panel (top portion)
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x, viewport->WorkPos.y + menu_height), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(width * 0.28f, height * 0.45f), ImGuiCond_FirstUseEver);

        // Left sidebar - Configuration panel (bottom portion)
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x, viewport->WorkPos.y + menu_height + height * 0.45f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(width * 0.28f, height * 0.55f - menu_height), ImGuiCond_FirstUseEver);

        // Central - Plots panel
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + width * 0.28f, viewport->WorkPos.y + menu_height), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(width * 0.72f, height * 0.75f), ImGuiCond_FirstUseEver);

        // Bottom - Statistics panel
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + width * 0.28f, viewport->WorkPos.y + menu_height + height * 0.75f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(width * 0.72f, height * 0.25f - menu_height), ImGuiCond_FirstUseEver);
    }
}

void UIRenderer::render_parameter_panel() {
    // Set initial position for first-time setup
    static bool first_time = true;
    if (first_time) {
        first_time = false;
        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x, viewport->WorkPos.y + 22), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x * 0.28f, viewport->WorkSize.y * 0.45f), ImGuiCond_FirstUseEver);
    }

    ImGuiWindowFlags window_flags = ImGuiWindowFlags_None;
    if (ImGui::Begin("⚙ Parameters", nullptr, window_flags)) {
        // Enhanced panel header with background
        ImGui::PushStyleColor(ImGuiCol_ChildBg, themes::PastelColors::PRIMARY_LIGHT);
        if (ImGui::BeginChild("ParamsHeader", ImVec2(0, 35), true, ImGuiWindowFlags_NoScrollbar)) {
            ImGui::PushStyleColor(ImGuiCol_Text, themes::PastelColors::PRIMARY_DARK);
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 8);
            ImGui::TextUnformatted("⚙ Kalman Filter Parameters");
            ImGui::PopStyleColor();
        }
        ImGui::EndChild();
        ImGui::PopStyleColor();
        ImGui::Spacing();

        auto& config = kf::config::config();
        auto& kalman_params = config.kalman();

        float process_noise = static_cast<float>(kalman_params.process_noise_std);
        if (ImGui::SliderFloat("Process Noise Std", &process_noise, 0.001f, 1.0f, "%.3f")) {
            config.kalman().process_noise_std = process_noise;
            // Update simulation if running
            if (simulation_engine_.is_running()) {
                simulation_engine_.update_configuration();
            }
        }

        float measurement_noise = static_cast<float>(kalman_params.measurement_noise_std);
        if (ImGui::SliderFloat("Measurement Noise Std", &measurement_noise, 0.001f, 1.0f, "%.3f")) {
            config.kalman().measurement_noise_std = measurement_noise;
            if (simulation_engine_.is_running()) {
                simulation_engine_.update_configuration();
            }
        }

        ImGui::Spacing();
        ImGui::Text("DC Motor Parameters");
        ImGui::Separator();

        auto& motor_params = config.motor();
        float voltage_range = static_cast<float>(motor_params.input_voltage_range);
        if (ImGui::SliderFloat("Voltage Range", &voltage_range, 1.0f, 48.0f, "%.1f V")) {
            config.motor().input_voltage_range = voltage_range;
            if (simulation_engine_.is_running()) {
                simulation_engine_.update_configuration();
            }
        }

        ImGui::Spacing();
        ImGui::Text("Simulation Parameters");
        ImGui::Separator();

        auto& sim_params = config.simulation();
        float time_step = static_cast<float>(sim_params.time_step);
        if (ImGui::SliderFloat("Time Step", &time_step, 0.0001f, 0.1f, "%.4f s")) {
            config.simulation().time_step = time_step;
            if (simulation_engine_.is_running()) {
                simulation_engine_.update_configuration();
            }
        }
    }
    ImGui::End();
}

void UIRenderer::render_plot_panel() {
    // Set initial position for first-time setup
    static bool first_time = true;
    if (first_time) {
        first_time = false;
        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.28f, viewport->WorkPos.y + 22), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x * 0.72f, viewport->WorkSize.y * 0.75f), ImGuiCond_FirstUseEver);
    }

    ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar;
    if (ImGui::Begin("📊 Plots", nullptr, window_flags)) {
        // Enhanced menu bar for plot controls
        if (ImGui::BeginMenuBar()) {
            ImGui::PushStyleColor(ImGuiCol_Text, themes::PastelColors::SUCCESS);
            ImGui::TextUnformatted("📊 Real-time Plots");
            ImGui::PopStyleColor();

            ImGui::Separator();

            // Enhanced plot control buttons with styling
            ImGui::PushStyleColor(ImGuiCol_Button, themes::PastelColors::TERTIARY_LIGHT);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, themes::PastelColors::TERTIARY);
            if (ImGui::SmallButton("🔍 Reset Zoom")) {
                // TODO: Implement plot zoom reset
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("💾 Export Data")) {
                // TODO: Implement data export
            }
            ImGui::SameLine();

            // Plot animation controls in menu bar
            const char* play_pause_icon = plot_is_playing_ ? "⏸ Pause" : "▶ Play";
            if (ImGui::SmallButton(play_pause_icon)) {
                plot_is_playing_ = !plot_is_playing_;
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("⏹ Reset")) {
                reset_plot_animation();
            }

            // Animation status in menu bar
            if (plot_animation_enabled_ && plot_max_time_ > 0.0f) {
                ImGui::SameLine();
                ImGui::Text("| %.1fs/%.1fs (%.0f%%)",
                           plot_playback_time_, plot_max_time_,
                           (plot_playback_time_ / plot_max_time_ * 100.0f));
            }

            ImGui::PopStyleColor(2);

            ImGui::EndMenuBar();
        }

        // Get latest simulation data
        auto full_data = simulation_engine_.get_data();
        if (full_data.empty()) {
            ImGui::Spacing();
            ImGui::TextColored(themes::PastelColors::WARNING, "⚠ No data available");
            ImGui::Text("Start simulation to see real-time plots.");
            ImGui::End();
            return;
        }

        // Update plot time bounds
        if (plot_max_time_ == 0.0f || full_data.back().time > plot_max_time_) {
            plot_max_time_ = static_cast<float>(full_data.back().time);
        }

        // Get animated data for progressive visualization
        auto data = plot_animation_enabled_ ? get_animated_data(full_data) : full_data;

        // Apply rolling window to data (controlled from configuration panel)
        if (use_rolling_window_) {
            // Apply rolling window to data
            if (!data.empty() && window_duration_ > 0.0f) {
                double end_time = data.back().time;
                double start_time = end_time - window_duration_;

                auto it = std::lower_bound(data.begin(), data.end(), start_time,
                    [](const auto& point, double time) { return point.time < time; });

                if (it != data.end()) {
                    data.erase(data.begin(), it);
                }
            }
        }

        if (data.empty()) {
            ImGui::Spacing();
            ImGui::TextColored(themes::PastelColors::INFO, "⏳ Waiting for data...");
            ImGui::End();
            return;
        }


        // Prepare data vectors for plotting
        std::vector<double> times, true_positions, estimated_positions, measured_positions;
        std::vector<double> true_velocities, estimated_velocities;
        std::vector<double> position_errors, velocity_errors;

        times.reserve(data.size());
        true_positions.reserve(data.size());
        estimated_positions.reserve(data.size());
        measured_positions.reserve(data.size());
        true_velocities.reserve(data.size());
        estimated_velocities.reserve(data.size());
        position_errors.reserve(data.size());
        velocity_errors.reserve(data.size());

        for (const auto& point : data) {
            times.push_back(point.time);
            true_positions.push_back(point.true_position);
            estimated_positions.push_back(point.estimated_position);
            measured_positions.push_back(point.measured_position);
            true_velocities.push_back(point.true_velocity);
            estimated_velocities.push_back(point.estimated_velocity);
            position_errors.push_back(point.position_error);
            velocity_errors.push_back(point.velocity_error);
        }

        // Position plot with improved clarity
        if (ImPlot::BeginPlot("Position vs Time")) {
            ImPlot::SetupAxes("Time [s]", "Position [rad]");
            ImPlot::SetupAxisLimits(ImAxis_X1, times.front(), times.back(), ImGuiCond_Always);

            // True position - solid blue line
            ImPlot::SetNextLineStyle(themes::PastelColors::PLOT_COLORS[0], 2.0f);
            ImPlot::PlotLine("True Position (Ground Truth)", times.data(), true_positions.data(), times.size());

            // Estimated position - solid orange line
            ImPlot::SetNextLineStyle(themes::PastelColors::PLOT_COLORS[1], 2.0f);
            ImPlot::PlotLine("Kalman Filter Estimate", times.data(), estimated_positions.data(), times.size());

            // Measured position - green scatter points with explicit color
            ImPlot::PushStyleColor(ImPlotCol_MarkerFill, themes::PastelColors::PLOT_COLORS[2]);
            ImPlot::PushStyleColor(ImPlotCol_MarkerOutline, themes::PastelColors::PLOT_COLORS[2]);
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 4.0f, themes::PastelColors::PLOT_COLORS[2], 1.0f, themes::PastelColors::PLOT_COLORS[2]);
            ImPlot::PlotScatter("Noisy Measurements (Sensor Data)", times.data(), measured_positions.data(), times.size());
            ImPlot::PopStyleColor(2);

            ImPlot::EndPlot();
        }

        // Velocity plot
        if (ImPlot::BeginPlot("Velocity vs Time")) {
            ImPlot::SetupAxes("Time [s]", "Velocity [rad/s]");
            ImPlot::SetupAxisLimits(ImAxis_X1, times.front(), times.back(), ImGuiCond_Always);

            ImPlot::SetNextLineStyle(themes::PastelColors::PLOT_COLORS[0]);
            ImPlot::PlotLine("True Velocity", times.data(), true_velocities.data(), times.size());

            ImPlot::SetNextLineStyle(themes::PastelColors::PLOT_COLORS[1]);
            ImPlot::PlotLine("Estimated Velocity", times.data(), estimated_velocities.data(), times.size());

            ImPlot::EndPlot();
        }

        // Measurement noise plot (to clarify what the green dots represent)
        if (ImPlot::BeginPlot("Measurement Noise Analysis")) {
            ImPlot::SetupAxes("Time [s]", "Noise Value [rad]");
            ImPlot::SetupAxisLimits(ImAxis_X1, times.front(), times.back(), ImGuiCond_Always);

            // Calculate actual noise values (measured - true)
            std::vector<double> measurement_noise;
            measurement_noise.reserve(data.size());
            for (const auto& point : data) {
                measurement_noise.push_back(point.measured_position - point.true_position);
            }

            // Plot noise as scatter points
            ImPlot::PushStyleColor(ImPlotCol_MarkerFill, themes::PastelColors::PLOT_COLORS[2]);
            ImPlot::PushStyleColor(ImPlotCol_MarkerOutline, themes::PastelColors::PLOT_COLORS[2]);
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 3.0f, themes::PastelColors::PLOT_COLORS[2], 1.0f, themes::PastelColors::PLOT_COLORS[2]);
            ImPlot::PlotScatter("Measurement Noise", times.data(), measurement_noise.data(), times.size());
            ImPlot::PopStyleColor(2);

            // Add zero reference line
            std::vector<double> zero_line(times.size(), 0.0);
            ImPlot::SetNextLineStyle(ImVec4{0.5f, 0.5f, 0.5f, 0.8f}, 1.0f);
            ImPlot::PlotLine("Zero Reference", times.data(), zero_line.data(), times.size());

            ImPlot::EndPlot();
        }

        // Error plot
        if (ImPlot::BeginPlot("Estimation Errors")) {
            ImPlot::SetupAxes("Time [s]", "Error [rad]");
            ImPlot::SetupAxisLimits(ImAxis_X1, times.front(), times.back(), ImGuiCond_Always);

            ImPlot::SetNextLineStyle(themes::PastelColors::ERROR, 2.0f);
            ImPlot::PlotLine("Position Error", times.data(), position_errors.data(), times.size());

            ImPlot::SetNextLineStyle(themes::PastelColors::WARNING, 2.0f);
            ImPlot::PlotLine("Velocity Error", times.data(), velocity_errors.data(), times.size());

            ImPlot::EndPlot();
        }
    }
    ImGui::End();
}

void UIRenderer::render_statistics_panel() {
    // Set initial position for first-time setup
    static bool first_time = true;
    if (first_time) {
        first_time = false;
        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.28f, viewport->WorkPos.y + 22 + viewport->WorkSize.y * 0.75f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x * 0.72f, viewport->WorkSize.y * 0.25f - 22), ImGuiCond_FirstUseEver);
    }

    ImGuiWindowFlags window_flags = ImGuiWindowFlags_HorizontalScrollbar;
    if (ImGui::Begin("📈 Statistics", nullptr, window_flags)) {
        // Enhanced panel header with background
        ImGui::PushStyleColor(ImGuiCol_ChildBg, themes::PastelColors::INFO);
        if (ImGui::BeginChild("StatsHeader", ImVec2(0, 35), true, ImGuiWindowFlags_NoScrollbar)) {
            ImGui::PushStyleColor(ImGuiCol_Text, themes::PastelColors::TEXT_PRIMARY);
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 8);
            ImGui::TextUnformatted("📈 Performance Statistics");
            ImGui::PopStyleColor();
        }
        ImGui::EndChild();
        ImGui::PopStyleColor();
        ImGui::Spacing();

        const auto& stats = simulation_engine_.get_statistics();

        // Estimation Performance Section
        ImGui::TextColored(themes::PastelColors::PRIMARY, "🎯 Estimation Performance");
        ImGui::Indent();

        ImGui::Text("Position RMSE: %.6f rad", stats.rmse_position);
        ImGui::Text("Velocity RMSE: %.6f rad/s", stats.rmse_velocity);
        ImGui::Text("Position MAE: %.6f rad", stats.mean_absolute_error_position);
        ImGui::Text("Velocity MAE: %.6f rad/s", stats.mean_absolute_error_velocity);

        ImGui::Spacing();
        ImGui::Text("Filter Status");
        ImGui::Separator();

        ImGui::Text("Converged: %s", stats.filter_converged ? "Yes" : "No");
        ImGui::Text("Stable: %s", stats.filter_stable ? "Yes" : "No");
        ImGui::Text("Convergence Time: %.3f s", stats.convergence_time);
        ImGui::Text("Innovation Whiteness: %.4f", stats.innovation_whiteness_metric);

        ImGui::Spacing();
        ImGui::Text("Motor Performance");
        ImGui::Separator();

        ImGui::Text("Average Efficiency: %.2f%%", stats.average_efficiency * 100.0);
        ImGui::Text("Peak Current: %.3f A", stats.peak_current);
        ImGui::Text("Energy Consumed: %.3f J", stats.total_energy_consumed);
        ImGui::Text("Mechanical Work: %.3f J", stats.total_mechanical_work);

        ImGui::Spacing();
        ImGui::Text("Computational Performance");
        ImGui::Separator();

        ImGui::Text("Total Updates: %zu", stats.total_updates);
        ImGui::Text("Failed Updates: %zu", stats.failed_updates);
        ImGui::Text("Average Update Time: %ld μs", stats.average_update_time.count());

        if (stats.total_updates > 0) {
            float success_rate = 100.0f * (1.0f - static_cast<float>(stats.failed_updates) / static_cast<float>(stats.total_updates));
            ImGui::Text("Success Rate: %.1f%%", success_rate);
        }
        ImGui::Unindent();
    }
    ImGui::End();
}

void UIRenderer::render_configuration_panel() {
    // Set initial position for first-time setup
    static bool first_time = true;
    if (first_time) {
        first_time = false;
        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x, viewport->WorkPos.y + 22 + viewport->WorkSize.y * 0.45f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x * 0.28f, viewport->WorkSize.y * 0.55f - 22), ImGuiCond_FirstUseEver);
    }

    ImGuiWindowFlags window_flags = ImGuiWindowFlags_None;
    if (ImGui::Begin("⚙ Configuration", nullptr, window_flags)) {
        // Enhanced panel header with background
        ImGui::PushStyleColor(ImGuiCol_ChildBg, themes::PastelColors::SECONDARY_LIGHT);
        if (ImGui::BeginChild("ConfigHeader", ImVec2(0, 35), true, ImGuiWindowFlags_NoScrollbar)) {
            ImGui::PushStyleColor(ImGuiCol_Text, themes::PastelColors::SECONDARY_DARK);
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 8);
            ImGui::TextUnformatted("⚙ System Configuration");
            ImGui::PopStyleColor();
        }
        ImGui::EndChild();
        ImGui::PopStyleColor();
        ImGui::Spacing();

        // === SIMULATION CONTROL SECTION ===
        ImGui::PushStyleColor(ImGuiCol_Text, themes::PastelColors::PRIMARY_DARK);
        ImGui::Text("🎮 SIMULATION CONTROL");
        ImGui::PopStyleColor();
        ImGui::Separator();
        ImGui::Spacing();

        // Simulation Mode Selection
        ImGui::PushStyleColor(ImGuiCol_ChildBg, themes::PastelColors::SURFACE);
        if (ImGui::BeginChild("SimMode", ImVec2(0, 100), true)) {
            ImGui::TextColored(themes::PastelColors::PRIMARY, "⚙ Simulation Mode");
            ImGui::Separator();
            ImGui::Spacing();

            static int simulation_mode = 1;  // Default to REAL_TIME for better visualization
            const char* mode_names[] = { "⚡ Batch (Fast)", "⏱ Real-Time", "🎯 Interactive" };

            if (ImGui::Combo("Mode", &simulation_mode, mode_names, 3)) {
                // Mode changed - could update config or store for next simulation
            }

            ImGui::PushStyleColor(ImGuiCol_Button, themes::PastelColors::SUCCESS);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, themes::PastelColors::TERTIARY);
            if (ImGui::Button("▶ Start with Selected Mode", ImVec2(-1, 0))) {
                simulation::SimulationEngine<double>::SimulationMode mode;
                switch(simulation_mode) {
                    case 0: mode = simulation::SimulationEngine<double>::SimulationMode::BATCH; break;
                    case 1: mode = simulation::SimulationEngine<double>::SimulationMode::REAL_TIME; break;
                    case 2: mode = simulation::SimulationEngine<double>::SimulationMode::INTERACTIVE; break;
                    default: mode = simulation::SimulationEngine<double>::SimulationMode::REAL_TIME; break;
                }
                auto result = simulation_engine_.start(mode);
                if (!result) {
                    std::cerr << "Failed to start simulation: " << result.error() << std::endl;
                }
            }
            ImGui::PopStyleColor(2);
        }
        ImGui::EndChild();
        ImGui::PopStyleColor();
        ImGui::Spacing();

        // === VISUALIZATION CONTROL SECTION ===
        ImGui::PushStyleColor(ImGuiCol_Text, themes::PastelColors::PRIMARY_DARK);
        ImGui::Text("📊 PLOT VISUALIZATION");
        ImGui::PopStyleColor();
        ImGui::Separator();
        ImGui::Spacing();

        // Plot Animation Controls
        ImGui::PushStyleColor(ImGuiCol_ChildBg, themes::PastelColors::SURFACE);
        if (ImGui::BeginChild("PlotAnimation", ImVec2(0, 130), true)) {
            ImGui::TextColored(themes::PastelColors::PRIMARY, "🎬 Animation Controls");
            ImGui::Separator();
            ImGui::Spacing();

            ImGui::Checkbox("Enable Animation", &plot_animation_enabled_);
            ImGui::SameLine();
            if (ImGui::SmallButton(plot_is_playing_ ? "⏸ Pause" : "▶ Play")) {
                plot_is_playing_ = !plot_is_playing_;
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("⏹ Reset")) {
                reset_plot_animation();
            }

            if (plot_animation_enabled_) {
                ImGui::SliderFloat("Playback Time", &plot_playback_time_, 0.0f, plot_max_time_, "%.2f s");
                ImGui::SliderFloat("Speed", &plot_animation_speed_, 0.1f, 5.0f, "%.1fx");
                ImGui::SliderFloat("Update Rate", &plot_update_rate_, 1.0f, 60.0f, "%.1f Hz");
            }
        }
        ImGui::EndChild();
        ImGui::PopStyleColor();
        ImGui::Spacing();

        // Rolling Window Controls
        ImGui::PushStyleColor(ImGuiCol_ChildBg, themes::PastelColors::SURFACE);
        if (ImGui::BeginChild("RollingWindow", ImVec2(0, 70), true)) {
            ImGui::TextColored(themes::PastelColors::PRIMARY, "🔄 Rolling Window");
            ImGui::Separator();
            ImGui::Spacing();

            ImGui::Checkbox("Enable Rolling Window", &use_rolling_window_);
            if (use_rolling_window_) {
                ImGui::SliderFloat("Duration", &window_duration_, 1.0f, 60.0f, "%.1f s");
            }
        }
        ImGui::EndChild();
        ImGui::PopStyleColor();
        ImGui::Spacing();

        // === SYSTEM CONFIGURATION SECTION ===
        ImGui::PushStyleColor(ImGuiCol_Text, themes::PastelColors::PRIMARY_DARK);
        ImGui::Text("⚙ SYSTEM CONFIGURATION");
        ImGui::PopStyleColor();
        ImGui::Separator();
        ImGui::Spacing();

        // Configuration Management Section with enhanced styling
        ImGui::PushStyleColor(ImGuiCol_ChildBg, themes::PastelColors::SURFACE);
        if (ImGui::BeginChild("ConfigMgmt", ImVec2(0, 80), true)) {
            ImGui::TextColored(themes::PastelColors::PRIMARY, "💾 Configuration Management");
            ImGui::Separator();
            ImGui::Spacing();

            // Enhanced buttons with consistent styling
            ImGui::PushStyleColor(ImGuiCol_Button, themes::PastelColors::PRIMARY);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, themes::PastelColors::PRIMARY_DARK);
            if (ImGui::Button("🔄 Load Default", ImVec2(-1, 0))) {
                kf::config::config().reset_to_defaults();
                simulation_engine_.initialize();
            }
            ImGui::PopStyleColor(2);

            ImGui::PushStyleColor(ImGuiCol_Button, themes::PastelColors::SUCCESS);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, themes::PastelColors::TERTIARY);
            if (ImGui::Button("💾 Save Config", ImVec2(-1, 0))) {
                auto result = kf::config::config().save_to_file("kalman_filter_config.ini");
                if (!result) {
                    std::cerr << "Failed to save configuration: " << result.error() << std::endl;
                }
            }
            ImGui::PopStyleColor(2);

            ImGui::PushStyleColor(ImGuiCol_Button, themes::PastelColors::INFO);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, themes::PastelColors::PRIMARY_ACCENT);
            if (ImGui::Button("📁 Load Config", ImVec2(-1, 0))) {
                auto result = kf::config::config().load_from_file("kalman_filter_config.ini");
                if (!result) {
                    std::cerr << "Failed to load configuration: " << result.error() << std::endl;
                } else {
                    simulation_engine_.update_configuration();
                }
            }
            ImGui::PopStyleColor(2);
        }
        ImGui::EndChild();
        ImGui::PopStyleColor();

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Input profile selection
        ImGui::TextColored(themes::PastelColors::PRIMARY, "🎛 Input Profile Selection");
        ImGui::Indent();
        static int current_profile = 0;
        const char* profile_names[] = {"Step", "Ramp", "Sinusoidal", "Chirp", "PRBS"};

        if (ImGui::Combo("Profile", &current_profile, profile_names, IM_ARRAYSIZE(profile_names))) {
            auto profile = static_cast<simulation::SimulationEngined::InputProfile>(current_profile);
            simulation_engine_.set_input_profile(profile);
        }
        ImGui::Unindent();

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Theme selection
        ImGui::TextColored(themes::PastelColors::PRIMARY, "🎨 Theme Selection");
        ImGui::Indent();
        static int current_theme = 0;
        const char* theme_names[] = {"Light", "Soft", "Vibrant", "Contrast"};

        if (ImGui::Combo("Theme Variant", &current_theme, theme_names, IM_ARRAYSIZE(theme_names))) {
            auto variant = static_cast<themes::PastelTheme::Variant>(current_theme);
            theme_.set_variant(variant);
        }
        ImGui::Unindent();
    }
    ImGui::End();
}

void UIRenderer::update_plot_animation(float delta_time) {
    plot_accumulated_time_ += delta_time;

    // Control plot update rate
    float update_interval = 1.0f / plot_update_rate_;
    if (plot_accumulated_time_ >= update_interval) {
        plot_accumulated_time_ = 0.0f;

        if (plot_animation_enabled_ && plot_is_playing_ && plot_max_time_ > 0.0f) {
            plot_playback_time_ += update_interval * plot_animation_speed_;

            // Stop animation when it reaches the end
            if (plot_playback_time_ > plot_max_time_) {
                plot_playback_time_ = plot_max_time_;
                plot_is_playing_ = false;
            }
        }
    }
}

void UIRenderer::reset_plot_animation() {
    plot_playback_time_ = 0.0f;
    plot_is_playing_ = false;
    plot_data_index_ = 0;
}

std::vector<simulation::SimulationDataPoint<double>>
UIRenderer::get_animated_data(const std::vector<simulation::SimulationDataPoint<double>>& full_data) {
    if (!plot_animation_enabled_ || full_data.empty()) {
        return full_data;
    }

    std::vector<simulation::SimulationDataPoint<double>> animated_data;
    animated_data.reserve(full_data.size());

    // Include all data points up to the current playback time
    for (const auto& point : full_data) {
        if (point.time <= plot_playback_time_) {
            animated_data.push_back(point);
        } else {
            break;
        }
    }

    return animated_data;
}



void UIRenderer::render_help_dialog() {
    if (ImGui::Begin("About Kalman Filter Simulation", &show_help_)) {
        ImGui::Text("Kalman Filter DC Motor Simulation");
        ImGui::Text("Modern C++23 Implementation");
        ImGui::Separator();

        ImGui::Text("This application demonstrates Kalman filtering for");
        ImGui::Text("DC motor state estimation using a physics-based model.");
        ImGui::Spacing();

        ImGui::Text("Features:");
        ImGui::BulletText("Real-time state estimation visualization");
        ImGui::BulletText("Interactive parameter tuning");
        ImGui::BulletText("Performance analytics and metrics");
        ImGui::BulletText("Multiple input signal profiles");
        ImGui::BulletText("Modern pastel UI theme");
        ImGui::BulletText("Parallel execution support");

        ImGui::Spacing();
        ImGui::Text("Controls:");
        ImGui::BulletText("F1 - Show this help dialog");
        ImGui::BulletText("F5 - Reset simulation");
        ImGui::BulletText("F11 - Toggle fullscreen");
        ImGui::BulletText("ESC - Exit application");
        ImGui::BulletText("Space - Start/Stop simulation");

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Built with modern C++23, ImGui, and SDL2");
    }
    ImGui::End();
}

} // namespace kf::ui
