#include "Configuration.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <ranges>
#include <cmath>

namespace kf::config {

std::expected<void, std::string> Configuration::validate() const noexcept {
    try {
        // Validate simulation configuration
        if (!is_positive(simulation_config_.time_step)) {
            return std::unexpected("Time step must be positive");
        }
        if (!is_positive(simulation_config_.simulation_duration)) {
            return std::unexpected("Simulation duration must be positive");
        }
        if (simulation_config_.max_data_points == 0) {
            return std::unexpected("Max data points must be greater than zero");
        }

        // Validate Kalman filter configuration
        if (!is_positive(kalman_config_.process_noise_std)) {
            return std::unexpected("Process noise standard deviation must be positive");
        }
        if (!is_positive(kalman_config_.measurement_noise_std)) {
            return std::unexpected("Measurement noise standard deviation must be positive");
        }

        // Validate initial covariance matrix (must be positive definite)
        const auto& P = kalman_config_.initial_covariance;
        if (P[0][0] <= 0.0 || P[1][1] <= 0.0) {
            return std::unexpected("Initial covariance diagonal elements must be positive");
        }
        
        // Check if covariance matrix is positive definite
        double det = P[0][0] * P[1][1] - P[0][1] * P[1][0];
        if (det <= 0.0) {
            return std::unexpected("Initial covariance matrix must be positive definite");
        }

        // Validate DC motor parameters
        if (!is_positive(motor_config_.torque_sensitivity)) {
            return std::unexpected("Torque sensitivity must be positive");
        }
        if (!is_positive(motor_config_.back_emf_constant)) {
            return std::unexpected("Back EMF constant must be positive");
        }
        if (!is_positive(motor_config_.terminal_resistance)) {
            return std::unexpected("Terminal resistance must be positive");
        }
        if (!is_positive(motor_config_.terminal_inductance)) {
            return std::unexpected("Terminal inductance must be positive");
        }
        if (!is_positive(motor_config_.rotational_inertia)) {
            return std::unexpected("Rotational inertia must be positive");
        }
        if (!is_positive(motor_config_.input_voltage_range)) {
            return std::unexpected("Input voltage range must be positive");
        }

        // Validate UI configuration
        auto validate_color = [](const std::array<float, 4>& color, const std::string& name) -> std::expected<void, std::string> {
            for (std::size_t i = 0; i < 4; ++i) {
                if (!is_in_range(color[i], 0.0f, 1.0f)) {
                    return std::unexpected(name + " color components must be between 0.0 and 1.0");
                }
            }
            return {};
        };

        if (auto result = validate_color(ui_config_.primary_color, "Primary"); !result) {
            return result;
        }
        if (auto result = validate_color(ui_config_.secondary_color, "Secondary"); !result) {
            return result;
        }
        if (auto result = validate_color(ui_config_.accent_color, "Accent"); !result) {
            return result;
        }
        if (auto result = validate_color(ui_config_.background_color, "Background"); !result) {
            return result;
        }

        if (!is_positive(ui_config_.plot_refresh_rate)) {
            return std::unexpected("Plot refresh rate must be positive");
        }
        if (ui_config_.plot_history_size == 0) {
            return std::unexpected("Plot history size must be greater than zero");
        }
        if (!is_positive(ui_config_.animation_speed)) {
            return std::unexpected("Animation speed must be positive");
        }

        return {};
    } catch (const std::exception& e) {
        return std::unexpected(std::string("Validation error: ") + e.what());
    }
}

std::expected<void, std::string> Configuration::save_to_file(const std::string& filename) const {
    try {
        std::ofstream file(filename);
        if (!file.is_open()) {
            return std::unexpected("Cannot open file for writing: " + filename);
        }

        // Write configuration in a simple key-value format
        file << "# Kalman Filter Simulation Configuration\n";
        file << "# Generated automatically - modify with care\n\n";

        // Simulation parameters
        file << "[Simulation]\n";
        file << "time_step=" << simulation_config_.time_step << "\n";
        file << "simulation_duration=" << simulation_config_.simulation_duration << "\n";
        file << "max_data_points=" << simulation_config_.max_data_points << "\n";
        file << "real_time_mode=" << (simulation_config_.real_time_mode ? "true" : "false") << "\n\n";

        // Kalman filter parameters
        file << "[Kalman]\n";
        file << "process_noise_std=" << kalman_config_.process_noise_std << "\n";
        file << "measurement_noise_std=" << kalman_config_.measurement_noise_std << "\n";
        file << "initial_state_0=" << kalman_config_.initial_state[0] << "\n";
        file << "initial_state_1=" << kalman_config_.initial_state[1] << "\n";
        file << "initial_cov_00=" << kalman_config_.initial_covariance[0][0] << "\n";
        file << "initial_cov_01=" << kalman_config_.initial_covariance[0][1] << "\n";
        file << "initial_cov_10=" << kalman_config_.initial_covariance[1][0] << "\n";
        file << "initial_cov_11=" << kalman_config_.initial_covariance[1][1] << "\n\n";

        // DC motor parameters
        file << "[Motor]\n";
        file << "torque_sensitivity=" << motor_config_.torque_sensitivity << "\n";
        file << "back_emf_constant=" << motor_config_.back_emf_constant << "\n";
        file << "terminal_resistance=" << motor_config_.terminal_resistance << "\n";
        file << "terminal_inductance=" << motor_config_.terminal_inductance << "\n";
        file << "damping_factor=" << motor_config_.damping_factor << "\n";
        file << "rotational_inertia=" << motor_config_.rotational_inertia << "\n";
        file << "input_voltage_range=" << motor_config_.input_voltage_range << "\n\n";

        // UI parameters
        file << "[UI]\n";
        file << "primary_color=" << ui_config_.primary_color[0] << "," 
             << ui_config_.primary_color[1] << "," 
             << ui_config_.primary_color[2] << "," 
             << ui_config_.primary_color[3] << "\n";
        file << "secondary_color=" << ui_config_.secondary_color[0] << "," 
             << ui_config_.secondary_color[1] << "," 
             << ui_config_.secondary_color[2] << "," 
             << ui_config_.secondary_color[3] << "\n";
        file << "accent_color=" << ui_config_.accent_color[0] << "," 
             << ui_config_.accent_color[1] << "," 
             << ui_config_.accent_color[2] << "," 
             << ui_config_.accent_color[3] << "\n";
        file << "background_color=" << ui_config_.background_color[0] << "," 
             << ui_config_.background_color[1] << "," 
             << ui_config_.background_color[2] << "," 
             << ui_config_.background_color[3] << "\n";
        file << "plot_refresh_rate=" << ui_config_.plot_refresh_rate << "\n";
        file << "plot_history_size=" << ui_config_.plot_history_size << "\n";
        file << "enable_animations=" << (ui_config_.enable_animations ? "true" : "false") << "\n";
        file << "animation_speed=" << ui_config_.animation_speed << "\n";

        file.close();
        return {};
    } catch (const std::exception& e) {
        return std::unexpected(std::string("Error saving configuration: ") + e.what());
    }
}

std::expected<void, std::string> Configuration::load_from_file(const std::string& filename) {
    try {
        std::ifstream file(filename);
        if (!file.is_open()) {
            return std::unexpected("Cannot open configuration file: " + filename);
        }

        std::string line;
        std::string current_section;
        
        while (std::getline(file, line)) {
            // Skip empty lines and comments
            if (line.empty() || line[0] == '#') {
                continue;
            }

            // Check for section headers
            if (line[0] == '[' && line.back() == ']') {
                current_section = line.substr(1, line.length() - 2);
                continue;
            }

            // Parse key-value pairs
            auto eq_pos = line.find('=');
            if (eq_pos == std::string::npos) {
                continue;
            }

            std::string key = line.substr(0, eq_pos);
            std::string value = line.substr(eq_pos + 1);

            // Trim whitespace
            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);

            // Parse based on section
            if (current_section == "Simulation") {
                if (key == "time_step") {
                    simulation_config_.time_step = std::stod(value);
                } else if (key == "simulation_duration") {
                    simulation_config_.simulation_duration = std::stod(value);
                } else if (key == "max_data_points") {
                    simulation_config_.max_data_points = std::stoull(value);
                } else if (key == "real_time_mode") {
                    simulation_config_.real_time_mode = (value == "true");
                }
            } else if (current_section == "Kalman") {
                if (key == "process_noise_std") {
                    kalman_config_.process_noise_std = std::stod(value);
                } else if (key == "measurement_noise_std") {
                    kalman_config_.measurement_noise_std = std::stod(value);
                } else if (key == "initial_state_0") {
                    kalman_config_.initial_state[0] = std::stod(value);
                } else if (key == "initial_state_1") {
                    kalman_config_.initial_state[1] = std::stod(value);
                } else if (key == "initial_cov_00") {
                    kalman_config_.initial_covariance[0][0] = std::stod(value);
                } else if (key == "initial_cov_01") {
                    kalman_config_.initial_covariance[0][1] = std::stod(value);
                } else if (key == "initial_cov_10") {
                    kalman_config_.initial_covariance[1][0] = std::stod(value);
                } else if (key == "initial_cov_11") {
                    kalman_config_.initial_covariance[1][1] = std::stod(value);
                }
            } else if (current_section == "Motor") {
                if (key == "torque_sensitivity") {
                    motor_config_.torque_sensitivity = std::stod(value);
                } else if (key == "back_emf_constant") {
                    motor_config_.back_emf_constant = std::stod(value);
                } else if (key == "terminal_resistance") {
                    motor_config_.terminal_resistance = std::stod(value);
                } else if (key == "terminal_inductance") {
                    motor_config_.terminal_inductance = std::stod(value);
                } else if (key == "damping_factor") {
                    motor_config_.damping_factor = std::stod(value);
                } else if (key == "rotational_inertia") {
                    motor_config_.rotational_inertia = std::stod(value);
                } else if (key == "input_voltage_range") {
                    motor_config_.input_voltage_range = std::stod(value);
                }
            } else if (current_section == "UI") {
                auto parse_color = [](const std::string& color_str) -> std::array<float, 4> {
                    std::array<float, 4> color{};
                    std::stringstream ss(color_str);
                    std::string component;
                    std::size_t i = 0;
                    
                    while (std::getline(ss, component, ',') && i < 4) {
                        color[i++] = std::stof(component);
                    }
                    return color;
                };

                if (key == "primary_color") {
                    ui_config_.primary_color = parse_color(value);
                } else if (key == "secondary_color") {
                    ui_config_.secondary_color = parse_color(value);
                } else if (key == "accent_color") {
                    ui_config_.accent_color = parse_color(value);
                } else if (key == "background_color") {
                    ui_config_.background_color = parse_color(value);
                } else if (key == "plot_refresh_rate") {
                    ui_config_.plot_refresh_rate = std::stof(value);
                } else if (key == "plot_history_size") {
                    ui_config_.plot_history_size = std::stoull(value);
                } else if (key == "enable_animations") {
                    ui_config_.enable_animations = (value == "true");
                } else if (key == "animation_speed") {
                    ui_config_.animation_speed = std::stof(value);
                }
            }
        }

        file.close();
        
        // Validate loaded configuration
        if (auto result = validate(); !result) {
            return std::unexpected("Invalid configuration loaded: " + result.error());
        }

        return {};
    } catch (const std::exception& e) {
        return std::unexpected(std::string("Error loading configuration: ") + e.what());
    }
}

void Configuration::reset_to_defaults() noexcept {
    simulation_config_ = SimulationConfig{};
    kalman_config_ = KalmanConfig{};
    motor_config_ = DCMotorConfig{};
    ui_config_ = UiConfig{};
}

void Configuration::notify_change(const std::string& parameter_name) const {
    auto it = change_callbacks_.find(parameter_name);
    if (it != change_callbacks_.end() && it->second) {
        try {
            it->second();
        } catch (const std::exception& e) {
            std::cerr << "Error in change callback for " << parameter_name << ": " << e.what() << std::endl;
        }
    }
}

} // namespace kf::config