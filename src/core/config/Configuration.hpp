#pragma once

#include <array>
#include <chrono>
#include <concepts>
#include <expected>
#include <string>
#include <unordered_map>
#include <functional>

namespace kf::config {

/**
 * @file Configuration.hpp
 * @brief Centralized configuration management system for Kalman Filter simulation
 *
 * This file implements a singleton-based configuration system that eliminates magic numbers
 * and provides type-safe parameter management for the entire simulation system.
 *
 * @author Yongkie Wiyogo
 * @version 1.0.0
 * @date 2025
 *
 * Key Design Principles:
 * - Single Source of Truth: All parameters defined in one place
 * - Type Safety: Strong typing prevents configuration errors
 * - Validation: Built-in parameter validation and range checking
 * - Persistence: Save/load configuration from files
 * - Runtime Updates: Support for live parameter adjustment
 * - Thread Safety: Safe for multi-threaded access patterns
 *
 * Architecture:
 * The Configuration class uses the singleton pattern to ensure global access
 * while maintaining single instance semantics. It's designed to be the foundation
 * layer of the application architecture, providing stable interfaces for all
 * higher-level components.
 */

/**
 * @brief Main configuration manager for Kalman Filter simulation
 *
 * This class centralizes all configuration parameters to avoid magic numbers
 * and provides a single source of truth for simulation parameters. It implements
 * the singleton pattern to ensure consistent configuration across the application.
 *
 * The configuration is divided into logical groups:
 * - SimulationConfig: Core simulation parameters (time steps, duration, etc.)
 * - KalmanConfig: Kalman filter specific parameters (noise levels, initial conditions)
 * - DCMotorConfig: Physical motor parameters (based on real hardware specs)
 * - UiConfig: User interface settings (colors, refresh rates, themes)
 *
 * Features:
 * - Type-safe parameter access with compile-time checking
 * - Runtime parameter validation with detailed error reporting
 * - Persistent configuration storage (INI file format)
 * - Change notification system for reactive updates
 * - Default value management with factory reset capability
 *
 * Thread Safety:
 * The singleton implementation is thread-safe for initialization.
 * Parameter access is read-heavy and designed to be fast.
 * Modifications should be done from the main thread for UI responsiveness.
 *
 * Usage Example:
 * @code
 * auto& config = kf::config::config();
 * config.kalman().process_noise_std = 0.1;
 * config.simulation().time_step = 0.01;
 *
 * auto result = config.validate();
 * if (result) {
 *     auto save_result = config.save_to_file("my_config.ini");
 * }
 * @endcode
 */
class Configuration {
public:
    // ============================================================================
    // Configuration Group Structures
    // ============================================================================

    /**
     * @brief Simulation engine configuration parameters
     *
     * Controls the core simulation behavior including timing, data management,
     * and execution modes. These parameters directly affect simulation accuracy
     * and performance characteristics.
     *
     * Parameter Guidelines:
     * - time_step: Smaller values increase accuracy but reduce performance
     * - simulation_duration: Total time for batch simulations
     * - max_data_points: Memory usage vs. data resolution trade-off
     * - real_time_mode: Enables wall-clock time synchronization
     */
    struct SimulationConfig {
        /** @brief Integration time step in seconds (Δt)
         *  Range: [1e-6, 1.0]
         *  Default: 0.01s (100 Hz update rate)
         *  Impact: Smaller values improve accuracy but increase computation */
        double time_step = 0.01;

        /** @brief Total simulation duration in seconds
         *  Range: [0.1, 3600.0]
         *  Default: 10.0s
         *  Impact: Determines length of batch simulations */
        double simulation_duration = 10.0;

        /** @brief Maximum number of data points to store in memory
         *  Range: [100, 1000000]
         *  Default: 1000 points
         *  Impact: Memory usage vs. data resolution trade-off */
        std::size_t max_data_points = 1000;

        /** @brief Enable real-time simulation mode
         *  Default: true
         *  Impact: When true, simulation runs at wall-clock time
         *          When false, simulation runs as fast as possible */
        bool real_time_mode = true;
    };

    /**
     * @brief Kalman filter algorithm configuration parameters
     *
     * These parameters control the Kalman filter's estimation behavior and
     * directly impact the quality of state estimation. The noise parameters
     * should be tuned based on the actual system characteristics.
     *
     * Tuning Guidelines:
     * - process_noise_std: Models uncertainty in system dynamics
     * - measurement_noise_std: Models sensor noise and quantization
     * - initial_state: Starting point for state estimation
     * - initial_covariance: Initial uncertainty in state estimate
     */
    struct KalmanConfig {
        /** @brief Process noise standard deviation (σ_a)
         *  Range: [1e-6, 10.0]
         *  Default: 0.1
         *  Physical meaning: Uncertainty in acceleration (rad/s²)
         *  Impact: Higher values make filter more responsive but less smooth */
        double process_noise_std = 0.1;

        /** @brief Measurement noise standard deviation (σ_t)
         *  Range: [1e-6, 10.0]
         *  Default: 0.2
         *  Physical meaning: Sensor noise level (radians)
         *  Impact: Should match actual sensor characteristics */
        double measurement_noise_std = 0.2;

        /** @brief Initial state estimate [position, velocity]
         *  Units: [radians, rad/s]
         *  Default: [0.0, 0.0] (at rest)
         *  Impact: Starting point for estimation - usually zero */
        std::array<double, 2> initial_state = {0.0, 0.0};

        /** @brief Initial covariance matrix (2x2)
         *  Units: [rad², rad²/s²; rad²/s², rad²/s²]
         *  Default: Identity matrix
         *  Impact: Initial uncertainty - larger values mean less confidence */
        std::array<std::array<double, 2>, 2> initial_covariance = {{
            {{1.0, 0.0}},
            {{0.0, 1.0}}
        }};
    };

    /**
     * @brief DC motor physical parameters configuration
     *
     * Based on Moog C23-L33-W10 datasheet specifications. These parameters
     * define the physical characteristics of the DC motor being simulated.
     *
     * Hardware Reference:
     * - Motor: Moog C23-L33-W10 brushless DC motor
     * - Datasheet: Available from Moog Inc.
     * - Application: Precision motion control systems
     *
     * Load Assumptions:
     * - Disk radius: 5cm
     * - Disk mass: 0.1kg (aluminum disk)
     * - Moment of inertia: J = 0.5 * m * r² = 0.000125 kg⋅m²
     */
    struct DCMotorConfig {
        /** @brief Torque sensitivity constant (kt)
         *  Value: 0.0187 Nm/A (from Moog datasheet)
         *  Physical meaning: Torque produced per amp of current
         *  Impact: Affects motor response characteristics */
        double torque_sensitivity = 0.0187;

        /** @brief Back EMF constant (ke)
         *  Value: 0.0191 V/(rad/s) (from Moog datasheet)
         *  Physical meaning: Voltage generated per unit angular velocity
         *  Impact: Affects motor dynamics and braking behavior */
        double back_emf_constant = 0.0191;

        /** @brief Terminal resistance (R)
         *  Value: 0.6 Ω (from Moog datasheet)
         *  Physical meaning: Electrical resistance of motor windings
         *  Impact: Affects current dynamics and power consumption */
        double terminal_resistance = 0.6;

        /** @brief Terminal inductance (L)
         *  Value: 0.035 H (from Moog datasheet)
         *  Physical meaning: Electrical inductance of motor windings
         *  Impact: Affects current rise time and high-frequency response */
        double terminal_inductance = 0.035;

        /** @brief Damping factor (b)
         *  Value: 0.0000095 Nm/(rad/s) (calculated from datasheet)
         *  Physical meaning: Mechanical friction and damping
         *  Impact: Affects steady-state behavior and oscillations */
        double damping_factor = 0.0000095;

        /** @brief Rotational inertia (J)
         *  Value: 0.000125 kg⋅m² (calculated for 5cm disk, 0.1kg)
         *  Physical meaning: Moment of inertia of rotor + load
         *  Impact: Affects acceleration response and dynamic behavior */
        double rotational_inertia = 0.000125;

        /** @brief Maximum input voltage range
         *  Range: [1.0, 48.0] V
         *  Default: 24.0 V
         *  Impact: Limits maximum torque and speed capability */
        double input_voltage_range = 24.0;
    };

    /**
     * @brief User interface configuration parameters
     *
     * Controls the appearance and behavior of the ImGui-based user interface.
     * Supports modern pastel color schemes and responsive design principles.
     *
     * Color System:
     * - All colors are in RGBA format with values [0.0, 1.0]
     * - Pastel colors provide gentle, eye-friendly interface
     * - High contrast options available for accessibility
     *
     * Performance Settings:
     * - plot_refresh_rate: Higher values provide smoother animation
     * - plot_history_size: More points provide better detail but use more memory
     */
    struct UiConfig {
        /** @brief Primary UI color (pastel blue)
         *  RGBA: [0.7, 0.8, 0.9, 1.0]
         *  Usage: Main UI elements, buttons, headers */
        std::array<float, 4> primary_color = {0.7f, 0.8f, 0.9f, 1.0f};

        /** @brief Secondary UI color (pastel orange)
         *  RGBA: [0.9, 0.8, 0.7, 1.0]
         *  Usage: Secondary buttons, highlights, accents */
        std::array<float, 4> secondary_color = {0.9f, 0.8f, 0.7f, 1.0f};

        /** @brief Accent color (pastel green)
         *  RGBA: [0.8, 0.9, 0.7, 1.0]
         *  Usage: Success indicators, active states */
        std::array<float, 4> accent_color = {0.8f, 0.9f, 0.7f, 1.0f};

        /** @brief Background color (light grey)
         *  RGBA: [0.95, 0.95, 0.97, 1.0]
         *  Usage: Window backgrounds, panels */
        std::array<float, 4> background_color = {0.95f, 0.95f, 0.97f, 1.0f};

        /** @brief Plot refresh rate in Hz
         *  Range: [1.0, 120.0]
         *  Default: 60.0 Hz
         *  Impact: Higher values provide smoother plots but use more CPU */
        float plot_refresh_rate = 60.0f;

        /** @brief Number of points to display in plots
         *  Range: [100, 10000]
         *  Default: 500 points
         *  Impact: More points show more history but use more memory */
        std::size_t plot_history_size = 500;

        /** @brief Enable UI animations
         *  Default: true
         *  Impact: Smoother transitions but slightly more GPU usage */
        bool enable_animations = true;

        /** @brief Animation speed multiplier
         *  Range: [0.1, 5.0]
         *  Default: 1.0 (normal speed)
         *  Impact: Controls speed of UI transitions and effects */
        float animation_speed = 1.0f;
    };

    // ============================================================================
    // Singleton Pattern Implementation
    // ============================================================================

    /**
     * @brief Get the singleton configuration instance
     *
     * Implements the Meyers' singleton pattern for thread-safe lazy initialization.
     * The instance is created on first access and guaranteed to be destroyed
     * at program termination.
     *
     * @return Reference to the global configuration instance
     *
     * Thread Safety:
     * C++11 guarantees that static local variable initialization is thread-safe.
     * Multiple threads can safely call this function concurrently.
     *
     * Lifetime:
     * The configuration object lives for the entire program duration.
     * Destruction happens automatically at program exit.
     *
     * Usage:
     * @code
     * auto& config = Configuration::instance();
     * config.simulation().time_step = 0.005;
     * @endcode
     */
    static Configuration& instance() {
        static Configuration config;
        return config;
    }

    // ============================================================================
    // Configuration Access Methods
    // ============================================================================

    /** @brief Get read-only access to simulation configuration
     *  @return Const reference to simulation parameters
     *  @note noexcept - safe for use in performance-critical paths */
    const SimulationConfig& simulation() const noexcept { return simulation_config_; }

    /** @brief Get read-only access to Kalman filter configuration
     *  @return Const reference to filter parameters
     *  @note noexcept - safe for use in performance-critical paths */
    const KalmanConfig& kalman() const noexcept { return kalman_config_; }

    /** @brief Get read-only access to DC motor configuration
     *  @return Const reference to motor parameters
     *  @note noexcept - safe for use in performance-critical paths */
    const DCMotorConfig& motor() const noexcept { return motor_config_; }

    /** @brief Get read-only access to UI configuration
     *  @return Const reference to UI parameters
     *  @note noexcept - safe for use in performance-critical paths */
    const UiConfig& ui() const noexcept { return ui_config_; }

    // ============================================================================
    // Mutable Configuration Access (for runtime modification)
    // ============================================================================

    /** @brief Get mutable access to simulation configuration
     *  @return Mutable reference to simulation parameters
     *  @warning Changes take effect immediately - validate before use */
    SimulationConfig& simulation() noexcept { return simulation_config_; }

    /** @brief Get mutable access to Kalman filter configuration
     *  @return Mutable reference to filter parameters
     *  @warning Filter may need reinitialization after parameter changes */
    KalmanConfig& kalman() noexcept { return kalman_config_; }

    /** @brief Get mutable access to DC motor configuration
     *  @return Mutable reference to motor parameters
     *  @warning Motor model may need reinitialization after changes */
    DCMotorConfig& motor() noexcept { return motor_config_; }

    /** @brief Get mutable access to UI configuration
     *  @return Mutable reference to UI parameters
     *  @warning UI theme may need reapplication after color changes */
    UiConfig& ui() noexcept { return ui_config_; }

    // Configuration validation
    std::expected<void, std::string> validate() const noexcept;

    // Save/Load configuration
    std::expected<void, std::string> save_to_file(const std::string& filename) const;
    std::expected<void, std::string> load_from_file(const std::string& filename);

    // Reset to defaults
    void reset_to_defaults() noexcept;

    // Configuration change notifications
    template<typename Callback>
    void register_change_callback(const std::string& name, Callback&& callback) {
        change_callbacks_[name] = std::forward<Callback>(callback);
    }

    void notify_change(const std::string& parameter_name) const;

private:
    Configuration() = default;
    ~Configuration() = default;
    Configuration(const Configuration&) = delete;
    Configuration& operator=(const Configuration&) = delete;

    SimulationConfig simulation_config_;
    KalmanConfig kalman_config_;
    DCMotorConfig motor_config_;
    UiConfig ui_config_;

    // Change notification system
    std::unordered_map<std::string, std::function<void()>> change_callbacks_;
};

// Convenience aliases
using SimConfig = Configuration::SimulationConfig;
using KalmanParams = Configuration::KalmanConfig;
using MotorParams = Configuration::DCMotorConfig;
using UiParams = Configuration::UiConfig;

// Global configuration access
inline Configuration& config() {
    return Configuration::instance();
}

// Validation concepts
template<typename T>
concept Numeric = std::integral<T> || std::floating_point<T>;

template<Numeric T>
constexpr bool is_positive(T value) noexcept {
    return value > T{0};
}

template<Numeric T>
constexpr bool is_in_range(T value, T min, T max) noexcept {
    return value >= min && value <= max;
}

} // namespace kf::config
