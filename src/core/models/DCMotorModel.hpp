#pragma once

#include "../math/Matrix.hpp"
#include "../config/Configuration.hpp"
#include <concepts>
#include <array>
#include <cmath>
#include <algorithm>
#include <limits>

namespace kf::models {

/**
 * @brief Physics-based DC Motor model implementation
 *
 * Models the electrical and mechanical dynamics of a DC motor based on:
 * - Electrical equation: Li' + Ri = V - ke*ω
 * - Mechanical equation: Jω' + bω = τ = kt*i
 *
 * Where:
 * - L: terminal inductance
 * - R: terminal resistance
 * - V: input voltage
 * - ke: back EMF constant
 * - ω: angular velocity
 * - J: rotational inertia
 * - b: damping factor
 * - kt: torque sensitivity
 * - i: armature current
 * - τ: torque
 */
template<std::floating_point T = double>
class DCMotorModel {
public:
    using ValueType = T;
    using State = math::Matrix<T, 4, 1>; // [θ, ω, i, V_applied]
    using Parameters = config::Configuration::DCMotorConfig;

    // State indices
    static constexpr std::size_t POSITION_IDX = 0;
    static constexpr std::size_t VELOCITY_IDX = 1;
    static constexpr std::size_t CURRENT_IDX = 2;
    static constexpr std::size_t VOLTAGE_IDX = 3;

    /**
     * @brief Constructor with motor parameters
     * @param params DC motor parameters from configuration
     */
    explicit DCMotorModel(const Parameters& params = config::config().motor())
        : params_(params), state_{}, time_(0.0) {
        validate_parameters();
    }

    /**
     * @brief Set initial state
     * @param initial_state Initial state vector [θ, ω, i, V]
     */
    void set_initial_state(const State& initial_state) {
        state_ = initial_state;
        time_ = T{0};
    }

    /**
     * @brief Set input voltage
     * @param voltage Input voltage [V]
     */
    void set_input_voltage(T voltage) noexcept {
        // Clamp voltage to safe range
        voltage = std::clamp(voltage, static_cast<T>(-params_.input_voltage_range), static_cast<T>(params_.input_voltage_range));
        state_(VOLTAGE_IDX, 0) = voltage;
    }

    /**
     * @brief Update motor state using Runge-Kutta 4th order integration
     * @param dt Time step [s]
     * @return Updated state
     */
    State update(T dt) {
        // RK4 integration for better accuracy
        State k1 = dt * compute_derivatives(state_);
        State k2 = dt * compute_derivatives(state_ + k1 * T{0.5});
        State k3 = dt * compute_derivatives(state_ + k2 * T{0.5});
        State k4 = dt * compute_derivatives(state_ + k3);

        state_ += (k1 + k2 * T{2} + k3 * T{2} + k4) / T{6};
        time_ += dt;

        return state_;
    }

    /**
     * @brief Update motor state using simple Euler integration (faster)
     * @param dt Time step [s]
     * @return Updated state
     */
    State update_euler(T dt) {
        State derivatives = compute_derivatives(state_);
        state_ += derivatives * dt;
        time_ += dt;
        return state_;
    }

    /**
     * @brief Get current motor state
     * @return Current state vector [θ, ω, i, V]
     */
    const State& get_state() const noexcept { return state_; }

    /**
     * @brief Get angular position [rad]
     */
    T get_position() const noexcept { return state_(POSITION_IDX, 0); }

    /**
     * @brief Get angular velocity [rad/s]
     */
    T get_velocity() const noexcept { return state_(VELOCITY_IDX, 0); }

    /**
     * @brief Get armature current [A]
     */
    T get_current() const noexcept { return state_(CURRENT_IDX, 0); }

    /**
     * @brief Get applied voltage [V]
     */
    T get_voltage() const noexcept { return state_(VOLTAGE_IDX, 0); }

    /**
     * @brief Get current simulation time [s]
     */
    T get_time() const noexcept { return time_; }

    /**
     * @brief Compute instantaneous torque [Nm]
     */
    T get_torque() const noexcept {
        return params_.torque_sensitivity * get_current();
    }

    /**
     * @brief Compute back EMF [V]
     */
    T get_back_emf() const noexcept {
        return params_.back_emf_constant * get_velocity();
    }

    /**
     * @brief Compute electrical power [W]
     */
    T get_electrical_power() const noexcept {
        return get_voltage() * get_current();
    }

    /**
     * @brief Compute mechanical power [W]
     */
    T get_mechanical_power() const noexcept {
        return get_torque() * get_velocity();
    }

    /**
     * @brief Compute motor efficiency [0-1]
     */
    T get_efficiency() const noexcept {
        T electrical_power = get_electrical_power();
        if (std::abs(electrical_power) < std::numeric_limits<T>::epsilon()) {
            return T{0};
        }
        return std::abs(get_mechanical_power() / electrical_power);
    }

    /**
     * @brief Reset motor to initial conditions
     */
    void reset() noexcept {
        state_ = State{};
        time_ = T{0};
    }

    /**
     * @brief Update motor parameters
     * @param new_params New motor parameters
     */
    void update_parameters(const Parameters& new_params) {
        params_ = new_params;
        validate_parameters();
    }

    /**
     * @brief Get current motor parameters
     */
    const Parameters& get_parameters() const noexcept { return params_; }

    /**
     * @brief Compute steady-state response for constant voltage
     * @param voltage Constant input voltage [V]
     * @return Steady-state [position, velocity, current] (position is meaningless for constant input)
     */
    std::array<T, 3> compute_steady_state(T voltage) const noexcept {
        // At steady state: di/dt = 0, dω/dt = 0
        // From electrical equation: R*i = V - ke*ω
        // From mechanical equation: b*ω = kt*i
        // Solving: ω = kt*V / (kt*ke + R*b)
        //          i = b*V / (kt*ke + R*b)

        T denominator = params_.torque_sensitivity * params_.back_emf_constant +
                       params_.terminal_resistance * params_.damping_factor;

        if (std::abs(denominator) < std::numeric_limits<T>::epsilon()) {
            return {T{0}, T{0}, T{0}};
        }

        T steady_velocity = (params_.torque_sensitivity * voltage) / denominator;
        T steady_current = (params_.damping_factor * voltage) / denominator;

        return {T{0}, steady_velocity, steady_current}; // Position is arbitrary for constant input
    }

    /**
     * @brief Get transfer function coefficients (velocity/voltage)
     * @return {numerator_coeff, denominator_coeffs} for s-domain transfer function
     */
    std::pair<T, std::array<T, 3>> get_transfer_function() const noexcept {
        // Transfer function: ω(s)/V(s) = kt / (LJs² + (RJ + Lb)s + (Rb + kt*ke))
        T num = params_.torque_sensitivity;
        std::array<T, 3> den = {
            static_cast<T>(params_.terminal_inductance * params_.rotational_inertia),  // s²
            static_cast<T>(params_.terminal_resistance * params_.rotational_inertia +
            params_.terminal_inductance * params_.damping_factor),      // s¹
            static_cast<T>(params_.terminal_resistance * params_.damping_factor +
            params_.torque_sensitivity * params_.back_emf_constant)     // s⁰
        };
        return {num, den};
    }

private:
    /**
     * @brief Compute state derivatives for integration
     * @param state Current state vector
     * @return Derivative vector [dθ/dt, dω/dt, di/dt, dV/dt]
     */
    State compute_derivatives(const State& state) const noexcept {
        State derivatives{};

        // T theta = state(POSITION_IDX, 0);  // Position - unused in derivatives
        T omega = state(VELOCITY_IDX, 0);
        T current = state(CURRENT_IDX, 0);
        T voltage = state(VOLTAGE_IDX, 0);

        // Position derivative: dθ/dt = ω
        derivatives(POSITION_IDX, 0) = omega;

        // Velocity derivative: dω/dt = (kt*i - b*ω) / J
        derivatives(VELOCITY_IDX, 0) = (params_.torque_sensitivity * current -
                                       params_.damping_factor * omega) /
                                       params_.rotational_inertia;

        // Current derivative: di/dt = (V - ke*ω - R*i) / L
        derivatives(CURRENT_IDX, 0) = (voltage - params_.back_emf_constant * omega -
                                      params_.terminal_resistance * current) /
                                      params_.terminal_inductance;

        // Voltage is externally controlled, so dV/dt = 0 (or could be set by controller)
        derivatives(VOLTAGE_IDX, 0) = T{0};

        return derivatives;
    }

    /**
     * @brief Validate motor parameters for physical consistency
     */
    void validate_parameters() const {
        if (params_.torque_sensitivity <= T{0}) {
            throw std::invalid_argument("Torque sensitivity must be positive");
        }
        if (params_.back_emf_constant <= T{0}) {
            throw std::invalid_argument("Back EMF constant must be positive");
        }
        if (params_.terminal_resistance <= T{0}) {
            throw std::invalid_argument("Terminal resistance must be positive");
        }
        if (params_.terminal_inductance <= T{0}) {
            throw std::invalid_argument("Terminal inductance must be positive");
        }
        if (params_.rotational_inertia <= T{0}) {
            throw std::invalid_argument("Rotational inertia must be positive");
        }
        if (params_.damping_factor < T{0}) {
            throw std::invalid_argument("Damping factor must be non-negative");
        }
    }

    Parameters params_;
    State state_;
    T time_;
};

// Type aliases
using DCMotorModeld = DCMotorModel<double>;
using DCMotorModelf = DCMotorModel<float>;

} // namespace kf::models
