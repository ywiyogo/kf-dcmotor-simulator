#include "DCMotorModel.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace kf::models {

// Explicit template instantiations for common floating point types
template class DCMotorModel<double>;
template class DCMotorModel<float>;

// Additional utility functions for DC Motor analysis

/**
 * @brief Compute time constants for DC motor
 * @tparam T Floating point type
 * @param params Motor parameters
 * @return {electrical_time_constant, mechanical_time_constant}
 */
template<std::floating_point T>
std::pair<T, T> compute_time_constants(const config::Configuration::DCMotorConfig& params) {
    T tau_electrical = params.terminal_inductance / params.terminal_resistance;

    // Mechanical time constant considering back EMF coupling
    T denominator = params.damping_factor +
                   (params.torque_sensitivity * params.back_emf_constant) / params.terminal_resistance;

    T tau_mechanical = (denominator > T{0}) ? params.rotational_inertia / denominator : T{0};

    return {tau_electrical, tau_mechanical};
}

/**
 * @brief Compute natural frequency and damping ratio
 * @tparam T Floating point type
 * @param params Motor parameters
 * @return {natural_frequency, damping_ratio}
 */
template<std::floating_point T>
std::pair<T, T> compute_dynamic_characteristics(const config::Configuration::DCMotorConfig& params) {
    // Coefficients from transfer function denominator: LJs² + (RJ + Lb)s + (Rb + kt*ke)
    T a2 = params.terminal_inductance * params.rotational_inertia;
    T a1 = params.terminal_resistance * params.rotational_inertia +
           params.terminal_inductance * params.damping_factor;
    T a0 = params.terminal_resistance * params.damping_factor +
           params.torque_sensitivity * params.back_emf_constant;

    if (a2 <= T{0} || a0 <= T{0}) {
        return {T{0}, T{0}};
    }

    T omega_n = std::sqrt(a0 / a2);  // Natural frequency [rad/s]
    T zeta = a1 / (2 * std::sqrt(a0 * a2));  // Damping ratio

    return {omega_n, zeta};
}

/**
 * @brief Compute step response characteristics
 * @tparam T Floating point type
 * @param params Motor parameters
 * @return {rise_time, settling_time, overshoot_percent}
 */
template<std::floating_point T>
std::array<T, 3> compute_step_response_characteristics(const config::Configuration::DCMotorConfig& params) {
    auto [omega_n, zeta] = compute_dynamic_characteristics<T>(params);

    if (omega_n <= T{0}) {
        return {T{0}, T{0}, T{0}};
    }

    T rise_time = T{0};
    T settling_time = T{0};
    T overshoot = T{0};

    if (zeta < T{1}) {
        // Underdamped case
        T omega_d = omega_n * std::sqrt(T{1} - zeta * zeta);

        // Rise time (10% to 90%)
        T beta = std::atan2(std::sqrt(T{1} - zeta * zeta), zeta);
        rise_time = (std::numbers::pi_v<T> - beta) / omega_d;

        // Settling time (2% criterion)
        settling_time = T{4} / (zeta * omega_n);

        // Overshoot percentage
        overshoot = T{100} * std::exp(-zeta * std::numbers::pi_v<T> / std::sqrt(T{1} - zeta * zeta));
    } else {
        // Overdamped or critically damped
        rise_time = T{2.2} / omega_n;
        settling_time = T{4} / (zeta * omega_n);
        overshoot = T{0};
    }

    return {rise_time, settling_time, overshoot};
}

/**
 * @brief Compute bandwidth and phase margin
 * @tparam T Floating point type
 * @param params Motor parameters
 * @return {bandwidth_hz, phase_margin_deg}
 */
template<std::floating_point T>
std::pair<T, T> compute_frequency_response_characteristics(const config::Configuration::DCMotorConfig& params) {
    auto [omega_n, zeta] = compute_dynamic_characteristics<T>(params);

    if (omega_n <= T{0}) {
        return {T{0}, T{0}};
    }

    // Bandwidth (frequency at -3dB)
    T bandwidth_rad_s = omega_n * std::sqrt(std::sqrt(T{2}) - T{1});
    T bandwidth_hz = bandwidth_rad_s / (T{2} * std::numbers::pi_v<T>);

    // Phase margin (simplified estimate)
    T phase_margin_deg = T{90} - T{180} / std::numbers::pi_v<T> * std::atan2(T{2} * zeta, T{1});

    return {bandwidth_hz, phase_margin_deg};
}

/**
 * @brief Validate motor parameters for stability
 * @tparam T Floating point type
 * @param params Motor parameters
 * @return Error message if unstable, empty string if stable
 */
template<std::floating_point T>
std::string validate_stability(const config::Configuration::DCMotorConfig& params) {
    // Check Routh-Hurwitz criterion for stability
    // For second-order system: all coefficients must be positive
    T a2 = params.terminal_inductance * params.rotational_inertia;
    T a1 = params.terminal_resistance * params.rotational_inertia +
           params.terminal_inductance * params.damping_factor;
    T a0 = params.terminal_resistance * params.damping_factor +
           params.torque_sensitivity * params.back_emf_constant;

    if (a2 <= T{0}) {
        return "System is unstable: negative inertia-inductance product";
    }
    if (a1 <= T{0}) {
        return "System is unstable: insufficient damping";
    }
    if (a0 <= T{0}) {
        return "System is unstable: negative steady-state gain";
    }

    // Check for reasonable parameter ranges
    auto [omega_n, zeta] = compute_dynamic_characteristics<T>(params);

    if (zeta < T{0.1}) {
        return "Warning: very low damping ratio, system may be oscillatory";
    }
    if (zeta > T{2.0}) {
        return "Warning: very high damping ratio, system response may be sluggish";
    }
    if (omega_n > T{1000}) {
        return "Warning: very high natural frequency, may require small time steps";
    }

    return ""; // Stable
}

/**
 * @brief Compute optimal sampling time for digital control
 * @tparam T Floating point type
 * @param params Motor parameters
 * @param oversampling_factor Oversampling factor (default: 10)
 * @return Recommended sampling time [s]
 */
template<std::floating_point T>
T compute_optimal_sampling_time(const config::Configuration::DCMotorConfig& params,
                               T oversampling_factor = T{10}) {
    auto [omega_n, zeta] = compute_dynamic_characteristics<T>(params);

    if (omega_n <= T{0}) {
        return T{0.01}; // Default 10ms
    }

    // Sample at least 10 times faster than the natural frequency
    T min_sampling_freq = oversampling_factor * omega_n / (T{2} * std::numbers::pi_v<T>);
    T max_sampling_time = T{1} / min_sampling_freq;

    // Ensure reasonable bounds
    max_sampling_time = std::clamp(max_sampling_time, T{0.0001}, T{0.1});

    return max_sampling_time;
}

/**
 * @brief Create motor model with realistic noise characteristics
 * @tparam T Floating point type
 * @param params Motor parameters
 * @return Motor model with typical noise levels
 */
template<std::floating_point T>
DCMotorModel<T> create_realistic_motor(const config::Configuration::DCMotorConfig& params) {
    DCMotorModel<T> motor(params);

    // Set realistic initial conditions
    typename DCMotorModel<T>::State initial_state{};
    initial_state(DCMotorModel<T>::POSITION_IDX, 0) = T{0};     // Start at zero position
    initial_state(DCMotorModel<T>::VELOCITY_IDX, 0) = T{0};     // Start at rest
    initial_state(DCMotorModel<T>::CURRENT_IDX, 0) = T{0};      // No initial current
    initial_state(DCMotorModel<T>::VOLTAGE_IDX, 0) = T{0};      // No initial voltage

    motor.set_initial_state(initial_state);

    return motor;
}

// Explicit instantiations for utility functions
template std::pair<double, double> compute_time_constants<double>(const config::Configuration::DCMotorConfig&);
template std::pair<float, float> compute_time_constants<float>(const config::Configuration::DCMotorConfig&);

template std::pair<double, double> compute_dynamic_characteristics<double>(const config::Configuration::DCMotorConfig&);
template std::pair<float, float> compute_dynamic_characteristics<float>(const config::Configuration::DCMotorConfig&);

template std::array<double, 3> compute_step_response_characteristics<double>(const config::Configuration::DCMotorConfig&);
template std::array<float, 3> compute_step_response_characteristics<float>(const config::Configuration::DCMotorConfig&);

template std::pair<double, double> compute_frequency_response_characteristics<double>(const config::Configuration::DCMotorConfig&);
template std::pair<float, float> compute_frequency_response_characteristics<float>(const config::Configuration::DCMotorConfig&);

template std::string validate_stability<double>(const config::Configuration::DCMotorConfig&);
template std::string validate_stability<float>(const config::Configuration::DCMotorConfig&);

template double compute_optimal_sampling_time<double>(const config::Configuration::DCMotorConfig&, double);
template float compute_optimal_sampling_time<float>(const config::Configuration::DCMotorConfig&, float);

template DCMotorModel<double> create_realistic_motor<double>(const config::Configuration::DCMotorConfig&);
template DCMotorModel<float> create_realistic_motor<float>(const config::Configuration::DCMotorConfig&);

} // namespace kf::models
