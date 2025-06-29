#include "KalmanFilter.hpp"
#include <algorithm>
#include <cmath>

namespace kf::models {

// Explicit template instantiations for common floating point types
template class KalmanFilter<double>;
template class KalmanFilter<float>;
template class ExtendedKalmanFilter<double>;
template class ExtendedKalmanFilter<float>;

// Additional utility functions for Kalman Filter analysis

/**
 * @brief Compute optimal Kalman filter gains for steady-state operation
 * @tparam T Floating point type
 * @param F State transition matrix
 * @param H Measurement matrix
 * @param Q Process noise covariance
 * @param R Measurement noise covariance
 * @return Steady-state Kalman gain
 */
template<std::floating_point T>
math::Matrix<T, 2, 1> compute_steady_state_gain(
    const math::Matrix<T, 2, 2>& F,
    const math::Matrix<T, 1, 2>& H,
    const math::Matrix<T, 2, 2>& Q,
    const math::Matrix<T, 1, 1>& R) {

    // Solve discrete-time algebraic Riccati equation iteratively
    math::Matrix<T, 2, 2> P = Q; // Initial guess
    math::Matrix<T, 2, 1> K{};

    const std::size_t max_iterations = 1000;
    const T tolerance = T{1e-10};

    for (std::size_t iter = 0; iter < max_iterations; ++iter) {
        // Compute innovation covariance
        auto S = H * P * H.transpose() + R;

        // Check for singularity
        if (std::abs(S(0, 0)) < std::numeric_limits<T>::epsilon()) {
            break;
        }

        // Compute Kalman gain
        auto S_inv = math::Matrix<T, 1, 1>{T{1} / S(0, 0)};
        K = P * H.transpose() * S_inv;

        // Update covariance using Riccati equation
        auto I_KH = math::Matrix<T, 2, 2>::identity() - K * H;
        math::Matrix<T, 2, 2> P_new = F * I_KH * P * F.transpose() + Q;

        // Check convergence
        if ((P_new - P).frobenius_norm() < tolerance) {
            break;
        }

        P = P_new;
    }

    return K;
}

/**
 * @brief Analyze filter observability
 * @tparam T Floating point type
 * @param F State transition matrix
 * @param H Measurement matrix
 * @return Observability matrix rank (should be 2 for full observability)
 */
template<std::floating_point T>
std::size_t analyze_observability(
    const math::Matrix<T, 2, 2>& F,
    const math::Matrix<T, 1, 2>& H) {

    // Construct observability matrix O = [H; H*F]
    math::Matrix<T, 2, 2> O{};
    O(0, 0) = H(0, 0); O(0, 1) = H(0, 1);

    auto HF = H * F;
    O(1, 0) = HF(0, 0); O(1, 1) = HF(0, 1);

    // Check rank by computing determinant
    T det = O.determinant();

    // If determinant is non-zero, rank is 2 (full observability)
    return (std::abs(det) > std::numeric_limits<T>::epsilon()) ? 2 : 1;
}

/**
 * @brief Compute filter performance metrics
 * @tparam T Floating point type
 * @param true_states Vector of true states for comparison
 * @param estimates Vector of filter estimates
 * @param dt Time step
 * @return Performance metrics {RMSE_position, RMSE_velocity, convergence_time}
 */
template<std::floating_point T>
std::array<T, 3> compute_performance_metrics(
    const std::vector<math::Matrix<T, 2, 1>>& true_states,
    const std::vector<math::Matrix<T, 2, 1>>& estimates,
    T dt) {

    if (true_states.size() != estimates.size() || true_states.empty()) {
        return {T{0}, T{0}, T{0}};
    }

    T rmse_position = T{0};
    T rmse_velocity = T{0};
    T convergence_time = T{0};

    std::size_t n = true_states.size();

    // Compute RMSE for position and velocity
    for (std::size_t i = 0; i < n; ++i) {
        T pos_error = true_states[i](0, 0) - estimates[i](0, 0);
        T vel_error = true_states[i](1, 0) - estimates[i](1, 0);

        rmse_position += pos_error * pos_error;
        rmse_velocity += vel_error * vel_error;
    }

    rmse_position = std::sqrt(rmse_position / n);
    rmse_velocity = std::sqrt(rmse_velocity / n);

    // Estimate convergence time (when error drops below 5% of initial error)
    if (n > 1) {
        T initial_error = (true_states[0] - estimates[0]).frobenius_norm();
        T convergence_threshold = T{0.05} * initial_error;

        for (std::size_t i = 1; i < n; ++i) {
            T current_error = (true_states[i] - estimates[i]).frobenius_norm();
            if (current_error < convergence_threshold) {
                convergence_time = i * dt;
                break;
            }
        }

        if (convergence_time == T{0}) {
            convergence_time = n * dt; // Didn't converge within simulation time
        }
    }

    return {rmse_position, rmse_velocity, convergence_time};
}

/**
 * @brief Tune Kalman filter parameters automatically
 * @tparam T Floating point type
 * @param measurements Vector of measurements for tuning
 * @param dt Time step
 * @return Optimal {process_noise_std, measurement_noise_std}
 */
template<std::floating_point T>
std::pair<T, T> auto_tune_parameters(
    const std::vector<T>& measurements,
    T dt) {

    if (measurements.size() < 10) {
        return {T{0.1}, T{0.2}}; // Default values
    }

    // Estimate measurement noise from high-frequency variations
    T measurement_noise_var = T{0};
    for (std::size_t i = 1; i < measurements.size(); ++i) {
        T diff = measurements[i] - measurements[i-1];
        measurement_noise_var += diff * diff;
    }
    measurement_noise_var /= (measurements.size() - 1);
    T measurement_noise_std = std::sqrt(measurement_noise_var);

    // Estimate process noise from low-frequency variations
    // Use Allan variance approach for better estimation
    std::vector<T> differences;
    for (std::size_t i = 2; i < measurements.size(); ++i) {
        T second_diff = measurements[i] - 2 * measurements[i-1] + measurements[i-2];
        differences.push_back(second_diff);
    }

    T process_noise_var = T{0};
    if (!differences.empty()) {
        for (T diff : differences) {
            process_noise_var += diff * diff;
        }
        process_noise_var /= differences.size();
        process_noise_var /= (dt * dt * dt * dt); // Convert to acceleration variance
    }

    T process_noise_std = std::sqrt(process_noise_var);

    // Apply reasonable bounds
    measurement_noise_std = std::clamp(measurement_noise_std, T{0.001}, T{1.0});
    process_noise_std = std::clamp(process_noise_std, T{0.001}, T{1.0});

    return {process_noise_std, measurement_noise_std};
}

/**
 * @brief Implement adaptive Kalman filter with time-varying noise
 * @tparam T Floating point type
 */
template<std::floating_point T>
class AdaptiveKalmanFilter : public KalmanFilter<T> {
private:
    T innovation_window_size_;
    std::vector<T> innovation_history_;
    T adaptation_factor_;

public:
    explicit AdaptiveKalmanFilter(T dt, const config::Configuration::KalmanConfig& params = config::config().kalman(),
                                 T window_size = T{10}, T adaptation_factor = T{0.1})
        : KalmanFilter<T>(dt, params), innovation_window_size_(window_size),
          adaptation_factor_(adaptation_factor) {}

    /**
     * @brief Adaptive update with noise estimation
     */
    std::expected<typename KalmanFilter<T>::State, std::string> adaptive_update(T measurement) {
        // Perform standard update
        auto result = this->update(measurement);
        if (!result) {
            return result;
        }

        // Compute innovation
        T innovation = this->compute_residual(measurement);
        innovation_history_.push_back(innovation * innovation);

        // Maintain window size
        if (innovation_history_.size() > static_cast<std::size_t>(innovation_window_size_)) {
            innovation_history_.erase(innovation_history_.begin());
        }

        // Adapt measurement noise if enough data
        if (innovation_history_.size() >= 5) {
            T empirical_variance = T{0};
            for (T innov_sq : innovation_history_) {
                empirical_variance += innov_sq;
            }
            empirical_variance /= innovation_history_.size();

            // Get current theoretical innovation variance
            T theoretical_variance = this->get_statistics().innovation_variance;

            if (theoretical_variance > T{0}) {
                T ratio = empirical_variance / theoretical_variance;

                // Adapt if ratio is significantly different from 1
                if (ratio > T{1.5} || ratio < T{0.5}) {
                    T current_noise = static_cast<T>(this->params_.measurement_noise_std);
                    T new_noise = current_noise * std::sqrt(ratio);
                    new_noise = (T{1} - adaptation_factor_) * current_noise + adaptation_factor_ * new_noise;

                    // Apply bounds
                    new_noise = std::clamp(new_noise, T{0.001}, T{1.0});

                    this->set_measurement_noise(new_noise);
                }
            }
        }

        return result;
    }
};

/**
 * @brief Compute innovation sequence for filter validation
 * @tparam T Floating point type
 * @param filter Kalman filter instance
 * @param measurements Vector of measurements
 * @return Vector of innovations
 */
template<std::floating_point T>
std::vector<T> compute_innovation_sequence(
    KalmanFilter<T>& filter,
    const std::vector<T>& measurements) {

    std::vector<T> innovations;
    innovations.reserve(measurements.size());

    for (T measurement : measurements) {
        // Predict step
        filter.predict();

        // Compute innovation before update
        T innovation = filter.compute_residual(measurement);
        innovations.push_back(innovation);

        // Update step
        filter.update(measurement);
    }

    return innovations;
}

/**
 * @brief Validate filter performance using innovation whiteness test
 * @tparam T Floating point type
 * @param innovations Vector of innovations
 * @return {is_white, autocorrelation_at_lag_1}
 */
template<std::floating_point T>
std::pair<bool, T> validate_innovation_whiteness(const std::vector<T>& innovations) {
    if (innovations.size() < 10) {
        return {false, T{0}};
    }

    // Compute sample mean
    T mean = T{0};
    for (T innov : innovations) {
        mean += innov;
    }
    mean /= innovations.size();

    // Compute autocorrelation at lag 1
    T autocorr = T{0};
    T variance = T{0};

    for (std::size_t i = 0; i < innovations.size() - 1; ++i) {
        T centered_i = innovations[i] - mean;
        T centered_i1 = innovations[i + 1] - mean;
        autocorr += centered_i * centered_i1;
        variance += centered_i * centered_i;
    }

    if (variance > T{0}) {
        autocorr /= variance;
    }

    // Test for whiteness (autocorrelation should be close to 0)
    T whiteness_threshold = T{0.1};
    bool is_white = std::abs(autocorr) < whiteness_threshold;

    return {is_white, autocorr};
}

// Explicit instantiations for utility functions
template math::Matrix<double, 2, 1> compute_steady_state_gain<double>(
    const math::Matrix<double, 2, 2>&, const math::Matrix<double, 1, 2>&,
    const math::Matrix<double, 2, 2>&, const math::Matrix<double, 1, 1>&);

template math::Matrix<float, 2, 1> compute_steady_state_gain<float>(
    const math::Matrix<float, 2, 2>&, const math::Matrix<float, 1, 2>&,
    const math::Matrix<float, 2, 2>&, const math::Matrix<float, 1, 1>&);

template std::size_t analyze_observability<double>(
    const math::Matrix<double, 2, 2>&, const math::Matrix<double, 1, 2>&);

template std::size_t analyze_observability<float>(
    const math::Matrix<float, 2, 2>&, const math::Matrix<float, 1, 2>&);

template std::array<double, 3> compute_performance_metrics<double>(
    const std::vector<math::Matrix<double, 2, 1>>&,
    const std::vector<math::Matrix<double, 2, 1>>&, double);

template std::array<float, 3> compute_performance_metrics<float>(
    const std::vector<math::Matrix<float, 2, 1>>&,
    const std::vector<math::Matrix<float, 2, 1>>&, float);

template std::pair<double, double> auto_tune_parameters<double>(
    const std::vector<double>&, double);

template std::pair<float, float> auto_tune_parameters<float>(
    const std::vector<float>&, float);

template class AdaptiveKalmanFilter<double>;
template class AdaptiveKalmanFilter<float>;

template std::vector<double> compute_innovation_sequence<double>(
    KalmanFilter<double>&, const std::vector<double>&);

template std::vector<float> compute_innovation_sequence<float>(
    KalmanFilter<float>&, const std::vector<float>&);

template std::pair<bool, double> validate_innovation_whiteness<double>(const std::vector<double>&);
template std::pair<bool, float> validate_innovation_whiteness<float>(const std::vector<float>&);

} // namespace kf::models
