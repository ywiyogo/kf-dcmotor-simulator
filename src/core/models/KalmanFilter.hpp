#pragma once

#include "../math/Matrix.hpp"
#include "../config/Configuration.hpp"
#include "../utils/NoiseGenerator.hpp"
#include <concepts>
#include <expected>
#include <string>
#include <array>
#include <vector>
#include <memory>
#include <chrono>
#include <cmath>
#include <numbers>
#include <limits>

namespace kf::models {

/**
 * @brief Modern C++23 Kalman Filter implementation for DC Motor state estimation
 * 
 * Estimates the state vector x = [θ, ω] (position, velocity) from noisy
 * position measurements using the kinematic model:
 * 
 * State equation: x_k = F * x_{k-1} + G * a_k + w_k
 * Measurement equation: z_k = H * x_k + v_k
 * 
 * Where:
 * - F = [1, Δt; 0, 1] (state transition matrix)
 * - G = [0.5*Δt², Δt] (process noise input matrix)
 * - H = [1, 0] (measurement matrix)
 * - w_k ~ N(0, Q) (process noise)
 * - v_k ~ N(0, R) (measurement noise)
 */
template<std::floating_point T = double>
class KalmanFilter {
public:
    using ValueType = T;
    using State = math::Matrix<T, 2, 1>;           // [θ, ω]
    using Covariance = math::Matrix<T, 2, 2>;      // State covariance
    using Measurement = math::Matrix<T, 1, 1>;     // [θ_measured]
    using ProcessNoise = math::Matrix<T, 2, 2>;    // Process noise covariance
    using MeasurementNoise = math::Matrix<T, 1, 1>; // Measurement noise covariance
    using StateTransition = math::Matrix<T, 2, 2>; // F matrix
    using MeasurementMatrix = math::Matrix<T, 1, 2>; // H matrix
    using ProcessNoiseMatrix = math::Matrix<T, 2, 1>; // G matrix
    using KalmanGain = math::Matrix<T, 2, 1>;      // Kalman gain

    // State indices
    static constexpr std::size_t POSITION_IDX = 0;
    static constexpr std::size_t VELOCITY_IDX = 1;

    /**
     * @brief Filter statistics for performance monitoring
     */
    struct FilterStatistics {
        T innovation_variance = T{0};      // Measurement innovation variance
        T position_variance = T{0};        // Position estimate variance
        T velocity_variance = T{0};        // Velocity estimate variance
        T condition_number = T{0};         // Covariance matrix condition number
        std::size_t update_count = 0;      // Number of updates performed
        std::chrono::steady_clock::time_point last_update_time;
        
        // Convergence metrics
        T log_likelihood = T{0};           // Log-likelihood of measurements
        T normalized_innovation = T{0};    // Normalized innovation (whiteness test)
        bool is_converged = false;         // Convergence flag
    };

    /**
     * @brief Constructor with Kalman filter parameters
     * @param dt Time step [s]
     * @param params Kalman filter parameters from configuration
     */
    explicit KalmanFilter(T dt, const config::KalmanParams& params = config::config().kalman())
        : dt_(dt), params_(params), statistics_{} {
        
        initialize_matrices();
        reset();
        
        // Initialize noise generator if needed
        noise_generator_ = std::make_unique<utils::NoiseGenerator<T>>();
    }

    /**
     * @brief Reset filter to initial conditions
     */
    void reset() {
        // Initialize state estimate
        x_hat_ = State{
            static_cast<T>(params_.initial_state[0]),
            static_cast<T>(params_.initial_state[1])
        };
        
        // Initialize covariance estimate
        P_ = Covariance{
            {static_cast<T>(params_.initial_covariance[0][0]), static_cast<T>(params_.initial_covariance[0][1])},
            {static_cast<T>(params_.initial_covariance[1][0]), static_cast<T>(params_.initial_covariance[1][1])}
        };
        
        // Reset statistics
        statistics_ = FilterStatistics{};
        statistics_.last_update_time = std::chrono::steady_clock::now();
        
        // Initialize matrices
        initialize_matrices();
    }

    /**
     * @brief Predict step of Kalman filter
     * @param control_input Optional control input (acceleration)
     * @return Predicted state
     */
    State predict(T control_input = T{0}) {
        // State prediction: x_hat^- = F * x_hat + G * u
        State x_pred = F_ * x_hat_;
        if (std::abs(control_input) > std::numeric_limits<T>::epsilon()) {
            x_pred += G_ * control_input;
        }
        
        // Covariance prediction: P^- = F * P * F^T + Q
        Covariance P_pred = F_ * P_ * F_.transpose() + Q_;
        
        // Ensure covariance remains positive definite
        P_pred = ensure_positive_definite(P_pred);
        
        // Update state
        x_hat_ = x_pred;
        P_ = P_pred;
        
        return x_hat_;
    }

    /**
     * @brief Update step of Kalman filter with measurement
     * @param measurement Position measurement [rad]
     * @return Updated state estimate
     */
    std::expected<State, std::string> update(T measurement) {
        try {
            Measurement z{measurement};
            
            // Innovation: y = z - H * x_hat^-
            Measurement innovation = z - H_ * x_hat_;
            
            // Innovation covariance: S = H * P^- * H^T + R
            auto S = H_ * P_ * H_.transpose() + R_;
            
            // Check for numerical issues
            if (std::abs(S(0, 0)) < std::numeric_limits<T>::epsilon()) {
                return std::unexpected("Innovation covariance is singular");
            }
            
            // Kalman gain: K = P^- * H^T * S^{-1}
            auto S_inv = math::Matrix<T, 1, 1>{T{1} / S(0, 0)};
            KalmanGain K = P_ * H_.transpose() * S_inv;
            
            // State update: x_hat = x_hat^- + K * y
            x_hat_ += K * innovation;
            
            // Covariance update: P = (I - K * H) * P^-
            // Using Joseph form for numerical stability: P = (I - K*H)*P*(I - K*H)^T + K*R*K^T
            auto I_KH = math::Matrix<T, 2, 2>::identity() - K * H_;
            P_ = I_KH * P_ * I_KH.transpose() + K * R_ * K.transpose();
            
            // Ensure covariance remains positive definite
            P_ = ensure_positive_definite(P_);
            
            // Update statistics
            update_statistics(innovation, S);
            
            return x_hat_;
            
        } catch (const std::exception& e) {
            return std::unexpected(std::string("Kalman filter update failed: ") + e.what());
        }
    }

    /**
     * @brief Get current state estimate
     */
    const State& get_state() const noexcept { return x_hat_; }

    /**
     * @brief Get current covariance estimate
     */
    const Covariance& get_covariance() const noexcept { return P_; }

    /**
     * @brief Get position estimate [rad]
     */
    T get_position() const noexcept { return x_hat_(POSITION_IDX, 0); }

    /**
     * @brief Get velocity estimate [rad/s]
     */
    T get_velocity() const noexcept { return x_hat_(VELOCITY_IDX, 0); }

    /**
     * @brief Get position uncertainty (standard deviation) [rad]
     */
    T get_position_uncertainty() const noexcept {
        return std::sqrt(std::max(P_(POSITION_IDX, POSITION_IDX), T{0}));
    }

    /**
     * @brief Get velocity uncertainty (standard deviation) [rad/s]
     */
    T get_velocity_uncertainty() const noexcept {
        return std::sqrt(std::max(P_(VELOCITY_IDX, VELOCITY_IDX), T{0}));
    }

    /**
     * @brief Get filter statistics
     */
    const FilterStatistics& get_statistics() const noexcept { return statistics_; }

    /**
     * @brief Update time step
     * @param new_dt New time step [s]
     */
    void set_time_step(T new_dt) {
        if (new_dt <= T{0}) {
            throw std::invalid_argument("Time step must be positive");
        }
        dt_ = new_dt;
        initialize_matrices();
    }

    /**
     * @brief Update process noise parameters
     * @param process_noise_std New process noise standard deviation
     */
    void set_process_noise(T process_noise_std) {
        if (process_noise_std <= T{0}) {
            throw std::invalid_argument("Process noise standard deviation must be positive");
        }
        params_.process_noise_std = process_noise_std;
        initialize_matrices();
    }

    /**
     * @brief Update measurement noise parameters
     * @param measurement_noise_std New measurement noise standard deviation
     */
    void set_measurement_noise(T measurement_noise_std) {
        if (measurement_noise_std <= T{0}) {
            throw std::invalid_argument("Measurement noise standard deviation must be positive");
        }
        params_.measurement_noise_std = measurement_noise_std;
        initialize_matrices();
    }

    /**
     * @brief Get current time step
     */
    T get_time_step() const noexcept { return dt_; }

    /**
     * @brief Compute measurement residual for given measurement
     * @param measurement Position measurement [rad]
     * @return Measurement residual
     */
    T compute_residual(T measurement) const noexcept {
        return measurement - H_(0, 0) * x_hat_(POSITION_IDX, 0);
    }

    /**
     * @brief Check if filter has converged
     * @param tolerance Convergence tolerance
     * @return true if converged
     */
    bool has_converged(T tolerance = T{1e-6}) const noexcept {
        return P_.frobenius_norm() < tolerance;
    }

    /**
     * @brief Compute likelihood of measurement
     * @param measurement Position measurement [rad]
     * @return Log-likelihood value
     */
    T compute_log_likelihood(T measurement) const noexcept {
        T residual = compute_residual(measurement);
        T innovation_variance = H_(0, 0) * P_(POSITION_IDX, POSITION_IDX) * H_(0, 0) + 
                               static_cast<T>(params_.measurement_noise_std * params_.measurement_noise_std);
        
        if (innovation_variance <= T{0}) {
            return -std::numeric_limits<T>::infinity();
        }
        
        return -T{0.5} * (std::log(T{2} * std::numbers::pi_v<T> * innovation_variance) + 
                         residual * residual / innovation_variance);
    }

    /**
     * @brief Generate synthetic measurement with noise
     * @param true_position True position value [rad]
     * @return Noisy measurement
     */
    T generate_measurement(T true_position) {
        if (!noise_generator_) {
            noise_generator_ = std::make_unique<utils::NoiseGenerator<T>>();
        }
        
        T noise = noise_generator_->gaussian(T{0}, static_cast<T>(params_.measurement_noise_std));
        return true_position + noise;
    }

protected:
    /**
     * @brief Initialize filter matrices based on current parameters
     */
    void initialize_matrices() {
        // State transition matrix F = [1, dt; 0, 1]
        F_ = StateTransition{
            {T{1}, dt_},
            {T{0}, T{1}}
        };
        
        // Measurement matrix H = [1, 0]
        H_ = MeasurementMatrix{{T{1}, T{0}}};
        
        // Process noise input matrix G = [0.5*dt^2; dt]
        G_ = ProcessNoiseMatrix{
            T{0.5} * dt_ * dt_,
            dt_
        };
        
        // Process noise covariance Q = G * G^T * σ_a^2
        T sigma_a_sq = static_cast<T>(params_.process_noise_std * params_.process_noise_std);
        Q_ = ProcessNoise{
            {G_(0, 0) * G_(0, 0) * sigma_a_sq, G_(0, 0) * G_(1, 0) * sigma_a_sq},
            {G_(1, 0) * G_(0, 0) * sigma_a_sq, G_(1, 0) * G_(1, 0) * sigma_a_sq}
        };
        
        // Measurement noise covariance R = σ_t^2
        T sigma_t_sq = static_cast<T>(params_.measurement_noise_std * params_.measurement_noise_std);
        R_ = MeasurementNoise{sigma_t_sq};
    }

    /**
     * @brief Ensure matrix is positive definite through regularization
     * @param matrix Input matrix
     * @return Regularized positive definite matrix
     */
    Covariance ensure_positive_definite(const Covariance& matrix) const {
        Covariance result = matrix;
        
        // Make symmetric
        for (std::size_t i = 0; i < 2; ++i) {
            for (std::size_t j = 0; j < 2; ++j) {
                result(i, j) = (matrix(i, j) + matrix(j, i)) / T{2};
            }
        }
        
        // Add small regularization to diagonal if needed
        T regularization = T{1e-12};
        if (result.determinant() <= regularization) {
            result(0, 0) += regularization;
            result(1, 1) += regularization;
        }
        
        return result;
    }

    /**
     * @brief Update filter statistics
     * @param innovation Measurement innovation
     * @param innovation_covariance Innovation covariance
     */
    void update_statistics(const Measurement& innovation, const math::Matrix<T, 1, 1>& innovation_covariance) {
        statistics_.innovation_variance = innovation_covariance(0, 0);
        statistics_.position_variance = P_(POSITION_IDX, POSITION_IDX);
        statistics_.velocity_variance = P_(VELOCITY_IDX, VELOCITY_IDX);
        statistics_.condition_number = math::condition_number_2x2(P_);
        statistics_.update_count++;
        statistics_.last_update_time = std::chrono::steady_clock::now();
        
        // Normalized innovation squared (should be ~1 for good filter)
        if (statistics_.innovation_variance > T{0}) {
            statistics_.normalized_innovation = innovation(0, 0) * innovation(0, 0) / statistics_.innovation_variance;
        }
        
        // Log-likelihood update
        if (statistics_.innovation_variance > T{0}) {
            statistics_.log_likelihood += -T{0.5} * (std::log(T{2} * std::numbers::pi_v<T> * statistics_.innovation_variance) + 
                                                   statistics_.normalized_innovation);
        }
        
        // Convergence check
        statistics_.is_converged = has_converged();
    }

    // Filter parameters
    T dt_;
    config::KalmanParams params_;
    
    // Filter state
    State x_hat_;           // State estimate
    Covariance P_;          // Covariance estimate
    
    // Filter matrices
    StateTransition F_;     // State transition matrix
    MeasurementMatrix H_;   // Measurement matrix
    ProcessNoiseMatrix G_;  // Process noise input matrix
    ProcessNoise Q_;        // Process noise covariance
    MeasurementNoise R_;    // Measurement noise covariance
    
    // Statistics and monitoring
    FilterStatistics statistics_;
    
    // Noise generation
    std::unique_ptr<utils::NoiseGenerator<T>> noise_generator_;
};

// Type aliases
using KalmanFilterd = KalmanFilter<double>;
using KalmanFilterf = KalmanFilter<float>;

/**
 * @brief Extended Kalman Filter for nonlinear systems (future extension)
 */
template<std::floating_point T = double>
class ExtendedKalmanFilter : public KalmanFilter<T> {
public:
    // Extended functionality can be added here for nonlinear DC motor models
    // This provides a foundation for future extensions
};

// Forward declarations for utility functions
template<std::floating_point T>
math::Matrix<T, 2, 1> compute_steady_state_gain(
    const math::Matrix<T, 2, 2>& F,
    const math::Matrix<T, 1, 2>& H,
    const math::Matrix<T, 2, 2>& Q,
    const math::Matrix<T, 1, 1>& R);

template<std::floating_point T>
std::size_t analyze_observability(
    const math::Matrix<T, 2, 2>& F,
    const math::Matrix<T, 1, 2>& H);

template<std::floating_point T>
std::pair<bool, T> validate_innovation_whiteness(const std::vector<T>& innovations);

} // namespace kf::models