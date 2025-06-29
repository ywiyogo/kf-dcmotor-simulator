#pragma once

#include "../models/DCMotorModel.hpp"
#include "../models/KalmanFilter.hpp"
#include "../config/Configuration.hpp"
#include "../utils/NoiseGenerator.hpp"
#include "../utils/TimeUtils.hpp"
#include <vector>
#include <memory>
#include <chrono>
#include <concepts>
#include <expected>
#include <string>
#include <functional>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace kf::simulation {

/**
 * @brief Simulation data point containing all relevant information at a time step
 */
template<std::floating_point T = double>
struct SimulationDataPoint {
    T time = T{0};                              // Simulation time [s]
    T true_position = T{0};                     // True angular position [rad]
    T true_velocity = T{0};                     // True angular velocity [rad/s]
    T measured_position = T{0};                 // Noisy position measurement [rad]
    T estimated_position = T{0};                // Kalman filter position estimate [rad]
    T estimated_velocity = T{0};                // Kalman filter velocity estimate [rad/s]
    T position_uncertainty = T{0};              // Position estimate uncertainty [rad]
    T velocity_uncertainty = T{0};              // Velocity estimate uncertainty [rad/s]
    T motor_current = T{0};                     // Motor armature current [A]
    T motor_voltage = T{0};                     // Applied motor voltage [V]
    T motor_torque = T{0};                      // Motor torque [Nm]
    T innovation = T{0};                        // Kalman filter innovation
    T log_likelihood = T{0};                    // Measurement log-likelihood

    // Motor efficiency and power metrics
    T electrical_power = T{0};                  // Electrical power [W]
    T mechanical_power = T{0};                  // Mechanical power [W]
    T efficiency = T{0};                        // Motor efficiency [0-1]

    // Performance metrics
    T position_error = T{0};                    // |true_position - estimated_position|
    T velocity_error = T{0};                    // |true_velocity - estimated_velocity|
    T normalized_innovation = T{0};             // Normalized innovation squared
};

/**
 * @brief Simulation statistics for performance analysis
 */
template<std::floating_point T = double>
struct SimulationStatistics {
    T rmse_position = T{0};                     // Root mean square error for position
    T rmse_velocity = T{0};                     // Root mean square error for velocity
    T mean_absolute_error_position = T{0};      // Mean absolute error for position
    T mean_absolute_error_velocity = T{0};      // Mean absolute error for velocity
    T convergence_time = T{0};                  // Time to convergence [s]
    T average_log_likelihood = T{0};            // Average measurement log-likelihood
    T innovation_whiteness_metric = T{0};       // Innovation whiteness test result
    bool filter_converged = false;              // Whether filter has converged
    bool filter_stable = false;                 // Whether filter remains stable

    // Motor performance statistics
    T average_efficiency = T{0};                // Average motor efficiency
    T peak_current = T{0};                      // Peak motor current [A]
    T total_energy_consumed = T{0};             // Total electrical energy [J]
    T total_mechanical_work = T{0};             // Total mechanical work [J]

    // Computational statistics
    std::chrono::microseconds average_update_time{0}; // Average update computation time
    std::size_t total_updates = 0;              // Total number of updates performed
    std::size_t failed_updates = 0;             // Number of failed updates
};

/**
 * @brief Main simulation engine orchestrating DC motor model and Kalman filter
 *
 * This class provides a complete simulation environment for testing and
 * understanding Kalman filter performance on DC motor state estimation.
 * It supports real-time and batch simulation modes with comprehensive
 * data collection and analysis capabilities.
 */
template<std::floating_point T = double>
class SimulationEngine {
public:
    using DataPoint = SimulationDataPoint<T>;
    using Statistics = SimulationStatistics<T>;
    using DataVector = std::vector<DataPoint>;
    using MotorModel = models::DCMotorModel<T>;
    using KalmanFilterType = models::KalmanFilter<T>;
    using NoiseGen = utils::NoiseGenerator<T>;
    using UpdateCallback = std::function<void(const DataPoint&)>;
    using CompletionCallback = std::function<void(const Statistics&)>;

    enum class SimulationMode {
        REAL_TIME,      // Real-time simulation with timing constraints
        BATCH,          // Fast batch simulation
        INTERACTIVE     // Interactive mode with user control
    };

    enum class InputProfile {
        STEP,           // Step input
        RAMP,           // Ramp input
        SINUSOIDAL,     // Sinusoidal input
        CHIRP,          // Frequency sweep
        PRBS,           // Pseudo-random binary sequence
        CUSTOM          // User-defined input function
    };

    /**
     * @brief Constructor with configuration
     * @param config Simulation configuration
     */
    explicit SimulationEngine(const config::Configuration& config = config::config())
        : config_(config),
          motor_(config.motor()),
          kalman_filter_(static_cast<T>(config.simulation().time_step), config.kalman()),
          noise_generator_(),
          running_(false),
          paused_(false),
          current_time_(T{0}),
          mode_(SimulationMode::BATCH) {

        initialize();
    }

    /**
     * @brief Destructor ensures proper cleanup
     */
    ~SimulationEngine() {
        stop();
    }

    /**
     * @brief Initialize simulation with current configuration
     */
    void initialize() {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // Clear previous data
        data_points_.clear();
        data_points_.reserve(config_.simulation().max_data_points);

        // Reset models
        motor_.reset();
        kalman_filter_.reset();

        // Initialize motor state
        typename MotorModel::State initial_motor_state{};
        initial_motor_state(MotorModel::POSITION_IDX, 0) = static_cast<T>(config_.kalman().initial_state[0]);
        initial_motor_state(MotorModel::VELOCITY_IDX, 0) = static_cast<T>(config_.kalman().initial_state[1]);
        initial_motor_state(MotorModel::CURRENT_IDX, 0) = T{0};
        initial_motor_state(MotorModel::VOLTAGE_IDX, 0) = T{0};
        motor_.set_initial_state(initial_motor_state);

        // Reset time and statistics
        current_time_ = T{0};
        statistics_ = Statistics{};

        // Initialize noise generator with reproducible seed for testing
        noise_generator_.reset(12345); // Fixed seed for reproducible results
    }

    /**
     * @brief Start simulation
     * @param mode Simulation mode
     * @param duration Simulation duration [s] (0 for infinite)
     * @return Expected void or error message
     */
    std::expected<void, std::string> start(SimulationMode mode = SimulationMode::BATCH,
                                          T duration = T{0}) {
        if (running_.load()) {
            return std::unexpected("Simulation is already running");
        }

        mode_ = mode;
        if (duration <= T{0}) {
            duration = static_cast<T>(config_.simulation().simulation_duration);
        }
        simulation_duration_ = duration;

        running_.store(true);
        paused_.store(false);

        try {
            switch (mode_) {
                case SimulationMode::REAL_TIME:
                    simulation_thread_ = std::thread(&SimulationEngine::run_real_time, this);
                    break;
                case SimulationMode::BATCH:
                    simulation_thread_ = std::thread(&SimulationEngine::run_batch, this);
                    break;
                case SimulationMode::INTERACTIVE:
                    simulation_thread_ = std::thread(&SimulationEngine::run_interactive, this);
                    break;
            }
            return {};
        } catch (const std::exception& e) {
            running_.store(false);
            return std::unexpected(std::string("Failed to start simulation: ") + e.what());
        }
    }

    /**
     * @brief Stop simulation
     */
    void stop() {
        running_.store(false);
        simulation_cv_.notify_all();

        if (simulation_thread_.joinable()) {
            simulation_thread_.join();
        }

        // Finalize statistics
        finalize_statistics();

        // Notify completion
        if (completion_callback_) {
            completion_callback_(statistics_);
        }
    }

    /**
     * @brief Pause/resume simulation (real-time and interactive modes only)
     */
    void pause() { paused_.store(true); }
    void resume() {
        paused_.store(false);
        simulation_cv_.notify_all();
    }
    bool is_paused() const { return paused_.load(); }
    bool is_running() const { return running_.load(); }

    /**
     * @brief Set input profile for simulation
     * @param profile Input profile type
     * @param parameters Profile-specific parameters
     */
    void set_input_profile(InputProfile profile, const std::vector<T>& parameters = {}) {
        input_profile_ = profile;
        input_parameters_ = parameters;
    }

    /**
     * @brief Set custom input function
     * @param input_func Function that takes time and returns voltage
     */
    void set_custom_input(std::function<T(T)> input_func) {
        custom_input_func_ = input_func;
        input_profile_ = InputProfile::CUSTOM;
    }

    /**
     * @brief Register callback for real-time data updates
     * @param callback Function called for each simulation step
     */
    void set_update_callback(UpdateCallback callback) {
        update_callback_ = callback;
    }

    /**
     * @brief Register callback for simulation completion
     * @param callback Function called when simulation completes
     */
    void set_completion_callback(CompletionCallback callback) {
        completion_callback_ = callback;
    }

    /**
     * @brief Get current simulation data (thread-safe)
     * @return Copy of current data points
     */
    DataVector get_data() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return data_points_;
    }

    /**
     * @brief Get latest data point
     * @return Latest data point or nullopt if no data
     */
    std::optional<DataPoint> get_latest_data() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return data_points_.empty() ? std::nullopt : std::make_optional(data_points_.back());
    }

    /**
     * @brief Get current simulation statistics
     * @return Current statistics
     */
    const Statistics& get_statistics() const {
        return statistics_;
    }

    /**
     * @brief Get current simulation time
     * @return Current simulation time [s]
     */
    T get_current_time() const {
        return current_time_.load();
    }

    /**
     * @brief Get simulation progress [0-1]
     * @return Progress ratio
     */
    T get_progress() const {
        T duration = simulation_duration_;
        if (duration <= T{0}) return T{0};
        return std::min(current_time_.load() / duration, T{1});
    }

    /**
     * @brief Perform single simulation step manually
     * @param input_voltage Motor input voltage [V]
     * @return Data point for this step
     */
    std::expected<DataPoint, std::string> step(T input_voltage = T{0}) {
        return perform_simulation_step(input_voltage);
    }

    /**
     * @brief Update models with current configuration
     */
    void update_configuration() {
        // Update models with current parameters
        motor_.update_parameters(config_.motor());
        kalman_filter_.set_process_noise(static_cast<T>(config_.kalman().process_noise_std));
        kalman_filter_.set_measurement_noise(static_cast<T>(config_.kalman().measurement_noise_std));
        kalman_filter_.set_time_step(static_cast<T>(config_.simulation().time_step));
    }

private:
    /**
     * @brief Real-time simulation loop
     */
    void run_real_time() {
        T dt = static_cast<T>(config_.simulation().time_step);
        auto dt_chrono = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::duration<T>(dt));

        while (running_.load() && current_time_.load() < simulation_duration_) {
            auto start_time = std::chrono::steady_clock::now();

            // Wait if paused
            std::unique_lock<std::mutex> lock(simulation_mutex_);
            simulation_cv_.wait(lock, [this]() { return !paused_.load() || !running_.load(); });
            lock.unlock();

            if (!running_.load()) break;

            // Perform simulation step
            T input = compute_input_voltage(current_time_.load());
            auto result = perform_simulation_step(input);

            if (!result) {
                // Handle error but continue simulation
                statistics_.failed_updates++;
            }

            // Maintain real-time timing
            auto end_time = std::chrono::steady_clock::now();
            auto elapsed = end_time - start_time;

            if (elapsed < dt_chrono) {
                std::this_thread::sleep_for(dt_chrono - elapsed);
            }

            current_time_ += dt;
        }

        running_.store(false);
    }

    /**
     * @brief Batch simulation loop (no timing constraints)
     */
    void run_batch() {
        T dt = static_cast<T>(config_.simulation().time_step);

        while (running_.load() && current_time_.load() < simulation_duration_) {
            T input = compute_input_voltage(current_time_.load());
            auto result = perform_simulation_step(input);

            if (!result) {
                statistics_.failed_updates++;
            }

            current_time_ += dt;
        }

        running_.store(false);
    }

    /**
     * @brief Interactive simulation loop
     */
    void run_interactive() {
        T dt = static_cast<T>(config_.simulation().time_step);

        while (running_.load()) {
            // Wait for external trigger or timeout
            std::unique_lock<std::mutex> lock(simulation_mutex_);
            simulation_cv_.wait_for(lock, std::chrono::milliseconds(100),
                                  [this]() { return !running_.load(); });
            lock.unlock();

            if (!running_.load()) break;

            if (!paused_.load() && current_time_.load() < simulation_duration_) {
                T input = compute_input_voltage(current_time_.load());
                auto result = perform_simulation_step(input);

                if (!result) {
                    statistics_.failed_updates++;
                }

                current_time_ += dt;
            }
        }

        running_.store(false);
    }

    /**
     * @brief Perform a single simulation step
     * @param input_voltage Motor input voltage [V]
     * @return Data point or error
     */
    std::expected<DataPoint, std::string> perform_simulation_step(T input_voltage) {
        auto step_start = std::chrono::steady_clock::now();

        try {
            DataPoint data_point;
            data_point.time = current_time_.load();
            data_point.motor_voltage = input_voltage;

            // Update motor model
            motor_.set_input_voltage(input_voltage);

            // Extract true values
            data_point.true_position = motor_.get_position();
            data_point.true_velocity = motor_.get_velocity();
            data_point.motor_current = motor_.get_current();
            data_point.motor_torque = motor_.get_torque();
            data_point.electrical_power = motor_.get_electrical_power();
            data_point.mechanical_power = motor_.get_mechanical_power();
            data_point.efficiency = motor_.get_efficiency();

            // Generate noisy measurement
            data_point.measured_position = kalman_filter_.generate_measurement(data_point.true_position);

            // Kalman filter prediction step
            kalman_filter_.predict();

            // Kalman filter update step
            auto update_result = kalman_filter_.update(data_point.measured_position);
            if (!update_result) {
                return std::unexpected(update_result.error());
            }

            // Extract estimates
            data_point.estimated_position = kalman_filter_.get_position();
            data_point.estimated_velocity = kalman_filter_.get_velocity();
            data_point.position_uncertainty = kalman_filter_.get_position_uncertainty();
            data_point.velocity_uncertainty = kalman_filter_.get_velocity_uncertainty();

            // Compute performance metrics
            data_point.position_error = std::abs(data_point.true_position - data_point.estimated_position);
            data_point.velocity_error = std::abs(data_point.true_velocity - data_point.estimated_velocity);
            data_point.innovation = kalman_filter_.compute_residual(data_point.measured_position);
            data_point.log_likelihood = kalman_filter_.compute_log_likelihood(data_point.measured_position);

            const auto& kf_stats = kalman_filter_.get_statistics();
            data_point.normalized_innovation = kf_stats.normalized_innovation;

            // Record timing
            auto step_end = std::chrono::steady_clock::now();
            auto step_duration = std::chrono::duration_cast<std::chrono::microseconds>(step_end - step_start);

            // Update statistics
            update_statistics(data_point, step_duration);

            // Store data point
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                data_points_.push_back(data_point);

                // Maintain data size limit
                if (data_points_.size() > config_.simulation().max_data_points) {
                    data_points_.erase(data_points_.begin());
                }
            }

            // Notify callback
            if (update_callback_) {
                update_callback_(data_point);
            }

            return data_point;

        } catch (const std::exception& e) {
            return std::unexpected(std::string("Simulation step failed: ") + e.what());
        }
    }

    /**
     * @brief Compute input voltage based on selected profile
     * @param time Current simulation time [s]
     * @return Input voltage [V]
     */
    T compute_input_voltage(T time) const {
        switch (input_profile_) {
            case InputProfile::STEP: {
                T amplitude = input_parameters_.empty() ? T{12} : input_parameters_[0];
                T step_time = input_parameters_.size() < 2 ? T{1} : input_parameters_[1];
                return NoiseGen::step_signal(amplitude, step_time, time);
            }

            case InputProfile::RAMP: {
                T slope = input_parameters_.empty() ? T{5} : input_parameters_[0];
                T start_time = input_parameters_.size() < 2 ? T{0} : input_parameters_[1];
                return NoiseGen::ramp_signal(slope, start_time, time);
            }

            case InputProfile::SINUSOIDAL: {
                T amplitude = input_parameters_.empty() ? T{10} : input_parameters_[0];
                T frequency = input_parameters_.size() < 2 ? T{0.5} : input_parameters_[1];
                T phase = input_parameters_.size() < 3 ? T{0} : input_parameters_[2];
                return NoiseGen::deterministic_sinusoidal(amplitude, frequency, phase, time);
            }

            case InputProfile::CHIRP: {
                T amplitude = input_parameters_.empty() ? T{8} : input_parameters_[0];
                T f0 = input_parameters_.size() < 2 ? T{0.1} : input_parameters_[1];
                T f1 = input_parameters_.size() < 3 ? T{2.0} : input_parameters_[2];
                T duration = input_parameters_.size() < 4 ? simulation_duration_ : input_parameters_[3];
                return NoiseGen::chirp_signal(amplitude, f0, f1, duration, time);
            }

            case InputProfile::PRBS: {
                T amplitude = input_parameters_.empty() ? T{6} : input_parameters_[0];
                T period = input_parameters_.size() < 2 ? T{0.5} : input_parameters_[1];
                return utils::SignalGenerator<T>::prbs_signal(amplitude, period, time);
            }

            case InputProfile::CUSTOM:
                return custom_input_func_ ? custom_input_func_(time) : T{0};

            default:
                return T{0};
        }
    }

    /**
     * @brief Update running statistics
     * @param data_point Latest data point
     * @param computation_time Computation time for this step
     */
    void update_statistics(const DataPoint& data_point, std::chrono::microseconds computation_time) {
        statistics_.total_updates++;

        // Update RMSE (running average)
        T n = static_cast<T>(statistics_.total_updates);
        statistics_.rmse_position = std::sqrt(((n - T{1}) * statistics_.rmse_position * statistics_.rmse_position +
                                             data_point.position_error * data_point.position_error) / n);
        statistics_.rmse_velocity = std::sqrt(((n - T{1}) * statistics_.rmse_velocity * statistics_.rmse_velocity +
                                             data_point.velocity_error * data_point.velocity_error) / n);

        // Update MAE
        statistics_.mean_absolute_error_position = ((n - T{1}) * statistics_.mean_absolute_error_position +
                                                   data_point.position_error) / n;
        statistics_.mean_absolute_error_velocity = ((n - T{1}) * statistics_.mean_absolute_error_velocity +
                                                   data_point.velocity_error) / n;

        // Update other metrics
        statistics_.average_log_likelihood = ((n - T{1}) * statistics_.average_log_likelihood +
                                            data_point.log_likelihood) / n;
        statistics_.average_efficiency = ((n - T{1}) * statistics_.average_efficiency +
                                        data_point.efficiency) / n;

        // Update peak values
        statistics_.peak_current = std::max(statistics_.peak_current, std::abs(data_point.motor_current));

        // Update energy integration (trapezoidal rule)
        T dt = static_cast<T>(config_.simulation().time_step);
        statistics_.total_energy_consumed += data_point.electrical_power * dt;
        statistics_.total_mechanical_work += data_point.mechanical_power * dt;

        // Update computation timing
        statistics_.average_update_time = std::chrono::microseconds(
            (static_cast<std::size_t>(n - 1) * statistics_.average_update_time.count() +
             computation_time.count()) / static_cast<std::size_t>(n));

        // Check convergence
        const auto& kf_stats = kalman_filter_.get_statistics();
        statistics_.filter_converged = kf_stats.is_converged;
        statistics_.filter_stable = kf_stats.condition_number < T{1e12};
    }

    /**
     * @brief Finalize statistics at end of simulation
     */
    void finalize_statistics() {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (data_points_.empty()) return;

        // Compute convergence time
        T error_threshold = T{0.05} * (statistics_.rmse_position + statistics_.rmse_velocity);
        for (const auto& point : data_points_) {
            if (point.position_error + point.velocity_error < error_threshold) {
                statistics_.convergence_time = point.time;
                break;
            }
        }

        if (statistics_.convergence_time == T{0}) {
            statistics_.convergence_time = data_points_.back().time;
        }

        // Compute innovation whiteness metric
        std::vector<T> innovations;
        innovations.reserve(data_points_.size());
        for (const auto& point : data_points_) {
            innovations.push_back(point.innovation);
        }

        auto [is_white, autocorr] = models::validate_innovation_whiteness(innovations);
        statistics_.innovation_whiteness_metric = autocorr;
    }

    // Configuration and models
    const config::Configuration& config_;
    MotorModel motor_;
    KalmanFilterType kalman_filter_;
    NoiseGen noise_generator_;

    // Simulation state
    std::atomic<bool> running_;
    std::atomic<bool> paused_;
    std::atomic<T> current_time_;
    T simulation_duration_;
    SimulationMode mode_;

    // Input generation
    InputProfile input_profile_ = InputProfile::STEP;
    std::vector<T> input_parameters_;
    std::function<T(T)> custom_input_func_;

    // Data storage
    mutable std::mutex data_mutex_;
    DataVector data_points_;
    Statistics statistics_;

    // Threading
    std::thread simulation_thread_;
    std::mutex simulation_mutex_;
    std::condition_variable simulation_cv_;

    // Callbacks
    UpdateCallback update_callback_;
    CompletionCallback completion_callback_;
};

// Type aliases
using SimulationEngined = SimulationEngine<double>;
using SimulationEnginef = SimulationEngine<float>;
using SimulationDataPointd = SimulationDataPoint<double>;
using SimulationDataPointf = SimulationDataPoint<float>;
using SimulationStatisticsd = SimulationStatistics<double>;
using SimulationStatisticsf = SimulationStatistics<float>;

} // namespace kf::simulation
