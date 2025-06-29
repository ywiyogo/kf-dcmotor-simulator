#include "SimulationEngine.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>

namespace kf::simulation {

// Explicit template instantiations for common floating point types
template class SimulationEngine<double>;
template class SimulationEngine<float>;

// Additional utility functions for simulation analysis

/**
 * @brief Compute simulation quality metrics
 * @tparam T Floating point type
 * @param data_points Simulation data points
 * @return Quality metrics {signal_to_noise_ratio, estimation_accuracy, convergence_rate}
 */
template<std::floating_point T>
std::array<T, 3> compute_simulation_quality(const std::vector<SimulationDataPoint<T>>& data_points) {
    if (data_points.empty()) {
        return {T{0}, T{0}, T{0}};
    }
    
    // Signal-to-noise ratio (true signal variance / measurement noise variance)
    T true_signal_variance = T{0};
    T measurement_noise_variance = T{0};
    T true_mean = T{0};
    T measurement_noise_mean = T{0};
    
    std::size_t n = data_points.size();
    
    // Compute means
    for (const auto& point : data_points) {
        true_mean += point.true_position;
        measurement_noise_mean += (point.measured_position - point.true_position);
    }
    true_mean /= n;
    measurement_noise_mean /= n;
    
    // Compute variances
    for (const auto& point : data_points) {
        T true_centered = point.true_position - true_mean;
        T noise_centered = (point.measured_position - point.true_position) - measurement_noise_mean;
        true_signal_variance += true_centered * true_centered;
        measurement_noise_variance += noise_centered * noise_centered;
    }
    true_signal_variance /= (n - 1);
    measurement_noise_variance /= (n - 1);
    
    T snr = (measurement_noise_variance > T{0}) ? 
            T{10} * std::log10(true_signal_variance / measurement_noise_variance) : T{0};
    
    // Estimation accuracy (ratio of estimate error to measurement error)
    T total_estimation_error = T{0};
    T total_measurement_error = T{0};
    
    for (const auto& point : data_points) {
        total_estimation_error += point.position_error * point.position_error;
        T measurement_error = std::abs(point.measured_position - point.true_position);
        total_measurement_error += measurement_error * measurement_error;
    }
    
    T estimation_accuracy = (total_measurement_error > T{0}) ? 
                           std::sqrt(total_estimation_error / total_measurement_error) : T{1};
    
    // Convergence rate (exponential decay constant)
    T convergence_rate = T{0};
    if (n > 10) {
        // Find where error drops to 1/e of initial value
        T initial_error = data_points[0].position_error + data_points[0].velocity_error;
        T target_error = initial_error / std::numbers::e;
        
        for (std::size_t i = 1; i < n; ++i) {
            T current_error = data_points[i].position_error + data_points[i].velocity_error;
            if (current_error < target_error) {
                convergence_rate = T{1} / data_points[i].time;
                break;
            }
        }
    }
    
    return {snr, estimation_accuracy, convergence_rate};
}

/**
 * @brief Analyze filter stability over time
 * @tparam T Floating point type
 * @param data_points Simulation data points
 * @return Stability metrics {max_position_drift, max_velocity_drift, covariance_growth_rate}
 */
template<std::floating_point T>
std::array<T, 3> analyze_filter_stability(const std::vector<SimulationDataPoint<T>>& data_points) {
    if (data_points.size() < 2) {
        return {T{0}, T{0}, T{0}};
    }
    
    T max_position_drift = T{0};
    T max_velocity_drift = T{0};
    T covariance_growth_rate = T{0};
    
    // Track maximum estimation drift from true values
    for (const auto& point : data_points) {
        max_position_drift = std::max(max_position_drift, point.position_error);
        max_velocity_drift = std::max(max_velocity_drift, point.velocity_error);
    }
    
    // Estimate covariance growth rate
    if (data_points.size() > 10) {
        T initial_uncertainty = data_points[0].position_uncertainty + data_points[0].velocity_uncertainty;
        T final_uncertainty = data_points.back().position_uncertainty + data_points.back().velocity_uncertainty;
        T time_span = data_points.back().time - data_points[0].time;
        
        if (time_span > T{0} && initial_uncertainty > T{0}) {
            covariance_growth_rate = std::log(final_uncertainty / initial_uncertainty) / time_span;
        }
    }
    
    return {max_position_drift, max_velocity_drift, covariance_growth_rate};
}

/**
 * @brief Compute power spectral density of estimation errors
 * @tparam T Floating point type
 * @param data_points Simulation data points
 * @param sampling_rate Sampling rate [Hz]
 * @return {frequencies, position_error_psd, velocity_error_psd}
 */
template<std::floating_point T>
std::tuple<std::vector<T>, std::vector<T>, std::vector<T>> 
compute_error_spectrum(const std::vector<SimulationDataPoint<T>>& data_points, T sampling_rate) {
    std::vector<T> position_errors;
    std::vector<T> velocity_errors;
    
    position_errors.reserve(data_points.size());
    velocity_errors.reserve(data_points.size());
    
    for (const auto& point : data_points) {
        position_errors.push_back(point.position_error);
        velocity_errors.push_back(point.velocity_error);
    }
    
    // Simple DFT-based PSD computation
    std::size_t n = data_points.size();
    std::vector<T> frequencies;
    std::vector<T> pos_psd;
    std::vector<T> vel_psd;
    
    if (n < 2) {
        return {frequencies, pos_psd, vel_psd};
    }
    
    std::size_t n_freq = n / 2 + 1;
    frequencies.reserve(n_freq);
    pos_psd.reserve(n_freq);
    vel_psd.reserve(n_freq);
    
    for (std::size_t k = 0; k < n_freq; ++k) {
        T freq = k * sampling_rate / n;
        frequencies.push_back(freq);
        
        // Compute DFT coefficients
        T pos_real = T{0}, pos_imag = T{0};
        T vel_real = T{0}, vel_imag = T{0};
        
        for (std::size_t m = 0; m < n; ++m) {
            T angle = -T{2} * std::numbers::pi * k * m / n;
            T cos_angle = std::cos(angle);
            T sin_angle = std::sin(angle);
            
            pos_real += position_errors[m] * cos_angle;
            pos_imag += position_errors[m] * sin_angle;
            vel_real += velocity_errors[m] * cos_angle;
            vel_imag += velocity_errors[m] * sin_angle;
        }
        
        T pos_mag_sq = pos_real * pos_real + pos_imag * pos_imag;
        T vel_mag_sq = vel_real * vel_real + vel_imag * vel_imag;
        
        T pos_psd_val = pos_mag_sq / (sampling_rate * n);
        T vel_psd_val = vel_mag_sq / (sampling_rate * n);
        
        // Scale non-DC components
        if (k > 0 && k < n/2) {
            pos_psd_val *= T{2};
            vel_psd_val *= T{2};
        }
        
        pos_psd.push_back(pos_psd_val);
        vel_psd.push_back(vel_psd_val);
    }
    
    return {frequencies, pos_psd, vel_psd};
}

/**
 * @brief Generate simulation report
 * @tparam T Floating point type
 * @param statistics Simulation statistics
 * @param data_points Simulation data points
 * @return Formatted report string
 */
template<std::floating_point T>
std::string generate_simulation_report(
    const SimulationStatistics<T>& statistics,
    const std::vector<SimulationDataPoint<T>>& data_points) {
    
    std::stringstream report;
    
    report << "=== KALMAN FILTER SIMULATION REPORT ===\n\n";
    
    // Basic simulation info
    report << "Simulation Duration: " << utils::TimeFormatter::format_duration(data_points.empty() ? T{0} : data_points.back().time) << "\n";
    report << "Total Updates: " << statistics.total_updates << "\n";
    report << "Failed Updates: " << statistics.failed_updates << "\n";
    report << "Success Rate: " << std::fixed << std::setprecision(2) 
           << (100.0 * (statistics.total_updates - statistics.failed_updates) / std::max(statistics.total_updates, std::size_t{1})) << "%\n\n";
    
    // Performance metrics
    report << "=== ESTIMATION PERFORMANCE ===\n";
    report << "Position RMSE: " << std::scientific << std::setprecision(3) << statistics.rmse_position << " rad\n";
    report << "Velocity RMSE: " << std::scientific << std::setprecision(3) << statistics.rmse_velocity << " rad/s\n";
    report << "Position MAE: " << std::scientific << std::setprecision(3) << statistics.mean_absolute_error_position << " rad\n";
    report << "Velocity MAE: " << std::scientific << std::setprecision(3) << statistics.mean_absolute_error_velocity << " rad/s\n";
    report << "Convergence Time: " << utils::TimeFormatter::format_duration(statistics.convergence_time) << "\n";
    report << "Average Log-Likelihood: " << std::fixed << std::setprecision(3) << statistics.average_log_likelihood << "\n\n";
    
    // Filter health
    report << "=== FILTER HEALTH ===\n";
    report << "Filter Converged: " << (statistics.filter_converged ? "Yes" : "No") << "\n";
    report << "Filter Stable: " << (statistics.filter_stable ? "Yes" : "No") << "\n";
    report << "Innovation Whiteness: " << std::fixed << std::setprecision(4) << statistics.innovation_whiteness_metric << "\n";
    
    // Quality assessment
    auto quality = compute_simulation_quality(data_points);
    report << "Signal-to-Noise Ratio: " << std::fixed << std::setprecision(2) << quality[0] << " dB\n";
    report << "Estimation Accuracy: " << std::fixed << std::setprecision(4) << quality[1] << "\n";
    report << "Convergence Rate: " << std::fixed << std::setprecision(4) << quality[2] << " rad⁻¹\n\n";
    
    // Motor performance
    report << "=== MOTOR PERFORMANCE ===\n";
    report << "Average Efficiency: " << std::fixed << std::setprecision(2) << (statistics.average_efficiency * 100) << "%\n";
    report << "Peak Current: " << std::fixed << std::setprecision(3) << statistics.peak_current << " A\n";
    report << "Total Energy Consumed: " << std::fixed << std::setprecision(3) << statistics.total_energy_consumed << " J\n";
    report << "Total Mechanical Work: " << std::fixed << std::setprecision(3) << statistics.total_mechanical_work << " J\n\n";
    
    // Computational performance
    report << "=== COMPUTATIONAL PERFORMANCE ===\n";
    report << "Average Update Time: " << statistics.average_update_time.count() << " μs\n";
    report << "Update Rate: " << std::fixed << std::setprecision(1) 
           << (1e6 / std::max(statistics.average_update_time.count(), 1L)) << " Hz\n\n";
    
    // Stability analysis
    auto stability = analyze_filter_stability(data_points);
    report << "=== STABILITY ANALYSIS ===\n";
    report << "Max Position Drift: " << std::scientific << std::setprecision(3) << stability[0] << " rad\n";
    report << "Max Velocity Drift: " << std::scientific << std::setprecision(3) << stability[1] << " rad/s\n";
    report << "Covariance Growth Rate: " << std::scientific << std::setprecision(3) << stability[2] << " s⁻¹\n\n";
    
    return report.str();
}

/**
 * @brief Export simulation data to CSV format
 * @tparam T Floating point type
 * @param data_points Simulation data points
 * @return CSV formatted string
 */
template<std::floating_point T>
std::string export_data_to_csv(const std::vector<SimulationDataPoint<T>>& data_points) {
    std::stringstream csv;
    
    // Header
    csv << "time,true_position,true_velocity,measured_position,estimated_position,estimated_velocity,";
    csv << "position_uncertainty,velocity_uncertainty,motor_current,motor_voltage,motor_torque,";
    csv << "innovation,log_likelihood,electrical_power,mechanical_power,efficiency,";
    csv << "position_error,velocity_error,normalized_innovation\n";
    
    // Data
    for (const auto& point : data_points) {
        csv << std::fixed << std::setprecision(6)
            << point.time << ","
            << point.true_position << ","
            << point.true_velocity << ","
            << point.measured_position << ","
            << point.estimated_position << ","
            << point.estimated_velocity << ","
            << point.position_uncertainty << ","
            << point.velocity_uncertainty << ","
            << point.motor_current << ","
            << point.motor_voltage << ","
            << point.motor_torque << ","
            << point.innovation << ","
            << point.log_likelihood << ","
            << point.electrical_power << ","
            << point.mechanical_power << ","
            << point.efficiency << ","
            << point.position_error << ","
            << point.velocity_error << ","
            << point.normalized_innovation << "\n";
    }
    
    return csv.str();
}

/**
 * @brief Create optimized simulation configuration for specific scenarios
 * @tparam T Floating point type
 * @param scenario Scenario type
 * @return Optimized configuration
 */
template<std::floating_point T>
void apply_optimized_config(const std::string& scenario) {
    auto& config = config::config();
    
    if (scenario == "high_accuracy") {
        // High accuracy scenario - small time steps, low noise
        config.simulation().time_step = 0.001;
        config.kalman().process_noise_std = 0.01;
        config.kalman().measurement_noise_std = 0.05;
        config.simulation().max_data_points = 10000;
        
    } else if (scenario == "real_time") {
        // Real-time scenario - larger time steps, moderate noise
        config.simulation().time_step = 0.01;
        config.kalman().process_noise_std = 0.1;
        config.kalman().measurement_noise_std = 0.2;
        config.simulation().real_time_mode = true;
        config.simulation().max_data_points = 1000;
        
    } else if (scenario == "noisy_environment") {
        // Noisy environment - higher noise levels
        config.simulation().time_step = 0.01;
        config.kalman().process_noise_std = 0.5;
        config.kalman().measurement_noise_std = 1.0;
        config.simulation().max_data_points = 2000;
        
    } else if (scenario == "fast_dynamics") {
        // Fast dynamics - small time steps, moderate noise
        config.simulation().time_step = 0.0005;
        config.kalman().process_noise_std = 0.2;
        config.kalman().measurement_noise_std = 0.3;
        config.motor().input_voltage_range = 48.0; // Higher voltage for faster response
        config.simulation().max_data_points = 5000;
        
    } else {
        // Default balanced scenario
        config.simulation().time_step = 0.01;
        config.kalman().process_noise_std = 0.1;
        config.kalman().measurement_noise_std = 0.2;
        config.simulation().max_data_points = 1000;
    }
}

// Explicit instantiations for utility functions
template std::array<double, 3> compute_simulation_quality<double>(const std::vector<SimulationDataPoint<double>>&);
template std::array<float, 3> compute_simulation_quality<float>(const std::vector<SimulationDataPoint<float>>&);

template std::array<double, 3> analyze_filter_stability<double>(const std::vector<SimulationDataPoint<double>>&);
template std::array<float, 3> analyze_filter_stability<float>(const std::vector<SimulationDataPoint<float>>&);

template std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> 
compute_error_spectrum<double>(const std::vector<SimulationDataPoint<double>>&, double);
template std::tuple<std::vector<float>, std::vector<float>, std::vector<float>> 
compute_error_spectrum<float>(const std::vector<SimulationDataPoint<float>>&, float);

template std::string generate_simulation_report<double>(
    const SimulationStatistics<double>&, const std::vector<SimulationDataPoint<double>>&);
template std::string generate_simulation_report<float>(
    const SimulationStatistics<float>&, const std::vector<SimulationDataPoint<float>>&);

template std::string export_data_to_csv<double>(const std::vector<SimulationDataPoint<double>>&);
template std::string export_data_to_csv<float>(const std::vector<SimulationDataPoint<float>>&);

template void apply_optimized_config<double>(const std::string&);
template void apply_optimized_config<float>(const std::string&);

} // namespace kf::simulation