#include "NoiseGenerator.hpp"
#include <thread>
#include <stdexcept>
#include <algorithm>
#include <numeric>

namespace kf::utils {

// Explicit template instantiations for common floating point types
template class NoiseGenerator<double>;
template class NoiseGenerator<float>;
template class SignalGenerator<double>;
template class SignalGenerator<float>;

// Additional utility functions for noise analysis and generation

/**
 * @brief Compute power spectral density of a signal
 * @tparam T Floating point type
 * @param signal Input signal samples
 * @param sampling_rate Sampling rate [Hz]
 * @return {frequencies, psd_values}
 */
template<std::floating_point T>
std::pair<std::vector<T>, std::vector<T>> compute_psd(
    const std::vector<T>& signal,
    T sampling_rate) {
    
    std::size_t n = signal.size();
    if (n < 2) {
        return {{}, {}};
    }
    
    // Simple periodogram method
    std::vector<T> frequencies;
    std::vector<T> psd_values;
    
    std::size_t n_freq = n / 2 + 1;
    frequencies.reserve(n_freq);
    psd_values.reserve(n_freq);
    
    for (std::size_t k = 0; k < n_freq; ++k) {
        T freq = k * sampling_rate / n;
        frequencies.push_back(freq);
        
        // Compute DFT coefficient (simplified)
        T real_part = T{0};
        T imag_part = T{0};
        
        for (std::size_t m = 0; m < n; ++m) {
            T angle = -T{2} * std::numbers::pi * k * m / n;
            real_part += signal[m] * std::cos(angle);
            imag_part += signal[m] * std::sin(angle);
        }
        
        T magnitude_sq = real_part * real_part + imag_part * imag_part;
        T psd = magnitude_sq / (sampling_rate * n);
        
        // Scale DC and Nyquist components
        if (k == 0 || (k == n/2 && n % 2 == 0)) {
            psd_values.push_back(psd);
        } else {
            psd_values.push_back(T{2} * psd);
        }
    }
    
    return {frequencies, psd_values};
}

/**
 * @brief Estimate autocorrelation function of a signal
 * @tparam T Floating point type
 * @param signal Input signal samples
 * @param max_lag Maximum lag to compute
 * @return Autocorrelation values
 */
template<std::floating_point T>
std::vector<T> compute_autocorrelation(
    const std::vector<T>& signal,
    std::size_t max_lag) {
    
    std::size_t n = signal.size();
    if (n < 2 || max_lag >= n) {
        return {};
    }
    
    // Compute sample mean
    T mean = std::accumulate(signal.begin(), signal.end(), T{0}) / n;
    
    // Compute sample variance
    T variance = T{0};
    for (T sample : signal) {
        T centered = sample - mean;
        variance += centered * centered;
    }
    variance /= (n - 1);
    
    if (variance <= T{0}) {
        return std::vector<T>(max_lag + 1, T{0});
    }
    
    std::vector<T> autocorr(max_lag + 1);
    
    for (std::size_t lag = 0; lag <= max_lag; ++lag) {
        T sum = T{0};
        std::size_t count = 0;
        
        for (std::size_t i = 0; i < n - lag; ++i) {
            sum += (signal[i] - mean) * (signal[i + lag] - mean);
            count++;
        }
        
        if (count > 0) {
            autocorr[lag] = sum / (count * variance);
        }
    }
    
    return autocorr;
}

/**
 * @brief Generate noise with specified power spectral density
 * @tparam T Floating point type
 * @param frequencies Frequency points [Hz]
 * @param psd_values Power spectral density values
 * @param n_samples Number of samples to generate
 * @param sampling_rate Sampling rate [Hz]
 * @param generator Random number generator
 * @return Generated noise samples
 */
template<std::floating_point T>
std::vector<T> generate_colored_noise_from_psd(
    const std::vector<T>& frequencies,
    const std::vector<T>& psd_values,
    std::size_t n_samples,
    T sampling_rate,
    NoiseGenerator<T>& generator) {
    
    if (frequencies.size() != psd_values.size() || n_samples == 0) {
        return {};
    }
    
    std::vector<T> noise(n_samples, T{0});
    
    // Generate noise by superposing sinusoids with random phases
    for (std::size_t i = 0; i < frequencies.size(); ++i) {
        if (psd_values[i] <= T{0}) continue;
        
        T amplitude = std::sqrt(T{2} * psd_values[i] * (sampling_rate / T{2}));
        T phase = generator.uniform(T{0}, T{2} * std::numbers::pi);
        
        for (std::size_t j = 0; j < n_samples; ++j) {
            T time = j / sampling_rate;
            noise[j] += amplitude * std::sin(T{2} * std::numbers::pi * frequencies[i] * time + phase);
        }
    }
    
    return noise;
}

/**
 * @brief Validate noise characteristics
 * @tparam T Floating point type
 * @param samples Noise samples
 * @return {mean, std_dev, skewness, kurtosis}
 */
template<std::floating_point T>
std::array<T, 4> analyze_noise_statistics(const std::vector<T>& samples) {
    if (samples.empty()) {
        return {T{0}, T{0}, T{0}, T{0}};
    }
    
    std::size_t n = samples.size();
    
    // Compute mean
    T mean = std::accumulate(samples.begin(), samples.end(), T{0}) / n;
    
    // Compute variance
    T variance = T{0};
    for (T sample : samples) {
        T diff = sample - mean;
        variance += diff * diff;
    }
    variance /= (n - 1);
    T std_dev = std::sqrt(variance);
    
    if (std_dev <= T{0}) {
        return {mean, T{0}, T{0}, T{0}};
    }
    
    // Compute skewness and kurtosis
    T skewness = T{0};
    T kurtosis = T{0};
    
    for (T sample : samples) {
        T standardized = (sample - mean) / std_dev;
        T standardized_sq = standardized * standardized;
        skewness += standardized * standardized_sq;
        kurtosis += standardized_sq * standardized_sq;
    }
    
    skewness /= n;
    kurtosis = kurtosis / n - T{3}; // Excess kurtosis
    
    return {mean, std_dev, skewness, kurtosis};
}

/**
 * @brief Generate realistic sensor noise model
 * @tparam T Floating point type
 * @param generator Noise generator
 * @param base_noise_std Base noise standard deviation
 * @param temperature Temperature [°C]
 * @param supply_voltage Supply voltage [V]
 * @param aging_factor Aging factor (0-1, where 1 is new)
 * @return Adjusted noise standard deviation
 */
template<std::floating_point T>
T generate_realistic_sensor_noise(
    NoiseGenerator<T>& generator,
    T base_noise_std,
    T temperature = T{25},
    T supply_voltage = T{5.0},
    T aging_factor = T{1.0}) {
    
    // Temperature coefficient (typical for semiconductor sensors)
    T temp_coeff = T{0.02}; // 2% per 10°C
    T temp_factor = T{1} + temp_coeff * (temperature - T{25}) / T{10};
    
    // Supply voltage sensitivity
    T voltage_coeff = T{0.01}; // 1% per 0.1V
    T voltage_factor = T{1} + voltage_coeff * std::abs(supply_voltage - T{5.0}) / T{0.1};
    
    // Aging factor (noise typically increases with age)
    T aging_multiplier = T{1} + (T{1} - aging_factor) * T{0.5};
    
    // Combined noise scaling
    T adjusted_std = base_noise_std * temp_factor * voltage_factor * aging_multiplier;
    
    // Add 1/f noise component
    T pink_component = generator.pink_noise(adjusted_std * T{0.1});
    T white_component = generator.gaussian(T{0}, adjusted_std);
    
    return white_component + pink_component;
}

/**
 * @brief Generate correlated noise for multiple sensors
 * @tparam T Floating point type
 * @tparam N Number of sensors
 * @param generator Noise generator
 * @param individual_stds Individual sensor noise standard deviations
 * @param correlation_matrix Correlation matrix between sensors
 * @return Correlated noise samples
 */
template<std::floating_point T, std::size_t N>
std::array<T, N> generate_correlated_sensor_noise(
    NoiseGenerator<T>& generator,
    const std::array<T, N>& individual_stds,
    const std::array<std::array<T, N>, N>& correlation_matrix) {
    
    // Generate independent samples
    std::array<T, N> independent_samples;
    for (std::size_t i = 0; i < N; ++i) {
        independent_samples[i] = generator.gaussian(T{0}, T{1});
    }
    
    // Create covariance matrix
    std::array<std::array<T, N>, N> covariance;
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = 0; j < N; ++j) {
            covariance[i][j] = correlation_matrix[i][j] * individual_stds[i] * individual_stds[j];
        }
    }
    
    // Generate correlated samples using multivariate generation
    std::array<T, N> mean{};
    return generator.multivariate_gaussian(mean, covariance);
}

/**
 * @brief Simulate ADC noise characteristics
 * @tparam T Floating point type
 * @param generator Noise generator
 * @param signal Input signal
 * @param resolution ADC resolution in bits
 * @param full_scale_range Full scale range [V]
 * @param dnl_rms RMS differential nonlinearity [LSB]
 * @param inl_rms RMS integral nonlinearity [LSB]
 * @return Digitized signal with ADC noise
 */
template<std::floating_point T>
T simulate_adc_noise(
    NoiseGenerator<T>& generator,
    T signal,
    std::size_t resolution,
    T full_scale_range,
    T dnl_rms = T{0.5},
    T inl_rms = T{1.0}) {
    
    T lsb = full_scale_range / static_cast<T>(1ULL << resolution);
    
    // Add quantization noise
    T quantization_noise = generator.uniform(-lsb / T{2}, lsb / T{2});
    
    // Add differential nonlinearity
    T dnl_noise = generator.gaussian(T{0}, dnl_rms * lsb);
    
    // Add integral nonlinearity (simplified model)
    T normalized_input = signal / full_scale_range;
    T inl_noise = inl_rms * lsb * std::sin(T{2} * std::numbers::pi * normalized_input);
    inl_noise += generator.gaussian(T{0}, inl_rms * lsb / T{4});
    
    // Add thermal noise
    T thermal_noise = generator.gaussian(T{0}, lsb / T{10});
    
    T noisy_signal = signal + quantization_noise + dnl_noise + inl_noise + thermal_noise;
    
    // Apply final quantization
    return NoiseGenerator<T>::quantize(noisy_signal, resolution, full_scale_range);
}

// Explicit instantiations for utility functions
template std::pair<std::vector<double>, std::vector<double>> compute_psd<double>(
    const std::vector<double>&, double);
template std::pair<std::vector<float>, std::vector<float>> compute_psd<float>(
    const std::vector<float>&, float);

template std::vector<double> compute_autocorrelation<double>(
    const std::vector<double>&, std::size_t);
template std::vector<float> compute_autocorrelation<float>(
    const std::vector<float>&, std::size_t);

template std::vector<double> generate_colored_noise_from_psd<double>(
    const std::vector<double>&, const std::vector<double>&,
    std::size_t, double, NoiseGenerator<double>&);
template std::vector<float> generate_colored_noise_from_psd<float>(
    const std::vector<float>&, const std::vector<float>&,
    std::size_t, float, NoiseGenerator<float>&);

template std::array<double, 4> analyze_noise_statistics<double>(const std::vector<double>&);
template std::array<float, 4> analyze_noise_statistics<float>(const std::vector<float>&);

template double generate_realistic_sensor_noise<double>(
    NoiseGenerator<double>&, double, double, double, double);
template float generate_realistic_sensor_noise<float>(
    NoiseGenerator<float>&, float, float, float, float);

template std::array<double, 2> generate_correlated_sensor_noise<double, 2>(
    NoiseGenerator<double>&, const std::array<double, 2>&,
    const std::array<std::array<double, 2>, 2>&);
template std::array<float, 2> generate_correlated_sensor_noise<float, 2>(
    NoiseGenerator<float>&, const std::array<float, 2>&,
    const std::array<std::array<float, 2>, 2>&);

template std::array<double, 3> generate_correlated_sensor_noise<double, 3>(
    NoiseGenerator<double>&, const std::array<double, 3>&,
    const std::array<std::array<double, 3>, 3>&);
template std::array<float, 3> generate_correlated_sensor_noise<float, 3>(
    NoiseGenerator<float>&, const std::array<float, 3>&,
    const std::array<std::array<float, 3>, 3>&);

template double simulate_adc_noise<double>(
    NoiseGenerator<double>&, double, std::size_t, double, double, double);
template float simulate_adc_noise<float>(
    NoiseGenerator<float>&, float, std::size_t, float, float, float);

} // namespace kf::utils