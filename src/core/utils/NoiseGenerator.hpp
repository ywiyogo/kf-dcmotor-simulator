#pragma once

#include <random>
#include <concepts>
#include <array>
#include <vector>
#include <memory>
#include <chrono>
#include <numbers>
#include <cmath>
#include <thread>
#include <execution>

#ifdef HAS_PARALLEL_STL
#include <execution>
#endif

namespace kf::utils {

/**
 * @brief High-quality noise generator for simulation purposes
 * 
 * Provides various noise distributions commonly used in control systems
 * and signal processing, with support for reproducible seeds and
 * parallel generation when available.
 * 
 * @tparam T Floating point type (float, double)
 */
template<std::floating_point T = double>
class NoiseGenerator {
public:
    using ValueType = T;
    using Generator = std::mt19937_64;
    using Seed = std::uint64_t;

    /**
     * @brief Constructor with optional seed
     * @param seed Random seed (0 for random seed based on time)
     */
    explicit NoiseGenerator(Seed seed = 0) {
        if (seed == 0) {
            seed = std::chrono::steady_clock::now().time_since_epoch().count();
        }
        generator_.seed(seed);
        current_seed_ = seed;
    }

    /**
     * @brief Generate Gaussian (normal) distributed noise
     * @param mean Mean value
     * @param std_dev Standard deviation
     * @return Random sample from N(mean, std_dev²)
     */
    T gaussian(T mean = T{0}, T std_dev = T{1}) {
        std::normal_distribution<T> dist(mean, std_dev);
        return dist(generator_);
    }

    /**
     * @brief Generate uniform distributed noise
     * @param min Minimum value
     * @param max Maximum value
     * @return Random sample from U(min, max)
     */
    T uniform(T min = T{0}, T max = T{1}) {
        std::uniform_real_distribution<T> dist(min, max);
        return dist(generator_);
    }

    /**
     * @brief Generate exponentially distributed noise
     * @param lambda Rate parameter (λ)
     * @return Random sample from Exp(λ)
     */
    T exponential(T lambda = T{1}) {
        std::exponential_distribution<T> dist(lambda);
        return dist(generator_);
    }

    /**
     * @brief Generate colored noise using AR(1) process
     * @param variance Noise variance
     * @param correlation Correlation coefficient (-1 < ρ < 1)
     * @return Correlated noise sample
     */
    T colored_noise(T variance = T{1}, T correlation = T{0.5}) {
        static T previous_sample = T{0};
        
        T white_noise = gaussian(T{0}, std::sqrt(variance * (T{1} - correlation * correlation)));
        T colored_sample = correlation * previous_sample + white_noise;
        
        previous_sample = colored_sample;
        return colored_sample;
    }

    /**
     * @brief Generate pink noise (1/f noise)
     * @param amplitude Amplitude scaling
     * @return Pink noise sample
     */
    T pink_noise(T amplitude = T{1}) {
        // Simple pink noise approximation using multiple white noise sources
        static std::array<T, 7> states{};
        static bool initialized = false;
        
        if (!initialized) {
            states.fill(T{0});
            initialized = true;
        }
        
        T white = gaussian(T{0}, T{1});
        
        states[0] = 0.99886 * states[0] + white * 0.0555179;
        states[1] = 0.99332 * states[1] + white * 0.0750759;
        states[2] = 0.96900 * states[2] + white * 0.1538520;
        states[3] = 0.86650 * states[3] + white * 0.3104856;
        states[4] = 0.55000 * states[4] + white * 0.5329522;
        states[5] = -0.7616 * states[5] - white * 0.0168980;
        
        T pink = states[0] + states[1] + states[2] + states[3] + states[4] + states[5] + states[6] + white * 0.5362;
        states[6] = white * 0.115926;
        
        return pink * amplitude * T{0.11};
    }

    /**
     * @brief Generate band-limited white noise
     * @param variance Noise variance
     * @param cutoff_freq Cutoff frequency [rad/s]
     * @param dt Sampling time [s]
     * @return Filtered noise sample
     */
    T band_limited_noise(T variance = T{1}, T cutoff_freq = T{10}, T dt = T{0.01}) {
        static T previous_output = T{0};
        
        T alpha = dt * cutoff_freq / (T{1} + dt * cutoff_freq);
        T white_noise = gaussian(T{0}, std::sqrt(variance));
        
        T filtered_noise = alpha * white_noise + (T{1} - alpha) * previous_output;
        previous_output = filtered_noise;
        
        return filtered_noise;
    }

    /**
     * @brief Generate Markov noise (first-order Markov process)
     * @param variance Steady-state variance
     * @param time_constant Time constant [s]
     * @param dt Sampling time [s]
     * @return Markov noise sample
     */
    T markov_noise(T variance = T{1}, T time_constant = T{1}, T dt = T{0.01}) {
        static T current_state = T{0};
        
        T beta = std::exp(-dt / time_constant);
        T white_noise = gaussian(T{0}, std::sqrt(variance * (T{1} - beta * beta)));
        
        current_state = beta * current_state + white_noise;
        return current_state;
    }

    /**
     * @brief Generate multi-variate Gaussian noise
     * @tparam N Dimension
     * @param mean Mean vector
     * @param covariance Covariance matrix
     * @return N-dimensional noise vector
     */
    template<std::size_t N>
    std::array<T, N> multivariate_gaussian(
        const std::array<T, N>& mean,
        const std::array<std::array<T, N>, N>& covariance) {
        
        // Generate independent standard normal samples
        std::array<T, N> independent_samples;
        for (std::size_t i = 0; i < N; ++i) {
            independent_samples[i] = gaussian(T{0}, T{1});
        }
        
        // Cholesky decomposition of covariance matrix (simplified for small N)
        std::array<std::array<T, N>, N> L{};
        
        // Compute Cholesky decomposition
        for (std::size_t i = 0; i < N; ++i) {
            for (std::size_t j = 0; j <= i; ++j) {
                if (i == j) {
                    T sum = T{0};
                    for (std::size_t k = 0; k < j; ++k) {
                        sum += L[j][k] * L[j][k];
                    }
                    L[j][j] = std::sqrt(covariance[j][j] - sum);
                } else {
                    T sum = T{0};
                    for (std::size_t k = 0; k < j; ++k) {
                        sum += L[i][k] * L[j][k];
                    }
                    L[i][j] = (covariance[i][j] - sum) / L[j][j];
                }
            }
        }
        
        // Transform independent samples
        std::array<T, N> result;
        for (std::size_t i = 0; i < N; ++i) {
            result[i] = mean[i];
            for (std::size_t j = 0; j <= i; ++j) {
                result[i] += L[i][j] * independent_samples[j];
            }
        }
        
        return result;
    }

    /**
     * @brief Generate vector of noise samples
     * @param count Number of samples
     * @param noise_func Noise generation function
     * @return Vector of noise samples
     */
    template<typename NoiseFunc>
    std::vector<T> generate_sequence(std::size_t count, NoiseFunc&& noise_func) {
        std::vector<T> samples(count);
        
#ifdef HAS_PARALLEL_STL
        if (count > 1000) {
            // Use parallel generation for large sequences
            std::vector<Generator> generators(std::thread::hardware_concurrency());
            for (std::size_t i = 0; i < generators.size(); ++i) {
                generators[i].seed(current_seed_ + i);
            }
            
            std::transform(std::execution::par_unseq,
                         samples.begin(), samples.end(), samples.begin(),
                         [&noise_func, &generators](T&) {
                             thread_local std::size_t thread_id = 
                                 std::hash<std::thread::id>{}(std::this_thread::get_id()) % generators.size();
                             return noise_func(generators[thread_id]);
                         });
        } else {
#endif
            std::generate(samples.begin(), samples.end(),
                         [&noise_func, this]() { return noise_func(generator_); });
#ifdef HAS_PARALLEL_STL
        }
#endif
        
        return samples;
    }

    /**
     * @brief Generate Gaussian sequence
     * @param count Number of samples
     * @param mean Mean value
     * @param std_dev Standard deviation
     * @return Vector of Gaussian samples
     */
    std::vector<T> gaussian_sequence(std::size_t count, T mean = T{0}, T std_dev = T{1}) {
        return generate_sequence(count, [mean, std_dev](Generator& gen) {
            std::normal_distribution<T> dist(mean, std_dev);
            return dist(gen);
        });
    }

    /**
     * @brief Reset generator with new seed
     * @param seed New random seed
     */
    void reset(Seed seed = 0) {
        if (seed == 0) {
            seed = std::chrono::steady_clock::now().time_since_epoch().count();
        }
        generator_.seed(seed);
        current_seed_ = seed;
    }

    /**
     * @brief Get current seed
     * @return Current random seed
     */
    Seed get_seed() const noexcept { return current_seed_; }

    /**
     * @brief Generate deterministic noise for testing
     * @param amplitude Peak amplitude
     * @param frequency Frequency [Hz]
     * @param phase Phase offset [rad]
     * @param time Current time [s]
     * @return Sinusoidal "noise" for testing
     */
    static T deterministic_sinusoidal(T amplitude, T frequency, T phase, T time) {
        return amplitude * std::sin(T{2} * std::numbers::pi_v<T> * frequency * time + phase);
    }

    /**
     * @brief Generate chirp signal (frequency sweep)
     * @param amplitude Peak amplitude
     * @param f0 Initial frequency [Hz]
     * @param f1 Final frequency [Hz]
     * @param duration Total duration [s]
     * @param time Current time [s]
     * @return Chirp signal value
     */
    static T chirp_signal(T amplitude, T f0, T f1, T duration, T time) {
        if (time >= duration) return T{0};
        
        T k = (f1 - f0) / duration;
        // T instantaneous_freq = f0 + k * time;  // Unused variable
        T phase = T{2} * std::numbers::pi_v<T> * (f0 * time + T{0.5} * k * time * time);
        
        return amplitude * std::sin(phase);
    }

    /**
     * @brief Generate step input signal
     * @param amplitude Step amplitude
     * @param step_time Time of step [s]
     * @param time Current time [s]
     * @return Step signal value
     */
    static T step_signal(T amplitude, T step_time, T time) {
        return (time >= step_time) ? amplitude : T{0};
    }

    /**
     * @brief Generate ramp input signal
     * @param slope Ramp slope
     * @param start_time Ramp start time [s]
     * @param time Current time [s]
     * @return Ramp signal value
     */
    static T ramp_signal(T slope, T start_time, T time) {
        return (time >= start_time) ? slope * (time - start_time) : T{0};
    }

    /**
     * @brief Apply quantization noise (ADC simulation)
     * @param value Input value
     * @param resolution Number of bits
     * @param full_scale Full-scale range
     * @return Quantized value
     */
    static T quantize(T value, std::size_t resolution, T full_scale) {
        T levels = static_cast<T>(1ULL << resolution);
        T lsb = full_scale / levels;
        
        // Clamp to range
        value = std::clamp(value, -full_scale / T{2}, full_scale / T{2});
        
        // Quantize
        T quantized = std::round(value / lsb) * lsb;
        
        return quantized;
    }

private:
    Generator generator_;
    Seed current_seed_;
};

// Type aliases
using NoiseGeneratord = NoiseGenerator<double>;
using NoiseGeneratorf = NoiseGenerator<float>;

/**
 * @brief Signal generator for system identification and testing
 * @tparam T Floating point type
 */
template<std::floating_point T = double>
class SignalGenerator {
public:
    /**
     * @brief Generate Pseudo-Random Binary Sequence (PRBS)
     * @param amplitude Signal amplitude
     * @param period Switching period [s]
     * @param time Current time [s]
     * @return PRBS signal value
     */
    static T prbs_signal(T amplitude, T period, T time) {
        // Simple PRBS using linear feedback shift register concept
        static std::uint32_t lfsr = 1;
        static T last_switch_time = 0;
        
        if (time - last_switch_time >= period) {
            // Update LFSR (maximal length sequence for 32-bit)
            bool feedback = ((lfsr >> 31) ^ (lfsr >> 21) ^ (lfsr >> 1) ^ lfsr) & 1;
            lfsr = (lfsr << 1) | feedback;
            last_switch_time = time;
        }
        
        return (lfsr & 1) ? amplitude : -amplitude;
    }

    /**
     * @brief Generate multi-sine signal for frequency response testing
     * @param amplitudes Vector of sine wave amplitudes
     * @param frequencies Vector of frequencies [Hz]
     * @param phases Vector of phase offsets [rad]
     * @param time Current time [s]
     * @return Multi-sine signal value
     */
    static T multi_sine(const std::vector<T>& amplitudes,
                       const std::vector<T>& frequencies,
                       const std::vector<T>& phases,
                       T time) {
        T result = T{0};
        std::size_t n = std::min({amplitudes.size(), frequencies.size(), phases.size()});
        
        for (std::size_t i = 0; i < n; ++i) {
            result += amplitudes[i] * std::sin(T{2} * std::numbers::pi_v<T> * frequencies[i] * time + phases[i]);
        }
        
        return result;
    }
};

// Type aliases for signal generator
using SignalGeneratord = SignalGenerator<double>;
using SignalGeneratorf = SignalGenerator<float>;

} // namespace kf::utils