#include "TimeUtils.hpp"
#include <thread>
#include <stdexcept>
#include <algorithm>
#include <numeric>

namespace kf::utils {

// Explicit template instantiations for common floating point types
template class RateLimiter<double>;
template class RateLimiter<float>;
template class PeriodicScheduler<double>;
template class PeriodicScheduler<float>;
template class MovingAverage<double>;
template class MovingAverage<float>;

// Additional utility functions for time management

/**
 * @brief Compute execution time statistics for a function
 * @tparam Func Function type
 * @tparam T Floating point type for timing
 * @param func Function to time
 * @param iterations Number of iterations to run
 * @return {mean_time, std_dev_time, min_time, max_time}
 */
template<typename Func, std::floating_point T = double>
std::array<T, 4> benchmark_function(Func&& func, std::size_t iterations = 100) {
    if (iterations == 0) {
        return {T{0}, T{0}, T{0}, T{0}};
    }
    
    std::vector<T> execution_times;
    execution_times.reserve(iterations);
    
    for (std::size_t i = 0; i < iterations; ++i) {
        Timer timer;
        timer.start();
        
        try {
            func();
        } catch (...) {
            // Continue timing even if function throws
        }
        
        T elapsed = timer.stop<T>();
        execution_times.push_back(elapsed);
    }
    
    // Compute statistics
    T mean_time = std::accumulate(execution_times.begin(), execution_times.end(), T{0}) / 
                  static_cast<T>(iterations);
    
    T variance = T{0};
    for (T time : execution_times) {
        T diff = time - mean_time;
        variance += diff * diff;
    }
    variance /= static_cast<T>(iterations - 1);
    T std_dev_time = std::sqrt(variance);
    
    auto [min_it, max_it] = std::minmax_element(execution_times.begin(), execution_times.end());
    T min_time = *min_it;
    T max_time = *max_it;
    
    return {mean_time, std_dev_time, min_time, max_time};
}

/**
 * @brief Adaptive rate limiter that adjusts based on system load
 * @tparam T Floating point type
 */
template<std::floating_point T>
class AdaptiveRateLimiter {
public:
    explicit AdaptiveRateLimiter(T initial_rate_hz, T min_rate_hz = T{1}, T max_rate_hz = T{1000})
        : current_rate_(initial_rate_hz), min_rate_(min_rate_hz), max_rate_(max_rate_hz),
          rate_limiter_(initial_rate_hz), load_tracker_(10) {}
    
    /**
     * @brief Check if execution should proceed, with adaptive rate adjustment
     * @param execution_time_seconds Last execution time
     * @return true if should execute
     */
    bool should_execute_adaptive(T execution_time_seconds = T{0}) {
        if (execution_time_seconds > T{0}) {
            load_tracker_.add_sample(execution_time_seconds);
            
            // Adjust rate based on load
            T avg_execution_time = load_tracker_.get_average();
            T target_utilization = T{0.8}; // 80% CPU utilization target
            T current_interval = T{1} / current_rate_;
            
            if (avg_execution_time > current_interval * target_utilization) {
                // System is overloaded, reduce rate
                current_rate_ = std::max(current_rate_ * T{0.9}, min_rate_);
            } else if (avg_execution_time < current_interval * target_utilization * T{0.5}) {
                // System has capacity, increase rate
                current_rate_ = std::min(current_rate_ * T{1.1}, max_rate_);
            }
            
            rate_limiter_.set_rate(current_rate_);
        }
        
        return rate_limiter_.should_execute();
    }
    
    T get_current_rate() const { return current_rate_; }
    
private:
    T current_rate_;
    T min_rate_;
    T max_rate_;
    RateLimiter<T> rate_limiter_;
    MovingAverage<T> load_tracker_;
};

/**
 * @brief High-precision sleep function using busy-wait for final precision
 * @tparam T Floating point type
 * @param duration_seconds Sleep duration in seconds
 * @param busy_wait_threshold Threshold below which to use busy-wait [s]
 */
template<std::floating_point T>
void precise_sleep(T duration_seconds, T busy_wait_threshold = T{0.001}) {
    if (duration_seconds <= T{0}) return;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    auto target_time = start_time + std::chrono::duration<T>(duration_seconds);
    
    // Use regular sleep for most of the duration
    if (duration_seconds > busy_wait_threshold) {
        T sleep_duration = duration_seconds - busy_wait_threshold;
        std::this_thread::sleep_for(std::chrono::duration<T>(sleep_duration));
    }
    
    // Busy-wait for final precision
    while (std::chrono::high_resolution_clock::now() < target_time) {
        std::this_thread::yield();
    }
}

/**
 * @brief Jitter-resistant periodic scheduler
 * @tparam T Floating point type
 */
template<std::floating_point T>
class JitterResistantScheduler {
public:
    explicit JitterResistantScheduler(T period_seconds, std::size_t jitter_window = 10)
        : nominal_period_(period_seconds), jitter_tracker_(jitter_window) {
        reset();
    }
    
    /**
     * @brief Wait for next execution with jitter compensation
     * @return Actual period achieved
     */
    T wait_for_next_compensated() {
        T current_time = RealTimeClock::monotonic_seconds<T>();
        
        // Compute expected next execution time
        T expected_next = last_execution_ + nominal_period_;
        
        // Apply jitter compensation
        T avg_jitter = jitter_tracker_.get_average();
        T compensated_next = expected_next - avg_jitter * T{0.5};
        
        if (current_time < compensated_next) {
            precise_sleep(compensated_next - current_time);
            current_time = RealTimeClock::monotonic_seconds<T>();
        }
        
        // Record actual period and jitter
        T actual_period = current_time - last_execution_;
        T jitter = actual_period - nominal_period_;
        jitter_tracker_.add_sample(jitter);
        
        last_execution_ = current_time;
        return actual_period;
    }
    
    /**
     * @brief Get jitter statistics
     * @return {mean_jitter, jitter_std_dev}
     */
    std::pair<T, T> get_jitter_stats() const {
        return {jitter_tracker_.get_average(), jitter_tracker_.get_std_dev()};
    }
    
    void reset() {
        last_execution_ = RealTimeClock::monotonic_seconds<T>();
        jitter_tracker_.clear();
    }
    
private:
    T nominal_period_;
    T last_execution_;
    MovingAverage<T> jitter_tracker_;
};

/**
 * @brief Timeout manager for operation timeouts
 * @tparam T Floating point type
 */
template<std::floating_point T>
class TimeoutManager {
public:
    explicit TimeoutManager(T timeout_seconds) : timeout_(timeout_seconds) {
        reset();
    }
    
    /**
     * @brief Check if timeout has been reached
     * @return true if timeout reached
     */
    bool is_timeout() const {
        T elapsed = RealTimeClock::monotonic_seconds<T>() - start_time_;
        return elapsed >= timeout_;
    }
    
    /**
     * @brief Get remaining time before timeout
     * @return Remaining time in seconds (0 if timeout reached)
     */
    T remaining_time() const {
        T elapsed = RealTimeClock::monotonic_seconds<T>() - start_time_;
        return std::max(T{0}, timeout_ - elapsed);
    }
    
    /**
     * @brief Get elapsed time since start
     * @return Elapsed time in seconds
     */
    T elapsed_time() const {
        return RealTimeClock::monotonic_seconds<T>() - start_time_;
    }
    
    /**
     * @brief Reset timeout timer
     */
    void reset() {
        start_time_ = RealTimeClock::monotonic_seconds<T>();
    }
    
    /**
     * @brief Set new timeout duration
     * @param timeout_seconds New timeout in seconds
     */
    void set_timeout(T timeout_seconds) {
        timeout_ = timeout_seconds;
    }
    
private:
    T timeout_;
    T start_time_;
};

/**
 * @brief Performance profiler for code sections
 */
class PerformanceProfiler {
public:
    struct ProfileData {
        std::string name;
        std::size_t call_count = 0;
        double total_time = 0.0;
        double min_time = std::numeric_limits<double>::max();
        double max_time = 0.0;
        double avg_time = 0.0;
    };
    
    /**
     * @brief Start profiling a section
     * @param section_name Name of the section
     */
    void start_section(const std::string& section_name) {
        current_section_ = section_name;
        section_timer_.start();
    }
    
    /**
     * @brief End profiling current section
     */
    void end_section() {
        if (current_section_.empty()) return;
        
        double elapsed = section_timer_.stop<double>();
        
        auto& data = profile_data_[current_section_];
        data.name = current_section_;
        data.call_count++;
        data.total_time += elapsed;
        data.min_time = std::min(data.min_time, elapsed);
        data.max_time = std::max(data.max_time, elapsed);
        data.avg_time = data.total_time / data.call_count;
        
        current_section_.clear();
    }
    
    /**
     * @brief Get profiling data for all sections
     * @return Map of section names to profile data
     */
    const std::unordered_map<std::string, ProfileData>& get_profile_data() const {
        return profile_data_;
    }
    
    /**
     * @brief Generate formatted profiling report
     * @return Formatted report string
     */
    std::string generate_report() const {
        std::stringstream report;
        report << "=== PERFORMANCE PROFILE ===\n";
        report << std::setw(20) << "Section" << " | ";
        report << std::setw(8) << "Calls" << " | ";
        report << std::setw(10) << "Total(ms)" << " | ";
        report << std::setw(10) << "Avg(μs)" << " | ";
        report << std::setw(10) << "Min(μs)" << " | ";
        report << std::setw(10) << "Max(μs)" << "\n";
        report << std::string(80, '-') << "\n";
        
        for (const auto& [name, data] : profile_data_) {
            report << std::setw(20) << name << " | ";
            report << std::setw(8) << data.call_count << " | ";
            report << std::setw(10) << std::fixed << std::setprecision(3) << (data.total_time * 1000) << " | ";
            report << std::setw(10) << std::fixed << std::setprecision(1) << (data.avg_time * 1e6) << " | ";
            report << std::setw(10) << std::fixed << std::setprecision(1) << (data.min_time * 1e6) << " | ";
            report << std::setw(10) << std::fixed << std::setprecision(1) << (data.max_time * 1e6) << "\n";
        }
        
        return report.str();
    }
    
    /**
     * @brief Clear all profiling data
     */
    void clear() {
        profile_data_.clear();
        current_section_.clear();
    }
    
private:
    std::unordered_map<std::string, ProfileData> profile_data_;
    std::string current_section_;
    Timer section_timer_;
};

/**
 * @brief RAII profiler section helper
 */
class ProfiledSection {
public:
    ProfiledSection(PerformanceProfiler& profiler, const std::string& section_name)
        : profiler_(profiler) {
        profiler_.start_section(section_name);
    }
    
    ~ProfiledSection() {
        profiler_.end_section();
    }
    
private:
    PerformanceProfiler& profiler_;
};

// Macro for easy profiling
#define PROFILE_SECTION(profiler, name) \
    ProfiledSection _prof_section_(profiler, name)

// Explicit instantiations for template functions and classes
template std::array<double, 4> benchmark_function<std::function<void()>, double>(std::function<void()>&&, std::size_t);
template std::array<float, 4> benchmark_function<std::function<void()>, float>(std::function<void()>&&, std::size_t);

template class AdaptiveRateLimiter<double>;
template class AdaptiveRateLimiter<float>;

template void precise_sleep<double>(double, double);
template void precise_sleep<float>(float, float);

template class JitterResistantScheduler<double>;
template class JitterResistantScheduler<float>;

template class TimeoutManager<double>;
template class TimeoutManager<float>;

} // namespace kf::utils