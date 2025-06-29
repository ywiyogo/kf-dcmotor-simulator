#pragma once

#include <chrono>
#include <string>
#include <concepts>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <vector>
#include <thread>
#include <unordered_map>
#include <functional>

namespace kf::utils {

/**
 * @brief High-precision timer for performance measurement
 */
class Timer {
public:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = Clock::time_point;
    using Duration = Clock::duration;

    /**
     * @brief Start the timer
     */
    void start() {
        start_time_ = Clock::now();
        running_ = true;
    }

    /**
     * @brief Stop the timer and return elapsed time
     * @return Elapsed time in seconds
     */
    template<std::floating_point T = double>
    T stop() {
        if (!running_) return T{0};
        
        end_time_ = Clock::now();
        running_ = false;
        
        auto duration = end_time_ - start_time_;
        return std::chrono::duration_cast<std::chrono::duration<T>>(duration).count();
    }

    /**
     * @brief Get elapsed time without stopping
     * @return Elapsed time in seconds
     */
    template<std::floating_point T = double>
    T elapsed() const {
        TimePoint current = running_ ? Clock::now() : end_time_;
        auto duration = current - start_time_;
        return std::chrono::duration_cast<std::chrono::duration<T>>(duration).count();
    }

    /**
     * @brief Reset the timer
     */
    void reset() {
        start_time_ = Clock::now();
        running_ = false;
    }

    /**
     * @brief Check if timer is running
     */
    bool is_running() const { return running_; }

private:
    TimePoint start_time_;
    TimePoint end_time_;
    bool running_ = false;
};

/**
 * @brief RAII-style scoped timer for automatic timing
 */
template<std::floating_point T = double>
class ScopedTimer {
public:
    explicit ScopedTimer(T& result_ref) : result_(result_ref) {
        timer_.start();
    }

    ~ScopedTimer() {
        result_ = timer_.stop<T>();
    }

private:
    Timer timer_;
    T& result_;
};

/**
 * @brief Rate limiter for controlling update frequencies
 */
template<std::floating_point T = double>
class RateLimiter {
public:
    explicit RateLimiter(T rate_hz) : interval_(T{1} / rate_hz) {}

    /**
     * @brief Check if enough time has passed since last call
     * @return true if rate limit allows execution
     */
    bool should_execute() {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::duration<T>>(now - last_execution_).count();
        
        if (elapsed >= interval_) {
            last_execution_ = now;
            return true;
        }
        return false;
    }

    /**
     * @brief Reset the rate limiter
     */
    void reset() {
        last_execution_ = std::chrono::steady_clock::time_point::min();
    }

    /**
     * @brief Set new rate
     * @param rate_hz New rate in Hz
     */
    void set_rate(T rate_hz) {
        interval_ = T{1} / rate_hz;
    }

private:
    T interval_;
    std::chrono::steady_clock::time_point last_execution_ = std::chrono::steady_clock::time_point::min();
};

/**
 * @brief Real-time clock synchronization utilities
 */
class RealTimeClock {
public:
    using SteadyClock = std::chrono::steady_clock;
    using SystemClock = std::chrono::system_clock;

    /**
     * @brief Get current time in seconds since epoch
     */
    template<std::floating_point T = double>
    static T now_seconds() {
        auto now = SystemClock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration_cast<std::chrono::duration<T>>(duration).count();
    }

    /**
     * @brief Get monotonic time in seconds
     */
    template<std::floating_point T = double>
    static T monotonic_seconds() {
        auto now = SteadyClock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration_cast<std::chrono::duration<T>>(duration).count();
    }

    /**
     * @brief Sleep for specified duration
     * @param seconds Sleep duration in seconds
     */
    template<std::floating_point T>
    static void sleep(T seconds) {
        auto duration = std::chrono::duration<T>(seconds);
        std::this_thread::sleep_for(duration);
    }

    /**
     * @brief Sleep until specific time point
     * @param target_time Target time in seconds since epoch
     */
    template<std::floating_point T>
    static void sleep_until(T target_time) {
        auto target = SystemClock::time_point(std::chrono::duration<T>(target_time));
        std::this_thread::sleep_until(target);
    }
};

/**
 * @brief Time formatting utilities
 */
class TimeFormatter {
public:
    /**
     * @brief Format time duration as human-readable string
     * @param seconds Duration in seconds
     * @return Formatted string (e.g., "1h 23m 45.67s")
     */
    template<std::floating_point T>
    static std::string format_duration(T seconds) {
        std::stringstream ss;
        
        if (seconds < T{0}) {
            ss << "-";
            seconds = -seconds;
        }

        auto hours = static_cast<int>(seconds / T{3600});
        seconds -= hours * T{3600};
        
        auto minutes = static_cast<int>(seconds / T{60});
        seconds -= minutes * T{60};

        if (hours > 0) {
            ss << hours << "h ";
        }
        if (minutes > 0 || hours > 0) {
            ss << minutes << "m ";
        }
        
        ss << std::fixed << std::setprecision(2) << seconds << "s";
        
        return ss.str();
    }

    /**
     * @brief Format timestamp as ISO 8601 string
     * @param seconds Seconds since epoch
     * @return ISO 8601 formatted string
     */
    template<std::floating_point T>
    static std::string format_timestamp(T seconds) {
        auto time_point = std::chrono::system_clock::time_point(
            std::chrono::duration_cast<std::chrono::system_clock::duration>(
                std::chrono::duration<T>(seconds)));
        
        auto time_t = std::chrono::system_clock::to_time_t(time_point);
        auto subseconds = seconds - std::floor(seconds);
        
        std::stringstream ss;
        ss << std::put_time(std::gmtime(&time_t), "%Y-%m-%dT%H:%M:%S");
        ss << std::fixed << std::setprecision(3) << subseconds;
        ss << "Z";
        
        return ss.str();
    }

    /**
     * @brief Format frequency as human-readable string
     * @param frequency_hz Frequency in Hz
     * @return Formatted string (e.g., "1.23 kHz", "456 mHz")
     */
    template<std::floating_point T>
    static std::string format_frequency(T frequency_hz) {
        std::stringstream ss;
        
        if (frequency_hz >= T{1e9}) {
            ss << std::fixed << std::setprecision(2) << frequency_hz / T{1e9} << " GHz";
        } else if (frequency_hz >= T{1e6}) {
            ss << std::fixed << std::setprecision(2) << frequency_hz / T{1e6} << " MHz";
        } else if (frequency_hz >= T{1e3}) {
            ss << std::fixed << std::setprecision(2) << frequency_hz / T{1e3} << " kHz";
        } else if (frequency_hz >= T{1}) {
            ss << std::fixed << std::setprecision(2) << frequency_hz << " Hz";
        } else if (frequency_hz >= T{1e-3}) {
            ss << std::fixed << std::setprecision(2) << frequency_hz * T{1e3} << " mHz";
        } else {
            ss << std::scientific << std::setprecision(2) << frequency_hz << " Hz";
        }
        
        return ss.str();
    }
};

/**
 * @brief Periodic task scheduler
 */
template<std::floating_point T = double>
class PeriodicScheduler {
public:
    explicit PeriodicScheduler(T period_seconds) 
        : period_(period_seconds), next_execution_(RealTimeClock::monotonic_seconds<T>()) {}

    /**
     * @brief Wait until next scheduled execution time
     * @return Actual time since last execution
     */
    T wait_for_next() {
        T current_time = RealTimeClock::monotonic_seconds<T>();
        
        if (current_time < next_execution_) {
            T sleep_time = next_execution_ - current_time;
            RealTimeClock::sleep(sleep_time);
            current_time = RealTimeClock::monotonic_seconds<T>();
        }
        
        T actual_period = current_time - last_execution_;
        last_execution_ = current_time;
        next_execution_ = current_time + period_;
        
        return actual_period;
    }

    /**
     * @brief Check if it's time for next execution without blocking
     * @return true if execution should happen now
     */
    bool is_time() const {
        return RealTimeClock::monotonic_seconds<T>() >= next_execution_;
    }

    /**
     * @brief Reset scheduler timing
     */
    void reset() {
        T current_time = RealTimeClock::monotonic_seconds<T>();
        last_execution_ = current_time;
        next_execution_ = current_time + period_;
    }

    /**
     * @brief Set new period
     * @param period_seconds New period in seconds
     */
    void set_period(T period_seconds) {
        period_ = period_seconds;
        next_execution_ = last_execution_ + period_;
    }

    /**
     * @brief Get current period
     */
    T get_period() const { return period_; }

private:
    T period_;
    T last_execution_ = T{0};
    T next_execution_;
};

/**
 * @brief Moving average for timing measurements
 */
template<std::floating_point T = double>
class MovingAverage {
public:
    explicit MovingAverage(std::size_t window_size = 10) 
        : window_size_(window_size) {
        samples_.reserve(window_size);
    }

    /**
     * @brief Add new sample
     * @param value New sample value
     */
    void add_sample(T value) {
        samples_.push_back(value);
        if (samples_.size() > window_size_) {
            samples_.erase(samples_.begin());
        }
    }

    /**
     * @brief Get current average
     * @return Average of samples in window
     */
    T get_average() const {
        if (samples_.empty()) return T{0};
        
        T sum = T{0};
        for (T sample : samples_) {
            sum += sample;
        }
        return sum / static_cast<T>(samples_.size());
    }

    /**
     * @brief Get standard deviation
     * @return Standard deviation of samples
     */
    T get_std_dev() const {
        if (samples_.size() < 2) return T{0};
        
        T mean = get_average();
        T variance = T{0};
        
        for (T sample : samples_) {
            T diff = sample - mean;
            variance += diff * diff;
        }
        
        variance /= static_cast<T>(samples_.size() - 1);
        return std::sqrt(variance);
    }

    /**
     * @brief Clear all samples
     */
    void clear() {
        samples_.clear();
    }

    /**
     * @brief Check if window is full
     */
    bool is_full() const {
        return samples_.size() >= window_size_;
    }

private:
    std::size_t window_size_;
    std::vector<T> samples_;
};

// Type aliases
using Timerd = Timer;
using RateLimiterd = RateLimiter<double>;
using RateLimiterf = RateLimiter<float>;
using PeriodicSchedulerd = PeriodicScheduler<double>;
using PeriodicSchedulerf = PeriodicScheduler<float>;
using MovingAveraged = MovingAverage<double>;
using MovingAveragef = MovingAverage<float>;

} // namespace kf::utils