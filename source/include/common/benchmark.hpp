#ifndef BENCHMARK_HPP
#define BENCHMARK_HPP

#include <chrono>

/**
 * Timer class that can be repeatedly started and stopped. Statistics are collected for all measured intervals .
 */
class RepeatedTimer {
   public:
    RepeatedTimer()
        : num_timed_intervals_(0),
          total_time_(std::chrono::nanoseconds::zero()),
          max_interval_time_(std::chrono::nanoseconds::zero()),
          last_interval_time_(std::chrono::nanoseconds::zero()),
          start_time_(std::chrono::steady_clock::now()) {}

    /**
     *  Reset the timer statistics
     */
    void reset() {
        num_timed_intervals_ = 0;
        total_time_ = std::chrono::nanoseconds::zero();
        max_interval_time_ = std::chrono::nanoseconds::zero();
        last_interval_time_ = std::chrono::nanoseconds::zero();
    }

    /**
     *  Start timing an interval
     */
    void start_timer() {
        start_time_ = std::chrono::steady_clock::now();
    }

    /**
     * Stop timing of an interval
     */
    void end_timer() {
        auto endTime = std::chrono::steady_clock::now();
        last_interval_time_ = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - start_time_);
        max_interval_time_ = std::max(max_interval_time_, last_interval_time_);
        total_time_ += last_interval_time_;
        num_timed_intervals_++;
    };

    /**
     * @return Number of intervals that were timed
     */
    int get_num_timed_intervals() const {
        return num_timed_intervals_;
    }

    /**
     * @return Total cumulative time of timed intervals
     */
    double get_total_in_milliseconds() const {
        return std::chrono::duration<double, std::milli>(total_time_).count();
    }

    /**
     * @return Maximum duration of a single interval
     */
    double get_max_interval_in_milliseconds() const {
        return std::chrono::duration<double, std::milli>(max_interval_time_).count();
    }

    /**
     * @return Duration of the last timed interval
     */
    double get_last_interval_in_milliseconds() const {
        return std::chrono::duration<double, std::milli>(last_interval_time_).count();
    }

    /**
     * @return Average duration of all timed intervals
     */
    double get_average_in_milliseconds() const {
        return get_total_in_milliseconds() / num_timed_intervals_;
    }

   private:
    int num_timed_intervals_;
    std::chrono::nanoseconds total_time_;
    std::chrono::nanoseconds max_interval_time_;
    std::chrono::nanoseconds last_interval_time_;
    std::chrono::steady_clock::time_point start_time_;
};

#endif  // BENCHMARK_HPP