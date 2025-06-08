#ifndef TIMER_HPP
#define TIMER_HPP

#include <chrono>  // std::chrono

class timer {
   public:
    // Get the single instance
    static timer& get_instance() {
        static timer instance;
        return instance;
    }

    // Get elapsed time in seconds since creation
    [[nodiscard]]
    inline double get() const {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_);
        return static_cast<double>(duration.count()) / 1'000'000.0;
    }

    // Reset the timer to current time
    void reset() { start_time_ = std::chrono::steady_clock::now(); }

    // Delete copy constructor and assignment operator
    timer(const timer&) = delete;
    timer& operator=(const timer&) = delete;

   private:
    timer() : start_time_(std::chrono::steady_clock::now()) {}

    std::chrono::steady_clock::time_point start_time_;
};

#define GET_TIME() timer::get_instance().get()

#endif  // TIMER_HPP