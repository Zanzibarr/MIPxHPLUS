/**
 * @file timer.hxx
 * @brief Class for keeping time and other related utilities
 * @version 1.0.0
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 * @see https://github.com/Zanzibarr/timer
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef TIMER_HXX
#define TIMER_HXX

#include <chrono>  // std::chrono

class timer {
   public:
    // Get the single instance
    /// @brief Returns the single instance of the timer. Implements the singleton pattern.
    static timer& get_instance() {
        static timer instance;
        return instance;
    }

    // Get elapsed time in seconds since creation
    [[nodiscard]]
    inline double get() const {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_);
        return static_cast<double>(duration.count()) / 1000000.0;
    }

    // Reset the timer to current time
    void reset() { start_time_ = std::chrono::steady_clock::now(); }

    // Delete copy constructor and assignment operator
    /// @brief Deleted copy constructor to prevent copying of the singleton instance.
    timer(const timer&) = delete;
    /// @brief Deleted assignment operator to prevent assignment of the singleton instance.
    timer& operator=(const timer&) = delete;

   private:
    timer() : start_time_(std::chrono::steady_clock::now()) {}

    std::chrono::steady_clock::time_point start_time_;
};

/**
 * @brief Convenience macro to get the current elapsed time from the global timer instance.
 * @return The elapsed time in seconds as a double.
 */
#define GET_TIME() timer::get_instance().get()

/// @brief Exception thrown when a predefined time limit is exceeded.
class timelimit_exception final : public std::runtime_error {
   public:
    /// @brief Constructs a timelimit_exception with a descriptive message.
    /// @param message The message describing the exception.
    explicit timelimit_exception(const std::string& message) : std::runtime_error(message) {}
};

#endif