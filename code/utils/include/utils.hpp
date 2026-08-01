#pragma once

// ##################################################################### //
// ############################## INCLUDE ############################## //
// ##################################################################### //

#include <chrono>
#include <cmath>
#include <string>

#include "logger.hxx"

using namespace utilz;
using enum LoggerLevel;

// ##################################################################### //
// ######################### VERSION AND DATES ######################### //
// ##################################################################### //

#include "version.hpp"

[[nodiscard]]
inline auto version() -> std::string {
    return "Version: " + std::string(PROJECT_VERSION);
}

[[nodiscard]]
inline auto date_today() -> std::string {
    auto now = std::chrono::system_clock::now();
    std::time_t end_time = std::chrono::system_clock::to_time_t(now);
    std::string time_str = std::ctime(&end_time);
    if (!time_str.empty() && time_str.back() == '\n') {
        time_str.pop_back();  // remove trailing newline
    }
    return time_str;
}

[[nodiscard]]
inline auto date_compile() -> std::string {
    return "Compiled: " + std::string(__DATE__) + " " + __TIME__;
}

// ##################################################################### //
// ############################# CONSTANTS ############################# //
// ##################################################################### //

#include "cli_descriptions.hpp"
#include "constants.hpp"

// ##################################################################### //
// ############################### LIMITS ############################## //
// ##################################################################### //

#include <csignal>
#include <limits.hxx>

/**
 * Raw terminate flag polled by CPLEX via CPXsetterminate.
 *
 * CPLEX needs a plain `volatile int*` it can read from its own worker threads,
 * so this cannot be one of the std::atomic<bool> flags in global_limits.
 * Set to 1 from every cooperative-stop source (Ctrl+C handler, time-limit and
 * memory-limit callbacks). `std::sig_atomic_t` is the only type whose writes
 * are async-signal-safe, and it aliases `int` on this platform.
 */
inline volatile std::sig_atomic_t GLOBAL_TERMINATE_CONDITION = 0;

// ##################################################################### //
// ###################### FLOATING POINT PRECISION ##################### //
// ##################################################################### //

constexpr auto is_same_double(double val1, double val2) -> bool { return std::abs(val1 - val2) < constants::epsilon; }

constexpr auto is_gr_or_eq_double(double val1, double val2) -> bool { return val1 >= val2 - constants::epsilon; }
constexpr auto is_lw_or_eq_double(double val1, double val2) -> bool { return is_gr_or_eq_double(val2, val1); }

constexpr auto is_gr_strict_double(double val1, double val2) -> bool { return val1 > val2 + constants::epsilon; }
constexpr auto is_lw_strict_double(double val1, double val2) -> bool { return is_gr_strict_double(val2, val1); }

// if val is epsilon close to an integer, return that integer
constexpr auto fix_precision(double val) -> double {
    const double rounded = std::round(val);
    return is_same_double(val, rounded) ? rounded : val;
}

// ##################################################################### //
// ##################### VECTOR SORTING AND SEARCH ##################### //
// ##################################################################### //

#include "stl_utils.hpp"

// ##################################################################### //
// ########################## BINARY SET UTILS ######################### //
// ##################################################################### //

#include "bs_utils.hpp"

// ##################################################################### //
// ########################## OTHER UTILITIES ########################## //
// ##################################################################### //

[[nodiscard]]
inline auto isint(const std::string& str, const long long from = std::numeric_limits<int>::min(), const long long to = std::numeric_limits<int>::max()) -> bool {
    // Handle empty string
    if (str.empty()) {
        return false;
    }

    // Check for leading whitespace or sign
    size_t i{0};
    if (str[i] == '+' || str[i] == '-') {
        i++;
    }

    // Must have at least one digit
    if (i == str.length() || (std::isdigit(str[i]) == 0)) {
        return false;
    }

    // Check remaining characters are digits
    for (; i < str.length(); i++) {
        if (std::isdigit(str[i]) == 0) {
            return false;
        }
    }

    try {
        // Parse in 64 bit: the comparison below is then free of any signed-overflow
        // assumption on the (int-sized) bounds the callers derive with '- 1'
        const long long num{std::stoll(str)};
        return num >= from && num <= to;
    } catch (const std::out_of_range&) {
        // Handle overflow cases
        return false;
    }
}