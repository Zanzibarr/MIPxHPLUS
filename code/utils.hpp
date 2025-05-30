/**
 * @file utils.hpp
 * @brief Utilities for the hplus master thesis project
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#ifndef UTILS_H
#define UTILS_H

// ##################################################################### //
// ############################ CODE VERSION ########################### //
// ##################################################################### //

#ifndef CODE_VERSION  // overwritten by cmake
#define CODE_VERSION "2.1.0"
#endif

// ##################################################################### //
// ############################## IMPORTS ############################## //
// ##################################################################### //

#include <algorithm>  // For std::count, std::all_of
#include <chrono>     // For std::chrono
#include <iostream>   // For console output
#include <string>     // For std::string
#include <vector>     // For std::vector

#include "log.hxx"  // For logger

// ##################################################################### //
// ######################### PATHS AND FOLDERS ######################### //
// ##################################################################### //

#ifndef HPLUS_HOME_DIR
#define HPLUS_HOME_DIR "placeholder"  // overwritten by cmake
#endif
#define HPLUS_LOG_DIR HPLUS_HOME_DIR "/logs/output_logs"
#define HPLUS_CPLEX_OUTPUT_DIR HPLUS_HOME_DIR "/logs/cpxout"

// ##################################################################### //
// ########################## C.L.I. ALGORITHMS ######################## //
// ##################################################################### //

#define HPLUS_CLI_ALG_HEUR "heur"
#define HPLUS_CLI_ALG_TL "tl"
#define HPLUS_CLI_ALG_VE "ve"
#define HPLUS_CLI_ALG_LM "lm"
#define HPLUS_CLI_HEUR_GREEDYCOST "greedycost"
#define HPLUS_CLI_HEUR_GREEDYCXE "greedycxe"
#define HPLUS_CLI_HEUR_GREEDYHMAX "hmax"
#define HPLUS_CLI_HEUR_GREEDYHADD "hadd"

// ##################################################################### //
// ########################### DEFAULT VALUES ########################## //
// ##################################################################### //

#define HPLUS_DEF_EXEC_STATUS exec_status::START
#define HPLUS_DEF_SOL_STATUS solution_status::NOTFOUND
#define HPLUS_DEF_LOG_FILE "N\\A"
#define HPLUS_DEF_LOG_NAME HPLUS_LOG_DIR "/hplus_log.log"
#define HPLUS_DEF_RUN_NAME "Unnamed run"
#define HPLUS_DEF_ALG HPLUS_CLI_ALG_VE
#define HPLUS_DEF_HEUR HPLUS_CLI_HEUR_GREEDYHADD
#define HPLUS_DEF_PREP true
#define HPLUS_DEF_WARM_START true
#define HPLUS_DEF_TIGHT_BOUNDS false
#define HPLUS_DEF_USING_CPLEX true
#define HPLUS_DEF_LOG false
#define HPLUS_DEF_WRITE_LP false
#define HPLUS_DEF_TIME_LIMIT 60
#define HPLUS_DEF_MLM true
#define HPLUS_DEF_CLM true
#define HPLUS_DEF_SEC true
#define HPLUS_DEF_FRACT true
#define HPLUS_DEF_FRACT_SEC false

// ====================================================== //
// ================== CPLEX PARAMETERS ================== //
// ====================================================== //

#define HPLUS_CPX_INT_ROUNDING 0.5
#define HPLUS_EPSILON 1e-6

#define HPLUS_DEF_CPX_SCREENOUTPUT CPX_OFF
#define HPLUS_DEF_CPX_CLONELOG -1
#define HPLUS_DEF_CPX_MIP_DISPLAY 3
#define HPLUS_DEF_CPX_TOL_GAP 0
#define HPLUS_DEF_CPX_TREE_MEM 12000
#define HPLUS_DEF_CPX_WORK_MEM 4096
#define HPLUS_DEF_CPX_STRAT_FILE 3

// ##################################################################### //
// ####################### PRINTING AND DEBUGGING ###################### //
// ##################################################################### //

#ifndef HPLUS_VERBOSE
#define HPLUS_VERBOSE 100  // overwritten by cmake
#endif
#ifndef HPLUS_WARN
#define HPLUS_WARN 0  // overwritten by cmake
#endif
#ifndef HPLUS_INTCHECK
#define HPLUS_INTCHECK 0  // overwritten by cmake
#endif

// ##################################################################### //
// #################### UTILS STRUCTS AND FUNCTIONS #################### //
// ##################################################################### //

/** Variable used to signal termination required (int cause cplex requires an
 * int) */
extern volatile int global_terminate;

/** Time keeper for execution time monitoring */
struct time_keeper {
    [[nodiscard]]
    double get_time() const {
        return (static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - this->timer).count())) /
               1'000;
    }
    explicit time_keeper() { this->timer = std::chrono::steady_clock::now(); }

   private:
    std::chrono::steady_clock::time_point timer;
};

/** Solution status for solution monitoring */
enum class solution_status { OPT = 0, FEAS = 1, INFEAS = 2, NOTFOUND = 404 };

/** Execution status for execution monitoring */
enum class exec_status { START = 0, READ_INPUT = 1, PROBLEM_SIMPL = 10, HEURISTIC = 20, MODEL_BUILD = 30, STOP_TL = 40, CPX_EXEC = 50, EXIT = 100 };

/** Split the string str using del as delimiter */
[[nodiscard]]
inline std::vector<std::string> split_string(const std::string& str, const char del) {
    std::vector<std::string> tokens;
    tokens.reserve(std::count(str.begin(), str.end(), del) + 1);

    size_t start{0}, end;

    while ((end = str.find(del, start)) != std::string::npos) {
        if (end > start)  // Avoid empty strings
            tokens.push_back(str.substr(start, end - start));
        start = end + 1;
    }

    // Add the last token if it exists
    if (start < str.length()) tokens.push_back(str.substr(start));

    return tokens;
}

/** Check if str is an integer between from and to (inclusive) */
[[nodiscard]]
inline bool isint(const std::string& str, const int from = std::numeric_limits<int>::min(), const int to = std::numeric_limits<int>::max()) {
    // Handle empty string
    if (str.empty()) return false;

    // Check for leading whitespace or sign
    size_t i{0};
    if (str[i] == '+' || str[i] == '-') i++;

    // Must have at least one digit
    if (i == str.length() || !std::isdigit(str[i])) return false;

    // Check remaining characters are digits
    for (; i < str.length(); i++) {
        if (!std::isdigit(str[i])) return false;
    }

    try {
        // Only convert to int if the string consists of valid digits
        int num{std::stoi(str)};
        return num >= from && num <= to;
    } catch (const std::out_of_range&) {
        // Handle overflow cases
        return false;
    }
}

template <typename T>
[[nodiscard]]
size_t find_insert_position(const std::vector<T>& vec, const T& target) {
    // Handle edge cases
    if (vec.empty() || target <= vec[0]) {
        return 0;  // Insert at the beginning
    }

    if (target > vec.back()) {
        return vec.size();  // Insert at the end
    }

    // Binary search
    size_t left = 0;
    size_t right = vec.size() - 1;

    while (left <= right) {
        size_t mid = left + (right - left) / 2;

        if (vec[mid] < target && (mid == vec.size() - 1 || vec[mid + 1] >= target)) {
            // Found the largest element smaller than target
            return mid + 1;  // Insert after this element
        } else if (vec[mid] < target) {
            left = mid + 1;  // Search right half
        } else {
            right = mid - 1;  // Search left half
        }
    }

    return left;  // Fallback (should not reach here with proper implementation)
}

/** Insert an element into a sorted vector, keeping it sorted */
template <typename T>
void insert_sorted(std::vector<T>& vec, const T& value) {
    int pos = find_insert_position(vec, value);
    vec.insert(vec.begin() + pos, value);
}

/** (Debugging) Exits with error message due to missing implementation, prints
 * through log the message msg formatted as error */
[[noreturn]]
inline void todo(const logger& log, const std::string& msg) {
    log.raise_error("%s: UNIMPLEMENTED.", msg.c_str());
}

/** (Debugging) Pauses the code execution until resuming, prints msg formatted
 * as warning */
inline void mypause(const std::string& msg = "") {
    std::string i;
    std::cout << msg << "(1 to exit, 0 to continue): ";
    std::cin >> i;
    if (i == "1") exit(1);
}

class timelimit_exception final : public std::exception {
    std::string msg;

   public:
    explicit timelimit_exception(const char* msg) : msg(msg) {}
    [[nodiscard]]
    const char* what() const noexcept override {
        return msg.c_str();
    }
};

/** Require user acknowledge to resume execution */
#define ACK_REQ(msg)                                           \
    {                                                          \
        std::cout << msg << ".\nPress any key to continue..."; \
        std::cin.ignore();                                     \
    }

/** Check for external execution termination required */
#define CHECK_STOP() global_terminate == 1

/** Define an assert function */
#ifndef ASSERT
#define ASSERT(cond)                                                                                             \
    {                                                                                                            \
        if (!(cond)) [[unlikely]] {                                                                              \
            std::cerr << "Assert check failed at " << __func__ << "(): " << __FILE__ << ":" << __LINE__ << "\n"; \
            exit(1);                                                                                             \
        }                                                                                                        \
    }
#endif

#define ASSERT_LOG(log, cond)                            \
    {                                                    \
        if (!(cond)) [[unlikely]] {                      \
            std::string msg = "Assert check failed at "; \
            msg += __func__;                             \
            msg += "() ";                                \
            msg += __FILE__;                             \
            msg += ":";                                  \
            msg += std::to_string(__LINE__);             \
            msg += "\n";                                 \
            log.raise_error("%s", msg.c_str());          \
        }                                                \
    }

#define CPX_HANDLE_CALL(log, code)                                                                                     \
    {                                                                                                                  \
        switch (code) {                                                                                                \
            case 1001:                                                                                                 \
                log.raise_error("OUT OF MEMORY.");                                                                     \
                break;                                                                                                 \
            case 0:                                                                                                    \
                break;                                                                                                 \
            default:                                                                                                   \
                log.raise_error("Unhandled CPLEX error code: %d at %s(): %s:%d.", code, __func__, __FILE__, __LINE__); \
        }                                                                                                              \
    }

/** Print a warning message only if the warn flag is set */
#if HPLUS_WARN
#define PRINT_WARN(log, msg) \
    {                        \
        log.print_warn(msg); \
    }
#else
#define PRINT_WARN(log, msg) \
    {                        \
    }
#endif

/** Print info message only with the right verbose settings */
#if HPLUS_VERBOSE >= 10
#define PRINT_INFO(log, msg) \
    {                        \
        log.print_info(msg); \
    }
#else
#define PRINT_INFO(log, msg) \
    {                        \
    }
#endif

/** Print info message only with the right verbose settings */
#if HPLUS_VERBOSE >= 20
#define PRINT_DEBUG(log, msg) PRINT_INFO(log, msg)
#else
#define PRINT_DEBUG(log, msg) \
    {                         \
    }
#endif

/** Print info message only with the right verbose settings */
#if HPLUS_VERBOSE >= 100
#define PRINT_VERBOSE(log, msg) PRINT_INFO(log, msg)
#else
#define PRINT_VERBOSE(log, msg) \
    {                           \
    }
#endif

/** Asserts to be made only if integrity checks flag is set */
#if HPLUS_INTCHECK
#define INTCHECK_ASSERT(cond) \
    {                         \
        ASSERT(cond);         \
    }
#define INTCHECK_ASSERT_LOG(log, cond) \
    {                                  \
        ASSERT_LOG(log, cond);         \
    }
#else
#define INTCHECK_ASSERT(cond) \
    {                         \
    }
#define INTCHECK_ASSERT_LOG(log, cond) \
    {                                  \
    }
#endif

#endif /* UTILS_H */