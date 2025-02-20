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
// ############################## IMPORTS ############################## //
// ##################################################################### //

#include <climits>
#include <string>
#include <vector>
#include <chrono>
#include <random>
#include <iostream>

#include "log.hxx"

#define CODE_VERSION "20/02/2025"

// ##################################################################### //
// ######################### PATHS AND FOLDERS ######################### //
// ##################################################################### //

#ifndef HPLUS_HOME_DIR
#define HPLUS_HOME_DIR "placeholder"                // overwritten by cmake
#endif
#define HPLUS_LOG_DIR HPLUS_HOME_DIR"/logs/AAA_output_logs"
#define HPLUS_CPLEX_OUTPUT_DIR HPLUS_HOME_DIR"/logs/cpxout"

// ##################################################################### //
// ########################## C.L.I. ARGUMENTS ######################### //
// ##################################################################### //

#define HPLUS_CLI_ALG_IMAI              "imai"
#define HPLUS_CLI_ALG_RANKOOH           "rankooh"
#define HPLUS_CLI_ALG_DYNAMIC_SMALL     "dynamic-s"
#define HPLUS_CLI_ALG_DYNAMIC_LARGE     "dynamic-l"
#define HPLUS_CLI_ALG_HEUR              "heur"

// ##################################################################### //
// ########################### DEFAULT VALUES ########################## //
// ##################################################################### //

#define HPLUS_CPX_INT_ROUNDING 0.5

// ##################################################################### //
// ####################### PRINTING AND DEBUGGING ###################### //
// ##################################################################### //

#ifndef HPLUS_VERBOSE
#define HPLUS_VERBOSE 5                            // overwritten by cmake
#endif
#ifndef HPLUS_WARN
#define HPLUS_WARN 1                                // overwritten by cmake
#endif
#ifndef HPLUS_INTCHECK
#define HPLUS_INTCHECK 1                            // overwritten by cmake
#endif

extern volatile int global_terminate;

// ##################################################################### //
// #################### UTILS STRUCTS AND FUNCTIONS #################### //
// ##################################################################### //

/** Time keeper for execution time monitoring */
struct time_keeper {
    std::chrono::steady_clock::time_point timer;
    void start_timer() { this->timer = std::chrono::steady_clock::now(); }
    double get_time() const { return ((double) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - this->timer).count())/1000; }
};
/** Solution status for solution monitoring */
enum solution_status {
    OPT = 0,
    FEAS = 1,
    INFEAS = 2,
    NOTFOUND = 404
};
/** Execution status for execution monitoring */
enum exec_status {
    START = 0,
    READ_INPUT = 1,
    PROBLEM_SIMPL = 10,
    HEURISTIC = 20,
    MODEL_BUILD = 30,
    STOP_TL = 40,
    CPX_EXEC = 50,
    EXIT = 100
};
/** Split the string @param _str using @param _del as delimiter */
static inline std::vector<std::string> split_string(const std::string& _str, char _del) {
    std::vector<std::string> tokens;
    std::string tmp;
    for (int i = 0; i < _str.length(); i++) {
        if (_str[i] == _del) {
            if (!tmp.empty()) tokens.push_back(tmp);
            tmp = "";
        } else tmp += _str[i];
    }
    if (!tmp.empty()) tokens.push_back(tmp);
    return tokens;
}
/** Check if @param _str is an integer between @param from and @param to (inclusive) */
static inline bool isint(const std::string& _str, const int _from = INT_MIN, const int _to = INT_MAX) {
    try {
        int num = stoi(_str);
        return num >= _from && num <= _to;
    } catch (std::invalid_argument&) { return false; }

}
/** Exits with error message due to missing implementation, prints through @param _log the message @param _msg formatted as error */
static inline void todo(const logger& _log, const std::string& _msg) { _log.raise_error("%s: UNIMPLEMENTED.", _msg.c_str()); }
/** Pauses the code execution until resuming, prints @param msg formatted as warning */
static inline void pause(const std::string& _msg) {
    size_t i = 0;
    std::cout << _msg << "(1 to exit, 0 to continue): ";
    std::cin >> i;
    if (i > 0) exit(1);
}

#ifndef assert
#define assert(cond)                                                                                        \
if (!(cond)) {                                                                                                \
    std::cerr << "Assert check failed at " << __func__ << "(): " << __FILE__ << ":"<< __LINE__ << "\n";     \
    exit(1);                                                                                                \
}
#endif

#if HPLUS_WARN == 1
#define _PRINT_WARN(msg) {  \
_l.print_warn(msg);         \
}
#else
#define _PRINT_WARN(msg) {}
#endif

#if HPLUS_VERBOSE > 1
#define _PRINT_INFO(msg) {  \
_l.print_info(msg);         \
}
#else
#define _PRINT_INFO(msg) {}
#endif

#if HPLUS_INTCHECK
#define _ASSERT(cond) {     \
assert(cond);               \
}
#else
#define _ASSERT(cond) {}
#endif

#endif /* UTILS_H */