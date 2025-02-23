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

#include <iostream>
#include <climits>
#include <string>
#include <vector>
#include <chrono>

#include "log.hxx"

// ##################################################################### //
// ############################ CODE VERSION ########################### //
// ##################################################################### //

#define CODE_VERSION "23/02/2025"

// ##################################################################### //
// ######################### PATHS AND FOLDERS ######################### //
// ##################################################################### //

#ifndef HPLUS_HOME_DIR
#define HPLUS_HOME_DIR "placeholder"                // overwritten by cmake
#endif
#define HPLUS_LOG_DIR HPLUS_HOME_DIR"/logs/AAA_output_logs"
#define HPLUS_CPLEX_OUTPUT_DIR HPLUS_HOME_DIR"/logs/cpxout"

// ##################################################################### //
// ########################## C.L.I. ALGORITHMS ######################## //
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

// ##################################################################### //
// #################### UTILS STRUCTS AND FUNCTIONS #################### //
// ##################################################################### //

/** Variable used to signal termination required (int cause cplex requires an int) */
extern volatile int global_terminate;

/** Time keeper for execution time monitoring */
class time_keeper {
    public:
        double get_time() const { return ((double) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - this->timer).count())/1000; }
        time_keeper() { this->timer = std::chrono::steady_clock::now(); }
    private:
        std::chrono::steady_clock::time_point timer;
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

/** Split the string _str using _del as delimiter */
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

/** Check if _str is an integer between from and to (inclusive) */
static inline bool isint(const std::string& _str, const int _from = INT_MIN, const int _to = INT_MAX) {
    try {
        int num = stoi(_str);
        return num >= _from && num <= _to;
    } catch (std::invalid_argument&) { return false; }

}

/** (Debugging) Exits with error message due to missing implementation, prints through _log the message _msg formatted as error */
static inline void todo(const logger& _log, const std::string& _msg) { _log.raise_error("%s: UNIMPLEMENTED.", _msg.c_str()); }

/** (Debugging) Pauses the code execution until resuming, prints msg formatted as warning */
static inline void pause(const std::string& _msg) {                      
    size_t i = 0;
    std::cout << _msg << "(1 to exit, 0 to continue): ";
    std::cin >> i;
    if (i > 0) exit(1);
}

class timelimit_exception : public std::exception {
    private:
        std::string msg;
    public:
        timelimit_exception(const char* _m) : msg(_m) {}
        const char* what() const throw() { return msg.c_str(); }
};

/** Require user acknowledge to resume execution */
#define _ACK_REQ(msg) {                                         \
    std::cout << msg << ".\nPress any key to continue...";      \
    std::cin.ignore();                                          \
}

/** Check for external execution termination required */
#define _CHECK_STOP() global_terminate == 1

/** Define an assert function */
#ifndef _ASSERT
#define _ASSERT(cond) {                                                                                     \
if (!(cond)) {                                                                                              \
    std::cerr << "Assert check failed at " << __func__ << "(): " << __FILE__ << ":"<< __LINE__ << "\n";     \
    exit(1);                                                                                                \
}                                                                                                           \
}
#endif

/** Print a warning message only if the warn flag is set */
#if HPLUS_WARN
#define _PRINT_WARN(_msg) {  \
_l.print_warn(_msg);         \
}
#else
#define _PRINT_WARN(_msg) {}
#endif

/** Print info message only with the right verbose settings */
#if HPLUS_VERBOSE >= 10
#define _PRINT_INFO(_msg) {  \
_l.print_info(_msg);         \
}
#else
#define _PRINT_INFO(_msg) {}
#endif

/** Print info message only with the right verbose settings */
#if HPLUS_VERBOSE >= 20
#define _PRINT_DEBUG(_msg) _PRINT_INFO(_msg)
#else
#define _PRINT_DEBUG(_msg) {}
#endif

/** Print info message only with the right verbose settings */
#if HPLUS_VERBOSE >= 100
#define _PRINT_VERBOSE(_msg) _PRINT_INFO(_msg)
#else
#define _PRINT_VERBOSE(_msg) {}
#endif

/** Asserts to be made only if integrity checks flag is set */
#if HPLUS_INTCHECK
#define _INTCHECK_ASSERT(_cond) {     \
_ASSERT(_cond);               \
}
#else
#define _INTCHECK_ASSERT(_cond) {}
#endif

#endif /* UTILS_H */