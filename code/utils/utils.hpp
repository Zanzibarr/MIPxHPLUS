/**
 * Utilities and constants used in various parts of this project
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#pragma once

// ##################################################################### //
// ############################## VERSION ############################## //
// ##################################################################### //

#define VERSION "3.1.6"
#define COMPILE_DATETIME __DATE__ + " " + __TIME__

// ##################################################################### //
// ############################## IMPORTS ############################## //
// ##################################################################### //

#include <random>
#include <vector>

#include "timer.hxx"

// ##################################################################### //
// ############################# CLI PARSER ############################ //
// ##################################################################### //

#define HPLUS_CLI_HELP_FLAG "h"
#define HPLUS_CLI_INFO_FLAG "info"
#define HPLUS_CLI_RUN_FLAG "run"

#define HPLUS_CLI_ALG_FLAG "a"
#define HPLUS_CLI_ALG_FLAG_TL "tl"
#define HPLUS_CLI_ALG_FLAG_VE "ve"
#define HPLUS_CLI_ALG_FLAG_CUTS "cuts"
#define HPLUS_CLI_ALG_FLAG_GREEDYCOST "gc"
#define HPLUS_CLI_ALG_FLAG_GREEDYCXE "gcxe"
#define HPLUS_CLI_ALG_FLAG_GREEDYHMAX "ghm"
#define HPLUS_CLI_ALG_FLAG_GREEDYHADD "gha"

#define HPLUS_CLI_WS_FLAG "ws"
#define HPLUS_CLI_WS_FLAG_NONE "0"
#define HPLUS_CLI_WS_FLAG_GREEDYCOST "gc"
#define HPLUS_CLI_WS_FLAG_GREEDYCXE "gcxe"
#define HPLUS_CLI_WS_FLAG_GREEDYHMAX "ghm"
#define HPLUS_CLI_WS_FLAG_GREEDYHADD "gha"

#define HPLUS_CLI_CUTOFF_FLAG "cutoff"
#define HPLUS_CLI_PREP_FLAG "prep"
#define HPLUS_CLI_PREP_LMCUT_FLAG "prep-lm"
#define HPLUS_CLI_PREP_LMCUT_MIN_FLAG "prep-lm-min"

#define HPLUS_CLI_LOG_FLAG "log"
#define HPLUS_CLI_TIMELIMIT_FLAG "t"
#define HPLUS_CLI_THREADS_FLAG "threads"
#define HPLUS_CLI_MEMORYLIMIT_FLAG "mem"
#define HPLUS_CLI_SEED_FLAG "s"

#define HPLUS_CLI_CANDCUTS_FLAG "cand"
#define HPLUS_CLI_CANDLM_MIN_FLAG "cand-lm-min"

#define HPLUS_CLI_FRACTCUTS_FLAG "fract"
#define HPLUS_CLI_FRACTCUTS_AT_NODES_FLAG "fract-nodes"
#define HPLUS_CLI_FRACTLM_MIN_FLAG "fract-lm-min"

#define HPLUS_CLI_MINIMIZATION_BOUND_IT "minlm-it"
#define HPLUS_CLI_MINIMIZATION_BOUND_LH "minlm-lh"
#define HPLUS_CLI_MINIMIZATION_SORT "minlm-sort"
#define HPLUS_CLI_MINIMIZATION_IMPROV "minlm-improv"
#define HPLUS_CLI_MINIMIZATION_BOUND_VIOL "minlm-viol"

#define HPLUS_CLI_CUTLOOP_FLAG "cloop"
#define HPLUS_CLI_CL_PRUNING_FLAG "cl-prune"
#define HPLUS_CLI_CUTLOOP_MIN_ITER_FLAG "cl-min-iter"
#define HPLUS_CLI_CUTLOOP_IMPROVEMENT_FLAG "cl-improv"
#define HPLUS_CLI_CUTLOOP_PAST_ITER_FLAG "cl-past-iter"
#define HPLUS_CLI_CUTLOOP_GAP_STOP_FLAG "cl-gap"

#define HPLUS_CLI_INOUT_FLAG "inout"
#define HPLUS_CLI_INOUT_MAX_ITER_FLAG "io-max-it"
#define HPLUS_CLI_INOUT_WEIGHT_FLAG "io-w"
#define HPLUS_CLI_INOUT_WEIGHT_UPD_FLAG "io-wupd"
#define HPLUS_CLI_TESTING_FLAG "test"

// ##################################################################### //
// ######################### PATHS AND FOLDERS ######################### //
// ##################################################################### //

#ifndef HPLUS_HOME_DIR
#define HPLUS_HOME_DIR "placeholder"  // overwritten by cmake
#endif
#define HPLUS_LOG_DIR HPLUS_HOME_DIR "/logs/output_logs"
#define HPLUS_CPLEX_OUTPUT_DIR HPLUS_HOME_DIR "/logs/cpxout"

// ##################################################################### //
// ############################ CLI DEFAULTS ########################### //
// ##################################################################### //

#define INFBOUND_DBL 1e20
#define INFBOUND_INT std::numeric_limits<unsigned int>::max()

#define HPLUS_DEF_RANDOM_SEED 2122187
#define HPLUS_DEF_ALG 2
#define HPLUS_DEF_ALG_STRING HPLUS_CLI_ALG_FLAG_CUTS

#define HPLUS_DEF_WS 4
#define HPLUS_DEF_WS_STRING HPLUS_CLI_WS_FLAG_GREEDYHADD

#define HPLUS_DEF_CUTOFF -1
#define HPLUS_DEF_PREP true
#define HPLUS_DEF_PREP_LMCUT "aiv"
#define HPLUS_ALL_LMCUT_PCF "aivr"
#define HPLUS_DEF_PREP_LMCUT_MIN "c"

#define HPLUS_DEF_LOG "0"
#define HPLUS_DEF_TIMELIMIT 60
#define HPLUS_DEF_THREADS 32
#define HPLUS_DEF_MEMORYLIMIT 4050

#define HPLUS_DEF_CANDCUTS "l"
#define HPLUS_DEF_CANDLM_MIN "c"

#define HPLUS_DEF_FRACTCUTS "l"
#define HPLUS_DEF_FRACTCUTS_AT_NODES true
#define HPLUS_DEF_FRACTLM_MIN "g"

#define HPLUS_DEF_MINIMIZATION_IT 1000
#define HPLUS_DEF_MINIMIZATION_LH 10
#define HPLUS_DEF_MINIMIZATION_SORT true
#define HPLUS_DEF_MINIMIZATION_IMPROV true
#define HPLUS_DEF_MINIMIZATION_VIOL 0

#define HPLUS_DEF_CUSTOM_CUTLOOP false
#define HPLUS_DEF_CL_PRUNING true
#define HPLUS_DEF_CL_MIN_ITER 20
#define HPLUS_DEF_CL_IMPROV 0.005
#define HPLUS_DEF_CL_PAST_ITER 10
#define HPLUS_DEF_CL_GAP_STOP .1
#define HPLUS_DEF_INOUT false
#define HPLUS_DEF_IO_MAX_IT 4
#define HPLUS_DEF_IO_WEIGHT .4
#define HPLUS_DEF_IO_WEIGHT_UPD .5

// ##################################################################### //
// ######################### EXECUTION DEFAULTS ######################## //
// ##################################################################### //

#define HPLUS_CPX_INT_ROUNDING 0.5
#define HPLUS_EPSILON 1e-6
#define HPLUS_FRACT_LM_VIOLATION_EPSILON 1e-1

#define HPLUS_DEF_CPX_SCREENOUTPUT CPX_OFF
#define HPLUS_DEF_CPX_CLONELOG -1
#define HPLUS_DEF_CPX_MIP_DISPLAY 3
#define HPLUS_DEF_CPX_TOL_GAP 0
#define HPLUS_DEF_CPX_TREE_MEM 12000
#define HPLUS_DEF_WORKMEM HPLUS_DEF_MEMORYLIMIT
#define HPLUS_DEF_CPX_STRAT_FILE 3

#define HPLUS_STATUS_OPT 0
#define HPLUS_STATUS_INFEAS 1
#define HPLUS_STATUS_FEAS 2
#define HPLUS_STATUS_NOTFOUND 3
#define HPLUS_STATUS_LOST 4

// ##################################################################### //
// #################### UTILITY FUNCTIONS AND MACROS ################### //
// ##################################################################### //

// Random number generator
inline std::mt19937 g_rng;

inline void init_rng(unsigned int seed) { g_rng.seed(seed); }

#define ASSERT(cond)                                                                                                             \
    {                                                                                                                            \
        if (!(cond)) [[unlikely]] {                                                                                              \
            LOG_ERROR_S("Assert check failed at " + std::string(__func__) + "(): " + __FILE__ + ":" + std::to_string(__LINE__)); \
        }                                                                                                                        \
    }

#define CPX_HANDLE_CALL(code)                                                                                                     \
    {                                                                                                                             \
        switch (code) {                                                                                                           \
            case 1001: /*CPXERR_NO_MEMORY*/                                                                                       \
                [[fallthrough]];                                                                                                  \
            case 1234: /*CPXERR_THREAD_FAILED*/                                                                                   \
                throw std::bad_alloc();                                                                                           \
                break;                                                                                                            \
            case 11: /*CPX_STAT_ABORT_TIME_LIM*/                                                                                  \
                [[fallthrough]];                                                                                                  \
            case 13: /*CPX_STAT_ABORT_USER*/                                                                                      \
                [[fallthrough]];                                                                                                  \
            case 0:                                                                                                               \
                break;                                                                                                            \
            default:                                                                                                              \
                LOG_ERROR_S("Unhandled CPLEX error code: " + std::to_string(code) + " at " + __func__ + "(): " + __FILE__ + ":" + \
                            std::to_string(__LINE__));                                                                            \
                break;                                                                                                            \
        }                                                                                                                         \
    }

[[nodiscard]]
inline auto compile_date() -> std::string {
    return std::string("Compiled on ") + COMPILE_DATETIME;
}

[[nodiscard]]
inline auto today() -> std::string {
    auto now = std::chrono::system_clock::now();
    std::time_t end_time = std::chrono::system_clock::to_time_t(now);
    std::string time_str = std::ctime(&end_time);
    if (!time_str.empty() && time_str.back() == '\n') {
        time_str.pop_back();  // remove trailing newline
    }
    return time_str;
}

[[nodiscard]]
inline auto version() -> std::string {
    return "Version: " VERSION;
}

[[nodiscard]]
inline auto isint(const std::string& str, const int from = std::numeric_limits<int>::min(), const int to = std::numeric_limits<int>::max()) -> bool {
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
        // Only convert to int if the string consists of valid digits
        int num{std::stoi(str)};
        return num >= from && num <= to;
    } catch (const std::out_of_range&) {
        // Handle overflow cases
        return false;
    }
}

[[nodiscard]]
inline auto split_string(const std::string& str, const char del) -> std::vector<std::string> {
    std::vector<std::string> tokens;
    tokens.reserve(static_cast<size_t>(std::count(str.begin(), str.end(), del)) + 1);

    size_t start{0};
    size_t end{0};

    while ((end = str.find(del, start)) != std::string::npos) {
        if (end > start) {  // Avoid empty strings
            tokens.push_back(str.substr(start, end - start));
        }
        start = end + 1;
    }

    // Add the last token if it exists
    if (start < str.length()) {
        tokens.push_back(str.substr(start));
    }

    return tokens;
}

/**
 * @brief Convert to string the content of a vector
 *
 * @tparam T The type of the elements in the vector (note: the elements of the vector will be added to the string using the std::to_string
 * function)
 * @param v The vector
 * @param size = -1 (all) The number of elements to be shown in the string (first size/2 and last size/2 if v.size() > size)
 * @return st::string The string representation of the vector (using std::to_string for each T element of the vector)
 */
template <typename T>
[[nodiscard]]
static inline auto vtos(std::vector<T> v, int size = -1) -> std::string {
    if (size == -1) {
        size = static_cast<int>(v.size());
    }
    std::string s;
    if (v.size() <= static_cast<unsigned int>(size)) {
        for (const auto& x : v) {
            s.append(std::to_string(x)).append(";");
        }
    } else {
        for (unsigned int i = 0; i < static_cast<unsigned int>(size) / 2; i++) {
            s.append(std::to_string(v[i])).append(";");
        }
        s.append("...[").append(std::to_string(v.size() - static_cast<unsigned int>(size))).append("];");
        for (unsigned int i = static_cast<unsigned int>(size) / 2; i > 0; i--) {
            s.append(std::to_string(v[v.size() - i])).append(";");
        }
    }
    return s;
}

extern Timer GLOBAL_TIMER;

// .elapsed returns milliseconds... need to convert it to seconds
#define GET_TIME() (GLOBAL_TIMER.elapsed() / 1'000)

/// @brief Exception thrown when a predefined time limit is exceeded.
class timelimit_exception final : public std::runtime_error {
   public:
    /// @brief Constructs a timelimit_exception with a descriptive message.
    /// @param message The message describing the exception.
    explicit timelimit_exception(const std::string& message) : std::runtime_error(message) {}
};
