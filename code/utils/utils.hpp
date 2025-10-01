/**
 * Utilities and constants used in various parts of this project
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#ifndef HPLUS_UTILS_HPP
#define HPLUS_UTILS_HPP

// ##################################################################### //
// ############################## VERSION ############################## //
// ##################################################################### //

#define VERSION "2.2.9"

// ##################################################################### //
// ############################## IMPORTS ############################## //
// ##################################################################### //

#include <vector>

#include "../external/limits.hpp"

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
#define HPLUS_CLI_PREP_FLAG "prep"
#define HPLUS_CLI_LOG_FLAG "log"
#define HPLUS_CLI_TIMELIMIT_FLAG "t"
#define HPLUS_CLI_THREADS_FLAG "threads"
#define HPLUS_CLI_MEMORYLIMIT_FLAG "mem"
#define HPLUS_CLI_VERBOSE_FLAG "v"
#define HPLUS_CLI_FRACTCUTS_FLAG "fract"
#define HPLUS_CLI_FRACTCUTS_AT_NODES_FLAG "fract-nodes"
#define HPLUS_CLI_CANDCUTS_FLAG "cand"
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

#define HPLUS_DEF_ALG 2
#define HPLUS_DEF_ALG_STRING HPLUS_CLI_ALG_FLAG_CUTS
#define HPLUS_DEF_WS 4
#define HPLUS_DEF_WS_STRING HPLUS_CLI_WS_FLAG_GREEDYHADD
#define HPLUS_DEF_PREP true
#define HPLUS_DEF_LOG "0"
#define HPLUS_DEF_TIMELIMIT 60
#define HPLUS_DEF_THREADS 32
#define HPLUS_DEF_MEMORYLIMIT 4050
#define HPLUS_DEF_VERBOSE 3
#define HPLUS_DEF_CANDCUTS "fcs"
#define HPLUS_DEF_FRACTCUTS "0"
#define HPLUS_DEF_FRACTCUTS_AT_NODES true
#define HPLUS_DEF_CUSTOM_CUTLOOP false
#define HPLUS_DEF_CL_PRUNING true
#define HPLUS_DEF_CL_MIN_ITER 20
#define HPLUS_DEF_CL_IMPROV 0.005
#define HPLUS_DEF_CL_PAST_ITER 10
#define HPLUS_DEF_CL_GAP_STOP .1
#define HPLUS_DEF_INOUT true
#define HPLUS_DEF_IO_MAX_IT 4
#define HPLUS_DEF_IO_WEIGHT .4
#define HPLUS_DEF_IO_WEIGHT_UPD .5

// ##################################################################### //
// ######################### EXECUTION DEFAULTS ######################## //
// ##################################################################### //

#define HPLUS_CPX_INT_ROUNDING 0.5
#define HPLUS_EPSILON 1e-6

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

#define ASSERT(cond)                                                                                     \
    {                                                                                                    \
        if (!(cond)) [[unlikely]] {                                                                      \
            LOG_ERROR << "Assert check failed at " << __func__ << "(): " << __FILE__ << ":" << __LINE__; \
        }                                                                                                \
    }

#define CPX_HANDLE_CALL(code)                                                                                                       \
    {                                                                                                                               \
        switch (code) {                                                                                                             \
            case 1001: /*CPXERR_NO_MEMORY*/                                                                                         \
                [[fallthrough]];                                                                                                    \
            case 1234: /*CPXERR_THREAD_FAILED*/                                                                                     \
                throw std::bad_alloc();                                                                                             \
                break;                                                                                                              \
            case 0:                                                                                                                 \
                break;                                                                                                              \
            default:                                                                                                                \
                LOG_ERROR << "Unhandled CPLEX error code: " << code << " at " << __func__ << "(): " << __FILE__ << ":" << __LINE__; \
                break;                                                                                                              \
        }                                                                                                                           \
    }

[[nodiscard]]
inline std::string today() {
    auto now = std::chrono::system_clock::now();
    std::time_t end_time = std::chrono::system_clock::to_time_t(now);
    std::string time_str = std::ctime(&end_time);
    if (!time_str.empty() && time_str.back() == '\n') {
        time_str.pop_back();  // remove trailing newline
    }
    return time_str;
}

[[nodiscard]]
inline std::string version() {
    return "Version: " VERSION;
}

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

[[nodiscard]]
inline const std::vector<std::string> split_string(const std::string& str, const char del) {
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

class timelimit_exception final : public std::exception {
    std::string msg;

   public:
    explicit timelimit_exception(const char* msg) : msg(msg) {}
    [[nodiscard]]
    const char* what() const noexcept override {
        return msg.c_str();
    }
};

#endif