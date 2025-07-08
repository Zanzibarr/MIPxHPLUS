#include <termios.h>  // For STDIN_FILENO
#include <unistd.h>   // For STDIN_FILENO

#include <new>
#include <tuple>

#include "domain/hplus_algs.hpp"
#include "utils/argparser.hpp"

static void signal_callback_handler([[maybe_unused]] const int _) {
    if (CHECK_STOP()) [[unlikely]]
        _Exit(EXIT_FAILURE);
    GLOBAL_TERMINATE_CONDITION = 1;
}

[[nodiscard]]
static std::tuple<hplus::execution, hplus::instance, hplus::statistics> init() {
    signal(SIGINT, signal_callback_handler);
    // Hide ^C from terminal
    struct termios t{};
    tcgetattr(STDIN_FILENO, &t);
    t.c_lflag &= ~ECHOCTL;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &t);
    hplus::execution exec;
    hplus::init(exec);
    hplus::instance inst;
    hplus::init(inst);
    hplus::statistics stats;
    hplus::init(stats);
    return std::tuple(exec, inst, stats);
}

static void close() { timelim::cancel_time_limit(); }

int main(const int argc, const char** argv) {
    try {
        auto [exec, inst, stats] = init();
        double start_time = GET_TIME();

        parse_cli(argc, argv, exec);
        hplus::print(exec);

        // Input file reading
        hplus::read_file(exec, inst, stats);
        if (exec.type == hplus::exec_type::INFO || STATS_VERBOSE()) hplus::print(inst);

        // Execution
        switch (exec.type) {
            case hplus::exec_type::INFO:
                close();
                return EXIT_SUCCESS;
            case hplus::exec_type::RUN:
                hplus::run(exec, inst, stats);
                break;
            default:
                LOG_ERROR << "Unhandled execution type in main: " << static_cast<int>(exec.type);
        }

        stats.total = GET_TIME() - start_time;

        // NOTE: All updates to the best solution and statistics, must be done immediatelly (es. in the cand callback or at the end of a search
        // algorithm), since however we get here, I assume that inst and stats are updated and no other operations are needed.

        hplus::print_sol(inst);
        if (inst.sol_s == hplus::solution_status::INFEAS) stats.lower_bound = 1e20;
        if (inst.sol_s == hplus::solution_status::LOST) stats.status = HPLUS_STATUS_LOST;
        if (STATS_VERBOSE()) hplus::print(stats);
        close();

    } catch (std::bad_alloc& e) {
        LOG_ERROR << "OUT OF MEMORY";
    }

    LOG_SUCCESS << "Execution terminated";

    return EXIT_SUCCESS;
}
