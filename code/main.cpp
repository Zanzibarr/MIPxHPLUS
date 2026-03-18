#include <termios.h>  // For STDIN_FILENO
#include <unistd.h>   // For STDIN_FILENO

#include <new>
#include <tuple>

#include "argparser.hpp"
#include "hplus_algs.hpp"
#include "limits.hxx"
#include "timer.hxx"
#include "utils.hpp"

using std::tuple;

Timer GLOBAL_TIMER;

static void signal_callback_handler(const int /*signal*/) {
    LOG_WARN_S("---------------------------------------------------");
    LOG_WARN_S(" > Ctrl+C signal detected, terminating execution. <");
    LOG_WARN_S("---------------------------------------------------");
    if (CHECK_STOP()) {
        [[unlikely]] _Exit(EXIT_FAILURE);
    }
    GLOBAL_TERMINATE_CONDITION = 1;
}

[[nodiscard]]
static auto init() -> std::tuple<hplus::execution, hplus::instance, hplus::statistics> {
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
    hplus::statistics stats{};
    hplus::init(stats);
    GLOBAL_TIMER.start();
    return {exec, inst, stats};
}

static void close() { timelim::cancel_time_limit(); }

auto main(const int argc, const char** argv) -> int {
    try {
        auto [exec, inst, stats] = init();
        double start_time = GET_TIME();

        parse_cli(argc, argv, exec);
        hplus::read_file(exec, inst, stats);

        hplus::print(exec);
        if (exec.type == hplus::exec_type::INFO) {
            hplus::print(inst);
        }

        switch (exec.type) {
            case hplus::exec_type::INFO:
                close();
                return EXIT_SUCCESS;
            case hplus::exec_type::RUN:
                hplus::run(exec, inst, stats);
                break;
            default:
                LOG_ERROR_S("Unhandled execution type in main: " + std::to_string(static_cast<int>(exec.type)));
        }

        stats.total = GET_TIME() - start_time;

        // NOTE: All updates to the best solution and statistics, must be done immediatelly (es. in the cand callback or at the end of a search
        // algorithm), since however we get here, I assume that inst and stats are updated and no other operations are needed.

        hplus::print_sol(inst);
        if (inst.sol_s == hplus::solution_status::INFEAS) {
            stats.lower_bound = INFBOUND_DBL;
        }
        if (inst.sol_s == hplus::solution_status::LOST) {
            stats.status = HPLUS_STATUS_LOST;
        }
        hplus::print(stats);
        close();

    } catch (std::bad_alloc& e) {
        LOG_ERROR_S("OUT OF MEMORY");
    }

    LOG_SUCCESS_S("Execution terminated");

    return EXIT_SUCCESS;
}
