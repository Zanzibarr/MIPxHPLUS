#include <termios.h>  // For STDIN_FILENO
#include <unistd.h>   // For STDIN_FILENO

#include <new>
#include <tuple>

#include "argparser.hpp"
#include "hplus_algs.hpp"
#include "limits.hxx"
#include "logger.hxx"
#include "timer.hxx"
#include "utils.hpp"

using std::tuple;

Timer GLOBAL_TIMER;

namespace {
void signal_callback_handler(const int /*signal*/) {
    LOG_WARN_S("---------------------------------------------------");
    LOG_WARN_S(" > Ctrl+C signal detected, terminating execution. <");
    LOG_WARN_S("---------------------------------------------------");
    if (CHECK_STOP()) {
        [[unlikely]] _Exit(EXIT_FAILURE);
    }
    GLOBAL_TERMINATE_CONDITION = 1;
}

[[nodiscard]]
auto init() -> std::tuple<hplus::execution, hplus::instance, hplus::statistics> {
    signal(SIGINT, signal_callback_handler);
    // Hide ^C from terminal
    struct termios t{};
    tcgetattr(STDIN_FILENO, &t);
    t.c_lflag &= static_cast<tcflag_t>(~ECHOCTL);
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

void close() { timelim::cancel_time_limit(); }
}  // namespace

auto main(const int argc, const char** argv) -> int {
    try {
        auto [exec, inst, stats] = init();

        {
            auto _total = make_scoped_timer<"total">(STATS);

            parse_cli(argc, argv, exec);
            hplus::read_file(exec, inst, stats);

            hplus::print(exec);
            hplus::print(inst);

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

            // NOTE: All updates to the best solution and statistics, must be done immediatelly (es. in the cand callback or at the end of a search
            // algorithm), since however we get here, I assume that inst and stats are updated and no other operations are needed.

            hplus::print_sol(inst);
            if (inst.sol_s == hplus::solution_status::INFEAS) {
                stats.lower_bound = INFBOUND_DBL;
            }
            if (inst.sol_s == hplus::solution_status::LOST) {
                stats.status = HPLUS_STATUS_LOST;
            }
        }
        hplus::print(stats);
        close();

    } catch (std::bad_alloc& e) {
        LOG_ERROR_S("OUT OF MEMORY");
    } catch (std::exception& e) {
        LOG_ERROR_S("Exception: " + std::string(e.what()));
    } catch (...) {
        LOG_ERROR_S("Unknown exception");
    }

    LOG_SUCCESS_S("Execution terminated");
    default_logger().flush();

    return EXIT_SUCCESS;
}
