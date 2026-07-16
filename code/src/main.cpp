#include <termios.h>  // For STDIN_FILENO
#include <unistd.h>   // For STDIN_FILENO

#include <argparser.hxx>
#include <cstdlib>
#include <exception>
#include <limits.hxx>
#include <logger.hxx>
#include <parameters.hxx>
#include <scope_guard.hxx>

#include "cli_parser.hpp"
#include "solver.hpp"
#include "utils.hpp"
#include "validation.hpp"

namespace {

void signal_callback_handler(const int /*signal*/) {
    // Only async-signal-safe work here: flip the lock-free global stop flags.
    // First Ctrl+C requests a cooperative stop; a second one hard-exits.
    if (global_limits::time_flag.load(std::memory_order_relaxed)) [[unlikely]] {
        _Exit(EXIT_FAILURE);
    }
    global_limits::time_flag.store(true, std::memory_order_relaxed);
    GLOBAL_TERMINATE_CONDITION = 1;  // tell CPLEX to stop
}

auto init(ParameterRegistry& params, Logger& logger) -> void {
    // ~~~~~~~~~~~~ signal handler ~~~~~~~~~~~ //
    signal(SIGINT, signal_callback_handler);
    // Hide ^C from terminal
    struct termios term{};
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag &= static_cast<tcflag_t>(~ECHOCTL);
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &term);

    // ~~~~~~~~~~~~~~~~ limits ~~~~~~~~~~~~~~~ //
    if (params.get<"time_limit", int>() > 0) {
        set_time_limit(static_cast<unsigned int>(params.get<"time_limit", int>()), [&logger] {
            GLOBAL_TERMINATE_CONDITION = 1;  // tell CPLEX to stop
            logger[WARNING] << "Time limit reached.";
        });
    }

    // Warn user of debug mode being used
#ifndef NDEBUG
    logger[WARNING] << "Asserts are enabled";
#endif
}

auto solve(const ParameterRegistry& params, Logger& logger) -> std::vector<std::string> {
    logger << date_compile();
    logger << date_today();
    logger << version();
    logger << params;

    Solver solver(params, logger);
    solver.solve();

    solver.show();

    return solver.get();
}

}  // namespace

auto main(const int argc, char** argv) -> int {
    Logger logger;

    auto params = parse_cli(argc, argv, logger);

    init(params, logger);
    auto _timerthread_scope = on_scope_exit([]() { cancel_time_limit(); });

    std::vector<std::string> solution;
    try {
        solution = solve(params, logger);
    } catch (const std::exception& e) {
        logger[FATAL] << std::format("Caught exception in main(): {}", e.what());
    } catch (...) {
        logger[FATAL] << std::format("Something wrong happened in main().");
    }

    // validate(params.get<cli_desc::input, std::string>(), solution, logger);

    logger[SUCCESS] << "Execution terminated succesfully";

    return EXIT_SUCCESS;
}
