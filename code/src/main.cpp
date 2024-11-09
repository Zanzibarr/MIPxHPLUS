#include <csignal>
#include <thread>
#include <sys/stat.h>
#include "../include/algorithms.hpp"

void HPLUS_start() {

    HPLUS_env.start_timer();

    HPLUS_env.status = my::status::NOTFOUND;
    HPLUS_env.cpx_terminate = 0;
    HPLUS_env.cplex_running = false;

    HPLUS_env.model_enhancements = true;
    HPLUS_env.imai_var_bound = true;
    HPLUS_env.heur_1 = true;
    HPLUS_env.heur_2 = true;
    HPLUS_env.warm_start = true;

    HPLUS_stats.parsing_time = 0;
    HPLUS_stats.opt_time = 0;
    HPLUS_stats.heuristic_time = 0;
    HPLUS_stats.build_time = 0;
    HPLUS_stats.exec_time = 0;

}

void HPLUS_show_info() {

    #if HPLUS_VERBOSE <= 1
    return;
    #endif

    lprint(LINE);

    std::time_t time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    HPLUS_env.logger.print("%s%s", std::ctime(&time), LINE);
    HPLUS_env.logger.print("Input file: %s.", HPLUS_env.infile.c_str());
    if (HPLUS_env.log && !HPLUS_env.log_name.empty()) HPLUS_env.logger.print("Log name: %s.", HPLUS_env.log_name.c_str());
    if (!HPLUS_env.run_name.empty()) HPLUS_env.logger.print("Run name: %s.", HPLUS_env.run_name.c_str());

    lprint(LINE);

    #if HPLUS_VERBOSE >= 100
    HPLUS_env.logger.print("Fast Downward translator version: %d.", HPLUS_inst.get_version());
    #endif
    HPLUS_env.logger.print("Metric: %s.", (HPLUS_inst.unitary_cost() ? "unitary costs" : "integer costs"));
    HPLUS_env.logger.print("# variables (binary expansion): %d.", HPLUS_inst.get_nvar());
    #if HPLUS_VERBOSE >= 100
    HPLUS_env.logger.print("Goal state: %s.", std::string(HPLUS_inst.get_gstate()).c_str());
    #endif
    HPLUS_env.logger.print("# actions: %d.", HPLUS_inst.get_nact());

    lprint(LINE);

    #if HPLUS_INTCHECK
    lprint("Integrity checks enabled.");
    lprint(LINE);
    #endif

    HPLUS_env.logger.print("Algorithm: %s.", HPLUS_env.alg.c_str());
    HPLUS_env.logger.print("Model enhancements: %s.", HPLUS_env.model_enhancements ? "enabled" : "disabled");
    if (HPLUS_env.alg == HPLUS_CLI_IMAI) HPLUS_env.logger.print("Tighter bounds on variable timestamps: %s.", HPLUS_env.imai_var_bound ? "enabled" : "disabled");
    HPLUS_env.logger.print("Initial heuristic: %s.", HPLUS_env.heur_1 ? "enabled" : "disabled");
    HPLUS_env.logger.print("Optimized heuristic: %s.", HPLUS_env.heur_2 ? "enabled" : "disabled");
    HPLUS_env.logger.print("Warm start: %s.", HPLUS_env.warm_start ? "enabled" : "disabled");
    HPLUS_env.logger.print("Time limit: %ds.", HPLUS_env.time_limit);
    lprint(LINE);

}

void HPLUS_parse_cli(const int argc, const char** argv) {

    // setting defaults
    HPLUS_env.log = HPLUS_DEF_LOG_OPTION;
    HPLUS_env.log_name = HPLUS_LOG_DIR"/" + std::string(HPLUS_DEF_LOG_FILE);
    HPLUS_env.run_name = HPLUS_DEF_RUN_NAME;
    HPLUS_env.time_limit = HPLUS_DEF_TIME_LIMIT;

    std::vector<std::string> unknown_args;

    for (unsigned int i = 1; i < argc; i++) {

        if (std::string(argv[i]) == HPLUS_CLI_INPUT_FILE_FLAG) {
            HPLUS_env.infile = std::string(argv[++i]);      // path relative to where the program is launched
            struct stat buffer{};
            my::assert(stat((HPLUS_env.infile).c_str(), &buffer) == 0,  "Failed to open input file.");
        }

        // -------- LOGGING ------- //
        
        else if (std::string(argv[i]) == HPLUS_CLI_LOG_FLAG) HPLUS_env.log = true;
        else if (std::string(argv[i]) == HPLUS_CLI_LOG_NAME_FLAG) HPLUS_env.log_name = HPLUS_LOG_DIR"/" + std::string(argv[++i]);
        else if (std::string(argv[i]) == HPLUS_CLI_RUN_NAME_FLAG) HPLUS_env.run_name = argv[++i];

        // ------- EXECUTION ------ //

        else if (std::string(argv[i]) == HPLUS_CLI_TIMELIMIT_FLAG) { my::assert(my::isint(argv[i+1]), "The time limit must be an integer."); HPLUS_env.time_limit = atoi(argv[++i]); }
        else if (std::string(argv[i]) == HPLUS_CLI_ALG_FLAG) HPLUS_env.alg = argv[++i];
        else if (std::string(argv[i]) == HPLUS_CLI_OPT_ENHANCEMENTS) {
            my::assert(std::string(argv[i+1])=="0" || std::string(argv[i+1])=="1", "The enhancement option has to be a 0 or a 1.");
            HPLUS_env.model_enhancements = std::string(argv[++i]) == "1";
        }
        else if (std::string(argv[i]) == HPLUS_CLI_OPT_TIME_BOUND) {
            my::assert(std::string(argv[i+1])=="0" || std::string(argv[i+1])=="1", "The timestamps bound option has to be a 0 or a 1.");
            HPLUS_env.imai_var_bound = std::string(argv[++i]) == "1";
        }
        else if (std::string(argv[i]) == HPLUS_CLI_OPT_WARM_START) {
            my::assert(std::string(argv[i+1])=="0" || std::string(argv[i+1])=="1", "The warm start option has to be a 0 or a 1.");
            HPLUS_env.warm_start = std::string(argv[++i]) == "1";
        }
        else if (std::string(argv[i]) == HPLUS_CLI_OPT_HEUR1) {
            my::assert(std::string(argv[i+1])=="0" || std::string(argv[i+1])=="1", "The initial heuristic option has to be a 0 or a 1.");
            HPLUS_env.heur_1 = std::string(argv[++i]) == "1";
        }
        else if (std::string(argv[i]) == HPLUS_CLI_OPT_HEUR2) {
            my::assert(std::string(argv[i+1])=="0" || std::string(argv[i+1])=="1", "The optimized heuristic option has to be a 0 or a 1.");
            HPLUS_env.heur_2 = std::string(argv[++i]) == "1";
        }

        else unknown_args.push_back(argv[i]);

    }

    for (const auto & unknown_arg : unknown_args) HPLUS_env.logger.print_warn("Unknown command-line option '%s'.", unknown_arg.c_str());
    if (!unknown_args.empty()) lraise_error("Exiting due to unknown cli arguments.");

    my::assert(!HPLUS_env.infile.empty(), "No input file specified.");
    my::assert(!HPLUS_env.alg.empty(), "No algorithm specified.");

    HPLUS_env.logger = my::Logger(HPLUS_env.run_name, HPLUS_env.log_name);

    if (HPLUS_env.warm_start && !HPLUS_env.heur_1 && !HPLUS_env.heur_2) {
        lprint_warn("Warm start has been activated but heuristics have been disabled, disabling warm start.");
        HPLUS_env.warm_start = false;
    }

}

void HPLUS_end() {

    if (HPLUS_env.get_time() > HPLUS_env.time_limit) lprint("Reached time limit.");

    switch(HPLUS_env.status) {
        case my::status::FEAS:
            lprint("The solution is not optimal.");
            HPLUS_inst.print_best_sol();
            break;
        case my::status::INFEAS:
            lprint("The problem is infeasible.");
            break;
        case my::status::NOTFOUND:
            lprint("No solution found.");
            break;
        default:
            HPLUS_inst.print_best_sol();
            break;
    }

    HPLUS_stats.total_time = HPLUS_env.get_time();
    HPLUS_stats.print();

}

void signal_callback_handler(const int signum) {

    if (HPLUS_env.cplex_running) HPLUS_env.cpx_terminate = 1;
    else {
        HPLUS_end();
        exit(0);
    }

}

void time_limit_termination(double duration) {

    while (!HPLUS_env.cplex_running) {
        if (HPLUS_env.get_time() >= duration) {
            raise(SIGINT);
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

}

int main(const int argc, const char** argv) {

    signal(SIGINT, signal_callback_handler);
    HPLUS_start();
    HPLUS_parse_cli(argc, argv);
    std::chrono::seconds time_limit(HPLUS_env.time_limit);
    std::thread timer_thread(time_limit_termination, HPLUS_env.time_limit);
    HPLUS_inst = HPLUS_instance(HPLUS_env.infile);
    HPLUS_show_info();
    HPLUS_run();
    timer_thread.join();
    HPLUS_end();

    return 0;

}