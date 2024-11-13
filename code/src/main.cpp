#include <csignal>
#include <thread>
#include <sys/stat.h>
#include "../include/algorithms.hpp"

pthread_mutex_t exit_mutex;

void HPLUS_start() {

    timer.start_timer();

    HPLUS_env.exec_status = my::execution_status::STARTING;
    HPLUS_env.sol_status = my::solution_status::NOTFOUND;
    HPLUS_env.cpx_terminate = 0;
    HPLUS_env.input_file = HPLUS_DEF_INPUT_FILE;
    HPLUS_env.log = HPLUS_DEF_LOG_ENABLED;
    HPLUS_env.log_name = HPLUS_LOG_DIR"/" + std::string(HPLUS_DEF_LOG_FILE);
    HPLUS_env.run_name = HPLUS_DEF_RUN_NAME;
    HPLUS_env.alg = HPLUS_DEF_ALG;
    HPLUS_env.problem_simplification_enabled = HPLUS_DEF_PROB_SIMPL_ENABLED;
    HPLUS_env.imai_tighter_var_bound_enabled = HPLUS_DEF_IMAI_VAR_BOUND_ENABLED;
    HPLUS_env.heuristic_enabled = HPLUS_DEF_HEURISTIC_ENABLED;
    HPLUS_env.warm_start_enabled = HPLUS_DEF_WARM_START_ENABLED;
    HPLUS_env.time_limit = HPLUS_DEF_TIME_LIMIT;

    HPLUS_stats.parsing_time = 0;
    HPLUS_stats.simplification_time = 0;
    HPLUS_stats.heuristic_time = 0;
    HPLUS_stats.build_time = 0;
    HPLUS_stats.execution_time = 0;
    HPLUS_stats.total_time = 0;

}

void HPLUS_show_info() {

    #if HPLUS_VERBOSE <= 1
    return;
    #endif

    mylog.print(LINE);

    std::time_t time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    mylog.print("%s%s", std::ctime(&time), LINE);
    mylog.print("Input file: %s.", HPLUS_env.input_file.c_str());
    if (HPLUS_env.log && !HPLUS_env.log_name.empty()) mylog.print("Log name: %s.", HPLUS_env.log_name.c_str());
    if (!HPLUS_env.run_name.empty()) mylog.print("Run name: %s.", HPLUS_env.run_name.c_str());

    mylog.print(LINE);

    #if HPLUS_VERBOSE >= 100
    mylog.print("Fast Downward translator version: %d.", HPLUS_inst.get_version());
    #endif
    mylog.print("Metric: %s.", (HPLUS_inst.unitary_cost() ? "unitary costs" : "integer costs"));
    mylog.print("# variables (binary expansion): %d.", HPLUS_inst.get_n_var());
    mylog.print("# actions: %d.", HPLUS_inst.get_n_act());
    #if HPLUS_VERBOSE >= 100
    mylog.print("Goal state: %s.", std::string(HPLUS_inst.get_goal_state()).c_str());
    #endif

    mylog.print(LINE);

    #if HPLUS_INTCHECK
    mylog.print("Integrity checks enabled.");
    mylog.print(LINE);
    #endif

    mylog.print("Algorithm: %s.", HPLUS_env.alg.c_str());
    mylog.print("Problem simplification: %s.", HPLUS_env.problem_simplification_enabled ? "enabled" : "disabled");
    if (HPLUS_env.alg == HPLUS_CLI_ALG_IMAI) mylog.print("Tighter bounds on variable timestamps: %s.", HPLUS_env.imai_tighter_var_bound_enabled ? "enabled" : "disabled");
    mylog.print("Heuristic: %s.", HPLUS_env.heuristic_enabled ? "enabled" : "disabled");
    mylog.print("Warm start: %s.", HPLUS_env.warm_start_enabled ? "enabled" : "disabled");
    mylog.print("Time limit: %ds.", HPLUS_env.time_limit);
    mylog.print(LINE);

}

void HPLUS_parse_cli(const int argc, const char** argv) {

    HPLUS_env.exec_status = my::execution_status::PARSING;

    std::vector<std::string> unknown_args;

    for (size_t i = 1; i < argc; i++) {

        if (std::string(argv[i]) == HPLUS_CLI_INPUT) {
            HPLUS_env.input_file = std::string(argv[++i]);      // path relative to where the program is launched
            struct stat buffer{};
            my::assert(stat((HPLUS_env.input_file).c_str(), &buffer) == 0,  "Failed to open input file.");
        }

        // -------- LOGGING ------- //
        
        else if (std::string(argv[i]) == HPLUS_CLI_LOG) HPLUS_env.log = true;
        else if (std::string(argv[i]) == HPLUS_CLI_LOG_NAME) HPLUS_env.log_name = HPLUS_LOG_DIR"/" + std::string(argv[++i]);
        else if (std::string(argv[i]) == HPLUS_CLI_RUN_NAME) HPLUS_env.run_name = argv[++i];

        // ------- EXECUTION ------ //

        else if (std::string(argv[i]) == HPLUS_CLI_ALG) HPLUS_env.alg = argv[++i];
        else if (std::string(argv[i]) == HPLUS_CLI_OPT_NO_SIMPL) HPLUS_env.problem_simplification_enabled = false;
        else if (std::string(argv[i]) == HPLUS_CLI_OPT_IMAI_NO_TB) HPLUS_env.imai_tighter_var_bound_enabled = false;
        else if (std::string(argv[i]) == HPLUS_CLI_OPT_NO_HEUR) HPLUS_env.heuristic_enabled = false;
        else if (std::string(argv[i]) == HPLUS_CLI_OPT_NO_WARM_START) HPLUS_env.warm_start_enabled = false;
        else if (std::string(argv[i]) == HPLUS_CLI_TIME_LIMIT) { my::assert(my::isint(argv[i+1]), "The time limit must be an integer."); HPLUS_env.time_limit = atoi(argv[++i]); }

        else unknown_args.push_back(argv[i]);

    }

    for (const auto & unknown_arg : unknown_args) mylog.print_warn("Unknown command-line option '%s'.", unknown_arg.c_str());
    if (!unknown_args.empty()) mylog.raise_error("Exiting due to unknown cli arguments.");

    my::assert(HPLUS_env.input_file != "N/A", "No input file specified.");
    my::assert(!HPLUS_env.alg.empty(), "No algorithm specified.");

    mylog = my::logger(HPLUS_env.run_name, HPLUS_env.log, HPLUS_env.log_name);

    if (HPLUS_env.warm_start_enabled && !HPLUS_env.heuristic_enabled) {
        mylog.print_warn("Warm start has been activated but heuristics have been disabled: disabling warm start.");
        HPLUS_env.warm_start_enabled = false;
    }

}

void HPLUS_end() {

    pthread_mutex_lock(&exit_mutex);

    if (timer.get_time() >= HPLUS_env.time_limit) {
        switch(HPLUS_env.exec_status) {
            case my::execution_status::STARTING:
                mylog.print("Reached time limit before the program could read the instance file.");
                break;
            case my::execution_status::PARSING:
                mylog.print("Reached time limit while parsing the instance file.");
                break;
            case my::execution_status::PROBLEM_SIMPL:
                mylog.print("Reached time limit while simplificating the problem.");
                break;
            case my::execution_status::HEURISTIC:
                mylog.print("Reached time limit while calculating an heuristic solution.");
                break;
            case my::execution_status::MODEL_BUILD:
                mylog.print("Reached time limit while building the model.");
                break;
            case my::execution_status::CPX_EXECUTION:
                mylog.print("Reached time limit during CPLEX's execution.");
                break;
            default: break;
        }
    }

    HPLUS_env.exec_status = my::execution_status::EXITING;

    switch(HPLUS_env.sol_status) {
        case my::solution_status::FEAS:
            mylog.print("The solution has not been proven optimal.");
            HPLUS_inst.print_best_sol();
            break;
        case my::solution_status::INFEAS:
            mylog.print("The problem is infeasible.");
            break;
        case my::solution_status::NOTFOUND:
            mylog.print("No solution found.");
            break;
        default:
            HPLUS_inst.print_best_sol();
            break;
    }

    HPLUS_stats.total_time = timer.get_time();
    HPLUS_stats.print();

    exit(0);

}

void signal_callback_handler(const int signum) {

    if (HPLUS_env.exec_status == my::execution_status::CPX_EXECUTION) HPLUS_env.cpx_terminate = 1;
    else if (HPLUS_env.exec_status < my::execution_status::CPX_EXECUTION) HPLUS_end();

}

void time_limit_termination(double duration) {

    while (HPLUS_env.exec_status < my::execution_status::CPX_EXECUTION && HPLUS_env.sol_status != my::solution_status::INFEAS) {
        if (timer.get_time() > duration) {
            raise(SIGINT);
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

}

int main(const int argc, const char** argv) {

    signal(SIGINT, signal_callback_handler);
    pthread_mutex_init(&exit_mutex, nullptr);
    HPLUS_start();
    HPLUS_parse_cli(argc, argv);
    std::chrono::seconds time_limit(HPLUS_env.time_limit);
    std::thread timer_thread(time_limit_termination, HPLUS_env.time_limit);
    HPLUS_inst = HPLUS_instance(HPLUS_env.input_file);
    HPLUS_show_info();
    HPLUS_run();
    timer_thread.join();
    HPLUS_end();

}