#include "../include/hplus_instance.hpp"
#include "../include/algorithms.hpp"

void signal_callback_handler(const int signum) {

    HPLUS_env.logger.print("\n%s", LINE);
    HPLUS_env.logger.print(" >>  Caught ctrl+C signal, exiting...  <<");
    HPLUS_env.logger.print(LINE);

    HPLUS_env.cpx_terminate = 1;                        // signal cplex to stop

}

/**
 * Initializations
 */
void HPLUS_start() {

    HPLUS_env.start_timer();
    HPLUS_env.status = my::status::NOTFOUND;

}

/**
 * Visual info of the execution of the code and the parsed problem
 */
void HPLUS_show_info(const HPLUS_instance* inst) {

    HPLUS_env.logger.print(LINE);

    HPLUS_env.logger.print("Input file: %s.", HPLUS_env.infile.c_str());
    if (!HPLUS_env.log_name.empty()) HPLUS_env.logger.print("Log name: %s.", HPLUS_env.log_name.c_str());
    if (!HPLUS_env.run_name.empty()) HPLUS_env.logger.print("Run name: %s.", HPLUS_env.run_name.c_str());

    HPLUS_env.logger.print(LINE);

    #if HPLUS_VERBOSE >= 100
    HPLUS_env.logger.print("Version: %d.", inst -> get_version());
    #endif
    HPLUS_env.logger.print("Metric: %s.", (inst -> unitary_cost() ? "unitary costs" : "integer costs"));
    HPLUS_env.logger.print("# variables: %d.", inst -> get_nvar());
    #if HPLUS_VERBOSE >= 100
    const unsigned int nvar = inst -> get_nvar();
    const HPLUS_variable** variables = inst -> get_variables();
    for (int i = 0; i < nvar; i++) HPLUS_env.logger.print("name(var_%d) = '%s'.", i, variables[i] -> get_name() -> c_str());
    for (int i = 0; i < nvar; i++) HPLUS_env.logger.print("range(var_%d) = '%d'.", i, variables[i] -> get_range());
    for (int i = 0; i < nvar; i++){
        const std::string* val_names = variables[i] -> get_val_names();
        for (int j = 0; j < variables[i] -> get_range(); j++)
            HPLUS_env.logger.print("name(var_%d[%d]) = '%s'.", i, j, val_names[j].c_str());
    }
    HPLUS_env.logger.print("Bitfield size: %d", inst -> get_bfsize());
    HPLUS_env.logger.print("Initial state: %s.", inst -> get_istate() -> view().c_str());
    HPLUS_env.logger.print("Goal state: %s.", inst -> get_gstate() -> view().c_str());
    #endif
    HPLUS_env.logger.print("# actions: %d.", inst -> get_nact());
    #if HPLUS_VERBOSE >= 100
    const HPLUS_action** actions = inst -> get_actions();
    for (int act_i = 0; act_i < inst -> get_nact(); act_i++) {
        HPLUS_env.logger.print("pre(act_%d) = '%s'.", act_i, actions[act_i] -> get_pre() -> view().c_str());
        HPLUS_env.logger.print("eff(act_%d) = '%s'.", act_i, actions[act_i] -> get_eff() -> view().c_str());
        HPLUS_env.logger.print("name(act_%d) = '%s'.", act_i, actions[act_i] -> get_name() -> c_str());
        HPLUS_env.logger.print("cost(act_%d) = '%d'.", act_i, actions[act_i] -> get_cost());
    }
    #endif

    HPLUS_env.logger.print(LINE);

    if (HPLUS_INTCHECK) {
        HPLUS_env.logger.print_warn("Integrity checks enabled.");
        HPLUS_env.logger.print(LINE);
    }

}

/**
 * Parse command-line arguments
 */
void HPLUS_parse_cli(const int argc, const char** argv) {

    std::vector<std::string> unknown_args;

    for (int i = 1; i < argc; i++) {

        if (!strcmp(argv[i], HPLUS_CLI_INPUT_FILE_FLAG)) {
            HPLUS_env.infile = HPLUS_INST_DIR"/" + std::string(argv[++i]);
            struct stat buffer;
            my::assert(stat((HPLUS_env.infile).c_str(), &buffer) == 0,  "Failed to open input file.");
        }
        else if (!strcmp(argv[i], HPLUS_CLI_LOG_FLAG)) HPLUS_env.log = true;
        else if (!strcmp(argv[i], HPLUS_CLI_LOG_NAME_FLAG)) HPLUS_env.log_name = HPLUS_LOG_DIR"/" + std::string(argv[++i]);
        else if (!strcmp(argv[i], HPLUS_CLI_RUN_NAME_FLAG)) HPLUS_env.run_name = argv[++i];
        else if (!strcmp(argv[i], HPLUS_CLI_ALG)) HPLUS_env.alg = argv[++i];

        else unknown_args.push_back(argv[i]);

    }

    my::assert(!HPLUS_env.infile.empty(), "No input file specified.");
    if (HPLUS_env.log && HPLUS_env.log_name.empty()) { HPLUS_env.log_name = HPLUS_LOG_DIR"/" + std::string(HPLUS_DEF_LOG_FILE); }
    if (HPLUS_env.run_name.empty()) { HPLUS_env.run_name = "UNNAMED RUN"; }

    HPLUS_env.logger = my::Logger(HPLUS_env.run_name, HPLUS_env.log_name);

    my::assert(!HPLUS_env.alg.empty(), "No algorithm specified.");

    for (int i = 0; i < unknown_args.size(); i++) HPLUS_env.logger.print_warn("Unknown command-line option '%s'.", unknown_args[i].c_str());

}

/**
 * Run the algorithm specified via cli
 */
void HPLUS_run(HPLUS_instance* inst) {

    double start_time = HPLUS_env.get_time();

    if (HPLUS_env.alg == "imai") HPLUS_run_imai(inst);

    else HPLUS_env.logger.raise_error("The specified algorithm %s is not recognised. Please refer to the README.md for instructions.", HPLUS_env.alg.c_str());

    HPLUS_stats.exec_time = HPLUS_env.get_time() - start_time;

    switch(HPLUS_env.status) {
        case my::status::INFEAS:
            HPLUS_env.logger.print("The problem is infeasible.");
            return;
        case my::status::NOTFOUND:
            HPLUS_env.logger.print("No solution found.");
            return;
        case my::status::TIMEL_NF:
            HPLUS_env.logger.print("No solution found due to time limit.");
            return;
        case my::status::USR_STOP_NF:
            HPLUS_env.logger.print("No solution found due to the user terminating the execution.");
            return;
        case my::status::TIMEL_FEAS:
            HPLUS_env.logger.print("The solution is not optimal due to time limit.");
            break;
        case my::status::USR_STOP_FEAS:
            HPLUS_env.logger.print("The solution is not optimal due to the user terminating the execution.");
            break;
        default:
            break;
    }

    const HPLUS_action** actions = inst -> get_actions();
    std::vector<unsigned int> solution;
    unsigned int cost;
    inst -> get_best_solution(&solution, &cost);

    HPLUS_env.logger.print("Cost of the solution: %d.", cost);
    for(auto idx : solution) HPLUS_env.logger.print("(%s)", actions[idx] -> get_name() -> c_str());

}

/**
 * Program termination hook
 */
void HPLUS_end() {

    #if HPLUS_VERBOSE >= 1
    HPLUS_stats.print();
    #endif

}

int main(const int argc, const char** argv) {

    signal(SIGINT, signal_callback_handler);
    HPLUS_start();

    HPLUS_parse_cli(argc, argv);

    HPLUS_instance* inst = new HPLUS_instance(HPLUS_env.infile);

    #if HPLUS_VERBOSE >= 1
    HPLUS_show_info(inst);
    #endif

    HPLUS_run(inst);

    MYDEL(inst);

    HPLUS_end();

    return HPLUS_env.status;

}