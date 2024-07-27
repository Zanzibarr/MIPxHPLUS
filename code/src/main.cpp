#include "../include/hplus_instance.hpp"

/**
 * Initializations
 */
void start() {

    my::start_timer();
    HPLUS_env.status = 0;

}

/**
 * Visual info of the execution of the code and the parsed problem
 */
void show_info(const HPLUS_instance* inst) {

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
void parse_cli(const int argc, const char** argv) {

    std::vector<std::string> unknown_args;

    for (int i = 1; i < argc; i++) {

        if (!strcmp(argv[i], CLI_INPUT_FILE_FLAG)) {
            HPLUS_env.infile = HPLUS_INST_DIR"/" + std::string(argv[i+1]);
            i++;
            struct stat buffer;
            my::assert(stat((HPLUS_env.infile).c_str(), &buffer) == 0,  "Failed to open input file.");
        }
        else if (!strcmp(argv[i], CLI_LOG_FLAG)) {
            HPLUS_env.log = true;
        }
        else if (!strcmp(argv[i], CLI_LOG_NAME_FLAG)) {
            HPLUS_env.log_name = HPLUS_LOG_DIR"/" + std::string(argv[i+1]);
            i++;
        }
        else if (!strcmp(argv[i], CLI_RUN_NAME_FLAG)) {
            HPLUS_env.run_name = argv[i+1];
            i++;
        }

        else unknown_args.push_back(argv[i]);

    }

    my::assert(!HPLUS_env.infile.empty(), "No input file specified.");
    if (HPLUS_env.log && HPLUS_env.log_name.empty()) { HPLUS_env.log_name = HPLUS_LOG_DIR"/" + std::string(DEF_LOG_FILE); }
    if (HPLUS_env.run_name.empty()) { HPLUS_env.run_name = "UNNAMED RUN"; }

    HPLUS_env.logger = Logger(HPLUS_env.run_name, HPLUS_env.log_name);

    for (int i = 0; i < unknown_args.size(); i++) HPLUS_env.logger.print_warn("Unknown command-line option '%s'.", unknown_args[i].c_str());

}

/**
 * Program termination hook
 */
void end() {

    #if HPLUS_VERBOSE >= 1
    HPLUS_stats.print();
    #endif

}

int main(const int argc, const char** argv) {

    start();

    parse_cli(argc, argv);

    HPLUS_instance* inst = new HPLUS_instance(HPLUS_env.infile);

    #if HPLUS_VERBOSE >= 1
    show_info(inst);
    #endif

    HPLUS_env.logger.print_warn("Only the parser has been implemented.");

    DEL(inst);

    end();

    return HPLUS_env.status;

}