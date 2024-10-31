#include <csignal>
#include <thread>
#include <sys/stat.h>
#include "../include/algorithms.hpp"

void HPLUS_start() {

    HPLUS_env.start_timer();
    HPLUS_env.status = my::status::NOTFOUND;
    HPLUS_env.cpx_terminate = 0;
    HPLUS_env.tl_terminate = 0;
    HPLUS_env.build_finished = 0;

}

void HPLUS_show_info(const HPLUS_instance& inst) {

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
    HPLUS_env.logger.print("Fast Downward translator version: %d.", inst.get_version());
    #endif
    HPLUS_env.logger.print("Metric: %s.", (inst.unitary_cost() ? "unitary costs" : "integer costs"));
    HPLUS_env.logger.print("# variables (binary expansion): %d.", inst.get_nvar());
    #if HPLUS_VERBOSE >= 100
    HPLUS_env.logger.print("Goal state: %s.", std::string(inst.get_gstate()).c_str());
    #endif
    HPLUS_env.logger.print("# actions: %d.", inst.get_nact());

    lprint(LINE);

    #if HPLUS_INTCHECK
    lprint("Integrity checks enabled.");
    lprint(LINE);
    #endif

    HPLUS_env.logger.print("Algorithm: %s.", HPLUS_env.alg.c_str());
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

        if (!strcmp(argv[i], HPLUS_CLI_INPUT_FILE_FLAG)) {
            HPLUS_env.infile = std::string(argv[++i]);      // path relative to where the program is launched
            struct stat buffer{};
            my::assert(stat((HPLUS_env.infile).c_str(), &buffer) == 0,  "Failed to open input file.");
        }
        // -------- LOGGING ------- //
        else if (!strcmp(argv[i], HPLUS_CLI_LOG_FLAG)) HPLUS_env.log = true;
        else if (!strcmp(argv[i], HPLUS_CLI_LOG_NAME_FLAG)) HPLUS_env.log_name = HPLUS_LOG_DIR"/" + std::string(argv[++i]);
        else if (!strcmp(argv[i], HPLUS_CLI_RUN_NAME_FLAG)) HPLUS_env.run_name = argv[++i];
        // ------- EXECUTION ------ //
        else if (!strcmp(argv[i], HPLUS_CLI_TIMELIMIT_FLAG)) { my::assert(my::isint(argv[i+1]), "The time limit must be an integer."); HPLUS_env.time_limit = atoi(argv[++i]); }
        else if (!strcmp(argv[i], HPLUS_CLI_ALG_FLAG)) HPLUS_env.alg = argv[++i];

        else unknown_args.push_back(argv[i]);

    }

    for (const auto & unknown_arg : unknown_args) HPLUS_env.logger.print_warn("Unknown command-line option '%s'.", unknown_arg.c_str());
    if (!unknown_args.empty()) lraise_error("Exiting due to unknown cli arguments.");

    my::assert(!HPLUS_env.infile.empty(), "No input file specified.");
    my::assert(!HPLUS_env.alg.empty(), "No algorithm specified.");

    HPLUS_env.logger = my::Logger(HPLUS_env.run_name, HPLUS_env.log_name);

}

void HPLUS_end() {

    HPLUS_stats.print();

}

void signal_callback_handler(const int signum) {

    if (HPLUS_env.tl_terminate) {

        HPLUS_env.logger.print("\n%s", LINE);
        lprint(" >> Reached time limit, terminating... <<");
        lprint(LINE);
        if (!HPLUS_env.build_finished) {
            lprint("Time limit reached while building the model.");
            HPLUS_end();
            exit(0);
        }

    } else {

        if (!HPLUS_env.build_finished) {
            HPLUS_env.logger.print("\n%s", LINE);
            lprint(" >>          Forcing exit...           <<");
            lprint(LINE);
            exit(0);
        } else {
            HPLUS_env.logger.print("\n%s", LINE);
            lprint(" >>  Caught ctrl+C signal, exiting...  <<");
            lprint(LINE);
            HPLUS_env.cpx_terminate = 1;
        }

    }

}

void time_limit_termination(double duration) {

    while (HPLUS_env.status == my::status::NOTFOUND) {
        if (HPLUS_env.get_time() >= duration) {
            HPLUS_env.tl_terminate = 1;
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
    HPLUS_instance inst = HPLUS_instance(HPLUS_env.infile);
    HPLUS_show_info(inst);
    HPLUS_run(inst);
    timer_thread.join();
    HPLUS_end();

    return 0;

}