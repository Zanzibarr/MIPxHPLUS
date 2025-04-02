/**
 * @file main.cpp
 * @brief Main program for the hplus master thesis project
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#include <sys/stat.h>  // For stat buffer {}

#include <cstring>  // For strncmp

#include "hplus_instance.hpp"

volatile int global_terminate{0};

static void init(hplus::environment& env) {
    env = hplus::environment{.exec_s = exec_status::START,
                             .sol_s = solution_status::NOTFOUND,
                             .input_file = "N/A",
                             .log_name = HPLUS_LOG_DIR "/hplus_log.log",
                             .run_name = "UnnamedRun",
                             .alg = HPLUS_CLI_ALG_RANKOOH,
                             .heur = "hadd",
                             .preprocessing = true,
                             .warm_start = true,
                             .tight_bounds = false,
                             .using_cplex = true,
                             .log = false,
                             .write_lp = false,
                             .time_limit = 60,
                             .timer = time_keeper()};
}

static void init(hplus::statistics& stats) {
    stats = hplus::statistics{.parsing = 0,
                              .preprocessing = 0,
                              .heuristic = 0,
                              .build = 0,
                              .callback = 0,
                              .execution = 0,
                              .total = 0,
                              .callback_time_mutex = PTHREAD_MUTEX_INITIALIZER,
                              .hcost = -1,
                              .fcost = -1,
                              .nnodes = -1,
                              .status = -1,
                              .nvar_base = -1,
                              .nvar_acyclic = -1,
                              .nconst_base = -1,
                              .nconst_acyclic = -1,
                              .nusercuts = -1,
                              .lb = -1};
}

static void info_parse_cli(const int& argc, const char** argv, hplus::environment& env) {
    if (argc != 2 || !strncmp(argv[1], "--h", 3) || !strncmp(argv[1], "--help", 6)) {
        std::cerr << "Usage: " << argv[0] << " <input_file/\"input file\">" << std::endl;
        exit(1);
    }
    env.input_file = std::string(argv[1]);
}

int main(const int argc, const char** argv) {
    hplus::instance inst;
    hplus::environment env;
    init(env);
    hplus::statistics stats;
    init(stats);
    info_parse_cli(argc, argv, env);
    logger log(env.log, HPLUS_LOG_DIR "/" + env.log_name, env.run_name);
    if (hplus::create_instance(inst, env, stats, log, true)) {
        log.print(LINE);
        std::time_t time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        log.print("%sCode version: %s.\n%s", std::ctime(&time), CODE_VERSION, LINE);
        log.print("Input file: %47s.", env.input_file.c_str());
        log.print(LINE);
        log.print("Metric:                                %20s.",
                  (inst.equal_costs ? (inst.actions[0].cost == 1 ? "unitary costs" : "constant costs") : "integer costs"));
        log.print("# facts:                                         %10d.", inst.n);
        log.print("# actions:                                       %10d.", inst.m);
        log.print(LINE);
    }
    return 0;
}