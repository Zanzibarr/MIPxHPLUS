/**
 * @file main.cpp
 * @brief Main program for the hplus master thesis project
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#include <sys/stat.h>  // For stat buffer {}

#define HPLUS_INFO 1

#include "hplus_instance.hpp"
#include "main.cpp"

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
    if (hplus::create_instance(inst, env, stats, log)) {
        log.print(LINE);
        std::time_t time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        log.print("%sCode version: %s.\n%s", std::ctime(&time), CODE_VERSION, LINE);
        log.print("Input file: %47s.", env.input_file.c_str());
        log.print(LINE);
        log.print("Metric:                                %20s.",
                  (inst.equal_costs ? (inst.actions[0].cost == 1 ? "unitary costs" : "constant costs") : "integer costs"));
        log.print("# variables:                                     %10d.", inst.n);
        log.print("# actions:                                       %10d.", inst.m);
        log.print(LINE);
    }
    return 0;
}