#ifndef HPLUS_ARGPARSER_HPP
#define HPLUS_ARGPARSER_HPP

#include <stdlib.h>
#include <sys/stat.h>  // For stat buffer {}

#include <csignal>

#include "../domain/execution.hpp"
#include "../external/args.hxx"

static void parse_cli(const int argc, const char** argv, hplus::execution& exec) {
    args::ArgumentParser parser("Find a solution / the optimal solution to the deletefree relaxation of a SAS+ planning task",
                                "Copyright 2025 Matteo Zanella, Domenico Salvagnin");
    args::HelpFlag help(parser, "help", "Display the help menu", {HPLUS_CLI_HELP_FLAG});
    args::Positional<std::string> input_file(parser, "input file", "Specify the input file (a .sas file provided by the FastDownward translator)");
    args::Flag info(parser, "info flag", "Show info about an instance", {HPLUS_CLI_INFO_FLAG});
    args::Flag run(parser, "run flag", "Run the specified algorithm", {HPLUS_CLI_RUN_FLAG});
    args::ValueFlag<std::string> algorithm(parser, "string",
                                           "Specify the algorithm to run (default: " + std::string(HPLUS_DEF_ALG_STRING) + ", options: [" +
                                               std::string(HPLUS_CLI_ALG_FLAG_TL) + "," + std::string(HPLUS_CLI_ALG_FLAG_VE) + "," +
                                               std::string(HPLUS_CLI_ALG_FLAG_CUTS) + "," + std::string(HPLUS_CLI_ALG_FLAG_GREEDYCOST) + "," +
                                               std::string(HPLUS_CLI_ALG_FLAG_GREEDYCXE) + "," + std::string(HPLUS_CLI_ALG_FLAG_GREEDYHMAX) + "," +
                                               std::string(HPLUS_CLI_ALG_FLAG_GREEDYHADD) + "])",
                                           {HPLUS_CLI_ALG_FLAG}, HPLUS_DEF_ALG_STRING);
    args::ValueFlag<std::string> warm_start(parser, "string",
                                            "Select an option for warm start (default: " + std::string(HPLUS_DEF_WS_STRING) + ", options: [" +
                                                std::string(HPLUS_CLI_WS_FLAG_NONE) + "," + std::string(HPLUS_CLI_WS_FLAG_GREEDYCOST) + "," +
                                                std::string(HPLUS_CLI_WS_FLAG_GREEDYCXE) + "," + std::string(HPLUS_CLI_WS_FLAG_GREEDYHMAX) + "," +
                                                std::string(HPLUS_CLI_WS_FLAG_GREEDYHADD) + "])",
                                            {HPLUS_CLI_WS_FLAG}, HPLUS_DEF_WS_STRING);
    args::ValueFlag<bool> prep(parser, "0/1",
                               "Perform preprocessing on the instance (def: " + std::to_string(HPLUS_DEF_PREP) + "; options: 0 (false), 1 (true))",
                               {HPLUS_CLI_PREP_FLAG}, HPLUS_DEF_PREP);
    args::ValueFlag<std::string> log(
        parser, "string",
        "Write on stdout / file (def: " + std::string(HPLUS_DEF_LOG) + "; options: 0 (stdout), <file_path> (absolute path to the file to write to))",
        {HPLUS_CLI_LOG_FLAG}, HPLUS_DEF_LOG);
    args::ValueFlag<unsigned int> time_limit(parser, "non-negative int",
                                             "Set the time limit (def: " + std::to_string(HPLUS_DEF_TIMELIMIT) +
                                                 " s; options: 0 (no timelimit), <positive int> (setting that as timelimit (seconds)))",
                                             {HPLUS_CLI_TIMELIMIT_FLAG}, HPLUS_DEF_TIMELIMIT);
    args::ValueFlag<unsigned int> threads(parser, "positive int",
                                          "Set the number of threads to use (def: " + std::to_string(HPLUS_DEF_THREADS) +
                                              " threads; options: <positive int (!= 0)> (setting that as number of threads to be used))",
                                          {HPLUS_CLI_THREADS_FLAG}, HPLUS_DEF_THREADS);
    args::ValueFlag<unsigned int> memory_limit(parser, "positive int",
                                               "Set the memory limit (def: " + std::to_string(HPLUS_DEF_MEMORYLIMIT) +
                                                   " MB; options: <positive int (!= 0)> (setting that as memory limit (MB)))",
                                               {HPLUS_CLI_MEMORYLIMIT_FLAG}, HPLUS_DEF_MEMORYLIMIT);
    args::ValueFlag<unsigned int> verbosity(
        parser, "non-negative int",
        "Set the verbosity (def: " + std::to_string(HPLUS_DEF_VERBOSE) +
            "; options: 0 (only final solution), 1 (statistics), 2 (execution log), 3 (debugging + integrity checks))",
        {HPLUS_CLI_VERBOSE_FLAG}, HPLUS_DEF_VERBOSE);
    args::ValueFlag<std::string> fract_cuts(
        parser, "string",
        "(ONLY FOR " + std::string(HPLUS_CLI_ALG_FLAG_CUTS) + " ALGORITHM) Specify what cuts to separate from the fractional solutions (def: " +
            std::string(HPLUS_DEF_FRACTCUTS) + "; options: 0 (don's separate cuts), a combination of ['l','s'] (respectively for landmarks and SEC))",
        {HPLUS_CLI_FRACTCUTS_FLAG}, HPLUS_DEF_FRACTCUTS);
    args::ValueFlag<std::string> cand_cuts(
        parser, "string",
        "(ONLY FOR " + std::string(HPLUS_CLI_ALG_FLAG_CUTS) +
            " ALGORITHM) Specify what cuts to separate from the candidate (integer) solutions (def: " + std::string(HPLUS_DEF_CANDCUTS) +
            "; options: 0 (don's separate cuts), a combination of ['f','c','s'] (respectively for frontier / complementary landmarks and SEC))",
        {HPLUS_CLI_CANDCUTS_FLAG}, HPLUS_DEF_CANDCUTS);
    args::ValueFlag<bool> custom_cutloop(
        parser, "bool", "Flag for using the custom cutloop (def: 0; options: 0 (don't use the custom cutloop), 1 (use the custom cutloop))",
        {HPLUS_CLI_CUTLOOP_FLAG}, HPLUS_DEF_CUSTOM_CUTLOOP);
    args::ValueFlag<bool> testing(parser, "bool",
                                  "Flag for testing or debugging (def: 0; options: 0 (testing flag set to false), 1 (testing flag set to true))",
                                  {HPLUS_CLI_TESTING_FLAG}, false);

    try {
        parser.ParseCLI(argc, argv);
    } catch (const args::Help&) {
        std::cout << parser;
        exit(EXIT_SUCCESS);
    } catch (const args::ParseError& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        exit(1);
    }

    // First info we need -> to setup the logger
    if (verbosity) {
        unsigned int v{args::get(verbosity)};
        if (v > 3) {
            std::cerr << "Incorrect verbose parameter";
            _Exit(EXIT_FAILURE);
        }
        exec.verbosity = static_cast<hplus::verbose>(v);
    }
    if (threads) {
        unsigned int t{args::get(threads)};
        if (t == 0) {
            std::cerr << "Please specify a positive (!= 0) number of threads.\n";
            _Exit(EXIT_FAILURE);
        }
        exec.threads = t;
    }

    if (log) {
        exec.log_file = HPLUS_LOG_DIR "/" + args::get(log);
        if (exec.log_file == "0")
            logger::get_instance().initialize(false, "", true, (exec.threads > 1 && DEBUG_VERBOSE()));
        else {
            logger::get_instance().initialize(true, exec.log_file, true, (exec.threads > 1 && DEBUG_VERBOSE()));
        }
    } else
        logger::get_instance().initialize(false, "", true, (exec.threads > 1 && DEBUG_VERBOSE()));

    if (BASIC_VERBOSE()) {
        LOG_INFO << today();
        LOG_INFO << version();
    }

    // Setup limits
    if (time_limit) {
        unsigned int t{args::get(time_limit)};
        exec.timelimit = t;
        if (t > 0) {
            timelim::set_time_limit(t);
        }
    }
    if (memory_limit) {
        unsigned int m{args::get(memory_limit)};
        ASSERT(m != 0);
        exec.memorylimit = m;
        if (!memlim::set_memory_limit(m)) LOG_ERROR << "An error occurred while setting up memory limits";
    }

    // Get parsed info
    if (input_file) {
        exec.file = args::get(input_file);
        exec.file_name = std::filesystem::path(exec.file).filename().string();
    } else
        LOG_ERROR << "Missing input file";
    struct stat buffer{};
    if (stat((exec.file).c_str(), &buffer) != 0) {
        LOG_ERROR << "Failed to open input file";
    }
    if (run)
        exec.type = hplus::exec_type::RUN;
    else if (info)
        exec.type = hplus::exec_type::INFO;
    if (algorithm) {
        std::string a{args::get(algorithm)};
        if (a == HPLUS_CLI_ALG_FLAG_TL)
            exec.alg = hplus::algorithm::TL;
        else if (a == HPLUS_CLI_ALG_FLAG_VE)
            exec.alg = hplus::algorithm::VE;
        else if (a == HPLUS_CLI_ALG_FLAG_CUTS)
            exec.alg = hplus::algorithm::CUTS;
        else if (a == HPLUS_CLI_ALG_FLAG_GREEDYCOST)
            exec.alg = hplus::algorithm::GC;
        else if (a == HPLUS_CLI_ALG_FLAG_GREEDYCXE)
            exec.alg = hplus::algorithm::GCXE;
        else if (a == HPLUS_CLI_ALG_FLAG_GREEDYHMAX)
            exec.alg = hplus::algorithm::GHM;
        else if (a == HPLUS_CLI_ALG_FLAG_GREEDYHADD)
            exec.alg = hplus::algorithm::GHA;
        else
            LOG_ERROR << "Algorithm '" << a << "' is not in the list of possible algorithms";
    }
    if (warm_start) {
        std::string ws{args::get(warm_start)};
        if (ws == HPLUS_CLI_WS_FLAG_NONE)
            exec.ws = hplus::warmstart::NONE;
        else if (ws == HPLUS_CLI_WS_FLAG_GREEDYCXE)
            exec.ws = hplus::warmstart::GCXE;
        else if (ws == HPLUS_CLI_WS_FLAG_GREEDYCOST)
            exec.ws = hplus::warmstart::GC;
        else if (ws == HPLUS_CLI_WS_FLAG_GREEDYHMAX)
            exec.ws = hplus::warmstart::GHM;
        else if (ws == HPLUS_CLI_WS_FLAG_GREEDYHADD)
            exec.ws = hplus::warmstart::GHA;
        else
            LOG_ERROR << "Warm start '" << ws << "' is not in the list of possible warm starts";
    }
    if (prep) exec.prep = args::get(prep);
    if (fract_cuts) {
        exec.fract_cuts = "";
        std::string s{args::get(fract_cuts)};
        if (s != "0") {
            if (s.find('l') != std::string::npos) exec.fract_cuts += "l";
            if (s.find('s') != std::string::npos) exec.fract_cuts += "s";
        }
    }
    if (cand_cuts) {
        exec.cand_cuts = "";
        std::string s{args::get(cand_cuts)};
        if (s.find('f') != std::string::npos) exec.cand_cuts += "f";
        if (s.find('c') != std::string::npos) exec.cand_cuts += "c";
        if (s.find('s') != std::string::npos) exec.cand_cuts += "s";
    }
    if (custom_cutloop) exec.custom_cutloop = args::get(custom_cutloop);
    if (testing) {
        exec.testing = args::get(testing);
        if (exec.testing) LOG_DEBUG << "Testing flag set to true";
    }

    // Check that it's all as it's supposed to be
    if (exec.threads > static_cast<unsigned int>(std::thread::hardware_concurrency())) {
        exec.threads = static_cast<unsigned int>(std::thread::hardware_concurrency());
        if (BASIC_VERBOSE()) LOG_WARNING << "This machine has " << exec.threads << " cores: using up to " << exec.threads << " threads";
    }
    if (info && run) LOG_ERROR << "You need to specify only one functionality among " << HPLUS_CLI_INFO_FLAG << " and " << HPLUS_CLI_RUN_FLAG;
    if (exec.alg >= hplus::algorithm::GC && exec.ws != hplus::warmstart::NONE) {
        LOG_WARNING << "With the specified algororithm there's no need for a warm start: disabling warm start option";
        exec.ws = hplus::warmstart::NONE;
    }
    if (exec.alg >= hplus::algorithm::GC) {
        switch (exec.alg) {
            case hplus::algorithm::GC:
                exec.ws = hplus::warmstart::GC;
                break;
            case hplus::algorithm::GCXE:
                exec.ws = hplus::warmstart::GCXE;
                break;
            case hplus::algorithm::GHM:
                exec.ws = hplus::warmstart::GHM;
                break;
            case hplus::algorithm::GHA:
                exec.ws = hplus::warmstart::GHA;
                break;
            default:
                LOG_ERROR << "Unhandled algorithm in parse_cli: " << static_cast<int>(exec.alg);
        }
    }
    if (exec.alg == hplus::algorithm::CUTS && exec.cand_cuts.empty())
        LOG_ERROR << "You can't disable all three candidate cuts from the " << HPLUS_CLI_ALG_FLAG_CUTS << " algorithm";
    if (exec.alg != hplus::algorithm::CUTS && !exec.fract_cuts.empty()) {
        LOG_WARNING << "Cuts on the fractional solutions aren't needed with this algorithm: disabling fractional cuts";
        exec.fract_cuts = "";
    }
    if (exec.alg != hplus::algorithm::CUTS && !exec.cand_cuts.empty()) {
        LOG_WARNING << "Cuts on the candidate solutions aren't needed with this algorithm: disabling candidate cuts";
        exec.cand_cuts = "";
    }
    if (exec.alg != hplus::algorithm::CUTS && exec.custom_cutloop) {
        LOG_WARNING << "Custom cutloop isn't needed with this algorithm: disabling custom cutloop";
        exec.custom_cutloop = false;
    }
    if (exec.ws == hplus::warmstart::NONE && exec.custom_cutloop) {
        LOG_WARNING << "No warm start used: not using InOut strategy.";
    }
}

#endif