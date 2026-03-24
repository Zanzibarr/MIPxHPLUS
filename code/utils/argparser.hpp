/**
 * CLI parser for this project
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#pragma once

#include <stdlib.h>
#include <sys/stat.h>  // For stat buffer {}

#include <algorithm>
#include <csignal>
#include <iostream>

#include "args.hxx"
#include "execution.hpp"
#include "limits.hxx"
#include "logger.hxx"

static inline void parse_cli(const int argc, const char** argv, hplus::execution& exec) {
    args::ArgumentParser parser(
        "Find a solution / the optimal solution to the deletefree relaxation of a SAS+ planning task\nVersion: " + std::string(VERSION),
        "Copyright 2025 Matteo Zanella, Domenico Salvagnin");
    args::HelpFlag help(parser, "help", "Display the help menu", {HPLUS_CLI_HELP_FLAG});

    // ~~~~~~~~~~~~~~ INPUT FILE ~~~~~~~~~~~~~ //
    args::Positional<std::string> input_file(parser, "input file", "Specify the input file (a .sas file provided by the FastDownward translator)");

    // ~~~~~~~~~~~~~~~ LOGGING ~~~~~~~~~~~~~~~ //
    args::ValueFlag<std::string> log(
        parser, "string",
        "Write on stdout / file (def: " + std::string(HPLUS_DEF_LOG) + "; options: 0 (stdout), <file_path> (absolute path to the file to write to))",
        {HPLUS_CLI_LOG_FLAG}, HPLUS_DEF_LOG);

    // ~~~~~~~~~~~~~~~~ LIMITS ~~~~~~~~~~~~~~~ //
    args::ValueFlag<unsigned int> time_limit(parser, "non-negative int, [0,+inf)",
                                             "Set the time limit (def: " + std::to_string(HPLUS_DEF_TIMELIMIT) +
                                                 " s; options: 0 (no timelimit), <positive int (!=0)> (setting that as timelimit (seconds)))",
                                             {HPLUS_CLI_TIMELIMIT_FLAG}, HPLUS_DEF_TIMELIMIT);
    args::ValueFlag<unsigned int> threads(parser, "positive int, (0,+inf)",
                                          "Set the number of threads to use (def: " + std::to_string(HPLUS_DEF_THREADS) +
                                              " threads; options: <positive int (!= 0)> (setting that as number of threads to be used))",
                                          {HPLUS_CLI_THREADS_FLAG}, HPLUS_DEF_THREADS);
    args::ValueFlag<unsigned int> memory_limit(parser, "positive int, (0,+inf)",
                                               "Set the memory limit (def: " + std::to_string(HPLUS_DEF_MEMORYLIMIT) +
                                                   " MB; options: <positive int (!= 0)> (setting that as memory limit (MB)))",
                                               {HPLUS_CLI_MEMORYLIMIT_FLAG}, HPLUS_DEF_MEMORYLIMIT);

    // ~~~~~~~~~~~~~ RANDOM SEED ~~~~~~~~~~~~~ //
    args::ValueFlag<int> seed(parser, "int",
                              "Set a seed for random operations (def: " + std::to_string(HPLUS_DEF_RANDOM_SEED) + "; options: <positive int>)",
                              {HPLUS_CLI_SEED_FLAG}, HPLUS_DEF_RANDOM_SEED);

    // ~~~~~~~~~~~~ EXECUTION TYPE ~~~~~~~~~~~ //
    args::Flag info(parser, "info flag", "Show info about an instance", {HPLUS_CLI_INFO_FLAG});
    args::Flag run(parser, "run flag", "Run the specified algorithm", {HPLUS_CLI_RUN_FLAG});

    // ~~~~~~~~~~~ ALGORITHM TO RUN ~~~~~~~~~~ //
    args::ValueFlag<std::string> algorithm(
        parser, "string",
        "Specify the algorithm to run (default: " + std::string(HPLUS_DEF_ALG_STRING) + ", options: [" + std::string(HPLUS_CLI_ALG_FLAG_TL) +
            " (timestamps)," + std::string(HPLUS_CLI_ALG_FLAG_VE) + " (vertex elimination graphs)," + std::string(HPLUS_CLI_ALG_FLAG_CUTS) +
            " (dynamic)," + std::string(HPLUS_CLI_ALG_FLAG_GREEDYCOST) + " (greedy cost)," + std::string(HPLUS_CLI_ALG_FLAG_GREEDYCXE) +
            " (greedy cost x effect)," + std::string(HPLUS_CLI_ALG_FLAG_GREEDYHMAX) + " (lookahead hmax)," +
            std::string(HPLUS_CLI_ALG_FLAG_GREEDYHADD) + " (lookahead hadd)])",
        {HPLUS_CLI_ALG_FLAG}, HPLUS_DEF_ALG_STRING);

    // ~~~~~ WARM START AND PREPROCESSING ~~~~ //
    args::ValueFlag<std::string> warm_start(parser, "string",
                                            "Select an option for warm start (default: " + std::string(HPLUS_DEF_WS_STRING) + ", options: [" +
                                                std::string(HPLUS_CLI_WS_FLAG_NONE) + " (none)," + std::string(HPLUS_CLI_WS_FLAG_GREEDYCOST) +
                                                " (greedy cost)," + std::string(HPLUS_CLI_WS_FLAG_GREEDYCXE) + " (greedy cost x effect)," +
                                                std::string(HPLUS_CLI_WS_FLAG_GREEDYHMAX) + " (lookahead hmax)," +
                                                std::string(HPLUS_CLI_WS_FLAG_GREEDYHADD) + " (lookahead hadd)])",
                                            {HPLUS_CLI_WS_FLAG}, HPLUS_DEF_WS_STRING);
    args::ValueFlag<bool> prep(parser, "0/1",
                               "Perform preprocessing on the instance (def: " + std::to_string(HPLUS_DEF_PREP) + "; options: 0 (false), 1 (true))",
                               {HPLUS_CLI_PREP_FLAG}, HPLUS_DEF_PREP);
    args::ValueFlag<std::string> prep_lmcut(
        parser, "string",
        "Specify the pcf to find additional landmark constraints using LM-cut (def: " + std::string(HPLUS_DEF_PREP_LMCUT) +
            "; options: 0 (none: no LM-cut), a (ARB), i (INV), v (VDM), r (RND))",
        {HPLUS_CLI_PREP_LMCUT_FLAG}, HPLUS_DEF_PREP_LMCUT);
    args::ValueFlag<std::string> preplm_min(parser, "string",
                                            "Specify the minimalization procedure to use in LM-cut (for preprocessing) (def: " +
                                                std::string(HPLUS_DEF_PREP_LMCUT_MIN) + "; options: 0 (none), g (greedy), c (complete))",
                                            {HPLUS_CLI_PREP_LMCUT_MIN_FLAG}, HPLUS_DEF_PREP_LMCUT_MIN);
    args::ValueFlag<int> cutoff(parser, "int >= -1",
                                "Specify the (upper) cutoff value to give to CPLEX (def: " + std::to_string(HPLUS_DEF_CUTOFF) +
                                    "; options: -1 (none), >= 0 (the cutoff vlaue)",
                                {HPLUS_CLI_CUTOFF_FLAG}, HPLUS_DEF_CUTOFF);

    // ~~~~~~~~~~ INTEGER CALLBACKS ~~~~~~~~~~ //
    args::ValueFlag<std::string> cand_cuts(
        parser, "string",
        "(ONLY FOR [" + std::string(HPLUS_CLI_ALG_FLAG_CUTS) +
            "] ALGORITHM) Specify what cuts to separate from the candidate (integer) solutions (def: " + std::string(HPLUS_DEF_CANDCUTS) +
            "; options: 0 (don't separate cuts), a combination of ['f','c','l','s'] (respectively for frontier / complementary / LMcut landmarks and "
            "SEC))",
        {HPLUS_CLI_CANDCUTS_FLAG}, HPLUS_DEF_CANDCUTS);
    args::ValueFlag<std::string> candlm_min(
        parser, "string",
        "(ONLY FOR [" + std::string(HPLUS_CLI_ALG_FLAG_CUTS) +
            "] ALGORITHM with landmark integer separators) Specify the minimalization procedure to use in landmark candidate callbacks (def: " +
            std::string(HPLUS_DEF_CANDLM_MIN) + "; options: 0 (none), g (greedy), c (complete))",
        {HPLUS_CLI_CANDLM_MIN_FLAG}, HPLUS_DEF_CANDLM_MIN);

    // ~~~~~~~~~~~ FRACT CALLBACKS ~~~~~~~~~~~ //
    args::ValueFlag<std::string> fract_cuts(
        parser, "string",
        "(ONLY FOR [" + std::string(HPLUS_CLI_ALG_FLAG_CUTS) +
            "] ALGORITHM) Specify what cuts to separate from the fractional solutions (def: " + std::string(HPLUS_DEF_FRACTCUTS) +
            "; options: 0 (don't separate cuts), a combination of ['m','l','s'] (respectively for landmarks (with max-flow or with lmcut) and SEC))",
        {HPLUS_CLI_FRACTCUTS_FLAG}, HPLUS_DEF_FRACTCUTS);
    args::ValueFlag<bool> fract_cuts_at_nodes(
        parser, "0/1",
        "(ONLY FOR [" + std::string(HPLUS_CLI_ALG_FLAG_CUTS) +
            "] ALGORITHM) Specify whether to separate cuts also at nodes or not (if not, specified cuts, will be applied only at the root node) "
            "(def: " +
            std::to_string(HPLUS_DEF_FRACTCUTS_AT_NODES) + "; options: 0 (don't separate cuts at nodes), 1 (separate cuts also at nodes))",
        {HPLUS_CLI_FRACTCUTS_AT_NODES_FLAG}, HPLUS_DEF_FRACTCUTS_AT_NODES);
    args::ValueFlag<std::string> fractlm_min(
        parser, "string",
        "(ONLY FOR [" + std::string(HPLUS_CLI_ALG_FLAG_CUTS) +
            "] ALGORITHM with landmark fractional separators) Specify the minimalization procedure to use in landmark fractional callbacks (def: " +
            std::string(HPLUS_DEF_FRACTLM_MIN) + "; options: 0 (none), g (greedy), c (complete), i (iterative, only for option 'm' of " +
            HPLUS_CLI_FRACTCUTS_FLAG + "))",
        {HPLUS_CLI_FRACTLM_MIN_FLAG}, HPLUS_DEF_FRACTLM_MIN);

    // ~~~~~~~~~~ ROOT NODE CUTLOOP ~~~~~~~~~~ //
    args::ValueFlag<bool> custom_cutloop(parser, "0/1",
                                         "(ONLY FOR [" + std::string(HPLUS_CLI_ALG_FLAG_CUTS) +
                                             "] ALGORITHM) Flag for using the custom cutloop (def: " + std::to_string(HPLUS_DEF_CUSTOM_CUTLOOP) +
                                             "; options: 0 (don't use the custom cutloop), 1 (use the custom cutloop))",
                                         {HPLUS_CLI_CUTLOOP_FLAG}, HPLUS_DEF_CUSTOM_CUTLOOP);
    args::ValueFlag<bool> cl_pruning(parser, "0/1",
                                     "(ONLY FOR [" + std::string(HPLUS_CLI_ALG_FLAG_CUTS) +
                                         "] ALGORITHM) Flag for pruning in/after the cutloop (def: " + std::to_string(HPLUS_DEF_CL_PRUNING) +
                                         "; options: 0 (don't do pruning in the cutloop), 1 (do pruning in the cutloop))",
                                     {HPLUS_CLI_CL_PRUNING_FLAG}, HPLUS_DEF_CL_PRUNING);
    args::ValueFlag<unsigned int> cl_min_iter(
        parser, "non-negative integer, [0,+inf)",
        "Minimum number of iterations in the custom cutloop (unless no new cuts are being found or we closed enough the gap with respect to [" +
            std::string(HPLUS_CLI_CUTLOOP_GAP_STOP_FLAG) + "]) (def: " + std::to_string(HPLUS_DEF_CL_MIN_ITER) + "; options: <non-negative integer>)",
        {HPLUS_CLI_CUTLOOP_MIN_ITER_FLAG}, HPLUS_DEF_CL_MIN_ITER);
    args::ValueFlag<double> cl_improv(
        parser, "non-negative double, [0,1]",
        "Threshold for custom cutloop's termination condition on relative lower bound improvements (def: " + std::to_string(HPLUS_DEF_CL_IMPROV) +
            "; options: 0 (stay in the cutloop till no new cuts are found), > 0 (exit whenever the lower bound hasn't improved more than this "
            "percentage in the last [" +
            HPLUS_CLI_CUTLOOP_PAST_ITER_FLAG + "] iterations))",
        {HPLUS_CLI_CUTLOOP_IMPROVEMENT_FLAG}, HPLUS_DEF_CL_IMPROV);
    args::ValueFlag<unsigned int> cl_past_iter(
        parser, "positive integer, (0,+inf)",
        "How many iterations in the past the current lower bound should be compared to for the custom cutloop (def: " +
            std::to_string(HPLUS_DEF_CL_PAST_ITER) + "; options: <positive int (!=0)>)",
        {HPLUS_CLI_CUTLOOP_PAST_ITER_FLAG}, HPLUS_DEF_CL_PAST_ITER);
    args::ValueFlag<double> cl_gap_stop(parser, "non-negative double, [0,1]",
                                        "Threshold for custom cutloop's termination condition on (lower bound / incumbent) gap (def: " +
                                            std::to_string(HPLUS_DEF_CL_GAP_STOP) + "; options: <double in range [0,1]>)",
                                        {HPLUS_CLI_CUTLOOP_GAP_STOP_FLAG}, HPLUS_DEF_CL_GAP_STOP);

    // ~~~~~~~ FRACT LM (i) MINIMIZATION ~~~~~ //
    args::ValueFlag<int> lm_min_iter(
        parser, "int >= -1",
        "(ONLY FOR [" + std::string(HPLUS_CLI_ALG_FLAG_CUTS) +
            "] ALGORITHM) Specify the maximum number of minimization iterations in the fractional landmark separation procedure (def: " +
            std::to_string(HPLUS_DEF_MINIMIZATION_IT) + "; options: -1 (no limit), 0 (no minimization procedure), <positive int>)",
        {HPLUS_CLI_MINIMIZATION_BOUND_IT}, HPLUS_DEF_MINIMIZATION_IT);
    args::ValueFlag<double> lm_min_viol(parser, "non-negative double, [0,1]",
                                        "(ONLY FOR [" + std::string(HPLUS_CLI_ALG_FLAG_CUTS) +
                                            "] ALGORITHM) Specify the cut violation ratio (with respect to the violation of the first cut found) "
                                            "below which we choose to ignore the cut (def: " +
                                            std::to_string(HPLUS_DEF_MINIMIZATION_VIOL) + "; options: <double in range [0,1]>",
                                        {HPLUS_CLI_MINIMIZATION_BOUND_VIOL}, HPLUS_DEF_MINIMIZATION_VIOL);
    args::ValueFlag<int> lm_min_lh(
        parser, "int >= -1",
        "(ONLY FOR [" + std::string(HPLUS_CLI_ALG_FLAG_CUTS) +
            "] ALGORITHM) Specify the maximum number of minimization iterations in the fractional landmark separation procedure (def: " +
            std::to_string(HPLUS_DEF_MINIMIZATION_LH) + "; options: -1 (no limit), 0 (no minimization procedure), <positive int>)",
        {HPLUS_CLI_MINIMIZATION_BOUND_LH}, HPLUS_DEF_MINIMIZATION_LH);
    args::ValueFlag<bool> lm_min_sort(parser, "0/1",
                                      "(ONLY FOR [" + std::string(HPLUS_CLI_ALG_FLAG_CUTS) +
                                          "] ALGORITHM) Specify whether to sort landmarks in minimization procedure (def: " +
                                          std::to_string(HPLUS_DEF_MINIMIZATION_SORT) + "; options: 0 (don't sort landmarks), 1 (sort landmarks))",
                                      {HPLUS_CLI_MINIMIZATION_SORT}, HPLUS_DEF_MINIMIZATION_SORT);
    args::ValueFlag<bool> lm_min_improv(parser, "0/1",
                                        "(ONLY FOR [" + std::string(HPLUS_CLI_ALG_FLAG_CUTS) +
                                            "] ALGORITHM) Specify whether to only pick better landmarks in minimization procedure (def: " +
                                            std::to_string(HPLUS_DEF_MINIMIZATION_IMPROV) +
                                            "; options: 0 (don't sort landmarks), 1 (sort landmarks))",
                                        {HPLUS_CLI_MINIMIZATION_IMPROV}, HPLUS_DEF_MINIMIZATION_IMPROV);

    // ~~~~~~~~~~~~~~~~ INOUT ~~~~~~~~~~~~~~~~ //
    args::ValueFlag<bool> inout(parser, "0/1",
                                "(ONLY FOR [" + std::string(HPLUS_CLI_ALG_FLAG_CUTS) + "] ALGORITHM with custom cutloop [" +
                                    std::string(HPLUS_CLI_CUTLOOP_FLAG) + "]) Using In-Out strategy in custom cutloop (def: " +
                                    std::to_string(HPLUS_DEF_INOUT) + "; options: 0 (don't use In-Out strategy, 1 (use In-Out strategy))",
                                {HPLUS_CLI_INOUT_FLAG}, HPLUS_DEF_INOUT);
    args::ValueFlag<unsigned int> io_max_it(
        parser, "positive integer, (0,+inf)",
        "Maximum number of iterations on In-Out strategy (def: " + std::to_string(HPLUS_DEF_IO_MAX_IT) + "; options: <positive int (!=0)>)",
        {HPLUS_CLI_INOUT_MAX_ITER_FLAG}, HPLUS_DEF_IO_MAX_IT);
    args::ValueFlag<double> io_weight(parser, "non-negative double, [0,1]",
                                      "Weight to be used as initial similarity to the incumbent in In-Out strategy (def: " +
                                          std::to_string(HPLUS_DEF_IO_WEIGHT) + "; options: <double in range [0,1]>)",
                                      {HPLUS_CLI_INOUT_WEIGHT_FLAG}, HPLUS_DEF_IO_WEIGHT);
    args::ValueFlag<double> io_weight_upd(parser, "non-negative double, [0,1)",
                                          "Quantity the [" + std::string(HPLUS_CLI_INOUT_WEIGHT_FLAG) +
                                              "] parameter will be multiplied for at each In-Out iteration (def: " +
                                              std::to_string(HPLUS_DEF_IO_WEIGHT_UPD) + "; options: <double in range [0,1)>)",
                                          {HPLUS_CLI_INOUT_WEIGHT_UPD_FLAG}, HPLUS_DEF_IO_WEIGHT_UPD);

    // ~~~~~~~~~~~~~~~~ OTHER ~~~~~~~~~~~~~~~~ //
    args::ValueFlag<bool> testing(parser, "0/1",
                                  "Flag for testing or debugging (def: 0; options: 0 (testing flag set to false), 1 (testing flag set to true))",
                                  {HPLUS_CLI_TESTING_FLAG}, false);

    try {
        parser.ParseCLI(argc, argv);
    } catch (const args::Help&) {
        std::cout << parser;
        exit(EXIT_SUCCESS);
    } catch (const args::ParseError& e) {
        std::cerr << e.what() << '\n';
        std::cerr << parser;
        exit(1);
    }

    // First info we need -> to setup the logger
    if (threads) {
        unsigned int t{args::get(threads)};
        if (t == 0) {
            std::cerr << "Please specify a positive (!= 0) number of threads.\n";
            _Exit(EXIT_FAILURE);
        }
        exec.threads = t;
    }
    if (seed) {
        exec.seed = args::get(seed);
    }

    init_rng(static_cast<unsigned int>(exec.seed));

    if (log) {
        exec.log_file = HPLUS_LOG_DIR "/" + args::get(log);
        if (exec.log_file == "0") {
            default_logger().initialize(false, "", true, (exec.threads > 1), true);
        } else {
            default_logger().initialize(true, exec.log_file, true, (exec.threads > 1), true);
        }
    } else {
        default_logger().initialize(false, "", true, (exec.threads > 1), true);
    }

    LOG_INFO_S(compile_date());
    LOG_INFO_S(today());
    LOG_INFO_S(version());

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
        if (!memlim::set_memory_limit(static_cast<int>(m))) {
            LOG_ERROR_S("An error occurred while setting up memory limits");
        }
    }

    // Get parsed info
    if (input_file) {
        exec.file = args::get(input_file);
        exec.file_name = std::filesystem::path(exec.file).filename().string();
    } else {
        LOG_ERROR_S("Missing input file");
    }
    struct stat buffer{};
    if (stat((exec.file).c_str(), &buffer) != 0) {
        LOG_ERROR_S("Failed to open input file");
    }
    if (run) {
        exec.type = hplus::exec_type::RUN;
    } else if (info) {
        exec.type = hplus::exec_type::INFO;
    }
    if (algorithm) {
        std::string a{args::get(algorithm)};
        if (a == HPLUS_CLI_ALG_FLAG_TL) {
            exec.alg = hplus::algorithm::TL;
        } else if (a == HPLUS_CLI_ALG_FLAG_VE) {
            exec.alg = hplus::algorithm::VE;
        } else if (a == HPLUS_CLI_ALG_FLAG_CUTS) {
            exec.alg = hplus::algorithm::CUTS;
        } else if (a == HPLUS_CLI_ALG_FLAG_GREEDYCOST) {
            exec.alg = hplus::algorithm::GC;
        } else if (a == HPLUS_CLI_ALG_FLAG_GREEDYCXE) {
            exec.alg = hplus::algorithm::GCXE;
        } else if (a == HPLUS_CLI_ALG_FLAG_GREEDYHMAX) {
            exec.alg = hplus::algorithm::GHM;
        } else if (a == HPLUS_CLI_ALG_FLAG_GREEDYHADD) {
            exec.alg = hplus::algorithm::GHA;
        } else {
            LOG_ERROR_S("Algorithm '" + a + "' is not in the list of possible algorithms");
        }
    }
    if (warm_start) {
        std::string ws{args::get(warm_start)};
        if (ws == HPLUS_CLI_WS_FLAG_NONE) {
            exec.ws = hplus::warmstart::NONE;
        } else if (ws == HPLUS_CLI_WS_FLAG_GREEDYCXE) {
            exec.ws = hplus::warmstart::GCXE;
        } else if (ws == HPLUS_CLI_WS_FLAG_GREEDYCOST) {
            exec.ws = hplus::warmstart::GC;
        } else if (ws == HPLUS_CLI_WS_FLAG_GREEDYHMAX) {
            exec.ws = hplus::warmstart::GHM;
        } else if (ws == HPLUS_CLI_WS_FLAG_GREEDYHADD) {
            exec.ws = hplus::warmstart::GHA;
        } else {
            LOG_ERROR_S("Warm start '" + ws + "' is not in the list of possible warm starts");
        }
    }
    if (prep) {
        exec.prep = args::get(prep);
    }
    if (prep_lmcut) {
        exec.prep_lmcut = "";
        std::string s{args::get(prep_lmcut)};
        if (s != "0") {
            std::erase_if(s, [&](const char a) { return std::string(HPLUS_ALL_LMCUT_PCF).find(a) == std::string::npos; });
            exec.prep_lmcut = s;
            if (exec.prep_lmcut.empty()) {
                LOG_WARN_S("Wrong parameter for " + std::string(HPLUS_CLI_PREP_LMCUT_FLAG) + "; using default value: " + HPLUS_DEF_PREP_LMCUT);
                exec.prep_lmcut = HPLUS_DEF_PREP_LMCUT;
            }
        } else {
            exec.prep_lmcut = s;
        }
    }
    if (preplm_min) {
        exec.preplm_min = "";
        std::string s = args::get(preplm_min);
        if (s != "0") {
            if (s.find('g') != std::string::npos) {
                exec.preplm_min += "g";
            }
            if (s.find('c') != std::string::npos) {
                exec.preplm_min += "c";
            }
            if (exec.preplm_min.empty()) {
                LOG_WARN_S("Wrong parameter for " + std::string(HPLUS_CLI_PREP_LMCUT_MIN_FLAG) +
                           "; using default value: " + HPLUS_DEF_PREP_LMCUT_MIN);
                exec.preplm_min = HPLUS_DEF_PREP_LMCUT_MIN;
            }
        } else {
            exec.preplm_min = s;
        }
    }
    if (cutoff) {
        exec.cutoff = args::get(cutoff);
        exec.cutoff = std::max(exec.cutoff, HPLUS_DEF_CUTOFF);
    }
    if (fract_cuts) {
        exec.fract_cuts = "";
        std::string s{args::get(fract_cuts)};
        if (s != "0") {
            if (s.find('m') != std::string::npos) {
                exec.fract_cuts += "m";
            }
            if (s.find('l') != std::string::npos) {
                exec.fract_cuts += "l";
            }
            if (s.find('s') != std::string::npos) {
                exec.fract_cuts += "s";
            }
            if (exec.fract_cuts.empty()) {
                LOG_WARN_S("Wrong parameter for " + std::string(HPLUS_CLI_FRACTCUTS_FLAG) + "; using default value: " + HPLUS_DEF_FRACTCUTS);
                exec.fract_cuts = HPLUS_DEF_FRACTCUTS;
            }
        } else {
            exec.fract_cuts = s;
        }
    }
    if (fractlm_min) {
        exec.fractlm_min = "";
        std::string s = args::get(fractlm_min);
        if (s != "0") {
            if (s.find('g') != std::string::npos) {
                exec.fractlm_min += "g";
            }
            if (s.find('c') != std::string::npos) {
                exec.fractlm_min += "c";
            }
            if (s.find('i') != std::string::npos) {
                exec.fractlm_min += "i";
            }
            if (exec.fractlm_min.empty()) {
                LOG_WARN_S("Wrong parameter for " + std::string(HPLUS_CLI_FRACTLM_MIN_FLAG) + "; using default value: " + HPLUS_DEF_FRACTLM_MIN);
                exec.fractlm_min = HPLUS_DEF_FRACTLM_MIN;
            }
        } else {
            exec.fractlm_min = s;
        }
    }
    if (exec.fractlm_min.find('i') != std::string::npos) {
        if (lm_min_iter) {
            int i = args::get(lm_min_iter);
            if (i == 0) {
                LOG_WARN_S(
                    "Setting 0 iterations means to not use the Iterative ('i') minimization procedure; removing 'i' landmark minimization procedure");
                std::erase_if(exec.fractlm_min, [](const auto val) -> auto { return val == 'i'; });
            } else if (i < 0) {
                exec.lm_min_it = INFBOUND_INT;
            } else {
                exec.lm_min_it = static_cast<unsigned int>(i);
            }
        }
        if (lm_min_viol) {
            double v = args::get(lm_min_viol);
            if (v < 0 || v > 1) {
                LOG_WARN_S("Illegal value for " + std::string(HPLUS_CLI_MINIMIZATION_BOUND_VIOL) +
                           "; using default value: " + std::to_string(HPLUS_DEF_MINIMIZATION_VIOL));
            } else {
                exec.lm_min_viol = v;
            }
        }
        if (lm_min_lh) {
            int i = args::get(lm_min_lh);
            if (i == 0) {
                LOG_WARN_S(
                    "Setting 0 lookahead iterations means to not use the Iterative ('i') minimization procedure; removing 'i' landmark minimization "
                    "procedure");
                std::erase_if(exec.fractlm_min, [](const auto val) -> auto { return val == 'i'; });
            } else if (i < 0) {
                exec.lm_min_lookahead = INFBOUND_INT;
            } else {
                exec.lm_min_lookahead = static_cast<unsigned int>(i);
            }
        }
        if (lm_min_sort) {
            exec.lm_min_sort = args::get(lm_min_sort);
        }
        if (lm_min_improv) {
            exec.lm_min_improv = args::get(lm_min_improv);
        }
    }
    if (fract_cuts_at_nodes) {
        exec.fract_cuts_at_nodes = args::get(fract_cuts_at_nodes);
    }

    if (cand_cuts) {
        exec.cand_cuts = "";
        std::string s{args::get(cand_cuts)};
        if (s != "0") {
            if (s.find('f') != std::string::npos) {
                exec.cand_cuts += "f";
            }
            if (s.find('c') != std::string::npos) {
                exec.cand_cuts += "c";
            }
            if (s.find('l') != std::string::npos) {
                exec.cand_cuts += "l";
            }
            if (s.find('s') != std::string::npos) {
                exec.cand_cuts += "s";
            }
            if (exec.cand_cuts.empty()) {
                LOG_WARN_S("Wrong parameter for " + std::string(HPLUS_CLI_CANDCUTS_FLAG) + "; using default value: " + HPLUS_DEF_CANDCUTS);
                exec.cand_cuts = HPLUS_DEF_CANDCUTS;
            }
        } else {
            exec.cand_cuts = s;
        }
    }
    if (candlm_min) {
        exec.candlm_min = "";
        std::string s = args::get(candlm_min);
        if (s != "0") {
            if (s.find('g') != std::string::npos) {
                exec.candlm_min += "g";
            }
            if (s.find('c') != std::string::npos) {
                exec.candlm_min += "c";
            }
            if (exec.candlm_min.empty()) {
                LOG_WARN_S("Wrong parameter for " + std::string(HPLUS_CLI_CANDLM_MIN_FLAG) + "; using default value: " + HPLUS_DEF_CANDLM_MIN);
                exec.candlm_min = HPLUS_DEF_CANDLM_MIN;
            }
        } else {
            exec.candlm_min = s;
        }
    }
    if (custom_cutloop) {
        exec.custom_cutloop = args::get(custom_cutloop);
    }
    if (cl_pruning) {
        exec.cl_pruning = args::get(cl_pruning);
    }
    if (cl_min_iter) {
        exec.cl_min_iter = args::get(cl_min_iter);
    }
    if (cl_improv) {
        double improv = args::get(cl_improv);
        if (improv < 0 || improv > 1) {
            LOG_WARN_S("Illegal value for " + std::string(HPLUS_CLI_CUTLOOP_IMPROVEMENT_FLAG) +
                       "; using default value: " + std::to_string(HPLUS_DEF_CL_IMPROV));
        } else {
            exec.cl_improv = improv;
        }
    }
    if (cl_past_iter) {
        unsigned int it = args::get(cl_past_iter);
        if (it == 0) {
            LOG_WARN_S("Illegal value for " + std::string(HPLUS_CLI_CUTLOOP_PAST_ITER_FLAG) +
                       "; using default value: " + std::to_string(HPLUS_DEF_CL_PAST_ITER));
        } else {
            exec.cl_past_iter = it;
        }
    }
    if (cl_gap_stop) {
        double gap = args::get(cl_gap_stop);
        if (gap < 0 || gap > 1) {
            LOG_WARN_S("Illegal value for " + std::string(HPLUS_CLI_CUTLOOP_GAP_STOP_FLAG) +
                       "; using default value: " + std::to_string(HPLUS_DEF_CL_GAP_STOP));
        } else {
            exec.cl_gap_stop = gap;
        }
    }
    if (inout) {
        exec.inout = args::get(inout);
    }
    if (io_max_it) {
        unsigned int it = args::get(io_max_it);
        if (it == 0) {
            LOG_WARN_S("Illegal value for " + std::string(HPLUS_CLI_INOUT_MAX_ITER_FLAG) +
                       "; using default value: " + std::to_string(HPLUS_DEF_IO_MAX_IT));
        } else {
            exec.io_max_iter = it;
        }
    }
    if (io_weight) {
        double weight = args::get(io_weight);
        if (weight < 0 || weight > 1) {
            LOG_WARN_S("Illegal value for " + std::string(HPLUS_CLI_INOUT_WEIGHT_FLAG) +
                       "; using default value: " + std::to_string(HPLUS_DEF_IO_WEIGHT));
        } else {
            exec.io_weight = weight;
        }
    }
    if (io_weight_upd) {
        double weight = args::get(io_weight_upd);
        if (weight < 0 || weight >= 1) {
            LOG_WARN_S("Illegal value for " + std::string(HPLUS_CLI_INOUT_WEIGHT_UPD_FLAG) +
                       "; using default value: " + std::to_string(HPLUS_DEF_IO_WEIGHT_UPD));
        } else {
            exec.io_weight_update = weight;
        }
    }
    if (testing) {
        exec.testing = args::get(testing);
        if (exec.testing) {
            LOG_WARN_S("Testing flag set to true");
        }
    }

    // Check that it's all as it's supposed to be
    if (exec.threads > std::thread::hardware_concurrency()) {
        exec.threads = std::thread::hardware_concurrency();
        LOG_WARN_S("This machine has " + std::to_string(exec.threads) + " cores: using up to " + std::to_string(exec.threads) + " threads");
    }
    if (info && run) {
        LOG_ERROR_S("You need to specify only one functionality among " + std::string(HPLUS_CLI_INFO_FLAG) + " and " + HPLUS_CLI_RUN_FLAG);
    }

    // Preprocessing
    if (!exec.prep) {
        exec.prep_lmcut = "0";
    }

    // Warm start
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
                LOG_ERROR_S("Unhandled algorithm in parse_cli: " + std::to_string(static_cast<int>(exec.alg)));
        }
    }

    // Ensure correct parameters for each model
    if (exec.alg != hplus::algorithm::CUTS) {
        exec.cand_cuts = "0";
        exec.fract_cuts = "0";
        exec.custom_cutloop = false;
    } else {
        if (exec.cand_cuts.empty()) {
            LOG_ERROR_S("You can't disable all three candidate cuts from the " + std::string(HPLUS_CLI_ALG_FLAG_CUTS) + " algorithm");
        }
        if (exec.fract_cuts == "0") {
            if (exec.custom_cutloop) {
                LOG_WARN_S("If you want to use the custom cutloop, please specify a fractional solution separator: disabling custom cutloop");
            }
            exec.fract_cuts_at_nodes = false;
            exec.fractlm_min = "0";
            exec.custom_cutloop = false;
        }
        if (exec.custom_cutloop && exec.ws == hplus::warmstart::NONE && exec.inout) {
            LOG_WARN_S("Warmstart disabled: disabling In-Out strategy");
            exec.inout = false;
        }
        if (exec.candlm_min.find('g') != std::string::npos && exec.cand_cuts.find('l') == std::string::npos) {
            LOG_WARN_S(
                "Greedy minimalization procedure is implicit in the landmark construction of Fronteer ('f') and Complementary ('c') cand separators: "
                "disabling greedy "
                "minimalization procedure");
            std::erase_if(exec.candlm_min, [](const auto val) { return val == 'g'; });
        }
        if (exec.fractlm_min.find('i') != std::string::npos && exec.fract_cuts.find('m') == std::string::npos) {
            LOG_WARN_S(
                "Iterative ('i') minimalization procedure is used only for Max-Flow-based ('m') fractional separato: disabling iterative "
                "minimalization "
                "procedure");
            std::erase_if(exec.fractlm_min, [](const auto val) { return val == 'i'; });
        }
        if (exec.fract_cuts.find('l') != std::string::npos && exec.fract_cuts.find('l') == std::string::npos &&
            exec.fractlm_min.find('g') != std::string::npos) {
            LOG_WARN_S(
                "Greedy minimalization procedure is implicit in the landmark construction of Max-Flow-based ('m') fractional separator: disabling "
                "greedy minimalization procedure");
            std::erase_if(exec.fractlm_min, [](const auto val) { return val == 'g'; });
        }
    }
}
