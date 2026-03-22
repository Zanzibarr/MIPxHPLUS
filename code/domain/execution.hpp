/**
 * Methods and objects related to the context of this project execution
 * CLI parameters, shared data structures, ...
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#pragma once

#include <filesystem>

#include "logger.hxx"
#include "utils.hpp"

namespace hplus {

enum class exec_status {
    START = 0,
    INSTANCE_BUILDING = 10,
    PREPROCESSING = 20,
    HEURISTIC = 30,
    MODEL_BUILD = 40,
    CUTLOOP = 45,
    CPX_EXEC = 50,
    EXIT = 100
};

enum class exec_type { INFO, RUN };
enum class algorithm { TL = 0, VE = 1, CUTS = 2, GC = 10, GCXE = 11, GHM = 12, GHA = 13 };
enum class warmstart { NONE = 0, GC = 1, GCXE = 2, GHM = 3, GHA = 4 };

struct execution {
    // Instance file
    std::string file, file_name;
    // Execution parameters
    exec_type type;
    algorithm alg;
    std::string log_file;
    unsigned int threads, timelimit, memorylimit;
    int seed;
    // Preprocessing
    bool prep;
    std::string prep_lmcut;
    std::string preplm_min;
    warmstart ws;
    // Cutoff
    int cutoff;
    // Integer cuts
    std::string cand_cuts;
    std::string candlm_min;
    // Fractional cuts
    std::string fract_cuts;
    bool fract_cuts_at_nodes;
    std::string fractlm_min;
    // Cutloop
    bool custom_cutloop, cl_pruning;
    unsigned int cl_min_iter, cl_past_iter;
    double cl_improv, cl_gap_stop;
    // Inout
    bool inout;
    unsigned int io_max_iter;
    double io_weight, io_weight_update;
    // Fract LM Minimization
    bool lm_min_sort, lm_min_improv;
    unsigned int lm_min_it, lm_min_lookahead;
    double lm_min_viol;
    // Execution/Solution status
    exec_status exec_s;
    // Testing
    bool testing;
};

inline void init(execution& exec) {
    exec = hplus::execution{.file = "",
                            .file_name = "",
                            .type = exec_type::RUN,
                            .alg = static_cast<algorithm>(HPLUS_DEF_ALG),
                            .log_file = HPLUS_DEF_LOG,
                            .threads = HPLUS_DEF_THREADS,
                            .timelimit = HPLUS_DEF_TIMELIMIT,
                            .memorylimit = HPLUS_DEF_MEMORYLIMIT,
                            .seed = HPLUS_DEF_RANDOM_SEED,
                            .prep = HPLUS_DEF_PREP,
                            .prep_lmcut = HPLUS_DEF_PREP_LMCUT,
                            .preplm_min = HPLUS_DEF_PREP_LMCUT_MIN,
                            .ws = static_cast<warmstart>(HPLUS_DEF_WS),
                            .cutoff = HPLUS_DEF_CUTOFF,
                            .cand_cuts = HPLUS_DEF_CANDCUTS,
                            .candlm_min = HPLUS_DEF_CANDLM_MIN,
                            .fract_cuts = HPLUS_DEF_FRACTCUTS,
                            .fract_cuts_at_nodes = HPLUS_DEF_FRACTCUTS_AT_NODES,
                            .fractlm_min = HPLUS_DEF_FRACTLM_MIN,
                            .custom_cutloop = HPLUS_DEF_CUSTOM_CUTLOOP,
                            .cl_pruning = HPLUS_DEF_CL_PRUNING,
                            .cl_min_iter = HPLUS_DEF_CL_MIN_ITER,
                            .cl_past_iter = HPLUS_DEF_CL_PAST_ITER,
                            .cl_improv = HPLUS_DEF_CL_IMPROV,
                            .cl_gap_stop = HPLUS_DEF_CL_GAP_STOP,
                            .inout = HPLUS_DEF_INOUT,
                            .io_max_iter = HPLUS_DEF_IO_MAX_IT,
                            .io_weight = HPLUS_DEF_IO_WEIGHT,
                            .io_weight_update = HPLUS_DEF_IO_WEIGHT_UPD,
                            .lm_min_sort = HPLUS_DEF_MINIMIZATION_SORT,
                            .lm_min_improv = HPLUS_DEF_MINIMIZATION_IMPROV,
                            .lm_min_it = HPLUS_DEF_MINIMIZATION_IT,
                            .lm_min_lookahead = HPLUS_DEF_MINIMIZATION_LH,
                            .lm_min_viol = HPLUS_DEF_MINIMIZATION_VIOL,
                            .exec_s = exec_status::START,
                            .testing = false};
}

[[nodiscard]]
inline auto to_string(exec_type type) -> std::string {
    switch (type) {
        case exec_type::INFO:
            return HPLUS_CLI_INFO_FLAG;
        case exec_type::RUN:
            return HPLUS_CLI_RUN_FLAG;
    }
    return "ERR";
}

[[nodiscard]]
inline auto to_string(algorithm alg) -> std::string {
    switch (alg) {
        case algorithm::TL:
            return HPLUS_CLI_ALG_FLAG_TL;
        case algorithm::VE:
            return HPLUS_CLI_ALG_FLAG_VE;
        case algorithm::CUTS:
            return HPLUS_CLI_ALG_FLAG_CUTS;
        case algorithm::GC:
            return HPLUS_CLI_ALG_FLAG_GREEDYCOST;
        case algorithm::GCXE:
            return HPLUS_CLI_ALG_FLAG_GREEDYCXE;
        case algorithm::GHM:
            return HPLUS_CLI_ALG_FLAG_GREEDYHMAX;
        case algorithm::GHA:
            return HPLUS_CLI_ALG_FLAG_GREEDYHADD;
    }
    return "ERR";
}

[[nodiscard]]
inline auto to_string(warmstart wst) -> std::string {
    switch (wst) {
        case warmstart::NONE:
            return HPLUS_CLI_WS_FLAG_NONE;
        case warmstart::GC:
            return HPLUS_CLI_WS_FLAG_GREEDYCOST;
        case warmstart::GCXE:
            return HPLUS_CLI_WS_FLAG_GREEDYCXE;
        case warmstart::GHM:
            return HPLUS_CLI_WS_FLAG_GREEDYHMAX;
        case warmstart::GHA:
            return HPLUS_CLI_WS_FLAG_GREEDYHADD;
    }
    return "ERR";
}

inline void print(const execution& exec) {
    LOG_S("------------------ List of parameters ------------------");
    if (exec.log_file != "0") {
        LOG_S("Log file: " + std::filesystem::absolute(exec.log_file).lexically_normal().string());
    }
    LOG << "Time limit:                                      " << std::setw(exec.timelimit > 0 ? 5 : 7) << exec.timelimit
        << (exec.timelimit > 0 ? " s" : "");
    LOG << "Memory limit:                              " << std::setw(exec.memorylimit > 0 ? 10 : 13) << exec.memorylimit
        << (exec.memorylimit > 0 ? " MB" : "");
    LOG << "Number of threads:                               " << std::setw(7) << exec.threads;
    LOG << "Seed:                                            " << std::setw(7) << exec.seed;
    LOG << "Execution type:                               " << std::setw(10) << to_string(exec.type);
    LOG << "Instance:  " << std::setw(45) << exec.file_name;
    if (exec.type == hplus::exec_type::INFO) {
        LOG_S("--------------------------------------------------------");
        return;
    }
    LOG_S("Preprocessing:                                         " + std::to_string(static_cast<int>(exec.prep)));
    LOG << "Preprocessing (LMcut):                        " << std::setw(10) << exec.prep_lmcut;
    LOG << "LMcut (preprocessing) minimalization:         " << std::setw(10) << exec.preplm_min;
    LOG << "Algorithm:                                    " << std::setw(10) << to_string(exec.alg);
    if (exec.alg < hplus::algorithm::GC) {
        LOG << "Warm start:                                   " << std::setw(10) << to_string(exec.ws);
    }
    if (exec.cutoff >= 0) {
        LOG << "Cutoff value:                                 " << std::setw(10) << exec.cutoff;
    }
    if (exec.alg == hplus::algorithm::CUTS) {
        LOG << "Candidate cuts:                                    " << std::setw(5) << exec.cand_cuts;
        if (exec.cand_cuts.find('l') != std::string::npos || exec.cand_cuts.find('c') != std::string::npos ||
            exec.cand_cuts.find('f') != std::string::npos) {
            LOG << "Landmark minimalization in candidate cuts:         " << std::setw(5) << exec.candlm_min;
        }

        LOG << "Fractional cuts:                                      " << std::setw(2) << exec.fract_cuts;
        if (exec.fract_cuts.find('l') != std::string::npos || exec.fract_cuts.find('m') != std::string::npos) {
            LOG << "Landmark minimalization in fractional cuts:        " << std::setw(5) << exec.fractlm_min;
        }
        if (exec.fractlm_min.find('i') != std::string::npos) {
            LOG << "- Upper bound on number of iterations:        " << std::setw(10) << exec.lm_min_it;
            LOG << "- Violation ratio threshold:                       " << std::fixed << std::setprecision(3) << exec.lm_min_viol;
            LOG << "- Lookahead success rate threshold:           " << std::setw(10) << exec.lm_min_lookahead;
            LOG_S("- Sorting of landmark:                                 " + std::to_string(static_cast<int>(exec.lm_min_sort)));
            LOG_S("- Expand only better landmarks:                        " + std::to_string(static_cast<int>(exec.lm_min_improv)));
        }
        if (exec.fract_cuts != "0") {
            LOG_S("Fractional cuts at nodes:                              " + std::to_string(static_cast<int>(exec.fract_cuts_at_nodes)));
        }
        if (exec.fract_cuts != "0") {
            LOG_S("Custom cut-loop                                        " + std::to_string(static_cast<int>(exec.custom_cutloop)));
        }
    }
    if (exec.custom_cutloop) {
        LOG_S("- Pruning                                              " + std::to_string(static_cast<int>(exec.cl_pruning)));
        LOG << "- Gap exit condition:                             " << std::fixed << std::setprecision(4) << exec.cl_gap_stop;
        LOG << "- Minimum iterations:                              " << std::setw(5) << exec.cl_min_iter;
        LOG << "- Improvement threshold:                          " << std::fixed << std::setprecision(4) << exec.cl_improv;
        LOG << "- Past iterations comparison:                      " << std::setw(5) << exec.cl_past_iter;
        LOG_S("In-Out strategy:                                       " + std::to_string(static_cast<int>(exec.inout)));
        if (exec.inout) {
            LOG << "- Maximum iterations:                              " << std::setw(5) << exec.io_max_iter;
            LOG << "- Initial incumbent weight:                         " << std::fixed << std::setprecision(2) << exec.io_weight;
            LOG << "- Weight update:                                    " << std::fixed << std::setprecision(2) << exec.io_weight_update;
        }
    }
    if (exec.testing) {
        LOG_S("Testing mode:                                          1");
    }
    LOG_S("--------------------------------------------------------");
}
}  // namespace hplus
