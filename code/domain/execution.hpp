/**
 * Methods and objects related to the context of this project execution
 * CLI parameters, shared data structures, ...
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#ifndef HPLUS_EXECUTION_HPP
#define HPLUS_EXECUTION_HPP

#include <filesystem>

#include "../utils/utils.hpp"

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
enum class verbose { NONE = 0, STATISTICS = 1, BASIC = 2, DEBUG = 3 };

struct execution {
    // Execution parameters
    exec_type type;
    bool prep;
    warmstart ws;
    algorithm alg;
    std::string fract_cuts, cand_cuts;
    bool custom_cutloop, cl_pruning, inout;
    unsigned int cl_min_iter, cl_past_iter, io_max_iter;
    double cl_improv, cl_gap_stop, io_weight, io_weight_update;
    std::string log_file;
    unsigned int threads;
    unsigned int timelimit;
    unsigned int memorylimit;
    verbose verbosity;
    // Instance file
    std::string file, file_name;
    // Execution/Solution status
    exec_status exec_s;
    // Testing
    bool testing;
};

inline void init(execution& exec) {
    exec = hplus::execution{.type = exec_type::RUN,
                            .prep = HPLUS_DEF_PREP,
                            .ws = static_cast<warmstart>(HPLUS_DEF_WS),
                            .alg = static_cast<algorithm>(HPLUS_DEF_ALG),
                            .fract_cuts = HPLUS_DEF_FRACTCUTS,
                            .cand_cuts = HPLUS_DEF_CANDCUTS,
                            .custom_cutloop = HPLUS_DEF_CUSTOM_CUTLOOP,
                            .cl_pruning = HPLUS_DEF_CL_PRUNING,
                            .inout = HPLUS_DEF_INOUT,
                            .cl_min_iter = HPLUS_DEF_CL_MIN_ITER,
                            .cl_past_iter = HPLUS_DEF_CL_PAST_ITER,
                            .io_max_iter = HPLUS_DEF_IO_MAX_IT,
                            .cl_improv = HPLUS_DEF_CL_IMPROV,
                            .cl_gap_stop = HPLUS_DEF_CL_GAP_STOP,
                            .io_weight = HPLUS_DEF_IO_WEIGHT,
                            .io_weight_update = HPLUS_DEF_IO_WEIGHT_UPD,
                            .log_file = HPLUS_DEF_LOG,
                            .threads = HPLUS_DEF_THREADS,
                            .timelimit = HPLUS_DEF_TIMELIMIT,
                            .memorylimit = HPLUS_DEF_MEMORYLIMIT,
                            .verbosity = static_cast<verbose>(HPLUS_DEF_VERBOSE),
                            .file = "",
                            .file_name = "",
                            .exec_s = exec_status::START,
                            .testing = false};
}

[[nodiscard]]
inline std::string to_string(exec_type t) {
    switch (t) {
        case exec_type::INFO:
            return HPLUS_CLI_INFO_FLAG;
        case exec_type::RUN:
            return HPLUS_CLI_RUN_FLAG;
    }
    return "ERR";
}

[[nodiscard]]
inline std::string to_string(algorithm a) {
    switch (a) {
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
inline std::string to_string(warmstart ws) {
    switch (ws) {
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
    if (exec.verbosity < hplus::verbose::STATISTICS) return;
    LOG << "------------------ List of parameters ------------------";
    LOG << "Verbosity:                                             " << static_cast<int>(exec.verbosity);
    if (exec.log_file != "0") LOG << "Log file: " << std::filesystem::absolute(exec.log_file).lexically_normal().string();
    LOG << "Time limit:                                      " << std::setw(exec.timelimit > 0 ? 5 : 7) << exec.timelimit
        << (exec.timelimit > 0 ? " s" : "");
    LOG << "Memory limit:                              " << std::setw(exec.memorylimit > 0 ? 10 : 13) << exec.memorylimit
        << (exec.memorylimit > 0 ? " MB" : "");
    LOG << "Number of threads:                               " << std::setw(7) << exec.threads;
    LOG << "Execution type:                               " << std::setw(10) << to_string(exec.type);
    LOG << "Instance:  " << std::setw(45) << exec.file_name;
    if (exec.type == hplus::exec_type::INFO) {
        LOG << "--------------------------------------------------------";
        return;
    }
    LOG << "Preprocessing:                                         " << exec.prep;
    LOG << "Algorithm:                                    " << std::setw(10) << to_string(exec.alg);
    if (exec.alg < hplus::algorithm::GC) LOG << "Warm start:                                   " << std::setw(10) << to_string(exec.ws);
    if (exec.alg == hplus::algorithm::CUTS && !exec.fract_cuts.empty())
        LOG << "Fractional cuts:                                      " << std::setw(2) << exec.fract_cuts;
    if (exec.alg == hplus::algorithm::CUTS && !exec.cand_cuts.empty())
        LOG << "Candidate cuts:                                    " << std::setw(5) << exec.cand_cuts;
    if (exec.alg == hplus::algorithm::CUTS) LOG << "Custom cut-loop                                        " << exec.custom_cutloop;
    if (exec.custom_cutloop) {
        LOG << "Custom cutloop pruning                                 " << exec.cl_pruning;
        LOG << "Custom cutloop gap exit condition:                " << std::fixed << std::setprecision(4) << exec.cl_gap_stop;
        LOG << "Custom cutloop minimum iterations:                 " << std::setw(5) << exec.cl_min_iter;
        LOG << "Custom cutloop improvement threshold:             " << std::fixed << std::setprecision(4) << exec.cl_improv;
        LOG << "Custom cutloop past iterations comparison:         " << std::setw(5) << exec.cl_past_iter;
        LOG << "In-Out strategy:                                       " << exec.inout;
        if (exec.inout) {
            LOG << "In-Out maximum iterations:                         " << std::setw(5) << exec.io_max_iter;
            LOG << "In-Out initial incumbent weight:                    " << std::fixed << std::setprecision(2) << exec.io_weight;
            LOG << "In-Out weight update:                               " << std::fixed << std::setprecision(2) << exec.io_weight_update;
        }
    }
    if (exec.testing) LOG << "Testing mode:                                          1";
    LOG << "--------------------------------------------------------";
}
}  // namespace hplus

#endif