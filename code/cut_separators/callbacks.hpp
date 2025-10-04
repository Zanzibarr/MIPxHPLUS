/**
 * Methods for CPLEX generic callbacks
 * Extends to cand_callback.hpp for the methods used for CPLEX's candidate callback, and relax_callback.hpp for the methods used for CPLEX's
 * relaxation callback
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#ifndef HPLUS_CALLBACKS_HPP
#define HPLUS_CALLBACKS_HPP

#include <cplex.h>

#include "cand_callback.hpp"
#include "relax_callback.hpp"

namespace relax_cuts {

/**
 * FLM-DET is the model used for Fractional LandMarks DETection in the relax callback
 */
inline void open_flmdet_model(CPXENVptr& env, CPXLPptr& lp, int threads = 1) {
    int cpxerror;
    env = CPXopenCPLEX(&cpxerror);
    CPX_HANDLE_CALL(cpxerror);
    lp = CPXcreateprob(env, &cpxerror, "flmdet");
    CPX_HANDLE_CALL(cpxerror);
    CPX_HANDLE_CALL(CPXsetintparam(env, CPXPARAM_Threads, threads));
    // log file
    CPX_HANDLE_CALL(CPXsetintparam(env, CPXPARAM_ScreenOutput, HPLUS_DEF_CPX_SCREENOUTPUT));
    CPX_HANDLE_CALL(CPXsetintparam(env, CPX_PARAM_CLONELOG, HPLUS_DEF_CPX_CLONELOG));
    // tolerance
    CPX_HANDLE_CALL(CPXsetdblparam(env, CPXPARAM_MIP_Tolerances_MIPGap, HPLUS_DEF_CPX_TOL_GAP));
    // memory/size limits
    CPX_HANDLE_CALL(CPXsetdblparam(env, CPXPARAM_MIP_Limits_TreeMemory, HPLUS_DEF_CPX_TREE_MEM));
    CPX_HANDLE_CALL(CPXsetdblparam(env, CPXPARAM_WorkMem, HPLUS_DEF_MEMORYLIMIT));
    CPX_HANDLE_CALL(CPXsetintparam(env, CPXPARAM_MIP_Strategy_File, HPLUS_DEF_CPX_STRAT_FILE));
    // terminate condition
    CPX_HANDLE_CALL(CPXsetterminate(env, &GLOBAL_TERMINATE_CONDITION));
}

/**
 * FLM-DET is the model used for Fractional LandMarks DETection in the relax callback
 * Model (minimum cut in a source-sink partition):
 * { min \sum_{a\in A} x_a^* z_a
 * { z_a >= \sum_{p\in pre(a)}y_p - |pre(a)| + 1 - y_q        \forall a,\forall q\in add(a)             (An action is crossing the cut if all
 * precondition are on the left side (y_p = 1 \forall p\in pre(a)) and at least one effect is on the right side (y_q = 1))
 * { \sum_{p\in G}y_p <= |G| - 1                                                                        (We cannot have all facts of the goal in the
 * left side)
 */
inline void build_flmdet_model(const hplus::instance& inst, CPXENVptr& env, CPXLPptr& lp) {
    // ~~~~~~~~~~~~~~ VARIABLES ~~~~~~~~~~~~~~ //
    size_t ncols{inst.m + inst.n};
    const auto get_za_idx = [](size_t idx) { return idx; };  // 0 -> inst.m - 1: z_a
    const auto get_yp_idx = [&inst](size_t idx) {            // inst.m + 1 -> ncols - 1: y_p
        return inst.m + idx;
    };

    std::vector<double> objs(ncols, 0.0);  // the coefficients of the objective function will be set in the callback, based on the fractional solution
    std::vector<double> lbs(ncols, 0.0);
    std::vector<double> ubs(ncols, 1.0);
    std::vector<char> types(ncols, 'B');
    CPX_HANDLE_CALL(CPXnewcols(env, lp, ncols, objs.data(), lbs.data(), ubs.data(), types.data(), nullptr));

    // ~~~~~~~~~~~~~ CONSTRAINTS ~~~~~~~~~~~~~ //
    std::vector<int> ind(inst.n + 1);
    std::vector<double> val(inst.n + 1);
    int nnz{0};
    constexpr char sense{'L'};
    constexpr int begin{0};

    for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
        nnz = 0;
        ind[nnz] = get_za_idx(act_i);
        val[nnz++] = -1;
        const double rhs{static_cast<double>(inst.actions[act_i].pre_sparse.size()) - 1};
        for (const auto& p : inst.actions[act_i].pre_sparse) {
            ind[nnz] = get_yp_idx(p);
            val[nnz++] = 1;
        }
        for (const auto& q : inst.actions[act_i].eff_sparse) {
            ind[nnz] = get_yp_idx(q);
            val[nnz] = -1;
            CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, nnz + 1, &rhs, &sense, &begin, ind.data(), val.data(), nullptr, nullptr));
        }
    }

    nnz = 0;
    for (const auto& p : inst.goal) {
        ind[nnz] = get_yp_idx(p);
        val[nnz++] = 1;
    }
    const double rhs{static_cast<double>(inst.goal.sparse().size()) - 1};
    CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, nnz, &rhs, &sense, &begin, ind.data(), val.data(), nullptr, nullptr));
}

inline void create_flmdet_model(const hplus::instance& inst, CPXENVptr& env, CPXLPptr& lp, int threads = 1) {
    open_flmdet_model(env, lp, threads);
    build_flmdet_model(inst, env, lp);
}

inline void close_flmdet_model(CPXENVptr& env, CPXLPptr& lp) {
    CPX_HANDLE_CALL(CPXfreeprob(env, &lp));
    CPX_HANDLE_CALL(CPXcloseCPLEX(&env));
}
}  // namespace relax_cuts

namespace callbacks {

// thread_data is defined in relax_callback.hpp (circular dependencies, idk how to avoid it

struct callback_userhandle {
    hplus::execution& exec;
    hplus::instance& inst;
    hplus::statistics& stats;
    std::vector<thread_data> thread_specific_data;
};

/**
 * Method called in CPLEX's generic callback
 * This methods handles the routing of the callbacks based on the context
 */
static int CPXPUBLIC callback_hub(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void* userhandle) {
    auto& [exec, inst, stats, thread_data]{*static_cast<callback_userhandle*>(userhandle)};
    int thread_id{-1};
    CPX_HANDLE_CALL(CPXcallbackgetinfoint(context, CPXCALLBACKINFO_THREADID, &thread_id));

    switch (contextid) {
        case CPX_CALLBACKCONTEXT_RELAXATION:
            relaxation_callback(context, exec, inst, thread_data[thread_id]);
            break;
        case CPX_CALLBACKCONTEXT_CANDIDATE:
            candidate_callback(context, exec, inst, stats, thread_data[thread_id].usercuts_lm, thread_data[thread_id].usercuts_sec,
                               thread_data[thread_id].cand_time, thread_data[thread_id].cand_calls);
            break;
        default:
            LOG_ERROR << "Unhandled CPLEX callback context: " << contextid;
    }

    return 0;
}

inline void set_cplex_callbacks(hplus::execution& exec, hplus::instance& inst, callback_userhandle& userhandle, CPXENVptr& env, CPXLPptr& lp) {
    if (BASIC_VERBOSE()) LOG_INFO << "Setting up CPLEX callbacks";

    for (unsigned int t = 0; t < exec.threads; ++t) {
        thread_data td{
            .usercuts_lm = 0,
            .usercuts_sec = 0,
            .relax_calls = 0,
            .cand_calls = 0,
            .cand_time = 0.0,
            .relax_time = 0.0,
            .flmdet_env = nullptr,
            .flmdet_lp = nullptr,
        };

        if (exec.fract_cuts.find('l') != std::string::npos)  // l here stands for landmarks -> if we need landmarks we need the mip that detects them
            relax_cuts::create_flmdet_model(inst, td.flmdet_env,
                                            td.flmdet_lp);  // We use only 1 thread since CPLEX might run in multithreading before it finds an initial
                                                            // basis (we must assure that the number of threads specified is used, and not more)

        userhandle.thread_specific_data.push_back(std::move(td));
    }

    // Setting up callbacks
    CPXLONG callback_contex = CPX_CALLBACKCONTEXT_CANDIDATE;  // The candidate callback is ALWAYS needed (otherwise the model might be incomplete due
                                                              // to missing causal acyclicity)

    // If we have no cust to add -> no callback
    // If we have cuts and we have no cutloop, we need to add the callback, since cuts at root must be done somewhere (wether cuts must be done at
    // nodes or not) -> yes callback
    // If we have cuts and we have cuts at nodes -> yes callback
    if (exec.fract_cuts != "0" && (exec.fract_cuts_at_nodes || !exec.custom_cutloop)) callback_contex |= CPX_CALLBACKCONTEXT_RELAXATION;

    CPX_HANDLE_CALL(CPXcallbacksetfunc(env, lp, callback_contex, callback_hub, &userhandle));
}

inline void gather_stats_from_threads(const hplus::execution& exec, hplus::statistics& stats, callback_userhandle& thread_data) {
    for (auto& data : thread_data.thread_specific_data) {
        stats.cand_callback += data.cand_time;
        stats.relax_callback += data.relax_time;
        stats.cuts_lm += data.usercuts_lm;
        stats.cuts_sec += data.usercuts_sec;
        stats.cand_calls += data.cand_calls;
        stats.relax_calls += data.relax_calls;
        if (exec.fract_cuts.find('l') != std::string::npos) relax_cuts::close_flmdet_model(data.flmdet_env, data.flmdet_lp);
    }
}
}  // namespace callbacks

#endif