/**
 * Methods for CPLEX generic callbacks
 * Extends to cand_callback.hpp for the methods used for CPLEX's candidate callback, and relax_callback.hpp for the methods used for CPLEX's
 * relaxation callback
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#pragma once

#include <cplex.h>

#include "cand_callback.hpp"
#include "relax_callback.hpp"

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
static auto CPXPUBLIC callback_hub(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void* userhandle) -> int {
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
            LOG_ERROR_S("Unhandled CPLEX callback context: " + std::to_string(contextid));
    }

    return 0;
}

inline void set_cplex_callbacks(hplus::execution& exec, callback_userhandle& userhandle, CPXENVptr& env, CPXLPptr& lp) {
    LOG_INFO_S("Setting up CPLEX callbacks");

    for (unsigned int _ = 0; _ < exec.threads; ++_) {
        thread_data thr_data{
            .usercuts_lm = 0,
            .usercuts_sec = 0,
            .relax_calls = 0,
            .cand_calls = 0,
            .acts_in_lm = 0,
            .n_lm = 0,
            .cand_time = 0.0,
            .relax_time = 0.0,
        };

        userhandle.thread_specific_data.push_back(std::move(thr_data));
    }

    // Setting up callbacks
    CPXLONG callback_contex = CPX_CALLBACKCONTEXT_CANDIDATE;  // The candidate callback is ALWAYS needed (otherwise the model might be incomplete due
                                                              // to missing causal acyclicity)

    // If we have no cust to add -> no callback
    // If we have cuts and we have no cutloop, we need to add the callback, since cuts at root must be done somewhere (wether cuts must be done at
    // nodes or not) -> yes callback
    // If we have cuts and we have cuts at nodes -> yes callback
    if (exec.fract_cuts != "0" && (exec.fract_cuts_at_nodes || !exec.custom_cutloop)) {
        callback_contex |= CPX_CALLBACKCONTEXT_RELAXATION;
    }

    CPX_HANDLE_CALL(CPXcallbacksetfunc(env, lp, callback_contex, callback_hub, &userhandle));
}

inline void gather_stats_from_threads(hplus::statistics& stats, callback_userhandle& thread_data) {
    for (auto& data : thread_data.thread_specific_data) {
        stats.cand_callback += data.cand_time;
        stats.relax_callback += data.relax_time;
        stats.cuts_lm += data.usercuts_lm;
        stats.cuts_sec += data.usercuts_sec;
        stats.cand_calls += data.cand_calls;
        stats.relax_calls += data.relax_calls;
        stats.total_act_in_lm += data.acts_in_lm;
        stats.total_n_lm += data.n_lm;
    }
}
}  // namespace callbacks
