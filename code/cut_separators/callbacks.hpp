/**
 * Methods for CPLEX generic callbacks
 * Extends to cand_callback.hpp for the methods used for CPLEX's candidate callback, relax_callback.hpp for the methods used for CPLEX's
 * relaxation callback and progress_callback.hpp for global progress stats gathering
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#pragma once

#include <cplex.h>

#include "cand_callback.hpp"
#include "execution.hpp"
#include "progress_callback.hpp"
#include "relax_callback.hpp"
#include "utils.hpp"

namespace callbacks {

struct callback_userhandle {
    hplus::execution& exec;
    hplus::instance& inst;
    hplus::statistics& stats;
};

/// This methods handles the routing of the callbacks based on the context
static auto CPXPUBLIC callback_hub(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void* userhandle) -> int {
    auto& [exec, inst, stats]{*static_cast<callback_userhandle*>(userhandle)};
    int thread_id{-1};
    CPX_HANDLE_CALL(CPXcallbackgetinfoint(context, CPXCALLBACKINFO_THREADID, &thread_id));

    switch (contextid) {
        case CPX_CALLBACKCONTEXT_GLOBAL_PROGRESS:
            progress_callback(context, exec, inst);
            break;
        case CPX_CALLBACKCONTEXT_RELAXATION:
            relaxation_callback(context, exec, inst);
            break;
        case CPX_CALLBACKCONTEXT_CANDIDATE:
            candidate_callback(context, exec, inst, stats);
            break;
        default:
            LOG_ERROR_S("Unhandled CPLEX callback context: " + std::to_string(contextid));
    }

    return 0;
}

/// Set the chosen callback context for CPLEX
inline void set_cplex_callbacks(const hplus::execution& exec, callback_userhandle& userhandle, CPXENVptr& env, CPXLPptr& lp) {
    LOG_INFO_S("Setting up CPLEX callbacks");

    // Setting up callbacks
    // Fix[Issue#3] : Use global progress callback to gather global info about the solution process
    CPXLONG callback_context = CPX_CALLBACKCONTEXT_GLOBAL_PROGRESS;

    // This function is used not only for the CUTS algorithm now, so if TL or VE wants to strenghten the lb with fractional cuts it can
    if (exec.cand_cuts != "0") {
        callback_context |= CPX_CALLBACKCONTEXT_CANDIDATE;
    }

    // The relaxation callback is used to gather info about the lb relaxation, so it's always enabled
    callback_context |= CPX_CALLBACKCONTEXT_RELAXATION;

    CPX_HANDLE_CALL(CPXcallbacksetfunc(env, lp, callback_context, callback_hub, &userhandle));
}

}  // namespace callbacks
