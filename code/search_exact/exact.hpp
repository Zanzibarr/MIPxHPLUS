/**
 * Methods used to compute the exact h+ solution
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#pragma once

#include <cplex.h>

#include <new>

#include "callbacks.hpp"
#include "limits.hxx"
#include "timer.hxx"

namespace tl {
void add_acyclicity_constraints(hplus::instance& inst, CPXENVptr& env, CPXLPptr& lp);
void post_warm_start(hplus::instance& inst, CPXENVptr& env, CPXLPptr& lp);
}  // namespace tl

namespace ve {
void add_acyclicity_constraints(hplus::instance& inst, CPXENVptr& env, CPXLPptr& lp);
void post_warm_start(hplus::instance& inst, CPXENVptr& env, CPXLPptr& lp);
}  // namespace ve

namespace cuts {
void post_warm_start(hplus::instance& inst, CPXENVptr& env, CPXLPptr& lp);
}  // namespace cuts

namespace cutloop {
void cutloop(CPXENVptr& env, CPXLPptr& lp, hplus::execution& exec, const hplus::instance& inst, hplus::statistics& stats);
}

namespace exact {

inline void init_cplex(hplus::execution& exec, CPXENVptr& env, CPXLPptr& lp) {
    LOG_INFO_S("Initializing CPLEX environment");

    int cpxerror = 0;
    env = CPXopenCPLEX(&cpxerror);
    CPX_HANDLE_CALL(cpxerror);
    lp = CPXcreateprob(env, &cpxerror, exec.file_name.c_str());
    CPX_HANDLE_CALL(cpxerror);
    // threads
    CPX_HANDLE_CALL(CPXsetintparam(env, CPXPARAM_Threads, static_cast<CPXINT>(exec.threads)));
    CPX_HANDLE_CALL(CPXsetintparam(env, CPXPARAM_Parallel, CPX_PARALLEL_DETERMINISTIC));
    // log file
    CPX_HANDLE_CALL(CPXsetintparam(env, CPXPARAM_ScreenOutput, HPLUS_DEF_CPX_SCREENOUTPUT));
    CPX_HANDLE_CALL(CPXsetlogfilename(env, (exec.cplex_log).c_str(), "w"));
    CPX_HANDLE_CALL(CPXsetintparam(env, CPX_PARAM_CLONELOG, HPLUS_DEF_CPX_CLONELOG));
    CPX_HANDLE_CALL(CPXsetintparam(env, CPXPARAM_MIP_Display, HPLUS_DEF_CPX_MIP_DISPLAY));
    // tolerance
    CPX_HANDLE_CALL(CPXsetdblparam(env, CPXPARAM_MIP_Tolerances_MIPGap, HPLUS_DEF_CPX_TOL_GAP));
    // memory/size limits
    CPX_HANDLE_CALL(CPXsetdblparam(env, CPXPARAM_MIP_Limits_TreeMemory, HPLUS_DEF_CPX_TREE_MEM));
    CPX_HANDLE_CALL(CPXsetdblparam(env, CPXPARAM_WorkMem, HPLUS_DEF_WORKMEM));
    CPX_HANDLE_CALL(CPXsetintparam(env, CPXPARAM_MIP_Strategy_File, HPLUS_DEF_CPX_STRAT_FILE));
    // terminate condition
    CPX_HANDLE_CALL(CPXsetterminate(env, &GLOBAL_TERMINATE_CONDITION));
    // random seed
    CPX_HANDLE_CALL(CPXsetintparam(env, CPXPARAM_RandomSeed, exec.seed));
}

void build_base_model(hplus::execution& exec, hplus::instance& inst, CPXENVptr& env, CPXLPptr& lp);

inline void add_acyclicity_constraints(const hplus::execution& exec, hplus::instance& inst, CPXENVptr& env, CPXLPptr& lp) {
    switch (exec.alg) {
        case hplus::algorithm::TL:
            tl::add_acyclicity_constraints(inst, env, lp);
            break;
        case hplus::algorithm::VE:
            ve::add_acyclicity_constraints(inst, env, lp);
            break;
        default:
            LOG_ERROR_S("Unhandled algorithm for acyclicity constraints: " + std::to_string(static_cast<int>(exec.alg)));
    }
}

inline void post_warm_start(const hplus::execution& exec, hplus::instance& inst, CPXENVptr& env, CPXLPptr& lp) {
    if (exec.cutoff >= 0) {
        LOG_INFO_S("Setting cutoff value of " + std::to_string(exec.cutoff));
        CPX_HANDLE_CALL(CPXsetdblparam(env, CPXPARAM_MIP_Tolerances_UpperCutoff, exec.cutoff));
    }

    switch (exec.alg) {
        case hplus::algorithm::TL:
            tl::post_warm_start(inst, env, lp);
            break;
        case hplus::algorithm::VE:
            ve::post_warm_start(inst, env, lp);
            break;
        case hplus::algorithm::CUTS:
            cuts::post_warm_start(inst, env, lp);
            break;
        default:
            LOG_ERROR_S("Unhandled algorithm for acyclicity constraints: " + std::to_string(static_cast<int>(exec.alg)));
    }
}

void get_cplex_solution(hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats, const CPXENVptr& env, const CPXLPptr& lp);

inline void set_cplex_timelimit(hplus::execution& exec, const CPXENVptr& env) {
    if (exec.timelimit > 0 && static_cast<double>(exec.timelimit) > GET_TIME()) {
        CPX_HANDLE_CALL(CPXsetdblparam(env, CPXPARAM_TimeLimit, static_cast<double>(exec.timelimit) - GET_TIME()));
    } else {
        throw timelimit_exception("Reached time limit.");
    }
    timelim::cancel_time_limit();
}

inline void close_cplex(CPXENVptr& env, CPXLPptr& lp) {
    CPX_HANDLE_CALL(CPXfreeprob(env, &lp));
    CPX_HANDLE_CALL(CPXcloseCPLEX(&env));
}

inline void run_cplex(CPXENVptr& env, CPXLPptr& lp, hplus::execution& exec) {
    exec.exec_s = hplus::exec_status::CPX_EXEC;
    auto _cpx_exec = make_scoped_timer<"cpx_execution">(STATS);

    LOG_INFO_S("Running CPLEX MIP");

    CPX_HANDLE_CALL(CPXmipopt(env, lp));
}

inline void exact(hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats) {
    LOG_INFO_S("Running exact search algorithm");

    ASSERT(inst.sol_s != hplus::solution_status::INFEAS);

    // ====================================================== //
    // ================= BUILDING THE MODEL ================= //
    // ====================================================== //

    CPXENVptr env = nullptr;
    CPXLPptr lp = nullptr;
    callbacks::callback_userhandle callback_userhandle{.exec = exec, .inst = inst, .stats = stats};

    exec.exec_s = hplus::exec_status::MODEL_BUILD;
    {
        auto _build = make_scoped_timer<"build">(STATS);
        init_cplex(exec, env, lp);

        // ~~~~ BASE MODEL + ACYC. CONSTRAINTS ~~~ //
        build_base_model(exec, inst, env, lp);
        if (exec.alg != hplus::algorithm::CUTS) {
            add_acyclicity_constraints(exec, inst, env, lp);
        }
    }

    // ====================================================== //
    // =================== CPLEX EXECUTION ================== //
    // ====================================================== //

    set_cplex_timelimit(exec, env);

    // Run cplex
    try {
        if (exec.custom_cutloop) {
            cutloop::cutloop(env, lp, exec, inst, stats);
        }
        if (exec.cand_cuts != "0" || (exec.fract_cuts != "0" && (!exec.custom_cutloop || exec.fract_cuts_at_nodes))) {
            callbacks::set_cplex_callbacks(exec, callback_userhandle, env, lp);
        }
        if (exec.ws != hplus::warmstart::NONE) {
            post_warm_start(exec, inst, env, lp);
        }

        run_cplex(env, lp, exec);

    } catch (std::bad_alloc& e) {
        LOG_WARN_S("OUT OF MEMORY");
    }

    // ====================================================== //
    // =============== GATHER INFO AND CLOSING ============== //
    // ====================================================== //

    // There are info to gather or resources to free only if we used the CUTS algorithm
    get_cplex_solution(exec, inst, stats, env, lp);
    close_cplex(env, lp);
}

}  // namespace exact
