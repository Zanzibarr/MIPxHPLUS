/**
 * Methods used to compute the exact h+ solution
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#pragma once

#include <cplex.h>

#include <algorithm>
#include <new>
#include <numeric>
#include <vector>

#include "callbacks.hpp"
#include "execution.hpp"
#include "instance.hpp"
#include "limits.hxx"
#include "scope_guard.hxx"
#include "timer.hxx"
#include "utils.hpp"

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

inline void init_cplex(const hplus::execution& exec, CPXENVptr& env, CPXLPptr& lp) {
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

void build_base_model(const hplus::execution& exec, hplus::instance& inst, CPXENVptr& env, CPXLPptr& lp);

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

void get_cplex_solution(const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats, const CPXENVptr& env, const CPXLPptr& lp);

inline void set_cplex_timelimit(const hplus::execution& exec, const CPXENVptr& env) {
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

// Solves the LP relaxation and updates stats.lower_bound
inline void solve_lp_relaxation(const hplus::execution& exec, CPXENVptr& env, CPXLPptr& lp, hplus::statistics& stats) {
    if (exec.timelimit > 0 && static_cast<double>(exec.timelimit) > GET_TIME()) {
        CPX_HANDLE_CALL(CPXsetdblparam(env, CPXPARAM_TimeLimit, static_cast<double>(exec.timelimit) - GET_TIME()));
    } else {
        throw timelimit_exception("Reached time limit.");
    }

    CPX_HANDLE_CALL(CPXlpopt(env, lp));

    double lb = 0;
    switch (const int status{CPXgetstat(env, lp)}) {
        case CPX_STAT_OPTIMAL:
            CPX_HANDLE_CALL(CPXgetobjval(env, lp, &lb));
            stats.lower_bound = std::max(stats.lower_bound, lb);
            break;
        case CPX_STAT_ABORT_TIME_LIM:
            [[fallthrough]];
        case CPX_STAT_ABORT_USER:
            break;
        default:
            LOG_ERROR_S("Unhandled CPLEX status in solve_lp_relaxation: " + std::to_string(status));
    }
}

// Restores the problem type from LP back to MILP (all variables binary)
inline void restore_milp(CPXENVptr& env, CPXLPptr& lp) {
    CPX_HANDLE_CALL(CPXchgprobtype(env, lp, CPXPROB_MILP));
    const int ncols = CPXgetnumcols(env, lp);
    std::vector<int> ind(static_cast<unsigned int>(ncols));
    std::vector<char> types(static_cast<unsigned int>(ncols), 'B');
    std::iota(ind.begin(), ind.end(), 0);
    CPX_HANDLE_CALL(CPXchgctype(env, lp, ncols, ind.data(), types.data()));
}

inline void run_cplex(hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats, CPXENVptr& env, CPXLPptr& lp,
                      callbacks::callback_userhandle& callback_userhandle) {
    auto _cpx_exec = make_scoped_timer<"cpx_execution">(STATS);
    exec.exec_s = hplus::exec_status::CPX_EXEC;

    if (exec.custom_cutloop) {
        try {
            cutloop::cutloop(env, lp, exec, inst, stats);
        } catch (timelimit_exception&) {
            callbacks::ensure_lb_rootnode(stats.lower_bound);
            throw;
        }
    }

    if (stats.lower_bound >= stats.cost - HPLUS_EPSILON) {
        stats.lower_bound = static_cast<double>(stats.cost);  // Fix possible precision errors
        stats.status = HPLUS_STATUS_OPT;
        inst.sol_s = hplus::solution_status::OPT;
        if (!exec.custom_cutloop) {
            STATS.gauge_record<"lb_relaxation">(stats.lower_bound);
        }
        callbacks::ensure_lb_rootnode(stats.lower_bound);
        return;
    }

    // ~~~ Full MIP ~~~

    callbacks::set_cplex_callbacks(exec, callback_userhandle, env, lp);
    try {
        set_cplex_timelimit(exec, env);
        // ! IMPORTANT : From this point onward, CHECK_STOP() and GLOBAL_TERMINATE_CONDITION are always returning true... the timelimit check is
        // completely handled by CPLEX, so avoid using them as timelimit-breaching checks
    } catch (timelimit_exception&) {
        if (!exec.custom_cutloop) {
            STATS.gauge_record<"lb_relaxation">(stats.lower_bound);
        }
        callbacks::ensure_lb_rootnode(stats.lower_bound);
        throw;
    }

    LOG_INFO_S("Running CPLEX MIP");
    CPX_HANDLE_CALL(CPXmipopt(env, lp));

    get_cplex_solution(exec, inst, stats, env, lp);

    // CHECK_STOP() might have fired
    if (GET_TIME() > exec.timelimit) {
        throw timelimit_exception("Reached time limit.");
    }
}

inline void exact(hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats) {
    LOG_INFO_S("Running exact search algorithm");

    ASSERT(inst.sol_s != hplus::solution_status::INFEAS);

    // ====================================================== //
    // ================= BUILDING THE MODEL ================= //
    // ====================================================== //

    CPXENVptr env = nullptr;
    CPXLPptr lp = nullptr;
    auto _build_cleanup = on_scope_exit([&] { close_cplex(env, lp); });
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

    // Post warm start before root node solve so CPLEX can use the known upper bound as a cutoff during root processing
    if (exec.ws != hplus::warmstart::NONE) {
        post_warm_start(exec, inst, env, lp);
    }

    // Run cplex
    try {
        run_cplex(exec, inst, stats, env, lp, callback_userhandle);
    } catch (std::bad_alloc&) {
        LOG_WARN_S("OUT OF MEMORY");
    } catch (timelimit_exception&) {
        LOG_WARN_S("OUT OF TIME");
    }
}

}  // namespace exact
