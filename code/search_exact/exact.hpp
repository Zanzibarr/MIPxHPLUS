#ifndef HPLUS_EXACT_HPP
#define HPLUS_EXACT_HPP

#include <cplex.h>

#include <new>
#include <numeric>

#include "../cut_separators/callbacks.hpp"

namespace tl {
void add_acyclicity_constraints(const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats, CPXENVptr& env, CPXLPptr& lp);
void post_warm_start(const hplus::execution& exec, hplus::instance& inst, CPXENVptr& env, CPXLPptr& lp);
}  // namespace tl

namespace ve {
void add_acyclicity_constraints(const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats, CPXENVptr& env, CPXLPptr& lp);
void post_warm_start(const hplus::execution& exec, hplus::instance& inst, CPXENVptr& env, CPXLPptr& lp);
}  // namespace ve

namespace cuts {
void post_warm_start(const hplus::execution& exec, hplus::instance& inst, CPXENVptr& env, CPXLPptr& lp);
}  // namespace cuts

namespace exact {

inline void init_cplex(hplus::execution& exec, CPXENVptr& env, CPXLPptr& lp) {
    if (BASIC_VERBOSE()) LOG_INFO << "Initializing CPLEX environment";

    int cpxerror;
    env = CPXopenCPLEX(&cpxerror);
    CPX_HANDLE_CALL(cpxerror);
    lp = CPXcreateprob(env, &cpxerror, exec.file_name.c_str());
    CPX_HANDLE_CALL(cpxerror);
    // threads
    CPX_HANDLE_CALL(CPXsetintparam(env, CPXPARAM_Threads, exec.threads));
    // log file
    CPX_HANDLE_CALL(CPXsetintparam(env, CPXPARAM_ScreenOutput, HPLUS_DEF_CPX_SCREENOUTPUT));
    CPX_HANDLE_CALL(CPXsetlogfilename(env, (HPLUS_CPLEX_OUTPUT_DIR "/log/" + exec.file_name + ".log").c_str(), "w"));
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
}

void build_base_model(hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats, CPXENVptr& env, CPXLPptr& lp);

inline void add_acyclicity_constraints(const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats, CPXENVptr& env, CPXLPptr& lp) {
    switch (exec.alg) {
        case hplus::algorithm::TL:
            tl::add_acyclicity_constraints(exec, inst, stats, env, lp);
            break;
        case hplus::algorithm::VE:
            ve::add_acyclicity_constraints(exec, inst, stats, env, lp);
            break;
        default:
            LOG_ERROR << "Unhandled algorithm for acyclicity constraints: " << static_cast<int>(exec.alg);
    }
}

inline void post_warm_start(const hplus::execution& exec, hplus::instance& inst, CPXENVptr& env, CPXLPptr& lp) {
    switch (exec.alg) {
        case hplus::algorithm::TL:
            tl::post_warm_start(exec, inst, env, lp);
            break;
        case hplus::algorithm::VE:
            ve::post_warm_start(exec, inst, env, lp);
            break;
        case hplus::algorithm::CUTS:
            cuts::post_warm_start(exec, inst, env, lp);
            break;
        default:
            LOG_ERROR << "Unhandled algorithm for acyclicity constraints: " << static_cast<int>(exec.alg);
    }
}

void get_cplex_solution(hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats, const CPXENVptr& env, const CPXLPptr& lp);

inline void set_cplex_timelimit(hplus::execution& exec, const CPXENVptr& env) {
    if (exec.timelimit > 0 && static_cast<double>(exec.timelimit) > GET_TIME()) {
        CPX_HANDLE_CALL(CPXsetdblparam(env, CPXPARAM_TimeLimit, static_cast<double>(exec.timelimit) - GET_TIME()));
    } else
        throw timelimit_exception("Reached time limit.");
    timelim::cancel_time_limit();
}

inline void close_cplex(CPXENVptr& env, CPXLPptr& lp) {
    CPX_HANDLE_CALL(CPXfreeprob(env, &lp));
    CPX_HANDLE_CALL(CPXcloseCPLEX(&env));
}

inline void exact(hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats) {
    if (BASIC_VERBOSE()) LOG_INFO << "Running exact search algorithm";

    ASSERT(inst.sol_s != hplus::solution_status::INFEAS);

    // ====================================================== //
    // ================= BUILDING THE MODEL ================= //
    // ====================================================== //

    double start_time = GET_TIME();
    CPXENVptr env = nullptr;
    CPXLPptr lp = nullptr;
    callbacks::callback_userhandle callback_userhandle{.exec = exec, .inst = inst, .stats = stats, .thread_specific_data = {}};

    stats.build = static_cast<double>(exec.timelimit) - start_time;
    exec.exec_s = hplus::exec_status::MODEL_BUILD;

    init_cplex(exec, env, lp);

    // ~~~~ BASE MODEL + ACYC. CONSTRAINTS ~~~ //
    build_base_model(exec, inst, stats, env, lp);
    if (exec.alg != hplus::algorithm::CUTS) add_acyclicity_constraints(exec, inst, stats, env, lp);

    // ~~~~~~~~~~~~~~ WARM START ~~~~~~~~~~~~~ //
    if (exec.ws != hplus::warmstart::NONE) post_warm_start(exec, inst, env, lp);

    //  CAND, RELAX AND GLOBAL INFO CALLBACKS  //
    if (exec.alg == hplus::algorithm::CUTS) callbacks::set_cplex_callbacks(exec, inst, callback_userhandle, env, lp);

    // Set time limit
    set_cplex_timelimit(exec, env);

    stats.build = GET_TIME() - start_time;

    // ====================================================== //
    // =================== CPLEX EXECUTION ================== //
    // ====================================================== //

    start_time = GET_TIME();
    stats.cplex_execution = static_cast<double>(exec.timelimit) - start_time;
    exec.exec_s = hplus::exec_status::CPX_EXEC;

    // Run cplex
    if (BASIC_VERBOSE()) LOG_INFO << "Running CPLEX MIP";
    try {
        CPX_HANDLE_CALL(CPXmipopt(env, lp));
    } catch (std::bad_alloc& e) {
        LOG_WARNING << "OUT OF MEMORY";
    }

    // ====================================================== //
    // =============== GATHER INFO AND CLOSING ============== //
    // ====================================================== //

    // There are info to gather or resources to free only if we used the CUTS algorithm -> the other algorithms only readed global informations in the
    // callback
    if (exec.alg == hplus::algorithm::CUTS) callbacks::gather_stats_from_threads(exec, stats, callback_userhandle);
    get_cplex_solution(exec, inst, stats, env, lp);
    close_cplex(env, lp);

    stats.cplex_execution = GET_TIME() - start_time;
}

}  // namespace exact

#endif