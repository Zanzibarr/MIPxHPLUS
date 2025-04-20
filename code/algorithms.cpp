#include "algorithms.hpp"

// ##################################################################### //
// ############################ CPLEX UTILS ############################ //
// ##################################################################### //

void cpx_init(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::environment& env, const logger& log, bool log_file = true) {
    PRINT_VERBOSE(log, "Initializing CPLEX.");
    int cpxerror;
    cpxenv = CPXopenCPLEX(&cpxerror);
    CPX_HANDLE_CALL(log, cpxerror);
    cpxlp = CPXcreateprob(cpxenv, &cpxerror, env.run_name.c_str());
    CPX_HANDLE_CALL(log, cpxerror);
    // log file
    CPX_HANDLE_CALL(log, CPXsetintparam(cpxenv, CPXPARAM_ScreenOutput, HPLUS_DEF_CPX_SCREENOUTPUT));
    if (log_file) CPX_HANDLE_CALL(log, CPXsetlogfilename(cpxenv, (HPLUS_CPLEX_OUTPUT_DIR "/log/" + env.run_name + ".log").c_str(), "w"));
    CPX_HANDLE_CALL(log, CPXsetintparam(cpxenv, CPX_PARAM_CLONELOG, HPLUS_DEF_CPX_CLONELOG));
    CPX_HANDLE_CALL(log, CPXsetintparam(cpxenv, CPXPARAM_MIP_Display, HPLUS_DEF_CPX_MIP_DISPLAY));
    // tolerance
    CPX_HANDLE_CALL(log, CPXsetdblparam(cpxenv, CPXPARAM_MIP_Tolerances_MIPGap, HPLUS_DEF_CPX_TOL_GAP));
    // memory/size limits
    CPX_HANDLE_CALL(log, CPXsetdblparam(cpxenv, CPXPARAM_MIP_Limits_TreeMemory, HPLUS_DEF_CPX_TREE_MEM));
    CPX_HANDLE_CALL(log, CPXsetdblparam(cpxenv, CPXPARAM_WorkMem, HPLUS_DEF_CPX_WORK_MEM));
    CPX_HANDLE_CALL(log, CPXsetintparam(cpxenv, CPXPARAM_MIP_Strategy_File, HPLUS_DEF_CPX_STRAT_FILE));
    // terminate condition
    CPX_HANDLE_CALL(log, CPXsetterminate(cpxenv, &global_terminate));
}

void cpx_close(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const logger& log) {
    CPX_HANDLE_CALL(log, CPXfreeprob(cpxenv, &cpxlp));
    CPX_HANDLE_CALL(log, CPXcloseCPLEX(&cpxenv));
}

[[nodiscard]]
bool parse_cpx_status(const CPXENVptr& cpxenv, const CPXLPptr& cpxlp, hplus::environment& env, const logger& log) {
    PRINT_VERBOSE(log, "Parsing CPLEX status code.");
    switch (const int cpxstatus{CPXgetstat(cpxenv, cpxlp)}) {
        case CPXMIP_TIME_LIM_FEAS:  // exceeded time limit, found intermediate solution
            [[fallthrough]];
        case CPXMIP_ABORT_FEAS:  // terminated by user, found solution
            env.sol_s = solution_status::FEAS;
            return true;
        case CPXMIP_MEM_LIM_INFEAS:  // exceeded memory limit, no intermediate solution found
            [[fallthrough]];
        case CPXMIP_MEM_LIM_FEAS:  // exceeded memory limit, found intermediate solution
            log.raise_error("OUT OF MEMORY");
            return false;
        case CPXMIP_TIME_LIM_INFEAS:  // exceeded time limit, no intermediate solution found
            [[fallthrough]];
        case CPXMIP_ABORT_INFEAS:  // terminated by user, not found solution
            if (!env.warm_start) env.sol_s = solution_status::NOTFOUND;
            return false;
        case CPXMIP_INFEASIBLE:  // proven to be infeasible
            env.sol_s = solution_status::INFEAS;
            return false;
        case CPXMIP_OPTIMAL_TOL:  // found optimal within the tollerance
            PRINT_WARN(log, "Found optimal within the tolerance.");
            [[fallthrough]];
        case CPXMIP_OPTIMAL:  // found optimal
            env.sol_s = solution_status::OPT;
            return true;
        default:  // unhandled status
            log.raise_error("Error in parse_cpx_status: unhandled cplex status: %d.", cpxstatus);
            return false;
    }
}

// ##################################################################### //
// ######################### ALGORITHM HANDLERS ######################## //
// ##################################################################### //

void run_heur(hplus::instance& inst, hplus::environment& env, const logger& log) {
    srand(time(nullptr));
    if (env.heur == HPLUS_CLI_HEUR_GREEDYCOST)
        greedycost::run(inst, env, log);
    else if (env.heur == HPLUS_CLI_HEUR_GREEDYCXE)
        greedycxe::run(inst, env, log);
    else if (env.heur == HPLUS_CLI_HEUR_GREEDYHMAX)
        hmax::run(inst, env, log);
    else if (env.heur == HPLUS_CLI_HEUR_GREEDYHADD)
        hadd::run(inst, env, log);
    else
        log.raise_error("The heuristic specified (%s) is not on the list of possible heuristics... Please use the --h flag for instructions.",
                        env.heur.c_str());
}

void run_model(hplus::instance& inst, hplus::environment& env, hplus::statistics& stats, const logger& log) {
    auto stopchk = [&env]() {
        if (CHECK_STOP()) {
            env.exec_s = exec_status::STOP_TL;
            throw timelimit_exception("Reached time limit.");
        }
    };

    // ~~~~~~~~~~~~ MODEL BUILDING ~~~~~~~~~~~ //
    env.exec_s = exec_status::MODEL_BUILD;

    stats.build = static_cast<double>(env.time_limit) - env.timer.get_time();
    double start_time = env.timer.get_time();

    CPXENVptr cpxenv = nullptr;
    CPXLPptr cpxlp = nullptr;

    cpx_init(cpxenv, cpxlp, env, log);
    stopchk();

    if (env.alg == HPLUS_CLI_ALG_TL)
        tl::build_cpx_model(cpxenv, cpxlp, inst, env, log, stats);
    else if (env.alg == HPLUS_CLI_ALG_VE)
        ve::build_cpx_model(cpxenv, cpxlp, inst, env, log, stats);
    else if (env.alg == HPLUS_CLI_ALG_LM)
        lm::build_cpx_model(cpxenv, cpxlp, inst, env, log, stats);
    stopchk();

    // time limit
    if (static_cast<double>(env.time_limit) > env.timer.get_time()) {
        CPX_HANDLE_CALL(log, CPXsetdblparam(cpxenv, CPXPARAM_TimeLimit, static_cast<double>(env.time_limit) - env.timer.get_time()));
    } else
        throw timelimit_exception("Reached the time limit");

    if (env.warm_start) {  // Post warm starto to CPLEX

        if (env.alg == HPLUS_CLI_ALG_TL)
            tl::post_cpx_warmstart(cpxenv, cpxlp, inst, env, log);
        else if (env.alg == HPLUS_CLI_ALG_VE)
            ve::post_cpx_warmstart(cpxenv, cpxlp, inst, env, log);
        else if (env.alg == HPLUS_CLI_ALG_LM)
            lm::post_cpx_warmstart(cpxenv, cpxlp, inst, env, log);
    }

    stats.nusercuts = 0;
    lm::cpx_callback_user_handle callback_data{.inst = inst, .env = env, .stats = stats, .log = log};
    if (env.alg == HPLUS_CLI_ALG_LM)
        CPX_HANDLE_CALL(log, CPXcallbacksetfunc(cpxenv, cpxlp, CPX_CALLBACKCONTEXT_CANDIDATE, lm::cpx_callback, &callback_data));

    stats.build = env.timer.get_time() - start_time;

    // ~~~~~~~~~~~ MODEL EXECUTION ~~~~~~~~~~~ //

    PRINT_INFO(log, "Running CPLEX.");
    env.exec_s = exec_status::CPX_EXEC;

    stats.execution = static_cast<double>(env.time_limit) - env.timer.get_time();
    start_time = env.timer.get_time();

    CPX_HANDLE_CALL(log, CPXmipopt(cpxenv, cpxlp));

    stats.nnodes = CPXgetnodecnt(cpxenv, cpxlp);

    if (parse_cpx_status(cpxenv, cpxlp, env, log)) {  // If CPLEX has found a solution
        CPX_HANDLE_CALL(log, CPXgetbestobjval(cpxenv, cpxlp, &stats.lb));
        if (env.alg == HPLUS_CLI_ALG_TL)
            tl::store_cpx_sol(cpxenv, cpxlp, inst, log);
        else if (env.alg == HPLUS_CLI_ALG_VE)
            ve::store_cpx_sol(cpxenv, cpxlp, inst, log);
        else if (env.alg == HPLUS_CLI_ALG_LM)
            lm::store_cpx_sol(cpxenv, cpxlp, inst, log);
    }

    cpx_close(cpxenv, cpxlp, log);

    stats.execution = env.timer.get_time() - start_time;
}
