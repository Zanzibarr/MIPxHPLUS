/**
 * @file algorithms.cpp
 * @brief Algorithms to solve the delete-free relaxation of the planning task
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

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
    CPX_HANDLE_CALL(log, CPXsetintparam(cpxenv, CPXPARAM_ScreenOutput, CPX_OFF));
    if (log_file) CPX_HANDLE_CALL(log, CPXsetlogfilename(cpxenv, (HPLUS_CPLEX_OUTPUT_DIR "/log/" + env.run_name + ".log").c_str(), "w"));
    CPX_HANDLE_CALL(log, CPXsetintparam(cpxenv, CPX_PARAM_CLONELOG, -1));
    CPX_HANDLE_CALL(log, CPXsetintparam(cpxenv, CPXPARAM_MIP_Display, 3));
    // CPX_HANDLE_CALL(log, CPXsetintparam(cpxenv, CPXPARAM_MIP_Limits_Solutions, 1));
    // tolerance
    CPX_HANDLE_CALL(log, CPXsetdblparam(cpxenv, CPXPARAM_MIP_Tolerances_MIPGap, 0));
    // memory/size limits
    CPX_HANDLE_CALL(log, CPXsetdblparam(cpxenv, CPXPARAM_MIP_Limits_TreeMemory, 12000));
    CPX_HANDLE_CALL(log, CPXsetdblparam(cpxenv, CPXPARAM_WorkMem, 4096));
    CPX_HANDLE_CALL(log, CPXsetintparam(cpxenv, CPXPARAM_MIP_Strategy_File, 3));
    // terminate condition
    CPX_HANDLE_CALL(log, CPXsetterminate(cpxenv, &global_terminate));
}

void cpx_close(CPXENVptr& cpxenv, CPXLPptr& cpxlp) {
    CPXfreeprob(cpxenv, &cpxlp);
    CPXcloseCPLEX(&cpxenv);
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
    if (env.heur == "greedycost")
        greedycost::run(inst, env, log);
    else if (env.heur == "greedycxe")
        greedycxe::run(inst, env, log);
    else if (env.heur == "hmax")
        hmax::run(inst, env, log);
    else if (env.heur == "hadd")
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
    PRINT_INFO(log, "Building model.");
    env.exec_s = exec_status::MODEL_BUILD;

    stats.build = static_cast<double>(env.time_limit) - env.timer.get_time();
    double start_time = env.timer.get_time();

    CPXENVptr cpxenv = nullptr;
    CPXLPptr cpxlp = nullptr;

    cpx_init(cpxenv, cpxlp, env, log);
    stopchk();

    if (env.alg == HPLUS_CLI_ALG_IMAI)
        imai::build_cpx_model(cpxenv, cpxlp, inst, env, log);
    else if (env.alg == HPLUS_CLI_ALG_RANKOOH)
        rankooh::build_cpx_model(cpxenv, cpxlp, inst, env, log);
    else if (env.alg == HPLUS_CLI_ALG_DYNAMIC_TIME)
        rankooh_dynamic::build_cpx_model(cpxenv, cpxlp, inst, env, log);
    stopchk();

    // time limit
    if (static_cast<double>(env.time_limit) > env.timer.get_time()) {
        CPX_HANDLE_CALL(log, CPXsetdblparam(cpxenv, CPXPARAM_TimeLimit, static_cast<double>(env.time_limit) - env.timer.get_time()));
    } else
        throw timelimit_exception("Reached the time limit");

    if (env.warm_start) {  // Post warm starto to CPLEX

        PRINT_INFO(log, "Posting warm start.");

        if (env.alg == HPLUS_CLI_ALG_IMAI)
            imai::post_cpx_warmstart(cpxenv, cpxlp, inst, env, log);
        else if (env.alg == HPLUS_CLI_ALG_RANKOOH)
            rankooh::post_cpx_warmstart(cpxenv, cpxlp, inst, env, log);
        else if (env.alg == HPLUS_CLI_ALG_DYNAMIC_TIME)
            rankooh_dynamic::post_cpx_warmstart(cpxenv, cpxlp, inst, env, log);
    }

    // TODO: Callback for dynamic model

    stats.build = env.timer.get_time() - start_time;

    // ~~~~~~~~~~~ MODEL EXECUTION ~~~~~~~~~~~ //

    PRINT_INFO(log, "Running CPLEX.");
    env.exec_s = exec_status::CPX_EXEC;

    stats.execution = static_cast<double>(env.time_limit) - env.timer.get_time();
    start_time = env.timer.get_time();

    CPX_HANDLE_CALL(log, CPXmipopt(cpxenv, cpxlp));

    if (parse_cpx_status(cpxenv, cpxlp, env, log)) {  // If CPLEX has found a solution
        if (env.alg == HPLUS_CLI_ALG_IMAI)
            imai::store_cpx_sol(cpxenv, cpxlp, inst, log);
        else if (env.alg == HPLUS_CLI_ALG_RANKOOH)
            rankooh::store_cpx_sol(cpxenv, cpxlp, inst, log);
        else if (env.alg == HPLUS_CLI_ALG_DYNAMIC_TIME)
            rankooh_dynamic::store_cpx_sol(cpxenv, cpxlp, inst, log);
    }

    cpx_close(cpxenv, cpxlp);

    stats.execution = env.timer.get_time() - start_time;
}
