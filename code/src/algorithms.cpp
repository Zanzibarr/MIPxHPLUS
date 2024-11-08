#include "../include/algorithms.hpp"
#include <cplex.h>

// ##################################################################### //
// ############################### CPLEX ############################### //
// ##################################################################### //

void HPLUS_cpx_init(CPXENVptr& env, CPXLPptr& lp) {

    int cpxerror;
    env = CPXopenCPLEX(&cpxerror); my::assert(!cpxerror, "CPXopenCPLEX failed.");
    lp = CPXcreateprob(env, &cpxerror, "HPLUS"); my::assert(!cpxerror, "CPXcreateprob failed.");

    // log file
    my::assert(!CPXsetintparam(env, CPXPARAM_ScreenOutput, CPX_OFF), "CPXsetintparam failed (CPXPARAM_ScreenOutput).");
    my::assert(!CPXsetlogfilename(env, (HPLUS_CPLEX_OUT_DIR"/log/"+HPLUS_env.run_name+".log").c_str(), "w"), "CPXsetlogfilename failed.");
    my::assert(!CPXsetintparam(env, CPX_PARAM_CLONELOG, -1), "CPXsetintparam (CPX_PARAM_CLONELOG) failed.");

    // tolerance
    my::assert(!CPXsetdblparam(env, CPXPARAM_MIP_Tolerances_MIPGap, 0), "CPXsetdblparam (CPXPARAM_MIP_Tolerances_MIPGap) failed.");

    // memory/size limits
    my::assert(!CPXsetdblparam(env, CPXPARAM_MIP_Limits_TreeMemory, 12000), "CPXsetdblparam (CPXPARAM_MIP_Limits_TreeMemory) failed.");
    my::assert(!CPXsetdblparam(env, CPXPARAM_WorkMem, 4096), "CPXsetdblparam (CPXPARAM_WorkMem) failed.");
    my::assert(!CPXsetintparam(env, CPXPARAM_MIP_Strategy_File, 3), "CPXsetintparam (CPXPARAM_MIP_Strategy_File) failed.");

    // terminate condition
    my::assert(!CPXsetterminate(env, &HPLUS_env.cpx_terminate), "CPXsetterminate failed.");

}

void HPLUS_cpx_close(CPXENVptr& env, CPXLPptr& lp) {

    CPXfreeprob(env, &lp);
    CPXcloseCPLEX(&env);

}

bool HPLUS_parse_cplex_status(const CPXENVptr& env, const CPXLPptr& lp) {

    switch ( int cpxstatus = CPXgetstat(env, lp) ) {
        case CPXMIP_TIME_LIM_FEAS:      // exceeded time limit, found intermediate solution
            HPLUS_env.status = my::status::FEAS;
            return true;
        case CPXMIP_TIME_LIM_INFEAS:    // exceeded time limit, no intermediate solution found
            if (!HPLUS_env.warm_start) HPLUS_env.status = my::status::NOTFOUND;
            return false;
        case CPXMIP_INFEASIBLE:         // proven to be unfeasible
            HPLUS_env.status = my::status::INFEAS;
            return false;
        case CPXMIP_ABORT_FEAS:         // terminated by user, found solution
            HPLUS_env.status = my::status::FEAS;
            return true;
        case CPXMIP_ABORT_INFEAS:       // terminated by user, not found solution
            if (!HPLUS_env.warm_start) HPLUS_env.status = my::status::NOTFOUND;
            return false;
        case CPXMIP_OPTIMAL_TOL:        // found optimal within the tollerance
            lprint_warn("Found optimal within the tolerance.");
            HPLUS_env.status = my::status::OPT;
            return true;
        case CPXMIP_OPTIMAL:            // found optimal
            HPLUS_env.status = my::status::OPT;
            return true;
        case 0:
            HPLUS_env.status = my::status::NOTFOUND;
            return false;
        default:                        // unhandled status
            HPLUS_env.logger.raise_error("Error in tsp_cplex: unhandled cplex status: %d.", cpxstatus);
            return false;
    }

}

// ##################################################################### //
// ############################# EXECUTION ############################# //
// ##################################################################### //

void HPLUS_run() {

    if (
        HPLUS_env.alg != HPLUS_CLI_IMAI &&
        HPLUS_env.alg != HPLUS_CLI_RANKOOH
    ) HPLUS_env.logger.raise_error("The algorithm specified (%s) is not on the list of possible algorithms... Please read the README.md for instructions.", HPLUS_env.alg.c_str());

    // ~~~~~~~~~~~~~ HEURISTIC 1 ~~~~~~~~~~~~~ //

    if (HPLUS_env.heur_1) HPLUS_inst.initial_heuristic();

    // ~~~~~~~~~~ MODEL OPTIMIZATION ~~~~~~~~~ //

    if (HPLUS_env.model_enhancements) HPLUS_inst.model_optimization();

    // ~~~~~~~~~~~~~ HEURISTIC 2 ~~~~~~~~~~~~~ //

    if (HPLUS_env.heur_2) HPLUS_inst.optimized_heuristic();

    // ~~~~~~~~~~~~ MODEL BUILDING ~~~~~~~~~~~ //

    HPLUS_stats.build_time = HPLUS_env.time_limit - HPLUS_env.get_time();
    double start_time = HPLUS_env.get_time();

    if (HPLUS_env.alg == HPLUS_CLI_IMAI) lprint_info("Running imai algorithm.");
    else if (HPLUS_env.alg == HPLUS_CLI_RANKOOH) lprint_info("Running rankooh algorithm.");

    CPXENVptr env = nullptr;
    CPXLPptr lp = nullptr;

    HPLUS_cpx_init(env, lp);

    //[ ]: Slow (idk...)
    if (HPLUS_env.alg == HPLUS_CLI_IMAI) HPLUS_cpx_build_imai(env, lp, HPLUS_inst);
    else if (HPLUS_env.alg == HPLUS_CLI_RANKOOH) HPLUS_cpx_build_rankooh(env, lp, HPLUS_inst);

    // time limit
    if ((double)HPLUS_env.time_limit > HPLUS_env.get_time()) my::assert(!CPXsetdblparam(env, CPXPARAM_TimeLimit, (double)HPLUS_env.time_limit - HPLUS_env.get_time()), "CPXsetdblparam (CPXPARAM_TimeLimit) failed.");
    else while(true);                   // handling edge case of time limit reached (if we enter the else, at any time the timer thread should terminate the execution, so we wait for him)

    //[ ]: Adding the warm start
    if (HPLUS_env.warm_start) {

    }

    HPLUS_stats.build_time = HPLUS_env.get_time() - start_time;

    // ~~~~~~~~~~~ MODEL EXECUTION ~~~~~~~~~~~ //

    HPLUS_stats.exec_time = HPLUS_env.time_limit - HPLUS_env.get_time();
    start_time = HPLUS_env.get_time();

    HPLUS_env.cplex_running = true;
    my::assert(!CPXmipopt(env, lp), "CPXmipopt failed.");

    if (HPLUS_parse_cplex_status(env, lp)) {        // If CPLEX has found a solution
        if (HPLUS_env.alg == HPLUS_CLI_IMAI) HPLUS_store_imai_sol(env, lp, HPLUS_inst);
        else if (HPLUS_env.alg == HPLUS_CLI_RANKOOH) HPLUS_store_rankooh_sol(env, lp, HPLUS_inst);
    }

    HPLUS_cpx_close(env, lp);

    HPLUS_stats.exec_time = HPLUS_env.get_time() - start_time;

}