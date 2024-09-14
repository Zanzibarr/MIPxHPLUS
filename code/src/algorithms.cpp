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

    // time limit
    my::assert(!CPXsetdblparam(env, CPXPARAM_TimeLimit, (double)HPLUS_env.time_limit), "CPXsetdblparam (CPXPARAM_TimeLimit) failed.");

    // terminate condition
    my::assert(!CPXsetterminate(env, &HPLUS_env.cpx_terminate), "CPXsetterminate failed.");

}

void HPLUS_cpx_close(CPXENVptr& env, CPXLPptr& lp) {

    CPXfreeprob(env, &lp);
    CPXcloseCPLEX(&env);

}

void HPLUS_parse_cplex_status(const CPXENVptr& env, const CPXLPptr& lp) {

    switch ( int cpxstatus = CPXgetstat(env, lp) ) {
        case CPXMIP_TIME_LIM_FEAS:      // exceeded time limit, found intermediate solution
            HPLUS_env.status = my::status::TIMEL_FEAS;
            break;
        case CPXMIP_TIME_LIM_INFEAS:    // exceeded time limit, no intermediate solution found
            HPLUS_env.status = my::status::TIMEL_NF;
            break;
        case CPXMIP_INFEASIBLE:         // proven to be unfeasible
            HPLUS_env.status = my::status::INFEAS;
            break;
        case CPXMIP_ABORT_FEAS:         // terminated by user, found solution
            HPLUS_env.status = my::status::USR_STOP_FEAS;
            break;
        case CPXMIP_ABORT_INFEAS:       // terminated by user, not found solution
            HPLUS_env.status = my::status::USR_STOP_NF;
            break;
        case CPXMIP_OPTIMAL_TOL:        // found optimal within the tollerance
            lprint_warn("Found optimal within the tolerance.");
            HPLUS_env.status = my::status::OPT;
            break;
        case CPXMIP_OPTIMAL:            // found optimal
            HPLUS_env.status = my::status::OPT;
            break;
        case 0:
            HPLUS_env.status = my::status::NOTFOUND;
            break;
        default:                        // unhandled status
            HPLUS_env.logger.raise_error("Error in tsp_cplex: unhandled cplex status: %d.", cpxstatus);
            break;
    }

}

// ##################################################################### //
// ############################# EXECUTION ############################# //
// ##################################################################### //

void HPLUS_run(HPLUS_instance& inst) {

    if (HPLUS_env.alg != HPLUS_CLI_IMAI && HPLUS_env.alg != HPLUS_CLI_RANKOOH) HPLUS_env.logger.raise_error("The algorithm specified (%s) is not on the list of possible algorithms... Please read the README.md for instructions.", HPLUS_env.alg.c_str());

    if (HPLUS_env.alg == HPLUS_CLI_IMAI) lprint_info("Running imai algorithm.");
    else if (HPLUS_env.alg == HPLUS_CLI_RANKOOH) lprint_info("Running rankooh algorithm.");

    CPXENVptr env = nullptr;
    CPXLPptr lp = nullptr;

    double start_time = HPLUS_env.get_time();

    HPLUS_cpx_init(env, lp);

    if (HPLUS_env.alg == HPLUS_CLI_IMAI) HPLUS_cpx_build_imai(env, lp, inst);
    else if (HPLUS_env.alg == HPLUS_CLI_RANKOOH) HPLUS_cpx_build_rankooh(env, lp, inst);

    HPLUS_stats.build_time = HPLUS_env.get_time() - start_time;

    start_time = HPLUS_env.get_time();

    my::assert(!CPXmipopt(env, lp), "CPXmipopt failed.");

    HPLUS_stats.exec_time = HPLUS_env.get_time() - start_time;

    HPLUS_parse_cplex_status(env, lp);
    if (HPLUS_env.found()) {
        if (HPLUS_env.alg == HPLUS_CLI_IMAI) HPLUS_store_imai_sol(env, lp, inst);
        else if (HPLUS_env.alg == HPLUS_CLI_RANKOOH) HPLUS_store_rankooh_sol(env, lp, inst);
    }

    HPLUS_cpx_close(env, lp);

    switch(HPLUS_env.status) {
        case my::status::INFEAS:
            lprint("The problem is infeasible.");
            return;
        case my::status::NOTFOUND:
            lprint("No solution found.");
            return;
        case my::status::TIMEL_NF:
            lprint("No solution found due to time limit.");
            return;
        case my::status::USR_STOP_NF:
            lprint("No solution found due to the user terminating the execution.");
            return;
        case my::status::TIMEL_FEAS:
            lprint("The solution is not optimal due to time limit.");
            break;
        case my::status::USR_STOP_FEAS:
            lprint("The solution is not optimal due to the user terminating the execution.");
            break;
        default:
            break;
    }

    inst.print_best_sol();

}