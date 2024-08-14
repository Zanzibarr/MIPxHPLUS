#include "../include/algorithms.hpp"
#include "../include/imai_model.hpp"

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
            HPLUS_env.logger.print_warn("Found optimal within the tolerance.");
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

// ~~~~~~~~~~~~~~~~~ IMAI ~~~~~~~~~~~~~~~~ //

void HPLUS_run_imai(HPLUS_instance& inst) {

    HPLUS_env.logger.print_info("Running imai algorithm.");

    CPXENVptr env = nullptr;
    CPXLPptr lp = nullptr;

    double start_time = HPLUS_env.get_time();

    HPLUS_cpx_init(env, lp);
    HPLUS_cpx_build_imai(env, lp, inst);

    HPLUS_stats.build_time = HPLUS_env.get_time() - start_time;

    start_time = HPLUS_env.get_time();

    my::assert(!CPXmipopt(env, lp), "CPXmipopt failed.");

    HPLUS_stats.exec_time = HPLUS_env.get_time() - start_time;

    HPLUS_parse_cplex_status(env, lp);

    if (HPLUS_env.found()) HPLUS_store_imai_sol(env, lp, inst);

    HPLUS_cpx_close(env, lp);

}