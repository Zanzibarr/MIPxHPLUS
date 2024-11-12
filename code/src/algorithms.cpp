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
    my::assert(!CPXsetlogfilename(env, (HPLUS_CPLEX_OUTPUT_DIR"/log/"+HPLUS_env.run_name+".log").c_str(), "w"), "CPXsetlogfilename failed.");
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
            HPLUS_env.sol_status = my::solution_status::FEAS;
            return true;
        case CPXMIP_TIME_LIM_INFEAS:    // exceeded time limit, no intermediate solution found
            if (!HPLUS_env.warm_start_enabled) HPLUS_env.sol_status = my::solution_status::NOTFOUND;
            return false;
        case CPXMIP_INFEASIBLE:         // proven to be unfeasible
            HPLUS_env.sol_status = my::solution_status::INFEAS;
            return false;
        case CPXMIP_ABORT_FEAS:         // terminated by user, found solution
            HPLUS_env.sol_status = my::solution_status::FEAS;
            return true;
        case CPXMIP_ABORT_INFEAS:       // terminated by user, not found solution
            if (!HPLUS_env.warm_start_enabled) HPLUS_env.sol_status = my::solution_status::NOTFOUND;
            return false;
        case CPXMIP_OPTIMAL_TOL:        // found optimal within the tollerance
            mylog.print_warn("Found optimal within the tolerance.");
            HPLUS_env.sol_status = my::solution_status::OPT;
            return true;
        case CPXMIP_OPTIMAL:            // found optimal
            HPLUS_env.sol_status = my::solution_status::OPT;
            return true;
        case 0:
            HPLUS_env.sol_status = my::solution_status::NOTFOUND;
            return false;
        default:                        // unhandled status
            mylog.raise_error("Error in tsp_cplex: unhandled cplex status: %d.", cpxstatus);
            return false;
    }

}

// ##################################################################### //
// ############################# EXECUTION ############################# //
// ##################################################################### //

void HPLUS_run() {

    if (
        HPLUS_env.alg != HPLUS_CLI_ALG_IMAI &&
        HPLUS_env.alg != HPLUS_CLI_ALG_RANKOOH
    ) mylog.raise_error("The algorithm specified (%s) is not on the list of possible algorithms... Please read the README.md for instructions.", HPLUS_env.alg.c_str());

    // ~~~~~~~~ PROBLEM SIMPLIFICATION ~~~~~~~ //

    if (HPLUS_env.problem_simplification_enabled) {

        mylog.print_info("Problem simplification.");
        HPLUS_env.exec_status = my::execution_status::PROBLEM_SIMPL;
        
        HPLUS_stats.simplification_time = HPLUS_env.time_limit - timer.get_time();
        double start_time = timer.get_time();

        HPLUS_inst.problem_simplification();

        HPLUS_stats.simplification_time = timer.get_time() - start_time;

    }

    // ~~~~~~~~~~~~~~ HEURISTIC ~~~~~~~~~~~~~~ //

    if (HPLUS_env.heuristic_enabled) {

        //[ ]: heuristic
        my::todo("heuristic");

        mylog.print_info("Calculating heuristic solution.");
        HPLUS_env.exec_status = my::execution_status::HEURISTIC;

        HPLUS_stats.heuristic_time = HPLUS_env.time_limit - timer.get_time();
        double start_time = timer.get_time();

        // ...

        HPLUS_env.sol_status = my::solution_status::FEAS;
        HPLUS_stats.heuristic_time = timer.get_time() - start_time;

    }

    // ~~~~~~~~~~~~ MODEL BUILDING ~~~~~~~~~~~ //

    mylog.print_info("Building model.");
    HPLUS_env.exec_status = my::execution_status::MODEL_BUILD;

    HPLUS_stats.build_time = HPLUS_env.time_limit - timer.get_time();
    double start_time = timer.get_time();

    CPXENVptr env = nullptr;
    CPXLPptr lp = nullptr;

    HPLUS_cpx_init(env, lp);

    //[ ]: Slow (idk...)
    if (HPLUS_env.alg == HPLUS_CLI_ALG_IMAI) HPLUS_cpx_build_imai(env, lp);
    else if (HPLUS_env.alg == HPLUS_CLI_ALG_RANKOOH) HPLUS_cpx_build_rankooh(env, lp);

    // time limit
    if ((double)HPLUS_env.time_limit > timer.get_time()) my::assert(!CPXsetdblparam(env, CPXPARAM_TimeLimit, (double)HPLUS_env.time_limit - timer.get_time()), "CPXsetdblparam (CPXPARAM_TimeLimit) failed.");
    else while(true);                   // handling edge case of time limit reached (if we enter the else, at any time the timer thread should terminate the execution, so we wait for him)

    if (HPLUS_env.warm_start_enabled) {     // Post warm starto to CPLEX

        #if HPLUS_INTCHECK
        my::assert(HPLUS_env.sol_status != my::solution_status::INFEAS && HPLUS_env.sol_status != my::solution_status::NOTFOUND, "Warm start failed.");
        #endif
        std::vector<size_t> warm_start;
        unsigned int cost;
        HPLUS_inst.get_best_sol(warm_start, cost);
        int* cpx_sol_ind = new int[HPLUS_inst.get_n_act(true)];
        double* cpx_sol_val = new double[HPLUS_inst.get_n_act(true)];
        int izero = 0;
        int effortlevel = CPX_MIPSTART_REPAIR;
        size_t tmp_cnt = 0;
        for (auto act_i : warm_start) {
            cpx_sol_ind[tmp_cnt] = HPLUS_inst.act_idx_post_simplification(act_i);
            cpx_sol_val[tmp_cnt++] = 1;
        }
        my::assert(!CPXaddmipstarts(env, lp, 1, warm_start.size(), &izero, cpx_sol_ind, cpx_sol_val, &effortlevel, nullptr), "CPXaddmipstarts failed.");
        delete[] cpx_sol_ind; cpx_sol_ind = nullptr;
        delete[] cpx_sol_val; cpx_sol_val = nullptr;

    }

    HPLUS_stats.build_time = timer.get_time() - start_time;

    // ~~~~~~~~~~~ MODEL EXECUTION ~~~~~~~~~~~ //

    mylog.print_info("Running CPLEX.");
    HPLUS_env.exec_status = my::execution_status::CPX_EXECUTION;

    HPLUS_stats.execution_time = HPLUS_env.time_limit - timer.get_time();
    start_time = timer.get_time();

    my::assert(!CPXmipopt(env, lp), "CPXmipopt failed.");

    if (HPLUS_parse_cplex_status(env, lp)) {        // If CPLEX has found a solution
        if (HPLUS_env.alg == HPLUS_CLI_ALG_IMAI) HPLUS_store_imai_sol(env, lp);
        else if (HPLUS_env.alg == HPLUS_CLI_ALG_RANKOOH) HPLUS_store_rankooh_sol(env, lp);
    }

    HPLUS_cpx_close(env, lp);

    HPLUS_stats.execution_time = timer.get_time() - start_time;

}