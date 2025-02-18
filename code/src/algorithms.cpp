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

// std::vector<size_t> heur_solution;
// unsigned int heur_cost = 0;
void greedy_heur_cost(std::vector<size_t>& heur_solution, unsigned int& heur_cost) {

    // TODO

}

void greedy_heur_costxeff(std::vector<size_t>& heur_solution, unsigned int& heur_cost) {

    // TODO

}

void rand_heur(std::vector<size_t>& heur_solution, unsigned int& heur_cost) {

    const auto& actions = HPLUS_inst.get_actions();
    my::binary_set remaining_actions(HPLUS_inst.get_n_act(), true);

    heur_cost = 0;
    
    // TODO: Optimize
    my::binary_set state(HPLUS_inst.get_n_var());
    while (!(state.contains(HPLUS_inst.get_goal_state()))) {
        for (auto act_i : remaining_actions) {
            if (state.contains(actions[act_i].get_pre())) {
                heur_solution.push_back(act_i);
                heur_cost += actions[act_i].get_cost();
                remaining_actions.remove(act_i);
                state |= actions[act_i].get_eff();
            }
        }
        mylog.print_info("%d", heur_solution.size());
    }

}

void rand_heur_repeat(std::vector<size_t>& heur_solution, unsigned int& heur_cost) {
    
    // TODO: Choose on CLI
    size_t repetitions = 10;
    std::vector<size_t> random_solution[repetitions];
    unsigned int random_costs[repetitions];

    // TODO: Threads
    for (size_t i = 0; i < repetitions; i++) rand_heur(random_solution[i], random_costs[i]);

    size_t best_sol = 0;
    unsigned int best_cost = random_costs[0];
    for (size_t i = 1; i < repetitions; i++) if (random_costs[i] < best_cost) {
        best_sol = i;
        best_cost = random_costs[i];
    }

    heur_solution = random_solution[best_sol];
    heur_cost = best_cost;

}

void hmax_heur(std::vector<size_t>& heur_solution, unsigned int& heur_cost) {

    // TODO

}

void hadd_heur(std::vector<size_t>& heur_solution, unsigned int& heur_cost) {

    // TODO

}

void linrel_heur(std::vector<size_t>& heur_solution, unsigned int& heur_cost) {

    // TODO

}

void local_search_heur(void (*initial_heur_function)(std::vector<size_t>&, unsigned int&), std::vector<size_t>& heur_solution, unsigned int& heur_cost) {

    initial_heur_function(heur_solution, heur_cost);

    // TODO

}

void HPLUS_find_heuristic() {

    std::vector<size_t> heur_solution;
    unsigned int heur_cost;

    rand_heur(heur_solution, heur_cost);
    if (HPLUS_env.sol_status == my::solution_status::INFEAS) return;

    HPLUS_inst.update_best_sol(heur_solution, heur_cost);
    HPLUS_env.sol_status = my::solution_status::FEAS;

}

void HPLUS_run() {

    if (
        HPLUS_env.alg != HPLUS_CLI_ALG_IMAI &&
        HPLUS_env.alg != HPLUS_CLI_ALG_RANKOOH && 
        HPLUS_env.alg != HPLUS_CLI_ALG_DYNAMIC_SMALL &&
        HPLUS_env.alg != HPLUS_CLI_ALG_DYNAMIC_LARGE &&
        HPLUS_env.alg != HPLUS_CLI_ALG_GREEDY 
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

    if (HPLUS_env.heuristic_enabled || HPLUS_env.alg == HPLUS_CLI_ALG_GREEDY) {

        mylog.print_info("Calculating heuristic solution.");
        HPLUS_env.exec_status = my::execution_status::HEURISTIC;

        HPLUS_stats.heuristic_time = HPLUS_env.time_limit - timer.get_time();
        double start_time = timer.get_time();

        HPLUS_find_heuristic();

        HPLUS_stats.heuristic_time = timer.get_time() - start_time;

    }

    if (HPLUS_env.sol_status == my::solution_status::INFEAS) return;

    if (HPLUS_env.alg == HPLUS_CLI_ALG_GREEDY) {
        HPLUS_env.exec_status = my::execution_status::STOP_TL;
        return;
    }

    // ~~~~~~~~~~~~ MODEL BUILDING ~~~~~~~~~~~ //

    mylog.print_info("Building model.");
    HPLUS_env.exec_status = my::execution_status::MODEL_BUILD;

    HPLUS_stats.build_time = HPLUS_env.time_limit - timer.get_time();
    double start_time = timer.get_time();

    CPXENVptr env = nullptr;
    CPXLPptr lp = nullptr;

    HPLUS_cpx_init(env, lp);

    if (HPLUS_env.alg == HPLUS_CLI_ALG_IMAI) HPLUS_cpx_build_imai(env, lp);
    else if (HPLUS_env.alg == HPLUS_CLI_ALG_RANKOOH) HPLUS_cpx_build_rankooh(env, lp);
    else if (HPLUS_env.alg == HPLUS_CLI_ALG_DYNAMIC_SMALL) HPLUS_cpx_build_dynamic_small(env, lp);
    else if (HPLUS_env.alg == HPLUS_CLI_ALG_DYNAMIC_LARGE) HPLUS_cpx_build_dynamic_large(env, lp);

    // time limit
    if ((double)HPLUS_env.time_limit > timer.get_time()) my::assert(!CPXsetdblparam(env, CPXPARAM_TimeLimit, (double)HPLUS_env.time_limit - timer.get_time()), "CPXsetdblparam (CPXPARAM_TimeLimit) failed.");
    else while(true);                   // handling edge case of time limit reached (if we enter the else, at any time the timer thread should terminate the execution, so we wait for him)

    if (HPLUS_env.warm_start_enabled) {     // Post warm starto to CPLEX

        if (HPLUS_env.alg == HPLUS_CLI_ALG_IMAI) HPLUS_cpx_post_warmstart_imai(env, lp);
        else if (HPLUS_env.alg == HPLUS_CLI_ALG_RANKOOH) HPLUS_cpx_post_warmstart_rankooh(env, lp);
        else if (HPLUS_env.alg == HPLUS_CLI_ALG_DYNAMIC_SMALL) HPLUS_cpx_post_warmstart_dynamic_small(env, lp);
        else if (HPLUS_env.alg == HPLUS_CLI_ALG_DYNAMIC_LARGE) HPLUS_cpx_post_warmstart_dynamic_large(env, lp);

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
        else if (HPLUS_env.alg == HPLUS_CLI_ALG_DYNAMIC_SMALL) HPLUS_store_dynamic_small_sol(env, lp);
        else if (HPLUS_env.alg == HPLUS_CLI_ALG_DYNAMIC_LARGE) HPLUS_store_dynamic_large_sol(env, lp);
    }

    HPLUS_cpx_close(env, lp);

    HPLUS_stats.execution_time = timer.get_time() - start_time;

}