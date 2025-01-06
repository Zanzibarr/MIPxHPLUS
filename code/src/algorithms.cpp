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

//[ ]: Better heuristic
void HPLUS_find_heuristic() {

    const auto& actions = HPLUS_inst.get_actions();
    my::binary_set priority_rem_actions = HPLUS_inst.get_fixed_actions();
    my::binary_set remaining_actions = HPLUS_inst.get_remaining_actions() & !priority_rem_actions;
    const auto& remaining_variables = HPLUS_inst.get_remaining_variables();

    my::binary_set current_state(HPLUS_inst.get_n_var());

    // prepare the set of feasible actions given the current_state
    my::subset_searcher feasible_actions = my::subset_searcher();
    for (auto act_i : remaining_actions) feasible_actions.add(act_i, actions[act_i].get_pre());

    // list of actions that have as precondition variable p
    std::vector<std::vector<size_t>> n_act_with_pre(HPLUS_inst.get_n_var());
    for (auto var_i : remaining_variables) for (auto act_i : remaining_actions) if (actions[act_i].get_pre()[var_i]) n_act_with_pre[var_i].push_back(act_i);

    // greedy idea: choose the next action as the one that makes me pay the less per (relevant) variable added to the current state
    auto find_best_act = [&n_act_with_pre, &remaining_actions, &remaining_variables, &feasible_actions, &actions](const my::binary_set& current_state) {

        // only actions that can be executed from the current state
        const auto& cand_actions = feasible_actions.find_subsets(current_state);

        // check if the problem is infeasible
        bool infeas = true;
        for (auto act_i : cand_actions) if (remaining_actions[act_i]) { infeas = false; break; }
        if (infeas) return -1;

        int best_act = -1;
        double best_cost_per_eff = INFINITY;

        for (auto act_i : cand_actions) if (remaining_actions[act_i]) {
            int n_eff = 0, n_new_act = 0;
            my::binary_set counted_new_actions = my::binary_set(actions.size());
            for (auto var_i : actions[act_i].get_eff_sparse()) if (!current_state[var_i] && remaining_variables[var_i]) {
                n_eff++;
                for (auto act_j : n_act_with_pre[var_i]) if (remaining_actions[act_j] && !counted_new_actions[act_j]) {
                    n_new_act++;
                    counted_new_actions.add(act_j);
                }
            }
            if (n_eff == 0) continue;           // actions that have no useful effects can be skipped (if there are no useful effects that can be added, the current state will never move on --> Infeasible --> We should have already detected that)
            double cost_per_eff = actions[act_i].get_cost() * ((double)HPLUS_inst.get_n_var(true) / n_eff + ((double)HPLUS_inst.get_n_act(true)) / (n_new_act == 0 ? 1 : n_new_act));
            if (cost_per_eff <= best_cost_per_eff) {
                best_act = act_i;
                best_cost_per_eff = cost_per_eff;
            }
        }

        #if HPLUS_INTCHECK
        my::assert(best_act != -1, "Error in finding best action for heuristic.");      // if no action found we have an error -> we already proved it's not infeasible, so we must have an action to use
        #endif

        return best_act;

    };

    std::vector<size_t> heur_solution;
    unsigned int heur_cost = 0;

    int priority_act_count = 0;
    for (auto _ : priority_rem_actions) priority_act_count++;
    const auto& goal_state = HPLUS_inst.get_goal_state();

    while (!current_state.contains(goal_state)) {

        int best_act_i = -1;

        if (priority_act_count > 0) for (auto act_i : priority_rem_actions) {       // actions that were fixed by the problem simplification are gonna be added to the plan as soon as we can, since we know that at least one optimal solution will include them
            if (current_state.contains(actions[act_i].get_pre())) {
                priority_act_count--;
                current_state |= actions[act_i].get_eff();
                heur_solution.push_back(act_i);
                heur_cost += actions[act_i].get_cost();
                priority_rem_actions.remove(act_i);
                best_act_i = act_i;
            }
        }

        if (best_act_i >= 0) continue;      // if I used an action among the fixed ones, the current state changed and another one may be used next, so go back and look again
        // otherwise seek for the next one among the rest of them

        best_act_i = find_best_act(current_state);
        if (best_act_i == -1) {
            HPLUS_env.sol_status = my::solution_status::INFEAS;
            return;
        }
        
        current_state |= actions[best_act_i].get_eff();
        heur_solution.push_back(best_act_i);
        heur_cost += actions[best_act_i].get_cost();
        remaining_actions.remove(best_act_i);

    }
    
    #if HPLUS_INTCHECK
    my::assert(priority_act_count == 0, "Error in finding heuristic.");
    #endif

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