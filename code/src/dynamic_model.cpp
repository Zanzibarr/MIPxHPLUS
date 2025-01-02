#include "../include/dynamic_model.hpp"

static int CPXPUBLIC HPLUS_cpx_candidate_callback(CPXCALLBACKCONTEXTptr context, CPXLONG context_id, void* user_handle) {

    if (context_id != CPX_CALLBACKCONTEXT_CANDIDATE) mylog.raise_error("Wrong callback context called.");

    const size_t n_act = HPLUS_inst.get_n_act(true);
    const size_t n_var = HPLUS_inst.get_n_var(false);
    const auto& actions = HPLUS_inst.get_actions();
    const auto& remaining_variables = HPLUS_inst.get_remaining_variables();

    // get candidate point
    double* xstar = new double[n_act];
    double cost = CPX_INFBOUND;
    my::assert(!CPXcallbackgetcandidatepoint(context, xstar, 0, n_act-1, &cost), "CPXcallbackgetcandidatepoint failed.");

    // split actions among used and unused
    std::vector<size_t> actions_used;
    std::vector<size_t> actions_unused;
    for (size_t act_i_cpx = 0; act_i_cpx < n_act; act_i_cpx++) {
        if (xstar[act_i_cpx] < HPLUS_CPX_INT_ROUNDING) actions_unused.push_back(act_i_cpx);
        else actions_used.push_back(act_i_cpx);
    }
    delete[] xstar; xstar = nullptr;

    // split used actions among reachable and unreachable
    std::vector<size_t> actions_reachable;
    std::vector<size_t> actions_unreachable = actions_used;
    my::binary_set current_state(n_var);
    bool loop;
    do {
        loop = false;
        std::vector<size_t> new_reached_actions;
        for (auto act_i_cpx : actions_unreachable) {
            size_t act_i = HPLUS_inst.cpx_idx_to_act_idx(act_i_cpx);
            if (current_state.contains(actions[act_i].get_pre() & remaining_variables)) {
                loop = true;
                current_state |= (actions[act_i].get_eff() & remaining_variables);
                new_reached_actions.push_back(act_i_cpx);
                actions_reachable.push_back(act_i_cpx);
            }
        }
        auto it = std::set_difference(actions_unreachable.begin(), actions_unreachable.end(), new_reached_actions.begin(), new_reached_actions.end(), actions_unreachable.begin());
        actions_unreachable.resize(it - actions_unreachable.begin());
    } while(loop);

    // solution is feasible
    if (actions_unreachable.empty()) return 0;

    #if HPLUS_VERBOSE >= 20
    int itcount = 0; my::assert(!CPXcallbackgetinfoint(context, CPXCALLBACKINFO_ITCOUNT, &itcount), "CPXcallbackgetinfoint(CPXCALLBACKINFO_ITCOUNT) failed.");
    if (itcount % 100 == 0) {
        double lower_bound = CPX_INFBOUND; my::assert(!CPXcallbackgetinfodbl(context, CPXCALLBACKINFO_BEST_BND, &lower_bound), "CPXcallbackgetinfodbl(CPXCALLBACKINFO_BEST_BND) failed.");
        double incumbent = CPX_INFBOUND; my::assert(!CPXcallbackgetinfodbl(context, CPXCALLBACKINFO_BEST_SOL, &incumbent), "CPXcallbackgetinfodbl(CPXCALLBACKINFO_BEST_SOL) failed.");
        double gap = (1 - lower_bound/incumbent) * 100;
        mylog.print_info("Pruned infeasible solution - cost : %7d - incumbent : %d - gap : %6.2f%%.", (int)cost, (int)incumbent, gap);
    }
    #endif

    // compute the reachability gap
    my::binary_set reachability_gap(n_var);
    for (auto act_i_cpx : actions_unreachable) reachability_gap |= actions[HPLUS_inst.cpx_idx_to_act_idx(act_i_cpx)].get_pre();
    for (auto act_i_cpx : actions_reachable) reachability_gap -= actions[HPLUS_inst.cpx_idx_to_act_idx(act_i_cpx)].get_eff();
    reachability_gap &= remaining_variables;

    // actions to be forced
    std::vector<size_t> forced_actions;
    for (auto act_i : actions_unused) if ((actions[HPLUS_inst.cpx_idx_to_act_idx(act_i)].get_eff()).intersects(reachability_gap)) forced_actions.push_back(act_i);

    // build constraint
    const char sense = 'L';
    const int izero = 0;
    int nnz_i = 0;
    const int nnz = actions_unreachable.size() + forced_actions.size();
    const double rhs = 0;
    int* ind = new int[nnz];
    double* val = new double[nnz];

    for (auto act_i_cpx : actions_unreachable) {
        ind[nnz_i] = act_i_cpx;
        val[nnz_i++] = 1.0 / actions_unreachable.size();
    }

    for (auto act_i_cpx : forced_actions) {
        ind[nnz_i] = act_i_cpx;
        val[nnz_i++] = -1;
    }

    my::assert(!CPXcallbackrejectcandidate(context, 1, nnz, &rhs, &sense, &izero, ind, val), "CPXcalbackrejectcandidate failed");

    delete[] val; val = nullptr;
    delete[] ind; ind = nullptr;

    // TODO: Look for loops and create new constraint (loops require external actions)

    return 0;

}

void HPLUS_cpx_build_dynamic(CPXENVptr& env, CPXLPptr& lp) {

    const auto& actions = HPLUS_inst.get_actions();

    const auto& remaining_variables = HPLUS_inst.get_remaining_variables().sparse();
    const auto& remaining_var_set = HPLUS_inst.get_remaining_variables();
    const auto& remaining_actions = HPLUS_inst.get_remaining_actions().sparse();

    const auto& fixed_variables = HPLUS_inst.get_fixed_variables();
    const auto& fixed_actions = HPLUS_inst.get_fixed_actions();

    const auto n_var_presim = HPLUS_inst.get_n_var();
    const auto n_var = HPLUS_inst.get_n_var(true);
    const auto n_act = HPLUS_inst.get_n_act(true);

    // ====================================================== //
    // =================== CPLEX VARIABLES ================== //
    // ====================================================== //

    size_t curr_col = 0;
    double* objs = new double[n_act];
    double* lbs = new double[n_act];
    double* ubs = new double[n_act];
    char* types = new char[n_act];

    auto resize_cpx_arrays = [&objs, &lbs, &ubs, &types](size_t new_size) {

        delete[] types; types = nullptr;
        delete[] ubs; ubs = nullptr;
        delete[] lbs; lbs = nullptr;
        delete[] objs; objs = nullptr;

        objs = new double[new_size];
        lbs = new double[new_size];
        ubs = new double[new_size];
        types = new char[new_size];

    };

    // -------- actions ------- //
    size_t act_start = curr_col;
    size_t count = 0;
    for (auto act_i : remaining_actions) {
        objs[count] = actions[act_i].get_cost();
        lbs[count] = fixed_actions[act_i] ? 1 : 0;
        ubs[count] = 1;
        types[count++] = 'B';
    }
    #if HPLUS_INTCHECK
    my::assert(count == n_act, "Wrong number of actions while building the model.");
    #endif
    curr_col += n_act;

    my::assert(!CPXnewcols(env, lp, n_act, objs, lbs, ubs, types, nullptr), "CPXnewcols (actions) failed.");

    resize_cpx_arrays(n_var);

    // ------- variables ------ //
    size_t var_start = curr_col;
    const auto& gstate = HPLUS_inst.get_goal_state();
    count = 0;
    for (auto var_i : remaining_variables) {
        objs[count] = 0;
        lbs[count] = (fixed_variables[var_i] || gstate[var_i]) ? 1 : 0;
        ubs[count] = 1;
        types[count++] = 'B';
    }
    #if HPLUS_INTCHECK
    my::assert(count == n_var, "Wrong number of variables while building the model.");
    #endif
    curr_col += n_var;

    my::assert(!CPXnewcols(env, lp, n_var, objs, lbs, ubs, types, nullptr), "CPXnewcols (variables) failed.");

    delete[] types; types = nullptr;
    delete[] ubs; ubs = nullptr;
    delete[] lbs; lbs = nullptr;
    delete[] objs; objs = nullptr;

    // ====================================================== //
    // ================== CPLEX CONSTRAINTS ================= //
    // ====================================================== //

    // mylog.print_info("Adding constraints to CPLEX.");

    // accessing cplex variables
    auto get_act_idx = [act_start](size_t idx) { return act_start + HPLUS_inst.act_idx_post_simplification(idx); };
    auto get_var_idx = [var_start](size_t idx) { return var_start + HPLUS_inst.var_idx_post_simplification(idx); };

    int* ind = new int[n_act + 1];
    double* val = new double[n_act + 1];
    int nnz = 0;
    const char sense = 'L';
    const double rhs = 0;
    const int begin = 0;
    
    // precompute list of actions that have a specific variable as effect (some sort of hashing)
    std::vector<std::vector<size_t>> act_with_eff(n_var_presim);
    for (auto var_i : remaining_variables) for (auto act_i : remaining_actions) if (actions[act_i].get_eff()[var_i]) act_with_eff[var_i].push_back(act_i);

    for (auto act_i : remaining_actions) {
        for (auto var_i : actions[act_i].get_pre_sparse()) if (remaining_var_set[var_i]) {
            ind[0] = get_act_idx(act_i);
            val[0] = 1;
            ind[1] = get_var_idx(var_i);
            val[1] = -1;
            my::assert(!CPXaddrows(env, lp, 0, 1, 2, &rhs, &sense, &begin, ind, val, nullptr, nullptr), "CPXaddrows(newc) failed.");
        }
    }

    for (auto var_i : remaining_variables) {
        nnz = 0;
        ind[nnz] = get_var_idx(var_i);
        val[nnz++] = 1;
        for (auto act_i : act_with_eff[var_i]) {
            ind[nnz] = get_act_idx(act_i);
            val[nnz++] = -1;
        }
        my::assert(!CPXaddrows(env, lp, 0, 1, nnz, &rhs, &sense, &begin, ind, val, nullptr, nullptr), "CPXaddrows(newc) failed.");
    }

    delete[] val; val = nullptr;
    delete[] ind; ind = nullptr;

    // Adding the callback function
    my::assert(!CPXcallbacksetfunc(env, lp, CPX_CALLBACKCONTEXT_CANDIDATE, HPLUS_cpx_candidate_callback, NULL), "CPXcallbacksetfunc failed.");

    // my::assert(!CPXwriteprob(env, lp, (HPLUS_CPLEX_OUT_DIR"/lp/"+HPLUS_env.run_name+".lp").c_str(), "LP"), "CPXwriteprob failed.");

    // mylog.print_info("Created CPLEX lp for my model.");

}

void HPLUS_cpx_post_warmstart_dynamic(CPXENVptr& env, CPXLPptr& lp) {
    
    #if HPLUS_INTCHECK
    my::assert(HPLUS_env.sol_status != my::solution_status::INFEAS && HPLUS_env.sol_status != my::solution_status::NOTFOUND, "Warm start failed.");
    #endif

    const auto& actions = HPLUS_inst.get_actions();
    size_t nvar = HPLUS_inst.get_n_var(true);
    size_t nact = HPLUS_inst.get_n_act(true);
    const auto& remaining_variables = HPLUS_inst.get_remaining_variables();
    my::binary_set current_state = my::binary_set(HPLUS_inst.get_n_var());

    std::vector<size_t> warm_start;
    unsigned int _;
    HPLUS_inst.get_best_sol(warm_start, _);

    size_t ncols = CPXgetnumcols(env, lp);
    int* cpx_sol_ind = new int[ncols];
    double* cpx_sol_val = new double[ncols];

    for (size_t i = 0; i < ncols; i++) {
        cpx_sol_ind[i] = i;
        cpx_sol_val[i] = 0;
    }

    int izero = 0;
    int effortlevel = CPX_MIPSTART_NOCHECK;
    size_t nnz = 0;

    for (auto act_i : warm_start) {
        cpx_sol_val[HPLUS_inst.act_idx_post_simplification(act_i)] = 1;
        for (auto var_i : actions[act_i].get_eff_sparse()) if (remaining_variables[var_i] && !current_state[var_i]) {
            cpx_sol_val[nact + HPLUS_inst.var_idx_post_simplification(var_i)] = 1;
            current_state.add(var_i);
        }
    }

    my::assert(!CPXaddmipstarts(env, lp, 1, ncols, &izero, cpx_sol_ind, cpx_sol_val, &effortlevel, nullptr), "CPXaddmipstarts failed.");
    delete[] cpx_sol_ind; cpx_sol_ind = nullptr;
    delete[] cpx_sol_val; cpx_sol_val = nullptr;

}

void HPLUS_store_dynamic_sol(const CPXENVptr& env, const CPXLPptr& lp) {

    const size_t n_var = HPLUS_inst.get_n_var(false);
    const size_t n_act = HPLUS_inst.get_n_act(false);

    double* plan = new double[HPLUS_inst.get_n_act(true)];
    my::assert(!CPXgetx(env, lp, plan, 0, HPLUS_inst.get_n_act(true)-1), "CPXgetx failed.");

    my::binary_set sol_rem_actions(n_act);
    for (size_t act_i_cpx = 0; act_i_cpx < HPLUS_inst.get_n_act(true); act_i_cpx++) if (plan[act_i_cpx] > HPLUS_CPX_INT_ROUNDING) sol_rem_actions.add(HPLUS_inst.cpx_idx_to_act_idx(act_i_cpx));

    delete[] plan; plan = nullptr;

    std::vector<size_t> solution;
    const auto& actions = HPLUS_inst.get_actions();

    my::binary_set current_state(n_var);
    while (!current_state.contains(HPLUS_inst.get_goal_state())) {
        for (auto act_i : sol_rem_actions) if (current_state.contains(actions[act_i].get_pre())) {
            current_state |= actions[act_i].get_eff();
            sol_rem_actions.remove(act_i);
            solution.push_back(act_i);
        }
    }

    HPLUS_inst.update_best_sol(solution,
        std::accumulate(solution.begin(), solution.end(), 0,
            [&actions](const size_t acc, const size_t index) {
                return acc + actions[index].get_cost();
            }
        )
    );

}