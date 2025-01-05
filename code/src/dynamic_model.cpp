#include "../include/dynamic_model.hpp"

// straight from Johnson's paper
void unblock(size_t u, std::vector<bool>& blocked, std::vector<std::set<size_t>>& block_map) {

    blocked[u] = false;
    for (int w : block_map[u]) if (blocked[w]) unblock(w, blocked, block_map);
    block_map[u].clear();

}

// straight from Johnson's paper
bool circuit(size_t v, size_t start, std::vector<std::vector<size_t>>& graph, std::vector<bool>& blocked, std::vector<std::set<size_t>>& block_map, std::stack<size_t>& path, std::vector<std::vector<size_t>>& cycles) {

    bool found_cycle = false;
    path.push(v);
    blocked[v] = true;
    for (auto w : graph[v]) {
        if (w == start) {
            // found a cycle
            std::stack<size_t> copy = path;
            std::vector<size_t> cycle;
            while (!copy.empty()) {
                cycle.push_back(copy.top());
                copy.pop();
            }
            std::reverse(cycle.begin(), cycle.end());
            cycles.push_back(cycle);
            found_cycle = true;
        } else if (!blocked[w] && circuit(w, start, graph, blocked, block_map, path, cycles)) found_cycle = true;
    }

    if (found_cycle) unblock(v, blocked, block_map);
    else for (auto w : graph[v]) block_map[w].insert(v);

    path.pop();
    return found_cycle;

}

static int CPXPUBLIC HPLUS_cpx_candidate_callback(CPXCALLBACKCONTEXTptr context, CPXLONG context_id, void* user_handle) {

    if (context_id != CPX_CALLBACKCONTEXT_CANDIDATE) mylog.raise_error("Wrong callback context called.");

    double start_time = timer.get_time();

    const size_t n_act = HPLUS_inst.get_n_act(true);
    const size_t n_act_presim = HPLUS_inst.get_n_act(false);
    const size_t n_var = HPLUS_inst.get_n_var(true);
    const size_t n_var_presim = HPLUS_inst.get_n_var(false);

    const auto& actions = HPLUS_inst.get_actions();
    const auto& remaining_actions = HPLUS_inst.get_remaining_actions().sparse();
    const auto& remaining_variables = HPLUS_inst.get_remaining_variables().sparse();
    const auto& remaining_variables_set = HPLUS_inst.get_remaining_variables();

    // get candidate point
    double* xstar = new double[n_act + n_act * n_var];
    double cost = CPX_INFBOUND;
    my::assert(!CPXcallbackgetcandidatepoint(context, xstar, 0, n_act + n_act * n_var - 1, &cost), "CPXcallbackgetcandidatepoint failed.");
    
    // fixing the solution to read the plan (some actions are set to 1 even if they are not a first archiever of anything (null cost))
    std::vector<my::binary_set> cpx_var_archieved(n_act, my::binary_set(n_var));
    for (size_t act_i_cpx = 0, fadd_i = n_act; act_i_cpx < n_act; act_i_cpx++, fadd_i += n_var) {
        bool set_zero = true;
        for (size_t var_i_cpx = 0; var_i_cpx < n_var; var_i_cpx++) {
            if (xstar[fadd_i + var_i_cpx] > HPLUS_CPX_INT_ROUNDING) {
                #if HPLUS_INTCHECK
                my::assert(xstar[act_i_cpx] > HPLUS_CPX_INT_ROUNDING, "Action is set to 0 even if it's a first archiever.");
                #endif
                set_zero = false;
                cpx_var_archieved[act_i_cpx].add(var_i_cpx);
            }
        }
        if (set_zero) xstar[act_i_cpx] = 0;
    }

    // split actions among used and unused
    std::vector<size_t> actions_used;
    std::vector<size_t> actions_unused;
    for (size_t act_i_cpx = 0; act_i_cpx < n_act; act_i_cpx++) {
        if (xstar[act_i_cpx] < HPLUS_CPX_INT_ROUNDING) actions_unused.push_back(act_i_cpx);
        else actions_used.push_back(act_i_cpx);
    }

    // split used actions among reachable and unreachable
    std::vector<size_t> actions_reachable;
    std::vector<size_t> actions_unreachable = actions_used;
    my::binary_set current_state(n_var_presim);
    bool loop;
    do {
        loop = false;
        std::vector<size_t> new_reached_actions;
        for (auto act_i_cpx : actions_unreachable) {
            size_t act_i = HPLUS_inst.cpx_idx_to_act_idx(act_i_cpx);
            if (current_state.contains(actions[act_i].get_pre() & remaining_variables_set)) {
                loop = true;
                current_state |= (actions[act_i].get_eff() & remaining_variables_set);
                new_reached_actions.push_back(act_i_cpx);
                actions_reachable.push_back(act_i_cpx);
            }
        }
        auto it = std::set_difference(actions_unreachable.begin(), actions_unreachable.end(), new_reached_actions.begin(), new_reached_actions.end(), actions_unreachable.begin());
        actions_unreachable.resize(it - actions_unreachable.begin());
    } while(loop);

    // solution is feasible
    if (actions_unreachable.empty()) {
        delete[] xstar; xstar = nullptr;
        return 0;
    }

    #if HPLUS_VERBOSE >= 20
    double lower_bound = CPX_INFBOUND; my::assert(!CPXcallbackgetinfodbl(context, CPXCALLBACKINFO_BEST_BND, &lower_bound), "CPXcallbackgetinfodbl(CPXCALLBACKINFO_BEST_BND) failed.");
    double incumbent = CPX_INFBOUND; my::assert(!CPXcallbackgetinfodbl(context, CPXCALLBACKINFO_BEST_SOL, &incumbent), "CPXcallbackgetinfodbl(CPXCALLBACKINFO_BEST_SOL) failed.");
    double gap = (1 - lower_bound/incumbent) * 100;
    mylog.print_info("Pruned infeasible solution - cost : %7d - incumbent : %d - gap : %6.2f%%.", (int)cost, (int)incumbent, gap);
    #endif

    std::vector<std::vector<size_t>> act_with_pre(n_var);
    for (auto var_i : remaining_variables) for (auto act_i_cpx : actions_unreachable) if (actions[HPLUS_inst.cpx_idx_to_act_idx(act_i_cpx)].get_pre()[var_i]) act_with_pre[HPLUS_inst.var_idx_post_simplification(var_i)].push_back(act_i_cpx);

    // build graph G=<A, {(ai, aj) ai, aj in A | ai is first archiever (in the current solution) to a precondition of aj}>
    std::vector<std::set<size_t>> graph(n_act);
    for (auto act_i_cpx : actions_unreachable) for (auto var_i_cpx : cpx_var_archieved[act_i_cpx]) graph[act_i_cpx].insert(act_with_pre[var_i_cpx].begin(), act_with_pre[var_i_cpx].end());

    // find all simple cycles (Johnson's paper)
    size_t n = graph.size();
    std::vector<bool> blocked(n, false);
    std::vector<std::set<size_t>> bock_map(n);
    std::stack<size_t> path;
    std::vector<std::vector<size_t>> cycles;

    for (auto act_i_cpx : actions_unreachable) {

        // find subgraph with vertices >= act_i_cpx
        std::vector<std::vector<size_t>> subgraph(n);
        for (size_t u = act_i_cpx; u < n; u++) for (auto v : graph[u]) if (v >= act_i_cpx) subgraph[u].push_back(v);

        // run circuit function to find all cycles starting at "act_i_cpx"
        circuit(act_i_cpx, act_i_cpx, subgraph, blocked, bock_map, path, cycles);

        // remove act_i_cpx vertex from the graph
        for (size_t u = 0; u < n; u++) graph[u].erase(act_i_cpx);

    }
    
    int* ind = new int[n_var];          // the size is n_var since we know that there can't be more that one first archiever selected per variable, hence the used first archievers are at most n_var
    double* val = new double[n_var];
    int nnz = 0;
    double rhs = 0;
    const int izero = 0;
    const char sense_l = 'L';

    std::vector<size_t> var_idx_presim(n_var);
    for (size_t var_i = 0, var_i_cpx = 0; var_i < n_var_presim; var_i++) {
        if (remaining_variables_set[var_i]) {
            var_idx_presim[var_i_cpx] = var_i;
            var_i_cpx++;
        }
    }

    // Simple Cycle Cuts
    for (auto cycle : cycles) {

        nnz = 0;

        // first archievers in the cycle
        for (size_t i = 0; i < cycle.size() - 1; i++) {
            for (auto var_i_cpx : cpx_var_archieved[cycle[i]]) {
                if (actions[HPLUS_inst.cpx_idx_to_act_idx(cycle[i+1])].get_pre()[var_idx_presim[var_i_cpx]]) {
                    ind[nnz] = n_act + cycle[i] * n_var + var_i_cpx;
                    val[nnz++] = 1;
                }
            }
        }
        for (auto var_i_cpx : cpx_var_archieved[cycle[cycle.size() - 1]]) {
            if (actions[HPLUS_inst.cpx_idx_to_act_idx(cycle[0])].get_pre()[var_idx_presim[var_i_cpx]]) {
                ind[nnz] = n_act + cycle[cycle.size() - 1] * n_var + var_i_cpx;
                val[nnz++] = 1;
            }
        }

        // cannot ALL be selected (at least one must come from outside the cycle)
        rhs = nnz - 1;

        my::assert(!CPXcallbackrejectcandidate(context, 1, nnz, &rhs, &sense_l, &izero, ind, val), "CPXcalbackrejectcandidate failed");

    }

    delete[] val; val = nullptr;
    delete[] ind; ind = nullptr;
    delete[] xstar; xstar = nullptr;

    pthread_mutex_lock(&(HPLUS_stats.callback_time_mutex_));
    HPLUS_stats.callback_time += timer.get_time() - start_time;
    pthread_mutex_unlock(&(HPLUS_stats.callback_time_mutex_));
    
    return 0;

}

void HPLUS_cpx_build_dynamic(CPXENVptr& env, CPXLPptr& lp) {

    const auto& actions = HPLUS_inst.get_actions();

    const auto& remaining_variables = HPLUS_inst.get_remaining_variables().sparse();
    const auto& remaining_var_set = HPLUS_inst.get_remaining_variables();
    const auto& remaining_actions = HPLUS_inst.get_remaining_actions().sparse();

    const auto& fixed_variables = HPLUS_inst.get_fixed_variables();
    const auto& fixed_actions = HPLUS_inst.get_fixed_actions();
    const auto& eliminated_fa = HPLUS_inst.get_eliminated_fa();
    const auto& fixed_fa = HPLUS_inst.get_fixed_fa();

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

    // --- first archievers --- //
    size_t fa_start = curr_col;
    std::vector<size_t> fa_individual_start(n_act);
    count = 0;
    for (auto act_i : remaining_actions) {
        fa_individual_start[count] = count * n_var;
        size_t count_var = 0;
        for (auto var_i : remaining_variables) {
            objs[count_var] = 0;
            lbs[count_var] = fixed_fa[act_i][var_i] ? 1 : 0;
            ubs[count_var] = (!actions[act_i].get_eff()[var_i] || eliminated_fa[act_i][var_i]) ? 0 : 1;
            types[count_var++] = 'B';
        }
        #if HPLUS_INTCHECK
        my::assert(count_var == n_var, "Wrong number of first archievers while building the model.");
        #endif
        curr_col += n_var;
        my::assert(!CPXnewcols(env, lp, n_var, objs, lbs, ubs, types, nullptr), "CPXnewcols (first archievers) failed.");
        count++;
    }

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

    // accessing cplex variables
    auto get_act_idx = [act_start](size_t idx) { return act_start + HPLUS_inst.act_idx_post_simplification(idx); };
    auto get_var_idx = [var_start](size_t idx) { return var_start + HPLUS_inst.var_idx_post_simplification(idx); };
    auto get_fa_idx = [fa_start, &fa_individual_start](size_t act_idx, size_t var_idx) { return fa_start + fa_individual_start[HPLUS_inst.act_idx_post_simplification(act_idx)] + HPLUS_inst.var_idx_post_simplification(var_idx); };

    int* ind = new int[n_act + 1];
    double* val = new double[n_act + 1];
    int nnz = 0;
    const char sense_l = 'L', sense_e = 'E';
    const double rhs_0 = 0;
    const int begin = 0;

    // precompute list of actions that have a specific variable as effect (some sort of hashing)
    std::vector<std::vector<size_t>> act_with_eff(n_var_presim);
    for (auto var_i : remaining_variables) for (auto act_i : remaining_actions) if (actions[act_i].get_eff()[var_i]) act_with_eff[var_i].push_back(act_i);

    for (auto var_i : remaining_variables) {

        nnz = 0;
        ind[nnz] = get_var_idx(var_i);
        val[nnz++] = 1;

        for (auto act_i : act_with_eff[var_i]) {
            ind[nnz] = get_fa_idx(act_i, var_i);
            val[nnz++] = -1;
        }

        my::assert(!CPXaddrows(env, lp, 0, 1, nnz, &rhs_0, &sense_e, &begin, ind, val, nullptr, nullptr), "CPXaddwors (c2) failed.");

    }
    
    for (auto var_i : remaining_variables) {
        for (auto var_j : remaining_variables) {
            nnz = 0;
            ind[nnz] = get_var_idx(var_j);
            val[nnz++] = -1;
            for (auto act_i : act_with_eff[var_i]) {
                if (actions[act_i].get_pre()[var_j]) {
                    ind[nnz] = get_fa_idx(act_i, var_i);
                    val[nnz++] = 1;
                }
            }
            my::assert(!CPXaddrows(env, lp, 0, 1, nnz, &rhs_0, &sense_l, &begin, ind, val, nullptr, nullptr), "CPXaddwors (c3) failed.");
        }
    }

    delete[] val; val = nullptr;
    delete[] ind; ind = nullptr;

    int ind_c5[2];
    double val_c5[2];
    
    for (auto act_i : remaining_actions) {
        for (auto var_i : actions[act_i].get_eff()) if (remaining_var_set[var_i]) {
            ind_c5[0] = get_act_idx(act_i);
            val_c5[0] = -1;
            ind_c5[1] = get_fa_idx(act_i, var_i);
            val_c5[1] = 1;
            my::assert(!CPXaddrows(env, lp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind_c5, val_c5, nullptr, nullptr), "CPXaddrows (c5) faliled.");
        }
    }

    // Adding the callback function
    my::assert(!CPXcallbacksetfunc(env, lp, CPX_CALLBACKCONTEXT_CANDIDATE, HPLUS_cpx_candidate_callback, NULL), "CPXcallbacksetfunc failed.");

    // my::assert(!CPXwriteprob(env, lp, (HPLUS_CPLEX_OUT_DIR"/lp/"+HPLUS_env.run_name+".lp").c_str(), "LP"), "CPXwriteprob failed.");

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

    int izero = 0;
    int effortlevel = CPX_MIPSTART_REPAIR;
    size_t nnz = 0;

    for (auto act_i : warm_start) {
        cpx_sol_ind[nnz] = HPLUS_inst.act_idx_post_simplification(act_i);
        cpx_sol_val[nnz++] = 1;
        for (auto var_i : actions[act_i].get_eff_sparse()) if (remaining_variables[var_i] && !current_state[var_i]) {
            cpx_sol_ind[nnz] = nact + nact * nvar + HPLUS_inst.var_idx_post_simplification(var_i);
            cpx_sol_val[nnz++] = 1;
            cpx_sol_ind[nnz] = nact + HPLUS_inst.fa_idx_post_simplification(act_i, var_i);
            cpx_sol_val[nnz++] = 1;
            current_state.add(var_i);
        }
    }

    my::assert(!CPXaddmipstarts(env, lp, 1, nnz, &izero, cpx_sol_ind, cpx_sol_val, &effortlevel, nullptr), "CPXaddmipstarts failed.");
    delete[] cpx_sol_ind; cpx_sol_ind = nullptr;
    delete[] cpx_sol_val; cpx_sol_val = nullptr;

}

void HPLUS_store_dynamic_sol(const CPXENVptr& env, const CPXLPptr& lp) {

    const size_t n_var = HPLUS_inst.get_n_var(true);
    const size_t n_act = HPLUS_inst.get_n_act(true);

    double* plan = new double[n_act + n_act * n_var];
    my::assert(!CPXgetx(env, lp, plan, 0, n_act + n_act * n_var - 1), "CPXgetx failed.");
    
    // fixing the solution to read the plan (some actions are set to 1 even if they are not a first archiever of anything)
    for (size_t act_i_cpx = 0, fadd_i = n_act; act_i_cpx < n_act; act_i_cpx++, fadd_i += n_var) {
        bool set_zero = true;
        for (size_t var_i_cpx = 0; var_i_cpx < n_var; var_i_cpx++) {
            if (plan[fadd_i + var_i_cpx] > HPLUS_CPX_INT_ROUNDING) {
                #if HPLUS_INTCHECK
                my::assert(plan[act_i_cpx] > HPLUS_CPX_INT_ROUNDING, "Action is set to 0 even if it's a first archiever.");
                #endif
                set_zero = false;
                break;
            }
        }
        if (set_zero) plan[act_i_cpx] = 0;
    }

    // convert to std collections for easier parsing
    std::vector<size_t> cpx_result;
    for (size_t i = 0; i < n_act; i++) if (plan[i] > .5) cpx_result.push_back(HPLUS_inst.cpx_idx_to_act_idx(i));
    delete[] plan; plan = nullptr;

    const auto& actions = HPLUS_inst.get_actions();
    std::vector<size_t> solution;
    my::binary_set sorted(cpx_result.size());
    my::binary_set current_state(HPLUS_inst.get_n_var());
    
    while (sorted != my::binary_set(cpx_result.size(), true)) {
        #if HPLUS_INTCHECK
        bool intcheck = false;
        #endif
        for (auto i : !sorted) {
            if (current_state.contains(actions[cpx_result[i]].get_pre())) {
                sorted.add(i);
                current_state |= actions[cpx_result[i]].get_eff();
                solution.push_back(cpx_result[i]);
                #if HPLUS_INTCHECK
                intcheck = true;
                #endif
                break;
            }
        }
        #if HPLUS_INTCHECK
        my::assert(intcheck, "Solution found doesn't respect preconditions.");
        #endif
    }

    // store solution
    HPLUS_inst.update_best_sol(solution,
        std::accumulate(solution.begin(), solution.end(), 0,
            [&actions](const unsigned int acc, const size_t index) {
                return acc + actions[index].get_cost();
            }
        )
    );

}