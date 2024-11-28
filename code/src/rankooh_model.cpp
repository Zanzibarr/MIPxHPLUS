#include "../include/rankooh_model.hpp"

void HPLUS_cpx_build_rankooh(CPXENVptr& env, CPXLPptr& lp) {

    const auto& actions = HPLUS_inst.get_actions();

    const auto& remaining_variables = HPLUS_inst.get_remaining_variables().sparse();
    const auto& remaining_var_set = HPLUS_inst.get_remaining_variables();
    const auto& remaining_actions = HPLUS_inst.get_remaining_actions().sparse();

    const auto& fixed_variables = HPLUS_inst.get_fixed_variables();
    const auto& fixed_actions = HPLUS_inst.get_fixed_actions();
    const auto& eliminated_fa = HPLUS_inst.get_eliminated_fa();
    const auto& fixed_fa = HPLUS_inst.get_fixed_fa();
    const auto& timestamps_var = HPLUS_inst.get_timestamps_var();
    const auto& timestamps_act = HPLUS_inst.get_timestamps_act();

    const auto n_var_presim = HPLUS_inst.get_n_var();
    const auto n_var = HPLUS_inst.get_n_var(true);
    const auto n_act = HPLUS_inst.get_n_act(true);

    // ====================================================== //
    // ================= VERTEX ELIMINATION ================= //
    // ====================================================== //

    // mylog.print_info("Vertex elimination from Rankooh's paper.");

    struct Node {
        size_t deg, id;
        Node(size_t i, size_t d) : deg(d), id(i) {}
    };

    struct CompareNode { bool operator()(const Node& n1, const Node& n2) { return n1.deg > n2.deg; } };

    struct Triangle {
        size_t first, second, third;
        Triangle(size_t f, size_t s, size_t t) : first(f), second(s), third(t) {}
    };

    std::vector<std::set<size_t>> graph(n_var_presim);                                                // sparse graph for construction
    std::vector<my::binary_set> cumulative_graph(n_var_presim, my::binary_set(n_var_presim));                           // dense graph for fast access
    std::vector<Triangle> triangles_list;                                                           // list of triangles

    std::priority_queue<Node, std::vector<Node>, CompareNode> nodes_queue;                          // node priority
    std::vector<size_t> degree_counter(n_var_presim, 0);                                              // ordering

    // G_0
    for (auto act_i : remaining_actions) {
        const auto pre = actions[act_i].get_pre() & remaining_var_set;
        const auto eff = actions[act_i].get_eff() & remaining_var_set;
        const auto eff_sparse = eff.sparse();
        for (auto var_i : pre) {
            for (auto var_j : eff_sparse) if (var_i != var_j) {
                size_t pre_size = graph[var_i].size();
                graph[var_i].insert(var_j);
                if (pre_size != graph[var_i].size()) {                                              // if the edge is new update the degree
                    degree_counter[var_i] += 1;
                    degree_counter[var_j] += 1;
                }
            }
            cumulative_graph[var_i] |= eff;
        }
    }

    for (size_t node_i = 0; node_i < n_var_presim; node_i++) if (degree_counter[node_i] > 0) nodes_queue.emplace(node_i, degree_counter[node_i]);

    // finding minimum degree node
    auto find_min = [&n_var_presim](std::priority_queue<Node, std::vector<Node>, CompareNode>& nodes_queue, const std::vector<size_t> degree_counter) {

        int idx = -1;
        while (!nodes_queue.empty() && idx < 0) {
            Node tmp = nodes_queue.top();
            nodes_queue.pop();
            if (degree_counter[tmp.id] == tmp.deg) idx = tmp.id;
        }
        return idx;

    };

    // G_i (minimum degree heuristics)
    for (auto _ : remaining_variables) {

        int idx = find_min(nodes_queue, degree_counter);
        if (idx == -1) break;

        // graph structure:
        // | \       > |
        // p -> idx -> q
        // | /       > |

        std::set<size_t> new_nodes;

        for (auto p : remaining_variables) if (graph[p].find(idx) != graph[p].end()) {

            for (auto q : graph[idx]) if (p != q) {

                // add edge p - q
                size_t pre_size = graph[p].size();
                graph[p].insert(q);
                if (pre_size != graph[p].size()) {
                    degree_counter[p] += 1;
                    degree_counter[q] += 1;
                }

                // update the overall graph
                cumulative_graph[p].add(q);
            
            }
            
            // remove the edge p - idx
            graph[p].erase(idx);
            degree_counter[p] -= 1;
            new_nodes.insert(p);

            // update triangles list
            for (auto q : graph[idx]) if (p != q) triangles_list.emplace_back(p, idx, q);

        }

        // remove the edge idx - q
        for (auto q : graph[idx]) {
            degree_counter[q] -= 1;
            new_nodes.insert(q);
        }
        graph[idx].clear();
        degree_counter[idx] = 0;
        
        // Update the priority queue
        for (auto node : new_nodes) if (degree_counter[node] > 0) nodes_queue.emplace(node, degree_counter[node]);

        #if HPLUS_INTCHECK              // care: this takes HUGE amount of time
        for (size_t node_i = 0; node_i < n_var_presim; node_i++) {
            size_t i_cnt = 0;
            i_cnt += graph[node_i].size();
            for (size_t tmp_j = 0; tmp_j < n_var_presim; tmp_j++) {
                for (auto tmp_k : graph[tmp_j]) {
                    if (tmp_k == node_i) i_cnt += 1;
                }
            }
            my::assert(i_cnt == degree_counter[node_i], "Wrong degree counter.");
        }
        #endif

    }

    // ====================================================== //
    // =================== CPLEX VARIABLES ================== //
    // ====================================================== //

    // mylog.print_info("Adding variables to CPLEX.");

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

    // vertex elimination graph edges
    size_t veg_edges_start = curr_col;
    for (auto var_i : remaining_variables) {
        count = 0;
        for (auto var_j : remaining_variables) {
            objs[count] = 0;
            lbs[count] = 0;
            ubs[count] = cumulative_graph[var_i][var_j] ? 1 : 0;
            types[count++] = 'B';
        }
        #if HPLUS_INTCHECK
        my::assert(count == n_var, "Wrong number of veg variables while building the model.");
        #endif
        my::assert(!CPXnewcols(env, lp, n_var, objs, lbs, ubs, types, nullptr), "CPXnewcols (V.E. graph edges) failed.");
    }
    curr_col += n_var * n_var;

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
    auto get_fa_idx = [fa_start, &fa_individual_start](size_t act_idx, size_t var_idx) { return fa_start + fa_individual_start[HPLUS_inst.act_idx_post_simplification(act_idx)] + HPLUS_inst.var_idx_post_simplification(var_idx); };
    auto get_veg_idx = [veg_edges_start, n_var](size_t idx_i, size_t idx_j) { return veg_edges_start + HPLUS_inst.var_idx_post_simplification(idx_i) * n_var + HPLUS_inst.var_idx_post_simplification(idx_j); };

    int* ind = new int[n_act + 1];
    double* val = new double[n_act + 1];
    int nnz = 0;
    const char sense_e = 'E', sense_l = 'L';
    const double rhs_0 = 0, rhs_1 = 1;
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

    int ind_c5_c6_c7[2], ind_c8[3];
    double val_c5_c6_c7[2], val_c8[3];
    
    for (auto act_i : remaining_actions) {
        for (auto var_i : actions[act_i].get_eff()) if (remaining_var_set[var_i]) {
            ind_c5_c6_c7[0] = get_act_idx(act_i);
            val_c5_c6_c7[0] = -1;
            ind_c5_c6_c7[1] = get_fa_idx(act_i, var_i);
            val_c5_c6_c7[1] = 1;
            my::assert(!CPXaddrows(env, lp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr), "CPXaddrows (c5) faliled.");
        }
    }
    
    for (auto act_i : remaining_actions) {
        const auto& pre = actions[act_i].get_pre_sparse();
        const auto& eff = actions[act_i].get_eff_sparse();
        for (auto var_i : pre) if (remaining_var_set[var_i]) {
            for (auto var_j : eff) if (remaining_var_set[var_j]) {
                ind_c5_c6_c7[0] = get_veg_idx(var_i, var_j);
                val_c5_c6_c7[0] = -1;
                ind_c5_c6_c7[1] = get_fa_idx(act_i, var_j);
                val_c5_c6_c7[1] = 1;
                my::assert(!CPXaddrows(env, lp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr), "CPXaddrows (c6) faliled.");
            }
        }
    }
    
    for (auto var_i : remaining_variables) for (auto var_j : cumulative_graph[var_i]) {
        ind_c5_c6_c7[0] = get_veg_idx(var_i, var_j);
        val_c5_c6_c7[0] = 1;
        ind_c5_c6_c7[1] = get_veg_idx(var_j, var_i);
        val_c5_c6_c7[1] = 1;
        my::assert(!CPXaddrows(env, lp, 0, 1, 2, &rhs_1, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr), "CPXaddrows (c7) faliled.");
    }

    for (size_t h = 0; h < triangles_list.size(); h++) {
        const size_t i = triangles_list[h].first, j = triangles_list[h].second, k = triangles_list[h].third;
        ind_c8[0] = get_veg_idx(i, j);
        val_c8[0] = 1;
        ind_c8[1] = get_veg_idx(j, k);
        val_c8[1] = 1;
        ind_c8[2] = get_veg_idx(i, k);
        val_c8[2] = -1;
        my::assert(!CPXaddrows(env, lp, 0, 1, 3, &rhs_1, &sense_l, &begin, ind_c8, val_c8, nullptr, nullptr), "CPXaddrows (c8) faliled.");
    }

    // my::assert(!CPXwriteprob(env, lp, (HPLUS_CPLEX_OUT_DIR"/lp/"+HPLUS_env.run_name+".lp").c_str(), "LP"), "CPXwriteprob failed.");

    // mylog.print_info("Created CPLEX lp for rankooh.");

}

void HPLUS_cpx_post_warmstart_rankooh(CPXENVptr& env, CPXLPptr& lp) {
    
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
        for (auto var_i : actions[act_i].get_eff_sparse()) {
            cpx_sol_ind[nnz] = nact + nact * nvar + HPLUS_inst.var_idx_post_simplification(var_i);
            cpx_sol_val[nnz++] = 1;
            if (!current_state[var_i] && remaining_variables[var_i]) {
                cpx_sol_ind[nnz] = nact + HPLUS_inst.fa_idx_post_simplification(act_i, var_i);
                cpx_sol_val[nnz++] = 1;
            }
            current_state.add(var_i);
        }
    }

    my::assert(!CPXaddmipstarts(env, lp, 1, nnz, &izero, cpx_sol_ind, cpx_sol_val, &effortlevel, nullptr), "CPXaddmipstarts failed.");
    delete[] cpx_sol_ind; cpx_sol_ind = nullptr;
    delete[] cpx_sol_val; cpx_sol_val = nullptr;

}

void HPLUS_store_rankooh_sol(const CPXENVptr& env, const CPXLPptr& lp) {

    const size_t n_var = HPLUS_inst.get_n_var(true);
    const size_t n_act = HPLUS_inst.get_n_act(true);
    double* plan = new double[n_act + n_act * n_var];
    my::assert(!CPXgetx(env, lp, plan, 0, n_act + n_act * n_var - 1), "CPXgetx failed.");
    
    // fixing the solution to read the plan (some actions are set to 1 even if they are not a first archiever of anything)
    for (size_t act_i = 0, fadd_i = n_act; act_i < n_act; act_i++, fadd_i += n_var) {
        bool set_zero = true;
        for (size_t var_i = 0; var_i < n_var; var_i++) {
            if (plan[fadd_i + var_i] > 0.5) {
                #if HPLUS_INTCHECK
                my::assert(plan[act_i] > 0.5, "Action is set to 0 even if it's a first archiever.");
                #endif
                set_zero = false;
                break;
            }
        }
        if (set_zero) plan[act_i] = 0;
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