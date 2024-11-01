#include "../include/rankooh_model.hpp"

void HPLUS_cpx_build_rankooh(CPXENVptr& env, CPXLPptr& lp, HPLUS_instance& inst) {

    const auto nact = inst.get_nact();
    const auto nvar = inst.get_nvar();
    const auto nvar_opt = inst.get_nvar_opt();
    const auto nact_opt = inst.get_nact_opt();
    const auto& eliminated_var = inst.get_model_opt_eliminated_var();
    const auto& eliminated_act = inst.get_model_opt_eliminated_act();
    std::vector<unsigned int> remaining_var_sparse = (!eliminated_var).sparse();
    std::vector<unsigned int> remaining_act_sparse = (!eliminated_act).sparse();
    const auto& actions = inst.get_actions();

    // ====================================================== //
    // ================= VERTEX ELIMINATION ================= //
    // ====================================================== //

    lprint_info("Vertex elimination from Rankooh's paper.");

    struct Node {
        unsigned int deg, id;
        Node(unsigned int i, unsigned int d) : deg(d), id(i) {}
    };

    struct CompareNode { bool operator()(const Node& n1, const Node& n2) { return n1.deg > n2.deg; } };

    struct Triangle {
        unsigned int first, second, third;
        Triangle(unsigned int f, unsigned int s, unsigned int t) : first(f), second(s), third(t) {}
    };

    std::vector<std::set<unsigned int>> graph(nvar);                                                // sparse graph for construction
    std::vector<my::BitField> cumulative_graph(nvar, my::BitField(nvar));                           // dense graph for fast access
    std::vector<Triangle> triangles_list;                                                           // list of triangles

    std::priority_queue<Node, std::vector<Node>, CompareNode> nodes_queue;                          // node priority
    std::vector<unsigned int> degree_counter(nvar, 0);                                              // ordering

    // G_0
    for (auto act_i : remaining_act_sparse) {
        const auto pre = actions[act_i].get_pre() & !eliminated_var;
        const auto eff = actions[act_i].get_eff() & !eliminated_var;
        const auto eff_sparse = eff.sparse();
        for (auto var_i : pre) {
            for (auto var_j : eff_sparse) if (var_i != var_j) {
                int pre_size = graph[var_i].size();
                graph[var_i].insert(var_j);
                if (pre_size != graph[var_i].size()) {                                              // if the edge is new update the degree
                    degree_counter[var_i] += 1;
                    degree_counter[var_j] += 1;
                }
            }
            cumulative_graph[var_i] |= eff;
        }
    }

    for (unsigned int node_i = 0; node_i < nvar; node_i++) if (degree_counter[node_i] > 0) nodes_queue.emplace(node_i, degree_counter[node_i]);

    // finding minimum degree node
    auto find_min = [&nvar](std::priority_queue<Node, std::vector<Node>, CompareNode>& nodes_queue, const std::vector<unsigned int> degree_counter) {

        int idx = -1;
        while (!nodes_queue.empty() && idx < 0) {
            Node tmp = nodes_queue.top();
            nodes_queue.pop();
            if (degree_counter[tmp.id] == tmp.deg) idx = tmp.id;
        }
        return idx;

    };

    // G_i (minimum degree heuristics)
    for (auto _ : remaining_var_sparse) {

        int idx = find_min(nodes_queue, degree_counter);
        if (idx == -1) break;

        // graph structure:
        // | \       > |
        // p -> idx -> q
        // | /       > |

        std::set<unsigned int> new_nodes;

        for (auto p : remaining_var_sparse) if (graph[p].find(idx) != graph[p].end()) {

            for (auto q : graph[idx]) if (p != q) {

                // add edge p - q
                int pre_size = graph[p].size();
                graph[p].insert(q);
                if (pre_size != graph[p].size()) {
                    degree_counter[p] += 1;
                    degree_counter[q] += 1;
                }

                // update the overall graph
                cumulative_graph[p].set(q);
            
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
        for (unsigned int node_i = 0; node_i < nvar; node_i++) {
            int i_cnt = 0;
            i_cnt += graph[node_i].size();
            for (unsigned int tmp_j = 0; tmp_j < nvar; tmp_j++) {
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

    lprint_info("Adding variables to CPLEX.");

    unsigned int curr_col = 0;
    std::vector<int> varidx_to_cpxidx(nvar, -1);
    std::vector<int> actidx_to_cpxidx(nact, -1);

    double* objs = new double[nact_opt];
    double* lbs = new double[nact_opt];
    double* ubs = new double[nact_opt];
    char* types = new char[nact_opt];

    auto resize_cpx_arrays = [&objs, &lbs, &ubs, &types](int new_size) {

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
    unsigned int act_start = curr_col;
    int count = 0;
    for (auto act_i : remaining_act_sparse) {
        actidx_to_cpxidx[act_i] = count;
        objs[count] = actions[act_i].get_cost();
        lbs[count] = inst.model_opt_fixed_act[act_i] ? 1 : 0;
        ubs[count] = 1;
        types[count++] = 'B';
    }
    curr_col += nact_opt;

    my::assert(!CPXnewcols(env, lp, nact_opt, objs, lbs, ubs, types, nullptr), "CPXnewcols (actions) failed.");

    resize_cpx_arrays(nvar_opt);

    // --- first archievers --- //
    unsigned int fa_start = curr_col;
    std::vector<unsigned int> fa_individual_start(nact_opt);
    count = 0;
    for (auto act_i : remaining_act_sparse) {
        fa_individual_start[count] = count * nvar_opt;
        int count_var = 0;
        for (auto var_i : remaining_var_sparse) {
            objs[count_var] = 0;
            lbs[count_var] = inst.model_opt_fixed_fa[act_i][var_i] ? 1 : 0;
            ubs[count_var] = (!actions[act_i].get_eff()[var_i] || inst.model_opt_eliminated_fa[act_i][var_i]) ? 0 : 1;
            types[count_var++] = 'B';
        }
        curr_col += nvar_opt;
        my::assert(!CPXnewcols(env, lp, nvar_opt, objs, lbs, ubs, types, nullptr), "CPXnewcols (first archievers) failed.");
        count++;
    }

    // ------- variables ------ //
    unsigned int var_start = curr_col;
    const auto& gstate = inst.get_gstate();
    count = 0;
    for (auto var_i : remaining_var_sparse) {
        varidx_to_cpxidx[var_i] = count;
        objs[count] = 0;
        lbs[count] = (inst.model_opt_fixed_var[var_i] || gstate[var_i]) ? 1 : 0;
        ubs[count] = 1;
        types[count++] = 'B';
    }
    curr_col += nvar_opt;

    my::assert(!CPXnewcols(env, lp, nvar_opt, objs, lbs, ubs, types, nullptr), "CPXnewcols (variables) failed.");

    // vertex elimination graph edges
    unsigned int veg_edges_start = curr_col;
    for (auto var_i : remaining_var_sparse) {
        count = 0;
        for (auto var_j : remaining_var_sparse) {
            objs[count] = 0;
            lbs[count] = 0;
            ubs[count] = cumulative_graph[var_i][var_j] ? 1 : 0;
            types[count++] = 'B';
        }
        my::assert(!CPXnewcols(env, lp, nvar_opt, objs, lbs, ubs, types, nullptr), "CPXnewcols (V.E. graph edges) failed.");
    }
    curr_col += nvar_opt * nvar_opt;

    delete[] types; types = nullptr;
    delete[] ubs; ubs = nullptr;
    delete[] lbs; lbs = nullptr;
    delete[] objs; objs = nullptr;

    // ====================================================== //
    // ================== CPLEX CONSTRAINTS ================= //
    // ====================================================== //

    lprint_info("Adding constraints to CPLEX.");

    // accessing cplex variables
    int new_nvar = nvar_opt;
    auto get_act_idx = [act_start, &actidx_to_cpxidx](int idx) { return act_start + actidx_to_cpxidx[idx]; };
    auto get_var_idx = [var_start, &varidx_to_cpxidx](int idx) { return var_start + varidx_to_cpxidx[idx]; };
    auto get_fa_idx = [fa_start, fa_individual_start, &varidx_to_cpxidx, &actidx_to_cpxidx](int act_idx, int var_idx) { return fa_start + fa_individual_start[actidx_to_cpxidx[act_idx]] + varidx_to_cpxidx[var_idx]; };
    auto get_veg_idx = [veg_edges_start, new_nvar, &varidx_to_cpxidx](int idx_i, int idx_j) { return veg_edges_start + varidx_to_cpxidx[idx_i] * new_nvar + varidx_to_cpxidx[idx_j]; };

    int* ind = new int[nact_opt + 1];
    double* val = new double[nact_opt + 1];
    int nnz = 0;
    const char sense_e = 'E', sense_l = 'L';
    const double rhs_0 = 0, rhs_1 = 1;
    const int begin = 0;
    
    // precompute list of actions that have a specific variable as effect (some sort of hashing)
    std::vector<std::vector<unsigned int>> act_with_eff(nvar);
    for (auto var_i : remaining_var_sparse) for (auto act_i : remaining_act_sparse) if (actions[act_i].get_eff()[var_i]) act_with_eff[var_i].push_back(act_i);

    for (auto var_i : remaining_var_sparse) {

        nnz = 0;
        ind[nnz] = get_var_idx(var_i);
        val[nnz++] = 1;

        for (auto act_i : act_with_eff[var_i]) {
            ind[nnz] = get_fa_idx(act_i, var_i);
            val[nnz++] = -1;
        }

        my::assert(!CPXaddrows(env, lp, 0, 1, nnz, &rhs_0, &sense_e, &begin, ind, val, nullptr, nullptr), "CPXaddwors (c2) failed.");

    }
    
    for (auto var_i : remaining_var_sparse) {
        for (auto var_j : remaining_var_sparse) {
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
    
    for (auto act_i : remaining_act_sparse) {
        for (auto var_i : actions[act_i].get_eff()) if (!eliminated_var[var_i]) {
            ind_c5_c6_c7[0] = get_act_idx(act_i);
            val_c5_c6_c7[0] = -1;
            ind_c5_c6_c7[1] = get_fa_idx(act_i, var_i);
            val_c5_c6_c7[1] = 1;
            my::assert(!CPXaddrows(env, lp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr), "CPXaddrows (c5) faliled.");
        }
    }
    
    for (auto act_i : remaining_act_sparse) {
        const auto& pre = actions[act_i].get_pre_sparse();
        const auto& eff = actions[act_i].get_eff_sparse();
        for (auto var_i : pre) if (!eliminated_var[var_i]) {
            for (auto var_j : eff) if (!eliminated_var[var_j]) {
                ind_c5_c6_c7[0] = get_veg_idx(var_i, var_j);
                val_c5_c6_c7[0] = -1;
                ind_c5_c6_c7[1] = get_fa_idx(act_i, var_j);
                val_c5_c6_c7[1] = 1;
                my::assert(!CPXaddrows(env, lp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr), "CPXaddrows (c6) faliled.");
            }
        }
    }
    
    for (auto var_i : remaining_var_sparse) for (auto var_j : cumulative_graph[var_i]) {
        ind_c5_c6_c7[0] = get_veg_idx(var_i, var_j);
        val_c5_c6_c7[0] = 1;
        ind_c5_c6_c7[1] = get_veg_idx(var_j, var_i);
        val_c5_c6_c7[1] = 1;
        my::assert(!CPXaddrows(env, lp, 0, 1, 2, &rhs_1, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr), "CPXaddrows (c7) faliled.");
    }
    
    for (unsigned int h = 0; h < triangles_list.size(); h++) {
        const int i = triangles_list[h].first, j = triangles_list[h].second, k = triangles_list[h].third;
        ind_c8[0] = get_veg_idx(i, j);
        val_c8[0] = 1;
        ind_c8[1] = get_veg_idx(j, k);
        val_c8[1] = 1;
        ind_c8[2] = get_veg_idx(i, k);
        val_c8[2] = -1;
        my::assert(!CPXaddrows(env, lp, 0, 1, 3, &rhs_1, &sense_l, &begin, ind_c8, val_c8, nullptr, nullptr), "CPXaddrows (c8) faliled.");
    }

    // my::assert(!CPXwriteprob(env, lp, (HPLUS_CPLEX_OUT_DIR"/lp/"+HPLUS_env.run_name+".lp").c_str(), "LP"), "CPXwriteprob failed.");

    lprint_info("Created CPLEX lp for rankooh.");

}

void HPLUS_store_rankooh_sol(const CPXENVptr& env, const CPXLPptr& lp, HPLUS_instance& inst) {

    const unsigned int nvar = inst.get_nvar_opt();
    const unsigned int nact = inst.get_nact_opt();
    double* plan = new double[nact + nact * nvar];
    my::assert(!CPXgetx(env, lp, plan, 0, nact + nact * nvar - 1), "CPXgetx failed.");
    
    // fixing the solution to read the plan (some actions are set to 1 even if they are not a first archiever of anything)
    for (unsigned int act_i = 0, f_i = nact; act_i < nact; act_i++, f_i += nvar) {
        bool set_zero = true;
        for (unsigned int var_i = 0; var_i < nvar; var_i++) {
            if (plan[f_i + var_i] > 0.5) {
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
    std::vector<unsigned int> cpx_result;
    for (unsigned int i = 0; i < nact; i++) if (plan[i] > .5) cpx_result.push_back(inst.cpxidx_to_actidx(i));
    delete[] plan; plan = nullptr;

    const auto& actions = inst.get_actions();
    std::vector<unsigned int> solution;
    my::BitField sorted(cpx_result.size());
    my::BitField current_state(inst.get_nvar());
    
    while (sorted != my::BitField(cpx_result.size(), true)) {
        #if HPLUS_INTCHECK
        bool intcheck = false;
        #endif
        for (auto i : !sorted) {
            if (current_state.contains(actions[cpx_result[i]].get_pre())) {
                sorted.set(i);
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
    inst.update_best_solution(solution,
        std::accumulate(solution.begin(), solution.end(), 0,
            [&actions](const unsigned int acc, const unsigned int index) {
                return acc + actions[index].get_cost();
            }
        )
    );

}