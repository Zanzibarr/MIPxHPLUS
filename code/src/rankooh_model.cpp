#include "../include/rankooh_model.hpp"

void HPLUS_cpx_build_rankooh(CPXENVptr& env, CPXLPptr& lp, const HPLUS_instance& inst) {

    const auto nvar = inst.get_nvar();
    const auto nact = inst.get_nact();
    const auto nvarstrips = inst.get_nvar_strips();
    const auto& actions = inst.get_actions();
    const auto& variables = inst.get_variables();

    // ~~~~~~~~~~ VERTEX ELIMINATION ~~~~~~~~~ //

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

    std::vector<std::set<unsigned int>> graph(nvarstrips);                                              // sparse graph for construction
    std::vector<my::BitField> cumulative_graph(nvarstrips, my::BitField(nvarstrips));                   // dense graph for fast access
    std::vector<Triangle> triangles_list;                                                               // list of triangles

    std::priority_queue<Node, std::vector<Node>, CompareNode> nodes_queue;                              // node priority
    std::vector<unsigned int> degree_counter(nvarstrips, 0);                                            // ordering

    // G_0
    for (unsigned int act_i = 0; act_i < nact; act_i++) {
        const auto& pre = actions[act_i].get_pre();
        const auto& eff = actions[act_i].get_eff();
        const auto& eff_sparse = actions[act_i].get_eff().sparse();
        for (auto p : pre) {
            for (auto q : eff_sparse) {
                int pre_size = graph[p].size();
                graph[p].insert(q);
                if (pre_size != graph[p].size()) {                                                      // if the edge is new update the degree
                    degree_counter[p] += 1;
                    degree_counter[q] += 1;
                }
            }
            cumulative_graph[p] |= eff;
        }
    }

    for (unsigned int node_i = 0; node_i < nvarstrips; node_i++) if (degree_counter[node_i] > 0) nodes_queue.push(Node(node_i, degree_counter[node_i]));

    // finding minimum degree node
    auto find_min = [](std::priority_queue<Node, std::vector<Node>, CompareNode>& nodes_queue, const std::vector<unsigned int> degree_counter) {

        int idx = -1;
        while (!nodes_queue.empty() && idx < 0) {
            Node tmp = nodes_queue.top();
            nodes_queue.pop();
            if (degree_counter[tmp.id] == tmp.deg) idx = tmp.id;
        }
        return idx;

    };

    // G_i (minimum degree heuristics)
    for (unsigned int i = 0; i < nvarstrips; i++) {

        int idx = find_min(nodes_queue, degree_counter);
        if (idx == -1) break;

        // graph structure:
        // | \       > |
        // p -> idx -> q
        // | /       > |

        std::set<unsigned int> new_nodes;

        // TODO
        for (unsigned int p = 0; p < nvarstrips; p++) if (graph[p].find(idx) != graph[p].end()) {

            new_nodes.insert(p);

            for (auto q : graph[idx]) {
    
                new_nodes.insert(q);
                
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

            // update triangles list
            for (auto q : graph[idx]) triangles_list.push_back(Triangle(p, idx, q));

        }

        // remove the edge idx - q
        for (auto q : graph[idx]) degree_counter[q] -= 1;
        graph[idx].clear();
        degree_counter[idx] = 0;
        
        // Update the priority queue
        for (auto node : new_nodes) if (degree_counter[node] > 0) nodes_queue.push(Node(node, degree_counter[node]));

    }

    // ~~~~~~~~~~~~ Enhanced model ~~~~~~~~~~~ //
    // (section 4 of Imai's paper)

    my::BitField eliminated_variables(nvarstrips);
    my::BitField fixed_variables(nvarstrips);
    my::BitField eliminated_actions(nact);
    my::BitField fixed_actions(nact);
    std::vector<my::BitField> eliminated_first_archievers(nact, my::BitField(nvarstrips));
    std::vector<my::BitField> fixed_first_archievers(nact, my::BitField(nvarstrips));
    std::vector<int> fixed_var_timestamps(nvarstrips, -1);
    std::vector<int> fixed_act_timestamps(nact, -1);
    std::vector<my::BitField> inverse_actions(nact, my::BitField(nact));

    inst.extract_imai_enhancements(eliminated_variables, fixed_variables, eliminated_actions, fixed_actions, inverse_actions, eliminated_first_archievers, fixed_first_archievers, fixed_var_timestamps, fixed_act_timestamps);

    fixed_variables |= inst.get_gstate();

    // ~~~~~~~~ Adding CPLEX variables ~~~~~~~ //

    unsigned int curr_col = 0;

    double* objs = new double[nact];
    double* lbs = new double[nact];
    double* ubs = new double[nact];
    char* types = new char[nact];

    auto rsz_cpx_arrays = [&objs, &lbs, &ubs, &types](int old_size, int new_size) {

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
    for (unsigned int act_i = 0; act_i < nact; act_i++) {
        objs[act_i] = actions[act_i].get_cost();
        lbs[act_i] = fixed_actions[act_i] ? 1 : 0;
        ubs[act_i] = eliminated_actions[act_i] ? 0 : 1;
        types[act_i] = 'B';
    }
    curr_col += nact;

    my::assert(!CPXnewcols(env, lp, nact, objs, lbs, ubs, types, nullptr), "CPXnewcols (actions) failed.");

    rsz_cpx_arrays(nact, nvarstrips);

    // ------- variables ------ //
    unsigned int var_start = curr_col;
    for (unsigned int i = 0; i < nvarstrips; i++) {
        objs[i] = 0;
        lbs[i] = fixed_variables[i] ? 1 : 0;
        ubs[i] = eliminated_variables[i] ? 0 : 1;
        types[i] = 'B';
    }
    curr_col += nvarstrips;

    my::assert(!CPXnewcols(env, lp, nvarstrips, objs, lbs, ubs, types, nullptr), "CPXnewcols (variables) failed.");

    rsz_cpx_arrays(nvarstrips, nact * nvarstrips);

    // --- first archievers --- //
    unsigned int fa_start = curr_col;
    std::vector<unsigned int> fa_individual_start(nact);
    for (unsigned int act_i = 0, nfa = 0; act_i < nact; act_i++) {
        fa_individual_start[act_i] = nfa;
        const my::BitField& eff = actions[act_i].get_eff();
        for (unsigned int i = 0; i < nvarstrips; i++) {
            objs[nfa] = 0;
            lbs[nfa] = fixed_first_archievers[act_i][i] ? 1 : 0;
            ubs[nfa] = (!eff[i] || eliminated_first_archievers[act_i][i]) ? 0 : 1;
            types[nfa++] = 'B';
        }
    }
    curr_col += nact * nvarstrips;

    my::assert(!CPXnewcols(env, lp, nact * nvarstrips, objs, lbs, ubs, types, nullptr), "CPXnewcols (first archievers) failed.");

    rsz_cpx_arrays(nact * nvarstrips, nvarstrips * nvarstrips);

    // vertex elimination graph edges
    unsigned int veg_edges_start = curr_col;
    for (unsigned int i = 0; i < nvarstrips; i++) {
        for (unsigned int j = 0; j < nvarstrips; j++) {
            objs[i * nvarstrips + j] = 0;
            lbs[i * nvarstrips + j] = 0;
            ubs[i * nvarstrips + j] = cumulative_graph[i][j] ? 1 : 0;
            types[i * nvarstrips + j] = 'B';
        }
    }
    curr_col += nvarstrips * nvarstrips;

    my::assert(!CPXnewcols(env, lp, nvarstrips * nvarstrips, objs, lbs, ubs, types, nullptr), "CPXnewcols (V.E. graph edges) failed.");

    delete[] types; types = nullptr;
    delete[] ubs; ubs = nullptr;
    delete[] lbs; lbs = nullptr;
    delete[] objs; objs = nullptr;

    // // ~~~~~~~ Adding CPLEX constraints ~~~~~~ //

    // accessing cplex variables
    auto get_act_idx = [act_start](int idx) { return act_start + idx; };
    auto get_var_idx = [var_start](int idx) { return var_start + idx; };
    auto get_fa_idx = [fa_start, fa_individual_start](int act_idx, int var_idx) { return fa_start + fa_individual_start[act_idx] + var_idx; };
    auto get_veg_idx = [veg_edges_start, nvarstrips](int idx_i, int idx_j) { return veg_edges_start + idx_i * nvarstrips + idx_j; };

    int* ind = new int[nact + 1];
    double* val = new double[nact + 1];
    int nnz = 0;
    const char sense_e = 'E', sense_l = 'L';
    const double rhs_0 = 0, rhs_1 = 1;
    const int begin = 0;

    // precompute list of actions that have a specific variable as effect (some sort of hashing)
    std::vector<std::vector<unsigned int>> act_with_eff(nvarstrips);
    for (unsigned int p = 0; p < nvarstrips; p++) for (unsigned int act_i = 0; act_i < nact; act_i++) if (actions[act_i].get_eff()[p]) act_with_eff[p].push_back(act_i);

    for (unsigned int p = 0; p < nvarstrips; p++) {

        nnz = 0;
        ind[nnz] = get_var_idx(p);
        val[nnz++] = 1;

        for (auto act_i : act_with_eff[p]) {
            ind[nnz] = get_fa_idx(act_i, p);
            val[nnz++] = -1;
        }

        my::assert(!CPXaddrows(env, lp, 0, 1, nnz, &rhs_0, &sense_e, &begin, ind, val, nullptr, nullptr), "CPXaddwors (c2) failed.");

    }

    for (unsigned int p = 0; p < nvarstrips; p++) {
        for (unsigned int q = 0; q < nvarstrips; q++) {
            nnz = 0;
            ind[nnz] = get_var_idx(q);
            val[nnz++] = -1;
            for (auto act_i : act_with_eff[p]) {
                if (actions[act_i].get_pre()[q]) {
                    ind[nnz] = get_fa_idx(act_i, p);
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

    for (unsigned int act_i = 0; act_i < nact; act_i++) {
        for (auto p : actions[act_i].get_eff()) {
            ind_c5_c6_c7[0] = get_act_idx(act_i);
            val_c5_c6_c7[0] = -1;
            ind_c5_c6_c7[1] = get_fa_idx(act_i, p);
            val_c5_c6_c7[1] = 1;
            my::assert(!CPXaddrows(env, lp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr), "CPXaddrows (c5) faliled.");
        }
    }

    for (unsigned int act_i = 0; act_i < nact; act_i++) {
        const auto& eff = actions[act_i].get_eff();
        for (auto i : actions[act_i].get_pre()) {
            for (auto j : eff) {
                ind_c5_c6_c7[0] = get_veg_idx(i, j);
                val_c5_c6_c7[0] = -1;
                ind_c5_c6_c7[1] = get_fa_idx(act_i, j);
                val_c5_c6_c7[1] = 1;
                my::assert(!CPXaddrows(env, lp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr), "CPXaddrows (c6) faliled.");
            }
        }
    }

    for (unsigned int i = 0; i < nvarstrips; i++) for (auto j : cumulative_graph[i]) {
        ind_c5_c6_c7[0] = get_veg_idx(i, j);
        val_c5_c6_c7[0] = 1;
        ind_c5_c6_c7[1] = get_veg_idx(j, i);
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

    unsigned int nact = inst.get_nact();
    double* plan = new double[nact];
    my::assert(!CPXgetx(env, lp, plan, 0, nact-1), "CPXgetx failed.");                  // TODO: Post processing

    // convert to std collections for easier parsing
    std::vector<unsigned int> cpx_result;
    for (unsigned int i = 0; i < nact; i++) if (plan[i] > .5) cpx_result.push_back(i);
    delete[] plan; plan = nullptr;

    const auto& actions = inst.get_actions();
    std::vector<unsigned int> solution;
    my::BitField sorted(cpx_result.size());
    my::BitField current_state(inst.get_nvar_strips());

    // int cost = 0;
    // for (auto act_i : cpx_result) cost += actions[act_i].get_cost();
    // HPLUS_env.logger.print_info("%d.", cost);
    
    // while (sorted != my::BitField(cpx_result.size(), true)) {
    while (!current_state.contains(inst.get_gstate())) {
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