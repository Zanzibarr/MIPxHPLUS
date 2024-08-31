#include "../include/rankooh_model.hpp"

void HPLUS_cpx_build_rankooh(CPXENVptr& env, CPXLPptr& lp, const HPLUS_instance& inst) {

    const auto nvar = inst.get_nvar();
    const auto nact = inst.get_nact();
    const auto nvarstrips = inst.get_nvar_strips();
    const auto& actions = inst.get_actions();
    const auto& variables = inst.get_variables();
    const auto& istate = inst.get_istate();

    // ~~~~~~~~~~ VERTEX ELIMINATION ~~~~~~~~~ //

    lprint_info("Vertex elimination from Rankooh's paper.");

    std::vector<my::BitField> graph(nvarstrips, my::BitField(nvarstrips));
    std::vector<my::BitField> cumulative_graph(nvarstrips, my::BitField(nvarstrips));
    std::vector<triple<int, int, int>> triples_list;

    // G_0
    for (unsigned int act_i = 0; act_i < nact; act_i++) {
        const auto& pre = actions[act_i].get_pre();
        const auto& eff = actions[act_i].get_eff();
        for (auto p : pre) {
            graph[p] |= eff;
            cumulative_graph[p] |= eff;
        }
    }

    auto find_min = [nvarstrips](std::vector<my::BitField> graph) {                 // TODO: Too slow

        int min = INT_MAX, min_idx = -1;
        for (unsigned int i = 0; i < nvarstrips; i++) {
            int count = 0;
            for (unsigned int j = 0; j < nvarstrips; j++) {
                if (graph[j][i]) count++;       // ingoing
                if (graph[i][j]) count++;       // outgoing
            }
            if (count < min && count != 0) {
                min = count;
                min_idx = i;
            }
        }

        return min_idx;

    };

    // G_i (minimum degree heuristics)
    for (unsigned int i = 0; i < nvarstrips; i++) {
        int idx = find_min(graph);
        if (idx == -1) break;
        for (unsigned int p = 0; p < nvarstrips; p++) if (graph[p][idx]) {
            graph[p] |= graph[idx];
            cumulative_graph[p] |= graph[idx];
            graph[p].unset(idx);
            for (auto q : graph[idx]) triples_list.push_back(std::make_tuple(p, idx, q));
        }
        graph[idx].clear();
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

    eliminated_variables |= istate;
    fixed_variables |= inst.get_gstate();
    fixed_variables -= istate;

    // ~~~~~~~~ Adding CPLEX variables ~~~~~~~ //

    lprint_info("(debug) Adding actions to the model.");

    unsigned int curr_col = 0;

    double* objs = new double[nact];
    double* lbs = new double[nact];
    double* ubs = new double[nact];
    char* types = new char[nact];
    char** names = new char*[nact];
    for (unsigned int i = 0; i < nact; i++) names[i] = new char[20];

    auto rsz_cpx_arrays = [&objs, &lbs, &ubs, &types, &names](int old_size, int new_size) {

        for (unsigned int i = 0; i < old_size; i++) delete[] names[i];
        delete[] names; names = nullptr;
        delete[] types; types = nullptr;
        delete[] ubs; ubs = nullptr;
        delete[] lbs; lbs = nullptr;
        delete[] objs; objs = nullptr;

        objs = new double[new_size];
        lbs = new double[new_size];
        ubs = new double[new_size];
        types = new char[new_size];
        names = new char*[new_size];
        for (unsigned int i = 0; i < new_size; i++) names[i] = new char[20];

    };

    // actions
    unsigned int act_start = curr_col;
    for (unsigned int act_i = 0; act_i < nact; act_i++) {
        objs[act_i] = actions[act_i].get_cost();
        lbs[act_i] = fixed_actions[act_i] ? 1 : 0;
        ubs[act_i] = eliminated_actions[act_i] ? 0 : 1;
        types[act_i] = 'B';
        snprintf(names[act_i], 20, "act(%d)", act_i);
    }
    curr_col += nact;

    my::assert(!CPXnewcols(env, lp, nact, objs, lbs, ubs, types, names), "CPXnewcols (actions) failed.");

    rsz_cpx_arrays(nact, nvarstrips);

    lprint_info("(debug) Adding variables to the model.");

    // variables
    unsigned int var_start = curr_col;
    for (unsigned int i = 0, count = 0; i < nvar; i++) for (unsigned int j = 0; j < variables[i].get_range(); j++, count++) {
        objs[count] = 0;
        lbs[count] = fixed_variables[count] ? 1 : 0;
        ubs[count] = eliminated_variables[count] ? 0 : 1;
        types[count] = 'B';
        snprintf(names[count], 20, "var(%d_%d)", i, j);
    }
    curr_col += nvarstrips;

    my::assert(!CPXnewcols(env, lp, nvarstrips, objs, lbs, ubs, types, names), "CPXnewcols (variables) failed.");

    rsz_cpx_arrays(nvarstrips, nact * nvarstrips);

    lprint_info("(debug) Adding first adders to the model.");

    // first archievers
    unsigned int fa_start = curr_col;
    std::vector<unsigned int> fa_individual_start(nact);
    for (unsigned int act_i = 0, nfa = 0; act_i < nact; act_i++) {
        fa_individual_start[act_i] = nfa;
        const my::BitField& eff = actions[act_i].get_eff();
        for (unsigned int i = 0, k = 0; i < nvar; k += variables[i].get_range(), i++) {
            for (unsigned int j = 0; j < variables[i].get_range(); j++) {
                objs[nfa] = 0;
                lbs[nfa] = fixed_first_archievers[act_i][k+j] ? 1 : 0;
                ubs[nfa] = (!eff[k+j] || eliminated_first_archievers[act_i][k+j]) ? 0 : 1;          // I create a first archiever variable for each pair action-variable, but I already fix at 0 the pairs that are not action - effect (easier to find the variables later -- cplex simplifies the model on his own)
                types[nfa] = 'B';
                snprintf(names[nfa], 20, "fa(%d_%d_%d)", act_i, i, j);
                nfa++;
            }
        }
    }
    curr_col += nact * nvarstrips;

    my::assert(!CPXnewcols(env, lp, nact * nvarstrips, objs, lbs, ubs, types, names), "CPXnewcols (first archievers) failed.");

    rsz_cpx_arrays(nact * nvarstrips, nvarstrips * nvarstrips);

    lprint_info("(debug) Adding vertex elimination variables to the model.");

    // vertex elimination graph edges
    unsigned int veg_edges_start = curr_col;
    for (unsigned int i = 0; i < nvarstrips; i++) {
        for (unsigned int j = 0; j < nvarstrips; j++) {
            objs[i * nvarstrips + j] = 0;
            lbs[i * nvarstrips + j] = 0;
            ubs[i * nvarstrips + j] = cumulative_graph[i][j] ? 1 : 0;
            types[i * nvarstrips + j] = 'B';
            snprintf(names[i * nvarstrips + j], 20, "e(%d_%d)", i, j);
        }
    }
    curr_col += nvarstrips * nvarstrips;

    my::assert(!CPXnewcols(env, lp, nvarstrips * nvarstrips, objs, lbs, ubs, types, names), "CPXnewcols (V.E. graph edges) failed.");

    for (unsigned int i = 0; i < nvarstrips * nvarstrips; i++) delete[] names[i];
    delete[] names; names = nullptr;
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

    lprint_info("(debug) Adding c2 to the model.");

    for (auto p : !istate) {

        nnz = 0;
        ind[nnz] = get_var_idx(p);
        val[nnz++] = 1;

        for (unsigned int i = 0; i < nact; i++) {
            if ((actions[i].get_eff() - istate)[p]) {
                ind[nnz] = get_fa_idx(i, p);
                val[nnz++] = -1;
            }
        }

        my::assert(!CPXaddrows(env, lp, 0, 1, nnz, &rhs_0, &sense_e, &begin, ind, val, nullptr, nullptr), "CPXaddwors (c2) failed.");

    }

    lprint_info("(debug) Adding c3 to the model.");

    for (auto p : !istate) {                                                    // TODO: Too slow
        for (auto q : !istate) {
            nnz = 0;
            ind[nnz] = get_var_idx(q);
            val[nnz++] = -1;
            for (unsigned int act_i = 0; act_i < nact; act_i++) {
                if ((actions[act_i].get_pre() - istate)[q] && (actions[act_i].get_eff() - istate)[p]) {
                    ind[nnz] = get_fa_idx(act_i, p);
                    val[nnz++] = 1;
                }
            }
            my::assert(!CPXaddrows(env, lp, 0, 1, nnz, &rhs_0, &sense_l, &begin, ind, val, nullptr, nullptr), "CPXaddwors (c3) failed.");
        }
    }

    lprint_info("(debug) Adding c5 to the model.");

    delete[] val; val = nullptr;
    delete[] ind; ind = nullptr;

    int ind_c5_c6_c7[2], ind_c8[3];
    double val_c5_c6_c7[2], val_c8[3];

    for (unsigned int act_i = 0; act_i < nact; act_i++) {
        for (auto p : actions[act_i].get_eff() - istate) {
            ind_c5_c6_c7[0] = get_act_idx(act_i);
            val_c5_c6_c7[0] = -1;
            ind_c5_c6_c7[1] = get_fa_idx(act_i, p);
            val_c5_c6_c7[1] = 1;
            my::assert(!CPXaddrows(env, lp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr), "CPXaddrows (c5) faliled.");
        }
    }

    lprint_info("(debug) Adding c6 to the model.");

    for (unsigned int act_i = 0; act_i < nact; act_i++) {
        const auto& eff = actions[act_i].get_eff() - istate;
        for (auto i : actions[act_i].get_pre() - istate) {
            for (auto j : eff) {
                ind_c5_c6_c7[0] = get_veg_idx(i, j);
                val_c5_c6_c7[0] = -1;
                ind_c5_c6_c7[1] = get_fa_idx(act_i, j);
                val_c5_c6_c7[1] = 1;
                my::assert(!CPXaddrows(env, lp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr), "CPXaddrows (c6) faliled.");
            }
        }
    }

    lprint_info("(debug) Adding c7 to the model.");

    for (auto i : !istate) for (auto j : cumulative_graph[i] & !istate) {
        ind_c5_c6_c7[0] = get_veg_idx(i, j);
        val_c5_c6_c7[0] = 1;
        ind_c5_c6_c7[1] = get_veg_idx(j, i);
        val_c5_c6_c7[1] = 1;
        my::assert(!CPXaddrows(env, lp, 0, 1, 2, &rhs_1, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr), "CPXaddrows (c7) faliled.");
    }

    lprint_info("(debug) Adding c8 to the model.");
    HPLUS_env.logger.print_info("(debug) %d.", triples_list.size());

    for (unsigned int h = 0; h < triples_list.size(); h++) {                    // TODO: Too slow
        const int i = std::get<0>(triples_list[h]), j = std::get<1>(triples_list[h]), k = std::get<2>(triples_list[h]);
        if (istate[i] || istate[j] || istate[k]) continue;
        ind_c8[0] = get_veg_idx(i, j);
        val_c8[0] = 1;
        ind_c8[1] = get_veg_idx(j, k);
        val_c8[1] = 1;
        ind_c8[2] = get_veg_idx(i, k);
        val_c8[2] = -1;
        my::assert(!CPXaddrows(env, lp, 0, 1, 3, &rhs_1, &sense_l, &begin, ind_c8, val_c8, nullptr, nullptr), "CPXaddrows (c8) faliled.");
    }

    my::assert(!CPXwriteprob(env, lp, (HPLUS_CPLEX_OUT_DIR"/lp/"+HPLUS_env.run_name+".lp").c_str(), "LP"), "CPXwriteprob failed.");

    lprint_info("Created CPLEX lp for rankooh.");

}

void HPLUS_store_rankooh_sol(const CPXENVptr& env, const CPXLPptr& lp, HPLUS_instance& inst) {

    unsigned int nact = inst.get_nact();
    double* plan = new double[nact];
    my::assert(!CPXgetx(env, lp, plan, 0, nact-1), "CPXgetx failed.");

    // convert to std collections for easier parsing
    std::vector<unsigned int> cpx_result;
    for (unsigned int i = 0; i < nact; i++) if (plan[i] > .5) cpx_result.push_back(i);
    delete[] plan; plan = nullptr;

    const auto& actions = inst.get_actions();
    std::vector<unsigned int> solution;
    my::BitField sorted(cpx_result.size());
    my::BitField current_state = inst.get_istate();

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