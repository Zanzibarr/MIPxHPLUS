#include "../include/rankooh_model.hpp"

void HPLUS_cpx_build_rankooh(CPXENVptr& env, CPXLPptr& lp, const HPLUS_instance& inst) {

    const auto nvar = inst.get_nvar();
    const auto nact = inst.get_nact();
    const auto nvarstrips = inst.get_nvar_strips();
    const auto& actions = inst.get_actions();
    const auto& variables = inst.get_variables();
    const auto& istate = inst.get_istate();

    // ~~~~~~~~~~ VERTEX ELIMINATION ~~~~~~~~~ //

    HPLUS_env.logger.print_info("Vertex elimination from Rankooh's paper.");

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

    // G_i
    for (unsigned int i = 0; i < nvarstrips; i++) {
        for (unsigned int p = 0; p < nvarstrips; p++) if (graph[p][i]) {
            graph[p] |= graph[i];
            cumulative_graph[p] |= graph[i];
            graph[p].unset(i);
            for (auto q : graph[i]) {
                triples_list.push_back(std::make_tuple(p, i, q));
            }
        }
        graph[i].clear();
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

    HPLUS_env.logger.print_info("Adding actions to the model.");

    unsigned int curr_col = 0;

    double* objs = new double[nact];
    double* lbs = new double[nact];
    double* ubs = new double[nact];
    char* types = new char[nact];
    char** names = new char*[nact];
    for (unsigned int i = 0; i < nact; i++) names[i] = new char[20];

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

    for (unsigned int i = 0; i < nact; i++) delete[] names[i];
    delete[] names; names = nullptr;
    delete[] types; types = nullptr;
    delete[] ubs; ubs = nullptr;
    delete[] lbs; lbs = nullptr;
    delete[] objs; objs = nullptr;

    HPLUS_env.logger.print_info("Adding variables to the model.");

    objs = new double[nvarstrips];
    lbs = new double[nvarstrips];
    ubs = new double[nvarstrips];
    types = new char[nvarstrips];
    names = new char*[nvarstrips];
    for (unsigned int i = 0; i < nvarstrips; i++) names[i] = new char[20];

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

    for (unsigned int i = 0; i < nvarstrips; i++) delete[] names[i];
    delete[] names; names = nullptr;
    delete[] types; types = nullptr;
    delete[] ubs; ubs = nullptr;
    delete[] lbs; lbs = nullptr;
    delete[] objs; objs = nullptr;

    HPLUS_env.logger.print_info("Adding first adders to the model.");

    objs = new double[nact * nvarstrips];
    lbs = new double[nact * nvarstrips];
    ubs = new double[nact * nvarstrips];
    types = new char[nact * nvarstrips];
    names = new char*[nact * nvarstrips];
    for (unsigned int i = 0; i < nact * nvarstrips; i++) names[i] = new char[20];

    // first archievers
    unsigned int nfa = 0;
    unsigned int fa_start = curr_col;
    std::vector<unsigned int> fa_individual_start(nact);
    for (unsigned int act_i = 0; act_i < nact; act_i++) {
        fa_individual_start[act_i] = nfa;
        const my::BitField& eff = actions[act_i].get_eff();
        for (unsigned int i = 0, k = 0; i < nvar; k += variables[i].get_range(), i++) {
            for (unsigned int j = 0; j < variables[i].get_range(); j++) if (eff[k + j] && !istate[k + j]) {
                objs[nfa] = 0;
                lbs[nfa] = fixed_first_archievers[act_i][k+j] ? 1 : 0;
                ubs[nfa] = eliminated_first_archievers[act_i][k+j] ? 0 : 1;
                types[nfa] = 'B';
                snprintf(names[nfa], 20, "fa(%d_%d_%d)", act_i, i, j);
                nfa++;
            }
        }
    }
    curr_col += nfa;

    my::assert(!CPXnewcols(env, lp, nfa, objs, lbs, ubs, types, names), "CPXnewcols (first archievers) failed.");

    for (unsigned int i = 0; i < nact * nvarstrips; i++) delete[] names[i];
    delete[] names; names = nullptr;
    delete[] types; types = nullptr;
    delete[] ubs; ubs = nullptr;
    delete[] lbs; lbs = nullptr;
    delete[] objs; objs = nullptr;

    HPLUS_env.logger.print_info("Adding vertex elimination variables to the model.");

    objs = new double[nvarstrips * nvarstrips];
    lbs = new double[nvarstrips * nvarstrips];
    ubs = new double[nvarstrips * nvarstrips];
    types = new char[nvarstrips * nvarstrips];
    names = new char*[nvarstrips * nvarstrips];
    for (unsigned int i = 0; i < nvarstrips * nvarstrips; i++) names[i] = new char[20];

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

    HPLUS_env.logger.print_info("Adding c2 to the model.");

    int* ind = new int[nact + 1];
    double* val = new double[nact + 1];
    int nnz = 0;
    const char sense_e = 'E', sense_l = 'L';
    const double rhs_0 = 0, rhs_1 = 1;
    const int begin = 0;

    for (auto p : !istate) {

        nnz = 0;
        ind[nnz] = var_start + p;
        val[nnz++] = 1;

        for (unsigned int i = 0; i < nact; i++) {
            const auto& eff = actions[i].get_eff() - istate;
            if (eff[p]) {

                int var_cnt = 0;
                for (auto j : eff) {
                    if(j == p) break;
                    var_cnt++;
                }

                ind[nnz] = fa_start + fa_individual_start[i] + var_cnt;
                val[nnz++] = -1;
            }
        }

        my::assert(!CPXaddrows(env, lp, 0, 1, nnz, &rhs_0, &sense_e, &begin, ind, val, nullptr, nullptr), "CPXaddwors (c2) failed.");

    }

    HPLUS_env.logger.print_info("Adding c3 to the model.");

    for (auto p : !istate) {                                                // FIXME: O(nvarstrips ^ 3 * nact)
        for (auto q : !istate) {
            nnz = 0;
            ind[nnz] = var_start + q;
            val[nnz++] = -1;
            for (unsigned int act_i = 0; act_i < nact; act_i++) {
                const auto& pre = actions[act_i].get_pre() - istate;
                if (pre[q]) {
                    const auto& eff = actions[act_i].get_eff() - istate;
                    if (eff[p]) {
                        int var_cnt = 0;
                        for (auto j : eff) {
                            if(j == p) break;
                            var_cnt++;
                        }

                        ind[nnz] = fa_start + fa_individual_start[act_i] + var_cnt;
                        val[nnz++] = 1;
                    }
                }
            }
            my::assert(!CPXaddrows(env, lp, 0, 1, nnz, &rhs_0, &sense_l, &begin, ind, val, nullptr, nullptr), "CPXaddwors (c3) failed.");
        }
    }

    HPLUS_env.logger.print_info("Adding c5 to the model.");

    delete[] val; val = nullptr;
    delete[] ind; ind = nullptr;

    int ind_c5_c6_c7[2], ind_c8[3];
    double val_c5_c6_c7[2], val_c8[3];

    for (unsigned int act_i = 0; act_i < nact; act_i++) {
        int var_cnt = 0;
        for (auto p : actions[act_i].get_eff() - istate) {
            ind_c5_c6_c7[0] = act_i;
            val_c5_c6_c7[0] = -1;
            ind_c5_c6_c7[1] = fa_start + fa_individual_start[act_i] + var_cnt;
            val_c5_c6_c7[1] = 1;
            var_cnt++;
            my::assert(!CPXaddrows(env, lp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr), "CPXaddrows (c5) faliled.");
        }
    }

    HPLUS_env.logger.print_info("Adding c6 to the model.");

    for (unsigned int act_i = 0; act_i < nact; act_i++) {
        const auto& pre = actions[act_i].get_pre() - istate;
        const auto& eff = actions[act_i].get_eff() - istate;
        for (auto i : pre) {
            int var_cnt = 0;
            for (auto j : eff) {
                ind_c5_c6_c7[0] = veg_edges_start + i * nvarstrips + j;
                val_c5_c6_c7[0] = -1;
                ind_c5_c6_c7[1] = fa_start + fa_individual_start[act_i] + var_cnt;
                val_c5_c6_c7[1] = 1;
                var_cnt++;
                my::assert(!CPXaddrows(env, lp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr), "CPXaddrows (c6) faliled.");
            }
        }
    }

    HPLUS_env.logger.print_info("Adding c7 to the model.");

    for (auto i : !istate) for (auto j : cumulative_graph[i] & !istate) {
        ind_c5_c6_c7[0] = veg_edges_start + i * nvarstrips + j;
        val_c5_c6_c7[0] = 1;
        ind_c5_c6_c7[1] = veg_edges_start + j * nvarstrips + i;
        val_c5_c6_c7[1] = 1;
        my::assert(!CPXaddrows(env, lp, 0, 1, 2, &rhs_1, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr), "CPXaddrows (c7) faliled.");
    }

    HPLUS_env.logger.print_info("Adding c8 to the model.");

    for (unsigned int h = 0; h < triples_list.size(); h++) {
        const int i = std::get<0>(triples_list[h]), j = std::get<1>(triples_list[h]), k = std::get<2>(triples_list[h]);
        if (istate[i] || istate[j] || istate[k]) continue;
        ind_c8[0] = veg_edges_start + i * nvarstrips + j;
        val_c8[0] = 1;
        ind_c8[1] = veg_edges_start + j * nvarstrips + k;
        val_c8[1] = 1;
        ind_c8[2] = veg_edges_start + i * nvarstrips + k;
        val_c8[2] = -1;
        my::assert(!CPXaddrows(env, lp, 0, 1, 3, &rhs_1, &sense_l, &begin, ind_c8, val_c8, nullptr, nullptr), "CPXaddrows (c8) faliled.");
    }

    my::assert(!CPXwriteprob(env, lp, (HPLUS_CPLEX_OUT_DIR"/lp/"+HPLUS_env.run_name+".lp").c_str(), "LP"), "CPXwriteprob failed.");

    HPLUS_env.logger.print_info("Created CPLEX lp for rankooh.");

}

void HPLUS_store_rankooh_sol(const CPXENVptr& env, const CPXLPptr& lp, HPLUS_instance& inst) {

    // get cplex result (interested only in the sequence of actions [0/nact-1] used and its ordering [nact/2nact-1])
    unsigned int nact = inst.get_nact();
    auto* plan = new double[nact];
    my::assert(!CPXgetx(env, lp, plan, 0, nact-1), "CPXgetx failed.");

    // convert to std collections for easier parsing
    std::vector<unsigned int> cpx_result;
    for (unsigned int i = 0; i < nact; i++) if (plan[i] > .5) cpx_result.push_back(i);
    delete[] plan; plan = nullptr;

    const auto& actions = inst.get_actions();

    // TODO: Order solution before passing it to the instance

    // TODO: Uncomment this
    // store solution
    // inst.update_best_solution(solution,
    //     std::accumulate(solution.begin(), solution.end(), 0,
    //         [&actions](const unsigned int acc, const unsigned int index) {
    //             return acc + actions[index].get_cost();
    //         }
    //     )
    // );

    // TODO: Remove this
    HPLUS_env.logger.print_info("Updated best solution - Cost:   %d.", std::accumulate(cpx_result.begin(), cpx_result.end(), 0,
            [&actions](const unsigned int acc, const unsigned int index) {
                return acc + actions[index].get_cost();
            }
        ));

}