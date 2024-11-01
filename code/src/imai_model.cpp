#include "../include/imai_model.hpp"

/**
 * Build the basic Imai model
*/
void HPLUS_cpx_build_imai(CPXENVptr& env, CPXLPptr& lp, HPLUS_instance& inst) {

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
    // =================== CPLEX VARIABLES ================== //
    // ====================================================== //
    // (section 3 of Imai's paper)

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

    // --- action timestamps -- //
    unsigned int tact_start = curr_col;
    count = 0;
    for (auto act_i : remaining_act_sparse) {
        objs[count] = 0;
        lbs[count] = inst.model_opt_timestamps_act[act_i] >= 0 ? inst.model_opt_timestamps_act[act_i] : 0;
        ubs[count] = inst.model_opt_timestamps_act[act_i] >= 0 ? inst.model_opt_timestamps_act[act_i] : nact-1;       //[ ]: Tighter bound
        types[count++] = 'I';
    }
    curr_col += nact_opt;

    my::assert(!CPXnewcols(env, lp, nact_opt, objs, lbs, ubs, types, nullptr), "CPXnewcols (action timestamps) failed.");

    resize_cpx_arrays(nvar_opt);
    
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

    // -- variable timestamps - //
    unsigned int tvar_start = curr_col;
    count = 0;
    for (auto i : remaining_var_sparse) {
        objs[count] = 0;
        lbs[count] = inst.model_opt_timestamps_var[i] >= 0 ? inst.model_opt_timestamps_var[i] : 0;
        ubs[count] = inst.model_opt_timestamps_var[i] >= 0 ? inst.model_opt_timestamps_var[i] : nact;                     //[ ]: Tighter bound
        types[count++] = 'I';
    }
    curr_col += nvar_opt;

    my::assert(!CPXnewcols(env, lp, nvar_opt, objs, lbs, ubs, types, nullptr), "CPXnewcols (variable timestamps) failed.");

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

    delete[] types; types = nullptr;
    delete[] ubs; ubs = nullptr;
    delete[] lbs; lbs = nullptr;
    delete[] objs; objs = nullptr;

    // ====================================================== //
    // ================== CPLEX CONSTRAINTS ================= //
    // ====================================================== //
    // (section 3 of Imai's paper)

    // accessing cplex variables
    auto get_act_idx = [act_start, &actidx_to_cpxidx](int idx) { return act_start + actidx_to_cpxidx[idx]; };
    auto get_tact_idx = [tact_start, &actidx_to_cpxidx](int idx) { return tact_start + actidx_to_cpxidx[idx]; };
    auto get_var_idx = [var_start, &varidx_to_cpxidx](int idx) { return var_start + varidx_to_cpxidx[idx]; };
    auto get_tvar_idx = [tvar_start, &varidx_to_cpxidx](int idx) { return tvar_start + varidx_to_cpxidx[idx]; };
    auto get_fa_idx = [fa_start, fa_individual_start, &varidx_to_cpxidx, &actidx_to_cpxidx](int act_idx, int var_idx) { return fa_start + fa_individual_start[actidx_to_cpxidx[act_idx]] + varidx_to_cpxidx[var_idx]; };
    
    int* ind_c1 = new int[nact_opt + 1];
    double* val_c1 = new double[nact_opt + 1];
    int ind_c2_4[2], ind_c5[3];
    double val_c2_4[2], val_c5[3];
    const int nnz_c2_4 = 2, nnz_c5 = 3;
    const char sensel = 'L', sensee = 'E';
    const double rhs_c1_2_4 = 0, rhs_c5 = nact;         //[ ]: Tighter bound
    const int begin = 0;

    std::vector<int*> ind_c3(nvar_opt);
    std::vector<double*> val_c3(nvar_opt);
    std::vector<int> nnz_c3(nvar_opt);
    std::vector<double> rhs_c3(nvar_opt);

    for (auto i : remaining_var_sparse) {
        ind_c3[varidx_to_cpxidx[i]] = new int[nact_opt + 1];
        val_c3[varidx_to_cpxidx[i]] = new double[nact_opt + 1];
        nnz_c3[varidx_to_cpxidx[i]] = 0;
        rhs_c3[varidx_to_cpxidx[i]] = 0;
        ind_c3[varidx_to_cpxidx[i]][nnz_c3[varidx_to_cpxidx[i]]] = get_var_idx(i);
        val_c3[varidx_to_cpxidx[i]][nnz_c3[varidx_to_cpxidx[i]]] = 1;
        nnz_c3[varidx_to_cpxidx[i]]++;
    }
    
    lprint_info("Adding constraints to CPLEX.");

    for (auto i : remaining_act_sparse) {
        const auto& pre = actions[i].get_pre() & !eliminated_var;
        for (auto j : pre) {
            // constraint 1: x_a + sum_{inv(a, p)}(z_a'vj) <= y_vj, vj in pre(a)
            ind_c1[0] = get_act_idx(i);
            val_c1[0] = 1;
            ind_c1[1] = get_var_idx(j);
            val_c1[1] = -1;
            int nnz0 = 2;
            // (section 4.6 of Imai's paper)        //[ ]: This might not be worth doing
            // if (inverse_actions.find(i) != inverse_actions.end()) {
            //     for (auto inv_act : inverse_actions[i]) if (actions[inv_act].get_eff()[j]) {
            //         ind_c1[nnz0] = get_fa_idx(inv_act, j);
            //         val_c1[nnz0++] = 1;
            //     }
            // }
            my::assert(!CPXaddrows(env, lp, 0, 1, nnz0, &rhs_c1_2_4, &sensel, &begin, ind_c1, val_c1, nullptr, nullptr), "CPXaddrows (c1) failed.");
            // constraint 4: t_vj <= t_a, vj in pre(a)
            ind_c2_4[0] = get_tvar_idx(j);
            val_c2_4[0] = 1;
            ind_c2_4[1] = get_tact_idx(i);
            val_c2_4[1] = -1;
            my::assert(!CPXaddrows(env, lp, 0, 1, nnz_c2_4, &rhs_c1_2_4, &sensel, &begin, ind_c2_4, val_c2_4, nullptr, nullptr), "CPXaddrows (c4) failed.");
        }
        const auto& eff = actions[i].get_eff() & !eliminated_var;
        for (auto j : eff) {
            // constraint 2: z_avj <= x_a, vj in eff(a)
            ind_c2_4[0] = get_fa_idx(i, j);
            val_c2_4[0] = 1;
            ind_c2_4[1] = get_act_idx(i);
            val_c2_4[1] = -1;
            my::assert(!CPXaddrows(env, lp, 0, 1, nnz_c2_4, &rhs_c1_2_4, &sensel, &begin, ind_c2_4, val_c2_4, nullptr, nullptr), "CPXaddrows (c2) failed.");
            // constraint 5: t_a + 1 <= t_vj + (|A|+1)(1-z_avj), vj in eff(a)
            ind_c5[0] = get_tact_idx(i);
            val_c5[0] = 1;
            ind_c5[1] = get_tvar_idx(j);
            val_c5[1] = -1;
            ind_c5[2] = get_fa_idx(i, j);
            val_c5[2] = nact + 1;                                                                   //[ ]: Tighter bound
            my::assert(!CPXaddrows(env, lp, 0, 1, nnz_c5, &rhs_c5, &sensel, &begin, ind_c5, val_c5, nullptr, nullptr), "CPXaddrows (c5) failed.");
            // constraint 3: I(v_j) + sum(z_avj) = y_vj
            ind_c3[varidx_to_cpxidx[j]][nnz_c3[varidx_to_cpxidx[j]]] = get_fa_idx(i, j);
            val_c3[varidx_to_cpxidx[j]][nnz_c3[varidx_to_cpxidx[j]]] = -1;
            nnz_c3[varidx_to_cpxidx[j]]++;
        }
    }

    for (unsigned int i = 0; i < nvar_opt; i++) my::assert(!CPXaddrows(env, lp, 0, 1, nnz_c3[i], &rhs_c3[i], &sensee, &begin, ind_c3[i], val_c3[i], nullptr, nullptr), "CPXaddrows (c3) failed.");

    for (unsigned int i = 0; i < nvar_opt; i++) {
        delete[] ind_c3[i]; ind_c3[i] = nullptr;
        delete[] val_c3[i]; val_c3[i] = nullptr;
    }
    delete[] val_c1; val_c1 = nullptr;
    delete[] ind_c1; ind_c1 = nullptr;

    // my::assert(!CPXwriteprob(env, lp, (HPLUS_CPLEX_OUT_DIR"/lp/"+HPLUS_env.run_name+".lp").c_str(), "LP"), "CPXwriteprob failed.");

    lprint_info("Created CPLEX lp for imai.");

}

/**
 * Check if this solution is the best one and if so, stores it
*/
void HPLUS_store_imai_sol(const CPXENVptr& env, const CPXLPptr& lp, HPLUS_instance& inst) {

    // get cplex result (interested only in the sequence of actions [0/nact-1] used and its ordering [nact/2nact-1])
    const unsigned int nact = inst.get_nact_opt();
    double* plan = new double[2 * nact];
    my::assert(!CPXgetx(env, lp, plan, 0, 2 * nact - 1), "CPXgetx failed.");

    // convert to std collections for easier parsing
    std::vector<std::pair<double, unsigned int>> cpx_result;
    for (unsigned int i = 0; i < nact; i++) if (plan[i] > .5) cpx_result.emplace_back(plan[nact+i], i);
    delete[] plan; plan = nullptr;

    // sort cpx_result based on actions timestamps
    std::sort(cpx_result.begin(), cpx_result.end(),
        [](const std::pair<double, unsigned int> &x, const std::pair<double, unsigned int> &y) {
            return x.first < y.first;
        }
    );

    // get solution from sorted cpx_result
    std::vector<unsigned int> solution;
    std::transform(cpx_result.begin(), cpx_result.end(), std::back_inserter(solution),
        [&inst](const std::pair<double, unsigned int> &p) {
            return inst.cpxidx_to_actidx(p.second);
        }
    );

    // store solution
    const auto& actions = inst.get_actions();
    inst.update_best_solution(solution,
        std::accumulate(solution.begin(), solution.end(), 0,
            [&actions](const unsigned int acc, const unsigned int index) {
                return acc + actions[index].get_cost();
            }
        )
    );

}