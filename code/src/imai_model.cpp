#include "../include/imai_model.hpp"

void HPLUS_cpx_build_imai(CPXENVptr& env, CPXLPptr& lp) {

    const auto& actions = HPLUS_inst.get_all_actions();

    const auto& remaining_variables = HPLUS_inst.get_variables(true).sparse();
    const auto& remaining_actions = HPLUS_inst.get_actions(true).sparse();

    const auto& fixed_variables = HPLUS_inst.get_fixed_variables();
    const auto& fixed_actions = HPLUS_inst.get_fixed_actions();
    const auto& eliminated_fa = HPLUS_inst.get_eliminated_fa();
    const auto& fixed_fa = HPLUS_inst.get_fixed_fa();
    const auto& timestamps_var = HPLUS_inst.get_timestamps_var();
    const auto& timestamps_act = HPLUS_inst.get_timestamps_act();

    const auto n_var = HPLUS_inst.get_n_var(true);
    const auto n_act = HPLUS_inst.get_n_act(true);

    // ====================================================== //
    // =================== CPLEX VARIABLES ================== //
    // ====================================================== //
    // (section 3 of Imai's paper)

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

    // --- action timestamps -- //
    size_t tact_start = curr_col;
    count = 0;
    for (auto act_i : remaining_actions) {
        objs[count] = 0;
        lbs[count] = timestamps_act[act_i] >= 0 ? timestamps_act[act_i] : 0;
        ubs[count] = timestamps_act[act_i] >= 0 ? timestamps_act[act_i] : n_act-1;       //[ ]: Tighter bound
        types[count++] = 'I';
    }
    #if HPLUS_INTCHECK
    my::assert(count == n_act, "Wrong number of action timestamps while building the model.");
    #endif
    curr_col += n_act;

    my::assert(!CPXnewcols(env, lp, n_act, objs, lbs, ubs, types, nullptr), "CPXnewcols (action timestamps) failed.");

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

    // -- variable timestamps - //
    size_t tvar_start = curr_col;
    count = 0;
    for (auto i : remaining_variables) {
        objs[count] = 0;
        lbs[count] = timestamps_var[i] >= 0 ? timestamps_var[i] : 0;
        ubs[count] = timestamps_var[i] >= 0 ? timestamps_var[i] : n_act;                     //[ ]: Tighter bound
        types[count++] = 'I';
    }
    #if HPLUS_INTCHECK
    my::assert(count == n_var, "Wrong number of variable timestamps while building the model.");
    #endif
    curr_col += n_var;

    my::assert(!CPXnewcols(env, lp, n_var, objs, lbs, ubs, types, nullptr), "CPXnewcols (variable timestamps) failed.");

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

    delete[] types; types = nullptr;
    delete[] ubs; ubs = nullptr;
    delete[] lbs; lbs = nullptr;
    delete[] objs; objs = nullptr;

    // ====================================================== //
    // ================== CPLEX CONSTRAINTS ================= //
    // ====================================================== //
    // (section 3 of Imai's paper)

    // accessing cplex variables
    auto get_act_idx = [act_start](size_t idx) { return act_start + HPLUS_inst.act_idx_post_simplification(idx); };
    auto get_tact_idx = [tact_start](size_t idx) { return tact_start + HPLUS_inst.act_idx_post_simplification(idx); };
    auto get_var_idx = [var_start](size_t idx) { return var_start + HPLUS_inst.var_idx_post_simplification(idx); };
    auto get_tvar_idx = [tvar_start](size_t idx) { return tvar_start + HPLUS_inst.var_idx_post_simplification(idx); };
    auto get_fa_idx = [fa_start, fa_individual_start](size_t act_idx, size_t var_idx) { return fa_start + fa_individual_start[HPLUS_inst.act_idx_post_simplification(act_idx)] + HPLUS_inst.var_idx_post_simplification(var_idx); };
    
    int* ind_c1 = new int[n_act + 1];
    double* val_c1 = new double[n_act + 1];
    int ind_c2_4[2], ind_c5[3];
    double val_c2_4[2], val_c5[3];
    const int nnz_c2_4 = 2, nnz_c5 = 3;
    const char sensel = 'L', sensee = 'E';
    const double rhs_c1_2_4 = 0, rhs_c5 = n_act;         //[ ]: Tighter bound
    const int begin = 0;

    std::vector<int*> ind_c3(n_var);
    std::vector<double*> val_c3(n_var);
    std::vector<int> nnz_c3(n_var);
    std::vector<double> rhs_c3(n_var);

    for (auto i : remaining_variables) {
        ind_c3[HPLUS_inst.var_idx_post_simplification(i)] = new int[n_act + 1];
        val_c3[HPLUS_inst.var_idx_post_simplification(i)] = new double[n_act + 1];
        nnz_c3[HPLUS_inst.var_idx_post_simplification(i)] = 0;
        rhs_c3[HPLUS_inst.var_idx_post_simplification(i)] = 0;
        ind_c3[HPLUS_inst.var_idx_post_simplification(i)][nnz_c3[HPLUS_inst.var_idx_post_simplification(i)]] = get_var_idx(i);
        val_c3[HPLUS_inst.var_idx_post_simplification(i)][nnz_c3[HPLUS_inst.var_idx_post_simplification(i)]] = 1;
        nnz_c3[HPLUS_inst.var_idx_post_simplification(i)]++;
    }
    
    // mylog.print_info("Adding constraints to CPLEX.");

    const auto& remaining_var_set = HPLUS_inst.get_variables(true);

    for (auto i : remaining_actions) {
        const auto& pre = actions[i].get_pre() & remaining_var_set;
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
        const auto& eff = actions[i].get_eff() & remaining_var_set;
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
            val_c5[2] = n_act + 1;                                                                   //[ ]: Tighter bound
            my::assert(!CPXaddrows(env, lp, 0, 1, nnz_c5, &rhs_c5, &sensel, &begin, ind_c5, val_c5, nullptr, nullptr), "CPXaddrows (c5) failed.");
            // constraint 3: I(v_j) + sum(z_avj) = y_vj
            ind_c3[HPLUS_inst.var_idx_post_simplification(j)][nnz_c3[HPLUS_inst.var_idx_post_simplification(j)]] = get_fa_idx(i, j);
            val_c3[HPLUS_inst.var_idx_post_simplification(j)][nnz_c3[HPLUS_inst.var_idx_post_simplification(j)]] = -1;
            nnz_c3[HPLUS_inst.var_idx_post_simplification(j)]++;
        }
    }

    for (size_t i = 0; i < n_var; i++) my::assert(!CPXaddrows(env, lp, 0, 1, nnz_c3[i], &rhs_c3[i], &sensee, &begin, ind_c3[i], val_c3[i], nullptr, nullptr), "CPXaddrows (c3) failed.");

    for (size_t i = 0; i < n_var; i++) {
        delete[] ind_c3[i]; ind_c3[i] = nullptr;
        delete[] val_c3[i]; val_c3[i] = nullptr;
    }
    delete[] val_c1; val_c1 = nullptr;
    delete[] ind_c1; ind_c1 = nullptr;

    // my::assert(!CPXwriteprob(env, lp, (HPLUS_CPLEX_OUT_DIR"/lp/"+HPLUS_env.run_name+".lp").c_str(), "LP"), "CPXwriteprob failed.");

    // mylog.print_info("Created CPLEX lp for imai.");

}

/**
 * Check if this solution is the best one and if so, stores it
*/
void HPLUS_store_imai_sol(const CPXENVptr& env, const CPXLPptr& lp) {

    // get cplex result (interested only in the sequence of actions [0/nact-1] used and its ordering [nact/2nact-1])
    const size_t nact = HPLUS_inst.get_n_act(true);
    double* plan = new double[2 * nact];
    my::assert(!CPXgetx(env, lp, plan, 0, 2 * nact - 1), "CPXgetx failed.");

    // convert to std collections for easier parsing
    std::vector<std::pair<double, size_t>> cpx_result;
    for (size_t i = 0; i < nact; i++) if (plan[i] > .5) cpx_result.emplace_back(plan[nact+i], i);
    delete[] plan; plan = nullptr;

    // sort cpx_result based on actions timestamps
    std::sort(cpx_result.begin(), cpx_result.end(),
        [](const std::pair<double, size_t> &x, const std::pair<double, size_t> &y) {
            return x.first < y.first;
        }
    );

    // get solution from sorted cpx_result
    std::vector<size_t> solution;
    std::transform(cpx_result.begin(), cpx_result.end(), std::back_inserter(solution),
        [](const std::pair<double, size_t> &p) {
            return HPLUS_inst.cpx_idx_to_act_idx(p.second);
        }
    );

    // store solution
    const auto& actions = HPLUS_inst.get_all_actions();
    HPLUS_inst.update_best_sol(solution,
        std::accumulate(solution.begin(), solution.end(), 0,
            [&actions](const size_t acc, const size_t index) {
                return acc + actions[index].get_cost();
            }
        )
    );

}