#include "../include/imai_model.hpp"

/**
 * Build the basic Imai model
*/
void HPLUS_cpx_build_imai(CPXENVptr& env, CPXLPptr& lp, const HPLUS_instance& inst) {

    const auto nvar = inst.get_nvar();
    const auto nact = inst.get_nact();
    const auto nvarstrips = inst.get_nvar_strips();
    const auto& actions = inst.get_actions();
    const auto& variables = inst.get_variables();
    const auto& istate = inst.get_istate();

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
    // (section 3 of Imai's paper)

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

    // -------- actions ------- //
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

    // --- action timestamps -- //
    unsigned int tact_start = curr_col;
    for (unsigned int act_i = 0; act_i < nact; act_i++) {
        objs[act_i] = 0;
        lbs[act_i] = fixed_act_timestamps[act_i] >= 0 ? fixed_act_timestamps[act_i] : 0;
        ubs[act_i] = fixed_act_timestamps[act_i] >= 0 ? fixed_act_timestamps[act_i] : nact-1;
        types[act_i] = 'I';
        snprintf(names[act_i], 20, "tact(%d)", act_i);
    }
    curr_col += nact;

    my::assert(!CPXnewcols(env, lp, nact, objs, lbs, ubs, types, names), "CPXnewcols (action timestamps) failed.");

    rsz_cpx_arrays(nact, nvarstrips);

    // ------- variables ------ //
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

    // -- variable timestamps - //
    unsigned int tvar_start = curr_col;
    for (unsigned int i = 0, count = 0; i < nvar; i++) for (unsigned int j = 0; j < variables[i].get_range(); j++, count++) {
        objs[count] = 0;
        lbs[count] = fixed_var_timestamps[count] >= 0 ? fixed_var_timestamps[count] : 0;
        ubs[count] = fixed_var_timestamps[count] >= 0 ? fixed_var_timestamps[count] : nact;
        types[count] = 'I';
        snprintf(names[count], 20, "tvar(%d_%d)", i, j);
    }
    curr_col += nvarstrips;

    my::assert(!CPXnewcols(env, lp, nvarstrips, objs, lbs, ubs, types, names), "CPXnewcols (variable timestamps) failed.");

    rsz_cpx_arrays(nvarstrips, nact*nvarstrips);

    // --- first archievers --- //
    unsigned int fa_start = curr_col;
    std::vector<unsigned int> fa_individual_start(nact);
    for (unsigned int act_i = 0, nfa = 0; act_i < nact; act_i++) {
        fa_individual_start[act_i] = nfa;
        const auto& eff = actions[act_i].get_eff();
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

    for (unsigned int i = 0; i < nact * nvarstrips; i++) delete[] names[i];
    delete[] names; names = nullptr;
    delete[] types; types = nullptr;
    delete[] ubs; ubs = nullptr;
    delete[] lbs; lbs = nullptr;
    delete[] objs; objs = nullptr;

    // ~~~~~~~ Adding CPLEX constraints ~~~~~~ //
    // (section 3 of Imai's paper)

    // accessing cplex variables
    auto get_act_idx = [act_start](int idx) { return act_start + idx; };
    auto get_tact_idx = [tact_start](int idx) { return tact_start + idx; };
    auto get_var_idx = [var_start](int idx) { return var_start + idx; };
    auto get_tvar_idx = [tvar_start](int idx) { return tvar_start + idx; };
    auto get_fa_idx = [fa_start, fa_individual_start](int act_idx, int var_idx) { return fa_start + fa_individual_start[act_idx] + var_idx; };

    int* ind_c1 = new int[nact+1];
    double* val_c1 = new double[nact+1];
    int ind_c2_4[2], ind_c5[3];
    double val_c2_4[2], val_c5[3];
    const int nnz_c2_4 = 2, nnz_c5 = 3;
    const char sensel = 'L', sensee = 'E';
    const double rhs_c1_2_4 = 0, rhs_c5 = nact;
    const int begin = 0;

    std::vector<int*> ind_c3(nvarstrips);
    std::vector<double*> val_c3(nvarstrips);
    std::vector<int> nnz_c3(nvarstrips);
    std::vector<double> rhs_c3(nvarstrips);

    for (unsigned int i = 0; i < nvarstrips; i++) {
        ind_c3[i] = new int[nact + 1];
        val_c3[i] = new double[nact + 1];
        nnz_c3[i] = 0;
        rhs_c3[i] = istate[i];
        ind_c3[i][nnz_c3[i]] = get_var_idx(i);
        val_c3[i][nnz_c3[i]] = 1;
        nnz_c3[i]++;
    }

    for (unsigned int i = 0; i < nact; i++) {
        const auto& pre = actions[i].get_pre();
        const auto& eff = actions[i].get_eff();
        for (unsigned int j = 0; j < nvarstrips; j++) {
            if (pre[j]) {
                // constraint 1: x_a + sum_{inv(a, p)}(z_a'vj) <= y_vj, vj in pre(a)
                ind_c1[0] = get_act_idx(i);
                val_c1[0] = 1;
                ind_c1[1] = get_var_idx(j);
                val_c1[1] = -1;
                int nnz0 = 2;
                // (section 4.6 of Imai's paper)
                for (auto inv_act : inverse_actions[i]) {
                    const auto& inv_eff = actions[inv_act].get_eff();
                    if (inv_eff[j]) {
                        int var_cnt = 0;
                        for (auto p : inv_eff) {
                            if (p == j) break;
                            var_cnt++;
                        }
                        ind_c1[nnz0] = get_fa_idx(inv_act, j);
                        val_c1[nnz0++] = 1;
                    }
                }
                my::assert(!CPXaddrows(env, lp, 0, 1, nnz0, &rhs_c1_2_4, &sensel, &begin, ind_c1, val_c1, nullptr, nullptr), "CPXaddrows (c1) failed.");
                // constraint 4: t_vj <= t_a, vj in pre(a)
                ind_c2_4[0] = get_tvar_idx(j);
                val_c2_4[0] = 1;
                ind_c2_4[1] = get_tact_idx(i);
                val_c2_4[1] = -1;
                my::assert(!CPXaddrows(env, lp, 0, 1, nnz_c2_4, &rhs_c1_2_4, &sensel, &begin, ind_c2_4, val_c2_4, nullptr, nullptr), "CPXaddrows (c4) failed.");
            }
            if (eff[j]) {
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
                val_c5[2] = nact + 1;
                my::assert(!CPXaddrows(env, lp, 0, 1, nnz_c5, &rhs_c5, &sensel, &begin, ind_c5, val_c5, nullptr, nullptr), "CPXaddrows (c5) failed.");
                // constraint 3: I(v_j) + sum(z_avj) = y_vj
                ind_c3[j][nnz_c3[j]] = get_fa_idx(i, j);
                val_c3[j][nnz_c3[j]] = -1;
                nnz_c3[j]++;
            }
        }
    }

    for (unsigned int i = 0; i < nvarstrips; i++) my::assert(!CPXaddrows(env, lp, 0, 1, nnz_c3[i], &rhs_c3[i], &sensee, &begin, ind_c3[i], val_c3[i], nullptr, nullptr), "CPXaddrows (c3) failed.");

    for (unsigned int i = 0; i < nvarstrips; i++) {
        delete[] ind_c3[i]; ind_c3[i] = nullptr;
        delete[] val_c3[i]; val_c3[i] = nullptr;
    }
    delete[] val_c1; val_c1 = nullptr;
    delete[] ind_c1; ind_c1 = nullptr;

    my::assert(!CPXwriteprob(env, lp, (HPLUS_CPLEX_OUT_DIR"/lp/"+HPLUS_env.run_name+".lp").c_str(), "LP"), "CPXwriteprob failed.");

    lprint_info("Created CPLEX lp for imai.");

}

/**
 * Check if this solution is the best one and if so, stores it
*/
void HPLUS_store_imai_sol(const CPXENVptr& env, const CPXLPptr& lp, HPLUS_instance& inst) {

    // get cplex result (interested only in the sequence of actions [0/nact-1] used and its ordering [nact/2nact-1])
    auto nact = inst.get_nact();
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
        [](const std::pair<double, unsigned int> &p) {
            return p.second;
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