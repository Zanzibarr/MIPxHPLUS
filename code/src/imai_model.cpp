#include "../include/imai_model.hpp"

/**
 * Explanation: Section 4.1 of Imai's paper
 */
void HPLUS_imai_extract_landmarks(const HPLUS_instance& inst, std::vector<my::BitField> landmarks_set, my::BitField& fact_landmarks, my::BitField& act_landmarks, my::BitField& fixed_variables, my::BitField& fixed_actions) {

    HPLUS_env.logger.print_info("Extracting landmarks.");

    const auto nvarstrips = inst.get_nvar_strips();
    const auto nact = inst.get_nact();
    const auto& istate = inst.get_istate();
    const auto& actions = inst.get_actions();

    for (unsigned int p = 0; p < nvarstrips; p++) {
        if (istate[p]) landmarks_set[p].set(p);
        // using the last bit as "full set" flag
        else landmarks_set[p].set(nvarstrips);
    }

    auto s_set = istate;

    std::deque<int> actions_queue;
    for (unsigned int i = 0; i < nact; i++)
        if(s_set.contains(actions[i].get_pre())) actions_queue.push_back(i);

    while(!actions_queue.empty()) {

        const HPLUS_action a = actions[actions_queue.front()];
        actions_queue.pop_front();

        const auto& pre_a = a.get_pre();
        const auto& add_a = a.get_eff();

        for (auto p : add_a) {

            s_set.set(p);

            auto x = my::BitField(nvarstrips + 1);
            for (auto p : add_a) x.set(p);

            for (auto pp : pre_a) {
                // if variable p' has the "full" flag then the unification
                // generates a "full" bitfield -> no need to unificate, just set the flag
                if (landmarks_set[pp][nvarstrips]) {
                    x.set(nvarstrips);
                    // if x is now full we can exit, since all further unions won't change x
                    break;
                } else x |= landmarks_set[pp];
            }

            // we then check if L[p] != X, and if they are the same we skip,
            // if X = P, then (X intersection L[P]) = L[P], hence we can already skip
            if (!x[nvarstrips]) {

                // if the set for variable p is the full set of variables,
                // the intersection generates back x -> we can skip the intersection
                if (!landmarks_set[p][nvarstrips]) x &= landmarks_set[p];

                // we already know that x is not the full set now, so if
                // the set for variable p is the full set, we know that x is not
                // equal to the set for variable p -> we can skip the check
                if (landmarks_set[p][nvarstrips] || x != landmarks_set[p]) {

                    landmarks_set[p] = x;
                    for (unsigned int aa = 0; aa < nact; aa++) if (actions[aa].get_pre()[p])
                        if (s_set.contains(actions[aa].get_pre()) && std::find(actions_queue.begin(), actions_queue.end(), aa) == actions_queue.end())
                            actions_queue.push_back(aa);

                }

            }

        }

    }

    fact_landmarks |= istate;

    for (auto p : inst.get_gstate()) for (auto j : !fact_landmarks) if (landmarks_set[p][j] || landmarks_set[p][nvarstrips]) {
        fact_landmarks.set(j);
        unsigned int count = 0, cand_act;
        for (unsigned int act_i = 0; act_i < nact && count <= 1; act_i++) if (actions[act_i].get_eff()[j]) {
            cand_act = act_i;
            count++;
        }
        if (count == 1) act_landmarks.set(cand_act);
    }

    fixed_variables |= fact_landmarks;
    fixed_actions |= act_landmarks;

}

/**
 * Explanation: Section 4.2 of Imai's paper
 */
void HPLUS_imai_extract_fadd(const HPLUS_instance& inst, const std::vector<my::BitField>& landmarks_set, std::vector<my::BitField>& fadd, std::vector<my::BitField>& eliminated_fas) {

    const auto nact = inst.get_nact();
    const auto nvarstrips = inst.get_nvar_strips();
    const auto& actions = inst.get_actions();

    for (unsigned int act_i = 0; act_i < nact; act_i++) {
        // f_lm_a is the set of fact landmarks of action act_i
        my::BitField f_lm_a(nvarstrips);
        for (auto p : actions[act_i].get_pre()) for (unsigned int i = 0; i < nvarstrips; i++) if (landmarks_set[p][i] || landmarks_set[p][nvarstrips]) f_lm_a.set(i);

        // fadd[a] := { p in add(a) s.t. p is not a fact landmark for a }
        fadd[act_i] |= (actions[act_i].get_eff() & !f_lm_a);
        eliminated_fas[act_i] |= (actions[act_i].get_eff() & f_lm_a);

    }

}

/**
 * Explanation: Section 4.2 of Imai's paper
 */
void HPLUS_imai_relevance_analysis(const HPLUS_instance& inst, const std::vector<my::BitField>& fadd, const my::BitField& fact_landmarks, my::BitField& eliminated_variables, my::BitField& eliminated_actions) {

    HPLUS_env.logger.print_info("Relevance analysis.");

    const auto nact = inst.get_nact();
    const auto nvarstrips = inst.get_nvar_strips();
    const auto& actions = inst.get_actions();
    const auto& gstate = inst.get_gstate();

    my::BitField relevant_variables(nvarstrips);
    my::BitField relevant_actions(nact);

    for (unsigned int act_i = 0; act_i < nact; act_i++) {
        if (fadd[act_i].intersects(gstate)) {
            relevant_variables |= actions[act_i].get_pre();
            relevant_actions.set(act_i);
        }
    }

    auto cand_actions = !relevant_actions;

    bool new_act = true;
    while (new_act) {
        new_act = false;
        for (auto act_i : cand_actions) {
            if (actions[act_i].get_eff().intersects(relevant_variables)) {
                relevant_actions.set(act_i);
                relevant_variables |= actions[act_i].get_pre();
                cand_actions.unset(act_i);
                new_act = true;
                break;
            }
        }
    }
    relevant_variables |= gstate;

    eliminated_variables |= (!relevant_variables & !fact_landmarks);
    eliminated_actions |= !relevant_actions;

}

/**
 * Explanation: Section 4.4 of Imai's paper
 */
void HPLUS_imai_immediate_action_application(const HPLUS_instance& inst, const my::BitField& act_landmarks, const my::BitField& eliminated_variables, my::BitField& fixed_variables, const my::BitField& eliminated_actions, my::BitField& fixed_actions, std::vector<my::BitField>& eliminated_fas, std::vector<my::BitField>& fixed_fas, std::vector<int>& fixed_var_timestamps, std::vector<int>& fixed_act_timestamps) {

    HPLUS_env.logger.print_info("Immediate action application.");

    const auto nact = inst.get_nact();
    const auto& actions = inst.get_actions();

    auto current_state = inst.get_istate();
    auto actions_left = !eliminated_actions;

    int counter = 0;
    bool found_next_action = true;
    while (found_next_action) {

        found_next_action = false;

        for (auto act_i : actions_left) {

            const auto& pre = actions[act_i].get_pre();
            auto add_eff = actions[act_i].get_eff() & !eliminated_variables;

            if (current_state.contains(pre) && (act_landmarks[act_i] || actions[act_i].get_cost() == 0)) {

                actions_left.unset(act_i);
                fixed_actions.set(act_i);
                fixed_act_timestamps[act_i] = counter;
                fixed_variables |= pre;
                for (auto p : add_eff & !current_state) {
                    fixed_variables.set(p);
                    fixed_var_timestamps[p] = counter+1;
                    fixed_fas[act_i].set(p);
                    for (auto act_j : actions_left) eliminated_fas[act_j].set(p);
                }
                current_state |= add_eff;
                counter++;
                found_next_action = true;

            }

        }

    }

}

/**
 * Explanation: Section 4.3 of Imai's paper
 */
void HPLUS_imai_dominated_actions_elimination(const HPLUS_instance& inst, const std::vector<my::BitField>& fadd, const std::vector<my::BitField>& landmarks_set, my::BitField& eliminated_actions) {

    HPLUS_env.logger.print_info("Dominated actions elimination.");

    const auto nvarstrips = inst.get_nvar_strips();
    const auto& istate = inst.get_istate();
    const auto& actions = inst.get_actions();

    auto remaining_actions = !eliminated_actions;

    for (auto act_i : remaining_actions) {                              // FIXME: O(nact^2)

        remaining_actions.unset(act_i);
        auto f_lm_a = istate;
        for (auto p : actions[act_i].get_pre()) for (unsigned int i = 0; i < nvarstrips; i++) if (landmarks_set[p][i] || landmarks_set[p][nvarstrips]) f_lm_a.set(i);

        for (auto act_j : remaining_actions) {

            if (actions[act_i].get_cost() < actions[act_j].get_cost() || !fadd[act_j].contains(fadd[act_i]) || !f_lm_a.contains(actions[act_j].get_pre())) continue;

            eliminated_actions.set(act_i);
            break;

        }

    }

}

/**
 * Explanation: Section 4.5 of Imai's paper
 */
void HPLUS_imai_iterative_variable_elimination(const HPLUS_instance& inst, my::BitField& eliminated_variables, my::BitField& fixed_variables, my::BitField& eliminated_actions, my::BitField& fixed_actions, std::vector<my::BitField>& eliminated_fas, std::vector<my::BitField>& fixed_fas, std::vector<int>& fixed_var_timestamps, std::vector<int>& fixed_act_timestamps) {

    const auto nvarstrips = inst.get_nvar_strips();
    const auto nact = inst.get_nact();
    const auto& istate = inst.get_istate();
    const auto& actions = inst.get_actions();

    // using the bit at position nvarstrips as a flag for "full set of variables"
    std::vector<my::BitField> landmarks_set(nvarstrips, my::BitField(nvarstrips+1));
    my::BitField fact_landmarks(nvarstrips);
    my::BitField act_landmarks(nact);

    // landmarks_set contains the fact landmarks for each variable p
    // fact_landmarks contains the fact landmarks for the goal G
    // act_landmarks contains the action landmarks for the goal G
    HPLUS_imai_extract_landmarks(inst, landmarks_set, fact_landmarks, act_landmarks, fixed_variables, fixed_actions);

    std::vector<my::BitField> fadd(nact, my::BitField(nvarstrips));
    // fadd[a] := { p in add(a) s.t. p is not a fact landmark for a }
    HPLUS_imai_extract_fadd(inst, landmarks_set, fadd, eliminated_fas);

    HPLUS_imai_relevance_analysis(inst, fadd, fact_landmarks, eliminated_variables, eliminated_actions);

    HPLUS_imai_immediate_action_application(inst, act_landmarks, eliminated_variables, fixed_variables, eliminated_actions, fixed_actions, eliminated_fas, fixed_fas, fixed_var_timestamps, fixed_act_timestamps);

    HPLUS_imai_dominated_actions_elimination(inst, fadd, landmarks_set, eliminated_actions);

    #if HPLUS_VERBOSE >= 10
    int count = 0;
    for (auto p : eliminated_variables) count++;
    for (auto p : fixed_variables) count++;
    HPLUS_env.logger.print_info("Eliminated %d/%d variables.", count, nvarstrips);
    count = 0;
    for (auto p : eliminated_actions) count++;
    for (auto p : fixed_actions) count++;
    HPLUS_env.logger.print_info("Eliminated %d/%d actions.", count, nact);
    count = 0;
    int count2 = 0;
    for (unsigned int a = 0; a < nact; a++) {
        for (auto i : eliminated_fas[a]) count++;
        for (auto i : fixed_fas[a]) count++;
        count2 += 2 * nvarstrips;
    }
    HPLUS_env.logger.print_info("Eliminated %d/%d first archievers.", count, count2);
    #endif

    #if HPLUS_INTCHECK
    my::assert(!eliminated_variables.intersects(fixed_variables), "Eliminated variables set intersects the fixed variables set.");
    my::assert(!eliminated_actions.intersects(fixed_actions), "Eliminated actions set intersects the fixed actions set.");
    for (unsigned int i = 0; i < nact; i++) my::assert(!eliminated_fas[i].intersects(fixed_fas[i]), "Eliminated first archievers set intersects the fixed first archievers set.");
    #endif

}

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
    const auto& gstate = inst.get_gstate();

    // ~~~~~~~~~~~~ Enhanced model ~~~~~~~~~~~ //
    // (section 4 of Imai's paper)

    my::BitField eliminated_variables(nvarstrips);
    my::BitField fixed_variables(nvarstrips);
    my::BitField eliminated_actions(nact);
    my::BitField fixed_actions(nact);
    std::vector<my::BitField> eliminated_fas(nact, my::BitField(nvarstrips));
    std::vector<my::BitField> fixed_fas(nact, my::BitField(nvarstrips));
    std::vector<int> fixed_var_timestamps(nvarstrips, -1);
    std::vector<int> fixed_act_timestamps(nact, -1);
    std::vector<my::BitField> inverse_actions(nact, my::BitField(nact));

    // (section 4.5 of Imai's paper)
    HPLUS_imai_iterative_variable_elimination(inst, eliminated_variables, fixed_variables, eliminated_actions, fixed_actions, eliminated_fas, fixed_fas, fixed_var_timestamps, fixed_act_timestamps);

    for (auto p : istate) fixed_var_timestamps[p] = 0;

    // (section 4.6 of Imai's paper)
    for (auto act_i : !eliminated_actions) {                                                    // FIXME: O(nact^2)
        const auto& pre = actions[act_i].get_pre();
        const auto& eff = actions[act_i].get_eff();
        for (unsigned int act_j = act_i + 1; act_j < nact; act_j++) if (!eliminated_actions[act_j]) {
            if (pre.contains(actions[act_j].get_eff()) && actions[act_j].get_pre().contains(eff)) {
                if (!fixed_actions[act_j]) inverse_actions[act_i].set(act_j);
                if (!fixed_actions[act_i]) inverse_actions[act_j].set(act_i);
            }
        }
    }

    #if HPLUS_VERBOSE >= 10
    int count = 0;
    for (unsigned int i = 0; i < nact; i++) for (auto j : inverse_actions[i]) count++;
    HPLUS_env.logger.print_info("Found %d pairs of inverse actions.", count/2);
    #endif

    fixed_variables |= gstate;

    // ~~~~~~~~ Adding CPLEX variables ~~~~~~~ //
    // (section 3 of Imai's paper)

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

    // action timestamps
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

    for (unsigned int i = 0; i < nact; i++) delete[] names[i];
    delete[] names; names = nullptr;
    delete[] types; types = nullptr;
    delete[] ubs; ubs = nullptr;
    delete[] lbs; lbs = nullptr;
    delete[] objs; objs = nullptr;

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

    // variable timestamps
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

    for (unsigned int i = 0; i < nvarstrips; i++) delete[] names[i];
    delete[] names; names = nullptr;
    delete[] types; types = nullptr;
    delete[] ubs; ubs = nullptr;
    delete[] lbs; lbs = nullptr;
    delete[] objs; objs = nullptr;

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
            for (unsigned int j = 0; j < variables[i].get_range(); j++) if (eff[k + j]) {
                objs[nfa] = 0;
                lbs[nfa] = fixed_fas[act_i][k+j] ? 1 : 0;
                ubs[nfa] = eliminated_fas[act_i][k+j] ? 0 : 1;
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

    // ~~~~~~~ Adding CPLEX constraints ~~~~~~ //
    // (section 3 of Imai's paper)

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
        ind_c3[i][nnz_c3[i]] = var_start + i;
        val_c3[i][nnz_c3[i]] = 1;
        nnz_c3[i]++;
    }

    for (unsigned int i = 0, count_fa = 0; i < nact; i++) {
        const my::BitField& pre = actions[i].get_pre();
        const my::BitField& eff = actions[i].get_eff();
        for (unsigned int j = 0; j < nvarstrips; j++) {
            if (pre[j]) {
                // constraint 1: x_a + sum_{inv(a, p)}(z_a'vj) <= y_vj, vj in pre(a)
                ind_c1[0] = act_start + i;
                val_c1[0] = 1;
                ind_c1[1] = var_start + j;
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
                        ind_c1[nnz0] = fa_start + fa_individual_start[inv_act] + var_cnt;
                        val_c1[nnz0++] = 1;
                    }
                }
                my::assert(!CPXaddrows(env, lp, 0, 1, nnz0, &rhs_c1_2_4, &sensel, &begin, ind_c1, val_c1, nullptr, nullptr), "CPXaddrows (c1) failed.");
                // constraint 4: t_vj <= t_a, vj in pre(a)
                ind_c2_4[0] = tvar_start + j;
                val_c2_4[0] = 1;
                ind_c2_4[1] = tact_start + i;
                val_c2_4[1] = -1;
                my::assert(!CPXaddrows(env, lp, 0, 1, nnz_c2_4, &rhs_c1_2_4, &sensel, &begin, ind_c2_4, val_c2_4, nullptr, nullptr), "CPXaddrows (c4) failed.");
            }
            if (eff[j]) {
                // constraint 2: z_avj <= x_a, vj in eff(a)
                ind_c2_4[0] = fa_start + count_fa;
                val_c2_4[0] = 1;
                ind_c2_4[1] = act_start + i;
                val_c2_4[1] = -1;
                my::assert(!CPXaddrows(env, lp, 0, 1, nnz_c2_4, &rhs_c1_2_4, &sensel, &begin, ind_c2_4, val_c2_4, nullptr, nullptr), "CPXaddrows (c2) failed.");
                // constraint 5: t_a + 1 <= t_vj + (|A|+1)(1-z_avj), vj in eff(a)
                ind_c5[0] = tact_start + i;
                val_c5[0] = 1;
                ind_c5[1] = tvar_start + j;
                val_c5[1] = -1;
                ind_c5[2] = fa_start + count_fa;
                val_c5[2] = nact + 1;
                my::assert(!CPXaddrows(env, lp, 0, 1, nnz_c5, &rhs_c5, &sensel, &begin, ind_c5, val_c5, nullptr, nullptr), "CPXaddrows (c5) failed.");
                // constraint 3: I(v_j) + sum(z_avj) = y_vj
                ind_c3[j][nnz_c3[j]] = fa_start + count_fa++;
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

    HPLUS_env.logger.print_info("Created CPLEX lp for imai.");

}

/**
 * Check if this solution is the best one and if so, stores it
*/
void HPLUS_store_imai_sol(const CPXENVptr& env, const CPXLPptr& lp, HPLUS_instance& inst) {

    // get cplex result (interested only in the sequence of actions [0/nact-1] used and its ordering [nact/2nact-1])
    unsigned int nact = inst.get_nact();
    auto* plan = new double[2 * nact];
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