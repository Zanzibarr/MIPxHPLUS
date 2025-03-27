#include "imai.hpp"

#include <numeric>  // std::accumulate

void imai::build_cpx_model(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log,
                           hplus::statistics& stats) {
    PRINT_VERBOSE(log, "Building Imai's model.");

    const auto stopchk1 = []() {
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    };

    // ====================================================== //
    // =================== TIGHTER BOUNDS =================== //
    // ====================================================== //

    unsigned int max_steps{static_cast<unsigned int>(inst.m_opt)};
    if (env.tight_bounds) {
        // number of variables
        if (inst.n_opt < max_steps) max_steps = inst.n_opt;

        // max number of steps to reach heuristic
        if (env.heur != "none") {
            unsigned int min_act_cost{inst.actions[0].cost + 1};  // +1 to avoid it being 0
            unsigned int n_act_zerocost{0};
            for (const auto& act_i : inst.act_rem) {
                if (inst.actions[act_i].cost == 0)
                    n_act_zerocost++;
                else if (inst.actions[act_i].cost < min_act_cost)
                    min_act_cost = inst.actions[act_i].cost;
            }
            const unsigned int nsteps{static_cast<unsigned int>(std::ceil(static_cast<double>(inst.best_sol.cost) / min_act_cost)) + n_act_zerocost};
            if (nsteps < max_steps) max_steps = nsteps;
        }
    }
    stopchk1();

    // ====================================================== //
    // =================== CPLEX VARIABLES ================== //
    // ====================================================== //
    // (section 3 of Imai's paper)

    size_t curr_col{0};

    double* objs{new double[inst.m_opt]};
    double* lbs{new double[inst.m_opt]};
    double* ubs{new double[inst.m_opt]};
    char* types{new char[inst.m_opt]};

    auto resize_cpx_arrays = [&objs, &lbs, &ubs, &types](size_t new_size) {
        delete[] types;
        types = nullptr;
        delete[] ubs;
        ubs = nullptr;
        delete[] lbs;
        lbs = nullptr;
        delete[] objs;
        objs = nullptr;

        objs = new double[new_size];
        lbs = new double[new_size];
        ubs = new double[new_size];
        types = new char[new_size];
    };

    const auto stopchk2 = [&objs, &lbs, &ubs, &types]() {
        if (CHECK_STOP()) [[unlikely]] {
            delete[] types;
            types = nullptr;
            delete[] ubs;
            ubs = nullptr;
            delete[] lbs;
            lbs = nullptr;
            delete[] objs;
            objs = nullptr;
            throw timelimit_exception("Reached time limit.");
        }
    };

    // -------- actions ------- //
    const size_t act_start{curr_col};
    size_t count{0};
    for (const auto& act_i : inst.act_rem) {
        objs[count] = static_cast<double>(inst.actions[act_i].cost);
        lbs[count] = (inst.act_f[act_i] ? 1 : 0);
        ubs[count] = 1;
        types[count++] = 'B';
    }
    curr_col += inst.m_opt;

    CPX_HANDLE_CALL(log, CPXnewcols(cpxenv, cpxlp, inst.m_opt, objs, lbs, ubs, types, nullptr));
    stopchk2();

    // --- action timestamps -- //
    const size_t tact_start{curr_col};
    count = 0;
    for (const auto& act_i : inst.act_rem) {
        objs[count] = 0;
        lbs[count] = (inst.act_t[act_i] >= 0 ? inst.act_t[act_i] : 0);
        ubs[count] = (inst.act_t[act_i] >= 0 ? inst.act_t[act_i] : max_steps);
        types[count++] = 'I';
    }
    curr_col += inst.m_opt;

    CPX_HANDLE_CALL(log, CPXnewcols(cpxenv, cpxlp, inst.m_opt, objs, lbs, ubs, types, nullptr));
    stopchk2();

    resize_cpx_arrays(inst.n_opt);

    // ------- variables ------ //
    const size_t var_start{curr_col};
    count = 0;
    for (const auto& var_i : inst.var_rem) {
        objs[count] = 0;
        lbs[count] = ((inst.var_f[var_i] || inst.goal[var_i]) ? 1 : 0);
        ubs[count] = 1;
        types[count++] = 'B';
    }
    curr_col += inst.n_opt;

    CPX_HANDLE_CALL(log, CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
    stopchk2();

    // -- variable timestamps - //
    const size_t tvar_start{curr_col};
    count = 0;
    for (const auto& i : inst.var_rem) {
        objs[count] = 0;
        lbs[count] = (inst.var_t[i] >= 0 ? inst.var_t[i] : 1);  // 1 since with no initial state, each variable needs to be archieved first
        ubs[count] = (inst.var_t[i] >= 0 ? inst.var_t[i] : max_steps);
        types[count++] = 'I';
    }
    curr_col += inst.n_opt;

    CPX_HANDLE_CALL(log, CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
    stopchk2();

    // --- first archievers --- //
    const size_t fa_start{curr_col};
    count = 0;
    for (const auto& act_i : inst.act_rem) {
        size_t count_var{0};
        for (const auto& var_i : inst.actions[act_i].eff_sparse) {
            objs[count_var] = 0;
            lbs[count_var] = (inst.fadd_f[act_i][var_i] ? 1 : 0);
            ubs[count_var] = (inst.fadd_e[act_i][var_i] ? 0 : 1);
            types[count_var++] = 'B';
        }
        curr_col += count_var;
        CPX_HANDLE_CALL(log, CPXnewcols(cpxenv, cpxlp, count_var, objs, lbs, ubs, types, nullptr));
        count++;
        stopchk2();
    }

    delete[] types;
    types = nullptr;
    delete[] ubs;
    ubs = nullptr;
    delete[] lbs;
    lbs = nullptr;
    delete[] objs;
    objs = nullptr;

    stats.nvar_base = inst.n_opt + inst.m_opt + inst.n_fadd;
    stats.nvar_acyclic = curr_col - stats.nvar_base;

    // ====================================================== //
    // ================== CPLEX CONSTRAINTS ================= //
    // ====================================================== //
    // (section 3 of Imai's paper)

    // accessing cplex variables
    const auto get_act_idx = [&inst, &act_start](size_t idx) { return static_cast<int>(act_start + inst.act_opt_conv[idx]); };
    const auto get_tact_idx = [&inst, &tact_start](size_t idx) { return static_cast<int>(tact_start + inst.act_opt_conv[idx]); };
    const auto get_var_idx = [&inst, &var_start](size_t idx) { return static_cast<int>(var_start + inst.var_opt_conv[idx]); };
    const auto get_tvar_idx = [&inst, &tvar_start](size_t idx) { return static_cast<int>(tvar_start + inst.var_opt_conv[idx]); };
    const auto get_fa_idx = [&inst, &fa_start](size_t act_idx, size_t var_count) {
        return static_cast<int>(fa_start + inst.fadd_cpx_start[inst.act_opt_conv[act_idx]] + var_count);
    };

    int* ind_c1{new int[inst.m_opt + 1]};
    double* val_c1{new double[inst.m_opt + 1]};
    int ind_c2_4[2], ind_c5[3];
    double val_c2_4[2], val_c5[3];
    constexpr char sense_l{'L'}, sense_e{'E'};
    constexpr double rhs_c1_2_4{0};
    const double rhs_c5{static_cast<double>(max_steps)};
    constexpr int begin{0};

    std::vector<int*> ind_c3(inst.n_opt);
    std::vector<double*> val_c3(inst.n_opt);
    std::vector<int> nnz_c3(inst.n_opt);
    std::vector<double> rhs_c3(inst.n_opt);

    const auto stopchk3 = [&inst, &ind_c1, &val_c1, &ind_c3, &val_c3]() {
        if (CHECK_STOP()) [[unlikely]] {
            for (size_t var_i = 0; var_i < inst.n_opt; var_i++) {
                delete[] ind_c3[var_i];
                ind_c3[var_i] = nullptr;
                delete[] val_c3[var_i];
                val_c3[var_i] = nullptr;
            }
            delete[] val_c1;
            val_c1 = nullptr;
            delete[] ind_c1;
            ind_c1 = nullptr;
            throw timelimit_exception("Reached time limit.");
        }
    };

    for (const auto& i : inst.var_rem) {
        ind_c3[inst.var_opt_conv[i]] = new int[inst.m_opt + 1];
        val_c3[inst.var_opt_conv[i]] = new double[inst.m_opt + 1];
        nnz_c3[inst.var_opt_conv[i]] = 0;
        rhs_c3[inst.var_opt_conv[i]] = 0;
        ind_c3[inst.var_opt_conv[i]][nnz_c3[inst.var_opt_conv[i]]] = get_var_idx(i);
        val_c3[inst.var_opt_conv[i]][nnz_c3[inst.var_opt_conv[i]]] = 1;
        nnz_c3[inst.var_opt_conv[i]]++;
    }
    stopchk3();

    stats.nconst_base = 0;
    stats.nconst_acyclic = 0;
    for (const auto& act_i : inst.act_rem) {
        constexpr int nnz_c2_4{2};
        size_t var_count{0};
        for (const auto& var_i : inst.actions[act_i].pre_sparse) {
            // constraint 1: x_a + sum_{inv(a, p)}(z_a'vj) <= y_vj, vj in pre(a)
            ind_c1[0] = get_act_idx(act_i);
            val_c1[0] = 1;
            ind_c1[1] = get_var_idx(var_i);
            val_c1[1] = -1;
            int nnz0{2};
            // (section 4.6 of Imai's paper)
            for (const auto& inverse_action : inst.act_inv[act_i]) {
                if (inst.actions[inverse_action].eff[var_i]) {
                    ind_c1[nnz0] = get_fa_idx(inverse_action, var_count);
                    val_c1[nnz0++] = 1;
                }
            }
            stats.nconst_base++;
            CPX_HANDLE_CALL(log, CPXaddrows(cpxenv, cpxlp, 0, 1, nnz0, &rhs_c1_2_4, &sense_l, &begin, ind_c1, val_c1, nullptr, nullptr));
            // constraint 4: t_vj <= t_a, vj in pre(a)
            ind_c2_4[0] = get_tvar_idx(var_i);
            val_c2_4[0] = 1;
            ind_c2_4[1] = get_tact_idx(act_i);
            val_c2_4[1] = -1;
            stats.nconst_acyclic++;
            CPX_HANDLE_CALL(log, CPXaddrows(cpxenv, cpxlp, 0, 1, nnz_c2_4, &rhs_c1_2_4, &sense_l, &begin, ind_c2_4, val_c2_4, nullptr, nullptr));
            var_count++;
        }
        var_count = 0;
        for (const auto& var_i : inst.actions[act_i].eff_sparse) {
            constexpr int nnz_c5{3};
            // constraint 2: z_avj <= x_a, vj in eff(a)
            ind_c2_4[0] = get_fa_idx(act_i, var_count);
            val_c2_4[0] = 1;
            ind_c2_4[1] = get_act_idx(act_i);
            val_c2_4[1] = -1;
            stats.nconst_base++;
            CPX_HANDLE_CALL(log, CPXaddrows(cpxenv, cpxlp, 0, 1, nnz_c2_4, &rhs_c1_2_4, &sense_l, &begin, ind_c2_4, val_c2_4, nullptr, nullptr));
            // constraint 5: t_a + 1 <= t_vj + (|A|+1)(1-z_avj), vj in eff(a)
            ind_c5[0] = get_tact_idx(act_i);
            val_c5[0] = 1;
            ind_c5[1] = get_tvar_idx(var_i);
            val_c5[1] = -1;
            ind_c5[2] = get_fa_idx(act_i, var_count);
            val_c5[2] = max_steps + 1;
            stats.nconst_acyclic++;
            CPX_HANDLE_CALL(log, CPXaddrows(cpxenv, cpxlp, 0, 1, nnz_c5, &rhs_c5, &sense_l, &begin, ind_c5, val_c5, nullptr, nullptr));
            // constraint 3: I(v_j) + sum(z_avj) = y_vj
            ind_c3[inst.var_opt_conv[var_i]][nnz_c3[inst.var_opt_conv[var_i]]] = get_fa_idx(act_i, var_count);
            val_c3[inst.var_opt_conv[var_i]][nnz_c3[inst.var_opt_conv[var_i]]] = -1;
            nnz_c3[inst.var_opt_conv[var_i]]++;
        }
        stopchk3();
    }

    for (size_t var_i = 0; var_i < inst.n_opt; var_i++) {
        stats.nconst_base++;
        CPX_HANDLE_CALL(
            log, CPXaddrows(cpxenv, cpxlp, 0, 1, nnz_c3[var_i], &rhs_c3[var_i], &sense_e, &begin, ind_c3[var_i], val_c3[var_i], nullptr, nullptr));
    }

    for (size_t var_i = 0; var_i < inst.n_opt; var_i++) {
        delete[] ind_c3[var_i];
        ind_c3[var_i] = nullptr;
        delete[] val_c3[var_i];
        val_c3[var_i] = nullptr;
    }

    delete[] val_c1;
    val_c1 = nullptr;
    delete[] ind_c1;
    ind_c1 = nullptr;

    if (env.write_lp) CPX_HANDLE_CALL(log, CPXwriteprob(cpxenv, cpxlp, (HPLUS_CPLEX_OUTPUT_DIR "/lp/" + env.run_name + ".lp").c_str(), "LP"));
}

void imai::post_cpx_warmstart(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log) {
    PRINT_VERBOSE(log, "Posting warm start to Imai's model.");
    ASSERT_LOG(log, env.sol_s != solution_status::INFEAS && env.sol_s != solution_status::NOTFOUND);

    binary_set state{inst.n};

    const auto& warm_start{inst.best_sol.plan};

    const size_t ncols{static_cast<size_t>(CPXgetnumcols(cpxenv, cpxlp))};
    int* cpx_sol_ind{new int[ncols]};
    double* cpx_sol_val{new double[ncols]};
    for (size_t i = 0; i < ncols; i++) {
        cpx_sol_ind[i] = i;
        cpx_sol_val[i] = 0;
    }

    constexpr int izero{0};
    constexpr int effortlevel{CPX_MIPSTART_NOCHECK};
    unsigned int timestamp{0};

#if HPLUS_INTCHECK
    binary_set fixed_act_check{inst.act_f}, fixed_var_check{inst.var_f}, fixed_t_act_check{inst.m}, fixed_t_var_check{inst.n};
    std::vector<binary_set> fixed_fadd_check(inst.m);
    for (size_t i = 0; i < inst.m; i++) fixed_fadd_check[i] = inst.fadd_f[i];
    for (size_t i = 0; i < inst.m; i++)
        if (inst.act_t[i] >= 0) fixed_t_act_check.add(i);
    for (size_t i = 0; i < inst.n; i++)
        if (inst.var_t[i] >= 0) fixed_t_var_check.add(i);
#endif

    for (const auto& act_i : warm_start) {
#if HPLUS_INTCHECK
        ASSERT_LOG(log, !(inst.act_t[act_i] >= 0 && timestamp != static_cast<unsigned int>(inst.act_t[act_i])));
        ASSERT_LOG(log, !inst.act_e[act_i]);
        fixed_act_check.remove(act_i);
        fixed_t_act_check.remove(act_i);
#endif
        size_t cpx_act_idx = inst.act_opt_conv[act_i];
        cpx_sol_ind[cpx_act_idx] = static_cast<int>(cpx_act_idx);
        cpx_sol_val[cpx_act_idx] = 1;
        size_t cpx_tact_idx = inst.m_opt + inst.act_opt_conv[act_i];
        cpx_sol_ind[cpx_tact_idx] = static_cast<int>(cpx_tact_idx);
        cpx_sol_val[cpx_tact_idx] = static_cast<int>(timestamp);
        timestamp++;
        int var_count{-1};
        for (const auto& var_i : inst.actions[act_i].eff_sparse) {
            var_count++;
            if (state[var_i]) continue;

#if HPLUS_INTCHECK
            ASSERT_LOG(log, !(inst.var_t[var_i] >= 0 && timestamp != static_cast<unsigned int>(inst.var_t[var_i])));
            ASSERT_LOG(log, !inst.var_e[var_i]);
            ASSERT_LOG(log, !inst.fadd_e[act_i][var_i]);
            fixed_var_check.remove(var_i);
            fixed_t_var_check.remove(var_i);
            fixed_fadd_check[act_i].remove(var_i);
#endif

            size_t cpx_var_idx = 2 * inst.m_opt + inst.var_opt_conv[var_i];
            cpx_sol_ind[cpx_var_idx] = static_cast<int>(cpx_var_idx);
            cpx_sol_val[cpx_var_idx] = 1;
            size_t cpx_tvar_idx = 2 * inst.m_opt + inst.n_opt + inst.var_opt_conv[var_i];
            cpx_sol_ind[cpx_tvar_idx] = static_cast<int>(cpx_tvar_idx);
            cpx_sol_val[cpx_tvar_idx] = static_cast<int>(timestamp);
            size_t cpx_fad_idx = 2 * inst.m_opt + 2 * inst.n_opt + inst.fadd_cpx_start[inst.act_opt_conv[act_i]] + var_count;
            cpx_sol_ind[cpx_fad_idx] = static_cast<int>(cpx_fad_idx);
            cpx_sol_val[cpx_fad_idx] = 1;
        }
        state |= inst.actions[act_i].eff;
    }

#if HPLUS_INTCHECK
    ASSERT_LOG(log, fixed_act_check.empty());
    ASSERT_LOG(log, fixed_var_check.empty());
    ASSERT_LOG(log, fixed_t_act_check.empty());
    ASSERT_LOG(log, fixed_t_var_check.empty());
    for (size_t i = 0; i < inst.m; i++) ASSERT_LOG(log, fixed_fadd_check[i].empty());
#endif
    CPX_HANDLE_CALL(log, CPXaddmipstarts(cpxenv, cpxlp, 1, ncols, &izero, cpx_sol_ind, cpx_sol_val, &effortlevel, nullptr));

    delete[] cpx_sol_ind;
    cpx_sol_ind = nullptr;
    delete[] cpx_sol_val;
    cpx_sol_val = nullptr;
}

void imai::store_cpx_sol(CPXENVptr& cpxenv, CPXLPptr& cpxlp, hplus::instance& inst, const logger& log) {
    PRINT_VERBOSE(log, "Storing Imai's solution.");

    // get cplex result (interested only in the sequence of actions [0/inst.m_opt-1] used and its ordering [inst.m_opt/2nact-1])
    double* plan{new double[2 * inst.m_opt]};
    CPX_HANDLE_CALL(log, CPXgetx(cpxenv, cpxlp, plan, 0, 2 * inst.m_opt - 1));

    // convert to std collections for easier parsing
    std::vector<std::pair<double, size_t>> cpx_result;
    for (size_t i = 0; i < inst.m_opt; i++) {
        if (plan[i] > HPLUS_CPX_INT_ROUNDING) cpx_result.emplace_back(plan[inst.m_opt + i], i);
    }
    delete[] plan;
    plan = nullptr;

    // sort cpx_result based on actions timestamps
    std::sort(cpx_result.begin(), cpx_result.end(),
              [](const std::pair<double, size_t>& x, const std::pair<double, size_t>& y) { return x.first < y.first; });

    // get solution from sorted cpx_result
    std::vector<size_t> solution;
    std::transform(cpx_result.begin(), cpx_result.end(), std::back_inserter(solution),
                   [inst](const std::pair<double, size_t>& p) { return inst.act_cpxtoidx[p.second]; });

    // store solution
    hplus::solution imai_sol{
        solution, static_cast<unsigned int>(std::accumulate(solution.begin(), solution.end(), 0, [inst](const unsigned int acc, const size_t index) {
            return acc + inst.actions[index].cost;
        }))};
    hplus::update_sol(inst, imai_sol, log);
}
