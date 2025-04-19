#include "tl.hpp"

#include <numeric>  // std::accumulate

// ##################################################################### //
// ############################ BUILD MODEL ############################ //
// ##################################################################### //

void tl::build_cpx_model(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log,
                         hplus::statistics& stats) {
    PRINT_VERBOSE(log, "Building TL model.");

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
            unsigned int min_act_cost{std::numeric_limits<unsigned int>::max()};
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

    size_t curr_col{0};
    double* objs{new double[inst.m_opt]};
    double* lbs{new double[inst.m_opt]};
    double* ubs{new double[inst.m_opt]};
    char* types{new char[inst.m_opt]};

    const auto resize_cpx_arrays = [&objs, &lbs, &ubs, &types](size_t new_size) {
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

    curr_col += count;

    CPX_HANDLE_CALL(log, CPXnewcols(cpxenv, cpxlp, count, objs, lbs, ubs, types, nullptr));
    stopchk2();

    resize_cpx_arrays(inst.n_opt);

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

    // ------- variables ------ //
    const size_t var_start{curr_col};
    count = 0;
    for (const auto& var_i : inst.var_rem) {
        objs[count] = 0;
        lbs[count] = (inst.var_f[var_i] || inst.goal[var_i]) ? 1 : 0;
        ubs[count] = 1;
        types[count++] = 'B';
    }
    curr_col += count;

    CPX_HANDLE_CALL(log, CPXnewcols(cpxenv, cpxlp, count, objs, lbs, ubs, types, nullptr));
    stopchk2();

    // - variables timestamps - //
    const size_t tvar_start{curr_col};
    count = 0;
    for ([[maybe_unused]] const auto& var_i : inst.var_rem) {
        objs[count] = 0;
        lbs[count] = 1;  // 1 since with no initial state, each variable needs to be archieved first
        ubs[count] = max_steps;
        types[count++] = 'I';
    }
    curr_col += count;

    CPX_HANDLE_CALL(log, CPXnewcols(cpxenv, cpxlp, count, objs, lbs, ubs, types, nullptr));
    stopchk2();

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

    // accessing cplex variables
    const auto get_act_idx = [&inst, &act_start](size_t idx) { return static_cast<int>(act_start + inst.act_opt_conv[idx]); };
    const auto get_var_idx = [&inst, &var_start](size_t idx) { return static_cast<int>(var_start + inst.var_opt_conv[idx]); };
    const auto get_tvar_idx = [&inst, &tvar_start](size_t idx) { return static_cast<int>(tvar_start + inst.var_opt_conv[idx]); };
    const auto get_fa_idx = [&inst, &fa_start](size_t act_idx, size_t var_count) {
        return static_cast<int>(fa_start + inst.fadd_cpx_start[inst.act_opt_conv[act_idx]] + var_count);
    };

    int* ind{new int[inst.m_opt + 1]};
    double* val{new double[inst.m_opt + 1]};
    int nnz{0};
    constexpr char sense_e{'E'}, sense_l{'L'};
    constexpr double rhs_0{0}, rhs_1{1};
    constexpr int begin{0};

    const auto stopchk3 = [&ind, &val]() {
        if (CHECK_STOP()) [[unlikely]] {
            delete[] val;
            val = nullptr;
            delete[] ind;
            ind = nullptr;
            throw timelimit_exception("Reached time limit.");
        }
    };

    stats.nconst_base = 0;
    stats.nconst_acyclic = 0;

    // Constraint C1
    for (const auto& var_i : inst.var_rem) {
        nnz = 0;
        ind[nnz] = get_var_idx(var_i);
        val[nnz++] = 1;

        bool fixed = false;
        for (const auto& act_i : inst.act_with_eff[var_i]) {
            // if one first adder is fixed, then also the variable should be fixed
            if (inst.fadd_f[act_i][var_i]) {
                const char fix = 'B';
                const double one = 1;
                fixed = true;
                CPX_HANDLE_CALL(log, CPXchgbds(cpxenv, cpxlp, 1, ind, &fix, &one));
                break;
            }
            // if the first adder we're about to add to the constraint was eliminated, it's useless to the constraint
            else if (inst.fadd_e[act_i][var_i])
                continue;
            size_t var_count = static_cast<size_t>(std::find(inst.actions[act_i].eff_sparse.begin(), inst.actions[act_i].eff_sparse.end(), var_i) -
                                                   inst.actions[act_i].eff_sparse.begin());
            ind[nnz] = get_fa_idx(act_i, var_count);
            val[nnz++] = -1;
        }
        // if we fixed the variable due to a fixed first adder (note also that if we had a fixed first adder, we already have all other first
        // adders for that effect eliminated), we don't need the constraint we're adding.
        if (fixed) continue;

        // if nnz == 1, then we'd have p = 0, meaning we could simply fix this variable to 0
        if (nnz == 1) {
            const char fix = 'B';
            const double zero = 0;
            CPX_HANDLE_CALL(log, CPXchgbds(cpxenv, cpxlp, 1, ind, &fix, &zero));
        } else {
            stats.nconst_base++;
            CPX_HANDLE_CALL(log, CPXaddrows(cpxenv, cpxlp, 0, 1, nnz, &rhs_0, &sense_e, &begin, ind, val, nullptr, nullptr));
        }
        stopchk3();
    }

    // Constraint C2
    for (const auto& p : inst.var_rem) {
        for (const auto& q : inst.var_rem) {
            nnz = 0;
            ind[nnz] = get_var_idx(q);
            val[nnz++] = -1;
            bool fixed = false;
            for (const auto& act_i : inst.act_with_eff[p]) {
                if (!inst.actions[act_i].pre[q]) continue;
                // if the first adder is fixed, than we have 1 <= q, hence we can directly fix q
                if (inst.fadd_f[act_i][p]) {
                    const char fix = 'B';
                    const double one = 1;
                    fixed = true;
                    CPX_HANDLE_CALL(log, CPXchgbds(cpxenv, cpxlp, 1, ind, &fix, &one));
                    break;
                }
                // if the first adder we're about to add to the constraint was eliminated, it's useless to the constraint
                else if (inst.fadd_e[act_i][p])
                    continue;
                size_t var_count = static_cast<size_t>(std::find(inst.actions[act_i].eff_sparse.begin(), inst.actions[act_i].eff_sparse.end(), p) -
                                                       inst.actions[act_i].eff_sparse.begin());
                ind[nnz] = get_fa_idx(act_i, var_count);
                val[nnz++] = 1;
            }
            // since we have a fixed first adder, all other first adder for that effect are eliminated, so we don't need the constraint
            if (fixed) continue;
            // if nnz == 1 than we have -p <= 0, hence it's always true, we can ignore this constraint
            if (nnz != 1) {
                stats.nconst_base++;
                CPX_HANDLE_CALL(log, CPXaddrows(cpxenv, cpxlp, 0, 1, nnz, &rhs_0, &sense_l, &begin, ind, val, nullptr, nullptr));
            }
            stopchk3();
        }
    }

    delete[] val;
    val = nullptr;
    delete[] ind;
    ind = nullptr;

    int ind2[3];
    double val2[3];

    // Constraint C3
    for (const auto& act_i : inst.act_rem) {
        int var_count{-1};
        for (const auto& var_i : inst.actions[act_i].eff_sparse) {
            var_count++;
            ind2[0] = get_act_idx(act_i);
            val2[0] = -1;
            // if the first adder was fixed, we can directly fix the action insthead of adding the constraint
            if (inst.fadd_f[act_i][var_i]) {
                const char fix = 'B';
                const double one = 1;
                CPX_HANDLE_CALL(log, CPXchgbds(cpxenv, cpxlp, 1, ind2, &fix, &one));
                continue;
            }
            // if the first adder was eliminated, we can skip the constraint
            else if (inst.fadd_e[act_i][var_i])
                continue;
            ind2[1] = get_fa_idx(act_i, var_count);
            val2[1] = 1;
            stats.nconst_base++;
            CPX_HANDLE_CALL(log, CPXaddrows(cpxenv, cpxlp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind2, val2, nullptr, nullptr));
        }
        stopchk1();
    }

    // Inverse actions constraint (preprocessing)
    if (env.preprocessing) {
        for (const auto& act_i : inst.act_rem) {
            nnz = 0;
            ind2[nnz] = get_act_idx(act_i);
            val2[nnz++] = 1;
            for (const auto& act_j : inst.act_inv[act_i]) {
                if (act_j <= act_i) continue;
                ind2[nnz] = get_act_idx(act_j);
                val2[nnz] = 1;
                stats.nconst_base++;
                CPX_HANDLE_CALL(log, CPXaddrows(cpxenv, cpxlp, 0, 1, 2, &rhs_1, &sense_l, &begin, ind2, val2, nullptr, nullptr));
            }
            stopchk3();
        }
    }

    // ~~~~~~~~~~~~ time labeling ~~~~~~~~~~~~ //

    // Constraint C5
    const double rhs2{static_cast<double>(max_steps) - 1};
    for (const auto& act_i : inst.act_rem) {
        size_t var_count{0};
        for (const auto& q : inst.actions[act_i].eff_sparse) {
            ind2[0] = get_fa_idx(act_i, var_count);
            val2[0] = max_steps;
            ind2[1] = get_tvar_idx(q);
            val2[1] = -1;
            for (const auto& p : inst.actions[act_i].pre_sparse) {
                ind2[2] = get_tvar_idx(p);
                val2[2] = 1;
                CPX_HANDLE_CALL(log, CPXaddrows(cpxenv, cpxlp, 0, 1, 3, &rhs2, &sense_l, &begin, ind2, val2, nullptr, nullptr));
                stats.nconst_acyclic++;
            }
            var_count++;
        }
    }

    if (env.write_lp) CPX_HANDLE_CALL(log, CPXwriteprob(cpxenv, cpxlp, (HPLUS_CPLEX_OUTPUT_DIR "/lp/" + env.run_name + ".lp").c_str(), "LP"));
}

// ##################################################################### //
// ############################# WARM START ############################ //
// ##################################################################### //

void tl::post_cpx_warmstart(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log) {
    PRINT_VERBOSE(log, "Posting warm start to TL model.");
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

    for (const auto& act_i : warm_start) {
        size_t cpx_act_idx = inst.act_opt_conv[act_i];
        cpx_sol_val[cpx_act_idx] = 1;
        timestamp++;
        int var_count{-1};
        for (const auto& var_i : inst.actions[act_i].eff_sparse) {
            var_count++;
            if (state[var_i]) continue;

            size_t cpx_fad_idx = inst.m_opt + inst.fadd_cpx_start[inst.act_opt_conv[act_i]] + var_count;
            cpx_sol_val[cpx_fad_idx] = 1;
            size_t cpx_var_idx = inst.m_opt + inst.n_fadd + inst.var_opt_conv[var_i];
            cpx_sol_val[cpx_var_idx] = 1;
            size_t cpx_tvar_idx = inst.m_opt + inst.n_fadd + inst.n_opt + inst.var_opt_conv[var_i];
            cpx_sol_val[cpx_tvar_idx] = static_cast<double>(timestamp);
        }
        state |= inst.actions[act_i].eff;
    }

    CPX_HANDLE_CALL(log, CPXaddmipstarts(cpxenv, cpxlp, 1, ncols, &izero, cpx_sol_ind, cpx_sol_val, &effortlevel, nullptr));

    delete[] cpx_sol_ind;
    cpx_sol_ind = nullptr;
    delete[] cpx_sol_val;
    cpx_sol_val = nullptr;
}

// ##################################################################### //
// ########################### STORE SOLUTION ########################## //
// ##################################################################### //

void tl::store_cpx_sol(CPXENVptr& cpxenv, CPXLPptr& cpxlp, hplus::instance& inst, const logger& log) {
    PRINT_VERBOSE(log, "Storing TL solution.");

    double* plan{new double[inst.m_opt + inst.n_fadd]};
    CPX_HANDLE_CALL(log, CPXgetx(cpxenv, cpxlp, plan, 0, inst.m_opt + inst.n_fadd - 1));

    // fixing the solution to read the plan (some actions are set to 1 even if they are not a first archiever of anything)
    for (size_t act_i_cpx = 0; act_i_cpx < inst.m_opt; act_i_cpx++) {
        bool set_zero{true};
        for (size_t var_count = 0; var_count < inst.actions[inst.act_cpxtoidx[act_i_cpx]].eff_sparse.size(); var_count++) {
            if (plan[inst.m_opt + inst.fadd_cpx_start[act_i_cpx] + var_count] > HPLUS_CPX_INT_ROUNDING) {
                ASSERT_LOG(log, plan[act_i_cpx] > HPLUS_CPX_INT_ROUNDING);
                set_zero = false;
                break;
            }
        }
        if (set_zero) {
            ASSERT_LOG(log, !(plan[act_i_cpx] > HPLUS_CPX_INT_ROUNDING && inst.actions[inst.act_cpxtoidx[act_i_cpx]].cost != 0));
            plan[act_i_cpx] = 0;
        }
    }

    // convert to std collections for easier parsing
    std::vector<size_t> cpx_result;
    cpx_result.reserve(inst.m_opt);
    for (size_t i = 0; i < inst.m_opt; i++) {
        if (plan[i] > HPLUS_CPX_INT_ROUNDING) cpx_result.push_back(inst.act_cpxtoidx[i]);
    }
    delete[] plan;
    plan = nullptr;

    std::vector<size_t> solution;
    solution.reserve(inst.m_opt);
    binary_set remaining{cpx_result.size(), true}, state{inst.n};

    while (!remaining.empty()) {
        bool intcheck{false};
        for (const auto& i : remaining) {
            if (!state.contains(inst.actions[cpx_result[i]].pre)) continue;

            remaining.remove(i);
            state |= inst.actions[cpx_result[i]].eff;
            solution.push_back(cpx_result[i]);
            intcheck = true;
        }
        ASSERT_LOG(log, intcheck);
    }

    // store solution
    hplus::solution sol{
        solution, static_cast<unsigned int>(std::accumulate(solution.begin(), solution.end(), 0, [&inst](const unsigned int acc, const size_t index) {
            return acc + inst.actions[index].cost;
        }))};
    hplus::update_sol(inst, sol, log);
}
