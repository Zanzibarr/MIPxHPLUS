#include "solver.hpp"

// Time labeling model TL(Pi), Section 3.2 - Salvagnin, Zanella: "MIP Formulations for Delete-Free AI Planning"
// Imai, Fukunaga: "On a Practical, Integer-Linear Programming Model for Delete-Free Tasks and its Use as a Heuristic for Cost-Optimal Planning"
void Solver::hplus_add_tl_constraints_() {
    // ====================================================== //
    // =================== CPLEX VARIABLES ================== //
    // ====================================================== //

    // Timestamps t_p in {1, ..., |P|}: |P| is also the big-M of Eq. (8) - Salvagnin, Zanella: "MIP Formulations for Delete-Free AI Planning"
    const double max_steps{static_cast<double>(inst_.n)};
    const unsigned int tvar_start{static_cast<unsigned int>(CPXgetnumcols(global_.hplus_env, global_.hplus_lp))};

    std::vector<double> objs(inst_.n, 0.0);
    std::vector<double> lbs(inst_.n, 1.0);
    std::vector<double> ubs(inst_.n, max_steps);
    std::vector<char> types(inst_.n, 'I');

    call_cplex(
        CPXnewcols(global_.hplus_env, global_.hplus_lp, static_cast<int>(inst_.n), objs.data(), lbs.data(), ubs.data(), types.data(), nullptr));

    stats_.counter_set<"n_var_acyc">(inst_.n);

    // ====================================================== //
    // ================== CPLEX CONSTRAINTS ================= //
    // ====================================================== //

    // accessing cplex variables
    const unsigned int fa_start{inst_.m};  // Look at base model
    const auto get_tvar_idx = [&tvar_start](unsigned int idx) { return static_cast<int>(tvar_start + idx); };
    const auto get_fa_idx = [this, &fa_start](unsigned int act_idx, unsigned int var_count) {
        return static_cast<int>(fa_start + global_.hplus_fadd_cpx_start[act_idx] + var_count);
    };

    std::vector<int> ind(3);
    std::vector<double> val(3);
    constexpr char sense_l{'L'};
    const double rhs{max_steps - 1};
    constexpr int begin{0};

    // Equation 9 - Feyzbakhsh Rankooh, Rintanen: "Efficient Computation and Informative Estimation of h+ by Integer and Linear Programming"
    // Equation 8 - Salvagnin, Zanella: "MIP Formulations for Delete-Free AI Planning"
    // t_p - t_q + 1 <= |P| (1 - z_{a,q}), written as |P| z_{a,q} - t_q + t_p <= |P| - 1   (p = pre, q = eff)
    for (unsigned int act_i = 0; act_i < inst_.m; ++act_i) {
        unsigned int var_count{0};
        for (const auto& eff : inst_.actions[act_i].eff_sparse) {
            ind[0] = get_fa_idx(act_i, var_count);
            val[0] = max_steps;
            ind[1] = get_tvar_idx(eff);
            val[1] = -1;
            for (const auto& pre : inst_.actions[act_i].pre_sparse) {
                ind[2] = get_tvar_idx(pre);
                val[2] = 1;
                call_cplex(
                    CPXaddrows(global_.hplus_env, global_.hplus_lp, 0, 1, 3, &rhs, &sense_l, &begin, ind.data(), val.data(), nullptr, nullptr));
                stats_.counter_inc<"n_const_acyc">();
            }
            var_count++;
        }
    }
}

// MIP start (Section 5.1) for TL(Pi) - Salvagnin, Zanella: "MIP Formulations for Delete-Free AI Planning"
// t_q is the (1-based) step of the warm start in which q is first achieved
void Solver::hplus_post_tl_warm_start_() {
    BinarySet state{inst_.n};
    const auto& warm_start{global_.solution};

    myassert(!warm_start.empty(), "No solution provided as warm start");

    const auto ncols = CPXgetnumcols(global_.hplus_env, global_.hplus_lp);
    std::vector<int> ind(static_cast<unsigned int>(ncols));
    std::iota(ind.begin(), ind.end(), 0);
    std::vector<double> val(static_cast<unsigned int>(ncols), 0.0);
    constexpr int izero{0};
    constexpr int effortlevel{CPX_MIPSTART_NOCHECK};
    unsigned int timestamp{0};

    // Timestamps have lb = 1 and NOCHECK trusts us: facts never reached by the warm start are unconstrained, so give them the lower bound
    std::fill(val.begin() + inst_.m + inst_.nfadd + inst_.n, val.begin() + inst_.m + inst_.nfadd + (2 * inst_.n), 1.0);

    for (const auto& act_i : warm_start) {
        val[act_i] = 1;
        timestamp++;
        int var_count{-1};
        for (const auto& var_i : inst_.actions[act_i].eff_sparse) {
            var_count++;
            if (state[var_i]) {
                continue;
            }

            unsigned int fadd_idx = inst_.m + global_.hplus_fadd_cpx_start[act_i] + static_cast<unsigned int>(var_count);
            val[fadd_idx] = 1;
            unsigned int var_idx = inst_.m + inst_.nfadd + var_i;
            val[var_idx] = 1;
            unsigned int tvar_idx = inst_.m + inst_.nfadd + inst_.n + var_i;
            val[tvar_idx] = static_cast<double>(timestamp);
        }
        state |= inst_.actions[act_i].eff_sparse;
    }

    call_cplex(CPXaddmipstarts(global_.hplus_env, global_.hplus_lp, 1, ncols, &izero, ind.data(), val.data(), &effortlevel, nullptr));
}
