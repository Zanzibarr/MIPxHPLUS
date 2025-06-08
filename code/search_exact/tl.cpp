#include "exact.hpp"

void tl::add_acyclicity_constraints(const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats, CPXENVptr& env, CPXLPptr& lp) {
    if (BASIC_VERBOSE()) LOG_INFO << "Adding acyclicity constraints for TL model";

    // ====================================================== //
    // =================== CPLEX VARIABLES ================== //
    // ====================================================== //

    const double max_steps{static_cast<double>(inst.n)};
    const unsigned int tvar_start{static_cast<unsigned int>(CPXgetnumcols(env, lp))};

    std::vector<double> objs(inst.n, 0.0);
    std::vector<double> lbs(inst.n, 1.0);
    std::vector<double> ubs(inst.n, max_steps);
    std::vector<char> types(inst.n, 'I');

    CPX_HANDLE_CALL(CPXnewcols(env, lp, inst.n, objs.data(), lbs.data(), ubs.data(), types.data(), nullptr));

    stats.var_acyc = inst.n;

    // ====================================================== //
    // ================== CPLEX CONSTRAINTS ================= //
    // ====================================================== //

    // accessing cplex variables
    const unsigned int fa_start{inst.m};  // Look at base model
    const auto get_tvar_idx = [&tvar_start](unsigned int idx) { return static_cast<int>(tvar_start + idx); };
    const auto get_fa_idx = [&inst, &fa_start](unsigned int act_idx, unsigned int var_count) {
        return static_cast<int>(fa_start + inst.fadd_cpx_start[act_idx] + var_count);
    };

    std::vector<int> ind(3);
    std::vector<double> val(3);
    constexpr char sense_l{'L'};
    const double rhs{max_steps - 1};
    constexpr int begin{0};

    for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
        unsigned int var_count{0};
        for (const auto& q : inst.actions[act_i].eff_sparse) {
            ind[0] = get_fa_idx(act_i, var_count);
            val[0] = max_steps;
            ind[1] = get_tvar_idx(q);
            val[1] = -1;
            for (const auto& p : inst.actions[act_i].pre_sparse) {
                ind[2] = get_tvar_idx(p);
                val[2] = 1;
                CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, 3, &rhs, &sense_l, &begin, ind.data(), val.data(), nullptr, nullptr));
                stats.const_acyc++;
            }
            var_count++;
        }
    }
}

void tl::post_warm_start(const hplus::execution& exec, hplus::instance& inst, CPXENVptr& env, CPXLPptr& lp) {
    if (BASIC_VERBOSE()) LOG_INFO << "Posting warm start to TL model";

    binary_set state{inst.n};
    const auto& warm_start{inst.sol.sequence};

    const unsigned int ncols{static_cast<unsigned int>(CPXgetnumcols(env, lp))};
    std::vector<int> ind(ncols);
    std::iota(ind.begin(), ind.end(), 0);
    std::vector<double> val(ncols, 0.0);
    constexpr int izero{0};
    constexpr int effortlevel{CPX_MIPSTART_NOCHECK};
    unsigned int timestamp{0};

    for (const auto& act_i : warm_start) {
        val[act_i] = 1;
        timestamp++;
        int var_count{-1};
        for (const auto& var_i : inst.actions[act_i].eff_sparse) {
            var_count++;
            if (state[var_i]) continue;

            unsigned int fadd_idx = inst.m + inst.fadd_cpx_start[act_i] + var_count;
            val[fadd_idx] = 1;
            unsigned int var_idx = inst.m + inst.nfadd + var_i;
            val[var_idx] = 1;
            unsigned int tvar_idx = inst.m + inst.nfadd + inst.n + var_i;
            val[tvar_idx] = static_cast<double>(timestamp);
        }
        state |= inst.actions[act_i].eff;
    }

    CPX_HANDLE_CALL(CPXaddmipstarts(env, lp, 1, ncols, &izero, ind.data(), val.data(), &effortlevel, nullptr));
}