#include "exact.hpp"

void cuts::post_warm_start(const hplus::execution& exec, hplus::instance& inst, CPXENVptr& env, CPXLPptr& lp) {
    if (BASIC_VERBOSE()) LOG_INFO << "Posting warm start to CUTS model";

    binary_set state{inst.n};
    const auto& warm_start{inst.sol.sequence};

    const unsigned int ncols{static_cast<unsigned int>(CPXgetnumcols(env, lp))};
    std::vector<int> ind(ncols);
    std::iota(ind.begin(), ind.end(), 0);
    std::vector<double> val(ncols, 0.0);
    constexpr int izero{0};
    constexpr int effortlevel{CPX_MIPSTART_NOCHECK};

    for (const auto& act_i : warm_start) {
        val[act_i] = 1;
        int var_count{-1};
        for (const auto& var_i : inst.actions[act_i].eff_sparse) {
            var_count++;
            if (state[var_i]) continue;

            unsigned int fadd_idx = inst.m + inst.fadd_cpx_start[act_i] + var_count;
            val[fadd_idx] = 1;
            unsigned int var_idx = inst.m + inst.nfadd + var_i;
            val[var_idx] = 1;
        }
        state |= inst.actions[act_i].eff;
    }

    CPX_HANDLE_CALL(CPXaddmipstarts(env, lp, 1, ncols, &izero, ind.data(), val.data(), &effortlevel, nullptr));
}