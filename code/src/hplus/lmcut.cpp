#include "solver.hpp"

void Solver::hplus_separate_cand_lmcut_cut_(CPXCALLBACKCONTEXTptr context, char minimization) {
    auto _sep_timer = scoped_timer("cand_lm_sep");
    const auto& [found, landmarks] = lmcut_int_separation_(local_.cand_used_actions.sparse(), &Solver::lmcut_hmax_arbitrary_, minimization);

    // We know at this point that a violated cut exists, so if lmcut doesn't find any (due to 0-cost actions) we need to fall back to a different
    // separator
    if (!found) {
        hplus_separate_cand_lmcomp_cut_(context);
        return;
    }

    hplus_reject_cand_lm_(context, landmarks);
}

void Solver::hplus_separate_relax_lmcut_cut_(CPXCALLBACKCONTEXTptr context, char minimization) {
    auto _sep_timer = scoped_timer("relax_lm_sep");
    const auto& [found, landmarks] = lmcut_relax_separation_(local_.relax_xstar, &Solver::lmcut_hmax_arbitrary_, minimization);

    hplus_reject_relax_lm_(context, landmarks);
}
