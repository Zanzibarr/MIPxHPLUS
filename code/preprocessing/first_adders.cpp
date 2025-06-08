#include "preprocessing.hpp"

void prep::first_adders_extraction(hplus::instance& inst, std::vector<std::vector<unsigned int>>& landmarks) {
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        binary_set not_f_lm_a{inst.n, true};
        for (const auto& p : inst.actions[act_i].pre_sparse) {
            for (const auto& q : landmarks[p]) not_f_lm_a.remove(q);
        }
        // replace adders (eff) with first adders
        // first_adders[a] := { p in add(a) s.t. p is not a fact landmark for a }
        inst.actions[act_i].eff &= not_f_lm_a;
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    }
}