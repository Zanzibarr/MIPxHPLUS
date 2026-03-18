#include <set>
#include <vector>

#include "limits.hxx"
#include "preprocessing.hpp"

void prep::first_adders_extraction(hplus::instance& inst, std::vector<std::vector<unsigned int>>& landmarks) {
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        std::set<unsigned int> fact_lm_for_a;
        for (const auto& pre : inst.actions[act_i].pre_sparse) {
            for (const auto& fact_q : landmarks[pre]) {
                fact_lm_for_a.insert(fact_q);
            }
        }
        // replace adders (eff) with first adders
        // first_adders[a] := { pre in add(a) s.t. pre is not a fact landmark for a }
        std::erase_if(inst.actions[act_i].eff_sparse, [&fact_lm_for_a](const auto val) { return fact_lm_for_a.contains(val); });
        if (CHECK_STOP()) {
            [[unlikely]] throw timelimit_exception("Reached time limit.");
        }
    }
}