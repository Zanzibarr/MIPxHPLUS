#include <vector>

#include "limits.hxx"
#include "preprocessing.hpp"

void prep::first_adders_extraction(hplus::instance& inst, std::vector<std::vector<unsigned int>>& landmarks) {
    BinarySet fact_lm_for_a(inst.n);

    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        fact_lm_for_a.clear();
        for (const auto& pre : inst.actions[act_i].pre_sparse) {
            for (const auto& fact_q : landmarks[pre]) {
                fact_lm_for_a.add(fact_q);
            }
        }
        std::erase_if(inst.actions[act_i].eff_sparse, [&fact_lm_for_a](const auto val) { return fact_lm_for_a[val]; });

        if (CHECK_STOP()) {
            [[unlikely]] throw timelimit_exception("Reached time limit.");
        }
    }
}
